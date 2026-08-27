from __future__ import annotations
from dataclasses import dataclass, field, replace
from types import MappingProxyType
from typing import Callable, Mapping, Any
import numpy as np

from .events import Event, EventAction
from .integrators import ScipyIVPIntegrator
from .state import StateSchema

UniverseHandler = Callable[["UniverseEventContext"], "UniverseMutation | None"]


@dataclass(frozen=True, slots=True)
class VehicleEvent:
    """Vehicle-local guard whose handler may mutate the global universe."""

    name: str
    guard: Callable[[float, np.ndarray], float]
    direction: float = 0.0
    priority: int = 0
    handler: UniverseHandler | None = None
    one_shot: bool = True

    def as_kernel_event(self) -> Event:
        return Event(
            self.name,
            self.guard,
            direction=self.direction,
            priority=self.priority,
            action=EventAction.TERMINATE,
        )


@dataclass(frozen=True, slots=True)
class VehicleSpec:
    """Complete runtime definition for one independently propagated vehicle.

    ``rhs`` may close over a vehicle-specific celestial body, environment,
    propulsion model, GNC system, or any user plug-in.  The universe engine is
    intentionally agnostic to those model details.
    """

    vehicle_id: str
    schema: StateSchema
    initial_state: np.ndarray
    rhs: Callable[[float, np.ndarray], np.ndarray]
    events: tuple[VehicleEvent, ...] = ()
    integrator: object | None = None
    mode: str = "default"
    dof: int | None = None
    model_context: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.vehicle_id or not isinstance(self.vehicle_id, str):
            raise ValueError("vehicle_id must be a non-empty string")
        y = np.asarray(self.initial_state, dtype=float)
        if y.shape != (self.schema.total_size,) or not np.all(np.isfinite(y)):
            raise ValueError("initial_state does not match schema")
        if self.dof not in (None, 3, 6):
            raise ValueError("dof must be None, 3, or 6")
        object.__setattr__(self, "initial_state", y.copy())
        object.__setattr__(self, "events", tuple(self.events))
        object.__setattr__(self, "model_context", MappingProxyType(dict(self.model_context)))


@dataclass(frozen=True, slots=True)
class VehicleSnapshot:
    vehicle_id: str
    schema: StateSchema
    state: np.ndarray
    mode: str
    dof: int | None
    model_context: Mapping[str, Any]

    def __post_init__(self) -> None:
        y = np.asarray(self.state, dtype=float)
        object.__setattr__(self, "state", y.copy())
        object.__setattr__(self, "model_context", MappingProxyType(dict(self.model_context)))


@dataclass(frozen=True, slots=True)
class UniverseEventContext:
    time: float
    vehicle_id: str
    event_name: str
    snapshots: Mapping[str, VehicleSnapshot]

    @property
    def source(self) -> VehicleSnapshot:
        return self.snapshots[self.vehicle_id]


@dataclass(frozen=True, slots=True)
class UniverseMutation:
    """Atomic topology/configuration change returned by an event handler.

    Existing IDs listed in ``upsert`` are replaced in-place with a new schema,
    state, RHS, events, or model context.  New IDs are spawned.  IDs in
    ``remove`` are deleted.  A mutation may therefore implement staging,
    docking-like replacement, 3<->6 DOF transitions, or vehicle destruction.
    """

    remove: tuple[str, ...] = ()
    upsert: tuple[VehicleSpec, ...] = ()
    note: str = ""

    def __post_init__(self) -> None:
        if len(set(self.remove)) != len(self.remove):
            raise ValueError("duplicate vehicle ID in remove list")
        ids = [s.vehicle_id for s in self.upsert]
        if len(set(ids)) != len(ids):
            raise ValueError("duplicate vehicle ID in upsert list")
        if set(self.remove) & set(ids):
            raise ValueError("a mutation may not remove and upsert the same vehicle ID")


@dataclass(frozen=True, slots=True)
class VehicleTrajectorySegment:
    vehicle_id: str
    mode: str
    dof: int | None
    schema: StateSchema
    start_time: float
    end_time: float
    times: np.ndarray
    states: np.ndarray  # (n_times, n_state)


@dataclass(frozen=True, slots=True)
class UniverseEventOccurrence:
    time: float
    vehicle_id: str
    event_name: str
    priority: int
    mutation_note: str
    active_vehicle_ids_after: tuple[str, ...]


@dataclass(frozen=True, slots=True)
class UniverseResult:
    start_time: float
    end_time: float
    segments: Mapping[str, tuple[VehicleTrajectorySegment, ...]]
    events: tuple[UniverseEventOccurrence, ...]
    final_vehicles: Mapping[str, VehicleSnapshot]
    success: bool
    message: str


@dataclass(slots=True)
class _RuntimeRecord:
    spec: VehicleSpec
    state: np.ndarray
    disabled_events: set[str] = field(default_factory=set)


class MultiVehicleUniverseEngine:
    """Event-synchronized concurrent propagation of heterogeneous vehicles.

    Each active vehicle is propagated independently from the current universe
    time to its first local event or the global final time.  The earliest root
    across all active vehicles becomes the next global synchronization point;
    every other vehicle is evaluated at that same time before topology changes
    are applied.  This permits different state dimensions, dynamics, celestial
    bodies/environments, GNC systems, and 3/6-DOF modes in one mission.
    """

    def __init__(self, default_integrator: object | None = None, event_time_tolerance: float = 1e-9):
        self.default_integrator = default_integrator or ScipyIVPIntegrator()
        if not np.isfinite(event_time_tolerance) or event_time_tolerance <= 0:
            raise ValueError("event_time_tolerance must be positive")
        self.event_time_tolerance = float(event_time_tolerance)

    @staticmethod
    def _active_events(rec: _RuntimeRecord) -> tuple[VehicleEvent, ...]:
        return tuple(e for e in rec.spec.events if e.name not in rec.disabled_events)

    def _solve(self, rec: _RuntimeRecord, t0: float, tf: float):
        integrator = rec.spec.integrator or self.default_integrator
        events = tuple(e.as_kernel_event() for e in self._active_events(rec))
        sol = integrator.solve_segment(rec.spec.rhs, (t0, tf), rec.state, events)
        if not sol.success:
            raise RuntimeError(sol.message)
        return sol, events

    @staticmethod
    def _first_root(sol) -> tuple[float, tuple[int, ...]] | None:
        roots: list[tuple[float, int]] = []
        for i, arr in enumerate(sol.t_events or []):
            if len(arr):
                roots.append((float(arr[0]), i))
        if not roots:
            return None
        te = min(t for t, _ in roots)
        tol = max(1e-12, 16*np.finfo(float).eps*max(1.0, abs(te)))
        return te, tuple(i for t, i in roots if abs(t-te) <= tol)

    def _state_at(self, rec: _RuntimeRecord, sol, t0: float, time: float) -> np.ndarray:
        if abs(time-t0) <= self.event_time_tolerance:
            return rec.state.copy()
        dense = getattr(sol, "sol", None)
        if callable(dense):
            y = np.asarray(dense(time), dtype=float)
            if y.shape == rec.state.shape and np.all(np.isfinite(y)):
                return y.copy()
        # Generic fallback for integrators without dense output.  No local event
        # can precede `time` here because the global scheduler has already found
        # the earliest event across the universe.
        integrator = rec.spec.integrator or self.default_integrator
        short = integrator.solve_segment(rec.spec.rhs, (t0, time), rec.state, ())
        if not short.success:
            raise RuntimeError(short.message)
        return np.asarray(short.y[:, -1], dtype=float).copy()

    def _trajectory_to(self, rec: _RuntimeRecord, sol, t0: float, te: float, y_end: np.ndarray) -> VehicleTrajectorySegment:
        t_arr = np.asarray(sol.t, dtype=float)
        y_arr = np.asarray(sol.y, dtype=float).T
        tol = self.event_time_tolerance
        mask = t_arr <= te + tol
        ts = t_arr[mask]
        ys = y_arr[mask]
        if len(ts) == 0 or abs(float(ts[0])-t0) > tol:
            ts = np.insert(ts, 0, t0)
            ys = np.vstack((rec.state, ys))
        if abs(float(ts[-1])-te) > tol:
            ts = np.append(ts, te)
            ys = np.vstack((ys, y_end))
        else:
            ts[-1] = te
            ys[-1] = y_end
        return VehicleTrajectorySegment(
            rec.spec.vehicle_id, rec.spec.mode, rec.spec.dof, rec.spec.schema,
            t0, te, np.asarray(ts), np.asarray(ys),
        )

    @staticmethod
    def _snapshots(records: Mapping[str, _RuntimeRecord]) -> Mapping[str, VehicleSnapshot]:
        return MappingProxyType({
            vid: VehicleSnapshot(
                vid, rec.spec.schema, rec.state, rec.spec.mode, rec.spec.dof,
                rec.spec.model_context,
            )
            for vid, rec in records.items()
        })

    @staticmethod
    def _apply_mutation(records: dict[str, _RuntimeRecord], mutation: UniverseMutation) -> None:
        for vid in mutation.remove:
            if vid not in records:
                raise KeyError(f"cannot remove inactive vehicle {vid!r}")
            del records[vid]
        for spec in mutation.upsert:
            # Upsert explicitly resets one-shot event history because a replaced
            # definition is a new vehicle configuration/mode generation.
            records[spec.vehicle_id] = _RuntimeRecord(spec, spec.initial_state.copy())

    def run(
        self,
        t_span: tuple[float, float],
        vehicles: tuple[VehicleSpec, ...] | list[VehicleSpec],
        *,
        max_global_events: int = 1000,
    ) -> UniverseResult:
        t0, tf = map(float, t_span)
        if not tf > t0:
            raise ValueError("t_span must increase")
        specs = tuple(vehicles)
        if not specs:
            raise ValueError("at least one vehicle is required")
        if len({s.vehicle_id for s in specs}) != len(specs):
            raise ValueError("initial vehicle IDs must be unique")
        records = {s.vehicle_id: _RuntimeRecord(s, s.initial_state.copy()) for s in specs}
        history: dict[str, list[VehicleTrajectorySegment]] = {s.vehicle_id: [] for s in specs}
        event_log: list[UniverseEventOccurrence] = []
        t = t0
        current_time = t0

        for _ in range(max_global_events + 1):
            if not records or t >= tf:
                break
            solved: dict[str, tuple[_RuntimeRecord, object, tuple[VehicleEvent, ...], tuple[float, tuple[int, ...]] | None]] = {}
            earliest: float | None = None
            for vid in sorted(records):
                rec = records[vid]
                sol, _ = self._solve(rec, t, tf)
                active = self._active_events(rec)
                root = self._first_root(sol)
                solved[vid] = (rec, sol, active, root)
                if root is not None:
                    earliest = root[0] if earliest is None else min(earliest, root[0])

            te = tf if earliest is None else earliest
            current_time = te
            # Synchronize all active vehicle states and append schema-tagged history.
            for vid in sorted(list(records)):
                rec, sol, _, _ = solved[vid]
                y_end = self._state_at(rec, sol, t, te)
                history.setdefault(vid, []).append(self._trajectory_to(rec, sol, t, te, y_end))
                rec.state = y_end

            if earliest is None:
                t = tf
                break

            tol = max(self.event_time_tolerance,
                      16*np.finfo(float).eps*max(1.0, abs(te)))
            fired: list[tuple[int, str, str, VehicleEvent]] = []
            for vid, (rec, _sol, active, root) in solved.items():
                if root is None or abs(root[0]-te) > tol:
                    continue
                for i in root[1]:
                    e = active[i]
                    fired.append((-e.priority, vid, e.name, e))
            fired.sort(key=lambda x: (x[0], x[1], x[2]))

            # All handlers observe the same pre-event synchronized snapshot.
            pre = self._snapshots(records)
            for _neg_prio, vid, _ename, event in fired:
                if vid not in pre:
                    continue
                # A higher-priority tied event may already have removed or
                # replaced this source. Lower-priority guards belong to the
                # pre-event generation and are not applied to the new one.
                source_spec = solved[vid][0].spec
                if vid not in records or records[vid].spec is not source_spec:
                    continue
                context = UniverseEventContext(te, vid, event.name, pre)
                mutation = event.handler(context) if event.handler is not None else None
                if mutation is None:
                    mutation = UniverseMutation(note="no topology change")
                self._apply_mutation(records, mutation)
                # If the source survived without replacement, suppress a one-shot
                # guard so a state-based zero does not re-trigger at nextafter().
                if event.one_shot and vid in records and all(s.vehicle_id != vid for s in mutation.upsert):
                    records[vid].disabled_events.add(event.name)
                event_log.append(UniverseEventOccurrence(
                    te, vid, event.name, event.priority, mutation.note,
                    tuple(sorted(records)),
                ))
                for spec in mutation.upsert:
                    history.setdefault(spec.vehicle_id, [])

            t_next = np.nextafter(te, tf)
            if t_next <= te:
                break
            t = t_next
        else:
            raise RuntimeError("maximum global event count exceeded")

        final = self._snapshots(records)
        return UniverseResult(
            t0, current_time,
            MappingProxyType({k: tuple(v) for k, v in history.items()}),
            tuple(event_log), final, True, "success",
        )
