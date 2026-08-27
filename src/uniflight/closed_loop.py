from __future__ import annotations
from dataclasses import dataclass
from typing import Callable
import numpy as np

from .events import Event, EventOccurrence
from .integrators import ScipyIVPIntegrator
from .simulation import SimulationEngine
from .sensors import PositionVelocitySensor, AttitudeRateSensor
from .state import StateSchema, StateView
from .estimation import TranslationalNavigationEKF
from .control import LandingGNCController, GNCDecision


@dataclass(frozen=True, slots=True)
class GNCRecord:
    time: float
    estimate_rv: np.ndarray
    covariance: np.ndarray
    measurement_rv: np.ndarray
    throttle_command: float
    torque_command_b: np.ndarray
    thrust_direction_i: np.ndarray


@dataclass(frozen=True, slots=True)
class ClosedLoopResult:
    times: np.ndarray
    states: np.ndarray
    gnc_records: tuple[GNCRecord, ...]
    events: tuple[EventOccurrence, ...]
    terminated_by: str | None
    success: bool
    message: str
    integration_segments: int = 0


class SampledDataClosedLoopEngine:
    """Propagate the plant between chronological sampled-data GNC updates.

    Mutating estimators/controllers are intentionally sampled only at explicit
    monotonically increasing times.  The continuous RHS therefore remains a
    pure function from the ODE solver's perspective.

    ``record_trajectory=False`` is intended for Monte Carlo campaigns.  It
    retains only the latest physical state while still preserving all GNC and
    event records, substantially reducing Python allocation pressure.
    """
    def __init__(self, rhs: Callable, schema: StateSchema,
                 position_velocity_sensor: PositionVelocitySensor,
                 attitude_rate_sensor: AttitudeRateSensor,
                 navigator: TranslationalNavigationEKF,
                 controller: LandingGNCController, sample_period: float,
                 integrator=None, seed: int = 0, record_trajectory: bool = True,
                 record_gnc_records: bool = True):
        if not np.isfinite(sample_period) or sample_period <= 0:
            raise ValueError("sample_period must be positive")
        self.rhs = rhs
        self.schema = schema
        self.pv_sensor = position_velocity_sensor
        self.att_sensor = attitude_rate_sensor
        self.navigator = navigator
        self.controller = controller
        self.sample_period = float(sample_period)
        self.integrator = integrator or ScipyIVPIntegrator()
        self.rng = np.random.default_rng(seed)
        self.record_trajectory = bool(record_trajectory)
        self.record_gnc_records = bool(record_gnc_records)

    def _sample_and_command(self, t: float, y: np.ndarray) -> tuple[GNCRecord | None, GNCDecision]:
        st = StateView(t, y, self.schema)
        pv = self.pv_sensor.measure(st, self.rng)
        self.navigator.update_position_velocity(pv)
        att = self.att_sensor.measure(st, self.rng)
        decision = self.controller.update(st, self.navigator.x.copy(), att)
        rec = None
        if self.record_gnc_records:
            rec = GNCRecord(
                t, self.navigator.x.copy(), self.navigator.covariance.copy(), pv.value.copy(),
                decision.throttle_command, decision.torque_command_b.copy(),
                decision.thrust_direction_i.copy(),
            )
        return rec, decision

    def run(self, t_span: tuple[float, float], y0: np.ndarray,
            events: tuple[Event, ...] | list[Event] = ()) -> ClosedLoopResult:
        t0, tf = map(float, t_span)
        if tf <= t0:
            raise ValueError("t_span must increase")
        t = t0
        y = np.asarray(y0, dtype=float).copy()
        all_t: list[float] = []
        all_y: list[np.ndarray] = []
        records: list[GNCRecord] = []
        occurrences: list[EventOccurrence] = []
        terminated = None
        segments = 0

        if not self.record_trajectory:
            all_t = [t]
            all_y = [y.copy()]

        rec, decision = self._sample_and_command(t, y)
        if rec is not None:
            records.append(rec)
        for _ in range(int(np.ceil((tf-t0)/self.sample_period))+2):
            target = min(tf, t+self.sample_period)
            seg = SimulationEngine(self.rhs, self.integrator).run((t, target), y, events)
            segments += 1
            if not seg.success:
                return ClosedLoopResult(
                    np.asarray(all_t), np.asarray(all_y), tuple(records), tuple(occurrences),
                    terminated, False, seg.message, segments,
                )
            stimes = seg.times
            sstates = seg.states
            if self.record_trajectory:
                if all_t and len(stimes) and np.isclose(stimes[0], all_t[-1], atol=0, rtol=0):
                    stimes = stimes[1:]
                    sstates = sstates[1:]
                all_t.extend(stimes.tolist())
                all_y.extend([row.copy() for row in sstates])
            occurrences.extend(seg.events)
            if len(seg.times) == 0:
                break
            tnew = float(seg.times[-1])
            y = seg.states[-1].copy()
            dt = tnew-t
            self.navigator.predict(dt, decision.thrust_acceleration_i)
            t = tnew
            if not self.record_trajectory:
                all_t = [t]
                all_y = [y.copy()]
            if seg.terminated_by is not None:
                terminated = seg.terminated_by
                break
            if t >= tf-1e-14:
                break
            rec, decision = self._sample_and_command(t, y)
            if rec is not None:
                records.append(rec)

        return ClosedLoopResult(
            np.asarray(all_t), np.asarray(all_y), tuple(records), tuple(occurrences),
            terminated, True, "success", segments,
        )
