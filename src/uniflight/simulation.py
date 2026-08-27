from __future__ import annotations
from dataclasses import dataclass
import numpy as np

from .events import Event, EventAction, EventOccurrence
from .integrators import ScipyIVPIntegrator

@dataclass(frozen=True, slots=True)
class SimulationResult:
    times: np.ndarray
    states: np.ndarray  # shape (n_times, n_state)
    events: tuple[EventOccurrence, ...]
    terminated_by: str | None
    success: bool
    message: str

class SimulationEngine:
    def __init__(self, rhs, integrator=None):
        self.rhs = rhs
        self.integrator = integrator or ScipyIVPIntegrator()

    def run(self, t_span: tuple[float,float], y0: np.ndarray,
            events: list[Event] | tuple[Event, ...] = ()) -> SimulationResult:
        t0, tf = map(float, t_span)
        if not tf > t0:
            raise ValueError("t_span must increase")
        y = np.asarray(y0, dtype=float).copy()
        t = t0
        all_t: list[float] = []
        all_y: list[np.ndarray] = []
        occurrences: list[EventOccurrence] = []
        terminated_by = None
        event_list = tuple(events)

        def time_tol(time: float) -> float:
            return max(1e-12, 16*np.finfo(float).eps*max(1.0, abs(time)))

        for _iteration in range(10000):
            sol = self.integrator.solve_segment(self.rhs, (t, tf), y, event_list)
            if not sol.success:
                return SimulationResult(np.array(all_t), np.array(all_y), tuple(occurrences),
                                        terminated_by, False, sol.message)

            seg_t = np.asarray(sol.t, dtype=float)
            seg_y = np.asarray(sol.y, dtype=float).T
            if all_t and len(seg_t) and np.isclose(seg_t[0], all_t[-1], rtol=0, atol=0):
                seg_t = seg_t[1:]
                seg_y = seg_y[1:]
            all_t.extend(seg_t.tolist())
            all_y.extend([row.copy() for row in seg_y])

            # Collect every root the integrator reported. Adaptive SciPy
            # integration may pass through pure CONTINUE/no-jump events without
            # restarting, while state-changing/terminating events stop the
            # segment. Fixed-step RK4 stops at its first root and is restarted.
            roots: list[tuple[float, int, np.ndarray]] = []
            for i, arr in enumerate(sol.t_events or []):
                for k, rt in enumerate(np.asarray(arr, dtype=float)):
                    y_events = np.asarray(sol.y_events[i], dtype=float)
                    if y_events.ndim == 2 and k < len(y_events):
                        y_root = y_events[k].copy()
                    else:
                        dense = getattr(sol, "sol", None)
                        if callable(dense):
                            y_root = np.asarray(dense(float(rt)), dtype=float).copy()
                        else:
                            y_root = np.asarray(sol.y[:, -1], dtype=float).copy()
                    roots.append((float(rt), i, y_root))
            roots.sort(key=lambda item: (item[0], -event_list[item[1]].priority, item[1]))

            if not roots:
                break

            restart = False
            terminate = False
            last_root_time: float | None = None
            last_root_state: np.ndarray | None = None
            pos = 0
            while pos < len(roots):
                root_time = roots[pos][0]
                tol = time_tol(root_time)
                group: list[tuple[float, int, np.ndarray]] = []
                while pos < len(roots) and abs(roots[pos][0]-root_time) <= tol:
                    group.append(roots[pos])
                    pos += 1
                # One event may only contribute one root to a tied group.
                by_index: dict[int, np.ndarray] = {}
                for _rt, idx, state in group:
                    by_index.setdefault(idx, state)
                tied = sorted(by_index, key=lambda i: (-event_list[i].priority, i))
                current = by_index[tied[0]].copy()

                changed = False
                for i in tied:
                    e = event_list[i]
                    before = current.copy()
                    after = np.asarray(e.jump(root_time, current.copy()), dtype=float) if e.jump else current.copy()
                    if after.shape != current.shape or not np.all(np.isfinite(after)):
                        raise ValueError(f"Jump map {e.name!r} returned invalid state")
                    occurrences.append(EventOccurrence(e.name, root_time, before, after.copy(), e.priority))
                    current = after
                    changed = changed or e.jump is not None
                    if e.action is EventAction.TERMINATE:
                        terminated_by = e.name
                        terminate = True

                last_root_time = root_time
                last_root_state = current.copy()

                if terminate or changed:
                    # State-changing events are terminal in the adaptive
                    # integrator, so no roots beyond this point are physically
                    # valid. Fixed-step RK4 also returns at the first root.
                    restart = not terminate
                    break

            if terminate:
                assert last_root_time is not None and last_root_state is not None
                if not all_t or all_t[-1] != last_root_time:
                    all_t.append(last_root_time)
                    all_y.append(last_root_state.copy())
                elif np.any(all_y[-1] != last_root_state):
                    all_y[-1] = last_root_state.copy()
                break

            if restart:
                assert last_root_time is not None and last_root_state is not None
                y = last_root_state
                t_next = np.nextafter(last_root_time, tf)
                if t_next <= last_root_time:
                    raise RuntimeError("Unable to advance beyond event time")
                t = t_next
                if t >= tf:
                    break
                continue

            # If the segment reached the requested final time, all roots were
            # pure observations and no restart is needed. This is the normal
            # adaptive path for CONTINUE/no-jump guards.
            segment_end = float(sol.t[-1])
            if segment_end >= tf - time_tol(tf):
                break

            # Fixed-step RK4 intentionally returns at the first event even for
            # a pure CONTINUE/no-jump guard. Its crossing logic does not treat
            # a zero at the new left endpoint as a fresh root, so a restart is
            # safe and preserves parity with the adaptive semantics.
            if last_root_time is None or last_root_state is None:
                raise RuntimeError("Integrator stopped early without an event root")
            y = last_root_state
            t_next = np.nextafter(last_root_time, tf)
            if t_next <= last_root_time:
                raise RuntimeError("Unable to advance beyond event time")
            t = t_next
            if t >= tf:
                break
        else:
            raise RuntimeError("Exceeded maximum event iterations; possible zero-time event cycle")

        return SimulationResult(np.asarray(all_t), np.asarray(all_y), tuple(occurrences),
                                terminated_by, True, "success")
