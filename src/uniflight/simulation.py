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

        for _iteration in range(10000):
            sol = self.integrator.solve_segment(self.rhs, (t, tf), y, event_list)
            if not sol.success:
                return SimulationResult(np.array(all_t), np.array(all_y), tuple(occurrences),
                                        terminated_by, False, sol.message)
            seg_t = sol.t
            seg_y = sol.y.T
            if all_t and len(seg_t) and np.isclose(seg_t[0], all_t[-1], rtol=0, atol=0):
                seg_t = seg_t[1:]; seg_y = seg_y[1:]
            all_t.extend(seg_t.tolist()); all_y.extend([row.copy() for row in seg_y])

            hit_indices = [i for i, arr in enumerate(sol.t_events or []) if len(arr)]
            if not hit_indices:
                break
            # solve_ivp stopped at first root; ties are resolved by priority among roots at same time.
            roots = [(i, float(sol.t_events[i][0])) for i in hit_indices]
            root_time = min(rt for _,rt in roots)
            tied = [i for i,rt in roots if abs(rt-root_time) <= max(1e-12, 16*np.finfo(float).eps*max(1,abs(root_time)))]
            tied.sort(key=lambda i: (-event_list[i].priority, i))
            # Obtain root state from first corresponding y_event.
            y_root = np.asarray(sol.y_events[tied[0]][0], dtype=float).copy()
            current = y_root
            terminate = False
            for i in tied:
                e = event_list[i]
                before = current.copy()
                after = np.asarray(e.jump(root_time, current.copy()), dtype=float) if e.jump else current.copy()
                if after.shape != current.shape or not np.all(np.isfinite(after)):
                    raise ValueError(f"Jump map {e.name!r} returned invalid state")
                occurrences.append(EventOccurrence(e.name, root_time, before, after.copy(), e.priority))
                current = after
                if e.action is EventAction.TERMINATE:
                    terminated_by = e.name
                    terminate = True
            if terminate:
                if not all_t or all_t[-1] != root_time:
                    all_t.append(root_time); all_y.append(current.copy())
                elif np.any(all_y[-1] != current):
                    all_y[-1] = current.copy()
                break
            # Continue immediately after the event. A jump should normally move off the guard.
            y = current
            t_next = np.nextafter(root_time, tf)
            if t_next <= root_time:
                raise RuntimeError("Unable to advance beyond event time")
            t = t_next
            if t >= tf:
                break
        else:
            raise RuntimeError("Exceeded maximum event iterations; possible zero-time event cycle")

        return SimulationResult(np.asarray(all_t), np.asarray(all_y), tuple(occurrences),
                                terminated_by, True, "success")
