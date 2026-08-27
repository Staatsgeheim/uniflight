from __future__ import annotations
from dataclasses import dataclass
from typing import Callable
import numpy as np
from scipy.integrate import solve_ivp

from .events import Event


@dataclass(frozen=True, slots=True)
class SolverConfig:
    method: str = "DOP853"
    rtol: float = 1e-10
    atol: float | np.ndarray = 1e-12
    max_step: float = np.inf
    dense_output: bool = True


class ScipyIVPIntegrator:
    """Adaptive SciPy integration path used as the high-accuracy reference."""
    def __init__(self, config: SolverConfig | None = None):
        self.config = config or SolverConfig()

    def solve_segment(self, rhs: Callable, t_span: tuple[float, float], y0: np.ndarray,
                      events: list[Event] | tuple[Event, ...] = ()):
        wrappers = []
        for event in events:
            def fn(t, y, e=event):
                return float(e.guard(t, y))
            fn.direction = event.direction
            fn.terminal = True  # kernel resolves ordered jumps itself
            wrappers.append(fn)
        return solve_ivp(
            rhs, t_span, np.asarray(y0, dtype=float), method=self.config.method,
            rtol=self.config.rtol, atol=self.config.atol, max_step=self.config.max_step,
            dense_output=self.config.dense_output, events=wrappers if wrappers else None,
        )


@dataclass(frozen=True, slots=True)
class FixedStepRK4Config:
    """Configuration for deterministic campaign integration.

    ``step`` is the maximum internal RK4 step.  ``save_every_step=False`` is
    recommended for Monte Carlo work: event guards are still evaluated at
    every internal step, but only segment endpoints are returned to the outer
    sampled-data engine.
    """
    step: float = 0.05
    event_time_tolerance: float = 1e-8
    event_guard_tolerance: float = 1e-10
    max_event_iterations: int = 50
    save_every_step: bool = False

    def __post_init__(self) -> None:
        if not np.isfinite(self.step) or self.step <= 0:
            raise ValueError("RK4 step must be positive")
        if self.event_time_tolerance <= 0 or self.event_guard_tolerance < 0:
            raise ValueError("event tolerances invalid")
        if self.max_event_iterations <= 0:
            raise ValueError("max_event_iterations must be positive")


@dataclass(slots=True)
class FixedStepSegmentSolution:
    success: bool
    message: str
    t: np.ndarray
    y: np.ndarray                    # solve_ivp-compatible shape (n_state,n_time)
    t_events: list[np.ndarray]
    y_events: list[np.ndarray]
    nfev: int


def _direction_crossed(g0: float, g1: float, direction: float) -> bool:
    if not np.isfinite(g0) or not np.isfinite(g1):
        raise ValueError("event guard returned a non-finite value")
    # Deliberately avoid treating a zero at the left endpoint as a fresh root;
    # SimulationEngine advances by nextafter() after nonterminal events.
    if direction > 0:
        return g0 < 0.0 and g1 >= 0.0
    if direction < 0:
        return g0 > 0.0 and g1 <= 0.0
    return (g0 < 0.0 <= g1) or (g0 > 0.0 >= g1)


class FixedStepRK4Integrator:
    """Low-overhead deterministic RK4 integrator with hybrid-event detection.

    This exists for high-volume Monte Carlo campaigns.  It intentionally has
    a smaller feature surface than SciPy's adaptive solvers, but implements the
    ``solve_segment`` protocol consumed by :class:`SimulationEngine`.
    """
    def __init__(self, config: FixedStepRK4Config | None = None,
                 state_projector: Callable[[np.ndarray], np.ndarray] | None = None):
        self.config = config or FixedStepRK4Config()
        self.state_projector = state_projector

    def _project(self, y: np.ndarray) -> np.ndarray:
        if self.state_projector is None:
            return y
        projected = np.asarray(self.state_projector(y.copy()), dtype=float)
        if projected.shape != y.shape or not np.all(np.isfinite(projected)):
            raise ValueError("state_projector returned invalid state")
        return projected

    def _rk4(self, rhs: Callable, t: float, y: np.ndarray, h: float) -> tuple[np.ndarray, int]:
        k1 = np.asarray(rhs(t, y), dtype=float)
        k2 = np.asarray(rhs(t + 0.5*h, y + 0.5*h*k1), dtype=float)
        k3 = np.asarray(rhs(t + 0.5*h, y + 0.5*h*k2), dtype=float)
        k4 = np.asarray(rhs(t + h, y + h*k3), dtype=float)
        yn = y + (h/6.0)*(k1 + 2.0*k2 + 2.0*k3 + k4)
        return self._project(yn), 4

    def _refine_root(self, rhs: Callable, event: Event, ta: float, ya: np.ndarray,
                     tb: float, yb: np.ndarray, ga: float, gb: float) -> tuple[float, np.ndarray, int]:
        """Refine a bracketed root by bisection with RK4 state reconstruction."""
        nfev = 0
        left_t, left_y, left_g = ta, ya, ga
        right_t, right_y, right_g = tb, yb, gb
        for _ in range(self.config.max_event_iterations):
            if right_t-left_t <= self.config.event_time_tolerance:
                break
            mid_t = 0.5*(left_t+right_t)
            # Integrate from the left bracket to the midpoint.  Reusing the
            # already-refined left state keeps each subproblem short.
            mid_y, used = self._rk4(rhs, left_t, left_y, mid_t-left_t)
            nfev += used
            mid_g = float(event.guard(mid_t, mid_y))
            if abs(mid_g) <= self.config.event_guard_tolerance:
                return mid_t, mid_y, nfev
            if _direction_crossed(left_g, mid_g, event.direction):
                right_t, right_y, right_g = mid_t, mid_y, mid_g
            else:
                left_t, left_y, left_g = mid_t, mid_y, mid_g
        # Linear time interpolation within the final small bracket, followed
        # by one RK4 propagation to the estimated event time.
        denom = abs(left_g) + abs(right_g)
        frac = 0.5 if denom == 0 else abs(left_g)/denom
        root_t = left_t + frac*(right_t-left_t)
        root_y, used = self._rk4(rhs, left_t, left_y, root_t-left_t)
        nfev += used
        return root_t, root_y, nfev

    def solve_segment(self, rhs: Callable, t_span: tuple[float, float], y0: np.ndarray,
                      events: list[Event] | tuple[Event, ...] = ()) -> FixedStepSegmentSolution:
        t0, tf = map(float, t_span)
        if not tf > t0:
            raise ValueError("t_span must increase")
        y = self._project(np.asarray(y0, dtype=float).copy())
        t = t0
        event_list = tuple(events)
        t_events = [np.empty(0, dtype=float) for _ in event_list]
        y_events = [np.empty((0, y.size), dtype=float) for _ in event_list]
        saved_t = [t]
        saved_y = [y.copy()]
        nfev = 0

        guards = [float(e.guard(t, y)) for e in event_list]
        while t < tf:
            h = min(self.config.step, tf-t)
            yn, used = self._rk4(rhs, t, y, h)
            nfev += used
            tn = t+h
            next_guards = [float(e.guard(tn, yn)) for e in event_list]

            candidates: list[tuple[float, int, np.ndarray]] = []
            for i, event in enumerate(event_list):
                if _direction_crossed(guards[i], next_guards[i], event.direction):
                    rt, ry, extra = self._refine_root(
                        rhs, event, t, y, tn, yn, guards[i], next_guards[i]
                    )
                    nfev += extra
                    candidates.append((rt, i, ry))

            if candidates:
                root_time = min(rt for rt, _, _ in candidates)
                tie_tol = max(self.config.event_time_tolerance,
                              16*np.finfo(float).eps*max(1.0, abs(root_time)))
                tied = [(rt, i, ry) for rt, i, ry in candidates if abs(rt-root_time) <= tie_tol]
                # State from the earliest candidate.  SimulationEngine applies
                # priority ordering and any jump maps after this return.
                tied.sort(key=lambda item: (-event_list[item[1]].priority, item[1]))
                root_y = tied[0][2]
                if self.config.save_every_step:
                    saved_t.append(root_time); saved_y.append(root_y.copy())
                else:
                    saved_t = [t0, root_time]
                    saved_y = [np.asarray(y0, dtype=float).copy(), root_y.copy()]
                for rt, i, ry in tied:
                    t_events[i] = np.array([rt], dtype=float)
                    y_events[i] = np.asarray([ry], dtype=float)
                return FixedStepSegmentSolution(
                    True, "success", np.asarray(saved_t), np.asarray(saved_y).T,
                    t_events, y_events, nfev,
                )

            t, y, guards = tn, yn, next_guards
            if self.config.save_every_step:
                saved_t.append(t); saved_y.append(y.copy())

        if not self.config.save_every_step:
            saved_t = [t0, tf]
            saved_y = [np.asarray(y0, dtype=float).copy(), y.copy()]
        return FixedStepSegmentSolution(
            True, "success", np.asarray(saved_t), np.asarray(saved_y).T,
            t_events, y_events, nfev,
        )
