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
    def __init__(self, config: SolverConfig | None = None):
        self.config = config or SolverConfig()

    def solve_segment(self, rhs: Callable, t_span: tuple[float,float], y0: np.ndarray,
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
