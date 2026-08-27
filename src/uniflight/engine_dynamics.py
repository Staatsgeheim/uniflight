from __future__ import annotations
from dataclasses import dataclass
from typing import Callable
import numpy as np

from .state import StateView

ScalarProvider = Callable[[StateView], float]


@dataclass(frozen=True, slots=True)
class EngineTransient:
    """Second-order engine thrust-state dynamics with hard physical limits.

    ``power`` is a normalized thrust/chamber-pressure fraction in [0,1].  A
    separate rate state permits ignition, shutdown, and throttle transients to
    be represented without embedding mutable engine logic in the ODE RHS.
    """
    command: float | ScalarProvider
    natural_frequency_hz: float = 3.0
    damping_ratio: float = 1.0
    max_rate: float = np.inf
    max_acceleration: float = np.inf
    power_key: str = "engine_power"
    rate_key: str = "engine_power_rate"

    def __post_init__(self)->None:
        if not np.isfinite(self.natural_frequency_hz) or self.natural_frequency_hz<=0: raise ValueError("natural_frequency_hz must be positive")
        if not np.isfinite(self.damping_ratio) or self.damping_ratio<0: raise ValueError("damping_ratio must be non-negative")
        for name in ("max_rate","max_acceleration"):
            v=float(getattr(self,name))
            if not (np.isfinite(v) or np.isinf(v)) or v<=0: raise ValueError(f"{name} must be positive")
        if not callable(self.command):
            v=float(self.command)
            if not np.isfinite(v) or not 0<=v<=1: raise ValueError("engine command must lie in [0,1]")

    def command_value(self,state:StateView)->float:
        u=float(self.command(state) if callable(self.command) else self.command)
        if not np.isfinite(u): raise ValueError("engine command is non-finite")
        return float(np.clip(u,0.0,1.0))

    def __call__(self, state: StateView) -> float:
        """Return the physically bounded engine power for propulsion closures."""
        return float(np.clip(state.get(self.power_key), 0.0, 1.0))

    def derivatives(self,state:StateView)->dict[str,float]:
        x=float(state.get(self.power_key)); v=float(state.get(self.rate_key)); u=self.command_value(state)
        wn=2*np.pi*self.natural_frequency_hz
        a=wn*wn*(u-x)-2*self.damping_ratio*wn*v
        a=float(np.clip(a,-self.max_acceleration,self.max_acceleration))
        xdot=float(np.clip(v,-self.max_rate,self.max_rate))
        # Hard-stop protection.  Damp outward velocity at the stops.
        if x<=0 and xdot<0: xdot=0.0
        if x>=1 and xdot>0: xdot=0.0
        if x<=0 and v<0 and a<0: a=0.0
        if x>=1 and v>0 and a>0: a=0.0
        return {self.power_key:xdot,self.rate_key:a}
