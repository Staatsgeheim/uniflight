from __future__ import annotations
from dataclasses import dataclass, field
from typing import Callable
import numpy as np

from .state import StateView
from .wrenches import Wrench

CommandProvider = Callable[[StateView], float]


@dataclass(slots=True)
class GNCCommandBus:
    """Zero-order-held commands shared by sampled GNC and continuous plant models.

    The bus is intentionally mutable, but is only changed at controller sample
    boundaries. Continuous RHS evaluations therefore see constant commands
    during each integration segment.
    """
    throttle: float = 0.0
    pitch_gimbal: float = 0.0
    yaw_gimbal: float = 0.0
    torque_b: np.ndarray = field(default_factory=lambda: np.zeros(3))

    def set(self, *, throttle: float | None = None, pitch_gimbal: float | None = None,
            yaw_gimbal: float | None = None, torque_b: np.ndarray | None = None) -> None:
        if throttle is not None:
            u = float(throttle)
            if not np.isfinite(u) or not 0.0 <= u <= 1.0:
                raise ValueError("throttle command must be finite in [0,1]")
            self.throttle = u
        if pitch_gimbal is not None:
            a = float(pitch_gimbal)
            if not np.isfinite(a):
                raise ValueError("pitch_gimbal command must be finite")
            self.pitch_gimbal = a
        if yaw_gimbal is not None:
            a = float(yaw_gimbal)
            if not np.isfinite(a):
                raise ValueError("yaw_gimbal command must be finite")
            self.yaw_gimbal = a
        if torque_b is not None:
            t = np.asarray(torque_b, dtype=float)
            if t.shape != (3,) or not np.all(np.isfinite(t)):
                raise ValueError("torque_b command must be a finite 3-vector")
            self.torque_b = t.copy()


@dataclass(frozen=True, slots=True)
class StateFieldProvider:
    field_name: str

    def __call__(self, state: StateView) -> float:
        return float(state.get(self.field_name))


@dataclass(frozen=True, slots=True)
class BusScalarProvider:
    bus: GNCCommandBus
    attribute: str

    def __call__(self, state: StateView) -> float:  # noqa: ARG002 - provider API
        return float(getattr(self.bus, self.attribute))


@dataclass(frozen=True, slots=True)
class FirstOrderLimitedStateActuator:
    """First-order actuator state with position and slew-rate limits."""
    field_name: str
    command: float | CommandProvider
    time_constant: float
    lower: float
    upper: float
    rate_limit: float = np.inf

    def __post_init__(self) -> None:
        if not np.isfinite(self.time_constant) or self.time_constant <= 0:
            raise ValueError("time_constant must be finite and positive")
        if not (np.isfinite(self.lower) and np.isfinite(self.upper) and self.upper > self.lower):
            raise ValueError("actuator bounds must be finite and increasing")
        if not (np.isfinite(self.rate_limit) or np.isinf(self.rate_limit)) or self.rate_limit <= 0:
            raise ValueError("rate_limit must be positive")

    def command_value(self, state: StateView) -> float:
        u = float(self.command(state) if callable(self.command) else self.command)
        if not np.isfinite(u):
            raise ValueError("actuator command is non-finite")
        return float(np.clip(u, self.lower, self.upper))

    def derivatives(self, state: StateView) -> dict[str, float]:
        x = float(state.get(self.field_name))
        u = self.command_value(state)
        dx = (u - x) / self.time_constant
        dx = float(np.clip(dx, -self.rate_limit, self.rate_limit))
        # Prevent numerical drift past hard stops.
        if x <= self.lower and dx < 0:
            dx = 0.0
        if x >= self.upper and dx > 0:
            dx = 0.0
        return {self.field_name: dx}


@dataclass(frozen=True, slots=True)
class CommandedBodyTorque:
    """Ideal bounded body-moment actuator driven by a sampled command bus."""
    bus: GNCCommandBus
    max_torque_b: np.ndarray | float
    source: str = "commanded-body-torque"

    def __post_init__(self) -> None:
        lim = np.asarray(self.max_torque_b, dtype=float)
        if lim.ndim == 0:
            lim = np.full(3, float(lim))
        if lim.shape != (3,) or not np.all(np.isfinite(lim)) or np.any(lim <= 0):
            raise ValueError("max_torque_b must be a positive scalar or finite 3-vector")
        object.__setattr__(self, "max_torque_b", lim.copy())

    def wrench(self, state: StateView) -> Wrench:  # noqa: ARG002 - WrenchModel API
        cmd = np.asarray(self.bus.torque_b, dtype=float)
        torque = np.clip(cmd, -self.max_torque_b, self.max_torque_b)
        return Wrench(np.zeros(3), torque, self.source)
