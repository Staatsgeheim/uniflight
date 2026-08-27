from __future__ import annotations
from dataclasses import dataclass
from typing import Callable
import numpy as np

from .environment import PlanetaryEnvironment
from .state import StateView

DirectionProvider = Callable[[StateView], np.ndarray]
ThrottleProvider = Callable[[StateView], float]


@dataclass(frozen=True, slots=True)
class RocketEvaluation:
    ambient_pressure: float
    throttle: float
    mass_flow: float
    thrust: float
    direction_i: np.ndarray
    force_i: np.ndarray


@dataclass(frozen=True, slots=True)
class RocketEngine:
    """Pressure-corrected rocket engine for point-mass flight.

    ``mdot_exhaust`` is the positive nominal exhaust mass flow. The pressure
    thrust term is multiplied by throttle in this Milestone-B closure.
    """

    environment: PlanetaryEnvironment
    exhaust_velocity: float
    mdot_exhaust: float
    exit_area: float = 0.0
    exit_pressure: float = 0.0
    direction_i: np.ndarray | DirectionProvider | None = None
    throttle: float | ThrottleProvider = 1.0
    dry_mass: float = 0.0

    def __post_init__(self) -> None:
        if not np.isfinite(self.exhaust_velocity) or self.exhaust_velocity <= 0:
            raise ValueError("exhaust_velocity must be finite and positive")
        if not np.isfinite(self.mdot_exhaust) or self.mdot_exhaust <= 0:
            raise ValueError("mdot_exhaust must be finite and positive")
        if not np.isfinite(self.exit_area) or self.exit_area < 0:
            raise ValueError("exit_area must be finite and non-negative")
        if not np.isfinite(self.exit_pressure) or self.exit_pressure < 0:
            raise ValueError("exit_pressure must be finite and non-negative")
        if not np.isfinite(self.dry_mass) or self.dry_mass < 0:
            raise ValueError("dry_mass must be finite and non-negative")
        if self.direction_i is None:
            object.__setattr__(self, "direction_i", np.array([1.0, 0.0, 0.0]))
        elif not callable(self.direction_i):
            d = np.asarray(self.direction_i, dtype=float)
            if d.shape != (3,) or not np.all(np.isfinite(d)) or np.linalg.norm(d) == 0:
                raise ValueError("direction_i must be a nonzero finite 3-vector or callable")
            object.__setattr__(self, "direction_i", d / np.linalg.norm(d))
        if not callable(self.throttle):
            u = float(self.throttle)
            if not np.isfinite(u) or not (0.0 <= u <= 1.0):
                raise ValueError("throttle must lie in [0,1] or be callable")

    def _direction(self, state: StateView) -> np.ndarray:
        d = self.direction_i(state) if callable(self.direction_i) else self.direction_i
        d = np.asarray(d, dtype=float)
        n = np.linalg.norm(d)
        if d.shape != (3,) or not np.all(np.isfinite(d)) or n == 0:
            raise ValueError("direction provider returned invalid direction")
        return d / n

    def _throttle(self, state: StateView) -> float:
        u = float(self.throttle(state) if callable(self.throttle) else self.throttle)
        if not np.isfinite(u) or not (0.0 <= u <= 1.0):
            raise ValueError("throttle provider returned value outside [0,1]")
        if float(state.get("mass")) <= self.dry_mass:
            return 0.0
        return u

    def evaluate(self, state: StateView) -> RocketEvaluation:
        env = self.environment.query(state.get("position"), state.time)
        pa = env.atmosphere.pressure
        u = self._throttle(state)
        mdot = u * self.mdot_exhaust
        thrust = u * (self.mdot_exhaust * self.exhaust_velocity + (self.exit_pressure - pa) * self.exit_area)
        d = self._direction(state)
        force = thrust * d
        return RocketEvaluation(pa, u, mdot, float(thrust), d, force)

    def acceleration(self, state: StateView) -> np.ndarray:
        m = float(state.get("mass"))
        if m <= 0:
            raise ValueError("rocket mass must remain positive")
        return self.evaluate(state).force_i / m

    def derivatives(self, state: StateView) -> dict[str, float]:
        return {"mass": -self.evaluate(state).mass_flow}
