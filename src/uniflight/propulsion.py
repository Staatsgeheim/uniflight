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

# ---------------------------------------------------------------------------
# Milestone C: body-mounted / thrust-vector-controlled propulsion
# ---------------------------------------------------------------------------
from .frames import body_to_inertial_matrix
from .mass_properties import MassPropertiesModel
from .wrenches import Wrench

AngleProvider = Callable[[StateView], float]


def _rot_y(angle: float) -> np.ndarray:
    c,s = np.cos(angle), np.sin(angle)
    return np.array([[c,0,s],[0,1,0],[-s,0,c]], dtype=float)


def _rot_z(angle: float) -> np.ndarray:
    c,s = np.cos(angle), np.sin(angle)
    return np.array([[c,-s,0],[s,c,0],[0,0,1]], dtype=float)


@dataclass(frozen=True, slots=True)
class Rocket6DOFEvaluation:
    ambient_pressure: float
    throttle: float
    mass_flow: float
    thrust: float
    pitch_gimbal: float
    yaw_gimbal: float
    direction_b: np.ndarray
    force_b: np.ndarray
    force_i: np.ndarray
    moment_b_about_cg: np.ndarray


@dataclass(frozen=True, slots=True)
class GimballedRocketEngine:
    """Pressure-corrected body-mounted rocket with two-axis TVC.

    The base thrust direction is expressed in B. Positive pitch applies a
    rotation about +y_B and positive yaw about +z_B. With the default +x_B
    engine direction and +z_B down, positive pitch produces a -z_B thrust
    component. The mounting position is expressed in B from the vehicle body
    reference origin; moment is evaluated about the instantaneous CG.
    """

    environment: PlanetaryEnvironment
    mass_properties: MassPropertiesModel
    exhaust_velocity: float
    mdot_exhaust: float
    exit_area: float = 0.0
    exit_pressure: float = 0.0
    mount_position_b: np.ndarray | None = None
    base_direction_b: np.ndarray | None = None
    throttle: float | ThrottleProvider = 1.0
    pitch_gimbal: float | AngleProvider = 0.0
    yaw_gimbal: float | AngleProvider = 0.0
    dry_mass: float = 0.0
    source: str = "gimballed-rocket"

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
        mount = np.zeros(3) if self.mount_position_b is None else np.asarray(self.mount_position_b, dtype=float)
        base = np.array([1.,0.,0.]) if self.base_direction_b is None else np.asarray(self.base_direction_b, dtype=float)
        if mount.shape != (3,) or not np.all(np.isfinite(mount)):
            raise ValueError("mount_position_b must be a finite 3-vector")
        if base.shape != (3,) or not np.all(np.isfinite(base)) or np.linalg.norm(base) == 0:
            raise ValueError("base_direction_b must be a nonzero finite 3-vector")
        object.__setattr__(self, "mount_position_b", mount.copy())
        object.__setattr__(self, "base_direction_b", base / np.linalg.norm(base))
        for name in ("throttle","pitch_gimbal","yaw_gimbal"):
            v = getattr(self, name)
            if not callable(v) and not np.isfinite(float(v)):
                raise ValueError(f"{name} must be finite or callable")
        if not callable(self.throttle) and not 0 <= float(self.throttle) <= 1:
            raise ValueError("throttle must lie in [0,1]")

    def _scalar(self, provider, state: StateView) -> float:
        return float(provider(state) if callable(provider) else provider)

    def _throttle(self, state: StateView) -> float:
        u = self._scalar(self.throttle, state)
        if not np.isfinite(u) or not 0 <= u <= 1:
            raise ValueError("throttle provider returned value outside [0,1]")
        if float(state.get("mass")) <= self.dry_mass:
            return 0.0
        return u

    def evaluate(self, state: StateView) -> Rocket6DOFEvaluation:
        env = self.environment.query(state.get("position"), state.time)
        u = self._throttle(state)
        pitch = self._scalar(self.pitch_gimbal, state)
        yaw = self._scalar(self.yaw_gimbal, state)
        if not np.isfinite(pitch) or not np.isfinite(yaw):
            raise ValueError("gimbal provider returned non-finite angle")
        direction_b = _rot_z(yaw) @ _rot_y(pitch) @ self.base_direction_b
        direction_b = direction_b / np.linalg.norm(direction_b)
        pa = env.atmosphere.pressure
        mdot = u * self.mdot_exhaust
        thrust = u * (self.mdot_exhaust*self.exhaust_velocity + (self.exit_pressure-pa)*self.exit_area)
        force_b = thrust * direction_b
        force_i = body_to_inertial_matrix(state.get("attitude")) @ force_b
        mp = self.mass_properties.evaluate(state)
        arm = self.mount_position_b - mp.cg_b
        moment_b = np.cross(arm, force_b)
        return Rocket6DOFEvaluation(pa,u,mdot,float(thrust),pitch,yaw,direction_b,force_b,force_i,moment_b)

    def wrench(self, state: StateView) -> Wrench:
        e = self.evaluate(state)
        return Wrench(e.force_i, e.moment_b_about_cg, self.source)

    def derivatives(self, state: StateView) -> dict[str, float]:
        return {"mass": -self.evaluate(state).mass_flow}
