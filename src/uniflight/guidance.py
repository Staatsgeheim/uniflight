from __future__ import annotations
from dataclasses import dataclass
import numpy as np

from .environment import PlanetaryEnvironment
from .state import StateView
from .terrain import TerrainModel


@dataclass(frozen=True, slots=True)
class DescentGuidanceEvaluation:
    altitude_agl: float
    radial_speed: float
    desired_radial_speed: float
    gravity_magnitude: float
    commanded_outward_acceleration: float
    available_thrust: float
    throttle: float


@dataclass(frozen=True, slots=True)
class VerticalDescentThrottle:
    """Reference radial powered-descent guidance law.

    The law is intentionally simple and planet agnostic: it tracks a descent
    speed schedule while feeding forward local gravity. It assumes the engine's
    thrust direction is controlled separately to point along the local outward
    normal.
    """

    environment: PlanetaryEnvironment
    terrain: TerrainModel
    exhaust_velocity: float
    mdot_exhaust: float
    exit_area: float = 0.0
    exit_pressure: float = 0.0
    max_descent_speed: float = 40.0
    touchdown_speed: float = 1.0
    speed_slope: float = 0.02
    velocity_gain: float = 0.8

    def __post_init__(self) -> None:
        positive = (self.exhaust_velocity, self.mdot_exhaust, self.max_descent_speed,
                    self.touchdown_speed, self.speed_slope, self.velocity_gain)
        if not all(np.isfinite(x) and x > 0 for x in positive):
            raise ValueError("guidance parameters must be finite and positive")
        if not np.isfinite(self.exit_area) or self.exit_area < 0 or not np.isfinite(self.exit_pressure) or self.exit_pressure < 0:
            raise ValueError("exit_area and exit_pressure must be finite and non-negative")

    def evaluate(self, state: StateView) -> DescentGuidanceEvaluation:
        pos = np.asarray(state.get("position"), dtype=float)
        vel = np.asarray(state.get("velocity"), dtype=float)
        env = self.environment.query(pos, state.time)
        terrain = self.terrain.query(pos, state.time)
        n = terrain.normal_i
        vr = float(np.dot(vel - terrain.surface_velocity_i, n))
        h = max(0.0, terrain.agl)
        desired_mag = min(self.max_descent_speed, self.touchdown_speed + self.speed_slope*h)
        desired_vr = -desired_mag
        gmag = max(0.0, -float(np.dot(env.gravity_i, n)))
        correction = self.velocity_gain * (desired_vr - vr)
        outward_accel = max(0.0, gmag + correction)
        max_thrust = self.mdot_exhaust*self.exhaust_velocity + (self.exit_pressure-env.atmosphere.pressure)*self.exit_area
        max_thrust = max(0.0, float(max_thrust))
        required = float(state.get("mass")) * outward_accel
        throttle = 0.0 if max_thrust <= 0.0 else float(np.clip(required/max_thrust, 0.0, 1.0))
        return DescentGuidanceEvaluation(h, vr, desired_vr, gmag, outward_accel, max_thrust, throttle)

    def __call__(self, state: StateView) -> float:
        return self.evaluate(state).throttle
