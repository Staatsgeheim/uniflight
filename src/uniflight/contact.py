from __future__ import annotations
from dataclasses import dataclass
import numpy as np

from .frames import body_to_inertial_matrix, inertial_to_body_matrix
from .mass_properties import MassPropertiesModel
from .state import StateView
from .terrain import TerrainModel, TerrainSample
from .wrenches import Wrench


@dataclass(frozen=True, slots=True)
class GearLeg:
    stowed_foot_b: np.ndarray
    deployed_foot_b: np.ndarray
    stiffness: float
    damping: float
    friction_coefficient: float = 0.0

    def __post_init__(self) -> None:
        for name in ("stowed_foot_b", "deployed_foot_b"):
            a = np.asarray(getattr(self, name), dtype=float)
            if a.shape != (3,) or not np.all(np.isfinite(a)):
                raise ValueError(f"{name} must be a finite 3-vector")
            object.__setattr__(self, name, a.copy())
        if not np.isfinite(self.stiffness) or self.stiffness <= 0:
            raise ValueError("stiffness must be finite and positive")
        if not np.isfinite(self.damping) or self.damping < 0:
            raise ValueError("damping must be finite and non-negative")
        if not np.isfinite(self.friction_coefficient) or self.friction_coefficient < 0:
            raise ValueError("friction_coefficient must be finite and non-negative")


@dataclass(frozen=True, slots=True)
class LegContactEvaluation:
    foot_position_i: np.ndarray
    foot_velocity_i: np.ndarray
    terrain: TerrainSample
    penetration: float
    normal_speed: float
    normal_force: float
    friction_force_i: np.ndarray
    total_force_i: np.ndarray
    moment_b_about_cg: np.ndarray


@dataclass(frozen=True, slots=True)
class LandingGearEvaluation:
    deployment: float
    legs: tuple[LegContactEvaluation, ...]
    force_i: np.ndarray
    moment_b_about_cg: np.ndarray
    in_contact: bool


@dataclass(frozen=True, slots=True)
class LandingGearContact:
    """Penalty-contact landing gear with regularized Coulomb friction."""

    terrain: TerrainModel
    mass_properties: MassPropertiesModel
    legs: tuple[GearLeg, ...]
    deployment_state_key: str = "gear_deployment"
    friction_velocity_scale: float = 0.1
    active_threshold: float = 0.95
    source: str = "landing-gear-contact"

    def __post_init__(self) -> None:
        if not self.legs:
            raise ValueError("at least one gear leg is required")
        if not np.isfinite(self.friction_velocity_scale) or self.friction_velocity_scale <= 0:
            raise ValueError("friction_velocity_scale must be finite and positive")
        if not np.isfinite(self.active_threshold) or not 0 <= self.active_threshold <= 1:
            raise ValueError("active_threshold must lie in [0,1]")

    def _leg_kinematics(self, state: StateView, foot_b: np.ndarray, cg_b: np.ndarray):
        R_ib = body_to_inertial_matrix(state.get("attitude"))
        arm_b = foot_b - cg_b
        foot_i = np.asarray(state.get("position"), dtype=float) + R_ib @ arm_b
        omega_b = np.asarray(state.get("angular_rate"), dtype=float)
        foot_vel_i = np.asarray(state.get("velocity"), dtype=float) + R_ib @ np.cross(omega_b, arm_b)
        return foot_i, foot_vel_i, arm_b, R_ib

    def evaluate(self, state: StateView) -> LandingGearEvaluation:
        eta = float(np.clip(state.get(self.deployment_state_key), 0.0, 1.0))
        mp = self.mass_properties.evaluate(state)
        total_f = np.zeros(3)
        total_m = np.zeros(3)
        evaluations: list[LegContactEvaluation] = []
        active = eta >= self.active_threshold
        for leg in self.legs:
            foot_b = leg.stowed_foot_b + eta * (leg.deployed_foot_b - leg.stowed_foot_b)
            foot_i, foot_vel_i, arm_b, R_ib = self._leg_kinematics(state, foot_b, mp.cg_b)
            ts = self.terrain.query(foot_i, state.time)
            rel_v = foot_vel_i - ts.surface_velocity_i
            vn = float(np.dot(rel_v, ts.normal_i))
            penetration = max(0.0, -ts.agl) if active else 0.0
            normal_force = 0.0
            friction = np.zeros(3)
            force = np.zeros(3)
            moment_b = np.zeros(3)
            if penetration > 0.0:
                compression_rate = -vn
                normal_force = max(0.0, leg.stiffness * penetration + leg.damping * compression_rate)
                normal = normal_force * ts.normal_i
                vt = rel_v - vn * ts.normal_i
                vt_mag = float(np.linalg.norm(vt))
                if vt_mag > 0.0 and leg.friction_coefficient > 0.0 and normal_force > 0.0:
                    regularization = np.tanh(vt_mag / self.friction_velocity_scale)
                    friction = -leg.friction_coefficient * normal_force * regularization * vt / vt_mag
                force = normal + friction
                force_b = R_ib.T @ force
                moment_b = np.cross(arm_b, force_b)
                total_f += force
                total_m += moment_b
            evaluations.append(LegContactEvaluation(
                foot_i, foot_vel_i, ts, float(penetration), vn, float(normal_force),
                friction, force, moment_b
            ))
        return LandingGearEvaluation(eta, tuple(evaluations), total_f, total_m, any(e.penetration > 0 for e in evaluations))

    def wrench(self, state: StateView) -> Wrench:
        e = self.evaluate(state)
        return Wrench(e.force_i, e.moment_b_about_cg, self.source)

    def minimum_foot_agl(self, state: StateView) -> float:
        eta = float(np.clip(state.get(self.deployment_state_key), 0.0, 1.0))
        mp = self.mass_properties.evaluate(state)
        values = []
        for leg in self.legs:
            foot_b = leg.stowed_foot_b + eta * (leg.deployed_foot_b - leg.stowed_foot_b)
            foot_i, *_ = self._leg_kinematics(state, foot_b, mp.cg_b)
            values.append(self.terrain.query(foot_i, state.time).agl)
        return float(min(values))
