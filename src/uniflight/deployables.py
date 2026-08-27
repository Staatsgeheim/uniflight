from __future__ import annotations
from dataclasses import dataclass
from typing import Callable
import numpy as np

from .environment import PlanetaryEnvironment
from .frames import body_to_inertial_matrix, inertial_to_body_matrix
from .mass_properties import MassPropertiesModel
from .state import StateView
from .wrenches import Wrench

CommandProvider = Callable[[StateView], float]
CoefficientProvider = Callable[[float], float]


@dataclass(frozen=True, slots=True)
class FirstOrderDeployable:
    """Generic monotone deployable-state dynamics.

    The state field is constrained conceptually to [0, 1]. A command in [0, 1]
    is followed with first-order deployment/retraction time constants. Setting
    ``retract_time_constant=None`` makes the device irreversible.
    """

    state_key: str
    command: float | CommandProvider
    deploy_time_constant: float
    retract_time_constant: float | None = None

    def __post_init__(self) -> None:
        if not np.isfinite(self.deploy_time_constant) or self.deploy_time_constant <= 0:
            raise ValueError("deploy_time_constant must be finite and positive")
        if self.retract_time_constant is not None and (
            not np.isfinite(self.retract_time_constant) or self.retract_time_constant <= 0
        ):
            raise ValueError("retract_time_constant must be finite and positive when provided")
        if not callable(self.command):
            u = float(self.command)
            if not np.isfinite(u) or not 0.0 <= u <= 1.0:
                raise ValueError("command must lie in [0,1] or be callable")

    def _command(self, state: StateView) -> float:
        u = float(self.command(state) if callable(self.command) else self.command)
        if not np.isfinite(u) or not 0.0 <= u <= 1.0:
            raise ValueError("deployable command provider returned value outside [0,1]")
        return u

    def derivatives(self, state: StateView) -> dict[str, float]:
        eta = float(state.get(self.state_key))
        u = self._command(state)
        if not -1e-12 <= eta <= 1.0 + 1e-12:
            raise ValueError(f"{self.state_key} must remain in [0,1]")
        eta = float(np.clip(eta, 0.0, 1.0))
        if u >= eta:
            rate = (u - eta) / self.deploy_time_constant
        elif self.retract_time_constant is None:
            rate = 0.0
        else:
            rate = (u - eta) / self.retract_time_constant
        return {self.state_key: float(rate)}


@dataclass(frozen=True, slots=True)
class ParachuteEvaluation:
    deployment: float
    effective_area: float
    drag_coefficient: float
    relative_velocity_i: np.ndarray
    speed: float
    dynamic_pressure: float
    drag: float
    force_i: np.ndarray
    moment_b_about_cg: np.ndarray


@dataclass(frozen=True, slots=True)
class InflatingParachute:
    """Reference parachute model with dynamic inflation and 6-DOF wrench output.

    This is intentionally an engineering closure, not a canopy CFD model. The
    effective area is ``A_max * eta**area_exponent`` and force opposes local
    atmosphere-relative velocity. The attachment point allows the parachute to
    create a stabilizing moment about the current CG.
    """

    environment: PlanetaryEnvironment
    mass_properties: MassPropertiesModel
    maximum_area: float
    drag_coefficient: float | CoefficientProvider
    deployment: FirstOrderDeployable
    attachment_point_b: np.ndarray | None = None
    area_exponent: float = 1.0
    source: str = "parachute"

    def __post_init__(self) -> None:
        if not np.isfinite(self.maximum_area) or self.maximum_area <= 0:
            raise ValueError("maximum_area must be finite and positive")
        if not np.isfinite(self.area_exponent) or self.area_exponent <= 0:
            raise ValueError("area_exponent must be finite and positive")
        p = np.zeros(3) if self.attachment_point_b is None else np.asarray(self.attachment_point_b, dtype=float)
        if p.shape != (3,) or not np.all(np.isfinite(p)):
            raise ValueError("attachment_point_b must be a finite 3-vector")
        object.__setattr__(self, "attachment_point_b", p.copy())
        if not callable(self.drag_coefficient):
            cd = float(self.drag_coefficient)
            if not np.isfinite(cd) or cd < 0:
                raise ValueError("drag_coefficient must be finite and non-negative or callable")

    @property
    def state_key(self) -> str:
        return self.deployment.state_key

    def derivatives(self, state: StateView) -> dict[str, float]:
        return self.deployment.derivatives(state)

    def evaluate(self, state: StateView) -> ParachuteEvaluation:
        env = self.environment.query(state.get("position"), state.time)
        eta = float(np.clip(state.get(self.state_key), 0.0, 1.0))
        rel = np.asarray(state.get("velocity"), dtype=float) - env.fluid_velocity_i
        speed = float(np.linalg.norm(rel))
        q = 0.5 * env.atmosphere.density * speed * speed
        cd = float(self.drag_coefficient(speed) if callable(self.drag_coefficient) else self.drag_coefficient)
        if not np.isfinite(cd) or cd < 0:
            raise ValueError("drag coefficient provider returned invalid value")
        area = self.maximum_area * eta ** self.area_exponent
        drag = q * cd * area
        force_i = np.zeros(3) if speed == 0.0 else -drag * rel / speed
        mp = self.mass_properties.evaluate(state)
        arm_b = self.attachment_point_b - mp.cg_b
        force_b = inertial_to_body_matrix(state.get("attitude")) @ force_i
        moment_b = np.cross(arm_b, force_b)
        return ParachuteEvaluation(
            eta, float(area), cd, rel.copy(), speed, float(q), float(drag), force_i, moment_b
        )

    def wrench(self, state: StateView) -> Wrench:
        e = self.evaluate(state)
        return Wrench(e.force_i, e.moment_b_about_cg, self.source)
