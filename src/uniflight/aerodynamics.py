from __future__ import annotations
from dataclasses import dataclass, field
from typing import Protocol
import numpy as np

from .environment import PlanetaryEnvironment, EnvironmentSample
from .flow import FlowState, compute_flow_state
from .state import StateView


class DragCoefficientModel(Protocol):
    def __call__(self, flow: FlowState) -> float: ...


@dataclass(frozen=True, slots=True)
class ConstantDragCoefficient:
    cd: float

    def __post_init__(self) -> None:
        if not np.isfinite(self.cd) or self.cd < 0:
            raise ValueError("cd must be finite and non-negative")

    def __call__(self, flow: FlowState) -> float:
        return float(self.cd)


@dataclass(frozen=True, slots=True)
class MachTableDragCoefficient:
    mach: np.ndarray
    cd: np.ndarray

    def __post_init__(self) -> None:
        m = np.asarray(self.mach, dtype=float)
        c = np.asarray(self.cd, dtype=float)
        if m.ndim != 1 or c.ndim != 1 or len(m) < 2 or m.shape != c.shape:
            raise ValueError("mach and cd must be same-length 1-D arrays with at least two points")
        if not np.all(np.isfinite(m)) or not np.all(np.isfinite(c)) or np.any(c < 0):
            raise ValueError("table values must be finite and cd non-negative")
        if np.any(np.diff(m) <= 0):
            raise ValueError("mach table must be strictly increasing")
        object.__setattr__(self, "mach", m.copy())
        object.__setattr__(self, "cd", c.copy())

    def __call__(self, flow: FlowState) -> float:
        return float(np.interp(flow.mach, self.mach, self.cd))


@dataclass(frozen=True, slots=True)
class AeroEvaluation:
    environment: EnvironmentSample
    flow: FlowState
    cd: float
    force_i: np.ndarray


@dataclass(frozen=True, slots=True)
class ContinuumDrag:
    """Milestone-B point-mass continuum drag model."""

    environment: PlanetaryEnvironment
    reference_area: float
    reference_length: float
    coefficient: DragCoefficientModel
    max_knudsen: float | None = None

    def __post_init__(self) -> None:
        if not np.isfinite(self.reference_area) or self.reference_area <= 0:
            raise ValueError("reference_area must be finite and positive")
        if not np.isfinite(self.reference_length) or self.reference_length <= 0:
            raise ValueError("reference_length must be finite and positive")
        if self.max_knudsen is not None and (not np.isfinite(self.max_knudsen) or self.max_knudsen <= 0):
            raise ValueError("max_knudsen must be finite and positive")

    def evaluate(self, state: StateView) -> AeroEvaluation:
        env = self.environment.query(state.get("position"), state.time)
        flow = compute_flow_state(state.get("velocity"), env, self.reference_length)
        if self.max_knudsen is not None and flow.knudsen > self.max_knudsen and flow.dynamic_pressure > 0:
            raise ValueError(
                f"continuum model outside declared Knudsen validity: Kn={flow.knudsen:g} > {self.max_knudsen:g}"
            )
        cd = float(self.coefficient(flow))
        if not np.isfinite(cd) or cd < 0:
            raise ValueError("drag coefficient model returned invalid cd")
        if flow.speed == 0.0 or flow.dynamic_pressure == 0.0 or cd == 0.0:
            force = np.zeros(3)
        else:
            force = -flow.dynamic_pressure * self.reference_area * cd * flow.relative_velocity_i / flow.speed
        return AeroEvaluation(env, flow, cd, force)

    def force_i(self, state: StateView) -> np.ndarray:
        return self.evaluate(state).force_i

    def acceleration(self, state: StateView) -> np.ndarray:
        m = float(state.get("mass"))
        if m <= 0:
            raise ValueError("vehicle mass must remain positive")
        return self.force_i(state) / m

# ---------------------------------------------------------------------------
# Milestone C: 6-DOF continuum aerodynamics
# ---------------------------------------------------------------------------
from scipy.interpolate import RegularGridInterpolator
from .flow import BodyFlowState, compute_body_flow_state
from .frames import body_to_inertial_matrix
from .mass_properties import MassPropertiesModel
from .wrenches import Wrench


@dataclass(frozen=True, slots=True)
class AeroCoefficients:
    """Wind-axis force coefficients and body-axis moment coefficients."""
    cd: float
    cl: float = 0.0       # lift, positive upward (-z_W force)
    cy: float = 0.0       # side force, positive +y_W
    c_roll: float = 0.0   # body x roll moment
    c_pitch: float = 0.0  # body y pitch moment
    c_yaw: float = 0.0    # body z yaw moment

    def __post_init__(self) -> None:
        vals = np.array([self.cd,self.cl,self.cy,self.c_roll,self.c_pitch,self.c_yaw], dtype=float)
        if not np.all(np.isfinite(vals)) or self.cd < 0:
            raise ValueError("aerodynamic coefficients must be finite and cd non-negative")


class AeroCoefficientModel6DOF(Protocol):
    def __call__(self, flow: BodyFlowState) -> AeroCoefficients: ...


@dataclass(frozen=True, slots=True)
class ConstantAeroCoefficients:
    coefficients: AeroCoefficients
    def __call__(self, flow: BodyFlowState) -> AeroCoefficients:
        return self.coefficients


@dataclass(frozen=True, slots=True)
class LinearStabilityAerodynamics:
    """Low-order coefficient closure useful for verification/reference flight.

    Alpha and beta are radians. Drag is ``cd0 + cd_alpha2*alpha^2 +
    cd_beta2*beta^2``; remaining coefficients are linear stability derivatives.
    """
    cd0: float
    cd_alpha2: float = 0.0
    cd_beta2: float = 0.0
    cl_alpha: float = 0.0
    cy_beta: float = 0.0
    c_roll_beta: float = 0.0
    c_pitch_alpha: float = 0.0
    c_yaw_beta: float = 0.0

    def __post_init__(self) -> None:
        vals = np.asarray([
            self.cd0,self.cd_alpha2,self.cd_beta2,self.cl_alpha,self.cy_beta,
            self.c_roll_beta,self.c_pitch_alpha,self.c_yaw_beta,
        ])
        if not np.all(np.isfinite(vals)) or self.cd0 < 0:
            raise ValueError("stability derivatives must be finite and cd0 non-negative")

    def __call__(self, flow: BodyFlowState) -> AeroCoefficients:
        a, b = flow.alpha, flow.beta
        return AeroCoefficients(
            cd=self.cd0 + self.cd_alpha2*a*a + self.cd_beta2*b*b,
            cl=self.cl_alpha*a,
            cy=self.cy_beta*b,
            c_roll=self.c_roll_beta*b,
            c_pitch=self.c_pitch_alpha*a,
            c_yaw=self.c_yaw_beta*b,
        )


@dataclass(frozen=True, slots=True)
class GridAeroCoefficientDatabase:
    """Trilinear Mach/alpha/beta coefficient database.

    Arrays must have shape ``(len(mach), len(alpha), len(beta))``. Queries are
    clamped to the tabulated domain, making validity behavior deterministic.
    Alpha and beta grids are in radians.
    """
    mach: np.ndarray
    alpha: np.ndarray
    beta: np.ndarray
    cd: np.ndarray
    cl: np.ndarray
    cy: np.ndarray
    c_roll: np.ndarray
    c_pitch: np.ndarray
    c_yaw: np.ndarray
    _cd_interp: object = field(init=False, repr=False, compare=False)
    _cl_interp: object = field(init=False, repr=False, compare=False)
    _cy_interp: object = field(init=False, repr=False, compare=False)
    _c_roll_interp: object = field(init=False, repr=False, compare=False)
    _c_pitch_interp: object = field(init=False, repr=False, compare=False)
    _c_yaw_interp: object = field(init=False, repr=False, compare=False)

    def __post_init__(self) -> None:
        axes = []
        for name in ("mach","alpha","beta"):
            a = np.asarray(getattr(self,name), dtype=float)
            if a.ndim != 1 or len(a) < 2 or np.any(np.diff(a) <= 0) or not np.all(np.isfinite(a)):
                raise ValueError(f"{name} grid must be finite, 1-D, strictly increasing, >=2 points")
            object.__setattr__(self, name, a.copy()); axes.append(a)
        shape = tuple(len(a) for a in axes)
        for name in ("cd","cl","cy","c_roll","c_pitch","c_yaw"):
            v = np.asarray(getattr(self,name), dtype=float)
            if v.shape != shape or not np.all(np.isfinite(v)):
                raise ValueError(f"{name} table must have shape {shape} and finite values")
            if name == "cd" and np.any(v < 0):
                raise ValueError("cd table cannot contain negative drag")
            object.__setattr__(self, name, v.copy())
        # Frozen dataclass can store private interpolators via object.__setattr__.
        for name in ("cd","cl","cy","c_roll","c_pitch","c_yaw"):
            interp = RegularGridInterpolator(tuple(axes), getattr(self,name), bounds_error=False, fill_value=None)
            object.__setattr__(self, f"_{name}_interp", interp)

    def __call__(self, flow: BodyFlowState) -> AeroCoefficients:
        p = np.array([
            np.clip(flow.mach, self.mach[0], self.mach[-1]),
            np.clip(flow.alpha, self.alpha[0], self.alpha[-1]),
            np.clip(flow.beta, self.beta[0], self.beta[-1]),
        ], dtype=float)
        vals = {name: float(np.asarray(getattr(self, f"_{name}_interp")(p)).item())
                for name in ("cd","cl","cy","c_roll","c_pitch","c_yaw")}
        return AeroCoefficients(**vals)


@dataclass(frozen=True, slots=True)
class GeometryEvaluation:
    reference_area: float
    reference_length: float
    reference_span: float
    reference_chord: float
    aerodynamic_center_b: np.ndarray

    def __post_init__(self) -> None:
        nums = [self.reference_area,self.reference_length,self.reference_span,self.reference_chord]
        if not all(np.isfinite(x) and x > 0 for x in nums):
            raise ValueError("reference dimensions must be finite and positive")
        p = np.asarray(self.aerodynamic_center_b, dtype=float)
        if p.shape != (3,) or not np.all(np.isfinite(p)):
            raise ValueError("aerodynamic_center_b must be a finite 3-vector")
        object.__setattr__(self, "aerodynamic_center_b", p.copy())


class GeometryModel(Protocol):
    def evaluate(self, flow: BodyFlowState, state: StateView) -> GeometryEvaluation: ...


@dataclass(frozen=True, slots=True)
class ConstantReferenceGeometry:
    reference_area: float
    reference_length: float
    reference_span: float
    reference_chord: float
    aerodynamic_center_b: np.ndarray

    def evaluate(self, flow: BodyFlowState, state: StateView) -> GeometryEvaluation:
        return GeometryEvaluation(
            self.reference_area,self.reference_length,self.reference_span,
            self.reference_chord,self.aerodynamic_center_b,
        )


@dataclass(frozen=True, slots=True)
class EllipsoidProjectedGeometry:
    """Attitude-dependent reference area from an ellipsoid projection.

    ``semi_axes_b = [a,b,c]``. The orthogonal projected area for unit flow
    direction n in B is ``pi*a*b*c*sqrt(sum(n_i^2/a_i^2))``.
    """
    semi_axes_b: np.ndarray
    reference_length: float
    reference_span: float
    reference_chord: float
    aerodynamic_center_b: np.ndarray

    def __post_init__(self) -> None:
        a = np.asarray(self.semi_axes_b, dtype=float)
        if a.shape != (3,) or not np.all(np.isfinite(a)) or np.any(a <= 0):
            raise ValueError("semi_axes_b must contain three positive finite values")
        object.__setattr__(self, "semi_axes_b", a.copy())

    def evaluate(self, flow: BodyFlowState, state: StateView) -> GeometryEvaluation:
        if flow.speed > 0:
            n = flow.relative_velocity_b / flow.speed
        else:
            n = np.array([1.0,0.0,0.0])
        a,b,c = self.semi_axes_b
        area = np.pi*a*b*c*np.sqrt((n[0]/a)**2 + (n[1]/b)**2 + (n[2]/c)**2)
        return GeometryEvaluation(area,self.reference_length,self.reference_span,
                                  self.reference_chord,self.aerodynamic_center_b)


@dataclass(frozen=True, slots=True)
class Aero6DOFEvaluation:
    environment: EnvironmentSample
    flow: BodyFlowState
    geometry: GeometryEvaluation
    coefficients: AeroCoefficients
    force_w: np.ndarray
    force_b: np.ndarray
    force_i: np.ndarray
    coefficient_moment_b: np.ndarray
    moment_b_about_cg: np.ndarray


@dataclass(frozen=True, slots=True)
class ContinuumAerodynamics6DOF:
    environment: PlanetaryEnvironment
    geometry: GeometryModel
    coefficient_model: AeroCoefficientModel6DOF
    mass_properties: MassPropertiesModel
    max_knudsen: float | None = None
    source: str = "continuum-aero-6dof"

    def evaluate(self, state: StateView) -> Aero6DOFEvaluation:
        env = self.environment.query(state.get("position"), state.time)
        # Geometry's reference length is required to construct flow. For dynamic
        # geometry, evaluate once with a harmless provisional flow, then recompute.
        # All current Milestone-C geometries keep reference_length independent of flow.
        if not hasattr(self.geometry, "reference_length"):
            raise TypeError("Milestone C geometry models must expose reference_length")
        L = float(getattr(self.geometry, "reference_length"))
        flow = compute_body_flow_state(state.get("velocity"), state.get("attitude"), env, L)
        geom = self.geometry.evaluate(flow, state)
        if geom.reference_length != L:
            flow = compute_body_flow_state(state.get("velocity"), state.get("attitude"), env, geom.reference_length)
        if self.max_knudsen is not None and flow.knudsen > self.max_knudsen and flow.dynamic_pressure > 0:
            raise ValueError(f"continuum model outside declared Knudsen validity: Kn={flow.knudsen:g}")
        coeff = self.coefficient_model(flow)
        qS = flow.dynamic_pressure * geom.reference_area
        force_w = qS * np.array([-coeff.cd, coeff.cy, -coeff.cl])
        force_b = flow.rotation_bw @ force_w
        force_i = body_to_inertial_matrix(state.get("attitude")) @ force_b
        coeff_m = qS * np.array([
            geom.reference_span*coeff.c_roll,
            geom.reference_chord*coeff.c_pitch,
            geom.reference_span*coeff.c_yaw,
        ])
        mp = self.mass_properties.evaluate(state)
        arm_b = geom.aerodynamic_center_b - mp.cg_b
        moment_b = coeff_m + np.cross(arm_b, force_b)
        return Aero6DOFEvaluation(env,flow,geom,coeff,force_w,force_b,force_i,coeff_m,moment_b)

    def wrench(self, state: StateView) -> Wrench:
        e = self.evaluate(state)
        return Wrench(e.force_i, e.moment_b_about_cg, self.source)
