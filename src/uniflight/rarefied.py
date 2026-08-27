from __future__ import annotations
from dataclasses import dataclass
import math
import numpy as np

from .aerodynamics import (
    AeroCoefficientModel6DOF, Aero6DOFEvaluation, GeometryModel,
)
from .environment import PlanetaryEnvironment
from .flow import compute_body_flow_state
from .frames import body_to_inertial_matrix
from .mass_properties import MassPropertiesModel
from .state import StateView
from .wrenches import Wrench


@dataclass(frozen=True, slots=True)
class FreeMolecularAerodynamics6DOF:
    """Reference free-molecular wrench closure.

    The mechanics are identical to the coefficient-based wind-axis force
    convention, but the coefficient model is expected to contain free-
    molecular/DSMC-derived coefficients. A future Sentman/DSMC model can
    replace the coefficient closure without changing the regime dispatcher.
    """

    environment: PlanetaryEnvironment
    geometry: GeometryModel
    coefficient_model: AeroCoefficientModel6DOF
    mass_properties: MassPropertiesModel
    min_knudsen: float | None = None
    source: str = "free-molecular-aero-6dof"

    def evaluate(self, state: StateView) -> Aero6DOFEvaluation:
        env = self.environment.query(state.get("position"), state.time)
        if not hasattr(self.geometry, "reference_length"):
            raise TypeError("Milestone D geometry models must expose reference_length")
        L = float(getattr(self.geometry, "reference_length"))
        flow = compute_body_flow_state(state.get("velocity"), state.get("attitude"), env, L)
        geom = self.geometry.evaluate(flow, state)
        if geom.reference_length != L:
            flow = compute_body_flow_state(state.get("velocity"), state.get("attitude"), env, geom.reference_length)
        if self.min_knudsen is not None and flow.knudsen < self.min_knudsen and flow.dynamic_pressure > 0:
            raise ValueError(f"free-molecular model outside declared Knudsen validity: Kn={flow.knudsen:g}")
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


@dataclass(frozen=True, slots=True)
class RegimeAeroEvaluation:
    knudsen: float
    rarefied_fraction: float
    regime: str
    continuum: Aero6DOFEvaluation
    rarefied: Aero6DOFEvaluation
    force_i: np.ndarray
    moment_b_about_cg: np.ndarray


@dataclass(frozen=True, slots=True)
class RegimeBlendedAerodynamics6DOF:
    """Smooth continuum -> transitional -> free-molecular dispatcher.

    Blending is performed in log10(Kn) with a cubic smoothstep. This is a
    numerical bridge, not a universal physical transition law; validated
    bridging functions or direct DSMC data can replace it behind this API.
    """

    continuum: object
    rarefied: FreeMolecularAerodynamics6DOF
    continuum_knudsen: float = 0.01
    free_molecular_knudsen: float = 10.0
    source: str = "regime-blended-aero-6dof"

    def __post_init__(self) -> None:
        if not np.isfinite(self.continuum_knudsen) or self.continuum_knudsen <= 0:
            raise ValueError("continuum_knudsen must be finite and positive")
        if not np.isfinite(self.free_molecular_knudsen) or self.free_molecular_knudsen <= self.continuum_knudsen:
            raise ValueError("free_molecular_knudsen must exceed continuum_knudsen")

    def rarefied_fraction(self, knudsen: float) -> float:
        if math.isinf(knudsen):
            return 1.0
        if knudsen <= self.continuum_knudsen:
            return 0.0
        if knudsen >= self.free_molecular_knudsen:
            return 1.0
        x = (
            math.log10(knudsen)-math.log10(self.continuum_knudsen)
        ) / (
            math.log10(self.free_molecular_knudsen)-math.log10(self.continuum_knudsen)
        )
        return float(x*x*(3.0-2.0*x))

    def evaluate(self, state: StateView) -> RegimeAeroEvaluation:
        cont = self.continuum.evaluate(state)
        rare = self.rarefied.evaluate(state)
        kn = cont.flow.knudsen
        w = self.rarefied_fraction(kn)
        f = (1.0-w)*cont.force_i + w*rare.force_i
        m = (1.0-w)*cont.moment_b_about_cg + w*rare.moment_b_about_cg
        if w <= 0.0:
            regime = "continuum"
        elif w >= 1.0:
            regime = "free-molecular"
        else:
            regime = "transitional"
        return RegimeAeroEvaluation(kn,w,regime,cont,rare,f,m)

    def wrench(self, state: StateView) -> Wrench:
        e = self.evaluate(state)
        return Wrench(e.force_i, e.moment_b_about_cg, self.source)
