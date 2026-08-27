from __future__ import annotations
from dataclasses import dataclass
from typing import Protocol
import numpy as np

from .chemistry import ChemistryCorrectionModel, FrozenChemistry, ThermochemicalCorrection
from .environment import EnvironmentSample, PlanetaryEnvironment
from .flow import FlowState, compute_flow_state
from .state import StateView


@dataclass(frozen=True, slots=True)
class AerothermalEvaluation:
    environment: EnvironmentSample
    flow: FlowState
    chemistry: ThermochemicalCorrection
    convective_heat_flux: float
    radiative_heat_flux: float
    total_heat_flux: float

    def __post_init__(self) -> None:
        vals = np.asarray([
            self.convective_heat_flux, self.radiative_heat_flux, self.total_heat_flux
        ], dtype=float)
        if not np.all(np.isfinite(vals)) or np.any(vals < 0):
            raise ValueError("heat fluxes must be finite and non-negative")


class AerothermalModel(Protocol):
    def evaluate(self, state: StateView) -> AerothermalEvaluation: ...


@dataclass(frozen=True, slots=True)
class PowerLawRadiativeHeating:
    """Replaceable empirical radiative-heating closure.

    q_rad = C * rho**density_exponent * V**speed_exponent.
    Set coefficient to zero to disable radiative heating. Because radiative
    entry heating is strongly chemistry/body dependent, the coefficient and
    exponents are intentionally user/model supplied rather than Earth constants.
    """

    coefficient: float = 0.0
    density_exponent: float = 1.0
    speed_exponent: float = 3.0

    def __post_init__(self) -> None:
        vals = (self.coefficient, self.density_exponent, self.speed_exponent)
        if not all(np.isfinite(v) for v in vals) or self.coefficient < 0:
            raise ValueError("radiative-heating parameters must be finite; coefficient non-negative")

    def heat_flux(self, environment: EnvironmentSample, flow: FlowState) -> float:
        if environment.atmosphere.density <= 0 or flow.speed <= 0 or self.coefficient == 0:
            return 0.0
        return float(
            self.coefficient
            * environment.atmosphere.density ** self.density_exponent
            * flow.speed ** self.speed_exponent
        )


@dataclass(frozen=True, slots=True)
class SuttonGravesHeating:
    """Generalized stagnation-point convective entry-heating closure.

    q_conv = k * sqrt(rho/R_n) * V^3 * chemistry_multiplier

    ``coefficient`` is explicitly supplied in coherent SI units because the
    Sutton-Graves constant depends on gas system and unit convention. This
    keeps the framework celestial-body and atmospheric-composition agnostic.
    """

    environment: PlanetaryEnvironment
    reference_length: float
    nose_radius: float
    coefficient: float
    chemistry: ChemistryCorrectionModel = FrozenChemistry()
    radiative_model: PowerLawRadiativeHeating = PowerLawRadiativeHeating()

    def __post_init__(self) -> None:
        for name in ("reference_length", "nose_radius", "coefficient"):
            v = float(getattr(self, name))
            if not np.isfinite(v) or v <= 0:
                raise ValueError(f"{name} must be finite and positive")

    def evaluate(self, state: StateView) -> AerothermalEvaluation:
        env = self.environment.query(state.get("position"), state.time)
        flow = compute_flow_state(state.get("velocity"), env, self.reference_length)
        correction = self.chemistry.evaluate(env, flow)
        rho = env.atmosphere.density
        if rho <= 0 or flow.speed <= 0:
            q_conv = 0.0
        else:
            q_conv = (
                self.coefficient
                * np.sqrt(rho / self.nose_radius)
                * flow.speed ** 3
                * correction.convective_heat_multiplier
            )
        q_rad = self.radiative_model.heat_flux(env, flow)
        return AerothermalEvaluation(env, flow, correction, float(q_conv), float(q_rad), float(q_conv + q_rad))
