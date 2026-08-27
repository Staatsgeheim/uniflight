from __future__ import annotations
from dataclasses import dataclass
from typing import Protocol
import math
import numpy as np

from .gases import GasMixture


@dataclass(frozen=True, slots=True)
class AtmosphereSample:
    altitude: float
    temperature: float
    pressure: float
    density: float
    viscosity: float
    speed_of_sound: float
    mean_free_path: float
    mixture: GasMixture | None

    @property
    def is_vacuum(self) -> bool:
        return self.density <= 0.0 or self.pressure <= 0.0


class AtmosphereModel(Protocol):
    def query(self, altitude: float, time: float = 0.0) -> AtmosphereSample: ...


@dataclass(frozen=True, slots=True)
class VacuumAtmosphere:
    def query(self, altitude: float, time: float = 0.0) -> AtmosphereSample:
        return AtmosphereSample(
            altitude=float(altitude), temperature=0.0, pressure=0.0, density=0.0,
            viscosity=0.0, speed_of_sound=math.inf, mean_free_path=math.inf, mixture=None,
        )


@dataclass(frozen=True, slots=True)
class IsothermalHydrostaticAtmosphere:
    """Isothermal, fixed-composition atmosphere in spherical point-mass gravity.

    The pressure law is the exact hydrostatic integral for g(r)=mu/r^2:

        p(h) = p0 exp[ mu/(R*T) * (1/(R+h) - 1/R) ]

    rather than the constant-g exponential approximation.
    """

    surface_pressure: float
    temperature: float
    mixture: GasMixture
    body_mu: float
    reference_radius: float
    ceiling: float | None = None

    def __post_init__(self) -> None:
        vals = (self.surface_pressure, self.temperature, self.body_mu, self.reference_radius)
        if not all(np.isfinite(v) and v > 0 for v in vals):
            raise ValueError("atmosphere parameters must be finite and positive")
        if self.ceiling is not None and (not np.isfinite(self.ceiling) or self.ceiling <= 0):
            raise ValueError("ceiling must be finite and positive when provided")

    def query(self, altitude: float, time: float = 0.0) -> AtmosphereSample:
        h = float(altitude)
        r = self.reference_radius + h
        if not np.isfinite(h) or r <= 0:
            raise ValueError("altitude places query at/inside body center")
        if self.ceiling is not None and h >= self.ceiling:
            return VacuumAtmosphere().query(h, time)

        R = self.mixture.specific_gas_constant
        exponent = self.body_mu / (R * self.temperature) * (1.0 / r - 1.0 / self.reference_radius)
        # exp underflow to zero is physically harmless at extremely low pressure.
        p = self.surface_pressure * math.exp(exponent) if exponent > -745.0 else 0.0
        if p <= 0.0:
            return VacuumAtmosphere().query(h, time)
        rho = self.mixture.density(p, self.temperature)
        mu = self.mixture.viscosity(self.temperature)
        a = self.mixture.speed_of_sound(self.temperature)
        mfp = self.mixture.mean_free_path(p, self.temperature)
        return AtmosphereSample(h, self.temperature, p, rho, mu, a, mfp, self.mixture)
