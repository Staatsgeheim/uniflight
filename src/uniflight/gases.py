from __future__ import annotations

from dataclasses import dataclass
import math
import numpy as np

R_UNIVERSAL = 8.31446261815324  # J/(mol K), exact to CODATA definition precision used here
BOLTZMANN = 1.380649e-23        # J/K, exact SI definition


@dataclass(frozen=True, slots=True)
class GasSpecies:
    """Thermophysical constants for one neutral gas species.

    Parameters are SI. ``cp_molar`` is treated as constant over the temperature
    range of a Milestone-B model. Later fidelity levels may replace this closure
    with temperature-dependent polynomials or nonequilibrium chemistry.
    """

    name: str
    molar_mass: float                 # kg/mol
    cp_molar: float                   # J/(mol K)
    viscosity_ref: float              # Pa s
    viscosity_ref_temperature: float  # K
    sutherland_constant: float        # K
    collision_diameter: float         # m

    def __post_init__(self) -> None:
        vals = (
            self.molar_mass,
            self.cp_molar,
            self.viscosity_ref,
            self.viscosity_ref_temperature,
            self.collision_diameter,
        )
        if not self.name:
            raise ValueError("species name must be non-empty")
        if not all(np.isfinite(v) and v > 0 for v in vals):
            raise ValueError("species thermophysical constants must be finite and positive")
        if not np.isfinite(self.sutherland_constant) or self.sutherland_constant < 0:
            raise ValueError("sutherland_constant must be finite and non-negative")

    def viscosity(self, temperature: float) -> float:
        """Dynamic viscosity from Sutherland's law."""
        T = float(temperature)
        if not np.isfinite(T) or T <= 0:
            raise ValueError("temperature must be finite and positive")
        T0 = self.viscosity_ref_temperature
        S = self.sutherland_constant
        return self.viscosity_ref * (T / T0) ** 1.5 * (T0 + S) / (T + S)


@dataclass(frozen=True, slots=True)
class GasMixture:
    """Ideal, calorically-perfect gas mixture with Wilke viscosity mixing."""

    species: tuple[GasSpecies, ...]
    mole_fractions: tuple[float, ...]

    def __post_init__(self) -> None:
        if not self.species:
            raise ValueError("mixture requires at least one species")
        if len(self.species) != len(self.mole_fractions):
            raise ValueError("species and mole_fractions lengths differ")
        x = np.asarray(self.mole_fractions, dtype=float)
        if not np.all(np.isfinite(x)) or np.any(x < 0):
            raise ValueError("mole fractions must be finite and non-negative")
        total = float(np.sum(x))
        if total <= 0:
            raise ValueError("mole fractions must have positive sum")
        # Normalize intentionally: input atmospheric databases often have small
        # rounding error in constituent fractions.
        x = x / total
        object.__setattr__(self, "mole_fractions", tuple(float(v) for v in x))

    @property
    def x(self) -> np.ndarray:
        a = np.asarray(self.mole_fractions, dtype=float)
        a.flags.writeable = False
        return a

    @property
    def molar_mass(self) -> float:
        return float(sum(x * s.molar_mass for x, s in zip(self.mole_fractions, self.species)))

    @property
    def specific_gas_constant(self) -> float:
        return R_UNIVERSAL / self.molar_mass

    @property
    def cp_molar(self) -> float:
        return float(sum(x * s.cp_molar for x, s in zip(self.mole_fractions, self.species)))

    @property
    def cp_mass(self) -> float:
        return self.cp_molar / self.molar_mass

    @property
    def cv_mass(self) -> float:
        return self.cp_mass - self.specific_gas_constant

    @property
    def gamma(self) -> float:
        cv = self.cv_mass
        if cv <= 0:
            raise ValueError("mixture has non-positive cv; check cp data")
        return self.cp_mass / cv

    @property
    def mass_fractions(self) -> np.ndarray:
        Mbar = self.molar_mass
        y = np.asarray([x * s.molar_mass / Mbar for x, s in zip(self.mole_fractions, self.species)])
        y.flags.writeable = False
        return y

    @property
    def effective_collision_diameter(self) -> float:
        # Milestone-B closure. A collision-integral model can replace this later.
        return float(sum(x * s.collision_diameter for x, s in zip(self.mole_fractions, self.species)))

    def density(self, pressure: float, temperature: float) -> float:
        p = float(pressure)
        T = float(temperature)
        if p < 0 or not np.isfinite(p):
            raise ValueError("pressure must be finite and non-negative")
        if T <= 0 or not np.isfinite(T):
            raise ValueError("temperature must be finite and positive")
        return p / (self.specific_gas_constant * T)

    def speed_of_sound(self, temperature: float) -> float:
        T = float(temperature)
        if T <= 0 or not np.isfinite(T):
            raise ValueError("temperature must be finite and positive")
        return math.sqrt(self.gamma * self.specific_gas_constant * T)

    def viscosity(self, temperature: float) -> float:
        """Wilke mixture rule using species Sutherland viscosities."""
        x = np.asarray(self.mole_fractions, dtype=float)
        M = np.asarray([s.molar_mass for s in self.species], dtype=float)
        mu = np.asarray([s.viscosity(temperature) for s in self.species], dtype=float)
        n = len(self.species)
        phi = np.empty((n, n), dtype=float)
        for i in range(n):
            for j in range(n):
                if i == j:
                    phi[i, j] = 1.0
                else:
                    phi[i, j] = (
                        (1.0 + math.sqrt(mu[i] / mu[j]) * (M[j] / M[i]) ** 0.25) ** 2
                        / math.sqrt(8.0 * (1.0 + M[i] / M[j]))
                    )
        denom = phi @ x
        return float(np.sum(x * mu / denom))

    def mean_free_path(self, pressure: float, temperature: float) -> float:
        p = float(pressure)
        T = float(temperature)
        if p <= 0:
            return math.inf
        if T <= 0 or not np.isfinite(T):
            raise ValueError("temperature must be finite and positive")
        d = self.effective_collision_diameter
        return BOLTZMANN * T / (math.sqrt(2.0) * math.pi * d * d * p)
