from __future__ import annotations
from dataclasses import dataclass
from typing import Protocol
import math
import numpy as np

from .flow import FlowState
from .environment import EnvironmentSample


@dataclass(frozen=True, slots=True)
class ThermochemicalCorrection:
    """Low-order thermochemical correction returned to entry closures.

    This is an interface object, not a chemical-kinetics solver. A future
    finite-rate chemistry module can return the same fields while evaluating
    detailed species/vibrational/electron states internally.
    """

    stagnation_temperature: float
    dissociation_fraction: float
    ionization_fraction: float
    convective_heat_multiplier: float

    def __post_init__(self) -> None:
        vals = np.asarray([
            self.stagnation_temperature,
            self.dissociation_fraction,
            self.ionization_fraction,
            self.convective_heat_multiplier,
        ], dtype=float)
        if not np.all(np.isfinite(vals)):
            raise ValueError("thermochemical correction must be finite")
        if self.stagnation_temperature < 0:
            raise ValueError("stagnation_temperature must be non-negative")
        if not (0 <= self.dissociation_fraction <= 1):
            raise ValueError("dissociation_fraction must lie in [0,1]")
        if not (0 <= self.ionization_fraction <= 1):
            raise ValueError("ionization_fraction must lie in [0,1]")
        if self.convective_heat_multiplier < 0:
            raise ValueError("convective_heat_multiplier must be non-negative")


class ChemistryCorrectionModel(Protocol):
    def evaluate(self, environment: EnvironmentSample, flow: FlowState) -> ThermochemicalCorrection: ...


def _perfect_gas_stagnation_temperature(environment: EnvironmentSample, flow: FlowState) -> float:
    atm = environment.atmosphere
    if atm.is_vacuum or atm.mixture is None:
        return 0.0
    gamma = atm.mixture.gamma
    return float(atm.temperature * (1.0 + 0.5 * (gamma - 1.0) * flow.mach * flow.mach))


@dataclass(frozen=True, slots=True)
class FrozenChemistry:
    """Frozen-composition reference closure with no chemistry heat sink."""

    def evaluate(self, environment: EnvironmentSample, flow: FlowState) -> ThermochemicalCorrection:
        return ThermochemicalCorrection(
            _perfect_gas_stagnation_temperature(environment, flow), 0.0, 0.0, 1.0
        )


@dataclass(frozen=True, slots=True)
class ThresholdDissociationCorrection:
    """Smooth reference hook for endothermic high-temperature chemistry.

    This deliberately is *not* a replacement for finite-rate chemistry. It is
    a deterministic placeholder that proves the software coupling: estimated
    perfect-gas stagnation temperature drives a smooth dissociation fraction,
    which reduces the convective heating closure by at most ``max_heat_sink``.
    A detailed chemistry solver can replace this class without changing the
    aerothermal/TPS interfaces.
    """

    onset_temperature: float
    full_temperature: float
    max_heat_sink: float = 0.25
    ionization_onset_temperature: float | None = None
    ionization_full_temperature: float | None = None

    def __post_init__(self) -> None:
        if not np.isfinite(self.onset_temperature) or self.onset_temperature <= 0:
            raise ValueError("onset_temperature must be finite and positive")
        if not np.isfinite(self.full_temperature) or self.full_temperature <= self.onset_temperature:
            raise ValueError("full_temperature must exceed onset_temperature")
        if not np.isfinite(self.max_heat_sink) or not 0 <= self.max_heat_sink < 1:
            raise ValueError("max_heat_sink must lie in [0,1)")
        a, b = self.ionization_onset_temperature, self.ionization_full_temperature
        if (a is None) != (b is None):
            raise ValueError("ionization onset/full temperatures must be supplied together")
        if a is not None:
            if not np.isfinite(a) or not np.isfinite(b) or a <= 0 or b <= a:
                raise ValueError("invalid ionization temperature interval")

    @staticmethod
    def _smooth_fraction(value: float, lo: float, hi: float) -> float:
        x = float(np.clip((value - lo) / (hi - lo), 0.0, 1.0))
        return x * x * (3.0 - 2.0 * x)

    def evaluate(self, environment: EnvironmentSample, flow: FlowState) -> ThermochemicalCorrection:
        t0 = _perfect_gas_stagnation_temperature(environment, flow)
        diss = self._smooth_fraction(t0, self.onset_temperature, self.full_temperature)
        ion = 0.0
        if self.ionization_onset_temperature is not None:
            ion = self._smooth_fraction(
                t0, self.ionization_onset_temperature, self.ionization_full_temperature
            )
        multiplier = 1.0 - self.max_heat_sink * diss
        return ThermochemicalCorrection(t0, diss, ion, multiplier)
