from __future__ import annotations
from dataclasses import dataclass
import numpy as np

from .heating import AerothermalModel, AerothermalEvaluation
from .state import StateView

STEFAN_BOLTZMANN = 5.670374419e-8  # W m^-2 K^-4, exact-enough SI reference


@dataclass(frozen=True, slots=True)
class TPSEvaluation:
    aerothermal: AerothermalEvaluation
    temperature: float
    tps_mass: float
    incident_power: float
    emitted_power: float
    net_power_before_ablation: float
    ablation_mass_rate: float
    temperature_rate: float


@dataclass(frozen=True, slots=True)
class LumpedAblatingTPS:
    """Lumped thermal-protection reference model with recession mass loss.

    Below ``ablation_temperature`` all net power changes the thermal-node
    temperature. At/above the threshold, positive net power is consumed by
    ablation latent heat while negative net power cools the node. The model is
    intentionally low-order but establishes the complete state/mass coupling.
    """

    heating_model: AerothermalModel
    heated_area: float
    thermal_mass: float
    specific_heat: float
    emissivity: float
    ablation_temperature: float
    effective_heat_of_ablation: float
    temperature_key: str = "tps_temperature"
    heat_load_key: str = "heat_load"
    tps_mass_key: str = "tps_mass"

    def __post_init__(self) -> None:
        for name in ("heated_area","thermal_mass","specific_heat","ablation_temperature","effective_heat_of_ablation"):
            v = float(getattr(self,name))
            if not np.isfinite(v) or v <= 0:
                raise ValueError(f"{name} must be finite and positive")
        if not np.isfinite(self.emissivity) or not 0 <= self.emissivity <= 1:
            raise ValueError("emissivity must lie in [0,1]")

    def evaluate(self, state: StateView) -> TPSEvaluation:
        aero = self.heating_model.evaluate(state)
        T = float(state.get(self.temperature_key))
        m_tps = float(state.get(self.tps_mass_key))
        if T <= 0 or not np.isfinite(T):
            raise ValueError("TPS temperature must be finite and positive")
        if m_tps < 0 or not np.isfinite(m_tps):
            raise ValueError("TPS mass must be finite and non-negative")
        T_inf = max(0.0, float(aero.environment.atmosphere.temperature))
        incident = aero.total_heat_flux * self.heated_area
        emitted = self.emissivity * STEFAN_BOLTZMANN * self.heated_area * max(0.0, T**4 - T_inf**4)
        net = incident - emitted
        mdot_ablation = 0.0
        if T >= self.ablation_temperature and net > 0.0 and m_tps > 0.0:
            mdot_ablation = net / self.effective_heat_of_ablation
            dT = 0.0
        else:
            dT = net / (self.thermal_mass * self.specific_heat)
        return TPSEvaluation(aero,T,m_tps,incident,emitted,net,float(mdot_ablation),float(dT))

    def derivatives(self, state: StateView) -> dict[str, float]:
        e = self.evaluate(state)
        return {
            self.temperature_key: e.temperature_rate,
            self.heat_load_key: e.aerothermal.total_heat_flux,
            self.tps_mass_key: -e.ablation_mass_rate,
        }

    def mass_rate(self, state: StateView) -> float:
        """Signed total-vehicle mass rate due to TPS ablation (kg/s)."""
        return -self.evaluate(state).ablation_mass_rate
