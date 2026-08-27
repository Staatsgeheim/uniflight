from __future__ import annotations
from dataclasses import dataclass
from typing import Protocol
import numpy as np

from .state import StateView


class MassFlowSource(Protocol):
    """Signed vehicle mass-rate source in kg/s.

    Negative values remove mass from the canonical vehicle state, positive
    values add mass. Propellant exhaust, venting and ablation are therefore
    negative; ingestion would be positive.
    """

    def mass_rate(self, state: StateView) -> float: ...


@dataclass(frozen=True, slots=True)
class MassFlowAggregator:
    """Single owner for the canonical ``mass`` derivative.

    Milestone A-C intentionally enforced one derivative writer per state field.
    This aggregator preserves that invariant while allowing several physical
    mass-transfer processes to contribute to the total vehicle mass rate.
    """

    sources: tuple[MassFlowSource, ...]

    def derivatives(self, state: StateView) -> dict[str, float]:
        mdot = 0.0
        for source in self.sources:
            contribution = float(source.mass_rate(state))
            if not np.isfinite(contribution):
                raise FloatingPointError("mass-flow source returned a non-finite rate")
            mdot += contribution
        return {"mass": mdot}
