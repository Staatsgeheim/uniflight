from __future__ import annotations
from dataclasses import dataclass
import numpy as np

from .frames import inertial_to_body_matrix
from .state import StateView


@dataclass(frozen=True, slots=True)
class SubsystemBundle:
    """Declarative grouping of subsystem contributions for vehicle assembly."""
    derivative_models: tuple[object,...] = ()
    wrench_models: tuple[object,...] = ()
    mass_flow_sources: tuple[object,...] = ()

    @classmethod
    def combine(cls,*bundles:"SubsystemBundle")->"SubsystemBundle":
        return cls(
            tuple(x for b in bundles for x in b.derivative_models),
            tuple(x for b in bundles for x in b.wrench_models),
            tuple(x for b in bundles for x in b.mass_flow_sources),
        )


@dataclass(frozen=True, slots=True)
class WrenchSpecificForceBodyProvider:
    """Body-frame specific force from selected non-slosh wrench models.

    This is an explicit coupling surface for low-order slosh excitation.  The
    caller chooses which rigid loads participate, avoiding an algebraic loop
    through the slosh reaction wrench itself.
    """
    wrench_models: tuple[object,...]
    mass_properties: object

    def __call__(self,state:StateView)->np.ndarray:
        total_i=np.zeros(3)
        for m in self.wrench_models: total_i += m.wrench(state).force_i
        mp=self.mass_properties.evaluate(state)
        return inertial_to_body_matrix(state.get("attitude")) @ (total_i/mp.mass)
