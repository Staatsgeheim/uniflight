from __future__ import annotations
from dataclasses import dataclass
from typing import Protocol
import numpy as np

from .state import StateView


@dataclass(frozen=True, slots=True)
class MassProperties:
    mass: float
    cg_b: np.ndarray
    inertia_b: np.ndarray
    inertia_rate_b: np.ndarray

    def __post_init__(self) -> None:
        cg = np.asarray(self.cg_b, dtype=float)
        I = np.asarray(self.inertia_b, dtype=float)
        Idot = np.asarray(self.inertia_rate_b, dtype=float)
        if not np.isfinite(self.mass) or self.mass <= 0:
            raise ValueError("mass must be finite and positive")
        if cg.shape != (3,) or not np.all(np.isfinite(cg)):
            raise ValueError("cg_b must be a finite 3-vector")
        if I.shape != (3,3) or Idot.shape != (3,3):
            raise ValueError("inertia tensors must be 3x3")
        if not np.all(np.isfinite(I)) or not np.all(np.isfinite(Idot)):
            raise ValueError("inertia tensors must be finite")
        if not np.allclose(I, I.T, atol=1e-12, rtol=0):
            raise ValueError("inertia_b must be symmetric")
        eig = np.linalg.eigvalsh(I)
        if np.any(eig <= 0):
            raise ValueError("inertia_b must be positive definite")
        object.__setattr__(self, "cg_b", cg.copy())
        object.__setattr__(self, "inertia_b", I.copy())
        object.__setattr__(self, "inertia_rate_b", Idot.copy())


class MassPropertiesModel(Protocol):
    def evaluate(self, state: StateView) -> MassProperties: ...


@dataclass(frozen=True, slots=True)
class ConstantMassProperties:
    """Constant CG/inertia model whose mass is read from the canonical state."""

    inertia_b: np.ndarray
    cg_b: np.ndarray | None = None

    def __post_init__(self) -> None:
        I = np.asarray(self.inertia_b, dtype=float)
        cg = np.zeros(3) if self.cg_b is None else np.asarray(self.cg_b, dtype=float)
        # Reuse MassProperties validation with unit mass.
        MassProperties(1.0, cg, I, np.zeros((3,3)))
        object.__setattr__(self, "inertia_b", I.copy())
        object.__setattr__(self, "cg_b", cg.copy())

    def evaluate(self, state: StateView) -> MassProperties:
        return MassProperties(
            float(state.get("mass")), self.cg_b, self.inertia_b, np.zeros((3,3))
        )


@dataclass(frozen=True, slots=True)
class AffineMassProperties:
    """Simple mass-dependent CG and inertia closure for reference simulations.

    ``I(m) = I_ref + inertia_slope * (m - reference_mass)`` and similarly for
    the center of gravity. ``mass_rate_provider`` is optional and allows an
    analytical inertia-rate term to be supplied to Euler's equation.
    """

    reference_mass: float
    inertia_ref_b: np.ndarray
    inertia_slope_b_per_kg: np.ndarray
    cg_ref_b: np.ndarray
    cg_slope_b_per_kg: np.ndarray
    mass_rate_provider: object | None = None

    def __post_init__(self) -> None:
        if not np.isfinite(self.reference_mass) or self.reference_mass <= 0:
            raise ValueError("reference_mass must be finite and positive")
        for name in ("inertia_ref_b", "inertia_slope_b_per_kg"):
            a = np.asarray(getattr(self, name), dtype=float)
            if a.shape != (3,3) or not np.all(np.isfinite(a)):
                raise ValueError(f"{name} must be a finite 3x3 array")
            object.__setattr__(self, name, a.copy())
        for name in ("cg_ref_b", "cg_slope_b_per_kg"):
            a = np.asarray(getattr(self, name), dtype=float)
            if a.shape != (3,) or not np.all(np.isfinite(a)):
                raise ValueError(f"{name} must be a finite 3-vector")
            object.__setattr__(self, name, a.copy())

    def evaluate(self, state: StateView) -> MassProperties:
        m = float(state.get("mass"))
        dm = m - self.reference_mass
        I = self.inertia_ref_b + self.inertia_slope_b_per_kg * dm
        cg = self.cg_ref_b + self.cg_slope_b_per_kg * dm
        mdot = 0.0
        if self.mass_rate_provider is not None:
            provider = self.mass_rate_provider
            mdot = float(provider(state) if callable(provider) else provider)
        Idot = self.inertia_slope_b_per_kg * mdot
        return MassProperties(m, cg, I, Idot)
