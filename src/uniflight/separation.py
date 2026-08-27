from __future__ import annotations
from dataclasses import dataclass
import numpy as np

from .state import StateSchema


@dataclass(frozen=True, slots=True)
class SeparatedBodyState:
    mass: float
    position_i: np.ndarray
    velocity_i: np.ndarray


@dataclass(frozen=True, slots=True)
class TwoBodySeparationResult:
    retained: SeparatedBodyState
    detached: SeparatedBodyState
    relative_velocity_i: np.ndarray
    momentum_error_i: np.ndarray


def separate_two_body(
    parent_mass: float,
    parent_position_i: np.ndarray,
    parent_velocity_i: np.ndarray,
    retained_mass: float,
    detached_mass: float,
    relative_separation_velocity_i: np.ndarray | None = None,
) -> TwoBodySeparationResult:
    """Split a parent into two co-located daughter bodies conserving momentum.

    ``relative_separation_velocity_i = v_detached - v_retained``. The caller may
    subsequently integrate each daughter using its own vehicle/environment model.
    """
    M = float(parent_mass)
    m1 = float(retained_mass)
    m2 = float(detached_mass)
    if not all(np.isfinite(v) and v > 0 for v in (M,m1,m2)):
        raise ValueError("masses must be finite and positive")
    if abs((m1 + m2) - M) > max(1e-10, 1e-12*M):
        raise ValueError("daughter masses must sum to parent mass")
    r = np.asarray(parent_position_i, dtype=float)
    V = np.asarray(parent_velocity_i, dtype=float)
    dv = np.zeros(3) if relative_separation_velocity_i is None else np.asarray(relative_separation_velocity_i, dtype=float)
    if any(a.shape != (3,) or not np.all(np.isfinite(a)) for a in (r,V,dv)):
        raise ValueError("positions and velocities must be finite 3-vectors")
    v1 = V - (m2/M) * dv
    v2 = V + (m1/M) * dv
    p_before = M * V
    p_after = m1*v1 + m2*v2
    return TwoBodySeparationResult(
        SeparatedBodyState(m1, r.copy(), v1),
        SeparatedBodyState(m2, r.copy(), v2),
        dv.copy(), p_after-p_before,
    )


@dataclass(frozen=True, slots=True)
class JettisonJump:
    """Hybrid jump map removing a fixed mass and optionally resetting fields."""

    schema: StateSchema
    jettison_mass: float
    reset_fields: dict[str, float | np.ndarray] | None = None
    minimum_remaining_mass: float = 0.0

    def __post_init__(self) -> None:
        if not np.isfinite(self.jettison_mass) or self.jettison_mass < 0:
            raise ValueError("jettison_mass must be finite and non-negative")
        if not np.isfinite(self.minimum_remaining_mass) or self.minimum_remaining_mass < 0:
            raise ValueError("minimum_remaining_mass must be finite and non-negative")

    def __call__(self, time: float, packed: np.ndarray) -> np.ndarray:
        values = self.schema.unpack(np.asarray(packed, dtype=float))
        remaining = float(values["mass"]) - self.jettison_mass
        if remaining <= self.minimum_remaining_mass:
            raise ValueError("jettison would violate minimum remaining mass")
        values["mass"] = remaining
        for key, value in (self.reset_fields or {}).items():
            if key not in {f.key for f in self.schema.fields}:
                raise KeyError(f"unknown state field {key!r}")
            values[key] = value
        return self.schema.pack(values)
