from __future__ import annotations
from dataclasses import dataclass
import numpy as np

@dataclass(frozen=True, slots=True)
class PointMassGravity:
    """Spherical point-mass gravity for arbitrary central-body gravitational parameter mu."""
    mu: float

    def __post_init__(self):
        if not np.isfinite(self.mu) or self.mu <= 0:
            raise ValueError("mu must be finite and positive")

    def acceleration(self, position_i: np.ndarray, time: float = 0.0) -> np.ndarray:
        r = np.asarray(position_i, dtype=float)
        rnorm = np.linalg.norm(r)
        if rnorm <= 0:
            raise ValueError("Point-mass gravity undefined at r=0")
        return -self.mu * r / rnorm**3

    def potential(self, position_i: np.ndarray) -> float:
        rnorm = np.linalg.norm(np.asarray(position_i, dtype=float))
        if rnorm <= 0:
            raise ValueError("Point-mass potential undefined at r=0")
        return -self.mu / rnorm

    def jacobian(self, position_i: np.ndarray, time: float = 0.0) -> np.ndarray:
        """Analytical gravity-gradient matrix da/dr for point-mass gravity."""
        r = np.asarray(position_i, dtype=float)
        rnorm = np.linalg.norm(r)
        if r.shape != (3,) or rnorm <= 0:
            raise ValueError("Point-mass gravity Jacobian undefined at invalid position")
        return self.mu * (3.0*np.outer(r, r)/rnorm**5 - np.eye(3)/rnorm**3)
