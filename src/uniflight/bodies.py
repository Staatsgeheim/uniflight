from __future__ import annotations
from dataclasses import dataclass, field
import numpy as np

from .gravity import PointMassGravity


@dataclass(frozen=True, slots=True)
class SphericalBody:
    """Minimal arbitrary rotating spherical celestial body."""

    mu: float
    radius: float
    rotation_vector_i: np.ndarray = field(default_factory=lambda: np.zeros(3))
    name: str = "unnamed-body"

    def __post_init__(self) -> None:
        omega = np.asarray(self.rotation_vector_i, dtype=float)
        if omega.shape != (3,) or not np.all(np.isfinite(omega)):
            raise ValueError("rotation_vector_i must be a finite 3-vector")
        if not np.isfinite(self.mu) or self.mu <= 0:
            raise ValueError("mu must be finite and positive")
        if not np.isfinite(self.radius) or self.radius <= 0:
            raise ValueError("radius must be finite and positive")
        object.__setattr__(self, "rotation_vector_i", omega.copy())

    @property
    def gravity(self) -> PointMassGravity:
        return PointMassGravity(self.mu)

    def altitude(self, position_i: np.ndarray) -> float:
        r = np.asarray(position_i, dtype=float)
        if r.shape != (3,) or not np.all(np.isfinite(r)):
            raise ValueError("position_i must be a finite 3-vector")
        return float(np.linalg.norm(r) - self.radius)

    def surface_normal_i(self, position_i: np.ndarray) -> np.ndarray:
        r = np.asarray(position_i, dtype=float)
        n = np.linalg.norm(r)
        if n <= 0:
            raise ValueError("surface normal undefined at body center")
        return r / n

    def rotational_velocity_i(self, position_i: np.ndarray) -> np.ndarray:
        return np.cross(self.rotation_vector_i, np.asarray(position_i, dtype=float))
