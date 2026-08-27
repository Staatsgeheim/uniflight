from __future__ import annotations
from dataclasses import dataclass
from typing import Callable, Protocol
import numpy as np

from .bodies import SphericalBody

ElevationProvider = Callable[[np.ndarray, float], float]


@dataclass(frozen=True, slots=True)
class TerrainSample:
    position_i: np.ndarray
    time: float
    elevation: float
    agl: float
    normal_i: np.ndarray
    surface_point_i: np.ndarray
    surface_velocity_i: np.ndarray


class TerrainModel(Protocol):
    def query(self, position_i: np.ndarray, time: float = 0.0) -> TerrainSample: ...


@dataclass(frozen=True, slots=True)
class RadialTerrain:
    """Reference terrain over a spherical body using radial elevation.

    ``elevation`` is measured from the body's reference radius. The surface
    normal is radial; this is exact for constant elevation and a deliberate
    low-order approximation for spatially varying elevation.
    """

    body: SphericalBody
    elevation: float | ElevationProvider = 0.0

    def __post_init__(self) -> None:
        if not callable(self.elevation) and not np.isfinite(float(self.elevation)):
            raise ValueError("elevation must be finite or callable")

    def query(self, position_i: np.ndarray, time: float = 0.0) -> TerrainSample:
        r = np.asarray(position_i, dtype=float)
        if r.shape != (3,) or not np.all(np.isfinite(r)):
            raise ValueError("position_i must be a finite 3-vector")
        radius = float(np.linalg.norm(r))
        if radius <= 0:
            raise ValueError("terrain query undefined at body center")
        n = r / radius
        elev = float(self.elevation(n.copy(), float(time)) if callable(self.elevation) else self.elevation)
        if not np.isfinite(elev):
            raise ValueError("terrain elevation provider returned non-finite value")
        terrain_radius = self.body.radius + elev
        if terrain_radius <= 0:
            raise ValueError("terrain elevation places surface at non-positive radius")
        surface_point = terrain_radius * n
        agl = radius - terrain_radius
        surface_velocity = self.body.rotational_velocity_i(surface_point)
        return TerrainSample(r.copy(), float(time), elev, float(agl), n, surface_point, surface_velocity)
