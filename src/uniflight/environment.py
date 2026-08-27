from __future__ import annotations
from dataclasses import dataclass
from typing import Callable
import numpy as np

from .atmosphere import AtmosphereModel, AtmosphereSample, VacuumAtmosphere
from .bodies import SphericalBody

WindModel = Callable[[np.ndarray, float, AtmosphereSample], np.ndarray]


@dataclass(frozen=True, slots=True)
class EnvironmentSample:
    time: float
    position_i: np.ndarray
    altitude: float
    gravity_i: np.ndarray
    surface_normal_i: np.ndarray
    atmosphere: AtmosphereSample
    fluid_velocity_i: np.ndarray
    wind_velocity_i: np.ndarray


@dataclass(frozen=True, slots=True)
class PlanetaryEnvironment:
    body: SphericalBody
    atmosphere: AtmosphereModel = VacuumAtmosphere()
    wind_model: WindModel | None = None

    def query(self, position_i: np.ndarray, time: float = 0.0) -> EnvironmentSample:
        r = np.asarray(position_i, dtype=float)
        if r.shape != (3,) or not np.all(np.isfinite(r)):
            raise ValueError("position_i must be a finite 3-vector")
        h = self.body.altitude(r)
        atm = self.atmosphere.query(h, time)
        wind = np.zeros(3) if self.wind_model is None else np.asarray(self.wind_model(r, time, atm), dtype=float)
        if wind.shape != (3,) or not np.all(np.isfinite(wind)):
            raise ValueError("wind_model must return a finite inertial 3-vector")
        fluid = self.body.rotational_velocity_i(r) + wind
        return EnvironmentSample(
            time=float(time), position_i=r.copy(), altitude=h,
            gravity_i=self.body.gravity.acceleration(r, time),
            surface_normal_i=self.body.surface_normal_i(r), atmosphere=atm,
            fluid_velocity_i=fluid, wind_velocity_i=wind,
        )
