from __future__ import annotations
from dataclasses import dataclass
import math
import numpy as np

from .environment import EnvironmentSample


@dataclass(frozen=True, slots=True)
class FlowState:
    relative_velocity_i: np.ndarray
    speed: float
    dynamic_pressure: float
    mach: float
    reynolds: float
    knudsen: float


def compute_flow_state(velocity_i: np.ndarray, env: EnvironmentSample, reference_length: float) -> FlowState:
    if not np.isfinite(reference_length) or reference_length <= 0:
        raise ValueError("reference_length must be finite and positive")
    v = np.asarray(velocity_i, dtype=float)
    if v.shape != (3,) or not np.all(np.isfinite(v)):
        raise ValueError("velocity_i must be a finite 3-vector")
    rel = v - env.fluid_velocity_i
    speed = float(np.linalg.norm(rel))
    atm = env.atmosphere
    q = 0.5 * atm.density * speed * speed
    mach = 0.0 if speed == 0.0 else (speed / atm.speed_of_sound if np.isfinite(atm.speed_of_sound) else 0.0)
    re = 0.0 if atm.viscosity <= 0.0 else atm.density * speed * reference_length / atm.viscosity
    kn = math.inf if not np.isfinite(atm.mean_free_path) else atm.mean_free_path / reference_length
    return FlowState(rel, speed, q, float(mach), float(re), float(kn))
