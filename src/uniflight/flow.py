from __future__ import annotations
from dataclasses import dataclass
import math
import numpy as np

from .environment import EnvironmentSample
from .frames import inertial_to_body_matrix


@dataclass(frozen=True, slots=True)
class FlowState:
    relative_velocity_i: np.ndarray
    speed: float
    dynamic_pressure: float
    mach: float
    reynolds: float
    knudsen: float


@dataclass(frozen=True, slots=True)
class BodyFlowState:
    """Canonical 6-DOF aerodynamic flow state.

    Body axes follow the aerospace convention +x forward, +y right, +z down.
    ``alpha = atan2(w,u)`` and ``beta = asin(v/V)`` for vehicle-relative air
    velocity [u,v,w] in B. ``rotation_bw`` maps wind-frame components to B.
    Wind +x is aligned with vehicle-relative air velocity, +y points right,
    and +z completes a right-handed frame; aerodynamic force components in W
    therefore use [-D, +Y, -L].
    """

    base: FlowState
    relative_velocity_b: np.ndarray
    alpha: float
    beta: float
    rotation_bw: np.ndarray

    @property
    def relative_velocity_i(self) -> np.ndarray: return self.base.relative_velocity_i
    @property
    def speed(self) -> float: return self.base.speed
    @property
    def dynamic_pressure(self) -> float: return self.base.dynamic_pressure
    @property
    def mach(self) -> float: return self.base.mach
    @property
    def reynolds(self) -> float: return self.base.reynolds
    @property
    def knudsen(self) -> float: return self.base.knudsen


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
    return FlowState(rel.copy(), speed, q, float(mach), float(re), float(kn))


def wind_to_body_matrix(alpha: float, beta: float) -> np.ndarray:
    """Return R_BW for the declared aerospace alpha/beta convention."""
    ca, sa = math.cos(alpha), math.sin(alpha)
    cb, sb = math.cos(beta), math.sin(beta)
    return np.array([
        [ca*cb, -ca*sb, -sa],
        [sb,     cb,       0.0],
        [sa*cb, -sa*sb,  ca],
    ])


def compute_body_flow_state(
    velocity_i: np.ndarray,
    attitude_b_to_i: np.ndarray,
    env: EnvironmentSample,
    reference_length: float,
) -> BodyFlowState:
    base = compute_flow_state(velocity_i, env, reference_length)
    R_bi = inertial_to_body_matrix(attitude_b_to_i)
    vb = R_bi @ base.relative_velocity_i
    if base.speed == 0.0:
        alpha = beta = 0.0
    else:
        u, v, w = vb
        alpha = math.atan2(w, u)
        beta = math.asin(float(np.clip(v / base.speed, -1.0, 1.0)))
    R_bw = wind_to_body_matrix(alpha, beta)
    return BodyFlowState(base, vb, float(alpha), float(beta), R_bw)
