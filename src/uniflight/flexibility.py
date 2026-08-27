from __future__ import annotations
from dataclasses import dataclass
from typing import Callable
import numpy as np

from .frames import quat_multiply, quat_normalize
from .sensors import AttitudeRateMeasurement, AttitudeRateSensor
from .state import StateView

GeneralizedForceProvider = Callable[[StateView], np.ndarray]
TorqueProvider = Callable[[StateView], np.ndarray]


def _as_vector(value, n: int, name: str, *, positive: bool = False, nonnegative: bool = False) -> np.ndarray:
    a = np.asarray(value, dtype=float)
    if a.ndim == 0:
        a = np.full(n, float(a))
    if a.shape != (n,) or not np.all(np.isfinite(a)):
        raise ValueError(f"{name} must be a finite scalar or {n}-vector")
    if positive and np.any(a <= 0):
        raise ValueError(f"{name} must be positive")
    if nonnegative and np.any(a < 0):
        raise ValueError(f"{name} must be non-negative")
    return a


@dataclass(frozen=True, slots=True)
class ModalFlexibleBody:
    """Linear modal structural dynamics.

    Modal coordinates are generalized displacements in metres.  The model is
    intentionally matrix-light for a reference simulator: modal masses are
    diagonal and the user supplies natural frequencies and damping ratios.

        qdd + 2*zeta*wn*qd + wn^2*q = Q / m_modal

    ``generalized_force`` may be a constant vector or a pure callable of the
    current state.  This makes the model compatible with adaptive integrators.
    """

    natural_frequency_hz: np.ndarray | float
    damping_ratio: np.ndarray | float
    modal_mass: np.ndarray | float = 1.0
    generalized_force: np.ndarray | GeneralizedForceProvider | None = None
    displacement_key: str = "flex_displacement"
    velocity_key: str = "flex_velocity"

    def __post_init__(self) -> None:
        f = np.asarray(self.natural_frequency_hz, dtype=float)
        n = 1 if f.ndim == 0 else int(f.size)
        f = _as_vector(f, n, "natural_frequency_hz", positive=True)
        z = _as_vector(self.damping_ratio, n, "damping_ratio", nonnegative=True)
        mm = _as_vector(self.modal_mass, n, "modal_mass", positive=True)
        if self.generalized_force is not None and not callable(self.generalized_force):
            gf = _as_vector(self.generalized_force, n, "generalized_force")
            object.__setattr__(self, "generalized_force", gf)
        object.__setattr__(self, "natural_frequency_hz", f)
        object.__setattr__(self, "damping_ratio", z)
        object.__setattr__(self, "modal_mass", mm)

    @property
    def mode_count(self) -> int:
        return int(self.natural_frequency_hz.size)

    @property
    def omega_n(self) -> np.ndarray:
        return 2.0 * np.pi * self.natural_frequency_hz

    def force(self, state: StateView) -> np.ndarray:
        if self.generalized_force is None:
            return np.zeros(self.mode_count)
        q = self.generalized_force(state) if callable(self.generalized_force) else self.generalized_force
        a = np.asarray(q, dtype=float)
        if a.shape != (self.mode_count,) or not np.all(np.isfinite(a)):
            raise ValueError("generalized force provider returned invalid vector")
        return a

    def derivatives(self, state: StateView) -> dict[str, np.ndarray]:
        q = np.asarray(state.get(self.displacement_key), dtype=float)
        qd = np.asarray(state.get(self.velocity_key), dtype=float)
        if q.shape != (self.mode_count,) or qd.shape != (self.mode_count,):
            raise ValueError("flexible state shape does not match mode count")
        wn = self.omega_n
        qdd = self.force(state) / self.modal_mass - 2.0*self.damping_ratio*wn*qd - wn*wn*q
        return {self.displacement_key: qd, self.velocity_key: qdd}

    def modal_energy(self, state: StateView) -> float:
        q = np.asarray(state.get(self.displacement_key), dtype=float)
        qd = np.asarray(state.get(self.velocity_key), dtype=float)
        wn = self.omega_n
        return float(0.5*np.sum(self.modal_mass*(qd*qd + (wn*q)**2)))


@dataclass(frozen=True, slots=True)
class TorqueToModalForce:
    """Map a body torque into modal generalized force by participation matrix."""
    torque_provider: TorqueProvider | object
    participation: np.ndarray  # shape (n_modes, 3), units 1/m for q in metres

    def __post_init__(self) -> None:
        p = np.asarray(self.participation, dtype=float)
        if p.ndim != 2 or p.shape[1] != 3 or not np.all(np.isfinite(p)):
            raise ValueError("participation must be a finite (n_modes,3) matrix")
        object.__setattr__(self, "participation", p.copy())

    def __call__(self, state: StateView) -> np.ndarray:
        if callable(self.torque_provider):
            t = self.torque_provider(state)
        elif hasattr(self.torque_provider, "torque_b"):
            t = getattr(self.torque_provider, "torque_b")
        else:
            t = self.torque_provider
        t = np.asarray(t, dtype=float)
        if t.shape != (3,) or not np.all(np.isfinite(t)):
            raise ValueError("torque provider must yield a finite 3-vector")
        return self.participation @ t


@dataclass(frozen=True, slots=True)
class FlexiblePointKinematics:
    """Linear modal map from generalized coordinates to local deflection/rotation."""
    translation_shape: np.ndarray  # (3,n), dimensionless
    rotation_shape: np.ndarray     # (3,n), rad/m
    displacement_key: str = "flex_displacement"
    velocity_key: str = "flex_velocity"

    def __post_init__(self) -> None:
        t = np.asarray(self.translation_shape, dtype=float)
        r = np.asarray(self.rotation_shape, dtype=float)
        if t.ndim != 2 or r.ndim != 2 or t.shape[0] != 3 or r.shape[0] != 3 or t.shape[1] != r.shape[1]:
            raise ValueError("shape matrices must both be (3,n_modes)")
        if not np.all(np.isfinite(t)) or not np.all(np.isfinite(r)):
            raise ValueError("shape matrices must be finite")
        object.__setattr__(self, "translation_shape", t.copy())
        object.__setattr__(self, "rotation_shape", r.copy())

    @property
    def mode_count(self) -> int:
        return self.translation_shape.shape[1]

    def evaluate(self, state: StateView) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        q = np.asarray(state.get(self.displacement_key), dtype=float)
        qd = np.asarray(state.get(self.velocity_key), dtype=float)
        if q.shape != (self.mode_count,) or qd.shape != (self.mode_count,):
            raise ValueError("flexible state shape does not match point kinematics")
        return (
            self.translation_shape @ q,
            self.translation_shape @ qd,
            self.rotation_shape @ q,
            self.rotation_shape @ qd,
        )


def _small_rotation_quaternion(theta_b: np.ndarray) -> np.ndarray:
    theta = np.asarray(theta_b, dtype=float)
    angle = float(np.linalg.norm(theta))
    if angle < 1e-14:
        return quat_normalize(np.array([1.0, 0.5*theta[0], 0.5*theta[1], 0.5*theta[2]]))
    axis = theta / angle
    return np.concatenate(([np.cos(0.5*angle)], np.sin(0.5*angle)*axis))


@dataclass(frozen=True, slots=True)
class FlexibleAttitudeRateSensor:
    """Wrap a rigid attitude/rate sensor with local flexible rotation and rate.

    This is a compact control-structure-interaction hook: the controller sees
    the attitude and angular rate at a sensor mounting station, rather than the
    perfectly rigid body reference frame.
    """
    base_sensor: AttitudeRateSensor
    point_kinematics: FlexiblePointKinematics

    def measure(self, state: StateView, rng: np.random.Generator) -> AttitudeRateMeasurement:
        base = self.base_sensor.measure(state, rng)
        _, _, rot_b, rot_rate_b = self.point_kinematics.evaluate(state)
        dq = _small_rotation_quaternion(rot_b)
        # q_BI maps body to inertial; local sensor frame is rotated from B by dq.
        q_local = quat_normalize(quat_multiply(base.attitude, dq))
        return AttitudeRateMeasurement(base.time, q_local, np.asarray(base.angular_rate) + rot_rate_b)
