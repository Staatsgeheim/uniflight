from __future__ import annotations
from dataclasses import dataclass, field
from typing import Callable
import numpy as np

from .sensors import SensorMeasurement

VectorFunction = Callable[[np.ndarray], np.ndarray]


def numerical_jacobian(function: VectorFunction, x: np.ndarray, rel_step: float | None = None) -> np.ndarray:
    x = np.asarray(x, dtype=float)
    f0 = np.asarray(function(x), dtype=float)
    if f0.ndim != 1:
        raise ValueError("function output must be a vector")
    eps = np.sqrt(np.finfo(float).eps) if rel_step is None else float(rel_step)
    J = np.empty((f0.size, x.size), dtype=float)
    for j in range(x.size):
        h = eps * max(1.0, abs(x[j]))
        xp = x.copy(); xm = x.copy()
        xp[j] += h; xm[j] -= h
        J[:, j] = (np.asarray(function(xp), float) - np.asarray(function(xm), float)) / (2*h)
    return J


@dataclass(frozen=True, slots=True)
class EKFUpdate:
    innovation: np.ndarray
    innovation_covariance: np.ndarray
    kalman_gain: np.ndarray
    nis: float


class ExtendedKalmanFilter:
    """Small generic discrete-time EKF with Joseph covariance update."""
    def __init__(self, x0: np.ndarray, covariance0: np.ndarray):
        x = np.asarray(x0, dtype=float)
        P = np.asarray(covariance0, dtype=float)
        if x.ndim != 1 or P.shape != (x.size, x.size):
            raise ValueError("EKF state/covariance dimensions disagree")
        if not np.all(np.isfinite(x)) or not np.all(np.isfinite(P)):
            raise ValueError("EKF initial values must be finite")
        self.x = x.copy()
        self.P = 0.5*(P+P.T)

    def predict(self, process: Callable[[np.ndarray], np.ndarray], process_covariance: np.ndarray,
                jacobian: np.ndarray | None = None) -> None:
        Q = np.asarray(process_covariance, dtype=float)
        if Q.shape != self.P.shape:
            raise ValueError("process covariance shape mismatch")
        F = numerical_jacobian(process, self.x) if jacobian is None else np.asarray(jacobian, dtype=float)
        if F.shape != self.P.shape:
            raise ValueError("process Jacobian shape mismatch")
        self.x = np.asarray(process(self.x), dtype=float)
        self.P = F @ self.P @ F.T + Q
        self.P = 0.5*(self.P+self.P.T)

    def update(self, measurement: SensorMeasurement, measurement_function: VectorFunction,
               jacobian: np.ndarray | None = None) -> EKFUpdate:
        z = np.asarray(measurement.value, dtype=float)
        R = np.asarray(measurement.covariance, dtype=float)
        h = np.asarray(measurement_function(self.x), dtype=float)
        H = numerical_jacobian(measurement_function, self.x) if jacobian is None else np.asarray(jacobian, dtype=float)
        if h.shape != z.shape or H.shape != (z.size, self.x.size):
            raise ValueError("measurement model dimension mismatch")
        innovation = z - h
        S = H @ self.P @ H.T + R
        PHt = self.P @ H.T
        K = np.linalg.solve(S.T, PHt.T).T
        self.x = self.x + K @ innovation
        I = np.eye(self.x.size)
        A = I - K@H
        self.P = A@self.P@A.T + K@R@K.T
        self.P = 0.5*(self.P+self.P.T)
        nis = float(innovation @ np.linalg.solve(S, innovation))
        return EKFUpdate(innovation, S, K, nis)


@dataclass(frozen=True, slots=True)
class KinematicProcessModel:
    """6-state [r_I,v_I] process model with arbitrary gravity field."""
    gravity: object
    acceleration_i: np.ndarray | Callable[[], np.ndarray] = field(default_factory=lambda: np.zeros(3))

    def __post_init__(self) -> None:
        if not callable(self.acceleration_i):
            a = np.asarray(self.acceleration_i, dtype=float)
            if a.shape != (3,) or not np.all(np.isfinite(a)):
                raise ValueError("acceleration_i must be a finite 3-vector or callable")
            object.__setattr__(self, "acceleration_i", a.copy())

    def propagate(self, x: np.ndarray, dt: float) -> np.ndarray:
        x = np.asarray(x, dtype=float)
        if x.shape != (6,) or dt < 0:
            raise ValueError("kinematic state must be length 6 and dt non-negative")
        r = x[:3]; v=x[3:]
        a_ctrl = np.asarray(self.acceleration_i() if callable(self.acceleration_i) else self.acceleration_i, dtype=float)
        a = np.asarray(self.gravity.acceleration(r, 0.0), dtype=float) + a_ctrl
        return np.concatenate((r + v*dt + 0.5*a*dt*dt, v + a*dt))


class TranslationalNavigationEKF:
    """Convenience EKF for inertial position/velocity navigation."""
    def __init__(self, x0: np.ndarray, covariance0: np.ndarray, gravity: object,
                 accel_process_std: float = 0.5):
        self.filter = ExtendedKalmanFilter(x0, covariance0)
        self.gravity = gravity
        self.accel_process_std = float(accel_process_std)
        if not np.isfinite(self.accel_process_std) or self.accel_process_std < 0:
            raise ValueError("accel_process_std must be finite and non-negative")

    @property
    def x(self) -> np.ndarray:
        return self.filter.x

    @property
    def covariance(self) -> np.ndarray:
        return self.filter.P

    def predict(self, dt: float, control_acceleration_i: np.ndarray) -> None:
        dt = float(dt)
        model = KinematicProcessModel(self.gravity, np.asarray(control_acceleration_i, dtype=float))
        fn = lambda x: model.propagate(x, dt)
        s2 = self.accel_process_std**2
        I3 = np.eye(3)
        Q = s2 * np.block([
            [0.25*dt**4*I3, 0.5*dt**3*I3],
            [0.5*dt**3*I3, dt**2*I3],
        ])
        self.filter.predict(fn, Q)

    def update_position_velocity(self, measurement: SensorMeasurement) -> EKFUpdate:
        H = np.eye(6)
        return self.filter.update(measurement, lambda x: x, H)
