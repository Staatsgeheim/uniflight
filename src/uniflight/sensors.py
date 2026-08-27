from __future__ import annotations
from dataclasses import dataclass
import numpy as np

from .frames import quat_multiply, quat_normalize
from .state import StateView
from .terrain import TerrainModel


@dataclass(frozen=True, slots=True)
class SensorMeasurement:
    name: str
    time: float
    value: np.ndarray
    covariance: np.ndarray

    def __post_init__(self) -> None:
        z = np.asarray(self.value, dtype=float)
        R = np.asarray(self.covariance, dtype=float)
        if z.ndim != 1 or R.shape != (z.size, z.size):
            raise ValueError("measurement covariance must match value dimension")
        if not np.all(np.isfinite(z)) or not np.all(np.isfinite(R)):
            raise ValueError("measurement contains non-finite values")
        if np.any(np.linalg.eigvalsh((R + R.T) * 0.5) < -1e-14):
            raise ValueError("measurement covariance must be positive semidefinite")
        object.__setattr__(self, "value", z.copy())
        object.__setattr__(self, "covariance", R.copy())


@dataclass(frozen=True, slots=True)
class PositionVelocitySensor:
    """Generic inertial position/velocity navigation measurement source."""
    position_std: float | np.ndarray
    velocity_std: float | np.ndarray
    position_bias_i: np.ndarray | None = None
    velocity_bias_i: np.ndarray | None = None
    name: str = "position-velocity"

    def __post_init__(self) -> None:
        ps = np.asarray(self.position_std, dtype=float)
        vs = np.asarray(self.velocity_std, dtype=float)
        if ps.ndim == 0: ps = np.full(3, float(ps))
        if vs.ndim == 0: vs = np.full(3, float(vs))
        if ps.shape != (3,) or vs.shape != (3,) or np.any(ps < 0) or np.any(vs < 0):
            raise ValueError("position_std and velocity_std must be non-negative scalars/3-vectors")
        pb = np.zeros(3) if self.position_bias_i is None else np.asarray(self.position_bias_i, dtype=float)
        vb = np.zeros(3) if self.velocity_bias_i is None else np.asarray(self.velocity_bias_i, dtype=float)
        if pb.shape != (3,) or vb.shape != (3,) or not np.all(np.isfinite(pb)) or not np.all(np.isfinite(vb)):
            raise ValueError("sensor biases must be finite 3-vectors")
        object.__setattr__(self, "position_std", ps.copy())
        object.__setattr__(self, "velocity_std", vs.copy())
        object.__setattr__(self, "position_bias_i", pb.copy())
        object.__setattr__(self, "velocity_bias_i", vb.copy())

    @property
    def covariance(self) -> np.ndarray:
        std = np.concatenate((self.position_std, self.velocity_std))
        return np.diag(std * std)

    def measure(self, state: StateView, rng: np.random.Generator) -> SensorMeasurement:
        truth = np.concatenate((np.asarray(state.get("position")), np.asarray(state.get("velocity"))))
        bias = np.concatenate((self.position_bias_i, self.velocity_bias_i))
        std = np.concatenate((self.position_std, self.velocity_std))
        value = truth + bias + rng.normal(0.0, std)
        return SensorMeasurement(self.name, state.time, value, self.covariance)


@dataclass(frozen=True, slots=True)
class RadarAltimeterSensor:
    terrain: TerrainModel
    altitude_std: float
    bias: float = 0.0
    name: str = "radar-altimeter"

    def __post_init__(self) -> None:
        if not np.isfinite(self.altitude_std) or self.altitude_std < 0 or not np.isfinite(self.bias):
            raise ValueError("altimeter parameters must be finite; std non-negative")

    def measure(self, state: StateView, rng: np.random.Generator) -> SensorMeasurement:
        h = self.terrain.query(state.get("position"), state.time).agl
        z = h + self.bias + float(rng.normal(0.0, self.altitude_std))
        return SensorMeasurement(self.name, state.time, np.array([z]), np.array([[self.altitude_std**2]]))


@dataclass(frozen=True, slots=True)
class AttitudeRateMeasurement:
    time: float
    attitude: np.ndarray
    angular_rate: np.ndarray


@dataclass(frozen=True, slots=True)
class AttitudeRateSensor:
    attitude_std_rad: float = 0.0
    rate_std: float | np.ndarray = 0.0

    def __post_init__(self) -> None:
        rs = np.asarray(self.rate_std, dtype=float)
        if rs.ndim == 0: rs = np.full(3, float(rs))
        if not np.isfinite(self.attitude_std_rad) or self.attitude_std_rad < 0:
            raise ValueError("attitude_std_rad must be finite and non-negative")
        if rs.shape != (3,) or np.any(rs < 0) or not np.all(np.isfinite(rs)):
            raise ValueError("rate_std must be a non-negative scalar/3-vector")
        object.__setattr__(self, "rate_std", rs.copy())

    @staticmethod
    def _rotvec_quaternion(rotvec: np.ndarray) -> np.ndarray:
        angle = float(np.linalg.norm(rotvec))
        if angle < 1e-15:
            return np.array([1.0, 0.5*rotvec[0], 0.5*rotvec[1], 0.5*rotvec[2]])
        axis = rotvec / angle
        return np.concatenate(([np.cos(angle/2)], axis*np.sin(angle/2)))

    def measure(self, state: StateView, rng: np.random.Generator) -> AttitudeRateMeasurement:
        q = np.asarray(state.get("attitude"), dtype=float)
        rotvec = rng.normal(0.0, self.attitude_std_rad, 3)
        dq = self._rotvec_quaternion(rotvec)
        # Small measurement rotation expressed in inertial-side convention.
        qm = quat_normalize(quat_multiply(dq, q))
        wm = np.asarray(state.get("angular_rate"), dtype=float) + rng.normal(0.0, self.rate_std)
        return AttitudeRateMeasurement(state.time, qm, wm)
