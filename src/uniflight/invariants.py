from __future__ import annotations
import numpy as np


def specific_energy(mu: float, position: np.ndarray, velocity: np.ndarray) -> float:
    return 0.5 * float(np.dot(velocity, velocity)) - mu / np.linalg.norm(position)


def specific_angular_momentum(position: np.ndarray, velocity: np.ndarray) -> np.ndarray:
    return np.cross(position, velocity)


def quaternion_norm_error(q: np.ndarray) -> float:
    return abs(np.linalg.norm(q) - 1.0)
