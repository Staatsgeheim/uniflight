from __future__ import annotations
from dataclasses import dataclass
from collections import deque
from typing import Callable
import numpy as np

Vec = np.ndarray

def quat_normalize(q: Vec) -> Vec:
    q = np.asarray(q, dtype=float)
    n = np.linalg.norm(q)
    if n == 0 or not np.isfinite(n):
        raise ValueError("Invalid zero/non-finite quaternion")
    return q / n

def quat_conjugate(q: Vec) -> Vec:
    q = np.asarray(q, dtype=float)
    return np.array([q[0], -q[1], -q[2], -q[3]])

def quat_multiply(a: Vec, b: Vec) -> Vec:
    aw, ax, ay, az = np.asarray(a, float)
    bw, bx, by, bz = np.asarray(b, float)
    return np.array([
        aw*bw - ax*bx - ay*by - az*bz,
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
    ])

def quat_to_matrix(q: Vec) -> np.ndarray:
    w, x, y, z = quat_normalize(q)
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])

def matrix_to_quat(R: np.ndarray) -> Vec:
    R = np.asarray(R, dtype=float)
    tr = np.trace(R)
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        q = np.array([0.25*s, (R[2,1]-R[1,2])/s, (R[0,2]-R[2,0])/s, (R[1,0]-R[0,1])/s])
    else:
        i = int(np.argmax(np.diag(R)))
        if i == 0:
            s = np.sqrt(1 + R[0,0]-R[1,1]-R[2,2]) * 2
            q = np.array([(R[2,1]-R[1,2])/s, .25*s, (R[0,1]+R[1,0])/s, (R[0,2]+R[2,0])/s])
        elif i == 1:
            s = np.sqrt(1 + R[1,1]-R[0,0]-R[2,2]) * 2
            q = np.array([(R[0,2]-R[2,0])/s, (R[0,1]+R[1,0])/s, .25*s, (R[1,2]+R[2,1])/s])
        else:
            s = np.sqrt(1 + R[2,2]-R[0,0]-R[1,1]) * 2
            q = np.array([(R[1,0]-R[0,1])/s, (R[0,2]+R[2,0])/s, (R[1,2]+R[2,1])/s, .25*s])
    return quat_normalize(q)

@dataclass(frozen=True, slots=True)
class Transform:
    """Rigid transform mapping coordinates from source frame to target frame."""
    rotation: np.ndarray
    translation: np.ndarray
    angular_velocity: np.ndarray

    @staticmethod
    def identity() -> "Transform":
        return Transform(np.eye(3), np.zeros(3), np.zeros(3))

    def vector(self, v: Vec) -> Vec:
        return self.rotation @ np.asarray(v, float)

    def point(self, p: Vec) -> Vec:
        return self.rotation @ np.asarray(p, float) + self.translation

    def tensor(self, T: np.ndarray) -> np.ndarray:
        return self.rotation @ np.asarray(T, float) @ self.rotation.T

    def inverse(self) -> "Transform":
        R = self.rotation.T
        return Transform(R, -R @ self.translation, -R @ self.angular_velocity)

    def then(self, next_tf: "Transform") -> "Transform":
        """Compose source->mid (self) then mid->target (next_tf)."""
        R = next_tf.rotation @ self.rotation
        t = next_tf.rotation @ self.translation + next_tf.translation
        w = next_tf.angular_velocity + next_tf.rotation @ self.angular_velocity
        return Transform(R, t, w)

TransformProvider = Callable[[float], Transform]

class FrameGraph:
    def __init__(self):
        self._edges: dict[tuple[str, str], TransformProvider] = {}

    def add_transform(self, source: str, target: str, provider: Transform | TransformProvider) -> None:
        if source == target:
            raise ValueError("Source and target frame must differ")
        fn = provider if callable(provider) else (lambda _t, p=provider: p)
        self._edges[(source, target)] = fn

    def transform(self, source: str, target: str, time: float) -> Transform:
        if source == target:
            return Transform.identity()
        adj: dict[str, list[tuple[str, Callable[[float], Transform]]]] = {}
        for (a,b), fn in self._edges.items():
            adj.setdefault(a, []).append((b, fn))
            adj.setdefault(b, []).append((a, lambda t, f=fn: f(t).inverse()))
        q = deque([(source, Transform.identity())])
        seen = {source}
        while q:
            node, tf = q.popleft()
            for nxt, fn in adj.get(node, []):
                if nxt in seen:
                    continue
                new_tf = tf.then(fn(time))
                if nxt == target:
                    return new_tf
                seen.add(nxt)
                q.append((nxt, new_tf))
        raise KeyError(f"No frame path {source!r}->{target!r}")


def body_to_inertial_matrix(attitude: Vec) -> np.ndarray:
    """Return R_IB, mapping body-frame components into inertial components.

    Milestone C makes the quaternion convention explicit: the canonical
    attitude state is a scalar-first quaternion whose rotation matrix maps
    B -> I. This is the convention consistent with ``QuaternionKinematics``
    and body angular rate expressed in B.
    """
    return quat_to_matrix(attitude)


def inertial_to_body_matrix(attitude: Vec) -> np.ndarray:
    """Return R_BI, mapping inertial-frame components into body components."""
    return body_to_inertial_matrix(attitude).T


def rotate_body_to_inertial(attitude: Vec, vector_b: Vec) -> Vec:
    return body_to_inertial_matrix(attitude) @ np.asarray(vector_b, dtype=float)


def rotate_inertial_to_body(attitude: Vec, vector_i: Vec) -> Vec:
    return inertial_to_body_matrix(attitude) @ np.asarray(vector_i, dtype=float)
