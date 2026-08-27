from __future__ import annotations
from dataclasses import dataclass
import numpy as np

from .state import StateSchema


@dataclass(frozen=True, slots=True)
class SeparatedBodyState:
    mass: float
    position_i: np.ndarray
    velocity_i: np.ndarray


@dataclass(frozen=True, slots=True)
class TwoBodySeparationResult:
    retained: SeparatedBodyState
    detached: SeparatedBodyState
    relative_velocity_i: np.ndarray
    momentum_error_i: np.ndarray


def separate_two_body(
    parent_mass: float,
    parent_position_i: np.ndarray,
    parent_velocity_i: np.ndarray,
    retained_mass: float,
    detached_mass: float,
    relative_separation_velocity_i: np.ndarray | None = None,
) -> TwoBodySeparationResult:
    """Split a parent into two co-located daughter bodies conserving momentum.

    ``relative_separation_velocity_i = v_detached - v_retained``. The caller may
    subsequently integrate each daughter using its own vehicle/environment model.
    """
    M = float(parent_mass)
    m1 = float(retained_mass)
    m2 = float(detached_mass)
    if not all(np.isfinite(v) and v > 0 for v in (M,m1,m2)):
        raise ValueError("masses must be finite and positive")
    if abs((m1 + m2) - M) > max(1e-10, 1e-12*M):
        raise ValueError("daughter masses must sum to parent mass")
    r = np.asarray(parent_position_i, dtype=float)
    V = np.asarray(parent_velocity_i, dtype=float)
    dv = np.zeros(3) if relative_separation_velocity_i is None else np.asarray(relative_separation_velocity_i, dtype=float)
    if any(a.shape != (3,) or not np.all(np.isfinite(a)) for a in (r,V,dv)):
        raise ValueError("positions and velocities must be finite 3-vectors")
    v1 = V - (m2/M) * dv
    v2 = V + (m1/M) * dv
    p_before = M * V
    p_after = m1*v1 + m2*v2
    return TwoBodySeparationResult(
        SeparatedBodyState(m1, r.copy(), v1),
        SeparatedBodyState(m2, r.copy(), v2),
        dv.copy(), p_after-p_before,
    )


@dataclass(frozen=True, slots=True)
class JettisonJump:
    """Hybrid jump map removing a fixed mass and optionally resetting fields."""

    schema: StateSchema
    jettison_mass: float
    reset_fields: dict[str, float | np.ndarray] | None = None
    minimum_remaining_mass: float = 0.0

    def __post_init__(self) -> None:
        if not np.isfinite(self.jettison_mass) or self.jettison_mass < 0:
            raise ValueError("jettison_mass must be finite and non-negative")
        if not np.isfinite(self.minimum_remaining_mass) or self.minimum_remaining_mass < 0:
            raise ValueError("minimum_remaining_mass must be finite and non-negative")

    def __call__(self, time: float, packed: np.ndarray) -> np.ndarray:
        values = self.schema.unpack(np.asarray(packed, dtype=float))
        remaining = float(values["mass"]) - self.jettison_mass
        if remaining <= self.minimum_remaining_mass:
            raise ValueError("jettison would violate minimum remaining mass")
        values["mass"] = remaining
        for key, value in (self.reset_fields or {}).items():
            if key not in {f.key for f in self.schema.fields}:
                raise KeyError(f"unknown state field {key!r}")
            values[key] = value
        return self.schema.pack(values)

# ---------------------------------------------------------------------------
# Milestone I: rigid two-body 6-DOF separation
# ---------------------------------------------------------------------------
from .frames import body_to_inertial_matrix, quat_normalize


@dataclass(frozen=True, slots=True)
class RigidSeparatedBodyState:
    mass: float
    position_i: np.ndarray
    velocity_i: np.ndarray
    attitude_bi: np.ndarray
    angular_rate_b: np.ndarray
    inertia_b: np.ndarray

    def __post_init__(self) -> None:
        if not np.isfinite(self.mass) or self.mass <= 0:
            raise ValueError("mass must be finite and positive")
        for name in ("position_i", "velocity_i", "angular_rate_b"):
            a = np.asarray(getattr(self, name), dtype=float)
            if a.shape != (3,) or not np.all(np.isfinite(a)):
                raise ValueError(f"{name} must be a finite 3-vector")
            object.__setattr__(self, name, a.copy())
        object.__setattr__(self, "attitude_bi", quat_normalize(self.attitude_bi))
        I = np.asarray(self.inertia_b, dtype=float)
        if I.shape != (3,3) or not np.all(np.isfinite(I)) or not np.allclose(I, I.T, atol=1e-12):
            raise ValueError("inertia_b must be a finite symmetric 3x3 tensor")
        if np.any(np.linalg.eigvalsh(I) <= 0):
            raise ValueError("inertia_b must be positive definite")
        object.__setattr__(self, "inertia_b", I.copy())


@dataclass(frozen=True, slots=True)
class RigidTwoBodySeparationResult:
    retained: RigidSeparatedBodyState
    detached: RigidSeparatedBodyState
    linear_momentum_error_i: np.ndarray
    angular_momentum_error_i: np.ndarray
    parent_com_error_b: np.ndarray
    composite_inertia_error_b: np.ndarray


def _parallel_axis(mass: float, offset_b: np.ndarray) -> np.ndarray:
    r = np.asarray(offset_b, dtype=float)
    return float(mass) * ((r @ r)*np.eye(3) - np.outer(r,r))


def separate_two_rigid_bodies(
    *,
    parent_mass: float,
    parent_position_i: np.ndarray,
    parent_velocity_i: np.ndarray,
    parent_attitude_bi: np.ndarray,
    parent_angular_rate_b: np.ndarray,
    parent_inertia_b: np.ndarray,
    retained_mass: float,
    detached_mass: float,
    retained_offset_b: np.ndarray,
    detached_offset_b: np.ndarray,
    retained_inertia_b: np.ndarray,
    detached_inertia_b: np.ndarray,
    relative_separation_velocity_i: np.ndarray | None = None,
    conserve_angular_momentum: bool = True,
) -> RigidTwoBodySeparationResult:
    """Split a rigid parent into two daughter rigid bodies.

    Daughter COM offsets are expressed in the parent body frame and must obey
    the parent COM condition ``m1*r1 + m2*r2 = 0``.  Both daughters initially
    inherit the parent attitude.  Their translational velocities include the
    local rigid-body velocity ``omega x r`` plus an optional prescribed
    relative separation velocity ``v2-v1``.  A shared spin-rate correction is
    then applied, when requested, so total angular momentum about the parent
    COM is conserved even if the separation impulse carries angular momentum.
    """
    M, m1, m2 = map(float, (parent_mass, retained_mass, detached_mass))
    if not all(np.isfinite(v) and v > 0 for v in (M,m1,m2)):
        raise ValueError("masses must be finite and positive")
    if abs(m1+m2-M) > max(1e-10, 1e-12*M):
        raise ValueError("daughter masses must sum to parent mass")
    r0 = np.asarray(parent_position_i, float)
    V0 = np.asarray(parent_velocity_i, float)
    q = quat_normalize(parent_attitude_bi)
    omega = np.asarray(parent_angular_rate_b, float)
    I0 = np.asarray(parent_inertia_b, float)
    r1b = np.asarray(retained_offset_b, float)
    r2b = np.asarray(detached_offset_b, float)
    I1 = np.asarray(retained_inertia_b, float)
    I2 = np.asarray(detached_inertia_b, float)
    dv = np.zeros(3) if relative_separation_velocity_i is None else np.asarray(relative_separation_velocity_i, float)
    for name,a,shape in (
        ("parent_position_i",r0,(3,)),("parent_velocity_i",V0,(3,)),
        ("parent_angular_rate_b",omega,(3,)),("retained_offset_b",r1b,(3,)),
        ("detached_offset_b",r2b,(3,)),("relative_separation_velocity_i",dv,(3,)),
        ("parent_inertia_b",I0,(3,3)),("retained_inertia_b",I1,(3,3)),
        ("detached_inertia_b",I2,(3,3)),
    ):
        if a.shape != shape or not np.all(np.isfinite(a)):
            raise ValueError(f"{name} has invalid shape or non-finite values")
    for I,name in ((I0,"parent_inertia_b"),(I1,"retained_inertia_b"),(I2,"detached_inertia_b")):
        if not np.allclose(I,I.T,atol=1e-12,rtol=0) or np.any(np.linalg.eigvalsh(I)<=0):
            raise ValueError(f"{name} must be symmetric positive definite")

    com_error_b = m1*r1b + m2*r2b
    if np.linalg.norm(com_error_b) > max(1e-9, 1e-11*M*max(1.0,np.linalg.norm(r1b),np.linalg.norm(r2b))):
        raise ValueError("daughter offsets do not satisfy the parent COM condition")

    R = body_to_inertial_matrix(q)
    r1i, r2i = R@r1b, R@r2b
    p1, p2 = r0+r1i, r0+r2i
    rigid_v1 = V0 + R@np.cross(omega,r1b)
    rigid_v2 = V0 + R@np.cross(omega,r2b)
    v1 = rigid_v1 - (m2/M)*dv
    v2 = rigid_v2 + (m1/M)*dv

    H_before = R@(I0@omega)
    H_trans = np.cross(r1i, m1*(v1-V0)) + np.cross(r2i, m2*(v2-V0))
    H_spin_base = R@((I1+I2)@omega)
    residual_i = H_before - (H_trans + H_spin_base)
    domega = np.zeros(3)
    if conserve_angular_momentum:
        domega = np.linalg.solve(I1+I2, R.T@residual_i)
    w1 = omega+domega
    w2 = omega+domega

    retained = RigidSeparatedBodyState(m1,p1,v1,q,w1,I1)
    detached = RigidSeparatedBodyState(m2,p2,v2,q,w2,I2)
    P_before = M*V0
    P_after = m1*v1+m2*v2
    H_after = (
        np.cross(r1i,m1*(v1-V0)) + np.cross(r2i,m2*(v2-V0))
        + R@(I1@w1 + I2@w2)
    )
    composite_I = I1+I2+_parallel_axis(m1,r1b)+_parallel_axis(m2,r2b)
    return RigidTwoBodySeparationResult(
        retained, detached, P_after-P_before, H_after-H_before,
        com_error_b.copy(), composite_I-I0,
    )
