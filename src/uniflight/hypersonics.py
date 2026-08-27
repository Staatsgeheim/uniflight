from __future__ import annotations
from dataclasses import dataclass
import math
import numpy as np

from .aerodynamics import AeroCoefficients, AeroCoefficientModel6DOF
from .flow import BodyFlowState


@dataclass(frozen=True, slots=True)
class NewtonianHypersonicCoefficients:
    """Low-order Newtonian-inspired high-Mach coefficient closure.

    This is a reference model for software integration and verification, not a
    substitute for a validated CFD/experimental database. Normal force scales
    with sin(theta)|sin(theta)|, producing the expected quadratic impact-law
    behavior while preserving force sign. ``cd0`` represents zero-incidence
    blunt/base drag.
    """

    cd0: float = 1.0
    normal_force_scale: float = 2.0
    pitch_stability: float = 0.0
    yaw_stability: float = 0.0
    roll_beta: float = 0.0

    def __post_init__(self) -> None:
        vals = np.asarray([
            self.cd0, self.normal_force_scale, self.pitch_stability,
            self.yaw_stability, self.roll_beta,
        ], dtype=float)
        if not np.all(np.isfinite(vals)) or self.cd0 < 0 or self.normal_force_scale < 0:
            raise ValueError("hypersonic coefficients must be finite; drag/normal scale non-negative")

    def __call__(self, flow: BodyFlowState) -> AeroCoefficients:
        a = float(np.clip(flow.alpha, -0.5*np.pi, 0.5*np.pi))
        b = float(np.clip(flow.beta, -0.5*np.pi, 0.5*np.pi))
        na = self.normal_force_scale * math.sin(a) * abs(math.sin(a))
        nb = self.normal_force_scale * math.sin(b) * abs(math.sin(b))
        induced_drag = abs(na * math.sin(a)) + abs(nb * math.sin(b))
        return AeroCoefficients(
            cd=self.cd0 + induced_drag,
            cl=na * math.cos(a),
            cy=-nb * math.cos(b),
            c_roll=self.roll_beta * b,
            c_pitch=-self.pitch_stability * a,
            c_yaw=-self.yaw_stability * b,
        )


@dataclass(frozen=True, slots=True)
class MachBlendedAeroCoefficients:
    """Smoothly blend any low- and high-Mach 6-DOF coefficient closures."""

    low_mach_model: AeroCoefficientModel6DOF
    high_mach_model: AeroCoefficientModel6DOF
    mach_start: float
    mach_end: float

    def __post_init__(self) -> None:
        if not np.isfinite(self.mach_start) or self.mach_start < 0:
            raise ValueError("mach_start must be finite and non-negative")
        if not np.isfinite(self.mach_end) or self.mach_end <= self.mach_start:
            raise ValueError("mach_end must exceed mach_start")

    def blend_fraction(self, mach: float) -> float:
        x = float(np.clip((mach-self.mach_start)/(self.mach_end-self.mach_start), 0.0, 1.0))
        return x*x*(3.0-2.0*x)

    def __call__(self, flow: BodyFlowState) -> AeroCoefficients:
        lo = self.low_mach_model(flow)
        hi = self.high_mach_model(flow)
        w = self.blend_fraction(flow.mach)
        vals = {}
        for name in ("cd","cl","cy","c_roll","c_pitch","c_yaw"):
            vals[name] = (1.0-w)*getattr(lo,name) + w*getattr(hi,name)
        return AeroCoefficients(**vals)
