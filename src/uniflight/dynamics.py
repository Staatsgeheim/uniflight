from __future__ import annotations
from dataclasses import dataclass
from typing import Protocol
import numpy as np

from .state import StateSchema, StateView

class DerivativeModel(Protocol):
    def derivatives(self, state: StateView) -> dict[str, np.ndarray | float]: ...

class DynamicsAssembler:
    """Adds derivative contributions while enforcing one owner per state derivative."""
    def __init__(self, schema: StateSchema, models: list[DerivativeModel]):
        self.schema = schema
        self.models = tuple(models)

    def rhs(self, time: float, packed: np.ndarray) -> np.ndarray:
        state = StateView(time, packed, self.schema)
        values: dict[str, np.ndarray | float] = {}
        for model in self.models:
            for key, derivative in model.derivatives(state).items():
                if key in values:
                    raise RuntimeError(f"Duplicate derivative writer for state field {key!r}")
                values[key] = derivative
        ydot = np.zeros(self.schema.total_size, dtype=float)
        for f in self.schema.fields:
            if f.continuity != "CONTINUOUS":
                continue
            if f.key not in values:
                # Explicit zero derivative is allowed for static states.
                continue
            a = np.asarray(values[f.key], dtype=float)
            if f.shape:
                if a.shape != f.shape:
                    raise ValueError(f"Derivative {f.key} has shape {a.shape}, expected {f.shape}")
                ydot[self.schema.sl(f.key)] = a.reshape(-1)
            else:
                if a.size != 1:
                    raise ValueError(f"Derivative {f.key} must be scalar")
                ydot[self.schema.sl(f.key)] = float(a.reshape(-1)[0])
        if not np.all(np.isfinite(ydot)):
            raise FloatingPointError("Non-finite RHS")
        return ydot

@dataclass(frozen=True, slots=True)
class TranslationalKinematics:
    gravity: object | None = None
    acceleration_models: tuple[object, ...] = ()

    def derivatives(self, state: StateView) -> dict[str, np.ndarray]:
        v = np.asarray(state.get("velocity"))
        a = np.zeros(3)
        if self.gravity is not None:
            a += self.gravity.acceleration(state.get("position"), state.time)
        for model in self.acceleration_models:
            a += np.asarray(model.acceleration(state), dtype=float)
        return {"position": v, "velocity": a}

@dataclass(frozen=True, slots=True)
class QuaternionKinematics:
    """Scalar-first quaternion q_BI with body angular rate expressed in B."""
    def derivatives(self, state: StateView) -> dict[str, np.ndarray]:
        q = np.asarray(state.get("attitude"), dtype=float)
        wx, wy, wz = np.asarray(state.get("angular_rate"), dtype=float)
        Omega = np.array([
            [0.0, -wx, -wy, -wz],
            [wx, 0.0, wz, -wy],
            [wy, -wz, 0.0, wx],
            [wz, wy, -wx, 0.0],
        ])
        return {"attitude": 0.5 * Omega @ q}

@dataclass(frozen=True, slots=True)
class IdealRocket:
    """Ideal constant-exhaust-speed rocket acceleration model for kernel verification.

    mdot_exhaust is positive. thrust direction is inertial and normalized on construction.
    """
    exhaust_velocity: float
    mdot_exhaust: float
    direction_i: np.ndarray

    def __post_init__(self):
        d = np.asarray(self.direction_i, dtype=float)
        n = np.linalg.norm(d)
        if self.exhaust_velocity <= 0 or self.mdot_exhaust <= 0 or n == 0:
            raise ValueError("exhaust_velocity, mdot_exhaust and direction must be valid")
        object.__setattr__(self, "direction_i", d/n)

    def acceleration(self, state: StateView) -> np.ndarray:
        m = state.get("mass")
        if m <= 0:
            raise ValueError("Rocket mass must remain positive")
        return self.mdot_exhaust * self.exhaust_velocity / m * self.direction_i

    def derivatives(self, state: StateView) -> dict[str, float]:
        return {"mass": -self.mdot_exhaust}

# ---------------------------------------------------------------------------
# Milestone C: coupled 6-DOF rigid-body dynamics
# ---------------------------------------------------------------------------
from .mass_properties import MassPropertiesModel
from .wrenches import WrenchModel


@dataclass(frozen=True, slots=True)
class RigidBody6DOFDynamics:
    """Own position, velocity, and body angular-rate derivatives.

    Gravity is supplied as an acceleration field. Every other interaction is
    supplied as a wrench model returning inertial force and body-frame moment
    about the instantaneous center of mass. Attitude and mass remain separate
    state owners (`QuaternionKinematics` and propulsion/tank models).
    """

    mass_properties: MassPropertiesModel
    gravity: object | None = None
    wrench_models: tuple[WrenchModel, ...] = ()
    external_acceleration_models: tuple[object, ...] = ()

    def derivatives(self, state: StateView) -> dict[str, np.ndarray]:
        mp = self.mass_properties.evaluate(state)
        state_mass = float(state.get("mass"))
        if abs(mp.mass-state_mass) > 1e-10*max(1.0, abs(state_mass)):
            raise ValueError("mass-properties model disagrees with canonical mass state")

        force_i = np.zeros(3)
        moment_b = np.zeros(3)
        for model in self.wrench_models:
            w = model.wrench(state)
            force_i += w.force_i
            moment_b += w.moment_b

        accel_i = force_i / mp.mass
        if self.gravity is not None:
            accel_i += np.asarray(self.gravity.acceleration(state.get("position"), state.time), dtype=float)
        for model in self.external_acceleration_models:
            accel_i += np.asarray(model.acceleration(state), dtype=float)

        omega = np.asarray(state.get("angular_rate"), dtype=float)
        gyro = np.cross(omega, mp.inertia_b @ omega)
        rhs_moment = moment_b - mp.inertia_rate_b @ omega - gyro
        omega_dot = np.linalg.solve(mp.inertia_b, rhs_moment)

        return {
            "position": np.asarray(state.get("velocity"), dtype=float),
            "velocity": accel_i,
            "angular_rate": omega_dot,
        }
