"""Milestone H deterministic trajectory-design reference cases."""
from __future__ import annotations
from dataclasses import dataclass
from typing import Mapping
import numpy as np

from .bodies import SphericalBody
from .dynamics import DynamicsAssembler, TranslationalKinematics, IdealRocket
from .events import Event, EventAction
from .integrators import ScipyIVPIntegrator, SolverConfig, FixedStepRK4Integrator, FixedStepRK4Config
from .optimization import (
    DesignSpace, DesignVariable, MetricConstraint, MetricObjective,
    OptimizationSettings, TrajectoryOptimizer, TrajectoryProblem,
    TrajectoryTargeter, TargetingSettings,
)
from .simulation import SimulationEngine
from .state import core_3dof_schema


BODY_H = SphericalBody(mu=2.0e10, radius=2.5e5, name="Nereid-H")
MASS0_H = 500.0
VE_H = 1800.0
TARGET_APOGEE_H = 20_000.0


def _powered_burn(parameters: Mapping[str, float]):
    mdot = float(parameters["mdot"])
    burn_time = float(parameters["burn_time"])
    if mdot <= 0 or burn_time <= 0 or mdot*burn_time >= 0.75*MASS0_H:
        raise ValueError("invalid reference-ascent mass flow / burn time")
    schema = core_3dof_schema()
    y0 = schema.pack({
        "position": np.array([BODY_H.radius + 1.0, 0.0, 0.0]),
        "velocity": np.zeros(3),
        "mass": MASS0_H,
    })
    rocket = IdealRocket(VE_H, mdot, np.array([1.0, 0.0, 0.0]))
    powered_rhs = DynamicsAssembler(schema, [
        TranslationalKinematics(BODY_H.gravity, (rocket,)), rocket,
    ]).rhs
    powered = SimulationEngine(
        powered_rhs,
        FixedStepRK4Integrator(FixedStepRK4Config(step=0.05, save_every_step=False)),
    ).run((0.0, burn_time), y0)
    if not powered.success:
        raise RuntimeError(powered.message)
    return schema, powered.states[-1]


def evaluate_radial_ascent(parameters: Mapping[str, float]) -> Mapping[str, float]:
    """Fast optimization metric path.

    The powered phase is numerically propagated by UniFlight.  The subsequent
    zero-angular-momentum coast apogee follows exactly from two-body specific
    orbital energy, avoiding hundreds of seconds of repeated coast integration
    inside finite-difference optimization.
    """
    schema, y_burn = _powered_burn(parameters)
    b = schema.unpack(y_burn)
    r = float(np.linalg.norm(b["position"]))
    v = float(np.linalg.norm(b["velocity"]))
    energy = 0.5*v*v - BODY_H.mu/r
    if energy >= 0:
        raise RuntimeError("reference ascent escaped instead of reaching a bound apogee")
    r_apogee = -BODY_H.mu / energy
    return {
        "apogee_altitude": r_apogee - BODY_H.radius,
        "propellant_used": MASS0_H - float(b["mass"]),
        "final_mass": float(b["mass"]),
        "burnout_altitude": BODY_H.altitude(b["position"]),
        "burnout_speed": v,
    }


def evaluate_radial_ascent_event(parameters: Mapping[str, float]) -> Mapping[str, float]:
    """Reference path that explicitly propagates to the apogee event."""
    schema, y_burn = _powered_burn(parameters)
    b = schema.unpack(y_burn)
    burn_time = float(parameters["burn_time"])
    coast_rhs = DynamicsAssembler(schema, [TranslationalKinematics(BODY_H.gravity)]).rhs
    vel_sl = schema.sl("velocity")
    apogee = Event(
        "apogee", lambda t, y: float(y[vel_sl][0]), direction=-1.0,
        action=EventAction.TERMINATE,
    )
    coast = SimulationEngine(
        coast_rhs,
        ScipyIVPIntegrator(SolverConfig(rtol=2e-9, atol=1e-10, max_step=5.0)),
    ).run((burn_time, burn_time + 2000.0), y_burn, (apogee,))
    if not coast.success or coast.terminated_by != "apogee":
        raise RuntimeError("reference ascent did not reach apogee")
    final = schema.unpack(coast.states[-1])
    return {
        "apogee_altitude": BODY_H.altitude(final["position"]),
        "propellant_used": MASS0_H - float(final["mass"]),
        "final_mass": float(final["mass"]),
        "burnout_altitude": BODY_H.altitude(b["position"]),
        "burnout_speed": float(np.linalg.norm(b["velocity"])),
        "apogee_time": float(coast.times[-1]),
    }


def build_radial_ascent_targeter(*, mdot: float = 5.0):
    space = DesignSpace([
        DesignVariable("burn_time", initial=7.0, lower=1.0, upper=30.0, scale=10.0),
    ])

    def residual(p):
        metrics = evaluate_radial_ascent_event({"mdot": mdot, "burn_time": p["burn_time"]})
        return np.array([(metrics["apogee_altitude"] - TARGET_APOGEE_H)/TARGET_APOGEE_H])

    return TrajectoryTargeter(space, residual, TargetingSettings(max_nfev=80))


def build_radial_ascent_optimizer():
    space = DesignSpace([
        DesignVariable("mdot", initial=4.0, lower=1.0, upper=8.0, scale=4.0),
        DesignVariable("burn_time", initial=8.0, lower=1.0, upper=30.0, scale=10.0),
    ])
    problem = TrajectoryProblem(
        space, evaluate_radial_ascent,
        MetricObjective("propellant_used", "minimize", scale=50.0),
        constraints=(MetricConstraint(
            "apogee_altitude", lower=TARGET_APOGEE_H, upper=TARGET_APOGEE_H,
            scale=TARGET_APOGEE_H, name="target_apogee",
        ),),
    )
    optimizer = TrajectoryOptimizer(OptimizationSettings(
        method="SLSQP", maxiter=80, ftol=1e-10,
        constraint_tolerance=2e-6, fallback_method="COBYLA",
    ))
    return problem, optimizer
