"""Milestone G robust terminal-landing scenario used for validation and campaigns.

This module deliberately keeps the scenario function importable at module scope
so it can be executed by ``ProcessPoolExecutor`` under spawn-based platforms.
"""
from __future__ import annotations
from dataclasses import dataclass
from typing import Mapping
import numpy as np

from .actuators import (
    BusScalarProvider, CommandedBodyTorque, FirstOrderLimitedStateActuator,
    GNCCommandBus, StateFieldProvider,
)
from .bodies import SphericalBody
from .closed_loop import ClosedLoopResult, SampledDataClosedLoopEngine
from .control import (AdaptiveThrustScaleEstimator, LandingGNCController, QuaternionPDController, VectorLandingGuidance)
from .dynamics import DynamicsAssembler, QuaternionKinematics, RigidBody6DOFDynamics
from .environment import PlanetaryEnvironment
from .events import Event, EventAction
from .estimation import TranslationalNavigationEKF
from .integrators import (
    FixedStepRK4Config, FixedStepRK4Integrator, ScipyIVPIntegrator, SolverConfig,
)
from .mass_properties import ConstantMassProperties
from .massflow import MassFlowAggregator
from .propulsion import GimballedRocketEngine
from .sensors import AttitudeRateSensor, PositionVelocitySensor
from .state import StateSchema, gnc_edl_6dof_schema


@dataclass(frozen=True, slots=True)
class GLandingResult:
    result: ClosedLoopResult
    metrics: Mapping[str, float | bool]


def build_g_landing_case(*, seed: int, lateral_y: float = 10.0, lateral_z: float = 0.0,
                         radial_speed: float = -12.0, thrust_scale: float = 1.0,
                         sensor_bias_y: float = 0.0, sample_period: float = 0.5,
                         integrator_kind: str = "scipy", rk4_step: float = 0.1,
                         record_trajectory: bool = True, record_gnc_records: bool | None = None):
    body = SphericalBody(mu=3.0e12, radius=8.0e5, name="Nereid-G")
    env = PlanetaryEnvironment(body)
    schema = gnc_edl_6dof_schema()
    bus = GNCCommandBus()
    mp = ConstantMassProperties(np.diag([200.0, 250.0, 250.0]))

    throttle_act = FirstOrderLimitedStateActuator(
        "throttle_actuator", BusScalarProvider(bus, "throttle"),
        time_constant=0.15, lower=0.0, upper=1.0, rate_limit=3.0,
    )
    pitch_act = FirstOrderLimitedStateActuator(
        "pitch_gimbal_actuator", BusScalarProvider(bus, "pitch_gimbal"),
        time_constant=0.1, lower=-0.15, upper=0.15, rate_limit=0.5,
    )
    yaw_act = FirstOrderLimitedStateActuator(
        "yaw_gimbal_actuator", BusScalarProvider(bus, "yaw_gimbal"),
        time_constant=0.1, lower=-0.15, upper=0.15, rate_limit=0.5,
    )

    actual_exhaust_velocity = 2000.0 * thrust_scale
    engine = GimballedRocketEngine(
        env, mp, exhaust_velocity=actual_exhaust_velocity, mdot_exhaust=6.0,
        throttle=StateFieldProvider("throttle_actuator"),
        pitch_gimbal=StateFieldProvider("pitch_gimbal_actuator"),
        yaw_gimbal=StateFieldProvider("yaw_gimbal_actuator"),
        dry_mass=300.0,
    )
    torque = CommandedBodyTorque(bus, np.array([350.0, 350.0, 350.0]))
    dynamics = RigidBody6DOFDynamics(mp, body.gravity, (engine, torque))
    rhs = DynamicsAssembler(schema, [
        dynamics, QuaternionKinematics(), throttle_act, pitch_act, yaw_act,
        MassFlowAggregator((engine,)),
    ]).rhs

    target = np.array([body.radius - 1.0, 0.0, 0.0])
    guidance = VectorLandingGuidance(
        env, target, kp_position=0.012, kd_velocity=0.32,
        max_thrust_acceleration=24.0,
        terminal_sink_rate=0.50, terminal_zone=30.0,
    )
    attitude = QuaternionPDController(220.0, 80.0, np.array([300.0] * 3))
    thrust_estimator = AdaptiveThrustScaleEstimator(
        adaptation_gain=0.08, minimum=0.85, maximum=1.15,
        min_commanded_acceleration=0.75, innovation_limit=0.20,
    )
    controller = LandingGNCController(
        guidance, attitude, 2000.0, 6.0, bus,
        thrust_scale_estimator=thrust_estimator,
    )

    values = {
        "position": np.array([body.radius + 120.0, lateral_y, lateral_z]),
        "velocity": np.array([radial_speed, -1.0, 0.0]),
        "attitude": np.array([1.0, 0.0, 0.0, 0.0]),
        "angular_rate": np.zeros(3),
        "mass": 500.0,
        "tps_temperature": 300.0,
        "heat_load": 0.0,
        "tps_mass": 0.0,
        "parachute_deployment": 0.0,
        "gear_deployment": 1.0,
        "throttle_actuator": 0.0,
        "pitch_gimbal_actuator": 0.0,
        "yaw_gimbal_actuator": 0.0,
    }
    y0 = schema.pack(values)

    pv_sensor = PositionVelocitySensor(
        position_std=0.5, velocity_std=0.05,
        position_bias_i=np.array([0.0, sensor_bias_y, 0.0]),
    )
    attitude_sensor = AttitudeRateSensor(0.001, 0.002)
    x0 = np.concatenate((
        values["position"] + np.array([3.0, -2.0, 1.0]),
        values["velocity"] + np.array([0.5, -0.2, 0.1]),
    ))
    navigator = TranslationalNavigationEKF(
        x0, np.diag([25.0, 25.0, 25.0, 1.0, 1.0, 1.0]),
        body.gravity, accel_process_std=0.25,
    )

    if integrator_kind == "scipy":
        integrator = ScipyIVPIntegrator(
            SolverConfig(rtol=2e-8, atol=1e-9, max_step=min(0.3, sample_period))
        )
    elif integrator_kind == "rk4":
        att_sl = schema.sl("attitude")
        def projector(y: np.ndarray) -> np.ndarray:
            q = y[att_sl]
            nq = float(np.linalg.norm(q))
            if nq == 0 or not np.isfinite(nq):
                raise ValueError("invalid attitude quaternion during RK4 projection")
            y[att_sl] = q/nq
            return y
        integrator = FixedStepRK4Integrator(
            FixedStepRK4Config(step=rk4_step, save_every_step=False),
            state_projector=projector,
        )
    else:
        raise ValueError("integrator_kind must be 'scipy' or 'rk4'")

    closed_loop = SampledDataClosedLoopEngine(
        rhs, schema, pv_sensor, attitude_sensor, navigator, controller,
        sample_period=sample_period, integrator=integrator, seed=seed,
        record_trajectory=record_trajectory,
        record_gnc_records=record_trajectory if record_gnc_records is None else record_gnc_records,
    )
    pos_sl = schema.sl("position")
    touchdown = Event(
        "touchdown",
        lambda t, y: np.linalg.norm(y[pos_sl]) - body.radius,
        direction=-1.0, action=EventAction.TERMINATE,
    )
    return body, schema, y0, closed_loop, touchdown, controller


def run_g_landing(*, seed: int = 7, sample_period: float = 0.5,
                  integrator_kind: str = "scipy", rk4_step: float = 0.1,
                  record_trajectory: bool = True, record_gnc_records: bool | None = None,
                  **kwargs) -> GLandingResult:
    body, schema, y0, sim, touchdown, controller = build_g_landing_case(
        seed=seed, sample_period=sample_period, integrator_kind=integrator_kind,
        rk4_step=rk4_step, record_trajectory=record_trajectory,
        record_gnc_records=record_gnc_records, **kwargs,
    )
    result = sim.run((0.0, 150.0), y0, (touchdown,))
    final = schema.unpack(result.states[-1])
    altitude = np.linalg.norm(final["position"]) - body.radius
    lateral_error = float(np.linalg.norm(final["position"][1:]))
    speed = float(np.linalg.norm(final["velocity"]))
    radial_speed = float(
        np.dot(final["velocity"], final["position"] / np.linalg.norm(final["position"]))
    )
    success = bool(
        result.success
        and result.terminated_by == "touchdown"
        and lateral_error < 5.0
        and speed < 3.0
        and final["mass"] > 300.0
    )
    metrics: dict[str, float | bool] = {
        "success": success,
        "touchdown_time": float(result.times[-1]),
        "altitude": float(altitude),
        "landing_error": lateral_error,
        "touchdown_speed": speed,
        "radial_speed": radial_speed,
        "final_mass": float(final["mass"]),
    }
    metrics["estimated_thrust_scale"] = float(controller.thrust_scale_estimator.estimate) if controller.thrust_scale_estimator is not None else 1.0
    return GLandingResult(result, metrics)


def g_landing_monte_carlo_case(params: Mapping[str, float], rng: np.random.Generator,
                               *, sample_period: float = 0.5,
                               integrator_kind: str = "rk4",
                               rk4_step: float = 0.1) -> Mapping[str, float | bool]:
    """Pickleable module-level case function for multiprocessing campaigns."""
    seed = int(rng.integers(0, 2**31-1))
    return run_g_landing(
        seed=seed, sample_period=sample_period, integrator_kind=integrator_kind,
        rk4_step=rk4_step, record_trajectory=False,
        lateral_y=float(params["lateral_y"]),
        lateral_z=float(params["lateral_z"]),
        radial_speed=float(params["radial_speed"]),
        thrust_scale=float(params["thrust_scale"]),
        sensor_bias_y=float(params["sensor_bias_y"]),
    ).metrics
