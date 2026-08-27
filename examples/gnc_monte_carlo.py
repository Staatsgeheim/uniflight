"""Milestone F: noisy closed-loop landing and deterministic robustness campaign.

The fictional airless world Nereid-F is used so the example contains no Earth
constants.  The default case count is deliberately small enough for a smoke
run.  Use --cases 100 (or more) for a local robustness campaign and --output
to save a reproducible JSON report.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import platform
import sys

import numpy as np

import uniflight
from uniflight import (
    AttitudeRateSensor, BusScalarProvider, CommandedBodyTorque,
    ConstantMassProperties, DynamicsAssembler, Event, EventAction,
    FirstOrderLimitedStateActuator, GNCCommandBus, GimballedRocketEngine,
    LandingGNCController, MassFlowAggregator, MonteCarloRunner, NormalDispersion,
    PlanetaryEnvironment, PositionVelocitySensor, QuaternionKinematics,
    QuaternionPDController, RigidBody6DOFDynamics, SampledDataClosedLoopEngine,
    ScipyIVPIntegrator, SolverConfig, SphericalBody, StateFieldProvider,
    TranslationalNavigationEKF, VectorLandingGuidance, gnc_edl_6dof_schema,
)


def build_case(*, seed: int, lateral_y: float = 10.0, lateral_z: float = 0.0,
               radial_speed: float = -12.0, thrust_scale: float = 1.0,
               sensor_bias_y: float = 0.0, sample_period: float = 0.75):
    body = SphericalBody(mu=3.0e12, radius=8.0e5, name="Nereid-F")
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
    )
    attitude = QuaternionPDController(220.0, 80.0, np.array([300.0] * 3))
    # Guidance assumes the nominal engine. thrust_scale is therefore a true
    # plant dispersion rather than a parameter disclosed to the controller.
    controller = LandingGNCController(guidance, attitude, 2000.0, 6.0, bus)

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
    closed_loop = SampledDataClosedLoopEngine(
        rhs, schema, pv_sensor, attitude_sensor, navigator, controller,
        sample_period=sample_period,
        integrator=ScipyIVPIntegrator(
            SolverConfig(rtol=2e-8, atol=1e-9, max_step=min(0.3, sample_period))
        ),
        seed=seed,
    )
    pos_sl = schema.sl("position")
    touchdown = Event(
        "touchdown",
        lambda t, y: np.linalg.norm(y[pos_sl]) - body.radius,
        direction=-1.0,
        action=EventAction.TERMINATE,
    )
    return body, schema, y0, closed_loop, touchdown


def run_landing(*, seed: int = 7, sample_period: float = 0.75, **kwargs):
    body, schema, y0, sim, touchdown = build_case(
        seed=seed, sample_period=sample_period, **kwargs
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
    return result, {
        "success": success,
        "touchdown_time": float(result.times[-1]),
        "altitude": float(altitude),
        "landing_error": lateral_error,
        "touchdown_speed": speed,
        "radial_speed": radial_speed,
        "final_mass": float(final["mass"]),
    }


def _summary_to_json(mc, *, cases: int, base_seed: int, sample_period: float, nominal: dict):
    return {
        "metadata": {
            "uniflight_version": getattr(uniflight, "__version__", "unknown"),
            "python": platform.python_version(),
            "numpy": np.__version__,
            "platform": platform.platform(),
            "cases": cases,
            "base_seed": base_seed,
            "sample_period_s": sample_period,
            "success_criteria": {
                "touchdown_event": True,
                "lateral_error_m_lt": 5.0,
                "touchdown_speed_mps_lt": 3.0,
                "final_mass_kg_gt": 300.0,
            },
        },
        "nominal": nominal,
        "summary": {
            "success_rate": mc.success_rate,
            "statistics": {
                name: {
                    "mean": s.mean,
                    "std": s.std,
                    "minimum": s.minimum,
                    "maximum": s.maximum,
                    "p05": s.p05,
                    "median": s.median,
                    "p95": s.p95,
                }
                for name, s in mc.statistics.items()
            },
        },
        "case_results": [
            {
                "index": r.index,
                "seed": r.seed,
                "parameters": dict(r.parameters),
                "metrics": dict(r.metrics),
            }
            for r in mc.cases
        ],
    }


def run_campaign(*, cases: int, base_seed: int, sample_period: float):
    nominal_result, nominal = run_landing(
        seed=7, lateral_y=10.0, sample_period=sample_period
    )

    def case(params, rng):
        _, metrics = run_landing(
            seed=int(rng.integers(0, 2**31 - 1)),
            sample_period=sample_period,
            lateral_y=params["lateral_y"],
            lateral_z=params["lateral_z"],
            radial_speed=params["radial_speed"],
            thrust_scale=params["thrust_scale"],
            sensor_bias_y=params["sensor_bias_y"],
        )
        return metrics

    mc = MonteCarloRunner(case, {
        "lateral_y": NormalDispersion(10.0, 4.0),
        "lateral_z": NormalDispersion(0.0, 4.0),
        "radial_speed": NormalDispersion(-12.0, 0.8),
        "thrust_scale": NormalDispersion(1.0, 0.015),
        "sensor_bias_y": NormalDispersion(0.0, 0.4),
    }, base_seed=base_seed).run(cases)
    return nominal_result, nominal, mc


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--cases", type=int, default=4,
                   help="Monte Carlo cases (default: 4 smoke cases)")
    p.add_argument("--seed", type=int, default=20260827,
                   help="deterministic campaign base seed")
    p.add_argument("--sample-period", type=float, default=0.75,
                   help="sampled GNC update period in seconds")
    p.add_argument("--output", type=Path,
                   help="optional JSON report path")
    p.add_argument("--nominal-only", action="store_true",
                   help="run only the nominal noisy landing")
    return p.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    if args.cases <= 0:
        raise SystemExit("--cases must be positive")
    if args.sample_period <= 0:
        raise SystemExit("--sample-period must be positive")

    if args.nominal_only:
        nominal_result, nominal = run_landing(
            seed=7, lateral_y=10.0, sample_period=args.sample_period
        )
        print("Nominal noisy closed-loop landing")
        for k, v in nominal.items():
            print(f"  {k:>18s}: {v}")
        print(f"  {'GNC updates':>18s}: {len(nominal_result.gnc_records)}")
        return 0

    nominal_result, nominal, mc = run_campaign(
        cases=args.cases, base_seed=args.seed, sample_period=args.sample_period
    )
    print("Nominal noisy closed-loop landing")
    for k, v in nominal.items():
        print(f"  {k:>18s}: {v}")
    print(f"  {'GNC updates':>18s}: {len(nominal_result.gnc_records)}")

    print(f"\nDeterministic {args.cases}-case Monte Carlo")
    print(f"  success rate: {mc.success_rate:.1%}")
    for name in ("landing_error", "touchdown_speed", "final_mass", "touchdown_time"):
        if name in mc.statistics:
            s = mc.statistics[name]
            print(
                f"  {name:>18s}: mean={s.mean:.3f}, "
                f"p05={s.p05:.3f}, p95={s.p95:.3f}"
            )

    if args.output:
        report = _summary_to_json(
            mc, cases=args.cases, base_seed=args.seed,
            sample_period=args.sample_period, nominal=nominal,
        )
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2), encoding="utf-8")
        print(f"\nJSON report: {args.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
