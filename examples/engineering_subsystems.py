"""Milestone J reference: coupled engineering subsystems on fictional Nereid-J."""
from __future__ import annotations
import argparse, json
from pathlib import Path
import numpy as np

from uniflight import (
    SphericalBody, PlanetaryEnvironment, ConstantMassProperties,
    core_6dof_schema, augment_engineering_schema, StateView,
    GimballedRocketEngine, StateFieldProvider, EngineTransient,
    SecondOrderLimitedStateActuator, ModalFlexibleBody, LinearSloshSubsystem,
    WrenchSpecificForceBodyProvider, RigidBody6DOFDynamics, QuaternionKinematics,
    DynamicsAssembler, ScipyIVPIntegrator, SolverConfig, SimulationEngine,
    FaultWindow, ScalarFaultSchedule, FaultedScalarProvider,
    RadialTerrain, DynamicGearLeg, DynamicLandingGear,
)


def zero_values(schema):
    out = {}
    for f in schema.fields:
        if f.key == "attitude": out[f.key] = np.array([1.,0.,0.,0.])
        elif f.key == "mass": out[f.key] = 1.0
        elif f.shape: out[f.key] = np.zeros(f.shape)
        else: out[f.key] = 0.0
    return out


def powered_subsystem_case():
    body = SphericalBody(mu=3.0e11, radius=1.0e6, name="Nereid-J")
    env = PlanetaryEnvironment(body)
    schema = augment_engineering_schema(core_6dof_schema(), flex_modes=2, slosh_modes=1,
                                        gear_legs=0, engine_dynamics=True,
                                        second_order_gimbals=True)
    vals = zero_values(schema)
    vals.update(position=np.array([body.radius+100.0,0.,0.]), mass=500.0)
    y0 = schema.pack(vals)

    mp = ConstantMassProperties(np.diag([800., 950., 1100.]))
    fault = ScalarFaultSchedule((FaultWindow(2.0, 3.5, "gain", 0.70),))
    engine_command = FaultedScalarProvider(1.0, fault, lower=0.0, upper=1.0)
    engine_transient = EngineTransient(engine_command, natural_frequency_hz=2.5,
                                       damping_ratio=1.2, max_rate=4.0,
                                       max_acceleration=30.0)

    def pitch_cmd(st):
        if st.time < 0.5: return 0.0
        if st.time < 2.5: return 0.04
        if st.time < 4.0: return -0.02
        return 0.0

    pitch_servo = SecondOrderLimitedStateActuator(
        "pitch_gimbal_actuator", "pitch_gimbal_rate", pitch_cmd,
        natural_frequency_hz=5.0, damping_ratio=0.85,
        lower=-0.08, upper=0.08, rate_limit=0.5, acceleration_limit=8.0,
    )
    yaw_servo = SecondOrderLimitedStateActuator(
        "yaw_gimbal_actuator", "yaw_gimbal_rate", 0.0,
        natural_frequency_hz=5.0, damping_ratio=0.85,
        lower=-0.08, upper=0.08, rate_limit=0.5, acceleration_limit=8.0,
    )
    engine = GimballedRocketEngine(
        env, mp, exhaust_velocity=2400.0, mdot_exhaust=1.2,
        mount_position_b=np.array([0.0, 0.6, 0.0]),
        throttle=engine_transient,
        pitch_gimbal=pitch_servo,
        yaw_gimbal=yaw_servo,
    )
    flex = ModalFlexibleBody(
        np.array([3.0, 7.5]), np.array([0.015, 0.025]), np.array([30.0, 12.0]),
        generalized_force=lambda st: np.array([
            0.08*engine.wrench(st).moment_b[2],
            -0.025*engine.wrench(st).moment_b[2],
        ]),
    )
    base_accel = WrenchSpecificForceBodyProvider((engine,), mp)
    slosh = LinearSloshSubsystem(
        35.0, 1.1, 0.04, np.array([[0.0,0.0,1.0]]),
        np.array([-0.5,0.0,0.0]), mp, base_accel,
    )
    rigid = RigidBody6DOFDynamics(mp, gravity=body.gravity, wrench_models=(engine, slosh))
    rhs = DynamicsAssembler(schema, [
        rigid, QuaternionKinematics(), engine_transient, pitch_servo, yaw_servo,
        flex, slosh, engine,
    ]).rhs
    integ = ScipyIVPIntegrator(SolverConfig(rtol=2e-9, atol=1e-11, max_step=0.01))
    res = SimulationEngine(rhs, integ).run((0.0, 6.0), y0)
    if not res.success: raise RuntimeError(res.message)
    views = [StateView(t,y,schema) for t,y in zip(res.times,res.states)]
    final = views[-1]
    return {
        "body": body.name,
        "duration_s": float(res.times[-1]),
        "final_altitude_m": body.altitude(final.get("position")),
        "final_speed_mps": float(np.linalg.norm(final.get("velocity"))),
        "final_mass_kg": float(final.get("mass")),
        "final_engine_power": float(final.get("engine_power")),
        "max_abs_pitch_gimbal_rad": max(abs(float(v.get("pitch_gimbal_actuator"))) for v in views),
        "max_flex_displacement_m": float(max(np.max(np.abs(v.get("flex_displacement"))) for v in views)),
        "max_slosh_displacement_m": float(max(np.max(np.abs(v.get("slosh_displacement"))) for v in views)),
        "max_body_rate_rad_s": float(max(np.linalg.norm(v.get("angular_rate")) for v in views)),
        "minimum_engine_power_during_fault": float(min(v.get("engine_power") for v in views if 2.0 <= v.time <= 3.5)),
    }


def gear_drop_case():
    body = SphericalBody(mu=3.0e11, radius=1.0e6, name="Nereid-J")
    terrain = RadialTerrain(body)
    schema = augment_engineering_schema(core_6dof_schema(), gear_legs=1,
                                        engine_dynamics=False, second_order_gimbals=False)
    vals = zero_values(schema)
    vals.update(position=np.array([body.radius+0.75,0.,0.]),
                velocity=np.array([-0.7,0.,0.]), mass=80.0,
                gear_deployment=1.0)
    y0=schema.pack(vals)
    mp=ConstantMassProperties(np.diag([40.,50.,50.]))
    leg=DynamicGearLeg(np.array([-0.85,0.,0.]), np.array([1.,0.,0.]),
                       stiffness=9000., damping=850., effective_mass=12.,
                       max_compression=0.28, friction_coefficient=0.0)
    gear=DynamicLandingGear(terrain,mp,(leg,))
    rigid=RigidBody6DOFDynamics(mp,gravity=body.gravity,wrench_models=(gear,))
    rhs=DynamicsAssembler(schema,[rigid,QuaternionKinematics(),gear]).rhs
    res=SimulationEngine(rhs,ScipyIVPIntegrator(SolverConfig(rtol=1e-8,atol=1e-10,max_step=0.002))).run((0.,1.0),y0)
    if not res.success: raise RuntimeError(res.message)
    views=[StateView(t,y,schema) for t,y in zip(res.times,res.states)]
    final=views[-1]
    return {
        "duration_s": float(res.times[-1]),
        "max_strut_compression_m": float(max(v.get("gear_compression")[0] for v in views)),
        "final_strut_compression_m": float(final.get("gear_compression")[0]),
        "final_radial_velocity_mps": float(final.get("velocity")[0]),
        "final_cg_agl_m": float(body.altitude(final.get("position"))),
    }


def main():
    ap=argparse.ArgumentParser(); ap.add_argument("--output", type=Path, default=None)
    args=ap.parse_args()
    report={"uniflight_version":"0.10.0","powered_subsystems":powered_subsystem_case(),"dynamic_gear_drop":gear_drop_case()}
    text=json.dumps(report,indent=2)
    print(text)
    if args.output:
        args.output.parent.mkdir(parents=True,exist_ok=True); args.output.write_text(text+"\n",encoding="utf-8")

if __name__=="__main__": main()
