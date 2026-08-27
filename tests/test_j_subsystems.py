import numpy as np

from uniflight.state import StateView, core_6dof_schema, augment_engineering_schema, engineering_6dof_schema
from uniflight.flexibility import ModalFlexibleBody, TorqueToModalForce, FlexiblePointKinematics, FlexibleAttitudeRateSensor
from uniflight.slosh import LinearSloshSubsystem
from uniflight.engine_dynamics import EngineTransient
from uniflight.actuators import GNCCommandBus, SecondOrderLimitedStateActuator, StateFieldProvider
from uniflight.faults import FaultWindow, FaultMode, ScalarFaultSchedule, FaultedScalarProvider, FaultedWrenchModel
from uniflight.gear_dynamics import DynamicGearLeg, DynamicLandingGear
from uniflight.mass_properties import ConstantMassProperties
from uniflight.bodies import SphericalBody
from uniflight.terrain import RadialTerrain
from uniflight.sensors import AttitudeRateSensor
from uniflight.environment import PlanetaryEnvironment
from uniflight.propulsion import GimballedRocketEngine
from uniflight.subsystems import WrenchSpecificForceBodyProvider, SubsystemBundle
from uniflight.dynamics import DynamicsAssembler, RigidBody6DOFDynamics, QuaternionKinematics
from uniflight.integrators import ScipyIVPIntegrator, SolverConfig
from uniflight.simulation import SimulationEngine


def base_values(schema):
    values = {}
    for f in schema.fields:
        if f.key == "attitude": values[f.key] = np.array([1.,0.,0.,0.])
        elif f.key == "mass": values[f.key] = 100.0
        elif f.shape: values[f.key] = np.zeros(f.shape)
        else: values[f.key] = 0.0
    return values


def test_engineering_schema_is_composable():
    core = core_6dof_schema()
    s = augment_engineering_schema(core, flex_modes=2, slosh_modes=1, gear_legs=4)
    assert s.field("flex_displacement").shape == (2,)
    assert s.field("slosh_displacement").shape == (1,)
    assert s.field("gear_compression").shape == (4,)
    assert s.field("engine_power").shape == ()
    assert engineering_6dof_schema(flex_modes=1, slosh_modes=1, gear_legs=2).total_size > core.total_size


def test_modal_flexible_body_equation_and_energy():
    s = augment_engineering_schema(core_6dof_schema(), flex_modes=1, engine_dynamics=False, second_order_gimbals=False)
    v = base_values(s); v["flex_displacement"] = np.array([0.2]); v["flex_velocity"] = np.array([0.0])
    st = StateView(0.0, s.pack(v), s)
    m = ModalFlexibleBody(2.0, 0.0, modal_mass=3.0)
    d = m.derivatives(st)
    wn = 4*np.pi
    assert np.allclose(d["flex_velocity"], [-wn*wn*0.2])
    expected = 0.5*3.0*(wn*0.2)**2
    assert np.isclose(m.modal_energy(st), expected)


def test_torque_participation_and_flexible_sensor_csi():
    s = augment_engineering_schema(core_6dof_schema(), flex_modes=1, engine_dynamics=False, second_order_gimbals=False)
    v = base_values(s); v["flex_displacement"] = np.array([0.01]); v["flex_velocity"] = np.array([0.02])
    st = StateView(0.0, s.pack(v), s)
    bus = GNCCommandBus(torque_b=np.array([0.0, 4.0, 0.0]))
    qforce = TorqueToModalForce(bus, np.array([[0.0, 0.5, 0.0]]))
    assert np.allclose(qforce(st), [2.0])
    kin = FlexiblePointKinematics(np.zeros((3,1)), np.array([[0.0],[1.0],[0.0]]))
    sensor = FlexibleAttitudeRateSensor(AttitudeRateSensor(), kin)
    meas = sensor.measure(st, np.random.default_rng(1))
    # Flex station rotates about +y and carries flex angular rate.
    assert meas.attitude[2] > 0
    assert np.isclose(meas.angular_rate[1], 0.02)


def test_linear_slosh_reaction_wrench():
    s = augment_engineering_schema(core_6dof_schema(), slosh_modes=1, engine_dynamics=False, second_order_gimbals=False)
    v = base_values(s); v["slosh_displacement"] = np.array([0.1]); v["slosh_velocity"] = np.array([0.0])
    st = StateView(0.0, s.pack(v), s)
    mp = ConstantMassProperties(np.diag([10.,20.,30.]))
    slosh = LinearSloshSubsystem(10.0, 1.0, 0.0, np.array([[0.,1.,0.]]), np.array([2.,0.,0.]), mp)
    d = slosh.derivatives(st)
    assert np.isclose(d["slosh_velocity"][0], -(2*np.pi)**2*0.1)
    w = slosh.wrench(st)
    fy = 10.0*(2*np.pi)**2*0.1
    assert np.allclose(w.force_i, [0.,fy,0.])
    assert np.allclose(w.moment_b, [0.,0.,2*fy])


def test_engine_transient_and_second_order_servo_limits():
    s = augment_engineering_schema(core_6dof_schema(), engine_dynamics=True, second_order_gimbals=True)
    v = base_values(s)
    st = StateView(0.0, s.pack(v), s)
    eng = EngineTransient(1.0, natural_frequency_hz=1.0, damping_ratio=1.0, max_acceleration=5.0)
    d = eng.derivatives(st)
    assert d["engine_power"] == 0.0
    assert np.isclose(d["engine_power_rate"], 5.0)
    servo = SecondOrderLimitedStateActuator("pitch_gimbal_actuator", "pitch_gimbal_rate", 0.2,
                                            2.0, 0.8, -0.1, 0.1,
                                            rate_limit=0.05, acceleration_limit=0.2)
    sd = servo.derivatives(st)
    assert np.isclose(sd["pitch_gimbal_rate"], 0.2)
    # Command itself is clipped to the position hard stop.
    assert np.isclose(servo.command_value(st), 0.1)


def test_fault_schedule_gain_bias_stuck_and_dropout():
    schedule = ScalarFaultSchedule((
        FaultWindow(1.0, 2.0, FaultMode.GAIN, 0.5),
        FaultWindow(2.0, 3.0, FaultMode.BIAS, 0.1),
        FaultWindow(3.0, 4.0, FaultMode.STUCK, 0.25),
        FaultWindow(4.0, None, FaultMode.DROPOUT),
    ))
    assert np.isclose(schedule.apply(0.8, 0.5), 0.8)
    assert np.isclose(schedule.apply(0.8, 1.5), 0.4)
    assert np.isclose(schedule.apply(0.8, 2.5), 0.9)
    assert np.isclose(schedule.apply(0.8, 3.5), 0.25)
    assert np.isclose(schedule.apply(0.8, 5.0), 0.0)


def test_dynamic_landing_gear_state_and_wrench():
    body = SphericalBody(mu=1e5, radius=100.0)
    terrain = RadialTerrain(body)
    s = augment_engineering_schema(core_6dof_schema(), gear_legs=1, engine_dynamics=False, second_order_gimbals=False)
    v = base_values(s); v["position"] = np.array([101.,0.,0.]); v["gear_deployment"] = 1.0
    v["gear_compression"] = np.array([0.1]); v["gear_compression_rate"] = np.array([0.0])
    st = StateView(0.0, s.pack(v), s)
    mp = ConstantMassProperties(np.diag([10.,10.,10.]))
    leg = DynamicGearLeg(np.array([-1.2,0,0]), np.array([1.,0,0]), 1000., 100., 10., 0.3)
    gear = DynamicLandingGear(terrain, mp, (leg,))
    d = gear.derivatives(st)
    assert d["gear_compression_rate"][0] > 0
    w = gear.wrench(st)
    assert w.force_i[0] > 0
    assert np.allclose(w.moment_b, 0.0)


def test_faulted_wrench_scaling():
    class ConstantWrench:
        def wrench(self, state):
            from uniflight.wrenches import Wrench
            return Wrench(np.array([2.,0,0]), np.array([0,3.,0]), "base")
    s=core_6dof_schema(); v=base_values(s); st=StateView(2.0,s.pack(v),s)
    fw=FaultedWrenchModel(ConstantWrench(), ScalarFaultSchedule((FaultWindow(1.,3.,"gain",0.25),)))
    w=fw.wrench(st)
    assert np.allclose(w.force_i,[0.5,0,0]) and np.allclose(w.moment_b,[0,0.75,0])


def test_subsystem_bundle_combination():
    a=SubsystemBundle((1,),(2,),(3,)); b=SubsystemBundle((4,),(5,6),())
    c=SubsystemBundle.combine(a,b)
    assert c.derivative_models==(1,4) and c.wrench_models==(2,5,6) and c.mass_flow_sources==(3,)


def test_coupled_engine_flex_slosh_rigid_body_integration():
    body=SphericalBody(mu=1e-6,radius=1.0)
    env=PlanetaryEnvironment(body)
    s=augment_engineering_schema(core_6dof_schema(), flex_modes=1, slosh_modes=1,
                                 engine_dynamics=True, second_order_gimbals=False)
    v=base_values(s); v["position"]=np.array([1000.,0,0.]); v["mass"]=100.0
    y0=s.pack(v)
    mp=ConstantMassProperties(np.diag([20.,25.,30.]))
    engine=GimballedRocketEngine(env,mp,exhaust_velocity=2000.,mdot_exhaust=0.5,
                                 mount_position_b=np.array([0.,0.5,0.]),
                                 throttle=StateFieldProvider("engine_power"))
    engine_dyn=EngineTransient(1.0,natural_frequency_hz=2.0,damping_ratio=1.0,max_rate=3.0,max_acceleration=20.0)
    flex=ModalFlexibleBody(3.0,0.02,modal_mass=5.0,
                           generalized_force=lambda st: np.array([0.01*engine.wrench(st).moment_b[2]]))
    base_acc=WrenchSpecificForceBodyProvider((engine,),mp)
    slosh=LinearSloshSubsystem(5.0,1.2,0.05,np.array([[0.,1.,0.]]),np.array([0.5,0,0]),mp,base_acc)
    rigid=RigidBody6DOFDynamics(mp,gravity=None,wrench_models=(engine,slosh))
    rhs=DynamicsAssembler(s,[rigid,QuaternionKinematics(),engine_dyn,flex,slosh,engine]).rhs
    result=SimulationEngine(rhs,ScipyIVPIntegrator(SolverConfig(rtol=1e-9,atol=1e-11,max_step=0.02))).run((0.,1.5),y0)
    assert result.success
    final=StateView(result.times[-1],result.states[-1],s)
    assert final.get("engine_power") > 0.9
    assert final.get("mass") < 100.0
    assert np.linalg.norm(final.get("angular_rate")) > 1e-4
    assert abs(final.get("flex_displacement")[0]) > 1e-6
    # Symmetric axial thrust should not directly excite the transverse slosh mode.
    assert np.isfinite(final.get("slosh_displacement")[0])
