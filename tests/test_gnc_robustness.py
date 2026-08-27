import numpy as np

from uniflight import (
    AbortManager, AttitudeRateSensor, BusScalarProvider, CommandedBodyTorque,
    ConstantMassProperties, DynamicsAssembler, FirstOrderLimitedStateActuator,
    GNCCommandBus, GimballedRocketEngine, LimitAbortRule, MassFlowAggregator,
    MonteCarloRunner, NormalDispersion, PlanetaryEnvironment, PositionVelocitySensor,
    QuaternionKinematics, QuaternionPDController, RigidBody6DOFDynamics,
    SampledDataClosedLoopEngine, ScipyIVPIntegrator, SolverConfig, SphericalBody,
    StateFieldProvider, StateView, TranslationalNavigationEKF, UniformDispersion,
    VectorLandingGuidance, LandingGNCController, Event, EventAction,
    gnc_edl_6dof_schema, quaternion_align_body_x,
)


def world():
    body = SphericalBody(mu=3.0e12, radius=8.0e5, name="Nereid-F")
    return body, PlanetaryEnvironment(body)


def base_values(schema, body, altitude=100.0, lateral=0.0, radial_speed=-10.0, lateral_speed=0.0, mass=500.0):
    return schema.pack({
        "position": np.array([body.radius+altitude,lateral,0.0]),
        "velocity": np.array([radial_speed,lateral_speed,0.0]),
        "attitude": np.array([1.0,0.0,0.0,0.0]),
        "angular_rate": np.zeros(3),
        "mass": mass,
        "tps_temperature": 300.0,
        "heat_load": 0.0,
        "tps_mass": 0.0,
        "parachute_deployment": 0.0,
        "gear_deployment": 1.0,
        "throttle_actuator": 0.0,
        "pitch_gimbal_actuator": 0.0,
        "yaw_gimbal_actuator": 0.0,
    })


def test_048_position_velocity_sensor_is_seed_deterministic():
    body,_=world(); schema=gnc_edl_6dof_schema(); y=base_values(schema,body)
    s=PositionVelocitySensor(2.0,0.2)
    a=s.measure(StateView(1.0,y,schema),np.random.default_rng(42))
    b=s.measure(StateView(1.0,y,schema),np.random.default_rng(42))
    assert np.allclose(a.value,b.value)
    assert np.allclose(np.diag(a.covariance),[4,4,4,.04,.04,.04])


def test_049_attitude_rate_sensor_preserves_unit_quaternion():
    body,_=world(); schema=gnc_edl_6dof_schema(); y=base_values(schema,body)
    m=AttitudeRateSensor(0.01,0.001).measure(StateView(0,y,schema),np.random.default_rng(4))
    assert abs(np.linalg.norm(m.attitude)-1.0)<1e-14


def test_050_navigation_ekf_update_reduces_covariance():
    body,_=world()
    nav=TranslationalNavigationEKF(np.zeros(6),np.eye(6)*100.0,body.gravity,0.1)
    sensor=PositionVelocitySensor(1.0,0.1)
    schema=gnc_edl_6dof_schema(); y=base_values(schema,body)
    meas=sensor.measure(StateView(0,y,schema),np.random.default_rng(0))
    before=np.trace(nav.covariance)
    update=nav.update_position_velocity(meas)
    assert np.trace(nav.covariance)<before
    assert update.nis>=0


def test_051_first_order_actuator_obeys_rate_and_position_limits():
    body,_=world(); schema=gnc_edl_6dof_schema(); y=base_values(schema,body)
    bus=GNCCommandBus(throttle=1.0)
    act=FirstOrderLimitedStateActuator("throttle_actuator",BusScalarProvider(bus,"throttle"),0.01,0.0,1.0,rate_limit=2.0)
    dx=act.derivatives(StateView(0,y,schema))["throttle_actuator"]
    assert dx==2.0
    vals=schema.unpack(y); vals["throttle_actuator"]=1.0; y2=schema.pack(vals)
    assert act.derivatives(StateView(0,y2,schema))["throttle_actuator"]==0.0


def test_052_quaternion_pd_zero_error_and_saturation():
    c=QuaternionPDController(100.0,20.0,5.0)
    q=np.array([1.,0.,0.,0.])
    assert np.allclose(c.command(q,np.zeros(3),q),0.0)
    qd=quaternion_align_body_x(np.array([0.,1.,0.]),np.array([0.,0.,1.]))
    tau=c.command(q,np.zeros(3),qd)
    assert np.max(np.abs(tau))<=5.0+1e-15
    assert np.linalg.norm(tau)>0


def test_053_vector_landing_guidance_includes_gravity_and_lateral_correction():
    body,env=world()
    target=np.array([body.radius-1.0,0.,0.])
    g=VectorLandingGuidance(env,target,0.01,0.3,30.0)
    x=np.array([body.radius+100.,20.,0.,-10.,0.,0.])
    cmd=g.evaluate(x,25.0)
    assert cmd.thrust_acceleration_i[0]>0.0  # braking + gravity support
    assert cmd.thrust_acceleration_i[1]<0.0  # correct lateral offset
    assert 0<cmd.throttle<=1


def test_054_commanded_body_torque_saturates():
    body,_=world(); schema=gnc_edl_6dof_schema(); y=base_values(schema,body)
    bus=GNCCommandBus(); bus.set(torque_b=np.array([10.,-20.,1.]))
    w=CommandedBodyTorque(bus,np.array([5.,6.,7.])).wrench(StateView(0,y,schema))
    assert np.allclose(w.moment_b,[5.,-6.,1.])


def test_055_abort_rule_produces_terminal_event_at_limit():
    body,_=world(); schema=gnc_edl_6dof_schema()
    speed=lambda st: float(np.linalg.norm(st.get("velocity")))
    rule=LimitAbortRule("overspeed",schema,speed,upper=20.0)
    event=rule.event()
    assert event.action is EventAction.TERMINATE
    y=base_values(schema,body,radial_speed=-25.0)
    assert rule.violated(StateView(0,y,schema))
    assert AbortManager((rule,)).violations(StateView(0,y,schema))==("overspeed",)


def test_056_monte_carlo_is_deterministic_for_same_seed():
    def case(params,rng):
        return {"success":params["x"]<1.0,"score":params["x"]+rng.normal(0,0.01)}
    dispersions={"x":NormalDispersion(0.0,1.0)}
    a=MonteCarloRunner(case,dispersions,123).run(10)
    b=MonteCarloRunner(case,dispersions,123).run(10)
    assert [r.parameters for r in a.cases]==[r.parameters for r in b.cases]
    assert np.allclose([r.metrics["score"] for r in a.cases],[r.metrics["score"] for r in b.cases])
    assert a.success_rate==b.success_rate


def test_057_monte_carlo_statistics_are_reported():
    def case(params,rng): return {"success":True,"metric":params["x"]}
    s=MonteCarloRunner(case,{"x":UniformDispersion(-1,1)},4).run(20)
    assert s.success_rate==1.0
    assert "metric" in s.statistics
    assert s.statistics["metric"].minimum<=s.statistics["metric"].median<=s.statistics["metric"].maximum


def make_closed_loop(seed=7, position_std=0.5, velocity_std=0.05, lateral=10.0):
    body,env=world(); schema=gnc_edl_6dof_schema()
    bus=GNCCommandBus()
    mp=ConstantMassProperties(np.diag([200.,250.,250.]))
    throttle_act=FirstOrderLimitedStateActuator("throttle_actuator",BusScalarProvider(bus,"throttle"),0.15,0,1,3.0)
    pitch_act=FirstOrderLimitedStateActuator("pitch_gimbal_actuator",BusScalarProvider(bus,"pitch_gimbal"),0.1,-0.15,0.15,0.5)
    yaw_act=FirstOrderLimitedStateActuator("yaw_gimbal_actuator",BusScalarProvider(bus,"yaw_gimbal"),0.1,-0.15,0.15,0.5)
    engine=GimballedRocketEngine(env,mp,2000.0,6.0,throttle=StateFieldProvider("throttle_actuator"),
                                pitch_gimbal=StateFieldProvider("pitch_gimbal_actuator"),
                                yaw_gimbal=StateFieldProvider("yaw_gimbal_actuator"),dry_mass=300.0)
    torque=CommandedBodyTorque(bus,np.array([350.,350.,350.]))
    dyn=RigidBody6DOFDynamics(mp,body.gravity,(engine,torque))
    rhs=DynamicsAssembler(schema,[dyn,QuaternionKinematics(),throttle_act,pitch_act,yaw_act,MassFlowAggregator((engine,))]).rhs
    target=np.array([body.radius-1.0,0.,0.])
    guide=VectorLandingGuidance(env,target,kp_position=0.012,kd_velocity=0.32,max_thrust_acceleration=24.0)
    attitude=QuaternionPDController(220.0,80.0,np.array([300.,300.,300.]))
    controller=LandingGNCController(guide,attitude,2000.0,6.0,bus)
    y0=base_values(schema,body,altitude=120.0,lateral=lateral,radial_speed=-12.0,lateral_speed=-1.0,mass=500.0)
    pv=PositionVelocitySensor(position_std,velocity_std)
    att=AttitudeRateSensor(0.001,0.002)
    x0=np.concatenate((schema.unpack(y0)["position"]+np.array([3.,-2.,1.]),schema.unpack(y0)["velocity"]+np.array([.5,-.2,.1])))
    nav=TranslationalNavigationEKF(x0,np.diag([25.,25.,25.,1.,1.,1.]),body.gravity,0.25)
    engine_loop=SampledDataClosedLoopEngine(rhs,schema,pv,att,nav,controller,0.5,
        ScipyIVPIntegrator(SolverConfig(rtol=2e-8,atol=1e-9,max_step=0.2)),seed=seed)
    pos=schema.sl("position")
    touchdown=Event("touchdown",lambda t,y: np.linalg.norm(y[pos])-body.radius,direction=-1,action=EventAction.TERMINATE)
    return body,schema,y0,engine_loop,touchdown


def test_058_sampled_data_closed_loop_lands_with_noisy_navigation():
    body,schema,y0,engine,touchdown=make_closed_loop()
    result=engine.run((0,150),y0,(touchdown,))
    assert result.success and result.terminated_by=="touchdown"
    final=schema.unpack(result.states[-1])
    radial=np.linalg.norm(final["position"])-body.radius
    lateral=np.linalg.norm(final["position"][1:])
    vr=float(np.dot(final["velocity"],final["position"]/np.linalg.norm(final["position"])))
    assert abs(radial)<1e-5
    assert lateral<5.0
    assert abs(vr)<5.0
    assert final["mass"]>300.0
    assert len(result.gnc_records)>10
    assert np.trace(result.gnc_records[-1].covariance)<np.trace(result.gnc_records[0].covariance)*2


def test_059_abort_event_can_terminate_closed_loop_before_touchdown():
    body,schema,y0,engine,touchdown=make_closed_loop(seed=9)
    rule=LimitAbortRule("mass-reserve",schema,lambda st:float(st.get("mass")),lower=495.0,priority=2000)
    result=engine.run((0,80),y0,(touchdown,rule.event()))
    assert result.success
    assert result.terminated_by=="mass-reserve"

