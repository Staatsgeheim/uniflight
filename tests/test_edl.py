import numpy as np

from uniflight import (
    ConstantMassProperties, DynamicsAssembler, Event, EventAction, FirstOrderDeployable,
    GasMixture, GasSpecies, GimballedRocketEngine, GearLeg, HybridModeEngine,
    InflatingParachute, IsothermalHydrostaticAtmosphere, JettisonJump,
    LandingGearContact, MassFlowAggregator, ModeDefinition, PlanetaryEnvironment,
    QuaternionKinematics, RadialTerrain, RigidBody6DOFDynamics, ScipyIVPIntegrator,
    SolverConfig, SphericalBody, StateView, VerticalDescentThrottle, edl_6dof_schema,
    separate_two_body,
)


def reference_world():
    gas = GasSpecies("X2", 0.029, 29.2, 1.75e-5, 300.0, 115.0, 3.7e-10)
    mix = GasMixture((gas,), (1.0,))
    body = SphericalBody(mu=3.0e12, radius=8.0e5, name="Nereid-E")
    atm = IsothermalHydrostaticAtmosphere(18_000.0, 230.0, mix, body.mu, body.radius)
    return body, PlanetaryEnvironment(body, atm)


def state_values(schema, body, altitude=1000.0, speed=-20.0, mass=500.0,
                 chute=0.0, gear=0.0):
    return schema.pack({
        "position": np.array([body.radius+altitude,0.0,0.0]),
        "velocity": np.array([speed,0.0,0.0]),
        "attitude": np.array([1.0,0.0,0.0,0.0]),
        "angular_rate": np.zeros(3),
        "mass": mass,
        "tps_temperature": 300.0,
        "heat_load": 0.0,
        "tps_mass": 0.0,
        "parachute_deployment": chute,
        "gear_deployment": gear,
    })


def test_038_first_order_deployable_has_expected_rate():
    body, _ = reference_world()
    schema = edl_6dof_schema()
    st = StateView(0.0, state_values(schema, body, chute=0.5), schema)
    deploy = FirstOrderDeployable("parachute_deployment", 1.0, 2.0)
    assert deploy.derivatives(st)["parachute_deployment"] == 0.25


def test_039_parachute_drag_opposes_relative_velocity_and_scales_with_inflation():
    body, env = reference_world()
    schema = edl_6dof_schema()
    mp = ConstantMassProperties(np.diag([100.0,100.0,100.0]))
    deploy = FirstOrderDeployable("parachute_deployment", 1.0, 1.0)
    chute = InflatingParachute(env, mp, 100.0, 1.5, deploy)
    s_half = StateView(0.0, state_values(schema, body, altitude=1000.0, speed=-100.0, chute=0.5), schema)
    s_full = StateView(0.0, state_values(schema, body, altitude=1000.0, speed=-100.0, chute=1.0), schema)
    e1 = chute.evaluate(s_half)
    e2 = chute.evaluate(s_full)
    assert e1.force_i[0] > 0.0
    assert np.allclose(e1.force_i[1:], 0.0)
    assert np.isclose(e2.drag/e1.drag, 2.0, rtol=1e-12)


def test_040_radial_terrain_reports_agl_and_surface_point():
    body, _ = reference_world()
    terrain = RadialTerrain(body, 10.0)
    e = terrain.query(np.array([body.radius+100.0,0.0,0.0]))
    assert np.isclose(e.agl, 90.0)
    assert np.isclose(np.linalg.norm(e.surface_point_i), body.radius+10.0)
    assert np.allclose(e.normal_i, [1.0,0.0,0.0])


def test_041_landing_gear_spring_damper_generates_outward_contact_force():
    body, _ = reference_world()
    schema = edl_6dof_schema()
    terrain = RadialTerrain(body)
    mp = ConstantMassProperties(np.diag([100.0,100.0,100.0]))
    leg = GearLeg(np.array([-0.5,0.0,0.0]), np.array([-2.0,0.0,0.0]), 1000.0, 100.0)
    gear = LandingGearContact(terrain, mp, (leg,))
    st = StateView(0.0, state_values(schema, body, altitude=1.0, speed=-1.0, gear=1.0), schema)
    e = gear.evaluate(st)
    assert e.in_contact
    assert np.isclose(e.legs[0].penetration, 1.0)
    assert np.isclose(e.legs[0].normal_force, 1100.0)
    assert e.force_i[0] > 0.0


def test_042_landing_gear_regularized_friction_opposes_tangential_motion():
    body, _ = reference_world()
    schema = edl_6dof_schema()
    terrain = RadialTerrain(body)
    mp = ConstantMassProperties(np.diag([100.0,100.0,100.0]))
    leg = GearLeg(np.array([-2.0,0.0,0.0]), np.array([-2.0,0.0,0.0]), 1000.0, 0.0, 0.5)
    gear = LandingGearContact(terrain, mp, (leg,), active_threshold=0.0)
    y = state_values(schema, body, altitude=1.0, speed=-0.1, gear=1.0)
    values = schema.unpack(y); values["velocity"] = np.array([-0.1,10.0,0.0]); y=schema.pack(values)
    e = gear.evaluate(StateView(0.0,y,schema))
    assert e.force_i[1] < 0.0
    assert abs(e.force_i[1]) <= 0.5*e.legs[0].normal_force + 1e-12


def test_043_two_body_separation_conserves_linear_momentum():
    result = separate_two_body(
        100.0, np.array([1.0,2.0,3.0]), np.array([10.0,-2.0,4.0]),
        80.0, 20.0, np.array([5.0,1.0,-2.0]),
    )
    assert np.linalg.norm(result.momentum_error_i) < 1e-12
    assert np.allclose(result.detached.velocity_i-result.retained.velocity_i, [5.0,1.0,-2.0])


def test_044_jettison_jump_removes_mass_and_resets_deployable():
    body, _ = reference_world()
    schema = edl_6dof_schema()
    y = state_values(schema, body, mass=500.0, chute=1.0)
    jump = JettisonJump(schema, 20.0, {"parachute_deployment": 0.0})
    out = schema.unpack(jump(0.0,y))
    assert out["mass"] == 480.0
    assert out["parachute_deployment"] == 0.0


def test_045_vertical_descent_guidance_contains_gravity_feedforward():
    body, env = reference_world()
    terrain = RadialTerrain(body)
    schema = edl_6dof_schema()
    guide = VerticalDescentThrottle(env, terrain, 2000.0, 3.0, max_descent_speed=20.0,
                                    touchdown_speed=1.0, speed_slope=0.02, velocity_gain=1.0)
    y = state_values(schema, body, altitude=0.0, speed=-1.0, mass=500.0)
    e = guide.evaluate(StateView(0.0,y,schema))
    expected = 500.0*e.gravity_magnitude/(3.0*2000.0)
    assert np.isclose(e.throttle, expected, rtol=1e-12)


def test_046_hybrid_mode_engine_sequences_terminal_events():
    def rhs(t,y): return np.array([1.0])
    a = ModeDefinition("A", rhs, (Event("to-B", lambda t,y:y[0]-1.0, direction=1, action=EventAction.TERMINATE),))
    b = ModeDefinition("B", rhs, (Event("done", lambda t,y:y[0]-2.0, direction=1, action=EventAction.TERMINATE),))
    eng = HybridModeEngine({"A":a,"B":b}, lambda event,t,y: "B" if event=="to-B" else None,
                           ScipyIVPIntegrator(SolverConfig(rtol=1e-11,atol=1e-13,max_step=0.1)))
    result = eng.run((0.0,3.0), np.array([0.0]), "A")
    assert result.success
    assert [m.mode for m in result.modes] == ["A","B"]
    assert [e.name for e in result.events] == ["to-B","done"]
    assert abs(result.times[-1]-2.0) < 1e-9


def test_047_end_to_end_parachute_powered_descent_jettison_and_gear_contact():
    body, env = reference_world()
    terrain = RadialTerrain(body)
    schema = edl_6dof_schema()
    mp = ConstantMassProperties(np.diag([250.0,300.0,300.0]))

    chute_deploy = FirstOrderDeployable("parachute_deployment", 1.0, 1.5)
    chute = InflatingParachute(env, mp, 80.0, 1.5, chute_deploy)
    gear_deploy = FirstOrderDeployable("gear_deployment", 1.0, 0.5)
    leg = GearLeg(np.array([-0.5,0.0,0.0]), np.array([-2.0,0.0,0.0]), 80_000.0, 12_000.0)
    gear = LandingGearContact(terrain, mp, (leg,))

    guidance = VerticalDescentThrottle(
        env, terrain, exhaust_velocity=2000.0, mdot_exhaust=3.0,
        max_descent_speed=30.0, touchdown_speed=1.0, speed_slope=0.02, velocity_gain=1.0,
    )
    engine = GimballedRocketEngine(
        env, mp, exhaust_velocity=2000.0, mdot_exhaust=3.0,
        base_direction_b=np.array([1.0,0.0,0.0]), throttle=guidance, dry_mass=300.0,
    )
    mass_flow = MassFlowAggregator((engine,))

    chute_dyn = RigidBody6DOFDynamics(mp, body.gravity, (chute,))
    powered_dyn = RigidBody6DOFDynamics(mp, body.gravity, (engine,))
    contact_dyn = RigidBody6DOFDynamics(mp, body.gravity, (gear,))
    chute_rhs = DynamicsAssembler(schema, [chute_dyn, QuaternionKinematics(), chute_deploy]).rhs
    powered_rhs = DynamicsAssembler(schema, [powered_dyn, QuaternionKinematics(), gear_deploy, mass_flow]).rhs
    contact_rhs = DynamicsAssembler(schema, [contact_dyn, QuaternionKinematics(), gear_deploy]).rhs

    pos = schema.sl("position")
    vel = schema.sl("velocity")
    jettison = JettisonJump(schema, 20.0, {"parachute_deployment":0.0})
    to_power = Event(
        "powered-descent",
        lambda t,y: np.linalg.norm(y[pos])-body.radius-500.0,
        direction=-1.0, action=EventAction.TERMINATE, jump=jettison,
    )
    touchdown = Event(
        "touchdown",
        lambda t,y: gear.minimum_foot_agl(StateView(t,y,schema)),
        direction=-1.0, action=EventAction.TERMINATE,
    )
    compression_stop = Event(
        "compression-stop",
        lambda t,y: float(np.dot(y[vel], y[pos]/np.linalg.norm(y[pos]))),
        direction=1.0, action=EventAction.TERMINATE,
    )

    modes = {
        "parachute": ModeDefinition("parachute", chute_rhs, (to_power,)),
        "powered": ModeDefinition("powered", powered_rhs, (touchdown,)),
        "contact": ModeDefinition("contact", contact_rhs, (compression_stop,)),
    }
    def transition(event,t,y):
        return {"powered-descent":"powered", "touchdown":"contact"}.get(event)

    y0 = state_values(schema, body, altitude=3000.0, speed=-120.0, mass=500.0, chute=0.0, gear=0.0)
    result = HybridModeEngine(
        modes, transition,
        ScipyIVPIntegrator(SolverConfig(rtol=2e-8,atol=1e-9,max_step=0.5)),
    ).run((0.0,500.0), y0, "parachute")

    assert result.success
    assert [m.mode for m in result.modes] == ["parachute","powered","contact"]
    assert [e.name for e in result.events] == ["powered-descent","touchdown","compression-stop"]
    final = schema.unpack(result.states[-1])
    assert final["mass"] < 480.0  # 20 kg jettison plus terminal-descent propellant
    assert final["gear_deployment"] > 0.95
    assert abs(np.linalg.norm(final["attitude"])-1.0) < 1e-7
    # Compression-stop is first zero radial speed after touchdown.
    radial_speed = float(np.dot(final["velocity"], final["position"]/np.linalg.norm(final["position"])))
    assert abs(radial_speed) < 1e-5
    assert gear.evaluate(StateView(result.times[-1], result.states[-1], schema)).in_contact
