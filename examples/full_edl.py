"""Milestone E demonstration: parachute -> jettison -> powered descent -> gear contact."""
import numpy as np

from uniflight import (
    ConstantMassProperties, DynamicsAssembler, Event, EventAction, FirstOrderDeployable,
    GasMixture, GasSpecies, GimballedRocketEngine, GearLeg, HybridModeEngine,
    InflatingParachute, IsothermalHydrostaticAtmosphere, JettisonJump,
    LandingGearContact, MassFlowAggregator, ModeDefinition, PlanetaryEnvironment,
    QuaternionKinematics, RadialTerrain, RigidBody6DOFDynamics, ScipyIVPIntegrator,
    SolverConfig, SphericalBody, StateView, VerticalDescentThrottle, edl_6dof_schema,
)


def main() -> None:
    gas = GasSpecies("X2", 0.029, 29.2, 1.75e-5, 300.0, 115.0, 3.7e-10)
    mixture = GasMixture((gas,), (1.0,))
    body = SphericalBody(mu=3.0e12, radius=8.0e5, name="Nereid-E")
    atmosphere = IsothermalHydrostaticAtmosphere(
        18_000.0, 230.0, mixture, body.mu, body.radius
    )
    environment = PlanetaryEnvironment(body, atmosphere)
    terrain = RadialTerrain(body)

    schema = edl_6dof_schema()
    mass_properties = ConstantMassProperties(np.diag([250.0, 300.0, 300.0]))

    chute_deploy = FirstOrderDeployable("parachute_deployment", 1.0, 1.5)
    parachute = InflatingParachute(
        environment, mass_properties, maximum_area=80.0, drag_coefficient=1.5,
        deployment=chute_deploy,
    )

    gear_deploy = FirstOrderDeployable("gear_deployment", 1.0, 0.5)
    leg = GearLeg(
        stowed_foot_b=np.array([-0.5, 0.0, 0.0]),
        deployed_foot_b=np.array([-2.0, 0.0, 0.0]),
        stiffness=80_000.0, damping=12_000.0,
    )
    landing_gear = LandingGearContact(terrain, mass_properties, (leg,))

    guidance = VerticalDescentThrottle(
        environment, terrain,
        exhaust_velocity=2000.0, mdot_exhaust=3.0,
        max_descent_speed=30.0, touchdown_speed=1.0,
        speed_slope=0.02, velocity_gain=1.0,
    )
    engine = GimballedRocketEngine(
        environment, mass_properties,
        exhaust_velocity=2000.0, mdot_exhaust=3.0,
        base_direction_b=np.array([1.0, 0.0, 0.0]),
        throttle=guidance, dry_mass=300.0,
    )
    mass_flow = MassFlowAggregator((engine,))

    parachute_dynamics = RigidBody6DOFDynamics(
        mass_properties, body.gravity, (parachute,)
    )
    powered_dynamics = RigidBody6DOFDynamics(
        mass_properties, body.gravity, (engine,)
    )
    contact_dynamics = RigidBody6DOFDynamics(
        mass_properties, body.gravity, (engine, landing_gear)
    )

    parachute_rhs = DynamicsAssembler(
        schema, [parachute_dynamics, QuaternionKinematics(), chute_deploy]
    ).rhs
    powered_rhs = DynamicsAssembler(
        schema, [powered_dynamics, QuaternionKinematics(), gear_deploy, mass_flow]
    ).rhs
    contact_rhs = DynamicsAssembler(
        schema, [contact_dynamics, QuaternionKinematics(), gear_deploy]
    ).rhs

    pos = schema.sl("position")
    vel = schema.sl("velocity")
    jettison = JettisonJump(schema, 20.0, {"parachute_deployment": 0.0})
    to_power = Event(
        "powered-descent",
        lambda t, y: np.linalg.norm(y[pos]) - body.radius - 500.0,
        direction=-1.0, action=EventAction.TERMINATE, jump=jettison,
    )
    touchdown = Event(
        "touchdown",
        lambda t, y: landing_gear.minimum_foot_agl(StateView(t, y, schema)),
        direction=-1.0, action=EventAction.TERMINATE,
    )
    compression_stop = Event(
        "compression-stop",
        lambda t, y: float(np.dot(y[vel], y[pos] / np.linalg.norm(y[pos]))),
        direction=1.0, action=EventAction.TERMINATE,
    )

    modes = {
        "parachute": ModeDefinition("parachute", parachute_rhs, (to_power,)),
        "powered": ModeDefinition("powered", powered_rhs, (touchdown,)),
        "contact": ModeDefinition("contact", contact_rhs, (compression_stop,)),
    }

    transition_map = {"powered-descent": "powered", "touchdown": "contact"}
    mission = HybridModeEngine(
        modes,
        lambda event, time, y: transition_map.get(event),
        ScipyIVPIntegrator(SolverConfig(rtol=2e-8, atol=1e-9, max_step=0.5)),
    )

    y0 = schema.pack({
        "position": np.array([body.radius + 3000.0, 0.0, 0.0]),
        "velocity": np.array([-120.0, 0.0, 0.0]),
        "attitude": np.array([1.0, 0.0, 0.0, 0.0]),
        "angular_rate": np.zeros(3),
        "mass": 500.0,
        "tps_temperature": 300.0,
        "heat_load": 0.0,
        "tps_mass": 0.0,
        "parachute_deployment": 0.0,
        "gear_deployment": 0.0,
    })

    result = mission.run((0.0, 500.0), y0, "parachute")
    final = schema.unpack(result.states[-1])

    print(f"World:                   {body.name}")
    print(f"Success:                 {result.success}")
    print(f"Mode sequence:           {' -> '.join(m.mode for m in result.modes)}")
    for e in result.events:
        print(f"Event {e.name:18s} t={e.time:8.3f} s")
    print(f"Final mission time:      {result.times[-1]:.3f} s")
    print(f"Final center altitude:   {np.linalg.norm(final['position'])-body.radius:.3f} m")
    print(f"Final radial speed:      {np.dot(final['velocity'], final['position']/np.linalg.norm(final['position'])):.6f} m/s")
    print(f"Final mass:              {final['mass']:.3f} kg")
    print(f"Gear deployment:         {final['gear_deployment']:.6f}")
    print(f"Quaternion norm:         {np.linalg.norm(final['attitude']):.12f}")
    contact = landing_gear.evaluate(StateView(result.times[-1], result.states[-1], schema))
    print(f"Gear in contact:         {contact.in_contact}")
    print(f"Contact normal force:    {sum(x.normal_force for x in contact.legs):.1f} N")


if __name__ == "__main__":
    main()
