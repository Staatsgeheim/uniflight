import numpy as np

from uniflight import (
    GasSpecies, GasMixture, SphericalBody, IsothermalHydrostaticAtmosphere,
    PlanetaryEnvironment, RocketEngine, ContinuumDrag, ConstantDragCoefficient,
    core_3dof_schema, TranslationalKinematics, DynamicsAssembler,
    SimulationEngine, ScipyIVPIntegrator, SolverConfig, Event, EventAction,
)


def _simulate(with_drag: bool):
    gas = GasSpecies("Z", 0.028, 29.0, 1.7e-5, 300.0, 115.0, 3.7e-10)
    mix = GasMixture((gas,), (1.0,))
    body = SphericalBody(mu=5.0e11, radius=5.0e5, name="Asteria")  # g_surface = 2 m/s^2
    atmosphere = IsothermalHydrostaticAtmosphere(
        surface_pressure=20_000.0, temperature=250.0, mixture=mix,
        body_mu=body.mu, reference_radius=body.radius, ceiling=150_000.0,
    )
    env = PlanetaryEnvironment(body, atmosphere)
    engine = RocketEngine(
        env, exhaust_velocity=1200.0, mdot_exhaust=5.0,
        exit_area=0.02, exit_pressure=20_000.0,
        direction_i=np.array([1.,0,0]), dry_mass=700.0,
    )
    accel = [engine]
    if with_drag:
        accel.append(ContinuumDrag(env, 1.5, 2.0, ConstantDragCoefficient(0.6)))

    schema = core_3dof_schema()
    y0 = schema.pack({
        "position":np.array([body.radius,0,0]),
        "velocity":np.zeros(3),
        "mass":1000.0,
    })
    asm = DynamicsAssembler(schema, [
        TranslationalKinematics(body.gravity, tuple(accel)),
        engine,
    ])
    msl = schema.sl("mass")
    burnout = Event("burnout", lambda t,y: y[msl][0]-700.0,
                    direction=-1, priority=100, action=EventAction.TERMINATE)
    integ = ScipyIVPIntegrator(SolverConfig(rtol=3e-11, atol=1e-10, max_step=0.1))
    result = SimulationEngine(asm.rhs, integ).run((0.0, 100.0), y0, [burnout])
    return body, schema, result


def test_017_end_to_end_atmospheric_ascent_drag_reduces_performance():
    body, schema, drag = _simulate(True)
    _, _, vacuumish = _simulate(False)
    sd = schema.unpack(drag.states[-1])
    sv = schema.unpack(vacuumish.states[-1])
    assert drag.success and vacuumish.success
    assert drag.terminated_by == vacuumish.terminated_by == "burnout"
    assert abs(drag.times[-1] - 60.0) < 1e-8
    assert abs(sd["mass"] - 700.0) < 1e-7
    assert sd["velocity"][0] > 0.0
    assert sd["velocity"][0] < sv["velocity"][0]
    assert np.linalg.norm(sd["position"]) - body.radius < np.linalg.norm(sv["position"]) - body.radius
