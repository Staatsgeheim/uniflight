import numpy as np

from uniflight import (
    GasSpecies, GasMixture, SphericalBody, IsothermalHydrostaticAtmosphere,
    PlanetaryEnvironment, compute_flow_state, ContinuumDrag, ConstantDragCoefficient,
    MachTableDragCoefficient, RocketEngine, core_3dof_schema, StateView,
)


def _world(surface_pressure=50_000.0, ceiling=None):
    gas = GasSpecies("X", 0.030, 30.0, 1.7e-5, 300.0, 120.0, 3.8e-10)
    mix = GasMixture((gas,), (1.0,))
    body = SphericalBody(mu=8e11, radius=5e5, rotation_vector_i=np.zeros(3), name="Testia")
    atm = IsothermalHydrostaticAtmosphere(surface_pressure, 250.0, mix, body.mu, body.radius, ceiling=ceiling)
    return body, PlanetaryEnvironment(body, atm)


def _state(position, velocity, mass=1000.0):
    schema = core_3dof_schema()
    y = schema.pack({"position":np.asarray(position,float), "velocity":np.asarray(velocity,float), "mass":mass})
    return StateView(0.0, y, schema)


def test_013_flow_state_uses_velocity_relative_to_atmosphere():
    body, env = _world()
    s = env.query(np.array([body.radius,0,0]), 0.0)
    flow = compute_flow_state(np.array([300.,0,0]), s, 2.0)
    assert flow.speed == 300.0
    assert abs(flow.dynamic_pressure - 0.5*s.atmosphere.density*300.0**2) < 1e-12
    assert flow.mach > 0
    assert flow.reynolds > 0
    assert flow.knudsen > 0


def test_014_continuum_drag_opposes_relative_velocity_and_matches_qcda():
    body, env = _world()
    state = _state([body.radius,0,0], [200.,0,0], 500.0)
    aero = ContinuumDrag(env, reference_area=3.0, reference_length=2.0,
                         coefficient=ConstantDragCoefficient(0.8))
    ev = aero.evaluate(state)
    expected_mag = ev.flow.dynamic_pressure * 3.0 * 0.8
    assert np.allclose(ev.force_i, [-expected_mag,0,0], rtol=1e-14, atol=1e-12)
    assert np.allclose(aero.acceleration(state), ev.force_i/500.0)


def test_015_mach_table_drag_interpolates_and_clamps():
    model = MachTableDragCoefficient(np.array([0.,1.,2.]), np.array([0.4,0.8,0.6]))
    body, env = _world()
    sample = env.query(np.array([body.radius,0,0]))
    a = sample.atmosphere.speed_of_sound
    f = compute_flow_state(np.array([1.5*a,0,0]), sample, 1.0)
    assert abs(model(f) - 0.7) < 1e-12


def test_016_rocket_pressure_thrust_and_mass_flow():
    body, env = _world(surface_pressure=50_000.0, ceiling=10_000.0)
    engine = RocketEngine(env, exhaust_velocity=1000.0, mdot_exhaust=1.0,
                          exit_area=0.1, exit_pressure=100_000.0,
                          direction_i=np.array([1.,0,0]), dry_mass=100.0)
    sea = _state([body.radius,0,0], [0,0,0], 500.0)
    vac = _state([body.radius+20_000.0,0,0], [0,0,0], 500.0)
    e0 = engine.evaluate(sea)
    ev = engine.evaluate(vac)
    assert abs(e0.thrust - 6_000.0) < 1e-9
    assert abs(ev.thrust - 11_000.0) < 1e-9
    assert engine.derivatives(sea)["mass"] == -1.0
    assert np.allclose(engine.acceleration(sea), [12.,0,0])
