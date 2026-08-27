from types import SimpleNamespace
import numpy as np

from uniflight import (
    AeroCoefficients, ConstantAeroCoefficients, ConstantMassProperties,
    ConstantReferenceGeometry, ContinuumAerodynamics6DOF, DynamicsAssembler,
    Event, EventAction, FreeMolecularAerodynamics6DOF, FrozenChemistry,
    GasMixture, GasSpecies, IsothermalHydrostaticAtmosphere,
    LumpedAblatingTPS, MachBlendedAeroCoefficients, MassFlowAggregator,
    NewtonianHypersonicCoefficients, PlanetaryEnvironment, QuaternionKinematics,
    RegimeBlendedAerodynamics6DOF, RigidBody6DOFDynamics, RocketEngine,
    ScipyIVPIntegrator, SimulationEngine, SolverConfig, SphericalBody,
    StateView, SuttonGravesHeating, ThresholdDissociationCorrection,
    entry_6dof_schema, matrix_to_quat, core_3dof_schema,
)


def reference_world():
    gas = GasSpecies("X2", 0.029, 29.2, 1.75e-5, 300.0, 115.0, 3.7e-10)
    mix = GasMixture((gas,), (1.0,))
    body = SphericalBody(mu=3.0e12, radius=8.0e5, name="Nereid-D")
    atm = IsothermalHydrostaticAtmosphere(
        18_000.0, 230.0, mix, body.mu, body.radius, ceiling=None
    )
    return body, PlanetaryEnvironment(body, atm)


def entry_state(schema, body, *, altitude=0.0, speed=1000.0, temperature=300.0, tps_mass=80.0):
    return schema.pack({
        "position": np.array([body.radius + altitude, 0.0, 0.0]),
        "velocity": np.array([0.0, speed, 0.0]),
        "attitude": np.array([1.0, 0.0, 0.0, 0.0]),
        "angular_rate": np.zeros(3),
        "mass": 1000.0,
        "tps_temperature": temperature,
        "heat_load": 0.0,
        "tps_mass": tps_mass,
    })


def test_027_entry_schema_extends_6dof_state_with_tps_states():
    schema = entry_6dof_schema()
    body, _ = reference_world()
    y = entry_state(schema, body)
    out = schema.unpack(y)
    assert schema.total_size == 17
    assert out["tps_temperature"] == 300.0
    assert out["heat_load"] == 0.0
    assert out["tps_mass"] == 80.0


def test_028_knudsen_dispatcher_has_smooth_logarithmic_bridge():
    body, env = reference_world()
    mp = ConstantMassProperties(np.diag([1.0, 1.0, 1.0]))
    geom = ConstantReferenceGeometry(1.0, 1.0, 1.0, 1.0, np.zeros(3))
    coeff = ConstantAeroCoefficients(AeroCoefficients(cd=1.0))
    cont = ContinuumAerodynamics6DOF(env, geom, coeff, mp)
    rare = FreeMolecularAerodynamics6DOF(env, geom, coeff, mp)
    dispatcher = RegimeBlendedAerodynamics6DOF(cont, rare, 0.01, 10.0)
    assert dispatcher.rarefied_fraction(1e-4) == 0.0
    assert dispatcher.rarefied_fraction(1e2) == 1.0
    assert abs(dispatcher.rarefied_fraction(np.sqrt(0.01*10.0)) - 0.5) < 1e-15


def test_029_free_molecular_reference_drag_points_opposite_relative_flow():
    body, env = reference_world()
    mp = ConstantMassProperties(np.eye(3))
    geom = ConstantReferenceGeometry(2.0, 2.0, 2.0, 2.0, np.zeros(3))
    fm = FreeMolecularAerodynamics6DOF(
        env, geom, ConstantAeroCoefficients(AeroCoefficients(cd=2.0)), mp
    )
    schema = entry_6dof_schema()
    y = entry_state(schema, body, altitude=450_000.0, speed=1400.0)
    st = StateView(0.0, y, schema)
    e = fm.evaluate(st)
    assert e.flow.dynamic_pressure > 0.0
    assert np.dot(e.force_i, e.flow.relative_velocity_i) < 0.0
    expected = e.flow.dynamic_pressure * geom.reference_area * 2.0
    assert abs(np.linalg.norm(e.force_i) - expected) / expected < 1e-12


def test_030_high_mach_coefficient_blend_reaches_declared_end_models():
    low = ConstantAeroCoefficients(AeroCoefficients(cd=0.4, cl=0.1))
    high = ConstantAeroCoefficients(AeroCoefficients(cd=1.4, cl=0.3))
    blend = MachBlendedAeroCoefficients(low, high, mach_start=2.0, mach_end=5.0)
    lo = blend(SimpleNamespace(mach=1.0))
    mid = blend(SimpleNamespace(mach=3.5))
    hi = blend(SimpleNamespace(mach=8.0))
    assert lo.cd == 0.4 and lo.cl == 0.1
    assert hi.cd == 1.4 and hi.cl == 0.3
    assert abs(mid.cd - 0.9) < 1e-15


def test_031_newtonian_reference_model_increases_drag_with_incidence():
    model = NewtonianHypersonicCoefficients(cd0=1.0, normal_force_scale=2.0)
    zero = model(SimpleNamespace(alpha=0.0, beta=0.0))
    angled = model(SimpleNamespace(alpha=np.deg2rad(30.0), beta=0.0))
    assert angled.cd > zero.cd
    assert angled.cl > 0.0


def test_032_sutton_graves_reference_heating_scales_as_velocity_cubed():
    body, env = reference_world()
    schema = entry_6dof_schema()
    model = SuttonGravesHeating(env, reference_length=3.0, nose_radius=1.0,
                                coefficient=1e-4, chemistry=FrozenChemistry())
    y1 = entry_state(schema, body, altitude=20_000.0, speed=500.0)
    y2 = entry_state(schema, body, altitude=20_000.0, speed=1000.0)
    q1 = model.evaluate(StateView(0.0, y1, schema)).convective_heat_flux
    q2 = model.evaluate(StateView(0.0, y2, schema)).convective_heat_flux
    assert q1 > 0.0
    assert abs(q2/q1 - 8.0) < 1e-12


def test_033_chemistry_hook_is_bounded_and_reduces_convective_heating():
    body, env = reference_world()
    schema = entry_6dof_schema()
    corr = ThresholdDissociationCorrection(600.0, 1600.0, max_heat_sink=0.3)
    model = SuttonGravesHeating(env, 3.0, 1.0, 1e-4, chemistry=corr)
    low = model.evaluate(StateView(0.0, entry_state(schema, body, altitude=20_000.0, speed=200.0), schema))
    high = model.evaluate(StateView(0.0, entry_state(schema, body, altitude=20_000.0, speed=1800.0), schema))
    assert 0.0 <= low.chemistry.dissociation_fraction <= 1.0
    assert 0.0 <= high.chemistry.dissociation_fraction <= 1.0
    assert high.chemistry.dissociation_fraction > low.chemistry.dissociation_fraction
    assert 0.7 <= high.chemistry.convective_heat_multiplier <= 1.0


def test_034_lumped_tps_accumulates_heat_and_heats_below_ablation_threshold():
    body, env = reference_world()
    schema = entry_6dof_schema()
    heat = SuttonGravesHeating(env, 3.0, 1.0, 2e-4)
    tps = LumpedAblatingTPS(
        heat, heated_area=4.0, thermal_mass=50.0, specific_heat=1000.0,
        emissivity=0.0, ablation_temperature=1200.0, effective_heat_of_ablation=5e6,
    )
    st = StateView(0.0, entry_state(schema, body, altitude=10_000.0, speed=1200.0, temperature=300.0), schema)
    d = tps.derivatives(st)
    assert d["tps_temperature"] > 0.0
    assert d["heat_load"] > 0.0
    assert d["tps_mass"] == 0.0


def test_035_ablation_couples_tps_mass_and_canonical_vehicle_mass():
    body, env = reference_world()
    schema = entry_6dof_schema()
    heat = SuttonGravesHeating(env, 3.0, 1.0, 5e-4)
    tps = LumpedAblatingTPS(
        heat, heated_area=5.0, thermal_mass=50.0, specific_heat=1000.0,
        emissivity=0.0, ablation_temperature=800.0, effective_heat_of_ablation=2e6,
    )
    st = StateView(0.0, entry_state(schema, body, altitude=5_000.0, speed=1500.0, temperature=850.0), schema)
    e = tps.evaluate(st)
    mass = MassFlowAggregator((tps,)).derivatives(st)["mass"]
    assert e.ablation_mass_rate > 0.0
    assert tps.derivatives(st)["tps_mass"] < 0.0
    assert mass == -e.ablation_mass_rate


def test_036_mass_flow_aggregator_can_combine_propulsion_with_other_sources():
    body, env = reference_world()
    schema = core_3dof_schema()
    y = schema.pack({"position": np.array([body.radius,0.0,0.0]),
                     "velocity": np.zeros(3), "mass": 1000.0})
    st = StateView(0.0, y, schema)
    rocket = RocketEngine(env, exhaust_velocity=1000.0, mdot_exhaust=2.0)
    class Vent:
        def mass_rate(self, state): return -0.25
    mdot = MassFlowAggregator((rocket, Vent())).derivatives(st)["mass"]
    assert mdot == -2.25


def test_037_end_to_end_deorbited_6dof_entry_crosses_rarefied_to_continuum_and_heats_tps():
    body, env = reference_world()
    mp = ConstantMassProperties(np.diag([600.0,800.0,800.0]))
    geom = ConstantReferenceGeometry(5.0,3.0,2.5,3.0,np.zeros(3))

    low = ConstantAeroCoefficients(AeroCoefficients(cd=0.8))
    high = NewtonianHypersonicCoefficients(cd0=1.2, normal_force_scale=0.0)
    continuum_coeff = MachBlendedAeroCoefficients(low, high, 2.0, 5.0)
    continuum = ContinuumAerodynamics6DOF(env, geom, continuum_coeff, mp)
    free_molecular = FreeMolecularAerodynamics6DOF(
        env, geom, ConstantAeroCoefficients(AeroCoefficients(cd=2.2)), mp
    )
    aero = RegimeBlendedAerodynamics6DOF(continuum, free_molecular)

    chemistry = ThresholdDissociationCorrection(1100.0,2200.0,max_heat_sink=0.2)
    heating = SuttonGravesHeating(env, 3.0, 1.0, 8e-4, chemistry=chemistry)
    tps = LumpedAblatingTPS(
        heating, heated_area=5.0, thermal_mass=100.0, specific_heat=1000.0,
        emissivity=0.8, ablation_temperature=900.0, effective_heat_of_ablation=5e6,
    )
    mass_flow = MassFlowAggregator((tps,))

    schema = entry_6dof_schema()
    r0 = body.radius + 450_000.0
    v_circular = np.sqrt(body.mu/r0)
    v0 = 0.90*v_circular  # post-deorbit state: apoapsis with atmospheric periapsis
    R_ib = np.array([[0.0,0.0,-1.0],[1.0,0.0,0.0],[0.0,-1.0,0.0]])
    y0 = schema.pack({
        "position": np.array([r0,0.0,0.0]), "velocity": np.array([0.0,v0,0.0]),
        "attitude": matrix_to_quat(R_ib), "angular_rate": np.zeros(3), "mass": 1000.0,
        "tps_temperature": 300.0, "heat_load": 0.0, "tps_mass": 80.0,
    })

    dynamics = RigidBody6DOFDynamics(mp, body.gravity, (aero,))
    rhs = DynamicsAssembler(schema, [dynamics, QuaternionKinematics(), tps, mass_flow]).rhs
    pos = schema.sl("position")
    target = Event(
        "entry-to-30km",
        lambda t,y: np.linalg.norm(y[pos])-body.radius-30_000.0,
        direction=-1.0, action=EventAction.TERMINATE,
    )
    result = SimulationEngine(
        rhs, ScipyIVPIntegrator(SolverConfig(rtol=3e-9,atol=1e-10,max_step=2.0))
    ).run((0.0,3000.0), y0, (target,))

    assert result.success and result.terminated_by == "entry-to-30km"
    final = schema.unpack(result.states[-1])
    assert abs(np.linalg.norm(final["position"])-body.radius-30_000.0) < 1e-5
    assert np.linalg.norm(final["velocity"]) < v0
    assert final["heat_load"] > 1e6
    assert final["tps_mass"] < 80.0
    assert abs((1000.0-final["mass"]) - (80.0-final["tps_mass"])) < 1e-6
    assert abs(np.linalg.norm(final["attitude"])-1.0) < 2e-8

    start = StateView(result.times[0], result.states[0], schema)
    finish = StateView(result.times[-1], result.states[-1], schema)
    assert aero.evaluate(start).rarefied_fraction > 0.99
    assert aero.evaluate(finish).rarefied_fraction < 1e-6
    assert heating.evaluate(start).chemistry.dissociation_fraction > 0.0
    assert np.all(np.isfinite(result.states))
