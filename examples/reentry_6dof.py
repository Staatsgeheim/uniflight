"""Milestone D demonstration: post-deorbit 6-DOF entry on a fictional world."""
import numpy as np

from uniflight import (
    AeroCoefficients, ConstantAeroCoefficients, ConstantMassProperties,
    ConstantReferenceGeometry, ContinuumAerodynamics6DOF, DynamicsAssembler,
    Event, EventAction, FreeMolecularAerodynamics6DOF, GasMixture, GasSpecies,
    IsothermalHydrostaticAtmosphere, LumpedAblatingTPS,
    MachBlendedAeroCoefficients, MassFlowAggregator, NewtonianHypersonicCoefficients,
    PlanetaryEnvironment, QuaternionKinematics, RegimeBlendedAerodynamics6DOF,
    RigidBody6DOFDynamics, ScipyIVPIntegrator, SimulationEngine, SolverConfig,
    SphericalBody, StateView, SuttonGravesHeating, ThresholdDissociationCorrection,
    entry_6dof_schema, matrix_to_quat,
)


def main() -> None:
    gas = GasSpecies("X2", 0.029, 29.2, 1.75e-5, 300.0, 115.0, 3.7e-10)
    mix = GasMixture((gas,), (1.0,))
    body = SphericalBody(mu=3.0e12, radius=8.0e5, name="Nereid-D")
    atmosphere = IsothermalHydrostaticAtmosphere(
        18_000.0, 230.0, mix, body.mu, body.radius
    )
    environment = PlanetaryEnvironment(body, atmosphere)

    mass_properties = ConstantMassProperties(np.diag([600.0, 800.0, 800.0]))
    geometry = ConstantReferenceGeometry(5.0, 3.0, 2.5, 3.0, np.zeros(3))

    low_mach = ConstantAeroCoefficients(AeroCoefficients(cd=0.8))
    hypersonic = NewtonianHypersonicCoefficients(cd0=1.2, normal_force_scale=0.0)
    continuum_coeff = MachBlendedAeroCoefficients(low_mach, hypersonic, 2.0, 5.0)
    continuum = ContinuumAerodynamics6DOF(
        environment, geometry, continuum_coeff, mass_properties
    )
    free_molecular = FreeMolecularAerodynamics6DOF(
        environment, geometry,
        ConstantAeroCoefficients(AeroCoefficients(cd=2.2)), mass_properties,
    )
    aerodynamics = RegimeBlendedAerodynamics6DOF(continuum, free_molecular)

    chemistry = ThresholdDissociationCorrection(1100.0, 2200.0, max_heat_sink=0.2)
    heating = SuttonGravesHeating(
        environment, reference_length=3.0, nose_radius=1.0,
        coefficient=8e-4, chemistry=chemistry,
    )
    tps = LumpedAblatingTPS(
        heating, heated_area=5.0, thermal_mass=100.0, specific_heat=1000.0,
        emissivity=0.8, ablation_temperature=900.0,
        effective_heat_of_ablation=5e6,
    )
    mass_flow = MassFlowAggregator((tps,))

    schema = entry_6dof_schema()
    r0 = body.radius + 450_000.0
    v0 = 0.90 * np.sqrt(body.mu / r0)  # state immediately after a deorbit impulse
    # B+x initially follows velocity; B+z points approximately toward the body.
    R_ib = np.array([[0.0, 0.0, -1.0], [1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
    y0 = schema.pack({
        "position": np.array([r0, 0.0, 0.0]),
        "velocity": np.array([0.0, v0, 0.0]),
        "attitude": matrix_to_quat(R_ib),
        "angular_rate": np.zeros(3),
        "mass": 1000.0,
        "tps_temperature": 300.0,
        "heat_load": 0.0,
        "tps_mass": 80.0,
    })

    rigid_body = RigidBody6DOFDynamics(mass_properties, body.gravity, (aerodynamics,))
    rhs = DynamicsAssembler(
        schema, [rigid_body, QuaternionKinematics(), tps, mass_flow]
    ).rhs

    pos = schema.sl("position")
    stop = Event(
        "30-km-interface",
        lambda t, y: np.linalg.norm(y[pos]) - body.radius - 30_000.0,
        direction=-1.0,
        action=EventAction.TERMINATE,
    )
    result = SimulationEngine(
        rhs, ScipyIVPIntegrator(SolverConfig(rtol=3e-9, atol=1e-10, max_step=2.0))
    ).run((0.0, 3000.0), y0, (stop,))

    final = schema.unpack(result.states[-1])
    max_q = 0.0
    max_heat = 0.0
    max_temperature = 0.0
    max_dissociation = 0.0
    for t, y in zip(result.times, result.states):
        state = StateView(t, y, schema)
        ae = aerodynamics.evaluate(state)
        he = heating.evaluate(state)
        max_q = max(max_q, ae.continuum.flow.dynamic_pressure)
        max_heat = max(max_heat, he.total_heat_flux)
        max_temperature = max(max_temperature, float(state.get("tps_temperature")))
        max_dissociation = max(max_dissociation, he.chemistry.dissociation_fraction)

    start_state = StateView(result.times[0], result.states[0], schema)
    end_state = StateView(result.times[-1], result.states[-1], schema)
    start_aero = aerodynamics.evaluate(start_state)
    end_aero = aerodynamics.evaluate(end_state)

    print(f"World:                 {body.name}")
    print(f"Termination:           {result.terminated_by} at t={result.times[-1]:.1f} s")
    print(f"Initial speed:         {v0:.1f} m/s")
    print(f"Final speed:           {np.linalg.norm(final['velocity']):.1f} m/s")
    print(f"Flow regime:           {start_aero.regime} -> {end_aero.regime}")
    print(f"Knudsen number:        {start_aero.knudsen:.3g} -> {end_aero.knudsen:.3g}")
    print(f"Maximum dynamic q:     {max_q:.1f} Pa")
    print(f"Maximum heat flux:     {max_heat:.1f} W/m^2")
    print(f"Integrated heat load:  {final['heat_load'] / 1e6:.3f} MJ/m^2")
    print(f"Maximum TPS temp:      {max_temperature:.1f} K")
    print(f"TPS mass lost:         {80.0-final['tps_mass']:.3f} kg")
    print(f"Vehicle mass lost:     {1000.0-final['mass']:.3f} kg")
    print(f"Max chemistry fraction:{max_dissociation:.3f}")
    print(f"Quaternion norm:       {np.linalg.norm(final['attitude']):.12f}")


if __name__ == "__main__":
    main()
