"""Milestone C demonstration: coupled 6-DOF atmospheric flight on a fictional world."""
import numpy as np

from uniflight import (
    GasSpecies, GasMixture, SphericalBody, IsothermalHydrostaticAtmosphere,
    PlanetaryEnvironment, ConstantMassProperties, GimballedRocketEngine,
    ConstantReferenceGeometry, LinearStabilityAerodynamics,
    ContinuumAerodynamics6DOF, core_6dof_schema, RigidBody6DOFDynamics,
    QuaternionKinematics, DynamicsAssembler, SimulationEngine,
    ScipyIVPIntegrator, SolverConfig, StateView,
)


def main() -> None:
    gas = GasSpecies("Q", 0.029, 29.2, 1.75e-5, 300.0, 115.0, 3.7e-10)
    mixture = GasMixture((gas,), (1.0,))

    body = SphericalBody(mu=5.0e11, radius=5.0e5, name="Asteria-C")
    atmosphere = IsothermalHydrostaticAtmosphere(
        surface_pressure=18_000.0,
        temperature=245.0,
        mixture=mixture,
        body_mu=body.mu,
        reference_radius=body.radius,
        ceiling=120_000.0,
    )
    environment = PlanetaryEnvironment(body, atmosphere)

    mass_properties = ConstantMassProperties(np.diag([400.0, 800.0, 800.0]))
    engine = GimballedRocketEngine(
        environment=environment,
        mass_properties=mass_properties,
        exhaust_velocity=1500.0,
        mdot_exhaust=2.0,
        mount_position_b=np.array([-1.5, 0.0, 0.0]),
        pitch_gimbal=0.001,
        dry_mass=850.0,
    )
    geometry = ConstantReferenceGeometry(
        reference_area=2.0,
        reference_length=3.0,
        reference_span=2.0,
        reference_chord=3.0,
        aerodynamic_center_b=np.array([0.4, 0.0, 0.0]),
    )
    aero = ContinuumAerodynamics6DOF(
        environment,
        geometry,
        LinearStabilityAerodynamics(
            cd0=0.35,
            cd_alpha2=0.8,
            cl_alpha=1.2,
            cy_beta=-0.5,
            c_pitch_alpha=-0.8,
            c_yaw_beta=0.4,
        ),
        mass_properties,
    )

    schema = core_6dof_schema()
    y0 = schema.pack({
        "position": np.array([body.radius, 0.0, 0.0]),
        "velocity": np.zeros(3),
        "attitude": np.array([1.0, 0.0, 0.0, 0.0]),
        "angular_rate": np.zeros(3),
        "mass": 1000.0,
    })
    rigid_body = RigidBody6DOFDynamics(
        mass_properties, gravity=body.gravity, wrench_models=(engine, aero)
    )
    rhs = DynamicsAssembler(schema, [rigid_body, QuaternionKinematics(), engine]).rhs
    result = SimulationEngine(
        rhs, ScipyIVPIntegrator(SolverConfig(rtol=3e-10, atol=1e-11, max_step=0.02))
    ).run((0.0, 15.0), y0)

    final = schema.unpack(result.states[-1])
    final_view = StateView(result.times[-1], result.states[-1], schema)
    aero_final = aero.evaluate(final_view)
    altitude = np.linalg.norm(final["position"]) - body.radius

    print(f"world:          {body.name}")
    print(f"time:           {result.times[-1]:.3f} s")
    print(f"altitude:       {altitude:.3f} m")
    print(f"speed:          {np.linalg.norm(final['velocity']):.3f} m/s")
    print(f"mass:           {final['mass']:.3f} kg")
    print(f"angular rate:   {np.linalg.norm(final['angular_rate']):.6f} rad/s")
    print(f"quaternion norm:{np.linalg.norm(final['attitude']):.12f}")
    print(f"Mach:           {aero_final.flow.mach:.4f}")
    print(f"alpha:          {np.degrees(aero_final.flow.alpha):.4f} deg")
    print(f"beta:           {np.degrees(aero_final.flow.beta):.4f} deg")
    print(f"dynamic pressure:{aero_final.flow.dynamic_pressure:.3f} Pa")


if __name__ == "__main__":
    main()
