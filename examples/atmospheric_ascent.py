"""Milestone B example: pressure-corrected rocket ascent through a fictional atmosphere."""

from __future__ import annotations
import numpy as np

from uniflight import (
    GasSpecies, GasMixture, SphericalBody, IsothermalHydrostaticAtmosphere,
    PlanetaryEnvironment, RocketEngine, ContinuumDrag, MachTableDragCoefficient,
    core_3dof_schema, StateView, TranslationalKinematics, DynamicsAssembler,
    SimulationEngine, ScipyIVPIntegrator, SolverConfig, Event, EventAction,
)


def main() -> None:
    # Deliberately fictional constants: no Earth-specific atmosphere or gravity is embedded.
    gas = GasSpecies(
        name="A2", molar_mass=0.028, cp_molar=29.0,
        viscosity_ref=1.70e-5, viscosity_ref_temperature=300.0,
        sutherland_constant=115.0, collision_diameter=3.7e-10,
    )
    mixture = GasMixture((gas,), (1.0,))

    body = SphericalBody(
        mu=5.0e11,
        radius=5.0e5,
        rotation_vector_i=np.array([0.0, 0.0, 1.0e-4]),
        name="Asteria",
    )
    atmosphere = IsothermalHydrostaticAtmosphere(
        surface_pressure=20_000.0,
        temperature=250.0,
        mixture=mixture,
        body_mu=body.mu,
        reference_radius=body.radius,
        ceiling=150_000.0,
    )
    environment = PlanetaryEnvironment(body, atmosphere)

    engine = RocketEngine(
        environment,
        exhaust_velocity=1200.0,
        mdot_exhaust=5.0,
        exit_area=0.02,
        exit_pressure=20_000.0,
        direction_i=np.array([1.0, 0.0, 0.0]),
        dry_mass=700.0,
    )
    aero = ContinuumDrag(
        environment,
        reference_area=1.5,
        reference_length=2.0,
        coefficient=MachTableDragCoefficient(
            np.array([0.0, 0.8, 1.0, 1.5, 3.0, 8.0]),
            np.array([0.40, 0.43, 0.62, 0.55, 0.47, 0.44]),
        ),
    )

    schema = core_3dof_schema()
    y0 = schema.pack({
        "position": np.array([body.radius, 0.0, 0.0]),
        "velocity": np.zeros(3),
        "mass": 1000.0,
    })

    dynamics = DynamicsAssembler(schema, [
        TranslationalKinematics(body.gravity, (engine, aero)),
        engine,
    ])

    mass_sl = schema.sl("mass")
    burnout = Event(
        "burnout",
        lambda t, y: y[mass_sl][0] - 700.0,
        direction=-1,
        priority=100,
        action=EventAction.TERMINATE,
    )

    solver = ScipyIVPIntegrator(SolverConfig(rtol=3e-10, atol=1e-9, max_step=0.2))
    result = SimulationEngine(dynamics.rhs, solver).run((0.0, 100.0), y0, [burnout])

    final = schema.unpack(result.states[-1])
    altitude = np.linalg.norm(final["position"]) - body.radius
    speed = np.linalg.norm(final["velocity"])

    max_q = 0.0
    max_mach = 0.0
    for t, y in zip(result.times, result.states):
        ev = aero.evaluate(StateView(t, y, schema))
        max_q = max(max_q, ev.flow.dynamic_pressure)
        max_mach = max(max_mach, ev.flow.mach)

    print(f"body:              {body.name}")
    print(f"terminated by:     {result.terminated_by}")
    print(f"burnout time:      {result.times[-1]:.3f} s")
    print(f"burnout altitude:  {altitude/1000:.3f} km")
    print(f"burnout speed:     {speed:.3f} m/s")
    print(f"remaining mass:    {final['mass']:.3f} kg")
    print(f"maximum q:         {max_q:.3f} Pa")
    print(f"maximum Mach:      {max_mach:.3f}")


if __name__ == "__main__":
    main()
