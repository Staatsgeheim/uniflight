# UniFlight — Milestone B

Milestone B extends the Milestone A celestial-body-agnostic dynamics kernel with the first atmospheric-flight physics layer. The package remains deliberately small and auditable: the kernel owns integration, state layout, event semantics, and derivative ownership; environmental and vehicle physics are interchangeable models.

## Added in Milestone B

- `SphericalBody`: arbitrary `mu`, radius, rotation vector, surface altitude/normal, and point-mass gravity.
- `GasSpecies` / `GasMixture`: species-resolved ideal-mixture thermodynamics, constant molar heat capacity, Sutherland species viscosity, Wilke mixture viscosity, sound speed, mass fractions, and an explicit mean-free-path closure.
- `AtmosphereModel`: protocol plus `VacuumAtmosphere` and `IsothermalHydrostaticAtmosphere`.
- Exact spherical hydrostatic reference atmosphere:

  `p(h) = p0 exp[ mu/(R T) (1/(R+h) - 1/R) ]`

  so the reference model does not assume constant Earth gravity or an Earth scale height.
- `PlanetaryEnvironment`: combines body gravity/geometry, atmospheric state, body rotation, and optional inertial wind velocity.
- `FlowState`: atmosphere-relative velocity, dynamic pressure, Mach, Reynolds, and Knudsen numbers.
- `ContinuumDrag`: point-mass continuum drag with pluggable coefficient model.
- `ConstantDragCoefficient` and `MachTableDragCoefficient`.
- `RocketEngine`: mass-flow and ambient-pressure-corrected thrust:

  `T = mdot * ve + (pe - pa) Ae`

  with throttle, dry-mass cutoff, fixed or callable thrust direction, and atmosphere-coupled ambient pressure.
- An end-to-end fictional-world atmospheric ascent example and verification case.

## Project layout

```text
src/uniflight/
    state.py          state schema, packing, immutable views
    frames.py         frame graph and rigid transforms
    gravity.py        point-mass gravity
    bodies.py         spherical arbitrary celestial body
    gases.py          species and gas-mixture closures
    atmosphere.py     atmosphere protocol/reference models
    environment.py    body + atmosphere + wind query
    flow.py           nondimensional/local flow state
    aerodynamics.py   continuum drag and Cd models
    propulsion.py     pressure-corrected rocket engine
    dynamics.py       RHS ownership/assembly
    events.py         guards, actions, jump maps
    integrators.py    SciPy IVP adapter
    simulation.py     hybrid segment/event runner
examples/
    suborbital_point_mass.py
    atmospheric_ascent.py
tests/
VERIFICATION.md
```

## Numeric conventions

All numeric kernel values use coherent SI units. Vectors ending in `_i` are expressed in the inertial frame. Atmosphere bulk velocity is returned inertially, so aerodynamic flow is always computed as

```text
V_rel = v_vehicle,I - v_fluid,I
```

where the fluid velocity may include body rotation and winds.

## Running

With NumPy, SciPy and pytest installed:

```bash
PYTHONPATH=src python -m pytest -q
PYTHONPATH=src python examples/atmospheric_ascent.py
```

For a local editable installation when build dependencies are already available:

```bash
python -m pip install -e . --no-build-isolation
python -m pytest -q
```

## Current fidelity boundary

Milestone B intentionally implements **continuum point-mass atmospheric flight**, not full entry physics. In particular:

- heat capacities are constant within a `GasSpecies`;
- composition is fixed within the reference hydrostatic atmosphere, although the atmosphere interface can be replaced;
- the mean-free-path mixture rule is an explicit engineering closure, not a collision-integral model;
- continuum aerodynamics currently produces drag only;
- there is no lift/side force or aerodynamic moment database yet;
- there is no rarefied-flow dispatch, chemical nonequilibrium, heating, ablation, parachute, or contact model yet;
- the pressure-thrust model assumes the configured exit state and scales its pressure term with throttle;
- engine burnout should be represented with an event guard at the desired dry mass for exact hybrid timing.

Those limitations correspond to later milestones rather than hidden assumptions in the kernel.

## Verification status

The suite contains **17 passing tests**: the eight Milestone A kernel tests plus nine Milestone B tests covering gas-mixture closures, spherical hydrostatics, atmosphere rotation/wind, vacuum ceiling behavior, flow quantities, drag magnitude/direction, Mach-table interpolation, ambient-pressure thrust, and integrated atmospheric ascent.

See `VERIFICATION.md` for details.
