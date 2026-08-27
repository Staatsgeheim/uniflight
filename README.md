# UniFlight — Milestone D

Milestone D extends the Milestone C planet-agnostic 6-DOF atmospheric-flight kernel into a first **entry / re-entry physics stack**. The goal remains architectural: implement physically meaningful reference closures behind stable interfaces so higher-fidelity CFD, DSMC, chemistry, radiation, and TPS models can later replace them without redesigning the trusted dynamics kernel.

## Implemented in D

- `entry_6dof_schema()` with thermal-protection state:
  - TPS temperature
  - integrated heat load
  - TPS remaining mass
- Smooth **continuum → transitional → free-molecular** aerodynamic dispatch in log10(Kn)
- Replaceable free-molecular 6-DOF coefficient closure
- Low-order Newtonian-inspired hypersonic coefficient model
- Smooth low-/high-Mach coefficient blending
- Generalized Sutton–Graves stagnation-point convective heating
  - no embedded Earth constant: the heating coefficient is supplied by the atmosphere/model
- Optional empirical radiative-heating hook
- Thermochemical correction interface
- Frozen-chemistry closure
- Smooth dissociation/ionization reference hook for proving chemistry coupling
- Lumped radiating thermal-protection model
- Heat-load integration
- Ablation/recession mass-loss closure
- Ablation coupled back into canonical vehicle mass
- `MassFlowAggregator` allowing propulsion, venting, ablation, ingestion, etc. to share one canonical mass derivative owner
- `RocketEngine` and `GimballedRocketEngine` now expose signed `mass_rate()` in addition to their backward-compatible derivative API
- Full regression coverage for Milestones A, B, and C
- End-to-end fictional-world **post-deorbit → free-molecular → transitional → continuum entry** example

## Important fidelity note

The new chemistry, free-molecular, hypersonic, and thermal closures are **reference implementations**, not claims of universally accurate entry physics. They establish the software contracts and the coupled state flow. In production work they should be replaced by validated atmosphere-specific correlations, experimental/CFD databases, DSMC/Sentman models, finite-rate chemistry, radiation solvers, and material-response/TPS models as mission fidelity requires.

## Numerical dependencies

- Python >= 3.11
- NumPy >= 2.0
- SciPy >= 1.13
- pytest >= 8 for development tests

## Install

```bash
python -m pip install -e .
```

For an offline environment with the build toolchain already installed:

```bash
python -m pip install -e . --no-build-isolation
```

## Run verification

```bash
PYTHONPATH=src pytest -q
```

The project currently passes **37 tests**.

## Run the re-entry example

```bash
PYTHONPATH=src python examples/reentry_6dof.py
```

The reference example uses the fictional world **Nereid-D**. It begins immediately after a deorbit impulse at 450 km altitude and terminates at 30 km. In the current reference configuration it:

- begins at Kn ≈ 75 in the free-molecular branch
- ends at Kn ≈ 7.6e-7 in continuum flow
- decelerates from about 1394 m/s to 313 m/s
- reaches about 2.1 kPa maximum dynamic pressure
- reaches about 79 kW/m² maximum reference heat flux
- accumulates about 21.7 MJ/m² heat load
- reaches the 900 K reference ablation threshold
- loses about 2.8 kg of TPS, identically reflected in total vehicle mass

No Earth radius, mass, gravity, atmospheric pressure, or composition is embedded in the dynamics.

## Core Milestone D assembly

```python
continuum = ContinuumAerodynamics6DOF(
    environment, geometry, continuum_coefficients, mass_properties
)
free_molecular = FreeMolecularAerodynamics6DOF(
    environment, geometry, free_molecular_coefficients, mass_properties
)
aerodynamics = RegimeBlendedAerodynamics6DOF(
    continuum, free_molecular,
    continuum_knudsen=0.01,
    free_molecular_knudsen=10.0,
)

heating = SuttonGravesHeating(
    environment,
    reference_length=3.0,
    nose_radius=1.0,
    coefficient=user_supplied_k,
    chemistry=chemistry_model,
)

tps = LumpedAblatingTPS(
    heating,
    heated_area=5.0,
    thermal_mass=100.0,
    specific_heat=1000.0,
    emissivity=0.8,
    ablation_temperature=900.0,
    effective_heat_of_ablation=5e6,
)

mass_flow = MassFlowAggregator((tps,))

rigid_body = RigidBody6DOFDynamics(
    mass_properties,
    gravity=body.gravity,
    wrench_models=(aerodynamics,),
)

rhs = DynamicsAssembler(
    entry_6dof_schema(),
    [rigid_body, QuaternionKinematics(), tps, mass_flow],
).rhs
```

## Mass ownership in D

The one-writer-per-state invariant remains intact. A TPS model owns its own `tps_mass`, `tps_temperature`, and `heat_load` derivatives. The canonical vehicle `mass` state has exactly one owner: `MassFlowAggregator`. Individual subsystems expose signed `mass_rate(state)` contributions.

This allows, for example:

```python
mass_flow = MassFlowAggregator((rocket_engine, tps, vent_model))
```

without allowing several subsystems to overwrite `dm/dt` independently.

## Knudsen transition convention

The reference dispatcher uses:

- continuum branch: Kn <= 0.01
- free-molecular branch: Kn >= 10
- smooth cubic blend in `log10(Kn)` between those limits

This is explicitly a numerical bridging law. It is intentionally replaceable by a validated bridging relation or a direct regime database.

## Intentionally deferred

Milestone D does **not** yet implement:

- finite-rate species kinetics as integrated state equations
- multi-temperature vibrational/electron energy equations
- shock-layer CFD
- DSMC particle simulation
- validated radiative transport
- multi-node / through-thickness TPS conduction
- geometry recession feeding aerodynamic shape changes
- parachutes / ballutes / multi-body EDL
- bank-angle entry guidance and closed-loop GNC
- rigid landing contact

Those remain compatible with the architecture and are natural later milestones.
