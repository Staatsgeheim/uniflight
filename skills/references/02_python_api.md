# Python API guide

## Imports
Prefer importing from the module that owns the concept when writing library code. Top-level `uniflight` re-exports many public symbols, but module imports make dependencies clearer.

## Minimal propagation pattern
1. Choose schema.
2. Build initial state in schema order using schema helpers.
3. Construct body/environment/models.
4. Construct RHS/dynamics object.
5. Choose integrator.
6. Define events.
7. Run `SimulationEngine` or `MultiVehicleUniverseEngine`.
8. Extract named state/output metrics.

## Dynamics composition
Useful classes:
- `DynamicsAssembler`
- `TranslationalKinematics`
- `QuaternionKinematics`
- `IdealRocket`
- `RigidBody6DOFDynamics`
- `MassFlowAggregator`
- `ConstantMassProperties`, `AffineMassProperties`

## Frames
Use:
- `quat_normalize`
- `quat_multiply`
- `quat_to_matrix`
- `body_to_inertial_matrix`
- `inertial_to_body_matrix`
- `rotate_body_to_inertial`
- `rotate_inertial_to_body`
- `FrameGraph`

## Invariants
Use `specific_energy`, `specific_angular_momentum`, `quaternion_norm_error` in tests and diagnostics.

## Solver APIs
`SolverConfig` + `ScipyIVPIntegrator` for adaptive reference.
`FixedStepRK4Config` + `FixedStepRK4Integrator` for deterministic fixed-step work.

## Recommended coding style
- dependency-inject model objects;
- pure RHS functions where possible;
- no hidden global state;
- deterministic seeds;
- dataclasses for configuration/results;
- named metrics rather than positional output assumptions.
