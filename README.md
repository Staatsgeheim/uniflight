# UniFlight — Milestone C

Milestone C extends the planet-agnostic Milestone B atmospheric kernel to **coupled 6-DOF atmospheric flight** while retaining the earlier 3-DOF APIs and verification cases.

## Implemented in C

- Explicit canonical attitude convention: scalar-first quaternion maps **body → inertial** (`R_IB`)
- Aerospace body axes: **+x forward, +y right, +z down**
- Body-relative atmospheric flow and canonical `alpha`, `beta`
- Wind-to-body frame construction
- Rigid-body Euler rotational dynamics with arbitrary positive-definite inertia tensor
- Optional analytic inertia-rate term for mass-varying mass-property closures
- Common force/moment `Wrench` interface
- Continuum 6-DOF aerodynamic force and moment model
- Wind-axis drag/lift/side-force coefficients and body-axis roll/pitch/yaw moments
- Mach/alpha/beta trilinear aerodynamic coefficient database
- Linear stability-derivative reference aerodynamic model
- Aerodynamic-center offset moment about instantaneous CG
- Constant geometry and attitude/flow-dependent ellipsoid projected-area geometry
- Pressure-corrected body-mounted rocket engine
- Two-axis thrust-vector control (pitch/yaw gimbal)
- Engine mounting-arm moment about instantaneous CG
- Coupled translation, rotation, attitude, and mass depletion
- Full regression coverage for Milestones A and B

## Numerical dependencies

- Python >= 3.11
- NumPy >= 2.0
- SciPy >= 1.13
- pytest >= 8 for development tests

## Install

```bash
python -m pip install -e .
```

## Run verification

```bash
pytest
```

The project currently passes **26 tests**.

## Run the 6-DOF example

```bash
python examples/sixdof_atmospheric_flight.py
```

The example flies a powered vehicle through the atmosphere of a fictional low-gravity world. No Earth constants are embedded in the 6-DOF dynamics.

## Core 6-DOF assembly

```python
rigid_body = RigidBody6DOFDynamics(
    mass_properties,
    gravity=body.gravity,
    wrench_models=(engine, aerodynamics),
)

rhs = DynamicsAssembler(
    core_6dof_schema(),
    [rigid_body, QuaternionKinematics(), engine],
).rhs
```

`RigidBody6DOFDynamics` owns position, velocity, and angular-rate derivatives. `QuaternionKinematics` owns attitude. The propulsion model owns mass depletion. This preserves the one-writer-per-state invariant established in Milestone A.

## Compatibility note

Milestones A/B named the attitude field but did not yet use it to transform aerodynamic or propulsive vectors. Milestone C fixes its semantic convention explicitly: the stored quaternion produces `R_IB`, mapping body-frame components into inertial components. The state key remains `attitude`; only its frame metadata is corrected to `I<-B`.

## Intentionally deferred

The following remain behind the architecture's extension interfaces rather than being implemented in Milestone C:

- angular-rate aerodynamic derivatives / unsteady aerodynamics
- rarefied/transitional aerodynamics
- aerothermal heating, chemistry, and ablation
- flexible-body and slosh dynamics
- parachute/multi-body EDL models
- navigation/filtering and closed-loop GNC
- rigid contact/landing gear

These are later milestones and do not require a kernel redesign.
