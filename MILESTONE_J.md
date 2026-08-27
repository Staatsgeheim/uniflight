# Milestone J — Engineering Subsystem Dynamics

UniFlight 0.10.0 adds low-order engineering subsystem dynamics to the 0.9.0 multi-vehicle/multi-DOF runtime.  The purpose is to move beyond a perfectly rigid vehicle with idealized actuators while preserving the existing compositional state/RHS/wrench interfaces.

## Design rule

Every engineering subsystem couples through one or more existing UniFlight surfaces:

1. **state derivative owner** — continuous subsystem state;
2. **wrench model** — force/moment feedback into rigid-body dynamics;
3. **mass-flow source** — mass transfer;
4. **measurement or command provider** — sampled-data GNC/CSI coupling.

Subsystems do not mutate state from inside the ODE RHS and do not create hidden global singletons.

## Flexible-body modal dynamics

`ModalFlexibleBody` implements diagonal linear modal dynamics

`qdd + 2*zeta*wn*qd + wn^2*q = Q/m_modal`.

Modal states can be appended to any base schema with `augment_engineering_schema()`.  `TorqueToModalForce` maps control/body torque into generalized modal force using a participation matrix.  `FlexiblePointKinematics` maps modal coordinates to local translation/rotation at a station.

`FlexibleAttitudeRateSensor` wraps the existing attitude/rate sensor and adds local flexible rotation and modal angular rate.  This provides an explicit low-order control-structure-interaction path: control torque can excite modes and a sensor at a flexible station can feed the resulting motion back to GNC.

## Propellant slosh

`LinearSloshSubsystem` represents one or more transverse slosh masses:

`xdd + 2*zeta*wn*xd + wn^2*x = -a_base . e_mode`.

Spring/damper reaction loads are converted to an inertial force and body moment about the current CG.  `WrenchSpecificForceBodyProvider` allows selected non-slosh loads (for example engine thrust) to excite slosh without forming an algebraic loop through the slosh reaction itself.

The model is a low-order engineering closure, not CFD/free-surface fluid dynamics.

## Engine and actuator dynamics

`EngineTransient` adds normalized engine-power and engine-power-rate states using a bounded second-order response.  The object is itself a bounded throttle provider for `GimballedRocketEngine`, so ODE overshoot cannot command unphysical negative or >100% thrust.

`SecondOrderLimitedStateActuator` adds second-order servo dynamics with:

- hard position limits;
- velocity limits;
- acceleration limits;
- bounded plant output provider.

This is suitable for TVC, fins, valves, or other position actuators.

## Stateful landing gear

`DynamicLandingGear` replaces purely algebraic penalty gear when desired.  Each `DynamicGearLeg` has:

- nominal foot geometry;
- compression axis;
- stiffness/damping;
- effective strut mass;
- maximum compression;
- optional regularized Coulomb friction.

Compression and compression-rate are continuous states. Terrain penetration drives strut dynamics, and the resulting stateful strut load feeds the rigid-body wrench balance.

## Fault injection

`ScalarFaultSchedule` and `FaultWindow` provide deterministic time-window faults:

- gain error;
- bias;
- stuck value;
- dropout.

`FaultedScalarProvider` applies schedules to arbitrary scalar commands or state-dependent providers. `FaultedWrenchModel` applies a schedule to any wrench contribution. Faults are therefore reusable for engines, actuators, sensors/commands, and subsystem forces.

## Subsystem composition

`SubsystemBundle` groups derivative, wrench, and mass-flow contributions without introducing a new integrator.  `WrenchSpecificForceBodyProvider` is an explicit dependency/coupling adapter.

The intended assembly pattern is:

```python
schema = augment_engineering_schema(
    core_6dof_schema(),
    flex_modes=2,
    slosh_modes=1,
    gear_legs=0,
)

rigid = RigidBody6DOFDynamics(
    mass_properties,
    gravity=body.gravity,
    wrench_models=(engine, slosh),
)

rhs = DynamicsAssembler(schema, [
    rigid,
    QuaternionKinematics(),
    engine_transient,
    pitch_servo,
    flex,
    slosh,
    engine,      # owns canonical mass derivative
]).rhs
```

## Reference cases

`examples/engineering_subsystems.py` runs two deterministic fictional Nereid-J cases.

### Coupled powered subsystem case

A body-mounted engine is driven through a second-order engine transient and second-order pitch TVC. A scheduled 30% command loss occurs from 2.0–3.5 s. Engine moment excites two structural modes; gimballed thrust excites one transverse slosh mode; both structural/slosh states coexist with rigid 6-DOF translation/rotation and changing mass.

### Dynamic gear drop

A 6-DOF vehicle with one stateful strut begins with its nominal foot below the terrain surface. Strut compression develops dynamically, generates normal reaction load, then rebounds.

## Deliberate boundaries

Milestone J is intentionally low-order engineering dynamics. It does not yet claim:

- finite-element flexible structures;
- nonlinear large-deformation structures;
- CFD/free-surface slosh;
- detailed turbomachinery/chamber/feed-system models;
- hydraulic landing-gear thermodynamics;
- hardware-specific FDIR/flight-software models;
- validated component datasets.

Those can be supplied later behind the same subsystem interfaces.
