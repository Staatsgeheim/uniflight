# Architecture and design contracts

## Core execution path
`StateSchema` defines the vector layout. Models read state through named fields. Environment/model providers produce forces, moments, mass flow, measurements, or subsystem derivatives. Dynamics assembles the RHS. An integrator propagates a segment. Event logic terminates/splits segments. The universe coordinates multiple vehicles. MDL compiles declarative documents into the same runtime objects.

## State schemas
Built-ins include:
- `core_3dof_schema`
- `core_6dof_schema`
- `entry_6dof_schema`
- `edl_6dof_schema`
- `gnc_edl_6dof_schema`
- `augment_engineering_schema`
- `engineering_6dof_schema`

Do not assume every vehicle has the same schema. Multi-vehicle histories are schema-tagged segments.

## Force/moment abstraction
Use `Wrench`/`WrenchModel` for body/inertial force and moment contributions. Keep propulsion, aero, contact, parachutes, slosh reactions, etc. composable.

## Environment abstraction
`PlanetaryEnvironment` binds gravity, atmosphere, rotation/winds and related sampling. Environment should be queried from position/time rather than copied into unrelated models.

## Hybrid architecture
Continuous propagation and discrete transitions are separate. An event consists of a guard plus semantics/action. Universe mutations may remove/spawn/replace vehicles or switch phases.

## Extensibility
Use public model interfaces first; use `MissionRegistry`/Plugin API for MDL-visible extensions. Avoid adding special-case branches to the mission compiler for one mission unless the capability is truly core.

## Fidelity boundary
Current 1.0.4 core is broad but not a universal highest-fidelity solver. Notable boundaries include point-mass built-in gravity, compliant contact rather than a general complementarity DAE solver, engineering-order flex/slosh/engine/gear, and surrogate chemistry options. Higher-fidelity implementations belong behind the same interfaces.
