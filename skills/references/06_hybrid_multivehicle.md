# Hybrid events, DOF transitions, and multi-vehicle runtime

## Single-vehicle events
`Event` carries guard/action semantics; `SimulationEngine` propagates until roots and applies ordered events.

### 1.0.2 correctness requirements
- simultaneous adaptive roots are collected at the same physical root time;
- tied events are ordered by priority;
- continuing non-jump events must not enter zero-time cycles.

Whenever modifying event code, test these three behaviors explicitly.

## Modes
`HybridModeEngine`, `ModeDefinition`, `ModeInterval` support discrete mode sequencing.

## DOF changes
`DOFTransition`, `promote_3dof_to_6dof`, `demote_6dof_to_3dof`.
3→6 requires an attitude/angular-rate policy; 6→3 projects rotational state out.

## Universe
`VehicleSpec` describes a vehicle. `MultiVehicleUniverseEngine` coordinates active vehicles on one global event timeline. `UniverseMutation` changes topology.

## Separation
Point-mass:
- `separate_two_body`

Rigid:
- `separate_two_rigid_bodies`
- `RigidChildTemplate`
- `RigidSeparationHandler`

Verify:
- mass conservation;
- parent/daughter COM consistency;
- linear momentum;
- angular momentum;
- inherited rotational velocity from COM offsets;
- intended relative separation impulse/velocity.

## History
Use `VehicleTrajectorySegment` and `UniverseResult`; never force all vehicles into one rectangular state matrix.

## Event test template
For every topology event, assert:
- pre-event active IDs;
- root time;
- occurrence ordering;
- post-event active IDs;
- daughter schemas/modes;
- conserved quantities;
- continuation to final time.
