# Milestone I — True Multi-Vehicle / Multi-DOF Runtime

UniFlight 0.9.0 generalizes the single-vehicle hybrid mission engine into a global event-synchronized universe containing an arbitrary active set of heterogeneous vehicles.

## Objective

Milestone I closes a major gap toward POST2-style workflow: vehicle count, state dimension, local model set, and DOF are no longer fixed for the whole mission.

The runtime now supports:

- arbitrary active vehicle count;
- dynamic spawn/remove/replace operations at hybrid events;
- different state schemas and state dimensions per vehicle;
- different RHS/model closures per vehicle;
- different integrators per vehicle;
- per-vehicle environment/GNC/model context;
- globally synchronized event time;
- deterministic event priority across vehicles;
- schema-tagged piecewise trajectory history;
- 3-DOF <-> 6-DOF configuration changes;
- rigid-body two-daughter separation with linear and angular momentum conservation.

## Global universe semantics

For active vehicles `i=1...N`, each vehicle has its own system

`dX_i/dt = F_i(t, X_i)`.

The universe propagates every active vehicle from the current global time to its own first local event or to the global final time. The earliest root among all active vehicles becomes the next global synchronization time.

At that time:

1. every active vehicle is evaluated at the identical global event time;
2. schema-tagged history segments are closed;
3. all simultaneous guards are ordered by priority, vehicle ID, and event name;
4. event handlers observe one immutable pre-event universe snapshot;
5. topology/configuration mutations are applied;
6. propagation resumes with the new active vehicle set.

This is intentionally different from concatenating multiple independent `SimulationEngine` runs: the universe owns a single event timeline.

## Vehicle specification

A `VehicleSpec` contains:

- stable `vehicle_id`;
- `StateSchema`;
- current/initial packed state;
- vehicle-local RHS;
- vehicle-local event set;
- optional vehicle-local integrator;
- discrete mode label;
- declared DOF (`3`, `6`, or unspecified);
- arbitrary read-only `model_context`.

The runtime does not interpret the contents of `model_context`. Environment, GNC, propulsion, flight-software, or user plugin objects can therefore remain vehicle specific without creating universe-level special cases.

## Topology mutations

A `UniverseMutation` is atomic and contains:

- vehicle IDs to remove;
- `VehicleSpec` objects to upsert;
- a provenance note.

Upsert semantics deliberately cover both spawn and replacement. This supports staging, jettison, vehicle destruction, state-schema changes, mode changes, and future docking/reconfiguration operations through one mechanism.

## Trajectory representation

A multi-vehicle mission cannot be represented by one rectangular state matrix because:

- active vehicle count changes;
- different vehicles can have different state sizes;
- one vehicle can switch schemas over time.

Each vehicle therefore owns a sequence of `VehicleTrajectorySegment` objects. Every segment stores its own schema, DOF, mode, time vector, and state matrix.

## Multi-DOF transition

`map_state_fields()` maps identically named compatible fields across schemas and requires explicit defaults for new fields.

Convenience policies are provided for:

- 6-DOF -> 3-DOF projection: retain position, velocity, and mass;
- 3-DOF -> 6-DOF promotion: retain translational state and initialize attitude/angular rate explicitly or align `+x_B` with velocity.

`DOFSwitchHandler` performs the runtime replacement while preserving vehicle identity and beginning a new schema-tagged segment.

## Rigid-body separation

`separate_two_rigid_bodies()` adds a 6-DOF staging primitive.

Given parent COM state, attitude, angular rate, parent inertia, daughter masses/inertias, daughter COM offsets, and an optional relative separation velocity, the operator computes:

- daughter inertial positions;
- inherited rigid-body `omega x r` translational velocity;
- mass-weighted separation delta-V;
- daughter angular rates;
- linear-momentum residual;
- angular-momentum residual;
- parent-COM consistency residual;
- composite-inertia consistency residual.

If angular-momentum conservation is enabled, a shared daughter spin correction is solved such that total post-separation angular momentum about the parent COM equals the pre-separation value.

`RigidSeparationHandler` integrates this directly into universe topology events.

## Numerical synchronization

Adaptive SciPy integrations use dense output to evaluate non-triggering vehicles at the earliest global event time.

Integrators without dense output, including the F.1 fixed-step RK4 campaign path, are supported through a deterministic short re-propagation from the last synchronization point to the global event time.

Thus the runtime does not require one common integrator across vehicles.

## Event ordering

Simultaneous events are ordered by:

1. descending priority;
2. vehicle ID;
3. event name.

All handlers see the same immutable pre-event snapshot. If a higher-priority event removes or replaces a vehicle generation, lower-priority tied guards from that old generation are suppressed rather than applied to the replacement.

One-shot guards are disabled after firing when a vehicle survives without replacement.

## Reference mission

`examples/multivehicle_mission.py` uses fictional body Nereid-I.

- t = 0 s: one 6-DOF stack;
- t = 5 s: rigid separation into upper stage + booster;
- both daughters propagate concurrently;
- t = 8 s: upper switches 6-DOF -> 3-DOF;
- booster remains 6-DOF;
- both reach t = 20 s in one universe run.

The example writes `reports/i_reference.json`.

## Deliberate boundaries

Milestone I does not yet implement:

- mutual gravitational/aerodynamic interaction between arbitrary vehicles;
- articulated constraints/tethers between active vehicles;
- collision detection between vehicles;
- docking/contact topology;
- distributed-memory multi-vehicle propagation;
- automatic state inference when a target schema introduces unknown subsystem states.

Those capabilities can be layered on the same universe/mutation model without changing the flight-dynamics kernels.
