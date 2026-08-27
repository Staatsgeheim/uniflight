# UniFlight — Milestone I Multi-Vehicle / Multi-DOF Runtime

UniFlight **0.9.0** adds a global event-synchronized multi-vehicle universe on top of the complete A–H simulation, GNC, Monte Carlo, and trajectory-optimization stack.

Milestone I is the POST2-parity step that removes the assumption that a mission contains one fixed state vector. Vehicle count, state dimension, DOF, dynamics, events, environment/GNC closure, and integrator may now differ by vehicle and change during flight.

## New in I

- `MultiVehicleUniverseEngine` with one global hybrid-event timeline
- arbitrary simultaneous active vehicle count
- per-vehicle state schema, RHS, events, integrator, mode, DOF, and model context
- dynamic vehicle spawn/remove/replace mutations
- schema-tagged piecewise vehicle trajectory histories
- global synchronization at the earliest event across all vehicles
- deterministic simultaneous-event priority semantics
- 3-DOF -> 6-DOF promotion and 6-DOF -> 3-DOF projection
- runtime `DOFSwitchHandler`
- rigid two-body 6-DOF separation with COM offsets and inherited `omega x r` velocity
- linear- and angular-momentum-conserving separation correction
- `RigidSeparationHandler` for direct topology-changing staging events
- adaptive dense-output and fixed-step resynchronization paths
- 11 new Milestone-I tests; **88/88 total tests pass in bounded groups**

Package version: **0.9.0**.

## Install

```bash
python -m pip install -e . --no-build-isolation
```

## Run the I reference mission

```bash
PYTHONPATH=src python examples/multivehicle_mission.py \
  --output reports/i_reference.json
```

The fictional Nereid-I reference mission begins with one 6-DOF stack, separates at 5 s into concurrently propagated upper and booster vehicles, and switches the upper vehicle from 6-DOF to 3-DOF at 8 s while the booster remains 6-DOF.

## Minimal universe pattern

```python
engine = MultiVehicleUniverseEngine()
result = engine.run((0.0, 600.0), (vehicle_a, vehicle_b))

for vehicle_id, snapshot in result.final_vehicles.items():
    print(vehicle_id, snapshot.mode, snapshot.dof)
```

Topology changes are returned by vehicle event handlers as `UniverseMutation(remove=..., upsert=...)`. A new `VehicleSpec` can therefore be a spawned vehicle or a replacement configuration for an existing vehicle ID.

## Documents

- `MILESTONE_I.md` — multi-vehicle/multi-DOF runtime semantics
- `MILESTONE_H.md` — trajectory targeting and optimization
- `VERIFICATION.md` — complete regression/acceptance record
- `reports/i_reference.json` — deterministic multi-vehicle reference output

## Project scope

The project goal remains functional/architectural proximity to NASA POST2 while explicitly **not claiming** real-mission validation, flight heritage, or certification/independent-IV&V pedigree.

---

## Previous Milestone H overview


UniFlight **0.8.0** adds a general trajectory-design layer to the complete A–G/F.1 celestial-body-agnostic simulation stack.

Milestone H is the first major step from “simulate a mission” toward the POST2-style workflow of **target, constrain, and optimize a mission**.

## New in H

- bounded/scaled design-variable system
- black-box `TrajectoryProblem` evaluator interface
- named scalar/vector objectives and nonlinear constraints
- equality and inequality constraints
- nonlinear least-squares trajectory targeting
- event-state/event-time targeting through ordinary evaluator metrics
- constrained SLSQP optimization
- derivative-free COBYLA fallback
- bound-aware finite-difference Jacobians
- exact-value LRU evaluation cache
- multiple-shooting continuity-defect transcription
- process-parallel independent candidate evaluation
- deterministic Nereid-H simulation-based targeting/optimization reference case
- 10 new H verification tests

Package version: **0.8.0**.

## Install

```bash
python -m pip install -e . --no-build-isolation
```

Dependencies:

- Python >= 3.11
- NumPy >= 2.0
- SciPy >= 1.13
- pytest >= 8 for development

## Run the H reference trajectory design

```bash
PYTHONPATH=src python examples/trajectory_optimization.py \
  --output reports/h_reference.json
```

The example performs two independent tasks on fictional body Nereid-H:

1. target burn duration at fixed mass flow to a 20 km apogee;
2. minimize propellant with mass flow + burn time as design variables while constraining apogee to exactly 20 km.

Typical optimized solution is approximately:

- `mdot`: 8 kg/s (upper design bound)
- burn time: 3.69 s
- propellant: 29.5 kg
- apogee: 20,000 m

The result is physically sensible: higher thrust reduces gravity loss in this constrained radial-burn problem.

## Public optimization pattern

```python
space = DesignSpace([
    DesignVariable("burn_time", 10.0, 1.0, 100.0, scale=10.0),
    DesignVariable("throttle", 0.8, 0.0, 1.0),
])

problem = TrajectoryProblem(
    space,
    evaluator=my_mission_simulation,
    objective=MetricObjective("propellant_used", "minimize"),
    constraints=(
        MetricConstraint("final_altitude", lower=100_000.0),
        MetricConstraint("final_flight_path_angle", lower=0.0, upper=0.0),
        MetricConstraint("max_q", upper=50_000.0),
    ),
)

result = TrajectoryOptimizer().solve(problem)
```

The evaluator can run any UniFlight mission model and simply returns named metrics.

## Verification

Milestone H defines **77 tests** total: the prior 67 A–G tests plus 10 new H targeting/optimization tests.

In constrained environments, run them in bounded groups; see `VERIFICATION.md`.

## Documents

- `MILESTONE_H.md` — design rationale and boundaries
- `VERIFICATION.md` — regression and H acceptance record
- `MILESTONE_G.md` — preceding robust-terminal-GNC work
- `PERFORMANCE.md` — F.1 Monte Carlo performance architecture
- `FULL_SCALE_VALIDATION_G.md` — workstation G robustness campaign

## Project scope

The project goal is functional/architectural proximity to NASA POST2 while explicitly **not claiming** real-mission validation, flight heritage, or certification/independent-IV&V pedigree.
