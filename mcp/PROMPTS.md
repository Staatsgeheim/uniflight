# Prompt Catalog

Prompt component version: `1`.

## `create_uniflight_mission`
Guides an agent from mission intent to valid MDL. It asks for/infers body, vehicle, initial state, phases, models, solver, events, outputs, and datasets, then validates before execution.

## `debug_uniflight_simulation`
Starts with run summary/events, then examines state, flow, force breakdown, solver settings, event roots, units, frames, and relevant model validity.

## `design_entry_trajectory`
Emphasizes entry state, atmosphere, continuum/rarefied regime, aero, heating/TPS, peak-q/heat/deceleration outputs, and solver verification.

## `design_landing_simulation`
Emphasizes EDL event sequencing, deployables, powered descent, GNC chronology, terrain/contact, touchdown metrics, and Monte Carlo robustness.

## `optimize_trajectory`
Builds a deterministic evaluator, bounded/scaled variables, objective/constraints, validates the declaration, solves, and independently replays the optimum.

## `analyze_monte_carlo_failures`
Queries failure pages rather than dumping all cases, clusters failure reasons, replays representative cases, and distinguishes physical failure from timeout/criteria failure.

## `verify_against_external_reference`
Requires untouched reference data/hash, exact published assumptions, channel mapping, timestamp audit, explicit tolerance/alignment policy, solver refinement, and evidence category `external_benchmark`.

## `create_uniflight_plugin`
Creates a separate package using `uniflight.plugins`, exact Plugin API version, namespaced capabilities, version-pinned mission requirement, and compatibility/collision tests.

## `review_uniflight_mission`
Checks MDL schema, units, frames, fidelity, data/plugin pinning, events/priorities, solver settings, outputs, reproducibility, and scientific claims.

## Mandatory system reminders embedded in prompts

- SI internally.
- Quaternion maps body → inertial.
- Body axes are +x forward, +y right, +z down.
- Never mutate sampled estimator/controller state inside an adaptive RHS.
- Pin datasets/plugins.
- Record solver tolerances/steps and seeds.
- Verify before declaring success.
- External benchmark agreement is not flight validation.
