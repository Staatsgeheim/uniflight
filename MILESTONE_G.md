# Milestone G — Robust Terminal Guidance

Milestone G addresses the failure mode exposed by the F.1 large-scale Monte Carlo campaign: trajectories with slightly positive thrust-scale error could converge to a quasi-static hover a few metres above the surface and reach the 150 s campaign limit without touchdown.

The physics plant, actuator limits, sensor dispersions, success criteria, RK4 campaign backend, and process-parallel Monte Carlo engine are unchanged. G changes only terminal guidance/controller robustness.

## Controller additions

### 1. Terminal contact/sink mode

`VectorLandingGuidance` now optionally applies a nonzero inward target velocity inside a configurable terminal zone. In the Nereid-G reference case:

- terminal zone: 30 m
- terminal sink rate: 0.5 m/s

This removes the static-hover equilibrium that can arise from multiplicative thrust error under pure PD position/zero-velocity regulation.

The feature is opt-in. Defaults (`terminal_sink_rate=0`, `terminal_zone=0`) reproduce the Milestone-F guidance law.

### 2. Adaptive thrust-scale estimator

`AdaptiveThrustScaleEstimator` is a sampled-data scalar estimator for effective propulsion scale. It compares observed non-gravitational acceleration with the previous nominal thrust-acceleration command and applies a bounded low-pass update.

It is intentionally lightweight and lives outside the continuous RHS. It is not a substitute for a future augmented-state propulsion/navigation EKF, but it reduces persistent model bias and provides a diagnostic estimate in GNC records and campaign metrics.

### 3. Diagnostics

`GNCRecord` now carries `thrust_scale_estimate`. The G campaign JSON also reports `estimated_thrust_scale` per case and aggregate statistics.

## Bounded sandbox paired acceptance

The exact same first 32 deterministic dispersion cases (`base_seed=20260827`) were run through F.1 and G with fixed RK4 `dt=0.1 s`, GNC cadence `0.5 s`, and four worker processes.

| Controller | Success | Mean touchdown time | Campaign wall time |
|---|---:|---:|---:|
| F.1 baseline | 16/32 (50.0%) | 111.35 s | 30.62 s |
| G robust | 32/32 (100.0%) | 54.36 s | 14.50 s |

G retained the original acceptance limits (`<5 m` lateral error, `<3 m/s` touchdown speed, `>300 kg` final mass). The improvement therefore comes from eliminating timeout/hover cases rather than relaxing the definition of success.

The paired reports are included in `reports/`.

## Boundary of the estimator

The scalar thrust estimator uses sampled velocity increments and the commanded thrust-acceleration vector. It does not yet model:

- throttle-actuator lag explicitly inside the estimator,
- gimbal dynamics in the propulsion effectiveness state,
- varying specific impulse / chamber pressure separately,
- correlated accelerometer or navigation biases,
- engine-out topology changes.

Those belong in a later augmented-state fault-detection/propulsion-estimation layer.
