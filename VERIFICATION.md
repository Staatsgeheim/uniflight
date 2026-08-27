# Verification Record — UniFlight Milestone F

Milestone F preserves all Milestone A-E regression tests and adds sampled-data GNC, estimation, actuators, aborts, and robustness verification.

## Total suite

**59 tests** are defined in `tests/`.

## Milestone F cases

48. position/velocity sensor deterministic replay for a fixed seed
49. noisy attitude sensor preserves unit-quaternion norm
50. navigation EKF measurement update reduces covariance trace
51. first-order actuator obeys position and slew-rate limits
52. quaternion PD controller has zero command at zero error and respects torque saturation
53. vector landing guidance includes arbitrary-body gravity feedforward and lateral correction
54. commanded body torque respects componentwise saturation
55. state-limit abort rule creates a terminal event at the configured threshold
56. Monte Carlo sampled parameters and stochastic case metrics reproduce exactly for the same base seed
57. Monte Carlo aggregate statistics and success rate are reported
58. sampled-data 6-DOF closed-loop landing succeeds with noisy navigation
59. abort event terminates the closed-loop trajectory before touchdown

## Key invariants / acceptance checks

- GNC execution occurs at explicit monotonically increasing sample times.
- Commands are held constant between GNC updates.
- Sensor noise and Monte Carlo dispersions are deterministic for a fixed seed.
- EKF covariance uses the Joseph update form.
- Attitude measurements remain normalized.
- Actuator position and rate constraints are enforced in the continuous plant state.
- Reference noisy landing terminates on the touchdown event with lateral error < 5 m and radial speed magnitude < 5 m/s in the unit test.
- Abort threshold events can pre-empt touchdown.
- All prior A-E verification cases remain regression tests.

## Sandbox acceptance profile

The bounded run used for the final Milestone F handoff is:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src pytest -q tests/test_gnc_robustness.py
PYTHONPATH=src python examples/gnc_monte_carlo.py --nominal-only --sample-period 0.5
```

Observed sandbox result:

- Milestone F-specific tests: 12/12 passed
- nominal trajectory terminated at `touchdown`
- touchdown time: ~125.046 s
- lateral error: ~0.0379 m
- total touchdown speed: ~0.0398 m/s
- radial touchdown speed: ~-0.0128 m/s
- final mass: ~370.755 kg
- GNC updates: 251

A coarse 0.75 s control cadence was also explored during development and materially degraded touchdown performance, so the acceptance profile retains 0.5 s rather than weakening the success criteria. Statistical acceptance is intentionally left to the larger local campaign.

## Full-scale handoff

Run the entire test suite and then a larger campaign locally as described in `FULL_SCALE_VALIDATION.md`. The JSON campaign report is sufficient for deterministic replay because it records every case seed, sampled dispersion, and outcome metric.
