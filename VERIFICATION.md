# Verification Record — UniFlight Milestone G

Milestone G preserves all A–F.1 verification cases and adds four terminal-guidance robustness cases.

## Defined suite

**67 tests** are defined in `tests/`.

### New G cases (64–67)

64. terminal sink mode produces an inward velocity command in the near-surface zone
65. adaptive thrust-scale estimator exactly recovers a synthetic noiseless multiplicative scale
66. a known positive-thrust F.1 hover/timeout case fails under the baseline controller and lands under G
67. negative thrust dispersion remains inside the original landing speed/error/mass limits

## Sandbox execution

The complete suite was executed in bounded groups due to command wall-clock limits:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src pytest -q \
  tests/test_rocket.py tests/test_state_and_frames.py tests/test_attitude.py \
  tests/test_gravity.py tests/test_events.py tests/test_atmosphere.py \
  tests/test_flow_aero_propulsion.py tests/test_atmospheric_ascent.py \
  tests/test_6dof_flow_and_aero.py tests/test_6dof_dynamics_and_tvc.py \
  tests/test_6dof_atmospheric_flight.py tests/test_entry_reentry.py tests/test_edl.py
# 47/47 passed

PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src pytest -q \
  tests/test_gnc_robustness.py tests/test_f1_performance.py \
  tests/test_g_terminal_robustness.py
# 20/20 passed
```

Total: **67/67 passed in bounded groups**.

## Paired 32-case acceptance

Fixed conditions:

- base seed: `20260827`
- RK4 step: 0.1 s
- GNC sample period: 0.5 s
- workers: 4
- same dispersion definitions
- unchanged success criteria

Results:

- F.1 baseline: 16/32 successful (50.0%), mean reported touchdown/timeout time 111.35 s
- G robust: 32/32 successful (100.0%), mean touchdown time 54.36 s
- G landing-error p95: ~2.01 m
- G touchdown-speed p95: ~0.80 m/s

The paired JSON reports are included in `reports/f1_baseline32.json` and `reports/g_acceptance32.json`.

## Adaptive/fixed-step nominal agreement under G

Nereid-G nominal, 0.5 s GNC cadence:

- SciPy DOP853 touchdown: ~53.440740 s
- RK4 dt=0.1 touchdown: ~53.440785 s
- time difference: ~45.6 microseconds
- SciPy final mass: ~438.636184 kg
- RK4 final mass: ~438.636127 kg

Both produce the same success classification and nearly identical terminal metrics.

## Acceptance invariants

- all F/F.1 physics and Monte Carlo infrastructure remain available
- baseline F controller remains reproducible
- terminal sink mode is opt-in and disabled by default
- adaptive thrust estimation is sampled-data logic outside the continuous RHS
- G does not relax the original success thresholds
- deterministic case parameters remain invariant across worker counts
