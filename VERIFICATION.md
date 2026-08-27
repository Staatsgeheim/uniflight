# Verification Record — UniFlight Milestone F.1

Milestone F.1 preserves the Milestone A–F verification basis and adds deterministic campaign-integration and multiprocessing tests.

## Defined suite

**63 tests** are defined in `tests/`.

### Original Milestone F cases (48–59)

48. position/velocity sensor deterministic replay
49. noisy attitude sensor preserves quaternion norm
50. navigation EKF update reduces covariance trace
51. actuator position/rate limits
52. quaternion PD zero error and torque saturation
53. arbitrary-body gravity feedforward/lateral guidance
54. body torque saturation
55. abort rule terminal event
56. deterministic Monte Carlo replay
57. Monte Carlo statistics
58. noisy sampled-data 6-DOF landing
59. abort pre-empts touchdown

### New F.1 cases (60–63)

60. fixed-step RK4 detects/refines a terminal hybrid root
61. analytical point-mass gravity Jacobian matches finite differences
62. multiprocessing Monte Carlo is exactly identical to serial for the same seed
63. automatic worker count remains within available-case bounds

## Sandbox execution

The sandbox has 5 visible logical CPUs and a strict command wall-clock limit. Verification was therefore split into bounded runs:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src pytest -q --ignore=tests/test_gnc_robustness.py
# 51/51 passed

PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src pytest -q tests/test_f1_performance.py tests/test_gnc_robustness.py
# 16/16 passed (12 F + 4 F.1)
```

The complete one-shot `pytest -q` command reached the sandbox timeout after **56 passing tests and zero failures**; the remaining affected tests were then exercised in the bounded groups above.

## Numerical backend comparison

Nominal Nereid-F landing, 0.5 s GNC cadence:

- SciPy DOP853: touchdown ~125.04603 s; final mass ~370.75509 kg
- fixed RK4 dt=0.1 s: touchdown ~125.04194 s; final mass ~370.75864 kg
- touchdown-time difference ~4.1 ms
- final-mass difference ~3.6 g
- both satisfy the same nominal success criteria

## Parallel smoke benchmark

Four identical-seed campaign cases were run once serially and once with four worker processes using RK4 dt=0.1 s:

- serial elapsed: ~9.51 s
- 4-worker elapsed: ~6.14 s
- sandbox speedup: ~1.55x
- per-case JSON records: **exactly identical**

The limited speedup is expected in this constrained environment because only 5 logical CPUs are visible and the effective CPU quota is lower than a normal workstation. Process startup is also a large fraction of a four-case run. Larger campaigns on a 32-core machine should amortize that overhead much better.

## Acceptance invariants

- parallel worker count does not alter stochastic results
- RK4 event guards are evaluated at every internal step
- event roots are refined before jump/termination handling
- quaternion state is projected back to unit norm in the F reference campaign
- adaptive DOP853 remains available as the reference path
- sampled-data GNC remains chronological and external to the ODE RHS
- process workers use portable `spawn` semantics
