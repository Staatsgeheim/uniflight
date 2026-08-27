# UniFlight — Milestone F.1 Performance & Parallel Monte Carlo

Milestone F.1 is a focused execution-layer optimization of Milestone F. **No flight-physics model or success criterion is intentionally relaxed.** The same sampled-data GNC architecture is retained, while high-volume robustness campaigns can now use deterministic fixed-step integration and process-level parallelism across CPU cores.

## What changed in F.1

- multiprocessing Monte Carlo with portable `spawn` workers
- `--workers 0` automatic CPU discovery (logical CPUs minus one)
- deterministic case seeds invariant to worker count
- multiprocessing chunksize control and progress reporting
- fixed-step classical RK4 campaign integrator with hybrid-event detection
- endpoint-only integration output for low-allocation campaign execution
- optional suppression of trajectory/GNC history for Monte Carlo cases
- analytical point-mass gravity Jacobian for the translational EKF
- serial and parallel campaign results are exactly reproducible for the same seed
- adaptive SciPy DOP853 remains the high-accuracy reference backend
- JSON reports include backend, worker count, elapsed time and throughput

The package version is **0.6.1**.

## Why this is faster

The trusted Milestone F loop executes GNC at explicit sample boundaries. With a 0.5 s cadence and a ~125 s landing, one case has roughly 250 controller intervals. Repeated adaptive `solve_ivp` calls are accurate but expensive for thousands of independent trajectories.

F.1 adds two orthogonal optimizations:

1. **Campaign RK4 backend.** A deterministic fixed step (default 0.1 s) removes most adaptive-solver startup/control overhead while retaining event checks at every internal step.
2. **Process parallelism.** Independent trajectories are dispatched to separate Python processes, so a many-core workstation can run many cases concurrently despite the CPython GIL.

The adaptive solver remains available and should be used to spot-check representative cases.

## Numerical agreement of the campaign backend

For the nominal Nereid-F landing at a 0.5 s GNC cadence, the sandbox comparison was:

| Metric | SciPy DOP853 | RK4, dt=0.1 s | Difference |
|---|---:|---:|---:|
| touchdown time | 125.04603 s | 125.04194 s | -0.00409 s |
| lateral error | 0.0379271 m | 0.0379168 m | -0.0000103 m |
| touchdown speed | 0.0397865 m/s | 0.0397945 m/s | +0.0000080 m/s |
| final mass | 370.75509 kg | 370.75864 kg | +0.00355 kg |

These values establish a software benchmark only; users should choose RK4 step size based on their own model fidelity and convergence study.

## Dependencies

- Python >= 3.11
- NumPy >= 2.0
- SciPy >= 1.13
- pytest >= 8 for development tests

## Install

```bash
python -m pip install -e . --no-build-isolation
```

## Verification

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src pytest -q
```

The complete A–F.1 suite contains **63 tests**. In the constrained sandbox, the suite was run in two bounded groups: 51 A–E regression tests and 16 F/F.1 tests (the groups overlap on the original F tests), with all tests passing. A single all-in-one invocation reached the sandbox wall-clock limit after 56 passing tests and no failures.

## Fast parallel campaign

On Linux/macOS:

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 \
PYTHONPATH=src python examples/gnc_monte_carlo.py \
  --cases 100 \
  --workers 0 \
  --backend rk4 \
  --rk4-step 0.1 \
  --sample-period 0.5 \
  --seed 20260827 \
  --skip-reference \
  --output reports/f1_100.json
```

`--workers 0` means automatic. On a 32-logical-CPU system it normally selects 31 worker processes. To use all 32 explicitly, pass `--workers 32`.

On Windows PowerShell, set the BLAS thread limits before launching if your NumPy distribution uses a threaded BLAS:

```powershell
$env:OPENBLAS_NUM_THREADS="1"
$env:OMP_NUM_THREADS="1"
$env:MKL_NUM_THREADS="1"
python examples/gnc_monte_carlo.py --cases 100 --workers 32 --backend rk4 --rk4-step 0.1 --sample-period 0.5 --seed 20260827 --skip-reference --output reports/f1_100.json
```

Limiting BLAS threads prevents `N workers × N BLAS threads` oversubscription. The UniFlight case itself mostly uses small matrices, but this is a safe campaign setting.

## Reference/adaptive campaign

For a smaller high-accuracy comparison:

```bash
PYTHONPATH=src python examples/gnc_monte_carlo.py \
  --cases 10 --workers 1 --backend scipy --sample-period 0.5
```

The default runner performs one adaptive reference nominal before the campaign. Use `--skip-reference` for repeated performance runs.

## Determinism

For a fixed base seed, the following are invariant to `--workers`:

- sampled dispersions
- per-case sensor RNG seeds
- per-case metrics
- case ordering in the JSON report
- aggregate statistics

The parallel runner requires a pickleable module-level case function because workers use `spawn`; this is portable to Windows and macOS and avoids unsafe state inheritance.

## Architecture retained from Milestone F

```text
truth state
    ↓
sensors + deterministic noise
    ↓
EKF navigation estimate
    ↓
3-D landing guidance
    ↓
quaternion attitude control
    ↓
limited actuator commands
    ↓
zero-order-held plant input
    ↓
continuous integration to next GNC sample
```

Estimator/controller mutation remains outside the ODE RHS. The performance work does not compromise chronological sampled-data semantics.

See `PERFORMANCE.md` for measured sandbox timings and `FULL_SCALE_VALIDATION.md` for the recommended workstation campaign.
