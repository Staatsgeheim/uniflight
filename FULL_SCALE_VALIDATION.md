# Milestone F.1 — Full-scale validation handoff

Milestone F.1 is designed specifically for the larger campaign that was impractical under the original serial Milestone F runner.

## 1. Install and verify

```bash
python -m pip install -e . --no-build-isolation
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest -q
```

## 2. Check the adaptive nominal

```bash
python examples/gnc_monte_carlo.py --nominal-only --sample-period 0.5
```

Expected reference values are approximately:

- touchdown time: 125.046 s
- lateral error: 0.0379 m
- touchdown speed: 0.0398 m/s
- final mass: 370.755 kg

## 3. Parallel 100-case campaign

For a 32-core machine, start with 31 workers so the OS retains one logical CPU:

```bash
python examples/gnc_monte_carlo.py \
  --cases 100 \
  --workers 31 \
  --backend rk4 \
  --rk4-step 0.1 \
  --sample-period 0.5 \
  --seed 20260827 \
  --skip-reference \
  --output reports/f1_100.json
```

Or use `--workers 0` for automatic selection.

### BLAS oversubscription

Before a large process campaign, set the BLAS/OpenMP thread count to one per worker when applicable:

Linux/macOS:

```bash
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
```

Windows PowerShell:

```powershell
$env:OPENBLAS_NUM_THREADS="1"
$env:OMP_NUM_THREADS="1"
$env:MKL_NUM_THREADS="1"
```

## 4. Convergence spot-check

Replay a small campaign with a smaller RK4 step:

```bash
python examples/gnc_monte_carlo.py \
  --cases 20 --workers 20 --backend rk4 --rk4-step 0.05 \
  --sample-period 0.5 --seed 20260827 --skip-reference \
  --output reports/f1_20_dt005.json
```

Compare the first 20 cases against the 0.1 s run. For cases close to success/failure boundaries, rerun individually with the adaptive SciPy backend.

## 5. Scale to 1,000 cases

```bash
python examples/gnc_monte_carlo.py \
  --cases 1000 \
  --workers 31 \
  --backend rk4 \
  --rk4-step 0.1 \
  --sample-period 0.5 \
  --seed 20260827 \
  --skip-reference \
  --output reports/f1_1000.json
```

## Current dispersions

| Parameter | Distribution |
|---|---|
| initial lateral Y | Normal(10 m, 4 m) |
| initial lateral Z | Normal(0 m, 4 m) |
| initial radial speed | Normal(-12 m/s, 0.8 m/s) |
| actual engine exhaust-velocity scale | Normal(1.0, 0.015) |
| Y position-sensor bias | Normal(0 m, 0.4 m) |

## Current software success criteria

A case is counted as successful when:

- integration terminates at `touchdown`;
- lateral touchdown error < 5 m;
- total touchdown speed < 3 m/s;
- final mass > 300 kg.

These remain reference-software criteria for fictional Nereid-F, not certified mission requirements.

## JSON handoff

The report records:

- UniFlight/Python/NumPy/platform versions
- campaign backend and RK4 step
- requested and actual worker counts
- elapsed wall time and throughput
- base seed
- every per-case deterministic seed
- every sampled parameter
- every outcome metric
- aggregate percentiles/statistics

Send the JSON report back for failure clustering, sensitivity/correlation analysis, outlier replay, and controller-margin assessment.
