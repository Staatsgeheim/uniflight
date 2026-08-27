# UniFlight — Milestone G Robust Terminal GNC

UniFlight 0.7.0 is a targeted guidance/control robustness upgrade on top of the complete Milestone F.1 physics and parallel Monte Carlo stack.

The F.1 large-scale campaign exposed a specific failure mode: slightly positive multiplicative thrust error could make the PD terminal controller settle into an almost motionless hover a few metres above the touchdown surface. Those cases satisfied lateral-error, speed, and propellant margins but never crossed the touchdown event before the 150 s campaign limit.

Milestone G fixes that failure mode without changing the plant physics or relaxing the success criteria.

## New in G

- opt-in terminal contact/sink mode in `VectorLandingGuidance`
- `AdaptiveThrustScaleEstimator` for sampled-data propulsion-effectiveness adaptation
- thrust-scale diagnostics in `GNCRecord`
- robust Nereid-G validation scenario (`validation_g.py`)
- parallel G Monte Carlo runner (`examples/gnc_monte_carlo_g.py`)
- paired F.1-vs-G sandbox acceptance reports
- 4 new terminal-robustness verification tests

Package version: **0.7.0**.

## Terminal anti-hover mode

Inside a configurable terminal zone, the guidance law commands a small nonzero velocity toward the surface instead of asymptotically requesting zero velocity before contact. This removes the static equilibrium produced by a positive thrust-model bias.

The Nereid-G reference configuration uses:

- terminal zone: **30 m**
- terminal sink rate: **0.5 m/s**

Defaults remain disabled, so existing Milestone-F behavior is preserved unless explicitly requested.

## Adaptive thrust effectiveness

`AdaptiveThrustScaleEstimator` compares sampled observed non-gravitational acceleration against the previous commanded thrust acceleration and estimates an effective multiplicative propulsion scale. The update is bounded and low-pass filtered.

The estimator is kept outside the adaptive/fixed-step continuous RHS and therefore preserves the sampled-data architecture introduced in F.

## Paired sandbox acceptance

Same 32 deterministic dispersion cases, same seed (`20260827`), same RK4 `dt=0.1 s`, same 0.5 s GNC cadence:

| Controller | Success | Mean touchdown time | Wall time (4 workers) |
|---|---:|---:|---:|
| F.1 | 16/32 (50.0%) | 111.35 s | 30.62 s |
| G | 32/32 (100.0%) | 54.36 s | 14.50 s |

For the G set:

- mean landing error: ~0.97 m
- 95th percentile landing error: ~2.01 m
- mean touchdown speed: ~0.60 m/s
- 95th percentile touchdown speed: ~0.80 m/s
- mean final mass: ~437.9 kg

The original success limits remain: touchdown event, `<5 m` lateral error, `<3 m/s` touchdown speed, and `>300 kg` final mass.

## Install

```bash
python -m pip install -e . --no-build-isolation
```

Dependencies:

- Python >= 3.11
- NumPy >= 2.0
- SciPy >= 1.13
- pytest >= 8 for development

## Verification

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src pytest -q
```

Milestone G defines **67 tests**. In the constrained sandbox they were run in bounded groups:

- 47 A–E/non-GNC regressions: all passed
- 20 F/F.1/G tests: all passed

## Run the robust campaign

Linux/macOS:

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 \
PYTHONPATH=src python examples/gnc_monte_carlo_g.py \
  --cases 1000 --workers 0 --backend rk4 --rk4-step 0.1 \
  --sample-period 0.5 --seed 20260827 --skip-reference \
  --output reports/g1000.json
```

Windows PowerShell:

```powershell
$env:OPENBLAS_NUM_THREADS="1"
$env:OMP_NUM_THREADS="1"
$env:MKL_NUM_THREADS="1"
python examples/gnc_monte_carlo_g.py --cases 1000 --workers 0 --backend rk4 --rk4-step 0.1 --sample-period 0.5 --seed 20260827 --skip-reference --output reports/g1000.json
```

`--workers 0` uses logical CPUs minus one (bounded by case count). Case dispersions and stochastic streams are deterministic across worker counts.

The original F.1 runner remains at `examples/gnc_monte_carlo.py` for direct baseline reproduction.

See:

- `MILESTONE_G.md` for the control-law rationale and boundaries
- `FULL_SCALE_VALIDATION_G.md` for the workstation campaign
- `PERFORMANCE.md` for F.1 execution-layer details
- `VERIFICATION.md` for the test record
