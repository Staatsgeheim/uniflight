# UniFlight Milestone G — Full-Scale Validation

Run this after installing the package from the project root.

## 1. Regression suite

```powershell
$env:PYTEST_DISABLE_PLUGIN_AUTOLOAD="1"
$env:PYTHONPATH="src"
pytest -q
```

There are 67 defined tests in Milestone G.

## 2. Recommended 100-case G campaign

On a many-core Windows workstation, first prevent BLAS oversubscription:

```powershell
$env:OPENBLAS_NUM_THREADS="1"
$env:OMP_NUM_THREADS="1"
$env:MKL_NUM_THREADS="1"
```

Then:

```powershell
python examples/gnc_monte_carlo_g.py `
  --cases 100 `
  --workers 0 `
  --backend rk4 `
  --rk4-step 0.1 `
  --sample-period 0.5 `
  --seed 20260827 `
  --skip-reference `
  --output reports/g100.json
```

## 3. Direct 1000-case comparison with F.1

Use exactly the same dispersion seed/settings as the existing F.1 campaign:

```powershell
python examples/gnc_monte_carlo_g.py `
  --cases 1000 `
  --workers 0 `
  --backend rk4 `
  --rk4-step 0.1 `
  --sample-period 0.5 `
  --seed 20260827 `
  --skip-reference `
  --output reports/g1000.json
```

The generated case parameters are deterministic for this base seed, so `g1000.json` can be paired case-by-case with the previous `f1000.json`.

## 4. What to send back

The single `g1000.json` file is sufficient. It contains:

- every case index and deterministic seed,
- sampled dispersion parameters,
- touchdown/timeout outcome,
- landing error and speed,
- final mass,
- estimated thrust scale,
- aggregate percentiles,
- backend/worker/timing metadata.

A useful follow-up analysis is the paired transition table:

- F fail -> G success,
- F success -> G success,
- F success -> G fail,
- F fail -> G fail.

The target is to eliminate the positive-thrust hover cluster without materially increasing hard-landing or lateral-error tails.
