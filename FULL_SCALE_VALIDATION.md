# Milestone F — Full-scale validation handoff

The sandbox acceptance run is intentionally small. The same deterministic runner can be scaled locally without changing the physics or dispersion definitions.

## Recommended local sequence

From the project root:

```bash
python -m pip install -e . --no-build-isolation
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest -q
```

Then establish the nominal closed-loop case (this is also the sandbox acceptance trajectory):

```bash
python examples/gnc_monte_carlo.py --nominal-only --sample-period 0.5
```

Run an initial 100-case campaign:

```bash
python examples/gnc_monte_carlo.py \
  --cases 100 \
  --sample-period 0.5 \
  --seed 20260827 \
  --output reports/f100.json
```

If that is stable, scale to 1,000 cases:

```bash
python examples/gnc_monte_carlo.py \
  --cases 1000 \
  --sample-period 0.5 \
  --seed 20260827 \
  --output reports/f1000.json
```

The JSON contains environment/version metadata, the exact base seed, all sampled dispersion values, every case metric, the campaign success rate, and aggregate statistics. Sending that JSON back is sufficient to reproduce and analyze individual cases.

## Current reference dispersions

| Parameter | Distribution |
|---|---|
| initial lateral Y | Normal(10 m, 4 m) |
| initial lateral Z | Normal(0 m, 4 m) |
| initial radial speed | Normal(-12 m/s, 0.8 m/s) |
| actual engine exhaust-velocity scale | Normal(1.0, 0.015) |
| Y position-sensor bias | Normal(0 m, 0.4 m) |

Each case also receives an independent deterministic sensor-noise stream derived from the campaign seed.

## Current reference success criteria

A Monte Carlo case is counted as successful when all of the following are true:

- the integration terminates at the `touchdown` event;
- lateral touchdown error < 5 m;
- total touchdown speed < 3 m/s;
- final mass > 300 kg.

These are software-verification criteria for the fictional Nereid-F reference case, not mission-certified requirements.

## Results to return

Please return the generated JSON report(s). If a campaign contains failures, the per-case `index`, `seed`, sampled `parameters`, and `metrics` allow exact replay and diagnosis.
