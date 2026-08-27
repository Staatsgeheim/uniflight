# UniFlight 0.14.0 — Milestone N

UniFlight is a planet-agnostic, hybrid, variable-mass 3/6-DOF research flight-dynamics framework targeting functional parity with NASA POST2 while explicitly excluding claims of real-mission validation, flight heritage, or certification/IV&V pedigree.

Milestone N adds the **integrated analysis/HPC layer** above the A–M flight, GNC, optimization, multi-vehicle, subsystem, engineering-data, mission-language, and plugin stacks.

## New in 0.14.0

- declarative Cartesian/zipped parameter sweeps;
- deterministic Monte Carlo uncertainty propagation;
- independent per-case stochastic mission seeds;
- Saltelli/Sobol first- and total-order global sensitivity;
- process-parallel multistart trajectory optimization;
- a common deterministic case-ledger architecture;
- transactional SQLite result/checkpoint storage;
- automatic restart by stable case IDs;
- portable JSON result export;
- serial and spawn-safe local multiprocessing backends;
- generic external/distributed `Executor` adapter;
- `analysis:` additions to MDL 1.0;
- installed `uniflight-analysis` CLI.

## Installation

```bash
python -m pip install --no-build-isolation -e .
```

The project requires Python 3.11+, NumPy, SciPy, and PyYAML.

## Reference mission

```bash
uniflight-mission validate missions/nereid_n_analysis.yaml
uniflight-analysis list missions/nereid_n_analysis.yaml
```

Run the declared grid sweep:

```bash
uniflight-analysis sweep missions/nereid_n_analysis.yaml propulsion-grid
```

Run Monte Carlo:

```bash
uniflight-analysis monte-carlo missions/nereid_n_analysis.yaml --cases 1000
```

Run Sobol sensitivity:

```bash
uniflight-analysis sobol missions/nereid_n_analysis.yaml propulsion-sensitivity
```

Run multistart H optimization:

```bash
uniflight-analysis optimize-batch missions/nereid_n_analysis.yaml multistart
```

Inspect a checkpoint/result database:

```bash
uniflight-analysis status reports/n_analysis.sqlite nereid-n-analysis.monte_carlo
```

Export it:

```bash
uniflight-analysis export reports/n_analysis.sqlite \
  nereid-n-analysis.monte_carlo reports/mc.json
```

## Checkpoint/restart

No special resume flag is required. Re-run the same campaign ID against the same mission SHA-256. Completed stable case IDs are skipped; failed or missing cases are eligible for execution.

## Declarative analysis example

```yaml
analysis:
  execution:
    backend: process
    workers: 0
    chunksize: 1
    store: ../reports/n_analysis.sqlite

  sweeps:
    - id: propulsion-grid
      mode: cartesian
      variables:
        - name: mass_flow
          pointer: /vehicles/vehicle/phases/0/dynamics/ideal_rocket/mass_flow
          values: [0.35, 0.50, 0.65]

  sobol:
    - id: propulsion-sensitivity
      metric: final_altitude
      base_samples: 64
      seed: 20260827
      variables:
        - name: mass_flow
          pointer: /vehicles/vehicle/phases/0/dynamics/ideal_rocket/mass_flow
          lower: 0.35
          upper: 0.65
```

## Verification

Milestone N preserves all A–M tests and adds 13 analysis/HPC tests. The complete suite is **155/155 passing** in bounded groups.

See:

- `MILESTONE_N.md` — architecture and capability specification;
- `HPC_API.md` — execution/result-store contracts;
- `VERIFICATION.md` — regression evidence;
- `FULL_SCALE_VALIDATION_N.md` — larger workstation campaigns;
- `reports/n_reference.json` — sandbox reference results.

## Scope boundary

UniFlight is an advanced research/engineering simulator. The project does **not** claim:

- validation against operational flight missions;
- flight heritage;
- certification or independent IV&V pedigree.
