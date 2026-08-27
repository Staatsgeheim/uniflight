# UniFlight 0.14.0 verification — Milestone N

Verification was executed in bounded groups because a monolithic pytest invocation exceeds the sandbox wall-clock limit.

## Regression totals

| Group | Result |
|---|---:|
| A–E flight physics / atmosphere / entry / EDL | 47/47 |
| F/F.1/G GNC / robustness / performance | 20/20 |
| H–M optimization / multi-vehicle / subsystems / data / MDL / plugins | 75/75 |
| N analysis/HPC | 13/13 |
| **Total** | **155/155** |

## N-specific verification

The N tests cover:

1. Cartesian sweep construction and stable content IDs;
2. deterministic Monte Carlo sampling;
3. a known two-variable linear Sobol problem (`S=[0.8,0.2]`);
4. transactional SQLite checkpoint persistence;
5. external executor adaptation;
6. multiprocessing backend execution;
7. MDL analysis declaration validation;
8. campaign restart skipping completed case IDs;
9. multistart optimization pointer resolution;
10. rejection of unpickleable multiprocessing workers;
11. duplicate analysis-ID rejection;
12. failed-case recording without crashing the campaign coordinator;
13. analysis CLI declaration inspection.

## Reference campaigns

Reference mission: `missions/nereid_n_analysis.yaml`

Mission SHA-256:

```text
9488bd5e0d8b5236d5eeb5bc0c198fc28c3c135e2cc9fae0a2c1fab23c2181b5
```

The sandbox exposed a small CPU allocation; four worker processes were used for the reference campaigns.

| Campaign | Cases | Completed | Failed | Wall time |
|---|---:|---:|---:|---:|
| Parameter sweep | 6 | 6 | 0 | 2.58 s |
| Monte Carlo | 32 | 32 | 0 | 2.65 s |
| Sobol sensitivity | 256 | 256 | 0 | 4.31 s |
| Optimization multistart | 3 | 3 | 0 | 2.36 s |

The shared SQLite store therefore contains **297 completed cases** and zero failed cases.

### Checkpoint/restart

The parameter sweep was immediately reissued with the same mission SHA and campaign ID:

```text
requested_cases = 6
executed_cases  = 0
resumed_cases   = 6
```

This confirms that the database serves as a restart checkpoint.

### Sobol reference

With 64 base samples and two propulsion variables:

```text
first-order:
  mass_flow         0.9413
  exhaust_velocity  0.1000

total-order:
  mass_flow         0.8900
  exhaust_velocity  0.0986
```

Finite-sample first-order estimators can lie slightly outside their asymptotic ordering; larger production studies should use the full-scale guidance in `FULL_SCALE_VALIDATION_N.md`.

### Multistart optimization reference

All three starts converged to the same constrained optimum within numerical tolerance:

```text
mass_flow       ≈ 0.4138534 kg/s
final_altitude  ≈ 8.000000 m
max constraint violation = 0
```

## Numerical/reproducibility notes

- process execution uses `spawn`, including on POSIX, to match Windows/macOS portability requirements;
- worker count does not change Monte Carlo dispersion values or case identity;
- Monte Carlo mission stochastic seeds are separately derived from dispersion random streams;
- result-store writes are coordinator-only and transactional;
- stable case IDs do not depend on scheduling order or wall-clock timing.
