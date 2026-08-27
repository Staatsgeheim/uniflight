# UniFlight Analysis/HPC API 1.0

## ExecutionBackend

An execution backend must expose:

```python
@property
def workers(self) -> int: ...

def map(self, function, items): ...
```

The iterator must yield one result for each submitted item. UniFlight case IDs make completion order irrelevant to restart semantics, although the core backends preserve submission order for reproducibility.

### SerialBackend

Runs in the caller process. Best for debugging and very cheap cases.

### ProcessBackend

Uses `ProcessPoolExecutor` with multiprocessing `spawn` semantics. Worker functions must be pickleable module-level callables. `max_workers=0` uses available logical CPUs minus a reserved core.

For CPU-heavy campaigns set BLAS/OpenMP libraries to one thread per worker:

```bash
OPENBLAS_NUM_THREADS=1
OMP_NUM_THREADS=1
MKL_NUM_THREADS=1
```

### ExternalExecutorBackend

Wraps an existing `concurrent.futures.Executor`-compatible object. It is the deployment seam for external/distributed schedulers.

```python
with MyClusterExecutor(...) as executor:
    backend = ExternalExecutorBackend(executor, workers=128)
    runner = MissionCampaignRunner(mission, backend=backend, store=store)
```

UniFlight intentionally does not import Dask, Ray, MPI, Slurm, Kubernetes, or a cloud SDK in core.

## SQLiteResultStore

`SQLiteResultStore(path)` uses WAL mode and transactional writes.

Core operations:

```python
store.begin_campaign(...)
store.completed_case_ids(campaign_id)
store.write_case(...)
store.cases(campaign_id)
store.summary(campaign_id)
store.export_json(campaign_id, output)
```

The coordinator is the sole database writer; worker processes return case results to it. This avoids cross-process SQLite write contention.

## Restart semantics

A campaign is compatible with an existing checkpoint only when both `campaign_id` and `mission_sha256` match. Completed case IDs are skipped. Failed cases are eligible to execute again, allowing transient failures to be retried without deleting successful work.

## Stable case identity

The default case ID is derived from:

- analysis kind;
- mission overrides;
- human-readable parameter metadata.

Worker count, execution order, and wall-clock timing do not affect identity.


# Milestone N — Integrated Analysis / HPC

UniFlight **0.14.0** adds a common campaign layer above the verified flight, mission, plugin, engineering-data, multi-vehicle, GNC, and optimization stacks.

## Goal

Milestone N closes the POST2-parity gap around analysis orchestration. A single declarative mission can now be used as the source model for:

- Cartesian or zipped parameter sweeps;
- Monte Carlo / uncertainty propagation;
- Saltelli/Sobol global sensitivity studies;
- parallel multistart trajectory optimization;
- deterministic checkpoint/restart;
- structured persistent result storage;
- local serial or multiprocessing execution;
- externally supplied distributed executors.

The physics kernel is unchanged. Analysis modifies mission inputs through the same RFC-6901 JSON-pointer override mechanism introduced in Milestone L and executes the same compiled mission runtime.

## Case-ledger architecture

All analysis types reduce to immutable `AnalysisCase` records:

```text
case = {
    stable_case_id,
    index,
    kind,
    mission_overrides,
    human_parameters
}
```

Case IDs are content-derived SHA-256 fragments. This makes restart independent of worker ordering and allows a campaign to skip already completed points safely.

## Execution backends

`ExecutionBackend` is the only contract required by the campaign engine.

Core backends:

- `SerialBackend`
- `ProcessBackend` — portable `spawn` multiprocessing
- `ExternalExecutorBackend` — adapter for a `concurrent.futures.Executor`-compatible external scheduler

The external-executor seam is intentionally dependency-free. A site may connect Dask, Ray, Slurm, Kubernetes, cloud batch, or an institutional scheduler without making one of those systems a UniFlight core dependency.

## Structured results and restart

`SQLiteResultStore` is a transactional campaign ledger. It records:

- campaign ID and kind;
- mission SHA-256;
- campaign metadata;
- stable case IDs;
- case input parameters;
- completion/failure state;
- output metrics;
- errors;
- per-case elapsed time.

Re-running an identical campaign ID against the same mission SHA skips completed case IDs. The SQLite database is therefore both the result store and the checkpoint.

Portable JSON export is provided for external analysis tools.

## Parameter sweeps

`ParameterSweep` supports:

- Cartesian product grids;
- zipped grids;
- arbitrary mission JSON pointers;
- deterministic case ordering and IDs.

## Monte Carlo / uncertainty propagation

`MissionMonteCarlo` accepts arbitrary `Dispersion` objects. Core distributions include normal and uniform.

Each case derives two deterministic random streams from the campaign base seed:

1. a dispersion stream;
2. an independent mission stochastic seed.

This prevents all noisy GNC/sensor realizations from accidentally sharing the same internal mission seed while retaining exact reproducibility.

## Sobol global sensitivity

`SobolSensitivity` generates a scrambled Sobol A/B/A_Bi Saltelli design and evaluates scalar output metrics.

It reports:

- first-order indices `S_i`;
- total-order indices `S_Ti`;
- output variance;
- parameter names and base-sample count.

The implementation uses a Saltelli first-order estimator and Jansen total-order estimator.

For `d` uncertain inputs and `N` base samples, the campaign contains:

```text
N * (d + 2)
```

trajectory evaluations.

## Multistart optimization

`OptimizationBatch` executes the Milestone H optimization declaration from multiple starting points concurrently. Each start may override the declared initial values of one or more H design variables.

The case record stores final objective, design, mission metrics, constraint violation, evaluation count, and iteration count.

## MDL integration

Mission Definition Language 1.0 now accepts:

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

  optimization_batches:
    - id: multistart
      starts:
        - name: low
          values: {mass_flow: 0.30}
        - name: high
          values: {mass_flow: 0.90}
```

The existing `monte_carlo:` MDL section is executable through the same N engine.

## CLI

A second installed command is provided:

```text
uniflight-analysis
```

Subcommands:

- `list`
- `sweep`
- `monte-carlo`
- `sobol`
- `optimize-batch`
- `status`
- `export`

Restart is automatic: rerun the same command/campaign ID and only missing cases execute.

## Nereid-N reference campaign

The included reference mission is intentionally lightweight enough for CI but exercises every N campaign type.

Sandbox reference execution on four worker processes:

| Campaign | Cases | Time | Failures |
|---|---:|---:|---:|
| propulsion grid | 6 | 2.58 s | 0 |
| Monte Carlo | 32 | 2.65 s | 0 |
| Sobol | 256 | 4.31 s | 0 |
| optimization multistart | 3 | 2.36 s | 0 |

All **297** stored analysis cases completed successfully.

The 64-base-sample Sobol run found final altitude to be dominated by mass flow, with total-order indices approximately:

```text
mass_flow         0.8900
exhaust_velocity  0.0986
```

The three independent optimization starts converge to the same mass-flow solution, approximately:

```text
mass_flow = 0.4138534 kg/s
final_altitude = 8.000000 m
```

A repeated sweep demonstrated checkpoint/restart by executing **0** new cases and resuming all **6/6** completed cases.

## Boundaries

Milestone N does not embed a particular cluster product, cloud service, or batch scheduler. Those are deployment decisions and belong behind `ExternalExecutorBackend` or a Milestone M plugin.

It also does not claim flight validation, flight heritage, or certification/IV&V pedigree.
