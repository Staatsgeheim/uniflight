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
