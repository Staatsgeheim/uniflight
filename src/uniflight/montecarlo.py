from __future__ import annotations
from concurrent.futures import ProcessPoolExecutor
import multiprocessing as mp
from dataclasses import dataclass
import os
import pickle
import time
from typing import Callable, Mapping
import numpy as np


class Dispersion:
    def sample(self, rng: np.random.Generator):
        raise NotImplementedError


@dataclass(frozen=True, slots=True)
class NormalDispersion(Dispersion):
    mean: float
    std: float

    def __post_init__(self):
        if not np.isfinite(self.mean) or not np.isfinite(self.std) or self.std < 0:
            raise ValueError("invalid normal dispersion")

    def sample(self, rng):
        return float(rng.normal(self.mean, self.std))


@dataclass(frozen=True, slots=True)
class UniformDispersion(Dispersion):
    low: float
    high: float

    def __post_init__(self):
        if not np.isfinite(self.low) or not np.isfinite(self.high) or self.high < self.low:
            raise ValueError("invalid uniform dispersion")

    def sample(self, rng):
        return float(rng.uniform(self.low, self.high))


@dataclass(frozen=True, slots=True)
class MonteCarloCaseResult:
    index: int
    seed: int
    parameters: Mapping[str, float]
    metrics: Mapping[str, float | bool]


@dataclass(frozen=True, slots=True)
class MetricStatistics:
    mean: float
    std: float
    minimum: float
    maximum: float
    p05: float
    median: float
    p95: float


@dataclass(frozen=True, slots=True)
class MonteCarloSummary:
    cases: tuple[MonteCarloCaseResult, ...]
    success_rate: float
    statistics: Mapping[str, MetricStatistics]
    elapsed_seconds: float = 0.0
    workers: int = 1


@dataclass(frozen=True, slots=True)
class _CasePayload:
    index: int
    seed: int
    parameters: Mapping[str, float]
    case_function: Callable[[Mapping[str, float], np.random.Generator], Mapping[str, float | bool]]


def _execute_case(payload: _CasePayload) -> MonteCarloCaseResult:
    rng = np.random.default_rng(payload.seed)
    metrics = dict(payload.case_function(payload.parameters, rng))
    return MonteCarloCaseResult(payload.index, payload.seed, payload.parameters, metrics)


def automatic_worker_count(n_cases: int, reserve_cores: int = 1) -> int:
    """Resolve a conservative process count for independent Monte Carlo cases."""
    if n_cases <= 0:
        raise ValueError("n_cases must be positive")
    cpus = os.cpu_count() or 1
    return max(1, min(n_cases, max(1, cpus-max(0, int(reserve_cores)))))


def _summarize(results: list[MonteCarloCaseResult], elapsed: float, workers: int) -> MonteCarloSummary:
    results.sort(key=lambda r: r.index)
    successes = np.array([bool(r.metrics.get("success", True)) for r in results], dtype=bool)
    names = sorted({
        k for r in results for k, v in r.metrics.items()
        if k != "success" and isinstance(v, (int, float, np.integer, np.floating))
    })
    stats: dict[str, MetricStatistics] = {}
    for name in names:
        vals = np.array([
            float(r.metrics[name]) for r in results
            if name in r.metrics and np.isfinite(float(r.metrics[name]))
        ], dtype=float)
        if vals.size:
            stats[name] = MetricStatistics(
                float(vals.mean()), float(vals.std(ddof=0)), float(vals.min()), float(vals.max()),
                float(np.quantile(vals, .05)), float(np.median(vals)), float(np.quantile(vals, .95)),
            )
    return MonteCarloSummary(tuple(results), float(successes.mean()), stats, float(elapsed), int(workers))


class MonteCarloRunner:
    """Deterministic serial/parallel Monte Carlo runner.

    Each case receives two deterministic random streams derived from the base
    seed: one for dispersions and one for the case itself.  Consequently serial
    and multiprocessing executions are bit-for-bit identical as long as the
    case function is deterministic for its supplied NumPy Generator.

    Parallel execution requires a pickleable case function (normally a module-
    level function or ``functools.partial`` of one).  This is required for
    portable multiprocessing, especially on Windows/macOS spawn semantics.
    """
    def __init__(self, case_function: Callable[[Mapping[str, float], np.random.Generator], Mapping[str, float | bool]],
                 dispersions: Mapping[str, Dispersion], base_seed: int = 0):
        self.case_function = case_function
        self.dispersions = dict(dispersions)
        self.base_seed = int(base_seed)

    def _payloads(self, n_cases: int) -> list[_CasePayload]:
        root_children = np.random.SeedSequence(self.base_seed).spawn(n_cases)
        payloads: list[_CasePayload] = []
        for i, root in enumerate(root_children):
            dispersion_ss, case_ss = root.spawn(2)
            dispersion_rng = np.random.default_rng(dispersion_ss)
            params = {k: d.sample(dispersion_rng) for k, d in self.dispersions.items()}
            seed = int(case_ss.generate_state(1, dtype=np.uint64)[0])
            payloads.append(_CasePayload(i, seed, params, self.case_function))
        return payloads

    def run(self, n_cases: int, *, workers: int = 1, chunksize: int = 1,
            progress: Callable[[int, int], None] | None = None) -> MonteCarloSummary:
        if n_cases <= 0:
            raise ValueError("n_cases must be positive")
        if chunksize <= 0:
            raise ValueError("chunksize must be positive")
        if workers == 0:
            workers = automatic_worker_count(n_cases)
        if workers < 0:
            raise ValueError("workers must be >= 0")
        workers = min(int(workers), n_cases)
        payloads = self._payloads(n_cases)
        t0 = time.perf_counter()

        if workers <= 1:
            results = []
            for completed, payload in enumerate(payloads, start=1):
                results.append(_execute_case(payload))
                if progress is not None:
                    progress(completed, n_cases)
            return _summarize(results, time.perf_counter()-t0, 1)

        try:
            pickle.dumps(self.case_function)
        except Exception as exc:
            raise TypeError(
                "Parallel Monte Carlo requires a pickleable case function. "
                "Define it at module scope (or use functools.partial of a module-level function)."
            ) from exc

        results = []
        # executor.map amortizes IPC with chunksize and preserves deterministic
        # case ordering. Independent trajectories then run on separate cores.
        with ProcessPoolExecutor(max_workers=workers, mp_context=mp.get_context("spawn")) as executor:
            for completed, result in enumerate(
                executor.map(_execute_case, payloads, chunksize=chunksize), start=1
            ):
                results.append(result)
                if progress is not None:
                    progress(completed, n_cases)
        return _summarize(results, time.perf_counter()-t0, workers)
