from __future__ import annotations

"""Execution backends for Milestone N analysis campaigns."""

from concurrent.futures import Executor, ProcessPoolExecutor, as_completed
from dataclasses import dataclass
from typing import Callable, Iterable, Iterator, Protocol, Sequence, TypeVar
import multiprocessing as mp
import os
import pickle

T = TypeVar("T")
R = TypeVar("R")


class ExecutionBackend(Protocol):
    @property
    def workers(self) -> int: ...
    def map(self, function: Callable[[T], R], items: Sequence[T]) -> Iterator[R]: ...


@dataclass(frozen=True, slots=True)
class SerialBackend:
    @property
    def workers(self) -> int:
        return 1

    def map(self, function, items):
        for item in items:
            yield function(item)


@dataclass(frozen=True, slots=True)
class ProcessBackend:
    """Portable spawn-based local multiprocessing backend."""
    max_workers: int = 0
    reserve_cores: int = 1
    chunksize: int = 1

    @property
    def workers(self) -> int:
        cpus = os.cpu_count() or 1
        requested = self.max_workers or max(1, cpus-max(0, self.reserve_cores))
        return max(1, int(requested))

    def map(self, function, items):
        if self.chunksize <= 0:
            raise ValueError("chunksize must be positive")
        try:
            pickle.dumps(function)
        except Exception as exc:
            raise TypeError("process backend requires a pickleable worker function") from exc
        if not items:
            return
        workers = min(self.workers, len(items))
        with ProcessPoolExecutor(max_workers=workers, mp_context=mp.get_context("spawn")) as executor:
            for result in executor.map(function, items, chunksize=self.chunksize):
                yield result


class ExternalExecutorBackend:
    """Adapter for any concurrent.futures-compatible executor.

    This is the extension seam for cluster/distributed systems. Dask, Ray,
    Slurm wrappers, cloud batch systems, or an institutional scheduler can
    expose an ``Executor``-like object without changing the campaign engine.
    """
    def __init__(self, executor: Executor, workers: int | None = None):
        self.executor = executor
        self._workers = int(workers or getattr(executor, "_max_workers", 1) or 1)

    @property
    def workers(self) -> int:
        return self._workers

    def map(self, function, items):
        futures = [self.executor.submit(function, item) for item in items]
        # Yield in submission order for reproducible result ordering.
        for future in futures:
            yield future.result()
