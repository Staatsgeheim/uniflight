from __future__ import annotations

"""Milestone N integrated analysis, uncertainty, sensitivity and batch optimization.

All campaigns reduce to deterministic case ledgers. Cases carry stable content
IDs, are executed through a pluggable backend, and are transactionally written
to :class:`SQLiteResultStore`. Restart is therefore automatic: completed case
IDs are skipped on a subsequent run.
"""

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Mapping, Sequence
from itertools import product
from hashlib import sha256
import json
import math
import time

import numpy as np
from scipy.stats import qmc

from .mission import load_mission, MissionCompiler, MissionDocument
from .hpc import ExecutionBackend, SerialBackend
from .result_store import SQLiteResultStore, StoredCase
from .montecarlo import Dispersion, NormalDispersion, UniformDispersion
from ._version import __version__ as _uniflight_version


def _plain(value: Any) -> Any:
    if isinstance(value, Mapping):
        return {str(k): _plain(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_plain(v) for v in value]
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    return value


def _digest(value: Any) -> str:
    payload = json.dumps(_plain(value), sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")
    return sha256(payload).hexdigest()


@dataclass(frozen=True, slots=True)
class AnalysisCase:
    index: int
    kind: str
    overrides: Mapping[str, Any]
    parameters: Mapping[str, Any]
    case_id: str = ""

    def __post_init__(self) -> None:
        if self.index < 0:
            raise ValueError("case index must be nonnegative")
        if not self.kind:
            raise ValueError("case kind cannot be empty")
        if not self.case_id:
            object.__setattr__(self, "case_id", _digest({
                "kind": self.kind,
                "overrides": dict(self.overrides),
                "parameters": dict(self.parameters),
            })[:24])


@dataclass(frozen=True, slots=True)
class CampaignExecution:
    campaign_id: str
    kind: str
    requested_cases: int
    executed_cases: int
    resumed_cases: int
    failed_cases: int
    elapsed_seconds: float
    workers: int
    store_path: str


@dataclass(frozen=True, slots=True)
class _MissionPayload:
    mission_path: str
    case: AnalysisCase


def _run_mission_case(payload: _MissionPayload) -> StoredCase:
    t0 = time.perf_counter()
    try:
        document = load_mission(payload.mission_path).with_overrides(payload.case.overrides)
        report = MissionCompiler().compile(document).run()
        metrics: dict[str, Any] = {str(k): _plain(v) for k, v in report.outputs.items()}
        metrics["success"] = bool(report.success)
        metrics["end_time"] = float(report.end_time)
        metrics["final_vehicle_count"] = int(len(report.final_vehicles))
        status = "completed" if report.success else "failed"
        error = None if report.success else str(report.message)
    except Exception as exc:
        metrics = {"success": False}
        status = "failed"
        error = f"{type(exc).__name__}: {exc}"
    return StoredCase(
        payload.case.case_id, payload.case.index, payload.case.kind, status,
        dict(payload.case.parameters), metrics, error, time.perf_counter()-t0,
    )


@dataclass(frozen=True, slots=True)
class _OptimizationPayload:
    mission_path: str
    case: AnalysisCase


def _run_optimization_case(payload: _OptimizationPayload) -> StoredCase:
    t0 = time.perf_counter()
    try:
        document = load_mission(payload.mission_path).with_overrides(payload.case.overrides)
        result = MissionCompiler().optimize(document)
        metrics = {
            "success": bool(result.success),
            "objective": float(result.objective),
            "max_constraint_violation": float(result.max_constraint_violation),
            "evaluations": int(result.nfev),
            "iterations": int(result.nit),
            **{f"design.{k}": float(v) for k, v in result.design.items()},
            **{f"metric.{k}": float(v) for k, v in result.metrics.items() if np.isscalar(v)},
        }
        status = "completed" if result.success else "failed"
        error = None if result.success else str(result.message)
    except Exception as exc:
        metrics = {"success": False}
        status = "failed"
        error = f"{type(exc).__name__}: {exc}"
    return StoredCase(
        payload.case.case_id, payload.case.index, payload.case.kind, status,
        dict(payload.case.parameters), metrics, error, time.perf_counter()-t0,
    )


class MissionCampaignRunner:
    """Execute deterministic mission cases with checkpoint/restart."""

    def __init__(self, mission_path: str | Path, *, backend: ExecutionBackend | None = None,
                 store: SQLiteResultStore | None = None):
        self.mission_path = Path(mission_path).resolve()
        if not self.mission_path.exists():
            raise FileNotFoundError(self.mission_path)
        self.document = load_mission(self.mission_path)
        self.backend = backend or SerialBackend()
        self.store = store

    def run_cases(self, cases: Sequence[AnalysisCase], *, campaign_id: str,
                  kind: str, worker: Callable[[_MissionPayload], StoredCase] = _run_mission_case,
                  progress: Callable[[int, int, StoredCase], None] | None = None,
                  metadata: Mapping[str, Any] | None = None) -> CampaignExecution:
        if not cases:
            raise ValueError("campaign requires at least one case")
        own_store = self.store is None
        store = self.store or SQLiteResultStore(self.mission_path.with_suffix(f".{campaign_id}.sqlite"))
        try:
            store.begin_campaign(campaign_id, kind, mission_sha256=self.document.digest_sha256,
                                 metadata={"uniflight_version": _uniflight_version,
                                           "mission_path": str(self.mission_path), **dict(metadata or {})})
            completed = store.completed_case_ids(campaign_id)
            pending = [c for c in cases if c.case_id not in completed]
            payloads = [_MissionPayload(str(self.mission_path), c) for c in pending]
            # Optimization worker has a compatible payload layout.
            if worker is _run_optimization_case:
                payloads = [_OptimizationPayload(str(self.mission_path), c) for c in pending]  # type: ignore[assignment]
            t0 = time.perf_counter()
            failures = 0
            for count, result in enumerate(self.backend.map(worker, payloads), start=1):
                store.write_case(campaign_id, result)
                failures += int(result.status != "completed")
                if progress is not None:
                    progress(count, len(pending), result)
            elapsed = time.perf_counter()-t0
            return CampaignExecution(
                campaign_id, kind, len(cases), len(pending), len(cases)-len(pending),
                failures, elapsed, int(self.backend.workers), str(store.path),
            )
        finally:
            if own_store:
                store.close()


@dataclass(frozen=True, slots=True)
class SweepVariable:
    name: str
    pointer: str
    values: tuple[float, ...]

    def __post_init__(self):
        if not self.name or not self.pointer.startswith("/") or not self.values:
            raise ValueError("invalid sweep variable")
        if not all(np.isfinite(self.values)):
            raise ValueError("sweep values must be finite")


class ParameterSweep:
    def __init__(self, variables: Sequence[SweepVariable], *, mode: str = "cartesian"):
        self.variables = tuple(variables)
        if not self.variables:
            raise ValueError("sweep requires variables")
        if mode not in ("cartesian", "zip"):
            raise ValueError("sweep mode must be cartesian or zip")
        if mode == "zip" and len({len(v.values) for v in self.variables}) != 1:
            raise ValueError("zip sweep variables must have equal lengths")
        self.mode = mode

    def cases(self) -> tuple[AnalysisCase, ...]:
        rows = product(*(v.values for v in self.variables)) if self.mode == "cartesian" else zip(*(v.values for v in self.variables))
        out = []
        for i, row in enumerate(rows):
            params = {v.name: float(x) for v, x in zip(self.variables, row)}
            overrides = {v.pointer: float(x) for v, x in zip(self.variables, row)}
            out.append(AnalysisCase(i, "sweep", overrides, params))
        return tuple(out)


@dataclass(frozen=True, slots=True)
class MonteCarloVariable:
    name: str
    pointer: str
    dispersion: Dispersion


class MissionMonteCarlo:
    def __init__(self, variables: Sequence[MonteCarloVariable], *, cases: int, seed: int = 0,
                 mission_seed_pointer: str | None = None):
        if cases <= 0:
            raise ValueError("cases must be positive")
        self.variables = tuple(variables); self.n_cases = int(cases); self.seed = int(seed)
        self.mission_seed_pointer = mission_seed_pointer
        if not self.variables:
            raise ValueError("Monte Carlo requires dispersions")

    def cases(self) -> tuple[AnalysisCase, ...]:
        roots = np.random.SeedSequence(self.seed).spawn(self.n_cases)
        out = []
        for i, root in enumerate(roots):
            dispersion_ss, mission_ss = root.spawn(2)
            rng = np.random.default_rng(dispersion_ss)
            values = {v.name: v.dispersion.sample(rng) for v in self.variables}
            case_seed = int(mission_ss.generate_state(1, dtype=np.uint64)[0])
            values["_case_seed"] = case_seed
            overrides = {v.pointer: values[v.name] for v in self.variables}
            if self.mission_seed_pointer is not None:
                overrides[self.mission_seed_pointer] = case_seed
            out.append(AnalysisCase(i, "monte_carlo", overrides, values))
        return tuple(out)


@dataclass(frozen=True, slots=True)
class SobolVariable:
    name: str
    pointer: str
    lower: float
    upper: float

    def __post_init__(self):
        if not self.name or not self.pointer.startswith("/") or not np.isfinite([self.lower, self.upper]).all() or self.upper <= self.lower:
            raise ValueError("invalid Sobol variable")


@dataclass(frozen=True, slots=True)
class SobolIndices:
    names: tuple[str, ...]
    first_order: np.ndarray
    total_order: np.ndarray
    variance: float
    base_samples: int

    def to_json_dict(self) -> Mapping[str, Any]:
        return {
            "names": list(self.names), "first_order": self.first_order.tolist(),
            "total_order": self.total_order.tolist(), "variance": self.variance,
            "base_samples": self.base_samples,
        }


class SobolSensitivity:
    """Saltelli A/B/A_Bi design with first/total-order estimators."""
    def __init__(self, variables: Sequence[SobolVariable], *, base_samples: int = 128, seed: int = 0):
        self.variables = tuple(variables); self.base_samples = int(base_samples); self.seed = int(seed)
        if not self.variables or self.base_samples < 2:
            raise ValueError("Sobol analysis requires variables and at least two base samples")

    def cases(self) -> tuple[AnalysisCase, ...]:
        d = len(self.variables)
        # Generate 2d independent Sobol coordinates and split into A and B.
        sampler = qmc.Sobol(2*d, scramble=True, seed=self.seed)
        m = int(round(math.log2(self.base_samples)))
        if 2**m == self.base_samples:
            u = sampler.random_base2(m)
        else:
            u = sampler.random(self.base_samples)
        A, B = u[:, :d], u[:, d:]
        lo = np.array([v.lower for v in self.variables]); hi = np.array([v.upper for v in self.variables])
        A = lo + A*(hi-lo); B = lo + B*(hi-lo)
        rows: list[tuple[str, int, int | None, np.ndarray]] = []
        for r in range(self.base_samples):
            rows.append(("A", r, None, A[r])); rows.append(("B", r, None, B[r]))
            for j in range(d):
                x = A[r].copy(); x[j] = B[r, j]
                rows.append(("AB", r, j, x))
        out=[]
        for i, (role, row, dim, x) in enumerate(rows):
            params = {v.name: float(z) for v, z in zip(self.variables, x)}
            params.update({"_sobol_role": role, "_sobol_row": row, "_sobol_dim": -1 if dim is None else dim})
            overrides = {v.pointer: float(z) for v, z in zip(self.variables, x)}
            out.append(AnalysisCase(i, "sobol", overrides, params))
        return tuple(out)

    def analyze(self, stored_cases: Sequence[StoredCase], metric: str) -> SobolIndices:
        n, d = self.base_samples, len(self.variables)
        YA=np.full(n,np.nan); YB=np.full(n,np.nan); YAB=np.full((d,n),np.nan)
        for c in stored_cases:
            if c.status != "completed" or metric not in c.metrics:
                continue
            role=str(c.parameters.get("_sobol_role")); row=int(c.parameters.get("_sobol_row",-1)); dim=int(c.parameters.get("_sobol_dim",-1))
            y=float(c.metrics[metric])
            if role=="A": YA[row]=y
            elif role=="B": YB[row]=y
            elif role=="AB": YAB[dim,row]=y
        if not (np.isfinite(YA).all() and np.isfinite(YB).all() and np.isfinite(YAB).all()):
            raise ValueError("Sobol analysis requires every design case to complete with the requested metric")
        variance=float(np.var(np.concatenate([YA,YB]),ddof=1))
        if variance <= np.finfo(float).eps:
            raise ValueError("requested metric has zero variance")
        # Saltelli/Jansen estimators.
        first=np.array([np.mean(YB*(YAB[j]-YA))/variance for j in range(d)],dtype=float)
        total=np.array([0.5*np.mean((YA-YAB[j])**2)/variance for j in range(d)],dtype=float)
        return SobolIndices(tuple(v.name for v in self.variables),first,total,variance,n)


@dataclass(frozen=True, slots=True)
class OptimizationStart:
    name: str
    initial_values: Mapping[str, float]


class OptimizationBatch:
    """Parallel multistart optimization using the mission H declaration."""
    def __init__(self, starts: Sequence[OptimizationStart]):
        self.starts = tuple(starts)
        if not self.starts:
            raise ValueError("optimization batch requires starts")

    def cases(self, document: MissionDocument) -> tuple[AnalysisCase, ...]:
        opt = document.raw.get("optimization")
        if not isinstance(opt, Mapping):
            raise ValueError("mission has no optimization declaration")
        dvs = list(opt.get("design_variables", []))
        name_to_ptr = {str(d["name"]): f"/optimization/design_variables/{i}/initial" for i,d in enumerate(dvs)}
        out=[]
        for i,start in enumerate(self.starts):
            unknown=set(start.initial_values)-set(name_to_ptr)
            if unknown: raise KeyError(f"unknown optimization design variables: {sorted(unknown)}")
            overrides={name_to_ptr[k]:float(v) for k,v in start.initial_values.items()}
            out.append(AnalysisCase(i,"optimization_batch",overrides,{"start":start.name,**{k:float(v) for k,v in start.initial_values.items()}}))
        return tuple(out)


def summarize_numeric_metrics(cases: Sequence[StoredCase]) -> Mapping[str, Mapping[str, float]]:
    names=sorted({k for c in cases if c.status=="completed" for k,v in c.metrics.items()
                  if isinstance(v,(int,float,np.integer,np.floating)) and not isinstance(v,(bool,np.bool_))})
    out={}
    for name in names:
        x=np.array([float(c.metrics[name]) for c in cases if c.status=="completed" and name in c.metrics and np.isfinite(float(c.metrics[name]))])
        if x.size:
            out[name]={"mean":float(x.mean()),"std":float(x.std()),"min":float(x.min()),"median":float(np.median(x)),"max":float(x.max()),
                       "p05":float(np.quantile(x,.05)),"p95":float(np.quantile(x,.95))}
    return out


# Public worker aliases: module-level names are intentionally exported so the
# process backend remains portable under Windows/macOS spawn semantics.
mission_case_worker = _run_mission_case
optimization_case_worker = _run_optimization_case
