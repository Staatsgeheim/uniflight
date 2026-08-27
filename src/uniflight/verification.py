from __future__ import annotations

from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Callable, Iterable, Mapping, Sequence
import json
import math
import numpy as np


@dataclass(frozen=True, slots=True)
class TolerancePolicy:
    """Explicit scalar tolerance policy used by verification comparisons."""
    absolute: float = 0.0
    relative: float = 0.0
    scale_floor: float = 0.0

    def __post_init__(self) -> None:
        if self.absolute < 0 or self.relative < 0 or self.scale_floor < 0:
            raise ValueError("verification tolerances must be non-negative")

    def allowed(self, reference_scale: float) -> float:
        scale = max(abs(float(reference_scale)), self.scale_floor)
        return self.absolute + self.relative * scale

    def accepts(self, error: float, reference_scale: float) -> bool:
        return abs(float(error)) <= self.allowed(reference_scale)


@dataclass(frozen=True, slots=True)
class VerificationResult:
    case_id: str
    title: str
    category: str
    status: str
    error: float | None = None
    tolerance: float | None = None
    reference: float | None = None
    actual: float | None = None
    details: Mapping[str, object] = field(default_factory=dict)

    @property
    def passed(self) -> bool:
        return self.status == "PASS"

    @property
    def skipped(self) -> bool:
        return self.status == "SKIP"

    def to_dict(self) -> dict[str, object]:
        d = asdict(self)
        d["passed"] = self.passed
        d["skipped"] = self.skipped
        return d


@dataclass(frozen=True, slots=True)
class VerificationReport:
    results: tuple[VerificationResult, ...]
    metadata: Mapping[str, object] = field(default_factory=dict)

    @property
    def passed(self) -> int:
        return sum(r.status == "PASS" for r in self.results)

    @property
    def failed(self) -> int:
        return sum(r.status == "FAIL" for r in self.results)

    @property
    def skipped(self) -> int:
        return sum(r.status == "SKIP" for r in self.results)

    @property
    def success(self) -> bool:
        return self.failed == 0

    def to_dict(self) -> dict[str, object]:
        return {
            "metadata": dict(self.metadata),
            "summary": {
                "total": len(self.results),
                "passed": self.passed,
                "failed": self.failed,
                "skipped": self.skipped,
                "success": self.success,
            },
            "results": [r.to_dict() for r in self.results],
        }

    def write_json(self, path: str | Path) -> Path:
        p = Path(path)
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(json.dumps(self.to_dict(), indent=2) + "\n", encoding="utf-8")
        return p

    def write_markdown(self, path: str | Path) -> Path:
        p = Path(path)
        p.parent.mkdir(parents=True, exist_ok=True)
        lines = [
            "# UniFlight formal verification report",
            "",
            f"- Total: **{len(self.results)}**",
            f"- Passed: **{self.passed}**",
            f"- Failed: **{self.failed}**",
            f"- Skipped: **{self.skipped}**",
            "",
            "| ID | Status | Category | Title | Error | Tolerance |",
            "|---|---:|---|---|---:|---:|",
        ]
        for r in self.results:
            e = "" if r.error is None else f"{r.error:.6g}"
            t = "" if r.tolerance is None else f"{r.tolerance:.6g}"
            lines.append(f"| {r.case_id} | {r.status} | {r.category} | {r.title} | {e} | {t} |")
        p.write_text("\n".join(lines) + "\n", encoding="utf-8")
        return p


@dataclass(frozen=True, slots=True)
class RegressionBaseline:
    baseline_id: str
    values: Mapping[str, float]
    tolerances: Mapping[str, TolerancePolicy]
    provenance: Mapping[str, object] = field(default_factory=dict)

    def compare(self, actual: Mapping[str, float]) -> tuple[VerificationResult, ...]:
        out: list[VerificationResult] = []
        for key, ref in self.values.items():
            if key not in actual:
                out.append(VerificationResult(
                    f"{self.baseline_id}:{key}", key, "regression", "FAIL",
                    details={"reason": "missing metric", "provenance": dict(self.provenance)},
                ))
                continue
            policy = self.tolerances.get(key, TolerancePolicy())
            act = float(actual[key])
            err = abs(act - float(ref))
            tol = policy.allowed(float(ref))
            out.append(VerificationResult(
                f"{self.baseline_id}:{key}", key, "regression",
                "PASS" if err <= tol else "FAIL",
                error=err, tolerance=tol, reference=float(ref), actual=act,
                details={"provenance": dict(self.provenance)},
            ))
        return tuple(out)


@dataclass(frozen=True, slots=True)
class ReferenceTimeHistory:
    time: np.ndarray
    channels: Mapping[str, np.ndarray]

    def __post_init__(self) -> None:
        t = np.asarray(self.time, dtype=float)
        if t.ndim != 1 or t.size < 2 or not np.all(np.isfinite(t)) or not np.all(np.diff(t) > 0):
            raise ValueError("reference time must be finite, 1-D, and strictly increasing")
        object.__setattr__(self, "time", t.copy())
        clean: dict[str, np.ndarray] = {}
        for name, values in self.channels.items():
            a = np.asarray(values, dtype=float)
            if a.shape != t.shape or not np.all(np.isfinite(a)):
                raise ValueError(f"invalid channel {name!r}")
            clean[str(name)] = a.copy()
        object.__setattr__(self, "channels", clean)

    @classmethod
    def from_csv(cls, path: str | Path, *, time_column: str = "time", channels: Sequence[str] | None = None) -> "ReferenceTimeHistory":
        data = np.genfromtxt(path, delimiter=",", names=True, dtype=float, encoding="utf-8")
        names = data.dtype.names or ()
        if time_column not in names:
            raise KeyError(f"time column {time_column!r} not found")
        selected = tuple(channels) if channels else tuple(n for n in names if n != time_column)
        return cls(np.asarray(data[time_column], float), {n: np.asarray(data[n], float) for n in selected})


def compare_time_histories(
    reference: ReferenceTimeHistory,
    actual: ReferenceTimeHistory,
    *,
    channels: Sequence[str] | None = None,
    tolerance: TolerancePolicy = TolerancePolicy(absolute=1e-9, relative=1e-9),
) -> tuple[VerificationResult, ...]:
    selected = tuple(channels) if channels else tuple(reference.channels)
    out: list[VerificationResult] = []
    if actual.time[0] > reference.time[0] or actual.time[-1] < reference.time[-1]:
        raise ValueError("actual time history does not span the reference interval")
    for name in selected:
        if name not in reference.channels or name not in actual.channels:
            out.append(VerificationResult(f"external:{name}", name, "external-reference", "FAIL", details={"reason": "missing channel"}))
            continue
        interp = np.interp(reference.time, actual.time, actual.channels[name])
        ref = reference.channels[name]
        err = float(np.max(np.abs(interp - ref)))
        scale = float(np.max(np.abs(ref)))
        tol = tolerance.allowed(scale)
        out.append(VerificationResult(
            f"external:{name}", name, "external-reference",
            "PASS" if err <= tol else "FAIL",
            error=err, tolerance=tol, reference=scale,
            details={"norm": "L_inf", "samples": int(reference.time.size)},
        ))
    return tuple(out)


def observed_order(step_sizes: Sequence[float], errors: Sequence[float]) -> float:
    h = np.asarray(step_sizes, dtype=float)
    e = np.asarray(errors, dtype=float)
    if h.ndim != 1 or e.shape != h.shape or h.size < 2 or np.any(h <= 0) or np.any(e <= 0):
        raise ValueError("positive step sizes/errors with matching shape required")
    slope, _ = np.polyfit(np.log(h), np.log(e), 1)
    return float(slope)


def scalar_result(case_id: str, title: str, category: str, *, actual: float, reference: float, policy: TolerancePolicy, details: Mapping[str, object] | None = None) -> VerificationResult:
    err = abs(float(actual) - float(reference))
    tol = policy.allowed(float(reference))
    return VerificationResult(
        case_id, title, category, "PASS" if err <= tol else "FAIL",
        error=err, tolerance=tol, reference=float(reference), actual=float(actual), details=details or {},
    )
