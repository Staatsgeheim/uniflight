from __future__ import annotations

"""General engineering-data tables for UniFlight Milestone K.

The numerical flight kernel uses coherent SI values.  This module adds the
metadata, interpolation policy, validity, uncertainty, provenance and storage
layer needed to make tabulated engineering models explicit and reproducible.

The core object is :class:`EngineeringTable`: an arbitrary-dimensional regular
rectilinear grid with one or more named outputs.  The table deliberately does
not know whether the data are aerodynamic, propulsion, material or terrain
quantities; domain-specific adapters live in ``data_models.py``.
"""

from dataclasses import dataclass, field, replace
from enum import Enum
from pathlib import Path
from typing import Mapping, Sequence, Iterable
import hashlib
import json
import math
import re

import numpy as np
from scipy.interpolate import RegularGridInterpolator


_NAME_RE = re.compile(r"^[A-Za-z_][A-Za-z0-9_.-]*$")


def _valid_name(name: str, what: str) -> str:
    name = str(name)
    if not _NAME_RE.match(name):
        raise ValueError(f"{what} name {name!r} must match {_NAME_RE.pattern}")
    return name


class InterpolationMethod(str, Enum):
    LINEAR = "linear"
    NEAREST = "nearest"


class ExtrapolationPolicy(str, Enum):
    ERROR = "error"
    CLAMP = "clamp"
    EXTRAPOLATE = "extrapolate"


class ValidityPolicy(str, Enum):
    ERROR = "error"
    FLAG = "flag"


@dataclass(frozen=True, slots=True)
class AxisMetadata:
    """Description of one regular-grid axis.

    ``period`` is expressed in the same coherent-SI unit as the axis.  If set,
    queries are wrapped into ``[values[0], values[0] + period)`` before domain
    handling.  The grid itself must span no more than one period.
    """

    name: str
    values: np.ndarray
    unit: str = "1"
    description: str = ""
    extrapolation: ExtrapolationPolicy = ExtrapolationPolicy.CLAMP
    period: float | None = None

    def __post_init__(self) -> None:
        _valid_name(self.name, "axis")
        a = np.asarray(self.values, dtype=float)
        if a.ndim != 1 or a.size < 2 or not np.all(np.isfinite(a)):
            raise ValueError(f"axis {self.name!r} must be a finite 1-D array with >=2 points")
        if np.any(np.diff(a) <= 0):
            raise ValueError(f"axis {self.name!r} must be strictly increasing")
        object.__setattr__(self, "values", a.copy())
        object.__setattr__(self, "extrapolation", ExtrapolationPolicy(self.extrapolation))
        if not isinstance(self.unit, str) or not self.unit:
            raise ValueError("axis unit must be a non-empty metadata string")
        if self.period is not None:
            p = float(self.period)
            if not np.isfinite(p) or p <= 0:
                raise ValueError("period must be finite and positive")
            span = float(a[-1] - a[0])
            if span > p * (1.0 + 1e-12):
                raise ValueError(f"periodic axis {self.name!r} spans more than one period")
            object.__setattr__(self, "period", p)

    @property
    def lower(self) -> float:
        return float(self.values[0])

    @property
    def upper(self) -> float:
        return float(self.values[-1])

    def wrap(self, value: float) -> float:
        x = float(value)
        if self.period is None:
            return x
        # Preserve an exact upper endpoint if supplied.  This matters for grids
        # that contain both sides of a periodic seam.
        if math.isclose(x, self.upper, rel_tol=0.0, abs_tol=8*np.finfo(float).eps*max(1.0, abs(x))):
            return self.upper
        return self.lower + ((x - self.lower) % self.period)


@dataclass(frozen=True, slots=True)
class UncertaintyMetadata:
    """Lightweight uncertainty annotation for one table output.

    It is intentionally metadata rather than a Monte-Carlo sampler.  The
    standard uncertainty reported at a query point is

    ``sqrt(absolute_sigma**2 + (relative_sigma*abs(value))**2)``.
    """

    distribution: str = "unspecified"
    absolute_sigma: float = 0.0
    relative_sigma: float = 0.0
    confidence: float | None = None
    correlation_group: str | None = None
    notes: str = ""

    def __post_init__(self) -> None:
        for name in ("absolute_sigma", "relative_sigma"):
            v = float(getattr(self, name))
            if not np.isfinite(v) or v < 0:
                raise ValueError(f"{name} must be finite and non-negative")
        if self.confidence is not None:
            c = float(self.confidence)
            if not np.isfinite(c) or not (0 < c <= 1):
                raise ValueError("confidence must lie in (0,1]")
        if not isinstance(self.distribution, str) or not self.distribution:
            raise ValueError("distribution must be a non-empty string")

    def standard_uncertainty(self, value: float) -> float:
        v = float(value)
        return float(math.hypot(self.absolute_sigma, self.relative_sigma * abs(v)))


@dataclass(frozen=True, slots=True)
class OutputMetadata:
    name: str
    unit: str = "1"
    description: str = ""
    uncertainty: UncertaintyMetadata | None = None

    def __post_init__(self) -> None:
        _valid_name(self.name, "output")
        if not isinstance(self.unit, str) or not self.unit:
            raise ValueError("output unit must be a non-empty metadata string")


@dataclass(frozen=True, slots=True)
class DataProvenance:
    """Provenance attached to a dataset or engineering table."""

    dataset_id: str
    version: str
    source: str = ""
    authors: tuple[str, ...] = ()
    citation: str = ""
    license: str = ""
    created_utc: str = ""
    source_sha256: str = ""
    notes: str = ""

    def __post_init__(self) -> None:
        _valid_name(self.dataset_id, "dataset")
        if not str(self.version):
            raise ValueError("dataset version must be non-empty")
        object.__setattr__(self, "authors", tuple(str(x) for x in self.authors))
        if self.source_sha256:
            s = self.source_sha256.lower()
            if len(s) != 64 or any(c not in "0123456789abcdef" for c in s):
                raise ValueError("source_sha256 must be a 64-character hexadecimal SHA-256")
            object.__setattr__(self, "source_sha256", s)

    @staticmethod
    def sha256_file(path: str | Path) -> str:
        h = hashlib.sha256()
        with open(path, "rb") as f:
            for chunk in iter(lambda: f.read(1 << 20), b""):
                h.update(chunk)
        return h.hexdigest()


@dataclass(frozen=True, slots=True)
class ValidityBound:
    axis: str
    lower: float | None = None
    upper: float | None = None

    def __post_init__(self) -> None:
        _valid_name(self.axis, "validity axis")
        if self.lower is not None and not np.isfinite(float(self.lower)):
            raise ValueError("validity lower bound must be finite")
        if self.upper is not None and not np.isfinite(float(self.upper)):
            raise ValueError("validity upper bound must be finite")
        if self.lower is not None and self.upper is not None and float(self.lower) > float(self.upper):
            raise ValueError("validity lower bound cannot exceed upper bound")


@dataclass(frozen=True, slots=True)
class ValidityEnvelope:
    bounds: tuple[ValidityBound, ...]
    policy: ValidityPolicy = ValidityPolicy.FLAG
    description: str = ""

    def __post_init__(self) -> None:
        object.__setattr__(self, "bounds", tuple(self.bounds))
        object.__setattr__(self, "policy", ValidityPolicy(self.policy))
        names = [b.axis for b in self.bounds]
        if len(names) != len(set(names)):
            raise ValueError("validity envelope contains duplicate axis bounds")

    def violations(self, coordinates: Mapping[str, float]) -> tuple[str, ...]:
        out: list[str] = []
        for b in self.bounds:
            if b.axis not in coordinates:
                out.append(f"missing validity coordinate {b.axis}")
                continue
            x = float(coordinates[b.axis])
            if b.lower is not None and x < b.lower:
                out.append(f"{b.axis}={x:g} < valid lower {float(b.lower):g}")
            if b.upper is not None and x > b.upper:
                out.append(f"{b.axis}={x:g} > valid upper {float(b.upper):g}")
        return tuple(out)


@dataclass(frozen=True, slots=True)
class TableQueryResult:
    requested_coordinates: Mapping[str, float]
    effective_coordinates: Mapping[str, float]
    values: Mapping[str, float]
    standard_uncertainty: Mapping[str, float]
    in_table_domain: bool
    validity_ok: bool
    adjusted_axes: tuple[str, ...]
    extrapolated_axes: tuple[str, ...]
    validity_violations: tuple[str, ...]
    dataset_id: str | None = None
    dataset_version: str | None = None

    def value(self, name: str) -> float:
        return float(self.values[name])


@dataclass(slots=True)
class EngineeringTable:
    """Arbitrary N-D rectilinear engineering lookup table.

    All numeric coordinates/outputs are assumed to be coherent SI.  Unit
    strings are metadata used for traceability and configuration validation.
    """

    axes: tuple[AxisMetadata, ...]
    outputs: Mapping[str, np.ndarray]
    output_metadata: Mapping[str, OutputMetadata] = field(default_factory=dict)
    interpolation: InterpolationMethod = InterpolationMethod.LINEAR
    validity: ValidityEnvelope | None = None
    provenance: DataProvenance | None = None
    description: str = ""
    _interpolators: dict[str, RegularGridInterpolator] = field(init=False, repr=False)

    def __post_init__(self) -> None:
        self.axes = tuple(self.axes)
        if not self.axes:
            raise ValueError("engineering table requires at least one axis")
        names = [a.name for a in self.axes]
        if len(names) != len(set(names)):
            raise ValueError("engineering table axis names must be unique")
        self.interpolation = InterpolationMethod(self.interpolation)
        shape = tuple(a.values.size for a in self.axes)
        copied: dict[str, np.ndarray] = {}
        for name, values in dict(self.outputs).items():
            _valid_name(name, "output")
            a = np.asarray(values, dtype=float)
            if a.shape != shape:
                raise ValueError(f"output {name!r} has shape {a.shape}, expected {shape}")
            if not np.all(np.isfinite(a)):
                raise ValueError(f"output {name!r} contains non-finite values")
            copied[name] = a.copy()
        if not copied:
            raise ValueError("engineering table requires at least one output")
        self.outputs = copied

        meta: dict[str, OutputMetadata] = dict(self.output_metadata)
        unknown = set(meta) - set(copied)
        if unknown:
            raise ValueError(f"output metadata supplied for unknown outputs: {sorted(unknown)}")
        for name in copied:
            if name not in meta:
                meta[name] = OutputMetadata(name=name)
            elif meta[name].name != name:
                raise ValueError(f"metadata key {name!r} disagrees with OutputMetadata.name={meta[name].name!r}")
        self.output_metadata = meta

        if self.validity is not None:
            axis_names = set(names)
            for b in self.validity.bounds:
                if b.axis not in axis_names:
                    raise ValueError(f"validity envelope references unknown axis {b.axis!r}")

        grid = tuple(a.values for a in self.axes)
        self._interpolators = {
            name: RegularGridInterpolator(
                grid, values, method=self.interpolation.value,
                bounds_error=False, fill_value=None,
            )
            for name, values in copied.items()
        }

    @property
    def axis_names(self) -> tuple[str, ...]:
        return tuple(a.name for a in self.axes)

    @property
    def output_names(self) -> tuple[str, ...]:
        return tuple(self.outputs.keys())

    @property
    def shape(self) -> tuple[int, ...]:
        return tuple(a.values.size for a in self.axes)

    def _coordinate_mapping(self, coordinates: Mapping[str, float] | Sequence[float]) -> dict[str, float]:
        if isinstance(coordinates, Mapping):
            extra = set(coordinates) - set(self.axis_names)
            missing = set(self.axis_names) - set(coordinates)
            if extra or missing:
                raise ValueError(f"coordinate keys mismatch; missing={sorted(missing)}, extra={sorted(extra)}")
            result = {name: float(coordinates[name]) for name in self.axis_names}
        else:
            seq = tuple(coordinates)
            if len(seq) != len(self.axes):
                raise ValueError(f"expected {len(self.axes)} coordinates, got {len(seq)}")
            result = {axis.name: float(x) for axis, x in zip(self.axes, seq)}
        if not all(np.isfinite(v) for v in result.values()):
            raise ValueError("table coordinates must be finite")
        return result

    def _prepare_point(self, coordinates: Mapping[str, float] | Sequence[float]) -> tuple[dict[str,float],dict[str,float],tuple[str,...],tuple[str,...],bool]:
        requested = self._coordinate_mapping(coordinates)
        effective: dict[str,float] = {}
        adjusted: list[str] = []
        extrapolated: list[str] = []
        in_domain = True
        for axis in self.axes:
            raw = requested[axis.name]
            x = axis.wrap(raw)
            if x != raw:
                adjusted.append(axis.name)
            if x < axis.lower or x > axis.upper:
                in_domain = False
                policy = axis.extrapolation
                if policy is ExtrapolationPolicy.ERROR:
                    raise ValueError(
                        f"coordinate {axis.name}={x:g} outside table domain [{axis.lower:g},{axis.upper:g}]"
                    )
                if policy is ExtrapolationPolicy.CLAMP:
                    x = float(np.clip(x, axis.lower, axis.upper))
                    if axis.name not in adjusted:
                        adjusted.append(axis.name)
                elif policy is ExtrapolationPolicy.EXTRAPOLATE:
                    extrapolated.append(axis.name)
            effective[axis.name] = x
        return requested, effective, tuple(adjusted), tuple(extrapolated), in_domain

    def query(self, coordinates: Mapping[str, float] | Sequence[float]) -> TableQueryResult:
        requested, effective, adjusted, extrapolated, in_domain = self._prepare_point(coordinates)
        violations: tuple[str,...] = ()
        if self.validity is not None:
            violations = self.validity.violations(effective)
            if violations and self.validity.policy is ValidityPolicy.ERROR:
                raise ValueError("query outside declared validity envelope: " + "; ".join(violations))
        p = np.asarray([effective[a.name] for a in self.axes], dtype=float)
        values = {
            name: float(np.asarray(interp(p)).reshape(-1)[0])
            for name, interp in self._interpolators.items()
        }
        uncertainties = {
            name: (meta.uncertainty.standard_uncertainty(values[name]) if meta.uncertainty else 0.0)
            for name, meta in self.output_metadata.items()
        }
        provenance = self.provenance
        return TableQueryResult(
            requested_coordinates=requested,
            effective_coordinates=effective,
            values=values,
            standard_uncertainty=uncertainties,
            in_table_domain=in_domain,
            validity_ok=not violations,
            adjusted_axes=adjusted,
            extrapolated_axes=extrapolated,
            validity_violations=violations,
            dataset_id=None if provenance is None else provenance.dataset_id,
            dataset_version=None if provenance is None else provenance.version,
        )

    def query_many(self, points: Iterable[Mapping[str, float] | Sequence[float]]) -> tuple[TableQueryResult, ...]:
        return tuple(self.query(p) for p in points)

    def derivative(self, output: str, coordinates: Mapping[str,float] | Sequence[float], axis: str, step: float | None = None) -> float:
        """Finite-difference partial derivative of a table output.

        This utility is primarily for gravity gradients and terrain normals.
        It honors periodic wrapping and table extrapolation policy.
        """
        if output not in self.outputs:
            raise KeyError(output)
        if axis not in self.axis_names:
            raise KeyError(axis)
        c = self._coordinate_mapping(coordinates)
        ax = self.axes[self.axis_names.index(axis)]
        if step is None:
            spacing = np.diff(ax.values)
            step = max(1e-9, float(np.median(spacing)) * 1e-3)
        h = float(step)
        if not np.isfinite(h) or h <= 0:
            raise ValueError("finite-difference step must be finite and positive")
        cp, cm = dict(c), dict(c)
        cp[axis] += h
        cm[axis] -= h
        try:
            fp = self.query(cp).value(output)
            fm = self.query(cm).value(output)
            return float((fp - fm) / (2*h))
        except ValueError:
            # One-sided fallback for strict ERROR table boundaries.
            f0 = self.query(c).value(output)
            try:
                fp = self.query(cp).value(output)
                return float((fp - f0) / h)
            except ValueError:
                fm = self.query(cm).value(output)
                return float((f0 - fm) / h)

    def _manifest_dict(self, include_provenance: bool = True) -> dict:
        validity = None
        if self.validity is not None:
            validity = {
                "policy": self.validity.policy.value,
                "description": self.validity.description,
                "bounds": [
                    {"axis": b.axis, "lower": b.lower, "upper": b.upper}
                    for b in self.validity.bounds
                ],
            }
        provenance = None
        if include_provenance and self.provenance is not None:
            provenance = {
                "dataset_id": self.provenance.dataset_id,
                "version": self.provenance.version,
                "source": self.provenance.source,
                "authors": list(self.provenance.authors),
                "citation": self.provenance.citation,
                "license": self.provenance.license,
                "created_utc": self.provenance.created_utc,
                "source_sha256": self.provenance.source_sha256,
                "notes": self.provenance.notes,
            }
        return {
            "format": "uniflight-engineering-table-v1",
            "description": self.description,
            "interpolation": self.interpolation.value,
            "axes": [
                {
                    "name": a.name, "unit": a.unit, "description": a.description,
                    "extrapolation": a.extrapolation.value, "period": a.period,
                    "array_key": f"axis__{i}",
                }
                for i, a in enumerate(self.axes)
            ],
            "outputs": [
                {
                    "name": name,
                    "unit": self.output_metadata[name].unit,
                    "description": self.output_metadata[name].description,
                    "array_key": f"output__{name}",
                    "uncertainty": None if self.output_metadata[name].uncertainty is None else {
                        "distribution": self.output_metadata[name].uncertainty.distribution,
                        "absolute_sigma": self.output_metadata[name].uncertainty.absolute_sigma,
                        "relative_sigma": self.output_metadata[name].uncertainty.relative_sigma,
                        "confidence": self.output_metadata[name].uncertainty.confidence,
                        "correlation_group": self.output_metadata[name].uncertainty.correlation_group,
                        "notes": self.output_metadata[name].uncertainty.notes,
                    },
                }
                for name in self.outputs
            ],
            "validity": validity,
            "provenance": provenance,
        }

    def content_sha256(self) -> str:
        h = hashlib.sha256()
        manifest = json.dumps(self._manifest_dict(), sort_keys=True, separators=(",", ":")).encode("utf-8")
        h.update(manifest)
        for axis in self.axes:
            h.update(np.ascontiguousarray(axis.values, dtype="<f8").tobytes())
        for name in sorted(self.outputs):
            h.update(name.encode("utf-8"))
            h.update(np.ascontiguousarray(self.outputs[name], dtype="<f8").tobytes())
        return h.hexdigest()

    def to_npz(self, path: str | Path) -> Path:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        manifest = self._manifest_dict()
        manifest["content_sha256"] = self.content_sha256()
        arrays: dict[str, np.ndarray] = {
            f"axis__{i}": axis.values for i, axis in enumerate(self.axes)
        }
        arrays.update({f"output__{name}": values for name, values in self.outputs.items()})
        arrays["manifest_json"] = np.asarray(json.dumps(manifest, sort_keys=True))
        np.savez_compressed(path, **arrays)
        return path

    @classmethod
    def from_npz(cls, path: str | Path, *, verify_checksum: bool = True) -> "EngineeringTable":
        path = Path(path)
        with np.load(path, allow_pickle=False) as z:
            manifest = json.loads(str(np.asarray(z["manifest_json"]).item()))
            if manifest.get("format") != "uniflight-engineering-table-v1":
                raise ValueError("unsupported engineering-table file format")
            axes = tuple(
                AxisMetadata(
                    name=a["name"], values=np.asarray(z[a["array_key"]], dtype=float),
                    unit=a.get("unit", "1"), description=a.get("description", ""),
                    extrapolation=ExtrapolationPolicy(a.get("extrapolation", "clamp")),
                    period=a.get("period"),
                ) for a in manifest["axes"]
            )
            outputs: dict[str,np.ndarray] = {}
            output_meta: dict[str,OutputMetadata] = {}
            for o in manifest["outputs"]:
                outputs[o["name"]] = np.asarray(z[o["array_key"]], dtype=float)
                u = o.get("uncertainty")
                uncertainty = None if u is None else UncertaintyMetadata(**u)
                output_meta[o["name"]] = OutputMetadata(
                    o["name"], o.get("unit", "1"), o.get("description", ""), uncertainty
                )
            validity_data = manifest.get("validity")
            validity = None
            if validity_data is not None:
                validity = ValidityEnvelope(
                    tuple(ValidityBound(**b) for b in validity_data.get("bounds", [])),
                    policy=ValidityPolicy(validity_data.get("policy", "flag")),
                    description=validity_data.get("description", ""),
                )
            p = manifest.get("provenance")
            provenance = None if p is None else DataProvenance(
                dataset_id=p["dataset_id"], version=p["version"], source=p.get("source", ""),
                authors=tuple(p.get("authors", [])), citation=p.get("citation", ""),
                license=p.get("license", ""), created_utc=p.get("created_utc", ""),
                source_sha256=p.get("source_sha256", ""), notes=p.get("notes", ""),
            )
            table = cls(
                axes=axes, outputs=outputs, output_metadata=output_meta,
                interpolation=InterpolationMethod(manifest.get("interpolation", "linear")),
                validity=validity, provenance=provenance,
                description=manifest.get("description", ""),
            )
        if verify_checksum:
            expected = manifest.get("content_sha256")
            if expected and table.content_sha256() != expected:
                raise ValueError("engineering-table content checksum mismatch")
        return table


@dataclass(slots=True)
class EngineeringDataCatalog:
    """In-memory provenance-aware dataset registry.

    Resolution never silently selects an arbitrary version.  Omitting
    ``version`` succeeds only when exactly one version of a dataset is loaded.
    """

    _tables: dict[tuple[str,str], EngineeringTable] = field(default_factory=dict)

    def register(self, table: EngineeringTable, *, replace_existing: bool = False) -> None:
        if table.provenance is None:
            raise ValueError("catalogued tables require DataProvenance")
        key = (table.provenance.dataset_id, table.provenance.version)
        if key in self._tables and not replace_existing:
            raise KeyError(f"dataset {key[0]!r} version {key[1]!r} already registered")
        self._tables[key] = table

    def load_npz(self, path: str | Path, *, replace_existing: bool = False, verify_checksum: bool = True) -> EngineeringTable:
        table = EngineeringTable.from_npz(path, verify_checksum=verify_checksum)
        self.register(table, replace_existing=replace_existing)
        return table

    def versions(self, dataset_id: str) -> tuple[str,...]:
        return tuple(sorted(v for (d,v) in self._tables if d == dataset_id))

    def resolve(self, dataset_id: str, version: str | None = None) -> EngineeringTable:
        if version is not None:
            try:
                return self._tables[(dataset_id, version)]
            except KeyError as e:
                raise KeyError(f"dataset {dataset_id!r} version {version!r} not registered") from e
        versions = self.versions(dataset_id)
        if not versions:
            raise KeyError(f"dataset {dataset_id!r} not registered")
        if len(versions) != 1:
            raise KeyError(
                f"dataset {dataset_id!r} has versions {versions}; version must be specified explicitly"
            )
        return self._tables[(dataset_id, versions[0])]

    def inventory(self) -> tuple[tuple[str,str,str], ...]:
        return tuple(sorted(
            (dataset_id, version, table.content_sha256())
            for (dataset_id,version), table in self._tables.items()
        ))


def load_long_form_csv(
    path: str | Path,
    *,
    axis_names: Sequence[str],
    output_names: Sequence[str],
    axis_units: Mapping[str,str] | None = None,
    output_metadata: Mapping[str,OutputMetadata] | None = None,
    extrapolation: Mapping[str,ExtrapolationPolicy | str] | ExtrapolationPolicy | str = ExtrapolationPolicy.CLAMP,
    interpolation: InterpolationMethod | str = InterpolationMethod.LINEAR,
    validity: ValidityEnvelope | None = None,
    provenance: DataProvenance | None = None,
    delimiter: str = ",",
    description: str = "",
) -> EngineeringTable:
    """Load a complete Cartesian N-D grid from a human-readable long-form CSV.

    Each row contains all axis coordinates followed by one or more outputs.  The
    row order is arbitrary. Duplicate grid points and incomplete Cartesian
    products are rejected so no silent data holes can enter a flight model.
    """
    import csv
    axis_names = tuple(_valid_name(x,"axis") for x in axis_names)
    output_names = tuple(_valid_name(x,"output") for x in output_names)
    if not axis_names or not output_names:
        raise ValueError("axis_names and output_names must be non-empty")
    rows=[]
    with open(path,"r",newline="",encoding="utf-8") as f:
        reader=csv.DictReader(f,delimiter=delimiter)
        required=set(axis_names)|set(output_names)
        missing=required-set(reader.fieldnames or ())
        if missing:
            raise ValueError(f"CSV missing required columns {sorted(missing)}")
        for row in reader:
            rows.append({k:float(row[k]) for k in required})
    if not rows:
        raise ValueError("CSV contains no data rows")
    units=dict(axis_units or {})
    if isinstance(extrapolation,(str,ExtrapolationPolicy)):
        policies={name:ExtrapolationPolicy(extrapolation) for name in axis_names}
    else:
        policies={name:ExtrapolationPolicy(extrapolation.get(name,ExtrapolationPolicy.CLAMP)) for name in axis_names}
    axis_values={name:np.array(sorted({r[name] for r in rows}),dtype=float) for name in axis_names}
    axes=tuple(AxisMetadata(name,axis_values[name],units.get(name,"1"),extrapolation=policies[name]) for name in axis_names)
    shape=tuple(len(axis_values[n]) for n in axis_names)
    outputs={name:np.full(shape,np.nan,dtype=float) for name in output_names}
    occupied=np.zeros(shape,dtype=bool)
    for row in rows:
        idx=tuple(int(np.searchsorted(axis_values[name],row[name])) for name in axis_names)
        if occupied[idx]:
            raise ValueError(f"duplicate CSV grid point at {tuple(row[n] for n in axis_names)}")
        occupied[idx]=True
        for name in output_names:
            outputs[name][idx]=row[name]
    if not np.all(occupied):
        raise ValueError(f"CSV does not form a complete Cartesian grid; missing {int(np.size(occupied)-occupied.sum())} point(s)")
    return EngineeringTable(
        axes,outputs,dict(output_metadata or {}),InterpolationMethod(interpolation),
        validity,provenance,description,
    )


def save_long_form_csv(table: EngineeringTable, path: str | Path, *, delimiter: str = ",") -> Path:
    """Write a table to long-form CSV. Metadata remains in the native NPZ manifest."""
    import csv
    path=Path(path); path.parent.mkdir(parents=True,exist_ok=True)
    headers=list(table.axis_names)+list(table.output_names)
    with open(path,"w",newline="",encoding="utf-8") as f:
        writer=csv.DictWriter(f,fieldnames=headers,delimiter=delimiter)
        writer.writeheader()
        for idx in np.ndindex(table.shape):
            row={axis.name:float(axis.values[i]) for axis,i in zip(table.axes,idx)}
            row.update({name:float(values[idx]) for name,values in table.outputs.items()})
            writer.writerow(row)
    return path
