from __future__ import annotations

"""Milestone L declarative mission definition language.

The mission layer intentionally sits *above* the trusted flight kernel.  It
loads YAML/TOML/JSON, validates references and deterministic model choices,
then compiles the document into the same :class:`VehicleSpec`, dynamics,
integrators, events, engineering-data catalog and optimization objects that a
Python user could construct manually.

Version 1.0 is deliberately strict: unknown keys are rejected in the core
sections and engineering datasets must be version pinned.  The model registry
is extensible so Milestone M can expose the same compiler to third-party
plugins without redesigning the DSL.
"""

from dataclasses import dataclass, field
from pathlib import Path
from types import MappingProxyType
from typing import Any, Callable, Mapping, Sequence
from copy import deepcopy
from hashlib import sha256
import json
import math
import tomllib
import numpy as np

try:  # PyYAML is a project dependency in Milestone L.
    import yaml
except Exception:  # pragma: no cover - dependency error handled explicitly
    yaml = None

from ._version import __version__ as _uniflight_version
from .state import StateSchema, core_3dof_schema, core_6dof_schema
from .dof import map_state_fields, promote_3dof_to_6dof, demote_6dof_to_3dof
from .bodies import SphericalBody
from .gases import GasSpecies, GasMixture
from .atmosphere import VacuumAtmosphere, IsothermalHydrostaticAtmosphere
from .environment import PlanetaryEnvironment
from .gravity import PointMassGravity
from .dynamics import DynamicsAssembler, TranslationalKinematics, QuaternionKinematics, IdealRocket, RigidBody6DOFDynamics
from .mass_properties import ConstantMassProperties
from .wrenches import Wrench
from .frames import body_to_inertial_matrix
from .integrators import ScipyIVPIntegrator, SolverConfig, FixedStepRK4Integrator, FixedStepRK4Config
from .engineering_data import EngineeringDataCatalog
from .universe import VehicleEvent, VehicleSpec, UniverseMutation, MultiVehicleUniverseEngine, UniverseResult
from .separation import separate_two_rigid_bodies
from .plugins import (
    PLUGIN_API_VERSION, PluginManager, PluginRequirement, PluginError,
    CapabilityRegistration,
)
from .optimization import (
    DesignVariable, DesignSpace, MetricObjective, MetricConstraint,
    TrajectoryProblem, TrajectoryOptimizer, OptimizationSettings,
)


MISSION_FORMAT_VERSION = "1.0"


class MissionValidationError(ValueError):
    """Raised when a declarative mission is ambiguous or structurally invalid."""


class MissionCompilationError(RuntimeError):
    """Raised when a valid document cannot be compiled into runtime models."""


def _deep_freeze(value: Any) -> Any:
    """Recursively freeze mission data so its provenance digest cannot drift."""
    if isinstance(value, Mapping):
        return MappingProxyType({str(k): _deep_freeze(v) for k, v in value.items()})
    if isinstance(value, list):
        return tuple(_deep_freeze(v) for v in value)
    if isinstance(value, tuple):
        return tuple(_deep_freeze(v) for v in value)
    if isinstance(value, np.ndarray):
        arr = np.asarray(value).copy()
        arr.setflags(write=False)
        return arr
    return value


def _deep_thaw(value: Any) -> Any:
    """Return a fully mutable plain-Python copy of recursively frozen data."""
    if isinstance(value, Mapping):
        return {str(k): _deep_thaw(v) for k, v in value.items()}
    if isinstance(value, tuple):
        return [_deep_thaw(v) for v in value]
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    return deepcopy(value)


def _freeze_mapping(value: Mapping[str, Any] | None) -> Mapping[str, Any]:
    return MappingProxyType(dict(value or {}))


def _finite_float(value: Any, path: str) -> float:
    try:
        x = float(value)
    except Exception as exc:
        raise MissionValidationError(f"{path} must be numeric") from exc
    if not math.isfinite(x):
        raise MissionValidationError(f"{path} must be finite")
    return x


def _strict_keys(mapping: Mapping[str, Any], allowed: set[str], path: str) -> None:
    unknown = set(mapping) - allowed
    if unknown:
        raise MissionValidationError(f"{path} contains unknown key(s): {sorted(unknown)}")


def _require(mapping: Mapping[str, Any], key: str, path: str) -> Any:
    if key not in mapping:
        raise MissionValidationError(f"{path}.{key} is required")
    return mapping[key]


def _as_mapping(value: Any, path: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise MissionValidationError(f"{path} must be a mapping")
    return value


def _as_sequence(value: Any, path: str) -> Sequence[Any]:
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence):
        raise MissionValidationError(f"{path} must be a sequence")
    return value


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

def _json_canonical(value: Any) -> bytes:
    return json.dumps(_plain(value), sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode("utf-8")


def mission_sha256(raw: Mapping[str, Any]) -> str:
    return sha256(_json_canonical(raw)).hexdigest()


def _load_raw(path: Path) -> Mapping[str, Any]:
    suffix = path.suffix.lower()
    if suffix in (".yaml", ".yml"):
        if yaml is None:
            raise RuntimeError("PyYAML is required to load YAML missions")
        with path.open("r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
    elif suffix == ".toml":
        with path.open("rb") as f:
            data = tomllib.load(f)
    elif suffix == ".json":
        with path.open("r", encoding="utf-8") as f:
            data = json.load(f)
    else:
        raise MissionValidationError(f"unsupported mission format {suffix!r}; use YAML, TOML, or JSON")
    if not isinstance(data, Mapping):
        raise MissionValidationError("mission document root must be a mapping")
    return data


def _parse_pointer(pointer: str) -> list[str]:
    if not isinstance(pointer, str) or not pointer.startswith("/"):
        raise MissionValidationError("mission override paths must be RFC-6901 JSON Pointers beginning with '/'")
    if pointer == "/":
        return [""]
    return [part.replace("~1", "/").replace("~0", "~") for part in pointer[1:].split("/")]


def pointer_get(root: Any, pointer: str) -> Any:
    node = root
    for part in _parse_pointer(pointer):
        if isinstance(node, Mapping):
            if part not in node:
                raise KeyError(pointer)
            node = node[part]
        elif isinstance(node, (list, tuple)):
            node = node[int(part)]
        else:
            raise KeyError(pointer)
    return node


def pointer_set(root: Any, pointer: str, value: Any) -> None:
    parts = _parse_pointer(pointer)
    if not parts:
        raise MissionValidationError("cannot replace mission root")
    node = root
    for part in parts[:-1]:
        if isinstance(node, Mapping):
            if part not in node:
                raise KeyError(pointer)
            node = node[part]
        elif isinstance(node, list):
            node = node[int(part)]
        else:
            raise KeyError(pointer)
    last = parts[-1]
    if isinstance(node, dict):
        if last not in node:
            raise KeyError(pointer)
        node[last] = value
    elif isinstance(node, list):
        node[int(last)] = value
    else:
        raise KeyError(pointer)


@dataclass(frozen=True, slots=True)
class MissionDocument:
    raw: Mapping[str, Any]
    source_path: Path | None = None
    base_directory: Path = Path(".")
    digest_sha256: str = ""

    def __post_init__(self) -> None:
        raw = _deep_thaw(self.raw)
        validate_mission_dict(raw)
        digest = mission_sha256(raw)
        object.__setattr__(self, "raw", _deep_freeze(raw))
        object.__setattr__(self, "base_directory", Path(self.base_directory).resolve())
        object.__setattr__(self, "source_path", None if self.source_path is None else Path(self.source_path).resolve())
        object.__setattr__(self, "digest_sha256", digest)

    @property
    def mission_id(self) -> str:
        return str(self.raw["mission"]["id"])

    def mutable_copy(self) -> dict[str, Any]:
        return _deep_thaw(self.raw)

    def with_overrides(self, overrides: Mapping[str, Any]) -> "MissionDocument":
        raw = self.mutable_copy()
        for pointer, value in overrides.items():
            pointer_set(raw, pointer, value)
        return MissionDocument(raw, self.source_path, self.base_directory)


@dataclass(frozen=True, slots=True)
class MissionOptimizationDeclaration:
    design_variables: tuple[DesignVariable, ...]
    pointers: tuple[str, ...]
    objective: MetricObjective
    constraints: tuple[MetricConstraint, ...]
    method: str = "SLSQP"
    max_iterations: int = 100


@dataclass(frozen=True, slots=True)
class MissionDispersionDeclaration:
    name: str
    pointer: str
    distribution: str
    parameters: Mapping[str, float]


@dataclass(frozen=True, slots=True)
class MissionRunReport:
    mission_id: str
    success: bool
    message: str
    start_time: float
    end_time: float
    mission_sha256: str
    dataset_inventory: tuple[tuple[str, str, str], ...]
    plugin_inventory: tuple[tuple[str, str, str], ...]
    outputs: Mapping[str, float]
    final_vehicles: Mapping[str, Mapping[str, Any]]
    events: tuple[Mapping[str, Any], ...]

    def to_json_dict(self) -> dict[str, Any]:
        return {
            "metadata": {
                "uniflight_version": _uniflight_version,
                "mission_id": self.mission_id,
                "mission_sha256": self.mission_sha256,
                "dataset_inventory": [
                    {"dataset_id": d, "version": v, "sha256": s}
                    for d, v, s in self.dataset_inventory
                ],
                "plugin_inventory": [
                    {"plugin_id": p, "version": v, "api_version": a}
                    for p, v, a in self.plugin_inventory
                ],
            },
            "success": self.success,
            "message": self.message,
            "start_time": self.start_time,
            "end_time": self.end_time,
            "outputs": dict(self.outputs),
            "final_vehicles": {k: dict(v) for k, v in self.final_vehicles.items()},
            "events": [dict(e) for e in self.events],
        }


@dataclass(frozen=True, slots=True)
class CompiledMission:
    document: MissionDocument
    vehicles: tuple[VehicleSpec, ...]
    engine: MultiVehicleUniverseEngine
    t_span: tuple[float, float]
    bodies: Mapping[str, SphericalBody]
    environments: Mapping[str, PlanetaryEnvironment]
    data_catalog: EngineeringDataCatalog
    output_specs: tuple[Mapping[str, Any], ...]
    registry: MissionRegistry | None = None
    models: Mapping[str, Any] = field(default_factory=dict)
    plugin_inventory: tuple[tuple[str, str, str], ...] = ()
    optimization: MissionOptimizationDeclaration | None = None
    dispersions: tuple[MissionDispersionDeclaration, ...] = ()

    def run(self) -> MissionRunReport:
        result = self.engine.run(self.t_span, self.vehicles)
        outputs = _extract_outputs(result, self.output_specs, self.bodies, self.registry, self.models)
        finals: dict[str, dict[str, Any]] = {}
        for vid, snap in result.final_vehicles.items():
            unpacked = snap.schema.unpack(snap.state)
            finals[vid] = {
                "mode": snap.mode,
                "dof": snap.dof,
                "schema_hash": snap.schema.layout_hash,
                "state": _jsonable(unpacked),
            }
        events = tuple({
            "time": e.time,
            "vehicle": e.vehicle_id,
            "event": e.event_name,
            "priority": e.priority,
            "note": e.mutation_note,
            "active_after": list(e.active_vehicle_ids_after),
        } for e in result.events)
        return MissionRunReport(
            self.document.mission_id, result.success, result.message,
            result.start_time, result.end_time, self.document.digest_sha256,
            self.data_catalog.inventory(), self.plugin_inventory, MappingProxyType(outputs),
            MappingProxyType(finals), events,
        )


class MissionRegistry:
    """Version-aware capability registry used by MDL and third-party plugins.

    Core factories are owned by ``core``. Third-party registrations are
    namespaced by :class:`~uniflight.plugins.PluginRegistrar`, which prevents
    silent replacement of core or another vendor's capability. The historical
    three-argument ``register`` call remains source-compatible for Python users.
    """

    def __init__(self) -> None:
        self._registrations: dict[tuple[str, str], CapabilityRegistration] = {}

    def register(self, category: str, type_name: str, factory: Callable[..., Any], *,
                 replace: bool = False, owner: str = "core", owner_version: str = _uniflight_version,
                 description: str = "", validator: Callable[[Mapping[str, Any]], None] | None = None) -> None:
        key = (str(category), str(type_name))
        if key in self._registrations and not replace:
            raise KeyError(f"mission factory {key} already registered")
        if key in self._registrations and replace and self._registrations[key].owner != owner:
            raise KeyError(f"capability {key} is owned by {self._registrations[key].owner!r}; cross-owner replacement is forbidden")
        self._registrations[key] = CapabilityRegistration(
            key[0], key[1], factory, str(owner), str(owner_version), str(description), validator
        )

    def build(self, category: str, type_name: str, spec: Mapping[str, Any], context: Mapping[str, Any]) -> Any:
        key = (str(category), str(type_name))
        if key not in self._registrations:
            available = sorted(t for c, t in self._registrations if c == key[0])
            raise MissionCompilationError(f"unknown {category} type {type_name!r}; available={available}")
        reg = self._registrations[key]
        if reg.validator is not None:
            reg.validator(spec)
        return reg.factory(spec, context)

    def available(self, category: str) -> tuple[str, ...]:
        return tuple(sorted(t for c, t in self._registrations if c == str(category)))

    def registration(self, category: str, type_name: str) -> CapabilityRegistration:
        key=(str(category),str(type_name))
        if key not in self._registrations:
            raise KeyError(key)
        return self._registrations[key]

    def inventory(self) -> tuple[Mapping[str, str], ...]:
        return tuple(MappingProxyType({
            "category": reg.category, "type": reg.type_name, "owner": reg.owner,
            "owner_version": reg.owner_version, "description": reg.description,
        }) for _, reg in sorted(self._registrations.items()))


def load_mission(path: str | Path) -> MissionDocument:
    path = Path(path).resolve()
    return MissionDocument(_load_raw(path), path, path.parent)


def validate_mission_dict(raw: Mapping[str, Any]) -> None:
    _strict_keys(raw, {
        "format_version", "mission", "datasets", "bodies", "atmospheres", "environments",
        "plugins", "models", "solvers", "vehicles", "vehicle_templates", "events", "outputs", "optimization", "monte_carlo", "analysis", "metadata",
    }, "root")
    version = _require(raw, "format_version", "root")
    if str(version) != MISSION_FORMAT_VERSION:
        raise MissionValidationError(f"unsupported format_version {version!r}; expected {MISSION_FORMAT_VERSION!r}")

    mission = _as_mapping(_require(raw, "mission", "root"), "mission")
    _strict_keys(mission, {"id", "title", "description", "t_span", "default_solver", "seed", "tags"}, "mission")
    mid = _require(mission, "id", "mission")
    if not isinstance(mid, str) or not mid.strip():
        raise MissionValidationError("mission.id must be a non-empty string")
    tspan = _as_sequence(_require(mission, "t_span", "mission"), "mission.t_span")
    if len(tspan) != 2:
        raise MissionValidationError("mission.t_span must contain [t0, tf]")
    t0, tf = (_finite_float(tspan[0], "mission.t_span[0]"), _finite_float(tspan[1], "mission.t_span[1]"))
    if not tf > t0:
        raise MissionValidationError("mission.t_span must increase")

    plugins = raw.get("plugins", [])
    if not isinstance(plugins, list):
        raise MissionValidationError("plugins must be a list")
    plugin_ids: set[str] = set()
    for i, entry in enumerate(plugins):
        e = _as_mapping(entry, f"plugins[{i}]")
        _strict_keys(e, {"id", "version", "required"}, f"plugins[{i}]")
        pid = str(_require(e, "id", f"plugins[{i}]"))
        ver = str(_require(e, "version", f"plugins[{i}]"))
        if not pid or not ver or pid in plugin_ids:
            raise MissionValidationError("plugin id/version must be non-empty and plugin ids unique")
        plugin_ids.add(pid)

    models = raw.get("models", {})
    if not isinstance(models, Mapping):
        raise MissionValidationError("models must be a mapping")
    for name, spec in models.items():
        m = _as_mapping(spec, f"models.{name}")
        _strict_keys(m, {"category", "type", "config"}, f"models.{name}")
        _require(m, "category", f"models.{name}")
        _require(m, "type", f"models.{name}")
        if "config" in m:
            _as_mapping(m["config"], f"models.{name}.config")

    datasets = raw.get("datasets", [])
    if not isinstance(datasets, list):
        raise MissionValidationError("datasets must be a list")
    dataset_keys: set[tuple[str, str]] = set()
    for i, entry in enumerate(datasets):
        e = _as_mapping(entry, f"datasets[{i}]")
        _strict_keys(e, {"id", "version", "path", "format", "verify_checksum"}, f"datasets[{i}]")
        did = str(_require(e, "id", f"datasets[{i}]"))
        ver = str(_require(e, "version", f"datasets[{i}]"))
        _require(e, "path", f"datasets[{i}]")
        if not did or not ver:
            raise MissionValidationError("dataset id/version must be non-empty")
        if (did, ver) in dataset_keys:
            raise MissionValidationError(f"duplicate dataset declaration {(did, ver)}")
        dataset_keys.add((did, ver))

    bodies = _as_mapping(_require(raw, "bodies", "root"), "bodies")
    if not bodies:
        raise MissionValidationError("at least one body is required")
    for name, spec in bodies.items():
        s = _as_mapping(spec, f"bodies.{name}")
        typ = str(s.get("type", "spherical"))
        if ":" in typ:
            _strict_keys(s, {"type", "config", "name"}, f"bodies.{name}")
            if "config" in s: _as_mapping(s["config"], f"bodies.{name}.config")
        else:
            _strict_keys(s, {"type", "mu", "mass", "radius", "rotation_vector_i", "name"}, f"bodies.{name}")
            if "mu" not in s and "mass" not in s:
                raise MissionValidationError(f"bodies.{name} requires mu or mass")
            _require(s, "radius", f"bodies.{name}")

    solvers = raw.get("solvers", {})
    if not isinstance(solvers, Mapping):
        raise MissionValidationError("solvers must be a mapping")
    if mission.get("default_solver") is not None and mission["default_solver"] not in solvers:
        raise MissionValidationError("mission.default_solver references an undefined solver")

    atmospheres = raw.get("atmospheres", {})
    if not isinstance(atmospheres, Mapping):
        raise MissionValidationError("atmospheres must be a mapping")
    environments = raw.get("environments", {})
    if not isinstance(environments, Mapping):
        raise MissionValidationError("environments must be a mapping")

    vehicles = _as_mapping(_require(raw, "vehicles", "root"), "vehicles")
    if not vehicles:
        raise MissionValidationError("at least one vehicle is required")
    for vid, vspec in vehicles.items():
        v = _as_mapping(vspec, f"vehicles.{vid}")
        _strict_keys(v, {"body", "environment", "solver", "initial", "phases", "metadata"}, f"vehicles.{vid}")
        body = str(_require(v, "body", f"vehicles.{vid}"))
        if body not in bodies:
            raise MissionValidationError(f"vehicles.{vid}.body references unknown body {body!r}")
        env = v.get("environment")
        if env is not None and env not in environments:
            raise MissionValidationError(f"vehicles.{vid}.environment references unknown environment {env!r}")
        init = _as_mapping(_require(v, "initial", f"vehicles.{vid}"), f"vehicles.{vid}.initial")
        _strict_keys(init, {"dof", "state", "mode"}, f"vehicles.{vid}.initial")
        dof = int(_require(init, "dof", f"vehicles.{vid}.initial"))
        if dof not in (3, 6):
            raise MissionValidationError(f"vehicles.{vid}.initial.dof must be 3 or 6")
        _as_mapping(_require(init, "state", f"vehicles.{vid}.initial"), f"vehicles.{vid}.initial.state")
        phases = _as_sequence(_require(v, "phases", f"vehicles.{vid}"), f"vehicles.{vid}.phases")
        if not phases:
            raise MissionValidationError(f"vehicles.{vid}.phases cannot be empty")
        names: set[str] = set()
        for j, phase in enumerate(phases):
            p = _as_mapping(phase, f"vehicles.{vid}.phases[{j}]")
            _strict_keys(p, {"name", "dof", "until", "dynamics", "transition", "solver"}, f"vehicles.{vid}.phases[{j}]")
            name = str(_require(p, "name", f"vehicles.{vid}.phases[{j}]"))
            if not name or name in names:
                raise MissionValidationError(f"vehicle {vid!r} phase names must be unique and non-empty")
            names.add(name)
            pdof = int(p.get("dof", dof))
            if pdof not in (3, 6):
                raise MissionValidationError(f"vehicles.{vid}.phases[{j}].dof must be 3 or 6")
            _as_mapping(p.get("dynamics", {}), f"vehicles.{vid}.phases[{j}].dynamics")
            if j < len(phases)-1 and "until" not in p:
                raise MissionValidationError(f"non-final phase {vid}.{name} requires an until guard")
            if "until" in p:
                guard = _as_mapping(p["until"], f"vehicles.{vid}.phases[{j}].until")
                gtype = str(_require(guard, "type", f"vehicles.{vid}.phases[{j}].until"))
                if ":" in gtype:
                    _strict_keys(guard, {"type", "config", "direction", "priority"}, f"vehicles.{vid}.phases[{j}].until")
                    if "config" in guard: _as_mapping(guard["config"], f"vehicles.{vid}.phases[{j}].until.config")
                else:
                    _strict_keys(guard, {"type", "value", "direction", "field", "index", "priority"}, f"vehicles.{vid}.phases[{j}].until")
                    if gtype not in ("time", "altitude", "state"):
                        raise MissionValidationError(f"unsupported phase guard type {gtype!r}")
                    _require(guard, "value", f"vehicles.{vid}.phases[{j}].until")
                    if gtype == "state": _require(guard, "field", f"vehicles.{vid}.phases[{j}].until")

    templates = raw.get("vehicle_templates", {})
    if not isinstance(templates, Mapping):
        raise MissionValidationError("vehicle_templates must be a mapping")
    for tid, tspec in templates.items():
        t = _as_mapping(tspec, f"vehicle_templates.{tid}")
        _strict_keys(t, {"body", "environment", "solver", "phases", "metadata"}, f"vehicle_templates.{tid}")
        body = str(_require(t, "body", f"vehicle_templates.{tid}"))
        if body not in bodies:
            raise MissionValidationError(f"vehicle_templates.{tid}.body references unknown body {body!r}")
        if t.get("environment") is not None and t["environment"] not in environments:
            raise MissionValidationError(f"vehicle_templates.{tid}.environment references unknown environment")
        phases = _as_sequence(_require(t, "phases", f"vehicle_templates.{tid}"), f"vehicle_templates.{tid}.phases")
        if not phases:
            raise MissionValidationError(f"vehicle_templates.{tid}.phases cannot be empty")
        for j, phase in enumerate(phases):
            ph = _as_mapping(phase, f"vehicle_templates.{tid}.phases[{j}]")
            _strict_keys(ph, {"name", "dof", "until", "dynamics", "transition", "solver"}, f"vehicle_templates.{tid}.phases[{j}]")
            _require(ph, "name", f"vehicle_templates.{tid}.phases[{j}]")
            if int(ph.get("dof", 6)) not in (3,6):
                raise MissionValidationError("template phase dof must be 3 or 6")
            if j < len(phases)-1 and "until" not in ph:
                raise MissionValidationError(f"non-final template phase {tid}.{ph['name']} requires an until guard")

    event_names=set()
    for i, event in enumerate(raw.get("events", [])):
        e=_as_mapping(event, f"events[{i}]")
        _strict_keys(e, {"name","vehicle","guard","action","priority"}, f"events[{i}]")
        name=str(_require(e,"name",f"events[{i}]"))
        if not name or name in event_names:
            raise MissionValidationError("global event names must be unique and non-empty")
        event_names.add(name)
        if str(_require(e,"vehicle",f"events[{i}]")) not in vehicles:
            raise MissionValidationError(f"events[{i}] source vehicle must be initially declared")
        guard=_as_mapping(_require(e,"guard",f"events[{i}]"),f"events[{i}].guard")
        gtype=str(_require(guard,"type",f"events[{i}].guard"))
        if ":" in gtype:
            _strict_keys(guard,{"type","config","direction"},f"events[{i}].guard")
            if "config" in guard: _as_mapping(guard["config"],f"events[{i}].guard.config")
        else:
            _strict_keys(guard,{"type","value","direction","field","index"},f"events[{i}].guard")
            if gtype not in ("time","altitude","state"):
                raise MissionValidationError(f"unsupported event guard type {gtype!r}")
            _require(guard,"value",f"events[{i}].guard")
            if gtype=="state": _require(guard,"field",f"events[{i}].guard")
        action=_as_mapping(_require(e,"action",f"events[{i}]"),f"events[{i}].action")
        atype=str(_require(action,"type",f"events[{i}].action"))
        if atype=="remove_vehicle":
            _strict_keys(action,{"type","note"},f"events[{i}].action")
        elif atype=="rigid_separation":
            _strict_keys(action,{"type","parent_inertia_b","relative_separation_velocity_i","conserve_angular_momentum","retained","detached","note"},f"events[{i}].action")
            _require(action,"parent_inertia_b",f"events[{i}].action")
            for child_name in ("retained","detached"):
                c=_as_mapping(_require(action,child_name,f"events[{i}].action"),f"events[{i}].action.{child_name}")
                _strict_keys(c,{"vehicle_id","template","mass","offset_b","inertia_b"},f"events[{i}].action.{child_name}")
                template=str(_require(c,"template",f"events[{i}].action.{child_name}"))
                if template not in templates:
                    raise MissionValidationError(f"event child references unknown vehicle template {template!r}")
                for key in ("vehicle_id","mass","offset_b","inertia_b"):
                    _require(c,key,f"events[{i}].action.{child_name}")
        elif ":" in atype:
            _strict_keys(action,{"type","config","note"},f"events[{i}].action")
            if "config" in action: _as_mapping(action["config"],f"events[{i}].action.config")
        else:
            raise MissionValidationError(f"unsupported global event action type {atype!r}")

    outputs = raw.get("outputs", [])
    if not isinstance(outputs, list):
        raise MissionValidationError("outputs must be a list")
    output_names: set[str] = set()
    for i, spec in enumerate(outputs):
        o = _as_mapping(spec, f"outputs[{i}]")
        _strict_keys(o, {"name", "type", "vehicle", "field", "index", "body", "config"}, f"outputs[{i}]")
        name = str(_require(o, "name", f"outputs[{i}]"))
        if not name or name in output_names:
            raise MissionValidationError("output names must be unique and non-empty")
        output_names.add(name)
        otype = str(_require(o, "type", f"outputs[{i}]"))
        if otype not in ("state", "altitude", "speed", "time", "vehicle_count") and ":" not in otype:
            raise MissionValidationError(f"unsupported output type {otype!r}")
        if otype not in ("time", "vehicle_count") and ":" not in otype:
            vid = str(_require(o, "vehicle", f"outputs[{i}]"))
            if vid not in vehicles:
                # spawned vehicles may not exist initially; permit explicit IDs.
                pass
        if otype == "state":
            _require(o, "field", f"outputs[{i}]")

    optimization = raw.get("optimization")
    if optimization is not None:
        opt = _as_mapping(optimization, "optimization")
        _strict_keys(opt, {"method", "max_iterations", "design_variables", "objective", "constraints"}, "optimization")
        dvs = _as_sequence(_require(opt, "design_variables", "optimization"), "optimization.design_variables")
        for i, dv in enumerate(dvs):
            d = _as_mapping(dv, f"optimization.design_variables[{i}]")
            _strict_keys(d, {"name", "pointer", "lower", "upper", "initial", "scale"}, f"optimization.design_variables[{i}]")
            ptr = str(_require(d, "pointer", f"optimization.design_variables[{i}]"))
            pointer_get(raw, ptr)  # validates reference
        obj = _as_mapping(_require(opt, "objective", "optimization"), "optimization.objective")
        _strict_keys(obj, {"metric", "sense", "scale"}, "optimization.objective")
        if str(_require(obj, "metric", "optimization.objective")) not in output_names:
            raise MissionValidationError("optimization objective references undefined output metric")
        for i, c in enumerate(opt.get("constraints", [])):
            cm = _as_mapping(c, f"optimization.constraints[{i}]")
            _strict_keys(cm, {"metric", "lower", "upper", "scale"}, f"optimization.constraints[{i}]")
            if str(_require(cm, "metric", f"optimization.constraints[{i}]")) not in output_names:
                raise MissionValidationError("optimization constraint references undefined output metric")

    mc = raw.get("monte_carlo")
    if mc is not None:
        m = _as_mapping(mc, "monte_carlo")
        _strict_keys(m, {"cases", "seed", "workers", "dispersions"}, "monte_carlo")
        if int(m.get("cases", 1)) <= 0:
            raise MissionValidationError("monte_carlo.cases must be positive")
        for i, ds in enumerate(m.get("dispersions", [])):
            d = _as_mapping(ds, f"monte_carlo.dispersions[{i}]")
            _strict_keys(d, {"name", "pointer", "distribution", "mean", "std", "low", "high"}, f"monte_carlo.dispersions[{i}]")
            ptr = str(_require(d, "pointer", f"monte_carlo.dispersions[{i}]"))
            pointer_get(raw, ptr)
            dist = str(_require(d, "distribution", f"monte_carlo.dispersions[{i}]"))
            if dist not in ("normal", "uniform"):
                raise MissionValidationError("Monte Carlo distribution must be normal or uniform")

    # Milestone N integrated analysis declarations. These are orchestration
    # metadata only: the underlying mission dynamics remain unchanged.
    analysis = raw.get("analysis")
    if analysis is not None:
        a = _as_mapping(analysis, "analysis")
        _strict_keys(a, {"execution", "sweeps", "sobol", "optimization_batches"}, "analysis")
        execution = _as_mapping(a.get("execution", {}), "analysis.execution")
        _strict_keys(execution, {"backend", "workers", "chunksize", "store"}, "analysis.execution")
        if str(execution.get("backend", "process")) not in ("serial", "process"):
            raise MissionValidationError("analysis.execution.backend must be serial or process")
        if int(execution.get("workers", 0)) < 0 or int(execution.get("chunksize", 1)) <= 0:
            raise MissionValidationError("analysis execution workers/chunksize are invalid")

        seen=set()
        for i, entry in enumerate(a.get("sweeps", [])):
            e=_as_mapping(entry,f"analysis.sweeps[{i}]")
            _strict_keys(e,{"id","mode","variables"},f"analysis.sweeps[{i}]")
            aid=str(_require(e,"id",f"analysis.sweeps[{i}]"))
            if not aid or aid in seen: raise MissionValidationError("analysis IDs must be unique and non-empty")
            seen.add(aid)
            if str(e.get("mode","cartesian")) not in ("cartesian","zip"):
                raise MissionValidationError("sweep mode must be cartesian or zip")
            variables=_as_sequence(_require(e,"variables",f"analysis.sweeps[{i}]"),f"analysis.sweeps[{i}].variables")
            for j,var in enumerate(variables):
                v=_as_mapping(var,f"analysis.sweeps[{i}].variables[{j}]")
                _strict_keys(v,{"name","pointer","values"},f"analysis.sweeps[{i}].variables[{j}]")
                pointer_get(raw,str(_require(v,"pointer",f"analysis.sweeps[{i}].variables[{j}]")))
                vals=_as_sequence(_require(v,"values",f"analysis.sweeps[{i}].variables[{j}]"),f"analysis.sweeps[{i}].variables[{j}].values")
                if not vals: raise MissionValidationError("sweep values cannot be empty")
                for k,value in enumerate(vals): _finite_float(value,f"analysis.sweeps[{i}].variables[{j}].values[{k}]")

        for i, entry in enumerate(a.get("sobol", [])):
            e=_as_mapping(entry,f"analysis.sobol[{i}]")
            _strict_keys(e,{"id","metric","base_samples","seed","variables"},f"analysis.sobol[{i}]")
            aid=str(_require(e,"id",f"analysis.sobol[{i}]"))
            if not aid or aid in seen: raise MissionValidationError("analysis IDs must be unique and non-empty")
            seen.add(aid)
            if str(_require(e,"metric",f"analysis.sobol[{i}]")) not in output_names:
                raise MissionValidationError("Sobol metric references undefined output")
            if int(e.get("base_samples",128)) < 2: raise MissionValidationError("Sobol base_samples must be >= 2")
            variables=_as_sequence(_require(e,"variables",f"analysis.sobol[{i}]"),f"analysis.sobol[{i}].variables")
            for j,var in enumerate(variables):
                v=_as_mapping(var,f"analysis.sobol[{i}].variables[{j}]")
                _strict_keys(v,{"name","pointer","lower","upper"},f"analysis.sobol[{i}].variables[{j}]")
                pointer_get(raw,str(_require(v,"pointer",f"analysis.sobol[{i}].variables[{j}]")))
                lo=_finite_float(_require(v,"lower",f"analysis.sobol[{i}].variables[{j}]"),f"analysis.sobol[{i}].variables[{j}].lower")
                hi=_finite_float(_require(v,"upper",f"analysis.sobol[{i}].variables[{j}]"),f"analysis.sobol[{i}].variables[{j}].upper")
                if hi <= lo: raise MissionValidationError("Sobol upper bound must exceed lower bound")

        opt_names=set() if optimization is None else {str(d["name"]) for d in optimization.get("design_variables",[])}
        for i, entry in enumerate(a.get("optimization_batches", [])):
            e=_as_mapping(entry,f"analysis.optimization_batches[{i}]")
            _strict_keys(e,{"id","starts"},f"analysis.optimization_batches[{i}]")
            aid=str(_require(e,"id",f"analysis.optimization_batches[{i}]"))
            if not aid or aid in seen: raise MissionValidationError("analysis IDs must be unique and non-empty")
            seen.add(aid)
            if optimization is None: raise MissionValidationError("optimization batch requires an optimization declaration")
            starts=_as_sequence(_require(e,"starts",f"analysis.optimization_batches[{i}]"),f"analysis.optimization_batches[{i}].starts")
            for j,start in enumerate(starts):
                st=_as_mapping(start,f"analysis.optimization_batches[{i}].starts[{j}]")
                _strict_keys(st,{"name","values"},f"analysis.optimization_batches[{i}].starts[{j}]")
                values=_as_mapping(_require(st,"values",f"analysis.optimization_batches[{i}].starts[{j}]"),f"analysis.optimization_batches[{i}].starts[{j}].values")
                unknown=set(values)-opt_names
                if unknown: raise MissionValidationError(f"optimization batch references unknown design variables {sorted(unknown)}")
                for name,value in values.items(): _finite_float(value,f"analysis.optimization_batches[{i}].starts[{j}].values.{name}")

    # Every namespaced capability used by a declarative mission must be backed
    # by an explicit exact-version plugin requirement. Programmatic Python
    # registries remain free to register arbitrary factories without this rule.
    referenced_plugins: set[str] = set()
    def note_type(value: Any) -> None:
        if isinstance(value, str) and ":" in value:
            referenced_plugins.add(value.split(":",1)[0])
    for e in datasets: note_type(str(e.get("format","")))
    for spec in bodies.values(): note_type(str(spec.get("type","")))
    for spec in atmospheres.values(): note_type(str(spec.get("type","")))
    for spec in environments.values(): note_type(str(spec.get("type","")))
    for spec in solvers.values(): note_type(str(spec.get("type","")))
    for spec in models.values(): note_type(str(spec.get("type","")))
    for vspec in list(vehicles.values()) + list(templates.values()):
        for phase in vspec.get("phases",[]):
            dyn=phase.get("dynamics",{}); note_type(str(dyn.get("type","")))
            if "until" in phase: note_type(str(phase["until"].get("type","")))
    for e in raw.get("events",[]):
        note_type(str(e.get("guard",{}).get("type",""))); note_type(str(e.get("action",{}).get("type","")))
    for o in outputs: note_type(str(o.get("type","")))
    if optimization is not None: note_type(str(optimization.get("method","")))
    missing = referenced_plugins - plugin_ids
    if missing:
        raise MissionValidationError(f"namespaced capabilities require explicit plugins entries; missing {sorted(missing)}")


def mission_json_schema() -> Mapping[str, Any]:
    """Return a compact JSON-Schema-like contract for external editors.

    Runtime validation remains authoritative and stricter for cross references.
    This schema is intentionally dependency-free and can be emitted by the CLI.
    """
    return {
        "$schema": "https://json-schema.org/draft/2020-12/schema",
        "$id": "https://uniflight.local/schema/mission-1.0.json",
        "title": "UniFlight Mission Definition Language 1.0",
        "type": "object",
        "required": ["format_version", "mission", "bodies", "vehicles"],
        "properties": {
            "format_version": {"const": MISSION_FORMAT_VERSION},
            "mission": {
                "type": "object", "required": ["id", "t_span"],
                "properties": {
                    "id": {"type": "string", "minLength": 1},
                    "t_span": {"type": "array", "minItems": 2, "maxItems": 2, "items": {"type": "number"}},
                    "default_solver": {"type": "string"},
                },
            },
            "plugins": {"type": "array"},
            "models": {"type": "object"},
            "datasets": {"type": "array"},
            "bodies": {"type": "object", "minProperties": 1},
            "atmospheres": {"type": "object"},
            "environments": {"type": "object"},
            "solvers": {"type": "object"},
            "vehicles": {"type": "object", "minProperties": 1},
            "vehicle_templates": {"type": "object"},
            "events": {"type": "array"},
            "outputs": {"type": "array"},
            "optimization": {"type": "object"},
            "monte_carlo": {"type": "object"},
            "analysis": {"type": "object"},
            "metadata": {"type": "object"},
        },
        "additionalProperties": False,
    }


# --- Built-in runtime models -------------------------------------------------

@dataclass(frozen=True, slots=True)
class _ConstantBodyRocket6DOF:
    thrust: float
    mass_flow: float
    direction_b: np.ndarray
    mount_b: np.ndarray = field(default_factory=lambda: np.zeros(3))
    cg_b: np.ndarray = field(default_factory=lambda: np.zeros(3))

    def __post_init__(self) -> None:
        d = np.asarray(self.direction_b, dtype=float)
        n = np.linalg.norm(d)
        if self.thrust < 0 or self.mass_flow < 0 or not math.isfinite(self.thrust + self.mass_flow) or n <= 0:
            raise ValueError("invalid 6-DOF rocket parameters")
        object.__setattr__(self, "direction_b", d/n)
        object.__setattr__(self, "mount_b", np.asarray(self.mount_b, dtype=float).copy())
        object.__setattr__(self, "cg_b", np.asarray(self.cg_b, dtype=float).copy())

    def wrench(self, state) -> Wrench:
        force_b = self.thrust * self.direction_b
        force_i = body_to_inertial_matrix(state.get("attitude")) @ force_b
        return Wrench(force_i, np.cross(self.mount_b-self.cg_b, force_b), "mission-rocket")

    def derivatives(self, state) -> dict[str, float]:
        return {"mass": -self.mass_flow}


def _default_registry() -> MissionRegistry:
    reg = MissionRegistry()

    def body_spherical(spec: Mapping[str, Any], ctx: Mapping[str, Any]) -> SphericalBody:
        if "mu" in spec:
            mu = _finite_float(spec["mu"], "body.mu")
        else:
            # Gravitational constant in SI; mass remains a first-class planet variable.
            mu = 6.67430e-11 * _finite_float(spec["mass"], "body.mass")
        return SphericalBody(mu, _finite_float(spec["radius"], "body.radius"),
                             np.asarray(spec.get("rotation_vector_i", [0,0,0]), dtype=float),
                             str(spec.get("name", ctx.get("name", "body"))))

    def atm_vacuum(spec: Mapping[str, Any], ctx: Mapping[str, Any]):
        return VacuumAtmosphere()

    def atm_isothermal(spec: Mapping[str, Any], ctx: Mapping[str, Any]):
        body: SphericalBody = ctx["body"]
        _strict_keys(spec, {"type", "surface_pressure", "temperature", "ceiling", "species"}, "atmosphere")
        entries = _as_sequence(_require(spec, "species", "atmosphere"), "atmosphere.species")
        species: list[GasSpecies] = []
        fractions: list[float] = []
        for i, item in enumerate(entries):
            s = _as_mapping(item, f"atmosphere.species[{i}]")
            _strict_keys(s, {"name", "mole_fraction", "molar_mass", "cp_molar", "viscosity_ref", "viscosity_ref_temperature", "sutherland_constant", "collision_diameter"}, f"atmosphere.species[{i}]")
            species.append(GasSpecies(
                str(_require(s,"name",f"atmosphere.species[{i}]")),
                _finite_float(_require(s,"molar_mass",f"atmosphere.species[{i}]"),"molar_mass"),
                _finite_float(_require(s,"cp_molar",f"atmosphere.species[{i}]"),"cp_molar"),
                _finite_float(_require(s,"viscosity_ref",f"atmosphere.species[{i}]"),"viscosity_ref"),
                _finite_float(_require(s,"viscosity_ref_temperature",f"atmosphere.species[{i}]"),"viscosity_ref_temperature"),
                _finite_float(s.get("sutherland_constant",0.0),"sutherland_constant"),
                _finite_float(_require(s,"collision_diameter",f"atmosphere.species[{i}]"),"collision_diameter"),
            ))
            fractions.append(_finite_float(_require(s,"mole_fraction",f"atmosphere.species[{i}]"),"mole_fraction"))
        mixture = GasMixture(tuple(species), tuple(fractions))
        return IsothermalHydrostaticAtmosphere(
            _finite_float(_require(spec,"surface_pressure","atmosphere"),"surface_pressure"),
            _finite_float(_require(spec,"temperature","atmosphere"),"temperature"),
            mixture, body.mu, body.radius,
            None if spec.get("ceiling") is None else _finite_float(spec["ceiling"],"ceiling"),
        )

    reg.register("body", "spherical", body_spherical)
    reg.register("atmosphere", "vacuum", atm_vacuum)
    reg.register("atmosphere", "isothermal_hydrostatic", atm_isothermal)
    return reg


class MissionCompiler:
    """Compile MDL documents into the trusted UniFlight runtime.

    Milestone M loads only plugins explicitly required by the mission. Plugin
    factories receive immutable-ish context mappings and never require edits to
    this compiler for new namespaced capabilities.
    """

    def __init__(self, registry: MissionRegistry | None = None, plugin_manager: PluginManager | None = None) -> None:
        self.registry = registry or _default_registry()
        self.plugin_manager = plugin_manager or PluginManager()

    @staticmethod
    def _plugin_spec(type_name: str, spec: Mapping[str, Any]) -> Mapping[str, Any]:
        if ":" in str(type_name):
            return _as_mapping(spec.get("config", {}), f"plugin:{type_name}.config")
        return spec

    def _load_required_plugins(self, raw: Mapping[str, Any]) -> tuple[tuple[str, str, str], ...]:
        reqs = tuple(PluginRequirement(str(e["id"]), str(e["version"]), bool(e.get("required", True)))
                     for e in raw.get("plugins", []))
        try:
            loaded = self.plugin_manager.load_requirements(reqs, self.registry)
        except PluginError as exc:
            raise MissionCompilationError(str(exc)) from exc
        return tuple(p.inventory_tuple() for p in loaded)

    def compile(self, document: MissionDocument) -> CompiledMission:
        raw = document.raw
        mission = raw["mission"]
        t_span = tuple(float(x) for x in mission["t_span"])
        plugin_inventory = self._load_required_plugins(raw)

        catalog = EngineeringDataCatalog()
        for entry in raw.get("datasets", []):
            fmt = str(entry.get("format", "npz"))
            p = (document.base_directory / str(entry["path"])).resolve()
            if fmt.lower() == "npz":
                table = catalog.load_npz(p, verify_checksum=bool(entry.get("verify_checksum", True)))
            elif ":" in fmt:
                table = self.registry.build("dataset_loader", fmt, entry, {
                    "path": p, "catalog": catalog, "document": document,
                })
                # A loader may register directly and return None, or return a table.
                if table is not None and getattr(table, "provenance", None) is not None:
                    catalog.register(table)
            else:
                raise MissionCompilationError("dataset format must be 'npz' or a namespaced plugin dataset_loader")
            if table is not None and getattr(table, "provenance", None) is not None:
                prov = table.provenance
                if prov.dataset_id != str(entry["id"]) or prov.version != str(entry["version"]):
                    raise MissionCompilationError(
                        f"dataset declaration {(entry['id'],entry['version'])} does not match file provenance {(prov.dataset_id,prov.version)}"
                    )
            try:
                loaded = catalog.resolve(str(entry["id"]), str(entry["version"]))
            except Exception as exc:
                raise MissionCompilationError(f"dataset loader did not provide {(entry['id'], entry['version'])}") from exc

        bodies: dict[str, Any] = {}
        for name, spec in raw["bodies"].items():
            typ = str(spec.get("type", "spherical"))
            bodies[name] = self.registry.build("body", typ, self._plugin_spec(typ, spec), {
                "name": name, "document": document, "catalog": catalog,
            })

        atmospheres: dict[str, Any] = {"vacuum": VacuumAtmosphere()}
        for name, spec in raw.get("atmospheres", {}).items():
            typ = str(spec.get("type", "vacuum"))
            body_ref = spec.get("body") if isinstance(spec, Mapping) else None
            if body_ref is None:
                if len(bodies) != 1 and typ != "vacuum" and ":" not in typ:
                    raise MissionCompilationError(f"atmosphere {name!r} must specify body when mission has multiple bodies")
                body = next(iter(bodies.values()))
            else:
                body = bodies[str(body_ref)]
            atmospheres[name] = self.registry.build("atmosphere", typ, self._plugin_spec(typ, spec), {
                "body": body, "name": name, "document": document, "catalog": catalog,
            })

        environments: dict[str, Any] = {}
        for name, spec in raw.get("environments", {}).items():
            s = _as_mapping(spec, f"environments.{name}")
            typ = str(s.get("type", "planetary"))
            if ":" in typ:
                environments[name] = self.registry.build("environment", typ, self._plugin_spec(typ, s), {
                    "name": name, "bodies": MappingProxyType(bodies), "atmospheres": MappingProxyType(atmospheres),
                    "document": document, "catalog": catalog,
                })
            else:
                _strict_keys(s, {"body", "atmosphere", "type"}, f"environments.{name}")
                body = bodies[str(_require(s,"body",f"environments.{name}"))]
                atm_name = str(s.get("atmosphere","vacuum"))
                if atm_name not in atmospheres:
                    raise MissionCompilationError(f"environment {name!r} references unknown atmosphere {atm_name!r}")
                environments[name] = PlanetaryEnvironment(body, atmospheres[atm_name])

        # Generic mission-level plugin models. Their category determines the
        # stable capability namespace (aero, propulsion, gnc, terrain, ...).
        models: dict[str, Any] = {}
        for name, spec in raw.get("models", {}).items():
            category = str(spec["category"])
            typ = str(spec["type"])
            models[name] = self.registry.build(category, typ, _as_mapping(spec.get("config", {}), f"models.{name}.config"), {
                "name": name, "document": document, "catalog": catalog,
                "bodies": MappingProxyType(bodies), "atmospheres": MappingProxyType(atmospheres),
                "environments": MappingProxyType(environments), "models": MappingProxyType(dict(models)),
            })

        solvers = self._compile_solvers(raw.get("solvers", {}), document, models)
        default_solver = mission.get("default_solver")
        default_integrator = solvers.get(default_solver) if default_solver else ScipyIVPIntegrator()
        engine = MultiVehicleUniverseEngine(default_integrator=default_integrator)

        vehicles: list[VehicleSpec] = []
        for vid, vspec in raw["vehicles"].items():
            vehicles.append(self._compile_initial_vehicle(
                document, vid, vspec, bodies, environments, solvers, default_integrator, models
            ))

        opt = self._parse_optimization(document)
        dispersions = self._parse_dispersions(document)
        return CompiledMission(
            document, tuple(vehicles), engine, t_span,
            MappingProxyType(bodies), MappingProxyType(environments), catalog,
            tuple(MappingProxyType(dict(o)) for o in raw.get("outputs", [])),
            self.registry, MappingProxyType(models), plugin_inventory, opt, dispersions,
        )

    def _compile_solvers(self, specs: Mapping[str, Any], document: MissionDocument,
                         models: Mapping[str, Any]) -> dict[str, Any]:
        out: dict[str, Any] = {}
        for name, spec in specs.items():
            s = _as_mapping(spec, f"solvers.{name}")
            typ = str(s.get("type", "scipy"))
            low = typ.lower()
            if low == "scipy":
                _strict_keys(s, {"type","method","rtol","atol","max_step","dense_output"}, f"solvers.{name}")
                cfg = SolverConfig(
                    method=str(s.get("method","DOP853")), rtol=float(s.get("rtol",1e-10)),
                    atol=float(s.get("atol",1e-12)), max_step=float(s.get("max_step",np.inf)),
                    dense_output=bool(s.get("dense_output",True)),
                )
                out[name] = ScipyIVPIntegrator(cfg)
            elif low == "rk4":
                _strict_keys(s, {"type","step","event_time_tolerance","save_every_step"}, f"solvers.{name}")
                out[name] = FixedStepRK4Integrator(FixedStepRK4Config(
                    step=float(s.get("step",0.1)), event_time_tolerance=float(s.get("event_time_tolerance",1e-8)),
                    save_every_step=bool(s.get("save_every_step",False)),
                ))
            elif ":" in typ:
                out[name] = self.registry.build("solver", typ, self._plugin_spec(typ, s), {
                    "name": name, "document": document, "models": models,
                })
            else:
                raise MissionCompilationError(f"unknown solver type {typ!r}")
        return out

    def _schema(self, dof: int) -> StateSchema:
        return core_3dof_schema() if int(dof)==3 else core_6dof_schema()

    def _compile_initial_vehicle(self, document: MissionDocument, vid: str, vspec: Mapping[str, Any],
                                 bodies: Mapping[str,Any], environments: Mapping[str,Any],
                                 solvers: Mapping[str,Any], default_integrator: Any,
                                 models: Mapping[str, Any]) -> VehicleSpec:
        initial = vspec["initial"]
        dof = int(initial["dof"])
        schema = self._schema(dof)
        state = schema.pack(initial["state"])
        return self._phase_spec(document, vid, vspec, 0, schema, state, bodies, environments, solvers, default_integrator, models)

    def _phase_spec(self, document: MissionDocument, vid: str, vspec: Mapping[str, Any], phase_index: int,
                    source_schema: StateSchema, source_state: np.ndarray,
                    bodies: Mapping[str,Any], environments: Mapping[str,Any],
                    solvers: Mapping[str,Any], default_integrator: Any,
                    models: Mapping[str, Any]) -> VehicleSpec:
        phases = list(vspec["phases"])
        phase = phases[phase_index]
        target_dof = int(phase.get("dof", 3 if source_schema.total_size==7 else 6))
        schema = self._schema(target_dof)
        state = self._transition_state(source_schema, schema, source_state, phase.get("transition", {}))
        body = bodies[str(vspec["body"])]
        env = environments.get(str(vspec.get("environment","")))
        if env is None:
            env = PlanetaryEnvironment(body, VacuumAtmosphere())
        rhs = self._build_rhs(schema, phase.get("dynamics", {}), body, env, models, document, vid, phase)
        solver_name = phase.get("solver", vspec.get("solver", document.raw["mission"].get("default_solver")))
        integrator = solvers.get(solver_name, default_integrator) if solver_name else default_integrator
        events: list[VehicleEvent] = []
        if phase_index < len(phases)-1:
            guard_spec = phase["until"]
            guard = self._build_guard(guard_spec, schema, body, models, document)
            next_index = phase_index+1

            def handler(ctx, *, _vid=vid, _vspec=vspec, _next=next_index, _schema=schema,
                        _bodies=bodies, _envs=environments, _solvers=solvers,
                        _default=default_integrator, _models=models):
                new_spec = self._phase_spec(
                    document, _vid, _vspec, _next, _schema, ctx.source.state,
                    _bodies, _envs, _solvers, _default, _models,
                )
                return UniverseMutation(upsert=(new_spec,), note=f"phase -> {_vspec['phases'][_next]['name']}")

            events.append(VehicleEvent(
                f"phase:{phase['name']}", guard, direction=float(guard_spec.get("direction", 0.0)),
                priority=int(guard_spec.get("priority", 0)), handler=handler,
            ))
        events.extend(self._global_events_for_vehicle(
            document, vid, schema, body, vspec, phase_index, bodies, environments, solvers, default_integrator, models
        ))
        return VehicleSpec(
            vid, schema, state, rhs, tuple(events), integrator, mode=str(phase["name"]), dof=target_dof,
            model_context={"body":str(vspec["body"]),"environment":vspec.get("environment"),"phase_index":phase_index},
        )

    def _global_events_for_vehicle(self, document: MissionDocument, vid: str, schema: StateSchema, body: Any,
                                   vspec: Mapping[str,Any], phase_index: int,
                                   bodies: Mapping[str,Any], environments: Mapping[str,Any], solvers: Mapping[str,Any],
                                   default_integrator: Any, models: Mapping[str, Any]) -> list[VehicleEvent]:
        out=[]
        templates=document.raw.get("vehicle_templates", {})
        for event_spec in document.raw.get("events", []):
            if str(event_spec["vehicle"]) != vid:
                continue
            guard_spec=event_spec["guard"]
            guard=self._build_guard(guard_spec,schema,body,models,document)
            action=event_spec["action"]; atype=str(action["type"]); event_name=str(event_spec["name"])
            if atype=="remove_vehicle":
                note=str(action.get("note",f"remove {vid}"))
                def handler(ctx, _vid=vid, _note=note):
                    return UniverseMutation(remove=(_vid,), note=_note)
            elif atype=="rigid_separation":
                parent_I=np.asarray(action["parent_inertia_b"],dtype=float)
                rel=np.asarray(action.get("relative_separation_velocity_i",[0,0,0]),dtype=float)
                conserve=bool(action.get("conserve_angular_momentum",True)); note=str(action.get("note","rigid separation"))
                retained=dict(action["retained"]); detached=dict(action["detached"])
                def handler(ctx, _r=retained, _d=detached, _pI=parent_I, _rel=rel, _conserve=conserve, _note=note):
                    src=ctx.source; vals=src.schema.unpack(src.state)
                    required={"position","velocity","attitude","angular_rate","mass"}
                    if not required.issubset(vals):
                        raise MissionCompilationError("rigid_separation requires a 6-DOF source vehicle")
                    result=separate_two_rigid_bodies(
                        parent_mass=float(vals["mass"]), parent_position_i=np.asarray(vals["position"]),
                        parent_velocity_i=np.asarray(vals["velocity"]), parent_attitude_bi=np.asarray(vals["attitude"]),
                        parent_angular_rate_b=np.asarray(vals["angular_rate"]), parent_inertia_b=_pI,
                        retained_mass=float(_r["mass"]), detached_mass=float(_d["mass"]),
                        retained_offset_b=np.asarray(_r["offset_b"],dtype=float), detached_offset_b=np.asarray(_d["offset_b"],dtype=float),
                        retained_inertia_b=np.asarray(_r["inertia_b"],dtype=float), detached_inertia_b=np.asarray(_d["inertia_b"],dtype=float),
                        relative_separation_velocity_i=_rel, conserve_angular_momentum=_conserve,
                    )
                    specs=[]
                    for child_cfg, child_state in ((_r,result.retained),(_d,result.detached)):
                        child_id=str(child_cfg["vehicle_id"]); tmpl=templates[str(child_cfg["template"])]
                        core_schema=core_6dof_schema()
                        y=core_schema.pack({"position":child_state.position_i,"velocity":child_state.velocity_i,
                                           "attitude":child_state.attitude_bi,"angular_rate":child_state.angular_rate_b,
                                           "mass":child_state.mass})
                        specs.append(self._phase_spec(document,child_id,tmpl,0,core_schema,y,bodies,environments,solvers,default_integrator,models))
                    remove=() if vid in {s.vehicle_id for s in specs} else (vid,)
                    return UniverseMutation(remove=remove,upsert=tuple(specs),note=_note)
            elif ":" in atype:
                handler = self.registry.build("event_action", atype, self._plugin_spec(atype, action), {
                    "document": document, "vehicle_id": vid, "schema": schema, "body": body,
                    "vehicle_spec": vspec, "phase_index": phase_index, "bodies": bodies,
                    "environments": environments, "solvers": solvers, "default_integrator": default_integrator,
                    "models": models, "compiler": self,
                })
                if not callable(handler):
                    raise MissionCompilationError(f"plugin event action {atype!r} must build a callable handler")
            else:
                continue
            out.append(VehicleEvent(event_name,guard,direction=float(guard_spec.get("direction",0.0)),
                                    priority=int(event_spec.get("priority",0)),handler=handler))
        return out

    def _transition_state(self, src: StateSchema, dst: StateSchema, state: np.ndarray, transition: Mapping[str,Any]) -> np.ndarray:
        if src.layout_hash == dst.layout_hash:
            return np.asarray(state,dtype=float).copy()
        src_keys = {f.key for f in src.fields}; dst_keys = {f.key for f in dst.fields}
        if "attitude" not in src_keys and "attitude" in dst_keys:
            attitude = transition.get("attitude"); rate = transition.get("angular_rate", [0,0,0])
            return promote_3dof_to_6dof(state, attitude=None if attitude is None else np.asarray(attitude,dtype=float),
                                        angular_rate_b=np.asarray(rate,dtype=float), source_schema=src, target_schema=dst)
        if "attitude" in src_keys and "attitude" not in dst_keys:
            return demote_6dof_to_3dof(state, source_schema=src, target_schema=dst)
        return map_state_fields(src, dst, state, defaults=transition.get("defaults", {}))

    def _build_rhs(self, schema: StateSchema, dynamics: Mapping[str,Any], body: Any,
                   env: Any, models: Mapping[str, Any], document: MissionDocument,
                   vehicle_id: str, phase: Mapping[str, Any]) -> Callable[[float,np.ndarray],np.ndarray]:
        typ = str(dynamics.get("type", "")) if isinstance(dynamics, Mapping) else ""
        if typ and ":" in typ:
            rhs = self.registry.build("dynamics", typ, self._plugin_spec(typ, dynamics), {
                "schema": schema, "body": body, "environment": env, "models": models,
                "document": document, "vehicle_id": vehicle_id, "phase": phase, "compiler": self,
            })
            if callable(rhs): return rhs
            if hasattr(rhs, "rhs") and callable(rhs.rhs): return rhs.rhs
            raise MissionCompilationError(f"plugin dynamics {typ!r} must build a callable RHS or object with rhs")

        _strict_keys(dynamics, {"gravity","ideal_rocket","constant_body_rocket","inertia_b"}, "phase.dynamics")
        gravity = body.gravity if bool(dynamics.get("gravity", True)) else None
        is6 = "attitude" in {f.key for f in schema.fields}
        if not is6:
            accel=[]; owners=[]
            rocket_spec = dynamics.get("ideal_rocket")
            if rocket_spec is not None:
                r = _as_mapping(rocket_spec, "phase.dynamics.ideal_rocket")
                _strict_keys(r,{"exhaust_velocity","mass_flow","direction_i"},"phase.dynamics.ideal_rocket")
                rocket=IdealRocket(float(r["exhaust_velocity"]),float(r["mass_flow"]),np.asarray(r.get("direction_i",[1,0,0]),dtype=float))
                accel.append(rocket); owners.append(rocket)
            return DynamicsAssembler(schema,[TranslationalKinematics(gravity,tuple(accel)),*owners]).rhs

        inertia=np.asarray(dynamics.get("inertia_b",np.eye(3)),dtype=float)
        mp=ConstantMassProperties(inertia); wrenches=[]; owners=[]
        rocket_spec=dynamics.get("constant_body_rocket")
        if rocket_spec is not None:
            r=_as_mapping(rocket_spec,"phase.dynamics.constant_body_rocket")
            _strict_keys(r,{"thrust","mass_flow","direction_b","mount_b","cg_b"},"phase.dynamics.constant_body_rocket")
            rocket=_ConstantBodyRocket6DOF(float(r["thrust"]),float(r.get("mass_flow",0.0)),
                                          np.asarray(r.get("direction_b",[1,0,0]),dtype=float),
                                          np.asarray(r.get("mount_b",[0,0,0]),dtype=float),
                                          np.asarray(r.get("cg_b",[0,0,0]),dtype=float))
            wrenches.append(rocket); owners.append(rocket)
        return DynamicsAssembler(schema,[RigidBody6DOFDynamics(mp,gravity,tuple(wrenches)),QuaternionKinematics(),*owners]).rhs

    def _build_guard(self, spec: Mapping[str,Any], schema: StateSchema, body: Any,
                     models: Mapping[str, Any], document: MissionDocument):
        typ=str(spec["type"])
        if ":" in typ:
            guard = self.registry.build("guard", typ, self._plugin_spec(typ, spec), {
                "schema": schema, "body": body, "models": models, "document": document,
            })
            if not callable(guard): raise MissionCompilationError(f"plugin guard {typ!r} must build a callable")
            return guard
        value=float(spec["value"])
        if typ=="time": return lambda t,y: float(t-value)
        if typ=="altitude":
            sl=schema.sl("position"); return lambda t,y: float(body.altitude(np.asarray(y[sl],dtype=float))-value)
        field=str(spec["field"]); sl=schema.sl(field); index=spec.get("index")
        if index is None: return lambda t,y: float(np.asarray(y[sl]).reshape(-1)[0]-value)
        idx=int(index); return lambda t,y: float(np.asarray(y[sl]).reshape(-1)[idx]-value)

    def _parse_optimization(self, document: MissionDocument) -> MissionOptimizationDeclaration | None:
        spec=document.raw.get("optimization")
        if spec is None: return None
        dvs=[]; pointers=[]
        for d in spec["design_variables"]:
            lower=float(d["lower"]); upper=float(d["upper"])
            initial=float(d.get("initial", pointer_get(document.raw,str(d["pointer"]))))
            dvs.append(DesignVariable(str(d["name"]),initial,lower,upper,float(d.get("scale",1.0))))
            pointers.append(str(d["pointer"]))
        objspec=spec["objective"]
        obj=MetricObjective(metric=str(objspec["metric"]),sense=str(objspec.get("sense","minimize")),scale=float(objspec.get("scale",1.0)))
        cons=tuple(MetricConstraint(metric=str(c["metric"]), lower=-np.inf if c.get("lower") is None else float(c["lower"]),
                                    upper=np.inf if c.get("upper") is None else float(c["upper"]), scale=float(c.get("scale",1.0)))
                   for c in spec.get("constraints",[]))
        return MissionOptimizationDeclaration(tuple(dvs),tuple(pointers),obj,cons,
                                              str(spec.get("method","SLSQP")),int(spec.get("max_iterations",100)))

    def _parse_dispersions(self, document: MissionDocument) -> tuple[MissionDispersionDeclaration,...]:
        mc=document.raw.get("monte_carlo")
        if mc is None: return ()
        out=[]
        for d in mc.get("dispersions",[]):
            params={k:float(d[k]) for k in ("mean","std","low","high") if k in d}
            out.append(MissionDispersionDeclaration(str(d["name"]),str(d["pointer"]),str(d["distribution"]),MappingProxyType(params)))
        return tuple(out)

    def build_trajectory_problem(self, document: MissionDocument) -> TrajectoryProblem:
        compiled=self.compile(document); decl=compiled.optimization
        if decl is None: raise MissionCompilationError("mission contains no optimization declaration")
        space=DesignSpace(list(decl.design_variables))
        def evaluator(values: Mapping[str,float]):
            overrides={ptr:float(values[dv.name]) for dv,ptr in zip(decl.design_variables,decl.pointers)}
            report=self.compile(document.with_overrides(overrides)).run()
            metrics=dict(report.outputs); metrics["mission_success"]=1.0 if report.success else 0.0
            return metrics
        return TrajectoryProblem(space,evaluator,decl.objective,decl.constraints)

    def optimize(self, document: MissionDocument):
        compiled=self.compile(document); decl=compiled.optimization
        if decl is None: raise MissionCompilationError("mission contains no optimization declaration")
        problem=self.build_trajectory_problem(document)
        if ":" in decl.method:
            built=self.registry.build("optimizer",decl.method,{"max_iterations":decl.max_iterations},{
                "problem":problem,"declaration":decl,"document":document,"compiler":self,
            })
            if hasattr(built,"solve") and callable(built.solve): return built.solve(problem)
            if callable(built): return built(problem)
            if hasattr(built,"success"): return built
            raise MissionCompilationError(f"plugin optimizer {decl.method!r} returned unsupported object")
        return TrajectoryOptimizer(OptimizationSettings(method=decl.method,maxiter=decl.max_iterations)).solve(problem)

    def sample_monte_carlo(self, document: MissionDocument, cases: int | None = None, seed: int | None = None) -> tuple[Mapping[str,Any], ...]:
        compiled=self.compile(document); mc=document.raw.get("monte_carlo") or {}
        n=int(cases if cases is not None else mc.get("cases",1)); rng=np.random.default_rng(int(seed if seed is not None else mc.get("seed",0)))
        samples=[]
        for i in range(n):
            overrides={}; values={}
            for d in compiled.dispersions:
                value=float(rng.normal(d.parameters["mean"],d.parameters["std"])) if d.distribution=="normal" else float(rng.uniform(d.parameters["low"],d.parameters["high"]))
                overrides[d.pointer]=value; values[d.name]=value
            samples.append(MappingProxyType({"index":i,"overrides":MappingProxyType(overrides),"values":MappingProxyType(values)}))
        return tuple(samples)

def _extract_outputs(result: UniverseResult, specs: Sequence[Mapping[str,Any]], bodies: Mapping[str,Any],
                     registry: MissionRegistry | None = None, models: Mapping[str,Any] | None = None) -> dict[str,float]:
    out: dict[str,float]={}; models=models or {}
    for spec in specs:
        name=str(spec["name"]); typ=str(spec["type"])
        if ":" in typ:
            if registry is None: raise MissionCompilationError(f"plugin output {typ!r} requires a mission registry")
            value=registry.build("output",typ,_as_mapping(spec.get("config",{}),f"outputs.{name}.config"),{
                "result":result,"spec":spec,"bodies":bodies,"models":models,"outputs_so_far":MappingProxyType(dict(out)),
            })
            out[name]=float(value() if callable(value) else value); continue
        if typ=="time": out[name]=float(result.end_time); continue
        if typ=="vehicle_count": out[name]=float(len(result.final_vehicles)); continue
        vid=str(spec["vehicle"])
        if vid not in result.final_vehicles: out[name]=math.nan; continue
        snap=result.final_vehicles[vid]; values=snap.schema.unpack(snap.state)
        if typ=="speed": out[name]=float(np.linalg.norm(values["velocity"])); continue
        if typ=="altitude":
            body_name=spec.get("body") or snap.model_context.get("body")
            if body_name not in bodies: raise MissionCompilationError(f"output {name!r} cannot resolve body for altitude")
            out[name]=float(bodies[str(body_name)].altitude(values["position"])); continue
        field=str(spec["field"]); value=values[field]; arr=np.asarray(value,dtype=float).reshape(-1); idx=int(spec.get("index",0))
        if idx >= arr.size: raise MissionCompilationError(f"output {name!r} state index out of range")
        out[name]=float(arr[idx])
    return out


def _jsonable(value: Any) -> Any:
    if isinstance(value,np.ndarray): return value.tolist()
    if isinstance(value,np.generic): return value.item()
    if isinstance(value,Mapping): return {str(k):_jsonable(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)): return [_jsonable(v) for v in value]
    return value


def save_report(report: MissionRunReport, path: str | Path) -> Path:
    p=Path(path); p.parent.mkdir(parents=True,exist_ok=True)
    with p.open("w",encoding="utf-8") as f:
        json.dump(report.to_json_dict(),f,indent=2,sort_keys=True,allow_nan=False)
    return p
