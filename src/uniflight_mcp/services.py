from __future__ import annotations

import asyncio
import csv
import inspect
import io
import json
import math
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping, Sequence


def _schedule_progress(value: Any) -> None:
    if not inspect.isawaitable(value):
        return
    try:
        loop = asyncio.get_running_loop()
    except RuntimeError:
        closer = getattr(value, "close", None)
        if callable(closer):
            closer()
        return
    loop.create_task(value)  # type: ignore[arg-type]


def _progress_current(progress: Any) -> int | None:
    try:
        current = progress.current
    except Exception:
        return None
    return None if current is None else int(current)


def _bump(progress: Any, *, completed: int | None = None, total: int | None = None) -> None:
    if progress is None:
        return
    if total is not None:
        setter = getattr(progress, "set_total", None)
        if callable(setter):
            _schedule_progress(setter(total))
    if completed is None:
        return
    setter = getattr(progress, "set_completed", None)
    if callable(setter):
        _schedule_progress(setter(completed))
        return
    increment = getattr(progress, "increment", None)
    if not callable(increment):
        return
    current = _progress_current(progress)
    delta = int(completed) if current is None else int(completed) - current
    if delta > 0:
        _schedule_progress(increment(delta))

import numpy as np
import yaml

import uniflight
from uniflight import (
    MissionCompilationError, MissionCompiler, MissionDocument, MissionValidationError,
    PlanetaryEnvironment, PluginManager, VacuumAtmosphere, compute_body_flow_state,
    compute_flow_state, installed_plugin_summary, mission_json_schema, mission_sha256,
    pointer_set, validate_mission_dict,
)
from uniflight.engineering_data import EngineeringDataCatalog, EngineeringTable
from uniflight.mission import _extract_outputs
from uniflight.hpc import ProcessBackend, SerialBackend
from uniflight.analysis import (
    MissionCampaignRunner, MissionMonteCarlo, MonteCarloVariable, OptimizationBatch,
    OptimizationStart, ParameterSweep, SobolSensitivity, SobolVariable, SweepVariable,
    mission_case_worker, optimization_case_worker, summarize_numeric_metrics,
)
from uniflight.montecarlo import Dispersion, NormalDispersion, UniformDispersion
from uniflight.plugins import PLUGIN_API_VERSION
from uniflight.result_store import SQLiteResultStore
from uniflight.verification import ReferenceTimeHistory, TolerancePolicy, compare_time_histories, observed_order
from uniflight.verification_cases import run_builtin_verification

from ._version import __version__ as _mcp_version
from .artifacts import ArtifactStore
from .auth import AuthorizationContext
from .config import ServerConfig
from .cursors import CursorCodec
from .errors import DomainError
from .ids import mission_id, run_id, snapshot_id, verification_id
from .models import ArtifactRef, PageRequest, jsonable
from .paths import resolve_under


def _now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _named_objects(value: Any, key: str = "id") -> list[dict[str, Any]]:
    if value is None:
        return []
    if isinstance(value, list):
        items = []
        for item in value:
            items.append(jsonable(item) if isinstance(item, Mapping) else {key: str(item)})
        return items
    if isinstance(value, Mapping):
        items = []
        for name, item in value.items():
            if isinstance(item, Mapping):
                items.append({key: str(name), **jsonable(item)})
            else:
                items.append({key: str(name), "value": jsonable(item)})
        return items
    return []


def _si_units() -> dict[str, str]:
    return {"system": "SI"}


def _violation_objects(values: Sequence[Any] | None) -> list[dict[str, Any]]:
    items = []
    for value in values or ():
        items.append(jsonable(value) if isinstance(value, Mapping) else {"message": str(value)})
    return items


def _case_error(raw: Any, correlation_id: str) -> dict[str, Any] | None:
    if raw is None or raw == "":
        return None
    if isinstance(raw, Mapping) and raw.get("code") and raw.get("message") is not None:
        return {
            "code": str(raw.get("code") or "CASE_FAILED"),
            "message": str(raw.get("message") or ""),
            "recoverable": bool(raw.get("recoverable", True)),
            "path": raw.get("path"),
            "details": dict(raw.get("details") or {}),
            "correlation_id": str(raw.get("correlation_id") or correlation_id),
        }
    return {
        "code": "CASE_FAILED",
        "message": str(raw),
        "recoverable": True,
        "path": None,
        "details": {},
        "correlation_id": correlation_id,
    }


def _alignment_object(alignment: Any) -> dict[str, Any]:
    if isinstance(alignment, Mapping):
        return dict(alignment)
    if isinstance(alignment, str) and alignment:
        return {"method": alignment}
    return {"method": "timestamp_grid"}


def _channel_results(results: Sequence[Any]) -> dict[str, Any]:
    items: dict[str, Any] = {}
    for result in results:
        row = result.to_dict() if hasattr(result, "to_dict") else dict(result)
        key = str(row.get("title") or row.get("case_id") or f"channel-{len(items)}")
        items[key] = row
    return items


def _objective_name(declaration: Mapping[str, Any] | None) -> str | None:
    objective = (declaration or {}).get("objective") or {}
    if not isinstance(objective, Mapping):
        return None
    name = objective.get("name") or objective.get("metric")
    return str(name) if name else None


def _enable_history_sampling(raw: dict[str, Any]) -> bool:
    changed = False
    for slot in (raw.get("solvers") or {}).values():
        if isinstance(slot, dict) and str(slot.get("type", "")).lower() == "rk4" and "save_every_step" not in slot:
            slot["save_every_step"] = True
            changed = True
    return changed


def _plugin_record(rec: Mapping[str, Any]) -> dict[str, Any]:
    caps = rec.get("capabilities") or []
    if not isinstance(caps, list):
        caps = []
    return {
        "plugin_id": str(rec.get("plugin_id") or rec.get("name") or ""),
        "version": str(rec.get("version") or "unknown"),
        "api_version": str(rec.get("api_version") or PLUGIN_API_VERSION),
        "distribution": str(rec.get("distribution") or rec.get("entry_point") or "unknown"),
        "capability_count": len(caps) if caps else int(rec.get("capability_count") or 0),
    }


def _write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True, default=str), encoding="utf-8")


def _read_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _coerce_scientific_strings(value: Any) -> Any:
    if isinstance(value, str) and "e" in value.lower():
        stripped = value.replace("+", "").replace("-", "").replace(".", "").replace("e", "").replace("E", "")
        if stripped.isdigit():
            try:
                return float(value)
            except ValueError:
                return value
        return value
    if isinstance(value, list):
        return [_coerce_scientific_strings(item) for item in value]
    if isinstance(value, Mapping):
        return {key: _coerce_scientific_strings(item) for key, item in value.items()}
    return value


def parse_document(document: Any, fmt: str | None) -> dict[str, Any]:
    if isinstance(document, Mapping):
        return jsonable(document)
    if not isinstance(document, str):
        raise DomainError("INVALID_REQUEST", "document must be an object or a YAML/TOML/JSON string")
    kind = (fmt or "yaml").lower()
    if kind == "json":
        return json.loads(document)
    if kind == "toml":
        import tomllib
        return tomllib.loads(document)
    return _coerce_scientific_strings(yaml.safe_load(document))


def resolve_ref(ref: Mapping[str, Any] | None, prefix: str, key: str) -> str:
    if not ref:
        raise DomainError("INVALID_REQUEST", f"{key} is required")
    if ref.get(key):
        return str(ref[key])
    uri = str(ref.get("uri") or "")
    token = f"uniflight://{prefix}/"
    if uri.startswith(token):
        return uri[len(token):].split("/", 1)[0]
    raise DomainError("INVALID_REQUEST", f"{key} or uri is required")


class AppServices:
    def __init__(self, config: ServerConfig):
        self.config = config
        self.artifacts = ArtifactStore(config)
        self.cursors = CursorCodec(config.cursor_secret, config.cursor_ttl_s)
        self.missions = MissionService(self)
        self.runs = RunService(self)
        self.data = DataService(self)
        self.analysis = AnalysisService(self)
        self.verification = VerificationService(self)
        self.plugins = PluginService(self)
        self.plugin_manager = PluginManager()
        self._catalog = EngineeringDataCatalog()
        self.current_mission: dict[str, str] = {}

    def close(self) -> None:
        return None

    def provenance(self, auth: AuthorizationContext, **extra: Any) -> dict[str, Any]:
        payload = {
            "server_version": _mcp_version,
            "uniflight_version": uniflight.__version__,
            "created_at": _now(),
            "tenant": auth.tenant_id,
            "correlation_id": auth.correlation_id,
        }
        payload.update(extra)
        return payload


class MissionService:
    def __init__(self, app: AppServices):
        self.app = app

    def _dir(self, mid: str, tenant: str) -> Path:
        return self.app.config.missions_dir / tenant / mid

    def persist(self, raw: Mapping[str, Any], *, tenant: str, parent: str | None = None) -> dict[str, Any]:
        mid = mission_id()
        root = self._dir(mid, tenant)
        root.mkdir(parents=True, exist_ok=True)
        doc = MissionDocument(dict(raw), None, root)
        _write_json(root / "mission.json", doc.mutable_copy())
        meta = {
            "mission_id": mid,
            "mdl_id": doc.mission_id,
            "digest_sha256": doc.digest_sha256,
            "parent_mission_id": parent,
            "created_at": _now(),
            "tenant": tenant,
        }
        _write_json(root / "meta.json", meta)
        return meta

    def load(self, mid: str, auth: AuthorizationContext) -> tuple[dict[str, Any], MissionDocument]:
        root = self._dir(mid, auth.tenant_id)
        if not (root / "meta.json").exists():
            raise DomainError("NOT_FOUND", f"mission {mid} was not found")
        meta = _read_json(root / "meta.json")
        if meta.get("tenant") not in {auth.tenant_id, "local"}:
            raise DomainError("NOT_FOUND", f"mission {mid} was not found")
        path = root / "mission.json"
        raw = _read_json(path)
        return meta, MissionDocument(raw, path, root)

    def from_ref(self, ref: Mapping[str, Any] | None, auth: AuthorizationContext) -> tuple[str, MissionDocument]:
        mid = resolve_ref(ref, "missions", "mission_id")
        if not mid.startswith("mis_"):
            raise DomainError("INVALID_REQUEST", "mission_id must start with mis_")
        meta, doc = self.load(mid, auth)
        return meta["mission_id"], doc

    def validate(self, document: Any, fmt: str | None, base_uri: str | None,
                 auth: AuthorizationContext) -> dict[str, Any]:
        try:
            raw = parse_document(document, fmt)
            if base_uri:
                raw.setdefault("metadata", {})
            validate_mission_dict(raw)
            digest = mission_sha256(raw)
            MissionCompiler().compile(MissionDocument(raw, None, self.app.config.temp_dir))
            return {"ok": True, "valid": True, "errors": [], "digest_sha256": digest}
        except (MissionValidationError, MissionCompilationError) as exc:
            return {"ok": True, "valid": False, "errors": [str(exc)], "digest_sha256": None}
        except DomainError:
            raise

    def inspect(self, ref: Mapping[str, Any] | None, auth: AuthorizationContext) -> dict[str, Any]:
        if ref is None:
            mid = self.app.current_mission.get(auth.tenant_id)
            if not mid:
                raise DomainError("INVALID_REQUEST", "mission is required")
            ref = {"mission_id": mid}
        mid, doc = self.from_ref(ref, auth)
        compiled = MissionCompiler().compile(doc)
        raw = doc.mutable_copy()
        return {
            "ok": True,
            "mission": {"mcp_id": mid, "id": doc.mission_id, "title": raw.get("mission", {}).get("title"),
                        "t_span": list(raw.get("mission", {}).get("t_span", [])), "sha256": doc.digest_sha256},
            "bodies": _named_objects(raw.get("bodies") or {}),
            "vehicles": _named_objects(raw.get("vehicles") or {}),
            "events": _named_objects(raw.get("events") or []),
            "solvers": _named_objects(raw.get("solvers") or {}),
            "datasets": [{"dataset_id": i, "version": v, "sha256": s} for i, v, s in compiled.data_catalog.inventory()],
            "plugins": [{"plugin_id": i, "version": v, "api_version": a} for i, v, a in compiled.plugin_inventory],
            "outputs": _named_objects(raw.get("outputs") or []),
        }

    def compile(self, document: Any, fmt: str | None, persist: bool, auth: AuthorizationContext) -> dict[str, Any]:
        raw = parse_document(document, fmt)
        try:
            validate_mission_dict(raw)
            compiled = MissionCompiler().compile(MissionDocument(raw, None, self.app.config.temp_dir))
            self.app.data.bind_catalog(compiled.data_catalog)
        except MissionValidationError as exc:
            raise DomainError("MISSION_VALIDATION_ERROR", str(exc)) from exc
        except MissionCompilationError as exc:
            raise DomainError("MISSION_COMPILATION_ERROR", str(exc)) from exc
        warnings: list[str] = []
        if persist:
            meta = self.persist(raw, tenant=auth.tenant_id)
            self.app.current_mission[auth.tenant_id] = meta["mission_id"]
            artifact = self.app.artifacts.put_json(compiled.document.mutable_copy(), tenant=auth.tenant_id)
            return {
                "ok": True, "mission_id": meta["mission_id"],
                "uri": f"uniflight://missions/{meta['mission_id']}",
                "digest_sha256": meta["digest_sha256"],
                "normalized_artifact": artifact.model_dump(),
                "warnings": warnings,
            }
        artifact = self.app.artifacts.put_json(raw, tenant=auth.tenant_id)
        return {
            "ok": True, "mission_id": "", "uri": "",
            "digest_sha256": compiled.document.digest_sha256,
            "normalized_artifact": artifact.model_dump(), "warnings": warnings,
        }

    def apply_overrides(self, ref: Mapping[str, Any], overrides: Sequence[Mapping[str, Any]],
                        auth: AuthorizationContext) -> dict[str, Any]:
        parent, doc = self.from_ref(ref, auth)
        raw = doc.mutable_copy()
        applied = []
        try:
            for item in overrides:
                pointer = str(item["pointer"])
                pointer_set(raw, pointer, item["value"])
                applied.append({"pointer": pointer, "value": jsonable(item["value"])})
            child = self.persist(raw, tenant=auth.tenant_id, parent=parent)
        except (KeyError, MissionValidationError) as exc:
            raise DomainError("MISSION_VALIDATION_ERROR", str(exc)) from exc
        self.app.current_mission[auth.tenant_id] = child["mission_id"]
        return {
            "ok": True, "mission_id": child["mission_id"],
            "uri": f"uniflight://missions/{child['mission_id']}",
            "digest_sha256": child["digest_sha256"],
            "parent_mission_id": parent,
            "applied_overrides": applied,
        }


class RunService:
    def __init__(self, app: AppServices):
        self.app = app

    def _dir(self, rid: str, tenant: str) -> Path:
        return self.app.config.runs_dir / tenant / rid

    def load(self, rid: str, auth: AuthorizationContext) -> dict[str, Any]:
        path = self._dir(rid, auth.tenant_id) / "run.json"
        if not path.exists():
            raise DomainError("NOT_FOUND", f"run {rid} was not found")
        return _read_json(path)

    def from_ref(self, ref: Mapping[str, Any] | None, auth: AuthorizationContext) -> dict[str, Any]:
        rid = resolve_ref(ref, "runs", "run_id")
        if not rid.startswith("run_"):
            raise DomainError("INVALID_REQUEST", "run_id must start with run_")
        return self.load(rid, auth)

    def _persist(self, record: dict[str, Any], auth: AuthorizationContext) -> None:
        root = self._dir(record["run_id"], auth.tenant_id)
        _write_json(root / "run.json", record)

    def run(self, mission_ref: Mapping[str, Any], solver_override: Mapping[str, Any] | None,
            save_history: bool, output_interval_s: float | None, auth: AuthorizationContext,
            progress=None) -> dict[str, Any]:
        mid, doc = self.app.missions.from_ref(mission_ref, auth)
        mission_sha = doc.digest_sha256
        raw = doc.mutable_copy()
        changed = bool(save_history and _enable_history_sampling(raw))
        if solver_override:
            solvers = raw.setdefault("solvers", {})
            default = raw.setdefault("mission", {}).get("default_solver")
            profile = solver_override.get("profile") or default
            if profile and profile in solvers:
                slot = solvers[profile]
                for key, dest in (("kind", "type"), ("method", "method"), ("rtol", "rtol"),
                                  ("atol", "atol"), ("max_step_s", "max_step"), ("step_s", "step")):
                    if solver_override.get(key) is not None:
                        slot[dest] = solver_override[key]
                changed = True
        if changed:
            doc = MissionDocument(raw, None, doc.base_directory)
        try:
            compiled = MissionCompiler().compile(doc)
            self.app.data.bind_catalog(compiled.data_catalog)
            _bump(progress, total=1)
            result = compiled.engine.run(compiled.t_span, compiled.vehicles)
        except MissionCompilationError as exc:
            raise DomainError("MISSION_COMPILATION_ERROR", str(exc)) from exc
        except Exception as exc:
            raise DomainError("SOLVER_FAILURE", str(exc)) from exc
        outputs = dict(_extract_outputs(result, compiled.output_specs, compiled.bodies,
                                        compiled.registry, compiled.models))
        events = [dict(e) if isinstance(e, Mapping) else {
            "time": e.time, "vehicle": e.vehicle_id, "event": e.event_name,
            "priority": e.priority, "note": e.mutation_note,
            "direction": int(getattr(e, "direction", 0) or 0),
            "action": str(getattr(getattr(e, "action", None), "name", None) or e.mutation_note or ""),
            "tied_group": getattr(e, "tied_group", None),
            "pre_mode": getattr(e, "pre_mode", None),
            "post_mode": getattr(e, "post_mode", None),
        } for e in result.events]
        finals = {}
        for vid, snap in result.final_vehicles.items():
            unpacked = snap.schema.unpack(snap.state)
            finals[vid] = {
                "mode": snap.mode, "dof": snap.dof, "schema_hash": snap.schema.layout_hash,
                "state": jsonable(unpacked),
            }
        success = bool(result.success)
        message = result.message
        end_time = float(result.end_time)
        rid = run_id()
        history: dict[str, list[dict[str, Any]]] = {}
        if save_history:
            for vid, segs in result.segments.items():
                rows: list[dict[str, Any]] = []
                for si, seg in enumerate(segs):
                    for t, y in zip(np.asarray(seg.times), np.asarray(seg.states)):
                        if output_interval_s and rows and (t - rows[-1]["time_s"]) < output_interval_s * 0.5:
                            continue
                        unpacked = seg.schema.unpack(y)
                        rows.append({
                            "time_s": float(t), "segment_index": si, "mode": seg.mode,
                            "state": jsonable(unpacked),
                            "schema_hash": seg.schema.layout_hash,
                        })
                history[vid] = rows
        record = {
            "run_id": rid, "tenant": auth.tenant_id, "mission_id": mid,
            "mission_sha256": mission_sha, "mdl_id": doc.mission_id,
            "status": "completed" if success else "failed",
            "message": message, "final_time_s": end_time,
            "outputs": outputs, "events": events, "finals": finals,
            "history": history, "warnings": [],
            "dataset_inventory": [{"dataset_id": i, "version": v, "sha256": s}
                                  for i, v, s in compiled.data_catalog.inventory()],
            "plugin_inventory": [{"plugin_id": i, "version": v, "api_version": a}
                                 for i, v, a in compiled.plugin_inventory],
            "solver": jsonable((doc.mutable_copy().get("solvers") or {})),
            "seed": (doc.mutable_copy().get("mission") or {}).get("seed"),
        }
        self._persist(record, auth)
        _bump(progress, completed=1)
        prov = self.app.provenance(
            auth, mission_id=mid, mission_sha256=mission_sha,
            solver=record["solver"], datasets=record["dataset_inventory"],
            plugins=record["plugin_inventory"], seed=record["seed"],
        )
        return {
            "ok": True, "run_id": rid, "uri": f"uniflight://runs/{rid}",
            "status": record["status"],
            "summary_uri": f"uniflight://runs/{rid}/summary",
            "manifest_uri": f"uniflight://runs/{rid}/manifest",
            "requested_outputs": outputs, "artifacts": [], "provenance": prov,
            "warnings": [],
        }

    def summary(self, ref: Mapping[str, Any] | None, auth: AuthorizationContext) -> dict[str, Any]:
        rec = self.from_ref(ref, auth)
        return {
            "ok": True, "run_id": rec["run_id"], "status": rec["status"],
            "mission_sha256": rec["mission_sha256"], "final_time_s": rec["final_time_s"],
            "vehicles": list((rec.get("finals") or {}).keys()),
            "requested_outputs": rec.get("outputs") or {},
            "event_count": len(rec.get("events") or []), "warnings": rec.get("warnings") or [],
            "provenance": self.app.provenance(auth, mission_sha256=rec["mission_sha256"], solver=rec.get("solver")),
        }

    def events(self, ref: Mapping[str, Any], filters: Mapping[str, Any],
               page: PageRequest | None, auth: AuthorizationContext) -> dict[str, Any]:
        rec = self.from_ref(ref, auth)
        items = []
        for seq, ev in enumerate(rec.get("events") or []):
            if filters.get("vehicle_id") and ev.get("vehicle") != filters["vehicle_id"]:
                continue
            if filters.get("event_id") and ev.get("event") != filters["event_id"]:
                continue
            t = float(ev.get("time", 0.0))
            if filters.get("start_time_s") is not None and t < float(filters["start_time_s"]):
                continue
            if filters.get("end_time_s") is not None and t > float(filters["end_time_s"]):
                continue
            items.append({
                "sequence": seq,
                "time_s": t,
                "vehicle_id": str(ev.get("vehicle") or ev.get("vehicle_id") or ""),
                "event_id": str(ev.get("event") or ev.get("event_id") or ""),
                "priority": int(ev.get("priority") or 0),
                "direction": int(ev.get("direction") or 0),
                "action": str(ev.get("action") or ev.get("note") or ""),
                "tied_group": ev.get("tied_group"),
                "pre_mode": ev.get("pre_mode"),
                "post_mode": ev.get("post_mode"),
            })
        items.sort(key=lambda r: (r["time_s"], -int(r.get("priority") or 0), r["sequence"]))
        window, info = self.app.cursors.paginate(
            items, page, tool="simulation_events", tenant=auth.tenant_id, filters=dict(filters),
        )
        return {"ok": True, "items": window, "page": info.model_dump()}

    def state_at(self, ref: Mapping[str, Any], vehicle_id: str, time_s: float,
                 fields: list[str] | None, interpolation: str | None,
                 auth: AuthorizationContext) -> dict[str, Any]:
        rec = self.from_ref(ref, auth)
        rows = rec.get("history", {}).get(vehicle_id) or []
        if not rows:
            raise DomainError("INVALID_STATE", f"no history for vehicle {vehicle_id}")
        times = np.array([r["time_s"] for r in rows], dtype=float)
        idx = int(np.argmin(np.abs(times - time_s)))
        row = rows[idx]
        state = dict(row["state"])
        if fields:
            state = {k: state[k] for k in fields if k in state}
        return {
            "ok": True, "vehicle_id": vehicle_id, "time_s": float(row["time_s"]),
            "schema": str(row.get("schema_hash") or ""), "state": state,
            "units": _si_units(), "interpolation": interpolation or "nearest",
        }

    def history(self, ref: Mapping[str, Any], vehicle_id: str, filters: Mapping[str, Any],
                page: PageRequest | None, auth: AuthorizationContext) -> dict[str, Any]:
        rec = self.from_ref(ref, auth)
        rows = list(rec.get("history", {}).get(vehicle_id) or [])
        stride = int(filters.get("stride") or 1)
        if stride < 1:
            raise DomainError("INVALID_REQUEST", "stride must be >= 1")
        items = []
        for r in rows[::stride]:
            t = float(r["time_s"])
            if filters.get("start_time_s") is not None and t < float(filters["start_time_s"]):
                continue
            if filters.get("end_time_s") is not None and t > float(filters["end_time_s"]):
                continue
            state = dict(r["state"])
            if filters.get("fields"):
                state = {k: state[k] for k in filters["fields"] if k in state}
            items.append({
                "time_s": t,
                "segment_index": int(r.get("segment_index") or 0),
                "schema": str(r.get("schema_hash") or ""),
                "values": state,
            })
        items.sort(key=lambda r: (r["time_s"], r["segment_index"]))
        window, info = self.app.cursors.paginate(
            items, page, tool="simulation_vehicle_history", tenant=auth.tenant_id, filters=dict(filters),
        )
        return {"ok": True, "vehicle_id": vehicle_id, "units": _si_units(), "items": window, "page": info.model_dump()}

    def export_csv(self, ref: Mapping[str, Any], vehicle_id: str, fields: list[str],
                   start: float | None, end: float | None, interval: float | None,
                   auth: AuthorizationContext) -> dict[str, Any]:
        rec = self.from_ref(ref, auth)
        rows = rec.get("history", {}).get(vehicle_id) or []
        buf = io.StringIO()
        writer = csv.writer(buf)
        writer.writerow(["time_s", *fields])
        last_t = None
        n = 0
        for r in rows:
            t = float(r["time_s"])
            if start is not None and t < start:
                continue
            if end is not None and t > end:
                continue
            if interval is not None and last_t is not None and (t - last_t) < interval:
                continue
            state = r["state"]
            writer.writerow([t, *[state.get(f) for f in fields]])
            last_t = t
            n += 1
        artifact = self.app.artifacts.put_text(
            buf.getvalue(), media_type="text/csv", suffix=".csv", tenant=auth.tenant_id,
        )
        return {"ok": True, "artifact": artifact.model_dump(), "rows": n, "fields": fields}

    def compare_solvers(self, mission: Mapping[str, Any], solver_a: Mapping[str, Any],
                        solver_b: Mapping[str, Any], metrics: list[str] | None,
                        sample_interval_s: float | None, auth: AuthorizationContext,
                        progress=None) -> dict[str, Any]:
        a = self.run(mission, solver_a, True, sample_interval_s, auth, progress)
        b = self.run(mission, solver_b, True, sample_interval_s, auth, progress)
        rec_a, rec_b = self.load(a["run_id"], auth), self.load(b["run_id"], auth)
        metric_errors = {}
        keys = metrics or list((rec_a.get("outputs") or {}).keys())
        for key in keys:
            va, vb = rec_a.get("outputs", {}).get(key), rec_b.get("outputs", {}).get(key)
            if va is None or vb is None:
                continue
            metric_errors[key] = abs(float(va) - float(vb))
        art = self.app.artifacts.put_json({"metric_errors": metric_errors}, tenant=auth.tenant_id)
        return {
            "ok": True, "run_a": a["run_id"], "run_b": b["run_id"],
            "metric_errors": metric_errors, "comparison_artifact": art.model_dump(),
            "provenance": self.app.provenance(auth, mission_sha256=rec_a["mission_sha256"]),
        }

    def _compiled_for_run(self, rec: Mapping[str, Any], auth: AuthorizationContext):
        _, doc = self.app.missions.load(rec["mission_id"], auth)
        compiled = MissionCompiler().compile(doc)
        self.app.data.bind_catalog(compiled.data_catalog)
        return compiled

    def _environment(self, compiled, environment_id: str | None):
        if environment_id and environment_id in compiled.environments:
            return compiled.environments[environment_id]
        if compiled.environments:
            return next(iter(compiled.environments.values()))
        if compiled.bodies:
            return PlanetaryEnvironment(next(iter(compiled.bodies.values())), VacuumAtmosphere())
        raise DomainError("NOT_FOUND", "no environment is available")

    def environment_sample(self, mission: Mapping[str, Any], environment_id: str,
                           time_s: float, position: list[float], auth: AuthorizationContext) -> dict[str, Any]:
        _, doc = self.app.missions.from_ref(mission, auth)
        compiled = MissionCompiler().compile(doc)
        self.app.data.bind_catalog(compiled.data_catalog)
        env = compiled.environments.get(environment_id)
        if env is None:
            raise DomainError("NOT_FOUND", f"environment {environment_id} was not found")
        sample = env.query(np.asarray(position, dtype=float), time_s)
        atm = sample.atmosphere
        return {
            "ok": True,
            "gravity_i_mps2": np.asarray(sample.gravity_i, dtype=float).tolist(),
            "atmosphere": {
                "altitude_m": float(sample.altitude),
                "temperature_k": float(atm.temperature),
                "pressure_pa": float(atm.pressure),
                "density_kgm3": float(atm.density),
                "viscosity_pas": float(atm.viscosity),
                "speed_of_sound_mps": None if not np.isfinite(atm.speed_of_sound) else float(atm.speed_of_sound),
            },
            "wind_i_mps": np.asarray(sample.wind_velocity_i, dtype=float).tolist(),
            "terrain": {"surface_normal_i": np.asarray(sample.surface_normal_i, dtype=float).tolist()},
            "validity": {"ok": True},
        }

    def flow_state(self, run_ref: Mapping[str, Any], vehicle_id: str, time_s: float,
                   auth: AuthorizationContext) -> dict[str, Any]:
        rec = self.from_ref(run_ref, auth)
        rows = rec.get("history", {}).get(vehicle_id) or []
        if not rows:
            raise DomainError("INVALID_STATE", "no history available")
        row = min(rows, key=lambda r: abs(float(r["time_s"]) - time_s))
        compiled = self._compiled_for_run(rec, auth)
        vehicle = next((v for v in compiled.vehicles if v.vehicle_id == vehicle_id), None)
        env_name = None if vehicle is None else (vehicle.model_context or {}).get("environment")
        env = self._environment(compiled, env_name)
        pos = np.asarray(row["state"].get("position") or [0, 0, 0], dtype=float)
        vel = np.asarray(row["state"].get("velocity") or [0, 0, 0], dtype=float)
        sample = env.query(pos, float(row["time_s"]))
        length = 1.0
        attitude = row["state"].get("attitude")
        if attitude is not None:
            body = compute_body_flow_state(vel, np.asarray(attitude, dtype=float), sample, length)
            flow, alpha, beta, v_b = body.base, body.alpha, body.beta, body.relative_velocity_b
        else:
            flow = compute_flow_state(vel, sample, length)
            alpha = beta = 0.0
            v_b = flow.relative_velocity_i
        kn = flow.knudsen
        return {
            "ok": True,
            "mach": float(flow.mach),
            "reynolds": None if not np.isfinite(flow.reynolds) else float(flow.reynolds),
            "knudsen": None if not np.isfinite(kn) else float(kn),
            "dynamic_pressure_pa": float(flow.dynamic_pressure),
            "alpha_rad": float(alpha),
            "beta_rad": float(beta),
            "relative_velocity_i_mps": np.asarray(flow.relative_velocity_i, dtype=float).tolist(),
            "body_relative_velocity_mps": np.asarray(v_b, dtype=float).tolist(),
        }

    def forces(self, run_ref: Mapping[str, Any], vehicle_id: str, time_s: float,
               auth: AuthorizationContext) -> dict[str, Any]:
        rec = self.from_ref(run_ref, auth)
        rows = rec.get("history", {}).get(vehicle_id) or []
        finals = rec.get("finals", {}).get(vehicle_id) or {}
        state = (min(rows, key=lambda r: abs(float(r["time_s"]) - time_s))["state"]
                 if rows else (finals.get("state") or {}))
        compiled = self._compiled_for_run(rec, auth)
        vehicle = next((v for v in compiled.vehicles if v.vehicle_id == vehicle_id), None)
        env_name = None if vehicle is None else (vehicle.model_context or {}).get("environment")
        env = self._environment(compiled, env_name)
        pos = np.asarray(state.get("position") or [0, 0, 0], dtype=float)
        sample = env.query(pos, time_s)
        mass = float(state.get("mass") or 0.0)
        gravity_force = (mass * np.asarray(sample.gravity_i, dtype=float)).tolist()
        contributions = [{
            "source": "gravity", "frame": "I",
            "force_n": gravity_force, "moment_nm": [0.0, 0.0, 0.0],
        }]
        return {
            "ok": True, "contributions": contributions,
            "net_force_i_n": gravity_force, "net_moment_b_nm": [0.0, 0.0, 0.0],
            "mass_kg": mass,
        }


class DataService:
    def __init__(self, app: AppServices):
        self.app = app
        self._tables: dict[tuple[str, str], EngineeringTable] = {}

    def register_table(self, table: EngineeringTable) -> None:
        if table.provenance is None:
            raise DomainError("DATASET_NOT_FOUND", "table provenance is required")
        self._tables[(table.provenance.dataset_id, table.provenance.version)] = table

    def bind_catalog(self, catalog: EngineeringDataCatalog) -> None:
        for table in getattr(catalog, "_tables", {}).values():
            if getattr(table, "provenance", None) is not None:
                self.register_table(table)

    def _table(self, dataset_id: str, version: str) -> EngineeringTable:
        key = (dataset_id, version)
        if key not in self._tables:
            raise DomainError("DATASET_NOT_FOUND", f"{dataset_id}@{version} was not found")
        return self._tables[key]

    def catalog(self, prefix: str | None, kind: str | None, page: PageRequest | None,
                auth: AuthorizationContext) -> dict[str, Any]:
        items = []
        for (did, ver), table in sorted(self._tables.items()):
            if prefix and not did.startswith(prefix):
                continue
            if kind and getattr(table, "kind", None) not in {kind, None}:
                continue
            items.append({
                "dataset_id": did, "version": ver, "sha256": table.content_sha256(),
                "kind": str(getattr(table, "kind", None) or "table"),
                "axes": [a.name for a in table.axes],
                "outputs": list(table.output_names),
                "source": getattr(table.provenance, "source", None) if table.provenance else None,
            })
        window, info = self.app.cursors.paginate(
            items, page, tool="data_catalog_list", tenant=auth.tenant_id,
            filters={"prefix": prefix, "kind": kind},
        )
        return {"ok": True, "items": window, "page": info.model_dump()}

    def query(self, dataset_id: str, version: str, coordinates: Mapping[str, float],
              outputs: list[str] | None, auth: AuthorizationContext) -> dict[str, Any]:
        table = self._table(dataset_id, version)
        result = table.query(coordinates)
        values = dict(result.values)
        if outputs:
            values = {k: values[k] for k in outputs if k in values}
        units = {}
        for name in values:
            meta = table.output_metadata.get(name)
            units[name] = getattr(meta, "unit", "1") if meta is not None else "1"
        return {
            "ok": True, "dataset_id": dataset_id, "version": version,
            "sha256": table.content_sha256(), "values": jsonable(values),
            "units": units,
            "inside_validity": bool(result.validity_ok),
            "validity_violations": _violation_objects(result.validity_violations),
        }

    def validity(self, dataset_id: str, version: str, coordinates: Mapping[str, float],
                 auth: AuthorizationContext) -> dict[str, Any]:
        table = self._table(dataset_id, version)
        result = table.query(coordinates)
        return {
            "ok": True, "inside_validity": bool(result.validity_ok),
            "violations": _violation_objects(result.validity_violations),
            "interpolation_domain": {"inside": bool(result.in_table_domain)},
            "engineering_validity": {"inside": bool(result.validity_ok)},
        }


class AnalysisService:
    def __init__(self, app: AppServices):
        self.app = app

    def _store_path(self, campaign_id: str, tenant: str, store_uri: str | None) -> Path:
        if store_uri:
            if store_uri.startswith("uniflight://"):
                raise DomainError("INVALID_REQUEST", "store_uri must be a workspace-relative sqlite path")
            return resolve_under(store_uri, self.app.config.allowlisted_roots)
        path = self.app.config.campaigns_dir / tenant / f"{campaign_id}.sqlite"
        path.parent.mkdir(parents=True, exist_ok=True)
        return path

    def _backend(self, spec: Mapping[str, Any] | None):
        spec = spec or {"type": "serial"}
        kind = spec.get("type", "serial")
        if kind == "process":
            return ProcessBackend(max_workers=int(spec.get("workers") or 0))
        if kind == "external":
            raise DomainError("INVALID_REQUEST", "external executor must be configured by the operator")
        return SerialBackend()

    def _runner(self, doc: MissionDocument, campaign_id: str, tenant: str,
                backend_spec: Mapping[str, Any] | None, store_uri: str | None) -> MissionCampaignRunner:
        store = SQLiteResultStore(self._store_path(campaign_id, tenant, store_uri))
        return MissionCampaignRunner(doc.source_path or (doc.base_directory / "mission.json"),
                                     backend=self._backend(backend_spec), store=store)

    def sweep(self, mission: Mapping[str, Any], campaign_id: str, variables: Sequence[Mapping[str, Any]],
              backend: Mapping[str, Any] | None, store_uri: str | None,
              auth: AuthorizationContext, progress=None) -> dict[str, Any]:
        mid, doc = self.app.missions.from_ref(mission, auth)
        vars_ = [SweepVariable(v["name"], v["pointer"], tuple(v["values"])) for v in variables]
        cases = ParameterSweep(vars_).cases()
        if len(cases) > self.app.config.max_campaign_cases:
            raise DomainError("QUOTA_EXCEEDED", "campaign exceeds case quota")
        runner = self._runner(doc, campaign_id, auth.tenant_id, backend, store_uri)
        def _prog(done, total, _case):
            _bump(progress, completed=done, total=total)
        pointers = {v["name"]: v["pointer"] for v in variables}
        exe = runner.run_cases(cases, campaign_id=campaign_id, kind="sweep",
                               worker=mission_case_worker, progress=_prog,
                               metadata={"mission_id": mid, "pointers": pointers})
        store = runner.store
        if store is not None:
            store.close()
        return {
            "ok": True, "campaign_id": campaign_id, "status": "completed",
            "requested_cases": exe.requested_cases, "executed_cases": exe.executed_cases,
            "resumed_cases": exe.resumed_cases, "failed_cases": exe.failed_cases,
            "summary_uri": f"uniflight://campaigns/{campaign_id}/summary",
            "provenance": self.app.provenance(auth, mission_id=mid, mission_sha256=doc.digest_sha256),
        }

    def monte_carlo(self, mission: Mapping[str, Any], campaign_id: str, cases: int, seed: int,
                    dispersions: Sequence[Mapping[str, Any]], backend: Mapping[str, Any] | None,
                    store_uri: str | None, auth: AuthorizationContext, progress=None) -> dict[str, Any]:
        mid, doc = self.app.missions.from_ref(mission, auth)
        vars_ = []
        for d in dispersions:
            dist = d.get("distribution", "normal")
            params = d.get("parameters") or {}
            disp: Dispersion
            if dist == "uniform":
                disp = UniformDispersion(float(params["low"]), float(params["high"]))
            else:
                disp = NormalDispersion(float(params.get("mean", 0.0)), float(params.get("stdev", 1.0)))
            vars_.append(MonteCarloVariable(d["name"], d["pointer"], disp))
        mc = MissionMonteCarlo(vars_, cases=cases, seed=seed)
        runner = self._runner(doc, campaign_id, auth.tenant_id, backend, store_uri)
        def _prog(done, total, _case):
            _bump(progress, completed=done, total=total)
        exe = runner.run_cases(mc.cases(), campaign_id=campaign_id, kind="monte_carlo",
                               worker=mission_case_worker, progress=_prog,
                               metadata={"mission_id": mid, "pointers": {d["name"]: d["pointer"] for d in dispersions},
                                         "seed": seed})
        store = runner.store
        if store is None:
            raise DomainError("INTERNAL_ERROR", "campaign store was not created")
        summary = summarize_numeric_metrics(store.cases(campaign_id))
        store.close()
        return {
            "ok": True, "campaign_id": campaign_id, "status": "completed",
            "requested_cases": exe.requested_cases, "executed_cases": exe.executed_cases,
            "resumed_cases": exe.resumed_cases, "failed_cases": exe.failed_cases,
            "summary": jsonable(summary),
            "summary_uri": f"uniflight://campaigns/{campaign_id}/summary",
            "provenance": self.app.provenance(auth, mission_id=mid, mission_sha256=doc.digest_sha256, seed=seed),
        }

    def sobol(self, mission: Mapping[str, Any], campaign_id: str, variables: Sequence[Mapping[str, Any]],
              base_samples: int, seed: int, backend: Mapping[str, Any] | None,
              store_uri: str | None, auth: AuthorizationContext, progress=None) -> dict[str, Any]:
        mid, doc = self.app.missions.from_ref(mission, auth)
        vars_ = [SobolVariable(v["name"], v["pointer"], float(v["lower"]), float(v["upper"])) for v in variables]
        sob = SobolSensitivity(vars_, base_samples=base_samples, seed=seed)
        runner = self._runner(doc, campaign_id, auth.tenant_id, backend, store_uri)
        cases = sob.cases()
        def _prog(done, total, _case):
            _bump(progress, completed=done, total=total)
        exe = runner.run_cases(cases, campaign_id=campaign_id, kind="sobol",
                               worker=mission_case_worker, progress=_prog,
                               metadata={"mission_id": mid, "pointers": {v["name"]: v["pointer"] for v in variables},
                                         "seed": seed})
        store = runner.store
        if store is None:
            raise DomainError("INTERNAL_ERROR", "campaign store was not created")
        try:
            indices = sob.analyze(store.cases(campaign_id), "success").to_json_dict()
        except Exception:
            indices = {"names": [v.name for v in vars_], "first_order": [], "total_order": [],
                       "variance": 0.0, "base_samples": base_samples}
        store.close()
        return {
            "ok": True, "campaign_id": campaign_id, "status": "completed",
            "samples": exe.requested_cases, "indices": jsonable(indices),
            "summary_uri": f"uniflight://campaigns/{campaign_id}/summary",
            "provenance": self.app.provenance(auth, mission_id=mid, mission_sha256=doc.digest_sha256, seed=seed),
        }

    def optimization_batch(self, mission: Mapping[str, Any], campaign_id: str,
                           declaration: Mapping[str, Any], starts: Sequence[Mapping[str, Any]],
                           backend: Mapping[str, Any] | None, store_uri: str | None,
                           auth: AuthorizationContext, progress=None) -> dict[str, Any]:
        mid, doc = self.app.missions.from_ref(mission, auth)
        if declaration:
            raw = doc.mutable_copy()
            raw["optimization"] = {**(raw.get("optimization") or {}), **dict(declaration)}
            child = self.app.missions.persist(raw, tenant=auth.tenant_id, parent=mid)
            mid, doc = self.app.missions.from_ref({"mission_id": child["mission_id"]}, auth)
        batch = OptimizationBatch([
            OptimizationStart(str(s.get("name") or f"start-{i}"),
                              {str(k): float(v) for k, v in (s.get("design") or s.get("initial_values") or {}).items()})
            for i, s in enumerate(starts)
        ])
        runner = self._runner(doc, campaign_id, auth.tenant_id, backend, store_uri)
        def _prog(done, total, _case):
            _bump(progress, completed=done, total=total)
        cases = batch.cases(doc)
        opt = doc.mutable_copy().get("optimization") or {}
        pointers = {str(d["name"]): f"/optimization/design_variables/{i}/initial"
                    for i, d in enumerate(opt.get("design_variables") or [])}
        exe = runner.run_cases(cases, campaign_id=campaign_id, kind="optimization",
                               worker=optimization_case_worker, progress=_prog,  # type: ignore[arg-type]
                               metadata={"mission_id": mid, "pointers": pointers})
        store = runner.store
        if store is None:
            raise DomainError("INTERNAL_ERROR", "campaign store was not created")
        best = None
        successful = 0
        for stored in store.cases(campaign_id):
            if stored.status == "completed":
                successful += 1
                obj = float((stored.metrics or {}).get("objective", math.inf))
                if best is None or obj < float(best.get("objective", math.inf)):
                    best = {"case_id": stored.case_id, **(stored.metrics or {})}
        store.close()
        return {
            "ok": True, "campaign_id": campaign_id, "status": "completed",
            "starts": len(starts), "successful": successful, "failed": exe.failed_cases,
            "best": best or {}, "summary_uri": f"uniflight://campaigns/{campaign_id}/summary",
            "provenance": self.app.provenance(auth, mission_id=mid, mission_sha256=doc.digest_sha256),
        }

    def status(self, campaign_id: str, auth: AuthorizationContext) -> dict[str, Any]:
        path = self._store_path(campaign_id, auth.tenant_id, None)
        if not path.exists():
            raise DomainError("NOT_FOUND", f"campaign {campaign_id} was not found")
        store = SQLiteResultStore(path)
        try:
            meta = store.campaign_metadata(campaign_id)
            summary = store.summary(campaign_id)
            counts = summary.get("case_counts") or {}
            completed = int(counts.get("completed", 0))
            failed = int(counts.get("failed", 0))
            total = int(summary.get("total_cases") or (completed + failed))
            started = meta.get("created_unix")
            started_at = (
                datetime.fromtimestamp(float(started), timezone.utc).isoformat()
                if started is not None else None
            )
            return {
                "ok": True, "campaign_id": campaign_id,
                "status": "completed" if total and failed == 0 else ("failed" if failed else "unknown"),
                "mission_sha256": str(meta.get("mission_sha256") or ""),
                "requested_cases": total, "completed_cases": completed,
                "failed_cases": failed, "started_at": started_at,
                "updated_at": _now(),
                "summary_uri": f"uniflight://campaigns/{campaign_id}/summary",
            }
        finally:
            store.close()

    def cases(self, campaign_id: str, status: str | None, page: PageRequest | None,
              auth: AuthorizationContext) -> dict[str, Any]:
        path = self._store_path(campaign_id, auth.tenant_id, None)
        store = SQLiteResultStore(path)
        try:
            items = []
            rows = list(store.cases(campaign_id))
            for c in rows:
                if status and c.status != status:
                    continue
                items.append({
                    "case_id": c.case_id, "case_index": c.index, "status": c.status,
                    "parameters": c.parameters or {}, "metrics": c.metrics or {},
                    "error": _case_error(c.error, auth.correlation_id),
                })
            items.sort(key=lambda r: (r["case_index"], r["case_id"]))
            snap = f"{campaign_id}:{path.stat().st_mtime_ns}:{len(rows)}"
            window, info = self.app.cursors.paginate(
                items, page, tool="analysis_cases", tenant=auth.tenant_id,
                filters={"campaign_id": campaign_id, "status": status},
                snapshot=snap,
            )
            return {"ok": True, "items": window, "page": info.model_dump()}
        finally:
            store.close()

    def failures(self, campaign_id: str, error_code: str | None, page: PageRequest | None,
                 auth: AuthorizationContext) -> dict[str, Any]:
        payload = self.cases(campaign_id, "failed", page, auth)
        if error_code:
            payload["items"] = [i for i in payload["items"] if error_code in str(i.get("error") or "")]
        return payload

    def replay(self, campaign_id: str, case_id: str, solver_override: Mapping[str, Any] | None,
               save_history: bool, auth: AuthorizationContext) -> dict[str, Any]:
        path = self._store_path(campaign_id, auth.tenant_id, None)
        store = SQLiteResultStore(path)
        try:
            stored = next((c for c in store.cases(campaign_id) if c.case_id == case_id), None)
            if stored is None:
                raise DomainError("NOT_FOUND", f"case {case_id} was not found")
            meta = store.campaign_metadata(campaign_id)
        finally:
            store.close()
        inner = meta.get("metadata") or {}
        mid = inner.get("mission_id") or self.app.current_mission.get(auth.tenant_id)
        if mid is None:
            raise DomainError("NOT_FOUND", "no mission is bound for replay")
        _, doc = self.app.missions.load(mid, auth)
        pointers = dict(inner.get("pointers") or {})
        overrides: dict[str, Any] = {}
        for key, value in (stored.parameters or {}).items():
            if str(key).startswith("/"):
                overrides[str(key)] = value
            elif key in pointers:
                overrides[str(pointers[key])] = value
        child = doc.with_overrides(overrides)
        persisted = self.app.missions.persist(child.mutable_copy(), tenant=auth.tenant_id, parent=mid)
        run = self.app.runs.run({"mission_id": persisted["mission_id"]}, solver_override, save_history, None, auth)
        rec = self.app.runs.load(run["run_id"], auth)
        diffs = {}
        for key, value in (stored.metrics or {}).items():
            if key in (rec.get("outputs") or {}) and isinstance(value, (int, float)):
                diffs[key] = float(rec["outputs"][key]) - float(value)
        return {
            "ok": True, "case_id": case_id, "run_id": run["run_id"],
            "metrics": dict(rec.get("outputs") or {}), "original_metrics": stored.metrics,
            "differences": diffs,
            "provenance": self.app.provenance(auth, campaign_id=campaign_id, mission_id=mid),
        }

    def optimize_validate(self, mission: Mapping[str, Any], declaration: Mapping[str, Any],
                          auth: AuthorizationContext) -> dict[str, Any]:
        _, doc = self.app.missions.from_ref(mission, auth)
        raw = doc.mutable_copy()
        decl = declaration or raw.get("optimization") or {}
        errors = []
        if not decl.get("design_variables"):
            errors.append({"message": "declaration requires design_variables"})
        return {"ok": True, "valid": not errors, "errors": errors,
                "estimated_evaluations": int(decl.get("max_iterations") or 100)}

    def optimize_evaluate(self, mission: Mapping[str, Any], design: Mapping[str, Any],
                          declaration: Mapping[str, Any] | None,
                          auth: AuthorizationContext) -> dict[str, Any]:
        mid, doc = self.app.missions.from_ref(mission, auth)
        raw = doc.mutable_copy()
        dvs = (declaration or {}).get("design_variables") or (raw.get("optimization") or {}).get("design_variables") or []
        overrides = {}
        for i, dv in enumerate(dvs):
            name = str(dv.get("name"))
            pointer = dv.get("pointer") or f"/optimization/design_variables/{i}/initial"
            if name in design:
                overrides[str(pointer)] = design[name]
        child = doc.with_overrides(overrides) if overrides else doc
        persisted = self.app.missions.persist(child.mutable_copy(), tenant=auth.tenant_id, parent=mid)
        run = self.app.runs.run({"mission_id": persisted["mission_id"]}, None, False, None, auth)
        rec = self.app.runs.load(run["run_id"], auth)
        outputs = rec.get("outputs") or {}
        objective_name = _objective_name(declaration) or _objective_name(raw.get("optimization"))
        raw_obj = outputs.get(objective_name) if objective_name else None
        objective = None if raw_obj is None or not np.isfinite(float(raw_obj)) else float(raw_obj)
        return {
            "ok": True, "feasible": rec.get("status") == "completed",
            "objective": objective, "metrics": dict(outputs),
            "constraint_values": {},
            "run_id": run["run_id"],
            "provenance": self.app.provenance(auth, mission_id=mid, mission_sha256=child.digest_sha256),
        }

    def optimize_run(self, mission: Mapping[str, Any], declaration: Mapping[str, Any],
                     settings: Mapping[str, Any] | None, auth: AuthorizationContext) -> dict[str, Any]:
        mid, doc = self.app.missions.from_ref(mission, auth)
        raw = doc.mutable_copy()
        if declaration:
            raw["optimization"] = {**(raw.get("optimization") or {}), **dict(declaration)}
            if settings:
                raw["optimization"].update(dict(settings))
            doc = MissionDocument(raw, doc.source_path, doc.base_directory)
        result = MissionCompiler().optimize(doc)
        report = self.app.artifacts.put_json({
            "design": dict(result.design), "objective": result.objective,
            "metrics": dict(result.metrics), "message": result.message,
        }, tenant=auth.tenant_id)
        replay = self.optimize_evaluate(mission, dict(result.design), declaration or raw.get("optimization"), auth)
        return {
            "ok": True, "success": bool(result.success), "message": result.message,
            "best_design": dict(result.design), "objective": float(result.objective),
            "metrics": dict(result.metrics),
            "constraint_violation": float(result.max_constraint_violation),
            "evaluations": int(result.nfev), "best_run_id": replay.get("run_id"),
            "optimized_mission_id": mid, "report_artifact": report.model_dump(),
            "provenance": self.app.provenance(auth, mission_id=mid, mission_sha256=doc.digest_sha256),
        }


class VerificationService:
    def __init__(self, app: AppServices):
        self.app = app

    def builtin(self, include_external: bool, auth: AuthorizationContext, progress=None) -> dict[str, Any]:
        report = run_builtin_verification()
        _bump(progress, completed=len(report.results), total=len(report.results))
        vid = verification_id()
        payload = report.to_dict()
        art = self.app.artifacts.put_json(payload, tenant=auth.tenant_id)
        root = self.app.config.verification_dir / auth.tenant_id
        _write_json(root / f"{vid}.json", payload)
        return {
            "ok": True, "verification_id": vid,
            "passed": report.passed, "failed": report.failed, "skipped": report.skipped,
            "success": report.success,
            "report_uri": f"uniflight://verification/{vid}",
            "report_artifact": art.model_dump(),
        }

    def compare_csv(self, ref_id: str, act_id: str, time_column: str, channels: list[str],
                    tolerances: Mapping[str, Any], alignment: Mapping[str, Any] | None,
                    auth: AuthorizationContext) -> dict[str, Any]:
        ref_meta, ref_bytes = self.app.artifacts.read_bytes(ref_id, tenant=auth.tenant_id)
        act_meta, act_bytes = self.app.artifacts.read_bytes(act_id, tenant=auth.tenant_id)
        ref_path = self.app.config.temp_dir / f"{ref_id}.csv"
        act_path = self.app.config.temp_dir / f"{act_id}.csv"
        ref_path.write_bytes(ref_bytes)
        act_path.write_bytes(act_bytes)
        ref = ReferenceTimeHistory.from_csv(ref_path, time_column=time_column, channels=channels)
        act = ReferenceTimeHistory.from_csv(act_path, time_column=time_column, channels=channels)
        tol = TolerancePolicy(
            float(tolerances.get("absolute", 0.0)),
            float(tolerances.get("relative", 0.0)),
            float(tolerances.get("scale_floor", 0.0)),
        )
        results = compare_time_histories(ref, act, channels=channels, tolerance=tol)
        passed = all(r.passed or r.skipped for r in results)
        vid = verification_id()
        channel_results = _channel_results(results)
        payload = {"results": channel_results, "alignment": _alignment_object(alignment)}
        art = self.app.artifacts.put_json(payload, tenant=auth.tenant_id)
        return {
            "ok": True, "verification_id": vid, "passed": passed,
            "channel_results": channel_results,
            "reference_sha256": ref_meta.sha256, "actual_sha256": act_meta.sha256,
            "alignment": payload["alignment"], "report_artifact": art.model_dump(),
        }

    def compare_runs(self, run_a: Mapping[str, Any], run_b: Mapping[str, Any], vehicle_id: str,
                     fields: list[str], sample_interval_s: float, tolerances: Mapping[str, Any],
                     auth: AuthorizationContext, progress=None) -> dict[str, Any]:
        a = self.app.runs.from_ref(run_a, auth)
        b = self.app.runs.from_ref(run_b, auth)
        hist_a = a.get("history", {}).get(vehicle_id) or []
        hist_b = b.get("history", {}).get(vehicle_id) or []
        _bump(progress, total=len(fields))
        field_results: dict[str, Any] = {}
        passed = True
        tol = TolerancePolicy(float(tolerances.get("absolute", 0.0)), float(tolerances.get("relative", 0.0)))
        for i, field in enumerate(fields):
            def series(hist):
                ts, vs = [], []
                for row in hist:
                    val = row["state"].get(field)
                    if val is None:
                        continue
                    ts.append(row["time_s"])
                    arr = np.asarray(val, dtype=float)
                    vs.append(float(arr.reshape(-1).item()) if arr.size == 1 else float(np.linalg.norm(arr)))
                return np.asarray(ts), np.asarray(vs)
            ta, va = series(hist_a)
            tb, vb = series(hist_b)
            if ta.size == 0 or tb.size == 0:
                field_results[field] = {"passed": False, "error": None}
                passed = False
                continue
            grid = np.arange(max(ta[0], tb[0]), min(ta[-1], tb[-1]) + 1e-12, sample_interval_s)
            if grid.size < 2:
                grid = np.array([ta[0], ta[-1]])
            ia = np.interp(grid, ta, va)
            ib = np.interp(grid, tb, vb)
            err = float(np.max(np.abs(ia - ib)))
            ok = tol.accepts(err, float(np.max(np.abs(ia))))
            passed = passed and ok
            field_results[field] = {"passed": ok, "error": err}
            _bump(progress, completed=i + 1)
        vid = verification_id()
        art = self.app.artifacts.put_json(field_results, tenant=auth.tenant_id)
        return {
            "ok": True, "verification_id": vid, "passed": passed,
            "field_results": field_results, "report_artifact": art.model_dump(),
            "provenance": self.app.provenance(auth, run_a=a["run_id"], run_b=b["run_id"]),
        }

    def convergence(self, mission: Mapping[str, Any], solver_family: str, refinements: Sequence[Mapping[str, Any]],
                    metrics: list[str], reference_mode: str | None, auth: AuthorizationContext,
                    progress=None) -> dict[str, Any]:
        runs = []
        values: dict[str, list[float]] = {m: [] for m in metrics}
        hs: list[float] = []
        _bump(progress, total=len(refinements))
        for i, refn in enumerate(refinements):
            override = {"kind": solver_family, **refn}
            result = self.app.runs.run(mission, override, False, None, auth)
            rec = self.app.runs.load(result["run_id"], auth)
            runs.append(result["run_id"])
            step = float(refn.get("step_s") or refn.get("rtol") or (i + 1))
            hs.append(step)
            for m in metrics:
                values[m].append(float((rec.get("outputs") or {}).get(m, 0.0)))
            _bump(progress, completed=i + 1)
        orders: dict[str, float | None] = {}
        errors: dict[str, list[float]] = {}
        for m, series in values.items():
            arr = np.asarray(series, dtype=float)
            if arr.size >= 3:
                try:
                    orders[m] = float(observed_order(list(hs), [abs(x - series[-1]) + 1e-30 for x in series]))
                except Exception:
                    orders[m] = None
            errors[m] = [abs(x - series[-1]) for x in series]
        vid = verification_id()
        art = self.app.artifacts.put_json({"orders": orders, "errors": errors, "runs": runs}, tenant=auth.tenant_id)
        return {
            "ok": True, "verification_id": vid, "passed": True,
            "observed_orders": orders, "errors": errors, "runs": runs,
            "report_artifact": art.model_dump(),
        }


class PluginService:
    def __init__(self, app: AppServices):
        self.app = app

    def list(self, category: str | None, page: PageRequest | None,
             auth: AuthorizationContext) -> dict[str, Any]:
        items = []
        for row in installed_plugin_summary():
            rec = dict(row)
            if category and category not in str(rec):
                continue
            items.append(_plugin_record(rec))
        items.sort(key=lambda r: (str(r.get("plugin_id")), str(r.get("version"))))
        window, info = self.app.cursors.paginate(
            items, page, tool="plugin_list", tenant=auth.tenant_id, filters={"category": category},
        )
        return {"ok": True, "items": window, "page": info.model_dump()}

    def inspect(self, plugin_id: str, auth: AuthorizationContext) -> dict[str, Any]:
        rows = [dict(r) for r in installed_plugin_summary()]
        match = next((r for r in rows if (r.get("plugin_id") or r.get("name")) == plugin_id), None)
        if match is None:
            raise DomainError("PLUGIN_MISSING", f"plugin {plugin_id} is not installed")
        caps = []
        for cap in match.get("capabilities") or []:
            if isinstance(cap, Mapping):
                caps.append({
                    "category": str(cap.get("category") or "plugin"),
                    "capability_id": str(cap.get("capability_id") or cap.get("id") or plugin_id),
                    "owner": str(cap.get("owner") or plugin_id),
                })
            else:
                caps.append({"category": "plugin", "capability_id": str(cap), "owner": plugin_id})
        record = _plugin_record(match)
        record["plugin_id"] = plugin_id
        return {
            "ok": True,
            "plugin": record,
            "capabilities": caps,
            "compatible": record["api_version"] == PLUGIN_API_VERSION,
            "warnings": [],
        }
