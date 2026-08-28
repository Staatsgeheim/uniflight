from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from fastmcp import FastMCP

from uniflight import mission_json_schema

from .contracts import contract_root
from .runtime import get_auth, get_services, system_version_payload


REMINDERS = (
    "SI internally. Quaternion maps body → inertial. "
    "Body axes are +x forward, +y right, +z down. "
    "External benchmark agreement is not flight validation."
)


def _md(name: str) -> str:
    root = contract_root()
    for candidate in (
        root / name,
        Path("skills/references") / name,
        Path("src/uniflight_mcp") / name,
    ):
        if candidate.exists():
            return candidate.read_text(encoding="utf-8")
    mapping = {
        "architecture": "UNIFLIGHT_MCP_SPEC.md",
        "plugin-api": "SECURITY.md",
        "hpc-api": "IMPLEMENTATION_PLAN.md",
        "verification": "TASKS.md",
    }
    path = root / mapping.get(name, "README.md")
    return path.read_text(encoding="utf-8") if path.exists() else REMINDERS


def register_resources(mcp: FastMCP) -> None:
    @mcp.resource("uniflight://system/version", mime_type="application/json")
    async def system_version() -> str:
        return json.dumps(system_version_payload())

    @mcp.resource("uniflight://system/capabilities", mime_type="application/json")
    async def system_capabilities() -> str:
        return json.dumps({"ok": True, "note": "use system_capabilities for paginated inventory"})

    @mcp.resource("uniflight://schema/mission/1.0", mime_type="application/schema+json")
    async def mission_schema() -> str:
        return json.dumps(mission_json_schema())

    @mcp.resource("uniflight://docs/architecture", mime_type="text/markdown")
    async def docs_architecture() -> str:
        return _md("architecture")

    @mcp.resource("uniflight://docs/plugin-api", mime_type="text/markdown")
    async def docs_plugin() -> str:
        return _md("plugin-api")

    @mcp.resource("uniflight://docs/hpc-api", mime_type="text/markdown")
    async def docs_hpc() -> str:
        return _md("hpc-api")

    @mcp.resource("uniflight://docs/verification", mime_type="text/markdown")
    async def docs_verification() -> str:
        return _md("verification")

    @mcp.resource("uniflight://missions/{mission_id}")
    async def mission_resource(mission_id: str) -> str:
        auth = get_auth()
        meta, doc = get_services().missions.load(mission_id, auth)
        return json.dumps({"meta": meta, "document": doc.mutable_copy()})

    @mcp.resource("uniflight://missions/{mission_id}/normalized")
    async def mission_normalized(mission_id: str) -> str:
        return await mission_resource(mission_id)

    @mcp.resource("uniflight://runs/{run_id}/manifest")
    async def run_manifest(run_id: str) -> str:
        rec = get_services().runs.load(run_id, get_auth())
        return json.dumps({k: rec[k] for k in rec if k != "history"})

    @mcp.resource("uniflight://runs/{run_id}/summary")
    async def run_summary(run_id: str) -> str:
        return json.dumps(get_services().runs.summary({"run_id": run_id}, get_auth()))

    @mcp.resource("uniflight://runs/{run_id}/events")
    async def run_events(run_id: str) -> str:
        rec = get_services().runs.load(run_id, get_auth())
        return json.dumps(rec.get("events") or [])

    @mcp.resource("uniflight://runs/{run_id}/exports/{artifact_id}")
    async def run_export(run_id: str, artifact_id: str) -> bytes:
        _ = run_id
        _ref, data = get_services().artifacts.read_bytes(artifact_id, tenant=get_auth().tenant_id)
        return data

    @mcp.resource("uniflight://campaigns/{campaign_id}/summary")
    async def campaign_summary(campaign_id: str) -> str:
        return json.dumps(get_services().analysis.status(campaign_id, get_auth()))

    @mcp.resource("uniflight://verification/{verification_id}")
    async def verification_resource(verification_id: str) -> str:
        path = get_services().config.verification_dir / get_auth().tenant_id / f"{verification_id}.json"
        return path.read_text(encoding="utf-8") if path.exists() else json.dumps({"ok": False})

    @mcp.resource("uniflight://datasets/{dataset_id}/{version}")
    async def dataset_resource(dataset_id: str, version: str) -> str:
        return json.dumps({"dataset_id": dataset_id, "version": version})

    @mcp.resource("uniflight://artifacts/{artifact_id}")
    async def artifact_resource(artifact_id: str) -> bytes:
        _ref, data = get_services().artifacts.read_bytes(artifact_id, tenant=get_auth().tenant_id)
        return data
