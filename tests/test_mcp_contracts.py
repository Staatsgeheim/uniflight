from __future__ import annotations

import asyncio
from pathlib import Path

import jsonschema
import pytest
from fastmcp import Client

from uniflight_mcp.contracts import tool_policy, tool_registry, tool_schema_resolved
from uniflight_mcp.prompts import SERVER_INSTRUCTIONS
from uniflight_mcp.server import create_server


REQUIRED_TOOLS = list(tool_registry())


@pytest.fixture
def server(tmp_path, monkeypatch):
    monkeypatch.setenv("UNIFLIGHT_MCP_WORKSPACE", str(tmp_path))
    monkeypatch.setenv("UNIFLIGHT_MCP_HTTP_AUTH", "0")
    return create_server()


def test_server_has_builtin_instructions(server):
    assert server.instructions == SERVER_INSTRUCTIONS
    assert "SI internally" in server.instructions
    assert "plugin-install" in server.instructions


def test_all_36_tools_registered(server):
    tools = asyncio.run(server.list_tools())
    names = {t.name for t in tools}
    assert set(REQUIRED_TOOLS) <= names
    assert len(REQUIRED_TOOLS) == 36


def test_tool_policy_annotations_and_timeouts(server):
    tools = {t.name: t for t in asyncio.run(server.list_tools())}
    policy = tool_policy()
    for name, pol in policy.items():
        tool = tools[name]
        anns = tool.annotations
        assert anns.readOnlyHint is pol["annotations"]["readOnlyHint"]
        assert anns.destructiveHint is pol["annotations"]["destructiveHint"]
        assert anns.openWorldHint is pol["annotations"]["openWorldHint"]
        assert tool.version == str(pol["component_version"])
        assert float(tool.timeout) == float(pol["timeout_s"])
        meta = getattr(tool, "meta", None) or {}
        if pol.get("task_mode") == "optional":
            assert getattr(tool, "task", None) in {True, None} or meta.get("task") is True


def test_system_version_matches_schema(server, tmp_path):
    async def _call():
        async with Client(server) as client:
            return await client.call_tool("system_version", {})
    result = asyncio.run(_call())
    payload = result.data if hasattr(result, "data") else result.structured_content
    if payload is None and hasattr(result, "content"):
        import json
        payload = json.loads(result.content[0].text)
    jsonschema.validate(payload, tool_schema_resolved("system_version", "output"))
    assert payload["ok"] is True


def test_no_plugin_install_tool(server):
    names = {t.name for t in asyncio.run(server.list_tools())}
    assert "plugin_install" not in names
    assert "shell" not in names
    assert "python_exec" not in names


def test_http_app_smoke(server):
    app = server.http_app()
    assert app is not None


def test_resolved_output_schema_validates_system_version():
    from uniflight_mcp.contracts import tool_schema_resolved
    from uniflight_mcp.runtime import system_version_payload

    jsonschema.validate(system_version_payload(), tool_schema_resolved("system_version", "output"))


def _payload(result):
    data = result.data if hasattr(result, "data") else result.structured_content
    if data is None and hasattr(result, "content"):
        import json
        data = json.loads(result.content[0].text)
    return data


def test_live_tool_payloads_match_output_schemas(server):
    mission = Path("missions/nereid_l_minimal.toml").read_text(encoding="utf-8")

    async def _call():
        async with Client(server) as client:
            capabilities = await client.call_tool("system_capabilities", {})
            compiled = await client.call_tool("mission_compile", {"document": mission, "format": "toml"})
            mid = _payload(compiled)["mission_id"]
            inspected = await client.call_tool("mission_inspect", {"mission": {"mission_id": mid}})
            overridden = await client.call_tool(
                "mission_apply_overrides",
                {"mission": {"mission_id": mid}, "overrides": [{"pointer": "/outputs/0/name", "value": "alt-x"}]},
            )
            ran = await client.call_tool("simulation_run", {"mission": {"mission_id": mid}, "save_history": True})
            rid = _payload(ran)["run_id"]
            summary = await client.call_tool("simulation_summary", {"run": {"run_id": rid}})
            events = await client.call_tool("simulation_events", {"run": {"run_id": rid}})
            history = await client.call_tool(
                "simulation_vehicle_history",
                {"run": {"run_id": rid}, "vehicle_id": "probe", "page": {"limit": 2}},
            )
            state = await client.call_tool(
                "simulation_state_at",
                {"run": {"run_id": rid}, "vehicle_id": "probe", "time_s": 1.0},
            )
            plugins = await client.call_tool("plugin_list", {})
            catalog = await client.call_tool("data_catalog_list", {})
            return {
                "system_capabilities": _payload(capabilities),
                "mission_compile": _payload(compiled),
                "mission_inspect": _payload(inspected),
                "mission_apply_overrides": _payload(overridden),
                "simulation_run": _payload(ran),
                "simulation_summary": _payload(summary),
                "simulation_events": _payload(events),
                "simulation_vehicle_history": _payload(history),
                "simulation_state_at": _payload(state),
                "plugin_list": _payload(plugins),
                "data_catalog_list": _payload(catalog),
            }

    payloads = asyncio.run(_call())
    for name, payload in payloads.items():
        assert payload["ok"] is True, payload
        jsonschema.validate(payload, tool_schema_resolved(name, "output"))
    assert len(payloads["system_capabilities"]["items"]) == 36
