from __future__ import annotations

import asyncio

from uniflight_mcp.contracts import tool_registry, tool_schema
from uniflight_mcp.server import create_server


def test_registered_output_schemas_match_contract(tmp_path, monkeypatch):
    monkeypatch.setenv("UNIFLIGHT_MCP_WORKSPACE", str(tmp_path))
    monkeypatch.setenv("UNIFLIGHT_MCP_HTTP_AUTH", "0")
    server = create_server()
    tools = {t.name: t for t in asyncio.run(server.list_tools())}
    for name in tool_registry():
        expected = tool_schema(name, "output")
        actual = tools[name].output_schema
        assert actual is not None
        assert actual.get("$id") == expected.get("$id") or actual.get("oneOf") == expected.get("oneOf")
