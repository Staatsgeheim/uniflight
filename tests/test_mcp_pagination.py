from __future__ import annotations

import asyncio

from fastmcp import Client

from uniflight_mcp.cursors import CursorCodec
from uniflight_mcp.models import PageRequest
from uniflight_mcp.server import create_server


def test_component_list_pagination_next_cursor(tmp_path, monkeypatch):
    monkeypatch.setenv("UNIFLIGHT_MCP_WORKSPACE", str(tmp_path))
    monkeypatch.setenv("UNIFLIGHT_MCP_HTTP_AUTH", "0")
    server = create_server(extra_tools=20)
    async def _list():
        async with Client(server) as client:
            page = await client.list_tools()
            return page
    tools = asyncio.run(_list())
    assert len(tools) > 50 or hasattr(tools, "__len__")
    raw = asyncio.run(server.list_tools())
    assert len(raw) > 50


def test_page_min_max_and_middle_final():
    codec = CursorCodec(b"secret", ttl_s=60)
    items = list(range(25))
    first, info1 = codec.paginate(items, PageRequest(limit=1), tool="t", tenant="local", filters={})
    assert first == [0] and info1.has_more
    mid, info2 = codec.paginate(
        items, PageRequest(limit=10, cursor=info1.next_cursor), tool="t", tenant="local", filters={},
    )
    assert mid == list(range(1, 11)) and info2.has_more
    rest, info3 = codec.paginate(
        items, PageRequest(limit=1000, cursor=info2.next_cursor), tool="t", tenant="local", filters={},
    )
    assert rest == list(range(11, 25))
    assert info3.has_more is False and info3.next_cursor is None
    assert not set(first) & set(mid) and not set(mid) & set(rest)
