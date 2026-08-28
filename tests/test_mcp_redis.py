from __future__ import annotations

import inspect

from uniflight.analysis import mission_case_worker
from uniflight_mcp.config import ServerConfig
from uniflight_mcp.server import create_server


def test_docket_url_from_env(tmp_path, monkeypatch):
    monkeypatch.setenv("UNIFLIGHT_MCP_WORKSPACE", str(tmp_path))
    monkeypatch.setenv("UNIFLIGHT_MCP_DOCKET_URL", "redis://localhost:6379/0")
    monkeypatch.setenv("UNIFLIGHT_MCP_HTTP_AUTH", "0")
    cfg = ServerConfig.from_env(tmp_path)
    assert cfg.docket_url == "redis://localhost:6379/0"
    server = create_server(cfg)
    assert server._uniflight_config.docket_url.startswith("redis://")


def test_workers_do_not_open_sqlite():
    source = inspect.getsource(mission_case_worker)
    assert "SQLite" not in source
    assert "sqlite3" not in source
