from __future__ import annotations

from pathlib import Path

import pytest

from uniflight_mcp.auth import trusted_local
from uniflight_mcp.config import ServerConfig
from uniflight_mcp.errors import DomainError
from uniflight_mcp.paths import reject_traversal, resolve_under
from uniflight_mcp.services import AppServices


def test_reject_dotdot():
    with pytest.raises(DomainError):
        reject_traversal("../secret")


def test_store_uri_cannot_escape(tmp_path):
    cfg = ServerConfig(workspace=tmp_path, allowlisted_roots=(tmp_path,), cursor_secret=b"x")
    cfg.ensure_layout()
    app = AppServices(cfg)
    auth = trusted_local("cid")
    with pytest.raises(DomainError):
        app.analysis._store_path("c1", auth.tenant_id, str(tmp_path / ".." / "outside.sqlite"))


def test_errors_are_masked_envelopes():
    err = DomainError("INTERNAL_ERROR", "boom")
    env = err.envelope("cid-1")
    assert env["ok"] is False
    assert env["error"]["correlation_id"] == "cid-1"
    assert env["error"]["recoverable"] is False


def test_static_token_verifier_configured(tmp_path, monkeypatch):
    import json
    from uniflight_mcp.server import create_server

    monkeypatch.setenv("UNIFLIGHT_MCP_WORKSPACE", str(tmp_path))
    monkeypatch.setenv("UNIFLIGHT_MCP_HTTP_AUTH", "1")
    monkeypatch.setenv("UNIFLIGHT_MCP_TOKENS", json.dumps({
        "secret-token": {"client_id": "ops", "scopes": ["uniflight:read"], "tenant": "ops"},
    }))
    server = create_server()
    assert server.auth is not None
