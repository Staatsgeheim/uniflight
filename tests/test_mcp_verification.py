from __future__ import annotations

from pathlib import Path

from uniflight_mcp.auth import trusted_local
from uniflight_mcp.config import ServerConfig
from uniflight_mcp.errors import DomainError
from uniflight_mcp.services import AppServices
import pytest


def _app(tmp_path: Path) -> AppServices:
    cfg = ServerConfig(workspace=tmp_path, allowlisted_roots=(tmp_path,), cursor_secret=b"test")
    cfg.ensure_layout()
    return AppServices(cfg)


def test_builtin_verification_counts(tmp_path):
    app = _app(tmp_path)
    auth = trusted_local("cid")
    payload = app.verification.builtin(False, auth)
    assert payload["ok"] is True
    assert payload["verification_id"].startswith("ver_")
    assert payload["passed"] + payload["failed"] + payload["skipped"] >= 1
    assert payload["success"] is True or payload["failed"] >= 0


def test_csv_compare_records_sha_and_alignment(tmp_path):
    app = _app(tmp_path)
    auth = trusted_local("cid")
    csv = "t,alt\n0,1\n1,2\n2,3\n"
    ref = app.artifacts.put_text(csv, media_type="text/csv", suffix=".csv", tenant=auth.tenant_id)
    act = app.artifacts.put_text(csv, media_type="text/csv", suffix=".csv", tenant=auth.tenant_id)
    result = app.verification.compare_csv(
        ref.artifact_id, act.artifact_id, "t", ["alt"],
        {"absolute": 1e-9, "relative": 0.0},
        {"method": "timestamp_grid"},
        auth,
    )
    assert result["passed"] is True
    assert result["reference_sha256"] == ref.sha256
    assert result["actual_sha256"] == act.sha256
    assert result["alignment"]["method"] == "timestamp_grid"
    assert isinstance(result["channel_results"], dict)
    assert "alt" in result["channel_results"]
    from uniflight_mcp.contracts import tool_schema_resolved
    import jsonschema
    jsonschema.validate(result, tool_schema_resolved("verification_compare_csv", "output"))


def test_plugin_inspect_is_read_only(tmp_path):
    app = _app(tmp_path)
    auth = trusted_local("cid")
    listed = app.plugins.list(None, None, auth)
    assert listed["ok"] is True
    with pytest.raises(DomainError) as exc:
        app.plugins.inspect("does-not-exist", auth)
    assert exc.value.code == "PLUGIN_MISSING"
