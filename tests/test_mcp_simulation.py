from __future__ import annotations

from pathlib import Path

from uniflight_mcp.auth import trusted_local
from uniflight_mcp.config import ServerConfig
from uniflight_mcp.models import PageRequest
from uniflight_mcp.services import AppServices


MINIMAL = Path("missions/nereid_l_minimal.toml")


def _app(tmp_path: Path) -> AppServices:
    cfg = ServerConfig(
        workspace=tmp_path,
        allowlisted_roots=(tmp_path, Path("missions").resolve(), Path(".").resolve()),
        cursor_secret=b"test",
    )
    cfg.ensure_layout()
    return AppServices(cfg)


def test_minimal_mission_run_is_deterministic(tmp_path):
    app = _app(tmp_path)
    auth = trusted_local("cid")
    compiled = app.missions.compile(MINIMAL.read_text(encoding="utf-8"), "toml", True, auth)
    first = app.runs.run({"mission_id": compiled["mission_id"]}, None, True, None, auth)
    second = app.runs.run({"mission_id": compiled["mission_id"]}, None, True, None, auth)
    rec_a = app.runs.load(first["run_id"], auth)
    rec_b = app.runs.load(second["run_id"], auth)
    assert first["status"] == "completed"
    assert rec_a["mission_sha256"] == compiled["digest_sha256"] == rec_b["mission_sha256"]
    assert rec_a["outputs"] == rec_b["outputs"]
    summary = app.runs.summary({"run_id": first["run_id"]}, auth)
    assert summary["ok"] is True
    history = app.runs.history({"run_id": first["run_id"]}, "probe", {}, PageRequest(limit=1), auth)
    assert history["items"]
    full = app.runs.history({"run_id": first["run_id"]}, "probe", {}, PageRequest(limit=100), auth)
    assert full["page"]["total_estimate"] > 2
    if history["page"]["has_more"]:
        page2 = app.runs.history(
            {"run_id": first["run_id"]}, "probe", {},
            PageRequest(limit=1, cursor=history["page"]["next_cursor"]), auth,
        )
        ids1 = [r["time_s"] for r in history["items"]]
        ids2 = [r["time_s"] for r in page2["items"]]
        assert not set(ids1) & set(ids2)
    events = app.runs.events({"run_id": first["run_id"]}, {}, PageRequest(limit=10), auth)
    assert events["ok"] is True
    state = app.runs.state_at({"run_id": first["run_id"]}, "probe", 1.0, None, None, auth)
    assert "position" in state["state"]
    export = app.runs.export_csv({"run_id": first["run_id"]}, "probe", ["mass"], None, None, None, auth)
    assert export["artifact"]["sha256"]
    flow = app.runs.flow_state({"run_id": first["run_id"]}, "probe", 1.0, auth)
    assert flow["dynamic_pressure_pa"] == 0.0
    forces = app.runs.forces({"run_id": first["run_id"]}, "probe", 1.0, auth)
    assert forces["contributions"][0]["source"] == "gravity"
    assert first["run_id"].startswith("run_")


def test_cancelled_status_is_persistable(tmp_path):
    app = _app(tmp_path)
    auth = trusted_local("cid")
    compiled = app.missions.compile(MINIMAL.read_text(encoding="utf-8"), "toml", True, auth)
    result = app.runs.run({"mission_id": compiled["mission_id"]}, None, False, None, auth)
    rec = app.runs.load(result["run_id"], auth)
    rec["status"] = "cancelled"
    app.runs._persist(rec, auth)
    loaded = app.runs.load(result["run_id"], auth)
    assert loaded["status"] == "cancelled"
    assert (app.config.runs_dir / auth.tenant_id / result["run_id"] / "run.json").exists()
