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


def test_sweep_checkpoint_restart_and_replay(tmp_path):
    app = _app(tmp_path)
    auth = trusted_local("cid")
    compiled = app.missions.compile(MINIMAL.read_text(encoding="utf-8"), "toml", True, auth)
    variables = [{"name": "mass", "pointer": "/vehicles/probe/initial/state/mass", "values": [10.0, 11.0]}]
    first = app.analysis.sweep({"mission_id": compiled["mission_id"]}, "sweep-1", variables, None, None, auth)
    assert first["executed_cases"] == 2
    second = app.analysis.sweep({"mission_id": compiled["mission_id"]}, "sweep-1", variables, None, None, auth)
    assert second["resumed_cases"] == 2
    assert second["executed_cases"] == 0
    status = app.analysis.status("sweep-1", auth)
    assert status["completed_cases"] == 2
    cases = app.analysis.cases("sweep-1", None, PageRequest(limit=1), auth)
    assert cases["page"]["has_more"] is True
    case_id = cases["items"][0]["case_id"]
    replayed = app.analysis.replay("sweep-1", case_id, None, False, auth)
    assert replayed["run_id"].startswith("run_")
    assert "altitude" in replayed["metrics"]


def test_monte_carlo_seed_invariance(tmp_path):
    app = _app(tmp_path)
    auth = trusted_local("cid")
    compiled = app.missions.compile(MINIMAL.read_text(encoding="utf-8"), "toml", True, auth)
    dispersions = [{
        "name": "mass", "pointer": "/vehicles/probe/initial/state/mass",
        "distribution": "uniform", "parameters": {"low": 9.0, "high": 11.0},
    }]
    a = app.analysis.monte_carlo({"mission_id": compiled["mission_id"]}, "mc-a", 3, 7, dispersions, None, None, auth)
    b = app.analysis.monte_carlo({"mission_id": compiled["mission_id"]}, "mc-b", 3, 7, dispersions, None, None, auth)
    cases_a = app.analysis.cases("mc-a", None, None, auth)["items"]
    cases_b = app.analysis.cases("mc-b", None, None, auth)["items"]
    assert [c["parameters"]["mass"] for c in cases_a] == [c["parameters"]["mass"] for c in cases_b]
    assert a["requested_cases"] == b["requested_cases"] == 3


def test_optimize_evaluate_reads_objective_metric(tmp_path):
    app = _app(tmp_path)
    auth = trusted_local("cid")
    compiled = app.missions.compile(MINIMAL.read_text(encoding="utf-8"), "toml", True, auth)
    result = app.analysis.optimize_evaluate(
        {"mission_id": compiled["mission_id"]},
        {"mass": 10.0},
        {
            "design_variables": [{
                "name": "mass",
                "pointer": "/vehicles/probe/initial/state/mass",
                "lower": 9.0, "upper": 11.0, "initial": 10.0,
            }],
            "objective": {"metric": "altitude", "sense": "maximize"},
        },
        auth,
    )
    assert result["ok"] is True
    assert result["objective"] == result["metrics"]["altitude"]
    assert result["objective"] is not None
