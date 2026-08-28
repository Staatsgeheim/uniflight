from __future__ import annotations

from pathlib import Path

from uniflight_mcp.auth import trusted_local
from uniflight_mcp.config import ServerConfig
from uniflight_mcp.services import AppServices, parse_document


MINIMAL = Path("missions/nereid_l_minimal.toml")


def _app(tmp_path: Path) -> AppServices:
    cfg = ServerConfig(
        workspace=tmp_path,
        allowlisted_roots=(tmp_path, Path("missions").resolve(), Path(".").resolve()),
        cursor_secret=b"test",
    )
    cfg.ensure_layout()
    return AppServices(cfg)


def test_validate_and_compile_minimal_mission(tmp_path):
    app = _app(tmp_path)
    auth = trusted_local("cid")
    text = MINIMAL.read_text(encoding="utf-8")
    validated = app.missions.validate(text, "toml", None, auth)
    assert validated["valid"] is True
    compiled = app.missions.compile(text, "toml", True, auth)
    assert compiled["ok"] is True
    assert compiled["mission_id"].startswith("mis_")
    assert compiled["digest_sha256"] == validated["digest_sha256"]
    inspected = app.missions.inspect({"mission_id": compiled["mission_id"]}, auth)
    assert inspected["ok"] is True
    assert isinstance(inspected["bodies"], list)
    assert inspected["bodies"][0]["id"] == "nereid"
    again = app.missions.validate(text, "toml", None, auth)
    assert again["digest_sha256"] == validated["digest_sha256"]


def test_overrides_create_new_identity(tmp_path):
    app = _app(tmp_path)
    auth = trusted_local("cid")
    compiled = app.missions.compile(MINIMAL.read_text(encoding="utf-8"), "toml", True, auth)
    child = app.missions.apply_overrides(
        {"mission_id": compiled["mission_id"]},
        [{"pointer": "/outputs/0/name", "value": "altitude-overridden"}],
        auth,
    )
    assert child["parent_mission_id"] == compiled["mission_id"]
    assert child["digest_sha256"] != compiled["digest_sha256"]
    assert child["mission_id"] != compiled["mission_id"]
    assert child["applied_overrides"] == [{"pointer": "/outputs/0/name", "value": "altitude-overridden"}]


def test_parse_document_json():
    raw = parse_document('{"format_version":"1.0"}', "json")
    assert raw["format_version"] == "1.0"


def test_parse_document_yaml_scientific_notation():
    raw = parse_document("format_version: \"1.0\"\nmu: 1.5e11\n", "yaml")
    assert raw["mu"] == 1.5e11
    assert isinstance(raw["mu"], float)
