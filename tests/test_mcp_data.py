from __future__ import annotations

from pathlib import Path

import numpy as np

from uniflight.engineering_data import (
    AxisMetadata, DataProvenance, EngineeringTable, ValidityBound, ValidityEnvelope, ValidityPolicy,
)
from uniflight_mcp.auth import trusted_local
from uniflight_mcp.config import ServerConfig
from uniflight_mcp.services import AppServices


def _app(tmp_path: Path) -> AppServices:
    cfg = ServerConfig(workspace=tmp_path, allowlisted_roots=(tmp_path,), cursor_secret=b"test")
    cfg.ensure_layout()
    return AppServices(cfg)


def _table() -> EngineeringTable:
    return EngineeringTable(
        axes=(AxisMetadata("mach", np.array([0.0, 1.0, 2.0])),),
        outputs={"cd": np.array([0.2, 0.4, 0.6])},
        validity=ValidityEnvelope((ValidityBound("mach", 0.0, 1.5),), ValidityPolicy.FLAG),
        provenance=DataProvenance("aero.cd", "1"),
    )


def test_table_query_and_validity_envelope(tmp_path):
    app = _app(tmp_path)
    auth = trusted_local("cid")
    table = _table()
    app.data.register_table(table)
    catalog = app.data.catalog(None, None, None, auth)
    assert catalog["items"][0]["sha256"] == table.content_sha256()
    inside = app.data.query("aero.cd", "1", {"mach": 0.5}, None, auth)
    assert inside["inside_validity"] is True
    assert inside["sha256"] == table.content_sha256()
    validity = app.data.validity("aero.cd", "1", {"mach": 1.8}, auth)
    assert validity["interpolation_domain"]["inside"] is False or validity["engineering_validity"]["inside"] is False
    assert validity["inside_validity"] is False
