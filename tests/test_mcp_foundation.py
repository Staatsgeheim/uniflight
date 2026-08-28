from __future__ import annotations

from pathlib import Path

import pytest

from uniflight_mcp.auth import trusted_local
from uniflight_mcp.config import ServerConfig
from uniflight_mcp.cursors import CursorCodec
from uniflight_mcp.errors import DomainError
from uniflight_mcp.models import PageRequest
from uniflight_mcp.paths import resolve_under
from uniflight_mcp.services import AppServices


def _app(tmp_path: Path) -> AppServices:
    cfg = ServerConfig(workspace=tmp_path, allowlisted_roots=(tmp_path,), cursor_secret=b"test")
    cfg.ensure_layout()
    return AppServices(cfg)


def test_cursor_roundtrip_and_filter_mismatch(tmp_path):
    codec = CursorCodec(b"secret", ttl_s=60)
    items = list(range(250))
    page1, info1 = codec.paginate(
        items, PageRequest(limit=10), tool="simulation_events", tenant="local", filters={"v": 1},
    )
    assert info1.has_more and info1.next_cursor and len(page1) == 10
    page2, info2 = codec.paginate(
        items, PageRequest(limit=10, cursor=info1.next_cursor),
        tool="simulation_events", tenant="local", filters={"v": 1},
    )
    assert page1[0] == 0 and page2[0] == 10
    assert set(page1).isdisjoint(page2)
    with pytest.raises(DomainError) as exc:
        codec.paginate(
            items, PageRequest(limit=10, cursor=info1.next_cursor),
            tool="simulation_events", tenant="local", filters={"v": 2},
        )
    assert exc.value.code == "INVALID_CURSOR"
    with pytest.raises(DomainError) as exc:
        codec.paginate(
            items, PageRequest(limit=10, cursor=info1.next_cursor),
            tool="simulation_events", tenant="other", filters={"v": 1},
        )
    assert exc.value.code == "INVALID_CURSOR"


def test_expired_cursor():
    codec = CursorCodec(b"secret", ttl_s=-1)
    items = [1, 2, 3]
    _, info = codec.paginate(items, PageRequest(limit=1), tool="t", tenant="local", filters={})
    with pytest.raises(DomainError) as exc:
        codec.paginate(items, PageRequest(limit=1, cursor=info.next_cursor), tool="t", tenant="local", filters={})
    assert exc.value.code == "CURSOR_EXPIRED"


def test_snapshot_bound_cursor_rejected():
    codec = CursorCodec(b"secret", ttl_s=60)
    items = list(range(20))
    _, info = codec.paginate(
        items, PageRequest(limit=5), tool="analysis_cases", tenant="local",
        filters={"campaign_id": "c1"}, snapshot="snap-a",
    )
    with pytest.raises(DomainError) as exc:
        codec.paginate(
            items, PageRequest(limit=5, cursor=info.next_cursor), tool="analysis_cases",
            tenant="local", filters={"campaign_id": "c1"}, snapshot="snap-b",
        )
    assert exc.value.code == "INVALID_CURSOR"


def test_path_traversal_rejected(tmp_path):
    with pytest.raises(DomainError):
        resolve_under(tmp_path / ".." / "etc" / "passwd", (tmp_path,))


def test_artifact_sha(tmp_path):
    app = _app(tmp_path)
    ref = app.artifacts.put_bytes(b"abc", media_type="text/plain", suffix=".txt", tenant="local")
    assert ref.artifact_id.startswith("art_")
    got, data = app.artifacts.read_bytes(ref.artifact_id, tenant="local")
    assert data == b"abc" and got.sha256 == ref.sha256


def test_progress_bump_uses_fastmcp_increment():
    from uniflight_mcp.services import _bump

    class _Progress:
        def __init__(self):
            self._current = None
            self.totals = []
            self.increments = []

        @property
        def current(self):
            return self._current

        def set_total(self, total):
            self.totals.append(total)

        def increment(self, amount=1):
            self.increments.append(amount)
            self._current = amount if self._current is None else self._current + amount

    progress = _Progress()
    _bump(progress, total=4, completed=1)
    _bump(progress, completed=3)
    _bump(progress, completed=3)
    assert progress.totals == [4]
    assert progress.increments == [1, 2]


def test_scope_enforcement():
    auth = trusted_local("cid")
    auth.require("uniflight:read")
    limited = type(auth)("t", "p", frozenset({"uniflight:read"}), "http", "cid")
    with pytest.raises(DomainError) as exc:
        limited.require("uniflight:compute")
    assert exc.value.code == "UNAUTHORIZED"
