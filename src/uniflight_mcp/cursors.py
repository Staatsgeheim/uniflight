from __future__ import annotations

import base64
import hashlib
import hmac
import json
import time
from typing import Any, Mapping, Sequence

from .errors import DomainError
from .ids import snapshot_id
from .models import PageInfo, PageRequest


def _b64(data: bytes) -> str:
    return base64.urlsafe_b64encode(data).rstrip(b"=").decode("ascii")


def _unb64(text: str) -> bytes:
    pad = "=" * (-len(text) % 4)
    return base64.urlsafe_b64decode(text + pad)


class CursorCodec:
    def __init__(self, secret: bytes, ttl_s: int = 3600):
        self.secret = secret
        self.ttl_s = int(ttl_s)

    def _sign(self, payload: bytes) -> str:
        return hmac.new(self.secret, payload, hashlib.sha256).hexdigest()

    def encode(self, *, tool: str, tenant: str, filters: Mapping[str, Any],
               snapshot: str, offset: int, sort: Sequence[str] | None = None) -> str:
        body = {
            "tool": tool,
            "ver": "1",
            "tenant": tenant,
            "filters": dict(filters),
            "sort": list(sort or []),
            "snapshot": snapshot,
            "offset": int(offset),
            "exp": int(time.time()) + self.ttl_s,
        }
        raw = json.dumps(body, sort_keys=True, separators=(",", ":")).encode()
        return f"{_b64(raw)}.{self._sign(raw)}"

    def decode(self, cursor: str, *, tool: str, tenant: str,
               filters: Mapping[str, Any]) -> dict[str, Any]:
        try:
            blob, sig = cursor.split(".", 1)
            raw = _unb64(blob)
        except Exception as exc:
            raise DomainError("INVALID_CURSOR", "cursor is malformed") from exc
        if not hmac.compare_digest(self._sign(raw), sig):
            raise DomainError("INVALID_CURSOR", "cursor authentication failed")
        body = json.loads(raw.decode())
        if body.get("exp", 0) < time.time():
            raise DomainError("CURSOR_EXPIRED", "cursor snapshot has expired")
        if body.get("tool") != tool or body.get("ver") != "1":
            raise DomainError("INVALID_CURSOR", "cursor does not match this tool")
        if body.get("tenant") != tenant:
            raise DomainError("INVALID_CURSOR", "cursor tenant mismatch")
        if body.get("filters") != dict(filters):
            raise DomainError("INVALID_CURSOR", "cursor filters do not match the request")
        return body

    def paginate(self, items: Sequence[Any], page: PageRequest | None, *,
                 tool: str, tenant: str, filters: Mapping[str, Any],
                 snapshot: str | None = None) -> tuple[list[Any], PageInfo]:
        req = page or PageRequest()
        snap = snapshot or snapshot_id()
        offset = 0
        if req.cursor:
            body = self.decode(req.cursor, tool=tool, tenant=tenant, filters=filters)
            if snapshot is not None and str(body.get("snapshot")) != str(snapshot):
                raise DomainError("INVALID_CURSOR", "cursor snapshot does not match the current page")
            snap = str(body["snapshot"])
            offset = int(body["offset"])
        limit = req.limit
        window = list(items[offset:offset + limit])
        next_off = offset + len(window)
        has_more = next_off < len(items)
        nxt = None
        if has_more:
            nxt = self.encode(tool=tool, tenant=tenant, filters=filters,
                              snapshot=snap, offset=next_off)
        return window, PageInfo(
            returned=len(window),
            has_more=has_more,
            next_cursor=nxt,
            snapshot_id=snap,
            total_estimate=len(items),
        )
