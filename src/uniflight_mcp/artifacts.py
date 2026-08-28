from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Any

from .config import ServerConfig
from .errors import DomainError
from .ids import artifact_id
from .models import ArtifactRef
from .paths import resolve_under


class ArtifactStore:
    def __init__(self, config: ServerConfig):
        self.config = config
        self._index = config.workspace / "artifacts.json"
        self._meta: dict[str, dict[str, Any]] = {}
        if self._index.exists():
            self._meta = json.loads(self._index.read_text(encoding="utf-8"))

    def _flush(self) -> None:
        self._index.write_text(json.dumps(self._meta, indent=2, sort_keys=True), encoding="utf-8")

    def put_bytes(self, data: bytes, *, media_type: str, suffix: str = "",
                  tenant: str = "local") -> ArtifactRef:
        aid = artifact_id()
        dest = self.config.exports_dir / tenant / f"{aid}{suffix}"
        dest.parent.mkdir(parents=True, exist_ok=True)
        dest.write_bytes(data)
        digest = hashlib.sha256(data).hexdigest()
        ref = ArtifactRef(
            artifact_id=aid,
            uri=f"uniflight://artifacts/{aid}",
            media_type=media_type,
            size_bytes=len(data),
            sha256=digest,
            expires_at=None,
        )
        self._meta[aid] = {"path": str(dest), "tenant": tenant, **ref.model_dump()}
        self._flush()
        return ref

    def put_json(self, payload: Any, *, tenant: str = "local") -> ArtifactRef:
        data = json.dumps(payload, indent=2, sort_keys=True, allow_nan=False).encode()
        return self.put_bytes(data, media_type="application/json", suffix=".json", tenant=tenant)

    def put_text(self, text: str, *, media_type: str, suffix: str, tenant: str = "local") -> ArtifactRef:
        return self.put_bytes(text.encode("utf-8"), media_type=media_type, suffix=suffix, tenant=tenant)

    def get(self, aid: str, *, tenant: str | None = None) -> tuple[ArtifactRef, Path]:
        meta = self._meta.get(aid)
        if meta is None:
            raise DomainError("NOT_FOUND", f"artifact {aid} was not found")
        if tenant is not None and meta.get("tenant") not in {tenant, "local"}:
            raise DomainError("NOT_FOUND", f"artifact {aid} was not found")
        path = resolve_under(meta["path"], self.config.allowlisted_roots)
        ref = ArtifactRef(
            artifact_id=meta["artifact_id"],
            uri=meta["uri"],
            media_type=meta["media_type"],
            size_bytes=int(meta["size_bytes"]),
            sha256=meta["sha256"],
            expires_at=meta.get("expires_at"),
        )
        return ref, path

    def read_bytes(self, aid: str, *, tenant: str | None = None) -> tuple[ArtifactRef, bytes]:
        ref, path = self.get(aid, tenant=tenant)
        data = path.read_bytes()
        if hashlib.sha256(data).hexdigest() != ref.sha256:
            raise DomainError("REFERENCE_DATA_INVALID", "artifact checksum mismatch")
        return ref, data
