from __future__ import annotations

import json
import os
from dataclasses import dataclass, field
from pathlib import Path


ALL_SCOPES = (
    "uniflight:read",
    "uniflight:compute",
    "uniflight:write-artifacts",
    "uniflight:analysis",
    "uniflight:admin",
)


@dataclass(frozen=True, slots=True)
class ServerConfig:
    workspace: Path
    allowlisted_roots: tuple[Path, ...]
    cursor_secret: bytes
    cursor_ttl_s: int = 3600
    list_page_size: int = 50
    docket_url: str | None = None
    require_http_auth: bool = True
    tokens: dict[str, dict] = field(default_factory=dict)
    max_concurrent_tasks: int = 8
    max_campaign_cases: int = 100_000
    max_artifact_bytes: int = 2 * 1024 * 1024 * 1024

    @property
    def missions_dir(self) -> Path:
        return self.workspace / "missions"

    @property
    def runs_dir(self) -> Path:
        return self.workspace / "runs"

    @property
    def campaigns_dir(self) -> Path:
        return self.workspace / "campaigns"

    @property
    def verification_dir(self) -> Path:
        return self.workspace / "verification"

    @property
    def exports_dir(self) -> Path:
        return self.workspace / "exports"

    @property
    def temp_dir(self) -> Path:
        return self.workspace / "temp"

    def ensure_layout(self) -> None:
        for path in (
            self.workspace, self.missions_dir, self.runs_dir, self.campaigns_dir,
            self.verification_dir, self.exports_dir, self.temp_dir,
        ):
            path.mkdir(parents=True, exist_ok=True)
            if not os.access(path, os.W_OK):
                raise PermissionError(f"workspace path is not writable: {path}")

    @classmethod
    def from_env(cls, workspace: Path | None = None) -> "ServerConfig":
        root = Path(workspace or os.environ.get("UNIFLIGHT_MCP_WORKSPACE", "./workspace")).resolve()
        extra_roots = [
            Path(p).resolve()
            for p in os.environ.get("UNIFLIGHT_MCP_ALLOW_ROOTS", "").split(os.pathsep)
            if p.strip()
        ]
        secret = os.environ.get("UNIFLIGHT_MCP_CURSOR_SECRET", "uniflight-mcp-dev-secret").encode()
        tokens_raw = os.environ.get("UNIFLIGHT_MCP_TOKENS", "")
        tokens = json.loads(tokens_raw) if tokens_raw else {}
        docket = os.environ.get("UNIFLIGHT_MCP_DOCKET_URL") or None
        http_auth = os.environ.get("UNIFLIGHT_MCP_HTTP_AUTH", "1") not in {"0", "false", "no"}
        return cls(
            workspace=root,
            allowlisted_roots=(root, *extra_roots),
            cursor_secret=secret,
            docket_url=docket,
            require_http_auth=http_auth,
            tokens=tokens,
        )
