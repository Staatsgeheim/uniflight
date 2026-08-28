from __future__ import annotations

from datetime import datetime, timezone
from typing import Any, Mapping

from pydantic import BaseModel, ConfigDict, Field


class PageRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    limit: int = Field(default=100, ge=1, le=1000)
    cursor: str | None = None


class PageInfo(BaseModel):
    model_config = ConfigDict(extra="forbid")
    returned: int
    has_more: bool
    next_cursor: str | None
    snapshot_id: str
    total_estimate: int | None = None


class ArtifactRef(BaseModel):
    model_config = ConfigDict(extra="forbid")
    artifact_id: str
    uri: str
    media_type: str
    size_bytes: int
    sha256: str
    expires_at: str | None = None


class MissionRef(BaseModel):
    model_config = ConfigDict(extra="forbid")
    mission_id: str | None = None
    uri: str | None = None


class RunRef(BaseModel):
    model_config = ConfigDict(extra="forbid")
    run_id: str | None = None
    uri: str | None = None


class CampaignRef(BaseModel):
    model_config = ConfigDict(extra="forbid")
    campaign_id: str


class SolverOverride(BaseModel):
    model_config = ConfigDict(extra="forbid")
    profile: str | None = None
    kind: str | None = None
    method: str | None = None
    rtol: float | None = None
    atol: float | None = None
    max_step_s: float | None = None
    step_s: float | None = None


class Override(BaseModel):
    model_config = ConfigDict(extra="forbid")
    pointer: str
    value: Any


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def page_dict(info: PageInfo) -> dict[str, Any]:
    return info.model_dump()


def artifact_dict(ref: ArtifactRef) -> dict[str, Any]:
    return ref.model_dump()


def jsonable(value: Any) -> Any:
    if isinstance(value, Mapping):
        return {str(k): jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [jsonable(v) for v in value]
    if hasattr(value, "tolist"):
        return value.tolist()
    if hasattr(value, "item"):
        try:
            return value.item()
        except Exception:
            return str(value)
    return value
