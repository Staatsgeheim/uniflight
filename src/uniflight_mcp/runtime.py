from __future__ import annotations

import contextvars
import platform
import sys
import uuid
from typing import Any

import fastmcp
import uniflight
from fastmcp.dependencies import Depends
from uniflight.plugins import PLUGIN_API_VERSION

from ._version import __version__
from .auth import AuthorizationContext, trusted_local
from .errors import DomainError
from .models import PageRequest
from .services import AppServices

_SERVICES: contextvars.ContextVar[AppServices] = contextvars.ContextVar("uniflight_mcp_services")


def set_services(services: AppServices) -> contextvars.Token:
    return _SERVICES.set(services)


def reset_services(token: contextvars.Token) -> None:
    _SERVICES.reset(token)


def get_services() -> AppServices:
    try:
        return _SERVICES.get()
    except LookupError as exc:
        raise RuntimeError("UniFlight MCP services are not initialized") from exc


def get_auth() -> AuthorizationContext:
    cid = str(uuid.uuid4())
    token = None
    try:
        from fastmcp.server.dependencies import get_access_token
        token = get_access_token()
    except Exception:
        token = None
    if token is None:
        return trusted_local(cid)
    scopes = frozenset(getattr(token, "scopes", ()) or ())
    claims = getattr(token, "claims", {}) or {}
    return AuthorizationContext(
        tenant_id=str(claims.get("tenant") or getattr(token, "client_id", None) or "http"),
        principal=str(getattr(token, "client_id", None) or "http"),
        scopes=scopes or frozenset(),
        transport="http",
        correlation_id=cid,
    )


def fail(exc: Exception, auth: AuthorizationContext) -> dict[str, Any]:
    if isinstance(exc, DomainError):
        return exc.envelope(auth.correlation_id)
    return DomainError("INTERNAL_ERROR", f"internal error; correlation_id={auth.correlation_id}").envelope(
        auth.correlation_id
    )


def page_req(page: dict[str, Any] | None) -> PageRequest | None:
    if not page:
        return None
    return PageRequest.model_validate(page)


def system_version_payload() -> dict[str, Any]:
    return {
        "ok": True,
        "server_version": __version__,
        "uniflight_version": uniflight.__version__,
        "fastmcp_version": fastmcp.__version__,
        "python_version": sys.version.split()[0],
        "platform": platform.platform(),
        "plugin_api_version": PLUGIN_API_VERSION,
    }


ServicesDep = Depends(get_services)
AuthDep = Depends(get_auth)
