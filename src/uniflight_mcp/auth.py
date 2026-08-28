from __future__ import annotations

from dataclasses import dataclass

from .config import ALL_SCOPES
from .errors import DomainError


@dataclass(frozen=True, slots=True)
class AuthorizationContext:
    tenant_id: str
    principal: str
    scopes: frozenset[str]
    transport: str
    correlation_id: str

    def require(self, *needed: str) -> None:
        have = set(self.scopes)
        missing = [s for s in needed if s not in have]
        if missing:
            raise DomainError("UNAUTHORIZED", f"missing scope(s): {missing}")


def trusted_local(correlation_id: str) -> AuthorizationContext:
    return AuthorizationContext(
        tenant_id="local",
        principal="stdio",
        scopes=frozenset(ALL_SCOPES),
        transport="stdio",
        correlation_id=correlation_id,
    )
