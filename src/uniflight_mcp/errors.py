from __future__ import annotations

from typing import Any, Mapping


DOMAIN_CODES = frozenset({
    "INVALID_REQUEST", "INVALID_CURSOR", "CURSOR_EXPIRED", "NOT_FOUND",
    "MISSION_VALIDATION_ERROR", "MISSION_COMPILATION_ERROR",
    "MISSION_IDENTITY_MISMATCH", "UNKNOWN_MODEL", "DATASET_NOT_FOUND",
    "DATASET_VERSION_MISMATCH", "DATASET_CHECKSUM_MISMATCH", "PLUGIN_MISSING",
    "PLUGIN_VERSION_MISMATCH", "PLUGIN_API_MISMATCH", "INVALID_STATE",
    "INVALID_FRAME", "INVALID_UNITS", "VALIDITY_ENVELOPE_VIOLATION",
    "SOLVER_FAILURE", "EVENT_CYCLE_DETECTED", "OPTIMIZATION_FAILED",
    "CAMPAIGN_IDENTITY_MISMATCH", "REFERENCE_DATA_INVALID", "TASK_REQUIRED",
    "QUOTA_EXCEEDED", "INTERNAL_ERROR", "UNAUTHORIZED",
})

RECOVERABLE = frozenset({
    "INVALID_REQUEST", "INVALID_CURSOR", "CURSOR_EXPIRED", "NOT_FOUND",
    "MISSION_VALIDATION_ERROR", "MISSION_IDENTITY_MISMATCH",
    "DATASET_NOT_FOUND", "DATASET_VERSION_MISMATCH", "DATASET_CHECKSUM_MISMATCH",
    "PLUGIN_MISSING", "PLUGIN_VERSION_MISMATCH", "PLUGIN_API_MISMATCH",
    "INVALID_STATE", "INVALID_FRAME", "INVALID_UNITS", "VALIDITY_ENVELOPE_VIOLATION",
    "REFERENCE_DATA_INVALID", "TASK_REQUIRED",
})


class DomainError(Exception):
    def __init__(self, code: str, message: str, *, path: str | None = None,
                 details: Mapping[str, Any] | None = None):
        super().__init__(message)
        self.code = code if code in DOMAIN_CODES else "INTERNAL_ERROR"
        self.message = message
        self.path = path
        self.details = dict(details or {})

    @property
    def recoverable(self) -> bool:
        return self.code in RECOVERABLE

    def envelope(self, correlation_id: str) -> dict[str, Any]:
        return {
            "ok": False,
            "error": {
                "code": self.code,
                "message": self.message,
                "recoverable": self.recoverable,
                "path": self.path,
                "details": self.details,
                "correlation_id": correlation_id,
            },
        }
