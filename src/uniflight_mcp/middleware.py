from __future__ import annotations

import logging
import time
import uuid
from typing import Any

from fastmcp.server.middleware import Middleware, MiddlewareContext

from .errors import DomainError

logger = logging.getLogger("uniflight_mcp")


def _correlation_id(context: MiddlewareContext[Any]) -> str:
    extra = getattr(context, "fastmcp_context", None)
    if extra is not None:
        stored = getattr(extra, "_uniflight_correlation_id", None)
        if stored:
            return str(stored)
        cid = str(uuid.uuid4())
        extra._uniflight_correlation_id = cid
        return cid
    return str(uuid.uuid4())


class CorrelationMiddleware(Middleware):
    async def on_call_tool(self, context: MiddlewareContext[Any], call_next):
        _correlation_id(context)
        return await call_next(context)


class TimingMiddleware(Middleware):
    async def on_call_tool(self, context: MiddlewareContext[Any], call_next):
        t0 = time.perf_counter()
        try:
            result = await call_next(context)
            elapsed = time.perf_counter() - t0
            logger.info("tool_ok method=%s elapsed=%.4f", context.method, elapsed)
            return result
        except Exception:
            elapsed = time.perf_counter() - t0
            logger.info("tool_err method=%s elapsed=%.4f", context.method, elapsed)
            raise


class SafeErrorMiddleware(Middleware):
    async def on_call_tool(self, context: MiddlewareContext[Any], call_next):
        try:
            return await call_next(context)
        except DomainError:
            raise
        except Exception:
            cid = _correlation_id(context)
            logger.exception("masked internal error correlation_id=%s", cid)
            raise DomainError("INTERNAL_ERROR", f"internal error; correlation_id={cid}")
