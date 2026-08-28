from __future__ import annotations

import argparse
import os
from pathlib import Path

from fastmcp import FastMCP
from fastmcp.server.auth.providers.jwt import StaticTokenVerifier
from fastmcp.server.lifespan import lifespan

from ._version import __version__
from .config import ServerConfig
from .middleware import CorrelationMiddleware, SafeErrorMiddleware, TimingMiddleware
from .prompts import SERVER_INSTRUCTIONS, register_prompts
from .resources import register_resources
from .runtime import reset_services, set_services
from .services import AppServices
from .tools import register_tools


def _auth_provider(config: ServerConfig):
    if not config.tokens:
        return None
    tokens = {}
    for raw, meta in config.tokens.items():
        tokens[raw] = {
            "client_id": meta.get("client_id", "uniflight"),
            "scopes": meta.get("scopes", []),
            "claims": {"tenant": meta.get("tenant", "http")},
        }
    return StaticTokenVerifier(tokens)


@lifespan
async def _services_lifespan(server: FastMCP):
    config = getattr(server, "_uniflight_config", None) or ServerConfig.from_env()
    config.ensure_layout()
    existing = getattr(server, "_uniflight_services", None)
    services = existing or AppServices(config)
    token = set_services(services)
    try:
        yield {"services": services, "config": config}
    finally:
        if existing is None:
            services.close()
        reset_services(token)


def create_server(config: ServerConfig | None = None, *, extra_tools: int = 0) -> FastMCP:
    cfg = config or ServerConfig.from_env()
    cfg.ensure_layout()
    auth = _auth_provider(cfg) if cfg.require_http_auth else None
    mcp = FastMCP(
        "UniFlight",
        instructions=SERVER_INSTRUCTIONS,
        version=__version__,
        lifespan=_services_lifespan,
        middleware=[
            SafeErrorMiddleware(),
            CorrelationMiddleware(),
            TimingMiddleware(),
        ],
        auth=auth,
        on_duplicate="error",
        mask_error_details=True,
        strict_input_validation=False,
        list_page_size=cfg.list_page_size,
        tasks=True,
    )
    if cfg.docket_url:
        os.environ.setdefault("FASTMCP_DOCKET_URL", cfg.docket_url)
    services = AppServices(cfg)
    mcp._uniflight_config = cfg  # type: ignore[attr-defined]
    mcp._uniflight_services = services  # type: ignore[attr-defined]
    set_services(services)
    register_tools(mcp)
    register_resources(mcp)
    register_prompts(mcp)
    for i in range(extra_tools):
        @mcp.tool(name=f"_test_component_{i}", version="1")
        async def _dummy(i: int = i) -> dict:
            return {"ok": True, "i": i}
    return mcp


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="uniflight-mcp", description="UniFlight FastMCP 3 server")
    parser.add_argument("--transport", choices=("stdio", "http"), default="stdio")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--workspace", default=None)
    args = parser.parse_args(argv)
    config = ServerConfig.from_env(Path(args.workspace) if args.workspace else None)
    if args.workspace:
        os.environ["UNIFLIGHT_MCP_WORKSPACE"] = str(Path(args.workspace).resolve())
    if config.docket_url:
        os.environ.setdefault("FASTMCP_DOCKET_URL", config.docket_url)
    mcp = create_server(config)
    if args.transport == "stdio":
        mcp.run(transport="stdio")
    else:
        mcp.run(transport="http", host=args.host, port=args.port)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
