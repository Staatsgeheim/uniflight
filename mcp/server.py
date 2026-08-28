"""FastMCP entrypoint: `fastmcp list mcp/server.py --json --input-schema --output-schema`."""

from uniflight_mcp.server import create_server

mcp = create_server()
