# UniFlight MCP contracts

JSON Schema, tool policy, and design notes for the FastMCP 3.x server. The running implementation is `src/uniflight_mcp/` and the `uniflight-mcp` console script (`pip install "uniflight[mcp]"`).

## Primary documents

1. `UNIFLIGHT_MCP_SPEC.md` — architecture and public contract.
2. `TOOL_CATALOG.md` — 36-tool catalog.
3. `TOOL_POLICY.json` — component version, scopes, annotations, task mode, timeout.
4. `RESOURCE_CATALOG.md` — resource/resource-template URIs.
5. `PROMPTS.md` — prompt catalog.
6. `PAGINATION.md` — MCP discovery pagination + domain cursor pagination.
7. `TASKS.md` — native MCP background-task policy.
8. `SECURITY.md` — trust boundaries, scopes, quotas and prohibited operations.
9. `architecture/FASTMCP_3_DECISIONS.md` — FastMCP-specific decisions.
10. `IMPLEMENTATION_PLAN.md` — phased build and CI acceptance matrix.

## Machine-readable contracts

- `TOOL_REGISTRY.json`
- `schemas/common/common.schema.json`
- `schemas/tools/*.input.json`
- `schemas/tools/*.output.json`

There are 36 v1 tools and 72 exact per-tool schemas.

## FastMCP baseline

Pin `fastmcp[tasks]>=3.4.7,<4`. Discovery:

```bash
fastmcp list mcp/server.py --json --input-schema --output-schema
```
