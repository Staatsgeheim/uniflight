# FastMCP 3.x Architecture Decisions

## Version policy
Pin `fastmcp[tasks]>=3.4.7,<4`. FastMCP 4 is outside this contract until a deliberate migration.

## Why FastMCP 3.x features are used

### Providers/transforms
The base server keeps a bounded semantic tool catalog. Providers/transforms are reserved for dynamic composition, plugin-contributed documentation/components, or future on-demand discovery. Do not create one MCP tool per dataset, run, vehicle, or campaign.

### Component pagination
Use `list_page_size=50`. This is the only pagination implementation for MCP component discovery.

### Domain pagination
Tool payload pagination is separate and specified in `PAGINATION.md`. Normal response budget is 256 KiB. Large exports become artifacts.

### Structured outputs
Return Pydantic models. Let FastMCP derive `outputSchema` from return annotations unless a checked-in manual schema is required. The checked-in JSON Schemas are the public contract and CI must compare generated schemas against them.

### Error behavior
Run with `mask_error_details=True`. Expected domain failures use controlled structured error results; unexpected failures expose only a correlation ID to clients.

FastMCP 3.4.3+ supports rich returnable tool errors; implementation should prefer an error result when the agent can recover by changing arguments, and protocol exceptions when the request itself is malformed/unauthorized.

### Background tasks
Use MCP-native background tasks. Tool code must be `async def` when `task=True`. Use the `Progress` dependency. Production task scaling uses Redis/Valkey Docket workers.

### Timeouts
Short tools have decorator timeouts. Task-enabled operations have server/tenant resource ceilings but should not be prematurely killed while making progress unless policy requires it.

### Dependency injection
Services are dependencies, not globals. Request auth/correlation context is injected. This makes handlers testable with fake services.

### Lifespan
Initialize catalogs/stores/plugin manager/telemetry once. No per-call reconstruction of large catalogs.

### Middleware
Order:
1. Error handling
2. Auth/AuthZ
3. Rate limiting/compute quota
4. Correlation/provenance
5. Timing/OpenTelemetry
6. Structured logging

Use retry middleware only for genuinely transient infrastructure calls, never to blindly retry deterministic scientific failures.

### Caching
Cache immutable resource reads and safe read-only metadata queries. Do not cache mutable campaign status without a short explicit TTL. Never let cache identity omit mission/dataset/plugin hashes.

### Response limiting
Do not use response truncation as normal behavior for structured tools. Truncation may violate the output schema. Pagination/artifact references are mandatory for large domain data.

### Session state
Session state may store convenience context such as current mission ID. Scientific identity never depends on ephemeral session state. Persist missions/runs/campaigns.

### Authorization
HTTP deployments use AuthProvider/TokenVerifier and callable authorization checks. STDIO is trusted-local because OAuth tokens are unavailable there.

### OpenTelemetry
Enable MCP semantic conventions and add UniFlight-specific span attributes such as mission SHA, run ID, campaign ID, solver, dataset count, and outcome.

### Dynamic visibility
FastMCP 3 session visibility can hide admin/heavy-analysis components for principals lacking scopes. Authorization remains enforced even when a component is hidden.

## Optional future FastMCP features

- Code Mode may be useful if the catalog grows far beyond v1.
- MCP Apps could provide interactive trajectory/Monte-Carlo dashboards, but should be a separate presentation layer and never replace machine-readable tool results.
