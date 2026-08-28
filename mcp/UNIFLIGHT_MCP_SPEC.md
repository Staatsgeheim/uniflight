# UniFlight MCP Server Specification

**Spec version:** 0.1.0-draft  
**Target:** UniFlight 1.0.1+  
**FastMCP:** `fastmcp[tasks]>=3.4.7,<4`  
**Server name:** `uniflight`  
**Status:** design only; no server implementation included.

## 1. Purpose

UniFlight MCP is an agent-facing scientific operations API for constructing, validating, compiling, running, inspecting, optimizing, analyzing, and verifying UniFlight missions while preserving units, frame conventions, solver settings, mission identity, engineering-data identity, plugin identity, seeds, and numerical provenance.

The MCP layer MUST call UniFlight public APIs and MUST NOT reimplement the flight dynamics, hybrid event engine, optimization algorithms, engineering-data interpolation, Monte Carlo logic, or verification mathematics.

## 2. Architectural principles

1. **Typed contracts.** Every tool uses Pydantic input/output models and FastMCP structured outputs.
2. **Immutable identity.** Mission/dataset/plugin identities are pinned; transformations create new artifacts instead of mutating persisted inputs.
3. **SI-first.** Numeric physics fields use SI unless the schema explicitly names another unit.
4. **Large-data discipline.** Large trajectories/campaign rows are paginated or exported as resources/artifacts, never dumped into one tool response.
5. **Native MCP tasks.** Long operations use FastMCP background tasks (SEP-1686), not a custom generic job protocol.
6. **Scientific provenance.** Every compute result records UniFlight version, mission SHA-256, solver, datasets, plugins, and seed where relevant.
7. **No inflated validation claims.** External benchmark agreement is not flight validation.
8. **No arbitrary execution.** v1 has no shell, arbitrary Python, plugin install, arbitrary filesystem, or arbitrary URL-fetch tools.

## 3. FastMCP 3.x baseline

Recommended constructor:

```python
mcp = FastMCP(
    "UniFlight",
    version="0.1.0",
    list_page_size=50,
    mask_error_details=True,
    strict_input_validation=False,
    on_duplicate="error",
)
```

`list_page_size=50` enables MCP pagination for `tools/list`, `resources/list`, `resources/templates/list`, and `prompts/list`.

Flexible validation is retained for LLM-client compatibility, but domain Pydantic models MUST use constrained/strict fields where coercion would be scientifically unsafe.

All public components use component version `"1"`.

### 3.1 Tool annotations

Read-only tools:
- `readOnlyHint=True`
- `idempotentHint=True`
- `openWorldHint=False`

Artifact-producing compute:
- `readOnlyHint=False`
- `destructiveHint=False`
- `openWorldHint=False`

No v1 tool is intentionally destructive.

### 3.2 Dependency injection

Use FastMCP `Depends()` / `CurrentContext()` for:
- `ServerConfig`
- `MissionService`
- `RunService`
- `DataService`
- `AnalysisService`
- `VerificationService`
- `ArtifactStore`
- `CursorCodec`
- `AuthorizationContext`

Avoid mutable module globals.

### 3.3 Lifespan

FastMCP lifespan performs one-time startup/shutdown:
- verify compatible UniFlight version;
- initialize artifact workspace;
- initialize engineering-data catalog;
- initialize plugin manager;
- initialize result-store factory;
- initialize telemetry;
- validate writable roots;
- close database/executor resources on shutdown.

### 3.4 Middleware order

Recommended:
1. safe error transformation;
2. authentication/authorization;
3. rate limiting and compute quotas;
4. correlation/provenance context;
5. timing/OpenTelemetry;
6. structured logging.

Response-limiting middleware is only a safety net. Paginated structured tools MUST NOT rely on truncation because truncation can invalidate output schemas.

## 4. Server composition

One parent FastMCP server, optionally mounting domain servers:

```text
uniflight
  system
  mission
  simulation
  physics
  data
  optimization
  analysis
  verification
  plugin
```

Public tool names remain flat and stable.

## 5. Persistence

Default logical workspace:

```text
workspace/
  missions/
  runs/
  campaigns/
  verification/
  exports/
  temp/
```

Identifiers:
- mission `mis_<ulid>`
- run `run_<ulid>`
- verification `ver_<ulid>`
- artifact `art_<ulid>`

Campaign identity remains UniFlight's stable campaign ID plus tenant namespace.

## 6. Resource URIs

Static:
- `uniflight://system/version`
- `uniflight://system/capabilities`
- `uniflight://schema/mission/1.0`
- `uniflight://docs/architecture`
- `uniflight://docs/plugin-api`
- `uniflight://docs/hpc-api`
- `uniflight://docs/verification`

Dynamic:
- `uniflight://missions/{mission_id}`
- `uniflight://missions/{mission_id}/normalized`
- `uniflight://runs/{run_id}/manifest`
- `uniflight://runs/{run_id}/summary`
- `uniflight://runs/{run_id}/events`
- `uniflight://runs/{run_id}/exports/{artifact_id}`
- `uniflight://campaigns/{campaign_id}/summary`
- `uniflight://verification/{verification_id}`
- `uniflight://datasets/{dataset_id}/{version}`
- `uniflight://artifacts/{artifact_id}`

Resources are read-only and idempotent. Huge tabular data are queried through paginated tools.

## 7. Pagination

There are two distinct mechanisms.

### 7.1 MCP component-list pagination

FastMCP owns component-list cursors. Clients treat them as opaque.

### 7.2 Domain-result pagination

Arbitrary tool results are not covered by MCP component pagination. Large UniFlight collections use:

```json
{"limit":100,"cursor":null}
```

with `limit` in `1..1000`, default `100`.

Response:

```json
{
  "items": [],
  "page": {
    "returned": 100,
    "has_more": true,
    "next_cursor": "opaque",
    "snapshot_id": "snap_...",
    "total_estimate": null
  }
}
```

Cursors MUST bind to:
- tool name;
- tenant/user scope;
- normalized filters;
- sort;
- snapshot;
- expiry.

Cursors are authenticated if client-side encoded. Mutable collections use keyset/snapshot pagination, not naive offsets.

Required paginated tools:
- `system_capabilities`
- `simulation_events`
- `simulation_vehicle_history`
- `analysis_cases`
- `analysis_failures`
- `data_catalog_list`
- `plugin_list`

Default ordering:
- events `(time_s ASC, priority DESC, sequence ASC)`
- trajectory `(time_s ASC, segment_index ASC)`
- cases `(case_index ASC, case_id ASC)`
- datasets `(dataset_id ASC, version ASC)`
- plugins `(plugin_id ASC, version ASC)`

## 8. Native background tasks

Task-capable:
- `simulation_run`
- `simulation_compare_solvers`
- `optimization_run`
- `analysis_sweep`
- `analysis_monte_carlo`
- `analysis_sobol`
- `analysis_optimization_batch`
- `verification_builtin`
- `verification_compare_runs`
- `verification_convergence`

Use FastMCP native tasks. Development may use Docket `memory://`; horizontally scaled production SHOULD use Redis/Valkey-backed Docket workers.

Progress examples:
- simulation: physical time / mission duration;
- Monte Carlo: completed cases / total;
- Sobol: completed samples / total;
- optimization: evaluations / budget;
- verification: checks completed / total.

MCP task cancellation stops new work promptly. Completed campaign cases remain checkpointed and reusable.

## 9. Security

STDIO is trusted-local; OAuth is not available there. Streamable HTTP is production default and SHOULD require authentication.

Suggested scopes:
- `uniflight:read`
- `uniflight:compute`
- `uniflight:write-artifacts`
- `uniflight:analysis`
- `uniflight:admin`

All client paths are logical resource/artifact IDs or constrained to allowlisted roots. Protect against traversal/symlink escape.

Plugins are trusted in-process Python. MCP exposes plugin inspection only in v1.

Unexpected exceptions are masked. Expected domain failures use controlled typed error results.

## 10. Observability

Enable OpenTelemetry in production. Spans/logs SHOULD record:
- correlation/request ID;
- tool name/version;
- tenant/principal;
- mission ID/SHA;
- run/campaign/task ID;
- UniFlight version;
- solver family;
- elapsed time;
- outcome/error code.

Do not log full proprietary mission/table contents by default.

## 11. Standard provenance

Compute results carry:
- server version;
- UniFlight version;
- creation time;
- mission ID/name/SHA;
- solver kind/method/tolerances/step;
- dataset `(id,version,sha256)` inventory;
- plugin `(id,version,api_version)` inventory;
- seed.

## 12. Error model

`ErrorEnvelope` fields:
- `code`
- `message`
- `recoverable`
- `path`
- `details`
- `correlation_id`

Canonical domain codes:
`INVALID_REQUEST`, `INVALID_CURSOR`, `CURSOR_EXPIRED`, `NOT_FOUND`,
`MISSION_VALIDATION_ERROR`, `MISSION_COMPILATION_ERROR`,
`MISSION_IDENTITY_MISMATCH`, `UNKNOWN_MODEL`, `DATASET_NOT_FOUND`,
`DATASET_VERSION_MISMATCH`, `DATASET_CHECKSUM_MISMATCH`, `PLUGIN_MISSING`,
`PLUGIN_VERSION_MISMATCH`, `PLUGIN_API_MISMATCH`, `INVALID_STATE`,
`INVALID_FRAME`, `INVALID_UNITS`, `VALIDITY_ENVELOPE_VIOLATION`,
`SOLVER_FAILURE`, `EVENT_CYCLE_DETECTED`, `OPTIMIZATION_FAILED`,
`CAMPAIGN_IDENTITY_MISMATCH`, `REFERENCE_DATA_INVALID`, `TASK_REQUIRED`,
`QUOTA_EXCEEDED`, `INTERNAL_ERROR`.

Malformed MCP/JSON requests remain protocol errors.

## 13. Public v1 tools

### System
1. `system_version`
2. `system_capabilities` — paginated

### Mission
3. `mission_validate`
4. `mission_inspect`
5. `mission_compile`
6. `mission_apply_overrides`

### Simulation / physics
7. `simulation_run`
8. `simulation_summary`
9. `simulation_events` — paginated
10. `simulation_state_at`
11. `simulation_vehicle_history` — paginated
12. `simulation_export_csv`
13. `simulation_compare_solvers`
14. `environment_sample`
15. `vehicle_flow_state`
16. `vehicle_forces`

### Optimization
17. `optimization_validate`
18. `optimization_evaluate`
19. `optimization_run`

### Analysis
20. `analysis_sweep`
21. `analysis_monte_carlo`
22. `analysis_sobol`
23. `analysis_optimization_batch`
24. `analysis_status`
25. `analysis_cases` — paginated
26. `analysis_failures` — paginated
27. `analysis_case_replay`

### Engineering data
28. `data_catalog_list` — paginated
29. `data_table_query`
30. `data_validity_check`

### Verification
31. `verification_builtin`
32. `verification_compare_csv`
33. `verification_compare_runs`
34. `verification_convergence`

### Plugins
35. `plugin_list` — paginated
36. `plugin_inspect`

Exact JSON Schemas are in `schemas/tools/`.

## 14. MCP prompts

Versioned prompts:
- `create_uniflight_mission`
- `debug_uniflight_simulation`
- `design_entry_trajectory`
- `design_landing_simulation`
- `optimize_trajectory`
- `analyze_monte_carlo_failures`
- `verify_against_external_reference`
- `create_uniflight_plugin`
- `review_uniflight_mission`

Every prompt reinforces SI units, body→inertial quaternions, body axes, explicit solver tolerances, pinned datasets/plugins, and evidence-based verification.

## 15. Artifact export

Large/binary exports return:

```json
{
  "artifact_id":"art_...",
  "uri":"uniflight://artifacts/art_...",
  "media_type":"text/csv",
  "size_bytes":123456,
  "sha256":"...",
  "expires_at":null
}
```

Canonical identity is the MCP resource URI, not a temporary HTTP URL.

## 16. Scientific execution policy

- DOP853 is the normal trusted adaptive reference when the mission selects adaptive propagation.
- Fixed RK4 is used only when explicitly selected and step size is recorded.
- GNC wrappers preserve sampled-data chronology; no estimator/controller mutation inside adaptive RHS.
- Event summaries preserve tie/priority ordering.
- External benchmark reports record reference SHA, channel mapping, timestamp alignment, interpolation, and tolerances.
- Agreement with NESC is `external_benchmark`, never `flight_validation`.

## 17. v1 non-goals

No arbitrary Python, shell, arbitrary filesystem, plugin install, dependency install, arbitrary URL download, destructive delete, unpinned model substitution, or direct ad-hoc mutable spawning outside mission/event semantics.

## 18. Acceptance criteria

Before implementation is releaseable:
- STDIO and Streamable HTTP both work;
- FastMCP component pagination is tested;
- every domain-paginated tool tests first/middle/final/invalid/expired cursors;
- all tools expose input/output schemas, versions, annotations, and timeouts/task modes;
- simulation and Monte Carlo work as native background tasks with progress;
- Redis task-worker mode is integration-tested;
- HTTP AuthN/AuthZ scopes are tested;
- internal exceptions are masked;
- path traversal is impossible;
- provenance is present on all compute outputs;
- large outputs use pages/artifacts;
- `fastmcp list ... --json --input-schema --output-schema` matches the checked-in contract.
