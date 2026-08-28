# Implementation Plan and Acceptance Matrix

No implementation is included in this package. This is the recommended build order after contract approval.

## Phase 1 — foundation

- package scaffold: `uniflight-mcp`
- FastMCP 3.4.7+ server constructor
- lifespan services
- Pydantic common models
- artifact store
- mission/run IDs
- cursor codec
- safe error mapping
- telemetry/correlation middleware
- STDIO transport
- Streamable HTTP transport

Acceptance:
- `fastmcp list server.py --json --input-schema --output-schema` works;
- schemas match checked-in contract;
- component list pagination returns `nextCursor` with >50 test components;
- errors are masked.

## Phase 2 — mission + deterministic simulation

Implement:
- system_version / capabilities
- mission_validate / inspect / compile / apply_overrides
- simulation_run / summary / events / state / history / export
- environment_sample / flow / forces

Acceptance:
- known UniFlight examples reproduce;
- mission SHA is stable;
- event tie/priority semantics preserved;
- paginated history/events produce no gaps/duplicates;
- cancellation leaves consistent run artifacts.

## Phase 3 — engineering data + optimization

Implement data catalog/query/validity and optimization validate/evaluate/run.

Acceptance:
- exact dataset version/checksum enforced;
- validity envelope separated from interpolation domain;
- optimization replay reproduces reported optimum.

## Phase 4 — persistent analysis/HPC

Implement sweeps, Monte Carlo, Sobol, multistart, status/cases/failures/replay.

Acceptance:
- SQLite checkpoint/restart;
- deterministic seeds invariant to worker count;
- page cursors bind to campaign snapshot;
- Redis-backed FastMCP task worker integration;
- no direct worker SQLite writes.

## Phase 5 — verification + plugins

Implement verification tools and plugin read-only inspection.

Acceptance:
- built-in verifier preserves pass/fail/skip counts;
- external comparison records reference SHA/alignment/tolerances;
- NESC-style timestamp-grid tests prevent artificial jitter;
- plugin version/API/capability inventory is correct;
- no plugin installation endpoint exists.

## CI matrix

Python: 3.11, 3.12, 3.13.

Required CI jobs:
1. unit tests;
2. JSON Schema validation;
3. generated FastMCP schema diff against `schemas/tools/`;
4. pagination property tests;
5. authorization tests;
6. task/cancellation tests;
7. Redis integration tests;
8. path traversal/security tests;
9. Ruff + mypy;
10. wheel build/install;
11. STDIO smoke;
12. Streamable HTTP smoke;
13. FastMCP CLI discovery smoke;
14. UniFlight regression/verification integration tests.

## Contract test invariants

For every tool:
- exact component name/version;
- exact annotations;
- exact input/output schema;
- expected scopes;
- timeout;
- task mode;
- no undocumented fields when `additionalProperties:false`.

For every paginated tool:
- default page;
- min/max limit;
- first/middle/final page;
- stable ordering;
- no duplicate/missing IDs;
- invalid cursor;
- expired cursor;
- cursor with changed filters;
- cross-tenant cursor rejection;
- snapshot consistency.

For every compute result:
- server + UniFlight version;
- mission SHA when relevant;
- solver;
- datasets/plugins;
- seed when stochastic;
- artifact SHA for exports.
