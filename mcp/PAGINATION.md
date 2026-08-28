# Pagination and Large-Result Contract

## Two layers

FastMCP component-list pagination and UniFlight domain-result pagination are separate.

### Component list
`FastMCP(..., list_page_size=50)` paginates `tools/list`, `resources/list`, `resources/templates/list`, and `prompts/list`. Clients treat protocol cursors as opaque.

### Domain results
Tools that can return large collections accept:

```json
{"limit":100,"cursor":null}
```

`limit` is 1–1000, default 100.

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

## Cursor requirements

Cursors bind to:
- tool name/version;
- principal/tenant;
- normalized filters;
- normalized sort;
- immutable snapshot/high-water mark;
- expiry.

Authenticated client-side cursor payload or server-side random cursor storage are both acceptable. Never expose raw SQL offsets or filesystem paths.

For mutable campaign rows use keyset pagination. For immutable finite sequences, FastMCP pagination utilities may be used internally.

## Snapshot consistency

A first request establishes `snapshot_id`. All pages from that cursor traverse the same logical snapshot. If it expires, return `CURSOR_EXPIRED`; do not silently jump to current state.

## Stable ordering

- `system_capabilities`: category, capability_id
- `simulation_events`: time_s ASC, priority DESC, sequence ASC
- `simulation_vehicle_history`: time_s ASC, segment_index ASC
- `analysis_cases`: case_index ASC, case_id ASC
- `analysis_failures`: case_index ASC, case_id ASC
- `data_catalog_list`: dataset_id ASC, version ASC
- `plugin_list`: plugin_id ASC, version ASC

## Filtering

Filters are cursor-bound. A request that supplies both an old cursor and different filters returns `INVALID_CURSOR`.

## Total counts

`total_estimate` may be null. Never execute an expensive COUNT only to populate UI metadata unless configured.

## Large binary/tabular data

If a result would exceed normal page budgets, export to `ArtifactRef`. Do not inline base64 trajectories, NPZ files, or thousands of cases.
