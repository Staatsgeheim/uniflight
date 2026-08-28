# Resource and Resource-Template Catalog

All resources are read-only and idempotent. Dynamic resources are registered as FastMCP resource templates using RFC 6570-compatible URI templates.

## Static resources

| URI | Media type | Purpose |
|---|---|---|
| `uniflight://system/version` | `application/json` | Server, UniFlight, Python, FastMCP versions |
| `uniflight://system/capabilities` | `application/json` | Compact capability summary; use `system_capabilities` for large inventories |
| `uniflight://schema/mission/1.0` | `application/schema+json` | MDL JSON Schema |
| `uniflight://docs/architecture` | `text/markdown` | Architecture reference |
| `uniflight://docs/plugin-api` | `text/markdown` | Plugin API reference |
| `uniflight://docs/hpc-api` | `text/markdown` | Analysis/HPC reference |
| `uniflight://docs/verification` | `text/markdown` | Verification reference |

## Dynamic templates

| Template | Purpose |
|---|---|
| `uniflight://missions/{mission_id}` | Original persisted mission representation |
| `uniflight://missions/{mission_id}/normalized` | Canonical normalized mission |
| `uniflight://runs/{run_id}/manifest` | Immutable run provenance manifest |
| `uniflight://runs/{run_id}/summary` | Compact run summary |
| `uniflight://runs/{run_id}/events` | Event export resource; large event queries should use paginated tool |
| `uniflight://runs/{run_id}/exports/{artifact_id}` | Run-scoped export |
| `uniflight://campaigns/{campaign_id}/summary` | Campaign summary/checkpoint state |
| `uniflight://verification/{verification_id}` | Verification report |
| `uniflight://datasets/{dataset_id}/{version}` | Dataset metadata/provenance |
| `uniflight://artifacts/{artifact_id}` | Generic stored artifact |

## Resource response policy

Resources representing JSON/Markdown summaries may be returned inline. Large CSV/NPZ/binary artifacts are served as resource contents only if transport/client limits permit; otherwise a deployment may provide a separately authenticated download endpoint. The MCP URI remains canonical.

Resource reads must authorize the caller against the owning tenant/project before resolving the underlying path.

## Notifications

If the server supports resource update notifications, use them only for mutable summaries such as an in-progress campaign summary. Immutable run/mission/artifact resources do not emit changes.
