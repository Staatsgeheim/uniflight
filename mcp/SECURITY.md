# Security, Authorization, and Deployment

## Transport trust

### STDIO
Trusted local process. OAuth authorization is unavailable; filesystem restrictions and safe tool design still apply.

### Streamable HTTP
Recommended shared/remote deployment. Require an AuthProvider/TokenVerifier and enforce scopes.

## Suggested scopes

- `uniflight:read`
- `uniflight:compute`
- `uniflight:write-artifacts`
- `uniflight:analysis`
- `uniflight:admin`

## Compute quotas

Per tenant/principal:
- concurrent native tasks;
- CPU workers;
- campaign case limit;
- artifact storage bytes;
- synchronous runtime ceiling;
- rate limit.

The server never silently reduces model fidelity or changes solver tolerances to meet quota.

## Path safety

Client-facing arguments use IDs/URIs. If a path import is enabled by deployment policy, it must resolve within an allowlisted root after symlink resolution. Reject traversal.

## Plugins

Plugins are trusted executable Python. v1 supports inspection only, not install/uninstall. Administrator deployment controls installed plugin packages.

## Prohibited v1 tool classes

- arbitrary Python
- arbitrary shell
- dependency install
- arbitrary URL downloader
- arbitrary filesystem read/write
- destructive delete
- plugin installer
- dynamic code upload

## Logging

Do not log OAuth tokens, full proprietary mission files, full engineering tables, or arbitrary file content. Prefer IDs, hashes, dimensions, and safe summaries.

## Errors

Set `mask_error_details=True`. Internal tracebacks stay server-side with correlation IDs. Controlled domain error data may be returned when safe and actionable.
