# Tool Catalog

All tools are component version `1`. Exact schemas live under `schemas/tools/`.

| Tool | Read-only | Native task | Timeout | Pagination | Purpose |
|---|---:|---:|---:|---:|---|
| `system_version` | yes | no | 10 s | no | Return server/runtime versions. |
| `system_capabilities` | yes | no | 15 s | yes | List server/UniFlight capabilities with opaque cursor pagination. |
| `mission_validate` | yes | no | 30 s | no | Validate MDL syntax, schema, semantic references, datasets and plugin requirements without running. |
| `mission_inspect` | yes | no | 20 s | no | Inspect a persisted mission in normalized semantic form. |
| `mission_compile` | no | no | 45 s | no | Validate, normalize and compile a mission, optionally persisting immutable mission artifacts. |
| `mission_apply_overrides` | no | no | 30 s | no | Create a new immutable mission by JSON-Pointer overrides. |
| `simulation_run` | no | yes | 3600 s | no | Compile and execute a deterministic UniFlight mission. |
| `simulation_summary` | yes | no | 20 s | no | Return compact persisted run summary. |
| `simulation_events` | yes | no | 20 s | yes | List actual event chronology with tied-event/priority metadata. |
| `simulation_state_at` | yes | no | 20 s | no | Return named vehicle state at a time. |
| `simulation_vehicle_history` | yes | no | 30 s | yes | Return paginated schema-tagged trajectory rows for one vehicle. |
| `simulation_export_csv` | no | no | 120 s | no | Create a CSV artifact for selected trajectory channels. |
| `simulation_compare_solvers` | no | yes | 3600 s | no | Run the same mission with two solver configurations and compare states/metrics. |
| `environment_sample` | yes | no | 20 s | no | Sample configured gravity/atmosphere/wind/terrain at a point. |
| `vehicle_flow_state` | yes | no | 20 s | no | Compute relative-flow diagnostics at a stored run state. |
| `vehicle_forces` | yes | no | 30 s | no | Break down force/moment contributions at a run state. |
| `optimization_validate` | yes | no | 30 s | no | Validate a targeting/optimization declaration without solving. |
| `optimization_evaluate` | no | no | 1800 s | no | Evaluate one deterministic design point. |
| `optimization_run` | no | yes | 7200 s | no | Solve a constrained trajectory targeting/optimization problem. |
| `analysis_sweep` | no | yes | 86400 s | no | Run a deterministic parameter sweep with checkpoint/restart. |
| `analysis_monte_carlo` | no | yes | 86400 s | no | Run deterministic-seeded Monte Carlo/uncertainty propagation. |
| `analysis_sobol` | no | yes | 86400 s | no | Run Saltelli/Sobol global sensitivity analysis. |
| `analysis_optimization_batch` | no | yes | 86400 s | no | Run parallel multistart optimization. |
| `analysis_status` | yes | no | 20 s | no | Inspect persistent campaign/checkpoint status. |
| `analysis_cases` | yes | no | 30 s | yes | List campaign cases using snapshot/keyset pagination. |
| `analysis_failures` | yes | no | 30 s | yes | List failed campaign cases with replayable parameters. |
| `analysis_case_replay` | no | no | 3600 s | no | Reproduce one campaign case with its exact seed/parameters. |
| `data_catalog_list` | yes | no | 30 s | yes | List versioned engineering datasets. |
| `data_table_query` | yes | no | 30 s | no | Query an exact-version N-D engineering table. |
| `data_validity_check` | yes | no | 20 s | no | Check engineering validity separately from mathematical interpolation domain. |
| `verification_builtin` | no | yes | 3600 s | no | Run UniFlight built-in formal mathematical/software verification. |
| `verification_compare_csv` | no | no | 600 s | no | Compare candidate CSV against independent reference with explicit tolerances/alignment. |
| `verification_compare_runs` | no | yes | 3600 s | no | Compare two persisted runs on a common time grid. |
| `verification_convergence` | no | yes | 7200 s | no | Run solver/timestep tolerance refinement and estimate convergence. |
| `plugin_list` | yes | no | 20 s | yes | List installed UniFlight plugins without installing/importing unrequired code beyond discovery policy. |
| `plugin_inspect` | yes | no | 20 s | no | Inspect plugin version/API compatibility and capability ownership. |

## Contract rules

- Every output schema is a root JSON object and accepts either the successful structured result or the shared typed failure envelope.
- FastMCP annotations must match the `read_only` classification in `TOOL_REGISTRY.json`.
- `task=true` means the component is eligible for native MCP background execution; it does not change the output schema.
- Tool-level pagination uses `PageRequest`/`PageInfo`; it is distinct from MCP component discovery pagination.
- Large binary exports return `ArtifactRef`, never inline base64 in the ordinary result.
- All compute tools that use a mission/run must enforce tenant authorization on referenced artifacts.

## `system_version`

Return server/runtime versions.

- Tags: `system`
- Read-only: `True`
- Native background task capable: `False`
- Synchronous timeout policy: `10 s`
- Input schema: `schemas/tools/system_version.input.json`
- Output schema: `schemas/tools/system_version.output.json`

## `system_capabilities`

List server/UniFlight capabilities with opaque cursor pagination.

- Tags: `system`
- Read-only: `True`
- Native background task capable: `False`
- Synchronous timeout policy: `15 s`
- Input schema: `schemas/tools/system_capabilities.input.json`
- Output schema: `schemas/tools/system_capabilities.output.json`
- Large-result policy: opaque cursor pagination with stable snapshot.

## `mission_validate`

Validate MDL syntax, schema, semantic references, datasets and plugin requirements without running.

- Tags: `mission`
- Read-only: `True`
- Native background task capable: `False`
- Synchronous timeout policy: `30 s`
- Input schema: `schemas/tools/mission_validate.input.json`
- Output schema: `schemas/tools/mission_validate.output.json`

## `mission_inspect`

Inspect a persisted mission in normalized semantic form.

- Tags: `mission`
- Read-only: `True`
- Native background task capable: `False`
- Synchronous timeout policy: `20 s`
- Input schema: `schemas/tools/mission_inspect.input.json`
- Output schema: `schemas/tools/mission_inspect.output.json`

## `mission_compile`

Validate, normalize and compile a mission, optionally persisting immutable mission artifacts.

- Tags: `mission`
- Read-only: `False`
- Native background task capable: `False`
- Synchronous timeout policy: `45 s`
- Input schema: `schemas/tools/mission_compile.input.json`
- Output schema: `schemas/tools/mission_compile.output.json`

## `mission_apply_overrides`

Create a new immutable mission by JSON-Pointer overrides.

- Tags: `mission`
- Read-only: `False`
- Native background task capable: `False`
- Synchronous timeout policy: `30 s`
- Input schema: `schemas/tools/mission_apply_overrides.input.json`
- Output schema: `schemas/tools/mission_apply_overrides.output.json`

## `simulation_run`

Compile and execute a deterministic UniFlight mission.

- Tags: `simulation`
- Read-only: `False`
- Native background task capable: `True`
- Synchronous timeout policy: `3600 s`
- Input schema: `schemas/tools/simulation_run.input.json`
- Output schema: `schemas/tools/simulation_run.output.json`

## `simulation_summary`

Return compact persisted run summary.

- Tags: `simulation`
- Read-only: `True`
- Native background task capable: `False`
- Synchronous timeout policy: `20 s`
- Input schema: `schemas/tools/simulation_summary.input.json`
- Output schema: `schemas/tools/simulation_summary.output.json`

## `simulation_events`

List actual event chronology with tied-event/priority metadata.

- Tags: `simulation, events`
- Read-only: `True`
- Native background task capable: `False`
- Synchronous timeout policy: `20 s`
- Input schema: `schemas/tools/simulation_events.input.json`
- Output schema: `schemas/tools/simulation_events.output.json`
- Large-result policy: opaque cursor pagination with stable snapshot.

## `simulation_state_at`

Return named vehicle state at a time.

- Tags: `simulation`
- Read-only: `True`
- Native background task capable: `False`
- Synchronous timeout policy: `20 s`
- Input schema: `schemas/tools/simulation_state_at.input.json`
- Output schema: `schemas/tools/simulation_state_at.output.json`

## `simulation_vehicle_history`

Return paginated schema-tagged trajectory rows for one vehicle.

- Tags: `simulation`
- Read-only: `True`
- Native background task capable: `False`
- Synchronous timeout policy: `30 s`
- Input schema: `schemas/tools/simulation_vehicle_history.input.json`
- Output schema: `schemas/tools/simulation_vehicle_history.output.json`
- Large-result policy: opaque cursor pagination with stable snapshot.

## `simulation_export_csv`

Create a CSV artifact for selected trajectory channels.

- Tags: `simulation, export`
- Read-only: `False`
- Native background task capable: `False`
- Synchronous timeout policy: `120 s`
- Input schema: `schemas/tools/simulation_export_csv.input.json`
- Output schema: `schemas/tools/simulation_export_csv.output.json`

## `simulation_compare_solvers`

Run the same mission with two solver configurations and compare states/metrics.

- Tags: `simulation, verification`
- Read-only: `False`
- Native background task capable: `True`
- Synchronous timeout policy: `3600 s`
- Input schema: `schemas/tools/simulation_compare_solvers.input.json`
- Output schema: `schemas/tools/simulation_compare_solvers.output.json`

## `environment_sample`

Sample configured gravity/atmosphere/wind/terrain at a point.

- Tags: `physics`
- Read-only: `True`
- Native background task capable: `False`
- Synchronous timeout policy: `20 s`
- Input schema: `schemas/tools/environment_sample.input.json`
- Output schema: `schemas/tools/environment_sample.output.json`

## `vehicle_flow_state`

Compute relative-flow diagnostics at a stored run state.

- Tags: `physics`
- Read-only: `True`
- Native background task capable: `False`
- Synchronous timeout policy: `20 s`
- Input schema: `schemas/tools/vehicle_flow_state.input.json`
- Output schema: `schemas/tools/vehicle_flow_state.output.json`

## `vehicle_forces`

Break down force/moment contributions at a run state.

- Tags: `physics, diagnostics`
- Read-only: `True`
- Native background task capable: `False`
- Synchronous timeout policy: `30 s`
- Input schema: `schemas/tools/vehicle_forces.input.json`
- Output schema: `schemas/tools/vehicle_forces.output.json`

## `optimization_validate`

Validate a targeting/optimization declaration without solving.

- Tags: `optimization`
- Read-only: `True`
- Native background task capable: `False`
- Synchronous timeout policy: `30 s`
- Input schema: `schemas/tools/optimization_validate.input.json`
- Output schema: `schemas/tools/optimization_validate.output.json`

## `optimization_evaluate`

Evaluate one deterministic design point.

- Tags: `optimization`
- Read-only: `False`
- Native background task capable: `False`
- Synchronous timeout policy: `1800 s`
- Input schema: `schemas/tools/optimization_evaluate.input.json`
- Output schema: `schemas/tools/optimization_evaluate.output.json`

## `optimization_run`

Solve a constrained trajectory targeting/optimization problem.

- Tags: `optimization`
- Read-only: `False`
- Native background task capable: `True`
- Synchronous timeout policy: `7200 s`
- Input schema: `schemas/tools/optimization_run.input.json`
- Output schema: `schemas/tools/optimization_run.output.json`

## `analysis_sweep`

Run a deterministic parameter sweep with checkpoint/restart.

- Tags: `analysis`
- Read-only: `False`
- Native background task capable: `True`
- Synchronous timeout policy: `86400 s`
- Input schema: `schemas/tools/analysis_sweep.input.json`
- Output schema: `schemas/tools/analysis_sweep.output.json`

## `analysis_monte_carlo`

Run deterministic-seeded Monte Carlo/uncertainty propagation.

- Tags: `analysis, monte-carlo`
- Read-only: `False`
- Native background task capable: `True`
- Synchronous timeout policy: `86400 s`
- Input schema: `schemas/tools/analysis_monte_carlo.input.json`
- Output schema: `schemas/tools/analysis_monte_carlo.output.json`

## `analysis_sobol`

Run Saltelli/Sobol global sensitivity analysis.

- Tags: `analysis, sobol`
- Read-only: `False`
- Native background task capable: `True`
- Synchronous timeout policy: `86400 s`
- Input schema: `schemas/tools/analysis_sobol.input.json`
- Output schema: `schemas/tools/analysis_sobol.output.json`

## `analysis_optimization_batch`

Run parallel multistart optimization.

- Tags: `analysis, optimization`
- Read-only: `False`
- Native background task capable: `True`
- Synchronous timeout policy: `86400 s`
- Input schema: `schemas/tools/analysis_optimization_batch.input.json`
- Output schema: `schemas/tools/analysis_optimization_batch.output.json`

## `analysis_status`

Inspect persistent campaign/checkpoint status.

- Tags: `analysis`
- Read-only: `True`
- Native background task capable: `False`
- Synchronous timeout policy: `20 s`
- Input schema: `schemas/tools/analysis_status.input.json`
- Output schema: `schemas/tools/analysis_status.output.json`

## `analysis_cases`

List campaign cases using snapshot/keyset pagination.

- Tags: `analysis`
- Read-only: `True`
- Native background task capable: `False`
- Synchronous timeout policy: `30 s`
- Input schema: `schemas/tools/analysis_cases.input.json`
- Output schema: `schemas/tools/analysis_cases.output.json`
- Large-result policy: opaque cursor pagination with stable snapshot.

## `analysis_failures`

List failed campaign cases with replayable parameters.

- Tags: `analysis`
- Read-only: `True`
- Native background task capable: `False`
- Synchronous timeout policy: `30 s`
- Input schema: `schemas/tools/analysis_failures.input.json`
- Output schema: `schemas/tools/analysis_failures.output.json`
- Large-result policy: opaque cursor pagination with stable snapshot.

## `analysis_case_replay`

Reproduce one campaign case with its exact seed/parameters.

- Tags: `analysis, debug`
- Read-only: `False`
- Native background task capable: `False`
- Synchronous timeout policy: `3600 s`
- Input schema: `schemas/tools/analysis_case_replay.input.json`
- Output schema: `schemas/tools/analysis_case_replay.output.json`

## `data_catalog_list`

List versioned engineering datasets.

- Tags: `data`
- Read-only: `True`
- Native background task capable: `False`
- Synchronous timeout policy: `30 s`
- Input schema: `schemas/tools/data_catalog_list.input.json`
- Output schema: `schemas/tools/data_catalog_list.output.json`
- Large-result policy: opaque cursor pagination with stable snapshot.

## `data_table_query`

Query an exact-version N-D engineering table.

- Tags: `data`
- Read-only: `True`
- Native background task capable: `False`
- Synchronous timeout policy: `30 s`
- Input schema: `schemas/tools/data_table_query.input.json`
- Output schema: `schemas/tools/data_table_query.output.json`

## `data_validity_check`

Check engineering validity separately from mathematical interpolation domain.

- Tags: `data`
- Read-only: `True`
- Native background task capable: `False`
- Synchronous timeout policy: `20 s`
- Input schema: `schemas/tools/data_validity_check.input.json`
- Output schema: `schemas/tools/data_validity_check.output.json`

## `verification_builtin`

Run UniFlight built-in formal mathematical/software verification.

- Tags: `verification`
- Read-only: `False`
- Native background task capable: `True`
- Synchronous timeout policy: `3600 s`
- Input schema: `schemas/tools/verification_builtin.input.json`
- Output schema: `schemas/tools/verification_builtin.output.json`

## `verification_compare_csv`

Compare candidate CSV against independent reference with explicit tolerances/alignment.

- Tags: `verification, external-benchmark`
- Read-only: `False`
- Native background task capable: `False`
- Synchronous timeout policy: `600 s`
- Input schema: `schemas/tools/verification_compare_csv.input.json`
- Output schema: `schemas/tools/verification_compare_csv.output.json`

## `verification_compare_runs`

Compare two persisted runs on a common time grid.

- Tags: `verification`
- Read-only: `False`
- Native background task capable: `True`
- Synchronous timeout policy: `3600 s`
- Input schema: `schemas/tools/verification_compare_runs.input.json`
- Output schema: `schemas/tools/verification_compare_runs.output.json`

## `verification_convergence`

Run solver/timestep tolerance refinement and estimate convergence.

- Tags: `verification`
- Read-only: `False`
- Native background task capable: `True`
- Synchronous timeout policy: `7200 s`
- Input schema: `schemas/tools/verification_convergence.input.json`
- Output schema: `schemas/tools/verification_convergence.output.json`

## `plugin_list`

List installed UniFlight plugins without installing/importing unrequired code beyond discovery policy.

- Tags: `plugin`
- Read-only: `True`
- Native background task capable: `False`
- Synchronous timeout policy: `20 s`
- Input schema: `schemas/tools/plugin_list.input.json`
- Output schema: `schemas/tools/plugin_list.output.json`
- Large-result policy: opaque cursor pagination with stable snapshot.

## `plugin_inspect`

Inspect plugin version/API compatibility and capability ownership.

- Tags: `plugin`
- Read-only: `True`
- Native background task capable: `False`
- Synchronous timeout policy: `20 s`
- Input schema: `schemas/tools/plugin_inspect.input.json`
- Output schema: `schemas/tools/plugin_inspect.output.json`
