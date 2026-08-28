# Example Agent Flows

These are protocol-level examples, not implementation code.

## 1. Build and run a mission

**User:** “Simulate a two-stage ascent and report apogee.”

Agent:
1. calls `system_capabilities` filtered to mission/simulation/staging;
2. constructs MDL;
3. calls `mission_validate`;
4. calls `mission_compile`;
5. calls `simulation_run`, requesting native background task execution if appropriate;
6. follows task progress;
7. calls `simulation_summary`;
8. calls `simulation_events` to confirm stage-separation chronology;
9. reports apogee plus mission SHA, solver and provenance.

The agent must not call a shell tool or manually integrate equations outside UniFlight.

## 2. Debug a staging problem

**User:** “The booster disappears at separation.”

Agent:
1. `simulation_summary`
2. `simulation_events` filtered around separation
3. `simulation_state_at` immediately before separation
4. `vehicle_forces` if needed
5. inspect mission via `mission_inspect`
6. identify event/action/schema issue
7. `mission_apply_overrides` to create corrected mission
8. rerun and compare.

If two events fire together, the agent explicitly checks priority/tied-event metadata.

## 3. Monte Carlo without flooding context

**User:** “Why did my 100,000-case landing campaign fail?”

Agent:
1. `analysis_status`
2. `analysis_failures(limit=100)`
3. paginate additional failure pages only as needed
4. group by error/metrics locally in reasoning
5. `analysis_case_replay` for representative failure IDs
6. compare replay with nominal
7. report dominant causes.

The agent never asks the server to return all 100,000 cases in one response.

## 4. Optimize trajectory

1. `optimization_validate`
2. `optimization_evaluate` at nominal
3. `optimization_run` as background task
4. inspect result
5. replay the best design with `simulation_run`
6. optionally `simulation_compare_solvers`
7. report feasibility, objective, constraint margins and provenance.

## 5. External benchmark

**User:** “Compare this trajectory to NESC.”

1. ingest external files through deployment-approved artifact path/process;
2. hash reference artifact;
3. run candidate;
4. export matching CSV;
5. audit reference timestamp cadence;
6. call `verification_compare_csv` with explicit alignment and tolerances;
7. run `verification_convergence`;
8. report evidence type `external_benchmark`.

If timestamps contain representational jitter on a documented uniform grid, use `snap_nominal_grid_then_interpolate` only with documented justification.

## 6. Data validity check before flight

1. `data_catalog_list(dataset_id_prefix=...)`
2. `data_validity_check` at expected Mach/alpha/Re conditions
3. `data_table_query`
4. if outside validity, warn or choose another dataset/model; do not silently present extrapolated values as validated.
