# Native MCP Background Task Contract

Long operations use FastMCP/MCP background tasks rather than custom `job_*` tools.

## Task-capable tools

- simulation_run
- simulation_compare_solvers
- optimization_run
- analysis_sweep
- analysis_monte_carlo
- analysis_sobol
- analysis_optimization_batch
- verification_builtin
- verification_compare_runs
- verification_convergence

## Execution mode policy

Normal simulation/optimization tools are `optional` task mode. A deployment may require task mode when a cost estimate crosses configured CPU/runtime/case thresholds.

Read-only metadata tools forbid background tasks.

## Progress semantics

Simulation: physical time or phase count.  
Optimization: evaluations completed and current stage.  
Monte Carlo/sweep: completed cases / total.  
Sobol: completed samples / total.  
Verification: checks / total.

Messages should use domain terms, not implementation internals.

## Cancellation

Cancellation should:
1. stop scheduling additional work;
2. allow already-running atomic evaluations to finish/cancel safely;
3. flush completed campaign cases to the result store;
4. leave the campaign resumable.

## Persistent campaigns vs tasks

An MCP task is execution lifecycle. A UniFlight campaign is persistent scientific state. `analysis_status`, `analysis_cases`, and restart semantics remain valid after an MCP task ends or a client disconnects.

## Production workers

Local development: Docket `memory://`.  
Horizontal production: Redis/Valkey-backed Docket plus `fastmcp tasks worker`.

Task-enabled components must be registered at startup.
