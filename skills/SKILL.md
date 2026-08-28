---
name: uniflight-framework
description: Expert agent skill for using, extending, testing, verifying, debugging, and operating UniFlight 1.0.3, a Python research flight-dynamics framework covering 3/6-DOF simulation, atmospheric/space flight, hybrid events, EDL, GNC, optimization, multi-vehicle dynamics, engineering subsystems/data, declarative missions, plugins, Monte Carlo/HPC analysis, numerical verification, and the FastMCP server.
---

# UniFlight Framework — Agent Skill

Use this skill whenever a task involves the UniFlight framework: creating or modifying a mission, writing Python simulations, using MDL YAML/TOML/JSON, selecting physics models, adding events, staging vehicles, configuring GNC, optimization, Monte Carlo/HPC campaigns, engineering-data tables, plugins, verification, debugging, benchmarking, or extending the framework.

This skill is grounded in UniFlight **1.0.3**. Treat the checked-out project supplied by the user as the source of truth if it differs from this skill. Never silently assume an API exists: inspect the installed version/source when uncertain.

## 0. Scope and scientific claims

UniFlight is research/engineering software. It supports mathematical/software verification and external benchmark comparison. Do **not** claim:
- validation against flown missions unless such validation has actually been performed and documented;
- flight heritage;
- certification;
- independent IV&V;
- NASA endorsement.

A NESC check-case comparison is an **independent software/model benchmark**, not flight validation.

## 1. Agent operating procedure

For every UniFlight task:

1. **Identify the execution surface.**
   - Existing Python project/source tree: inspect `pyproject.toml`, `src/uniflight`, tests, and mission files.
   - Installed package only: query `uniflight.__version__`, CLI `--help`, and import symbols.
   - MDL mission task: prefer declarative mission editing unless the required model is not exposed by MDL.
   - Framework-extension task: use the Python API or Plugin API; do not hack mission compiler internals when a public seam exists.

2. **Classify the problem.**
   Choose one or more:
   - deterministic 3-DOF/6-DOF propagation;
   - atmospheric ascent / orbital coast / entry / EDL;
   - hybrid events or phase changes;
   - multi-vehicle/staging;
   - GNC/estimation/actuation;
   - flexible/slosh/gear/engine subsystem dynamics;
   - engineering data;
   - targeting/optimization;
   - Monte Carlo/sensitivity/HPC;
   - plugin development;
   - verification/benchmarking;
   - MCP server tools/resources (`uniflight-mcp`).

3. **Choose fidelity explicitly.**
   State which gravity, atmosphere, aero, rarefied, thermal, TPS, contact, subsystem, sensor, and solver models are active. Do not imply higher fidelity than configured.

4. **Use SI internally.**
   UniFlight's model contracts are SI-first. Convert external engineering units at boundaries and document conversions. Never mix ft/lbf/slug data directly into SI model fields.

5. **Preserve frames and quaternion conventions.**
   - Canonical attitude quaternion maps **body → inertial**.
   - Body axes: `+x` forward, `+y` right, `+z` down.
   - Use framework frame/quaternion helpers instead of ad-hoc transforms.

6. **Preserve state ownership.**
   Build/augment a `StateSchema`; use `StateView` or schema slices rather than magic indices. Subsystems own their state fields.

7. **Treat events as hybrid-system operations.**
   Guards detect roots; actions/jumps mutate state/topology. Set direction and priority intentionally. In 1.0.2 simultaneous adaptive roots are collected and ordered; continuing non-jump events are protected against zero-time re-trigger cycles.

8. **Separate sampled GNC from adaptive RHS evaluation.**
   Never mutate EKF/controller/command state inside an adaptive `solve_ivp` RHS. Use the sampled-data closed-loop engine: propagate physics between chronological GNC ticks and hold commands zero-order between ticks.

9. **Make runs reproducible.**
   Pin mission/dataset/plugin versions, seeds, solver settings, and engineering-data checksums. Record mission SHA-256 and package version.

10. **Verify before declaring success.**
    Run the smallest relevant unit/regression test, then the relevant example/mission, then broader verification when the change touches core dynamics/events/state/frames.

## 2. Quick decision tree

- User wants a mission without custom Python → use **MDL** (`uniflight-mission`), see `references/05_mission_dsl.md`.
- User wants a custom research model → Python API, see `references/02_python_api.md`.
- User wants reusable proprietary/custom model selectable from MDL → **Plugin API**, see `references/10_plugins.md`.
- User wants target/optimize trajectory → `TrajectoryProblem` / MDL optimization, see `references/07_optimization.md`.
- User wants staging/multiple vehicles/DOF changes → `MultiVehicleUniverseEngine` / declarative topology, see `references/06_hybrid_multivehicle.md`.
- User wants Monte Carlo/sweeps/Sobol/multistart → analysis/HPC layer, see `references/11_analysis_hpc.md`.
- User wants confidence/correctness comparison → verification layer, see `references/12_verification.md`.
- User wants NESC Case 04 reproduction → see `references/13_benchmarking.md`.
- User wants an agent/MCP surface → `uniflight-mcp` / `src/uniflight_mcp/`; contracts in `mcp/`.

## 3. Installation and environment

Supported Python: **3.11+**.

Typical editable development install:
```bash
python -m venv .venv
# Windows PowerShell:
.venv\Scripts\Activate.ps1
# POSIX:
source .venv/bin/activate
python -m pip install -U pip
python -m pip install -e ".[dev]"
```

Wheel install:
```bash
python -m pip install uniflight-1.0.3-py3-none-any.whl
python -m pip install "uniflight[mcp]"
```

Check:
```bash
python -c "import uniflight; print(uniflight.__version__)"
uniflight-mission --help
uniflight-analysis --help
uniflight-verify --help
uniflight-mcp --help
```

For multiprocessing campaigns, especially on Windows:
```powershell
$env:OPENBLAS_NUM_THREADS="1"
$env:OMP_NUM_THREADS="1"
$env:MKL_NUM_THREADS="1"
```
and put process-spawning code behind `if __name__ == "__main__":`.

## 4. Core architecture mental model

Think of UniFlight as layers:

**state/schema → environment → force/wrench/subsystem models → dynamics RHS → integrator → hybrid event engine → multi-vehicle universe → mission compiler → optimization/analysis → verification**

Do not bypass layers casually. Prefer composition:
- state: `StateSchema`, `StateView`;
- forces/moments: `Wrench`, `WrenchModel`;
- environment: `PlanetaryEnvironment`;
- dynamics: `DynamicsAssembler` or `RigidBody6DOFDynamics`;
- events: `Event`, `VehicleEvent`;
- integration: `ScipyIVPIntegrator` or `FixedStepRK4Integrator`;
- universe: `MultiVehicleUniverseEngine`;
- mission: `MissionCompiler`;
- analysis: `MissionCampaignRunner`;
- verification: `VerificationReport`, `ReferenceTimeHistory`.

## 5. Built-in capability map

### State / math / frames
`state.py`, `frames.py`, `units.py`, `invariants.py`.

### Bodies / gravity / environment
`bodies.py`, `gravity.py`, `atmosphere.py`, `environment.py`, `terrain.py`.

### Gas / flow / aerodynamics
`gases.py`, `flow.py`, `aerodynamics.py`, `hypersonics.py`, `rarefied.py`, `chemistry.py`.

### Propulsion / mass
`propulsion.py`, `massflow.py`, `mass_properties.py`, `engine_dynamics.py`.

### Thermal / TPS
`heating.py`, `tps.py`.

### 6-DOF and forces
`wrenches.py`, `dynamics.py`.

### EDL / surface
`deployables.py`, `contact.py`, `gear_dynamics.py`, `guidance.py`.

### GNC
`sensors.py`, `estimation.py`, `control.py`, `actuators.py`, `aborts.py`, `closed_loop.py`.

### Engineering subsystems
`flexibility.py`, `slosh.py`, `subsystems.py`, `faults.py`.

### Hybrid / multi-vehicle
`events.py`, `simulation.py`, `modes.py`, `dof.py`, `separation.py`, `multibody.py`, `universe.py`.

### Optimization
`optimization.py`.

### Engineering data
`engineering_data.py`, `data_models.py`.

### Declarative missions / plugins
`mission.py`, `mission_cli.py`, `plugins.py`.

### Analysis / HPC
`analysis.py`, `hpc.py`, `result_store.py`, `montecarlo.py`, `analysis_cli.py`.

### Verification
`verification.py`, `verification_cases.py`, `verify_cli.py`.

### MCP server
`uniflight_mcp/` (optional extra). Tools call UniFlight public APIs only. Contracts: `mcp/TOOL_POLICY.json`, `mcp/schemas/tools/`.

Read the matching reference file before making nontrivial changes.

## 6. Solver selection

Use **DOP853** as the trusted adaptive reference for smooth/high-accuracy deterministic propagation. Use fixed-step **RK4** for deterministic high-throughput campaigns when convergence against the adaptive reference has been demonstrated.

Adaptive example:
```python
from uniflight.integrators import SolverConfig, ScipyIVPIntegrator

integrator = ScipyIVPIntegrator(
    SolverConfig(method="DOP853", rtol=1e-10, atol=1e-12, max_step=0.25)
)
```

Campaign example:
```python
from uniflight.integrators import FixedStepRK4Config, FixedStepRK4Integrator

integrator = FixedStepRK4Integrator(
    FixedStepRK4Config(step=0.05, save_every_step=False)
)
```

Do not choose a large RK4 step solely for speed. Establish a convergence/error budget first.

## 7. Hybrid-event rules

When defining a guard:
- choose a physically meaningful scalar with a clean sign change;
- set `direction` to reject the wrong crossing;
- set `priority` for simultaneous events;
- use a jump/action only when state/topology changes;
- use continuation for observation-only roots.

For staging, touchdown, deployment, cutoff, abort, phase changes, or DOF transitions, add regression tests for:
- event time;
- direction;
- simultaneous ties;
- priority ordering;
- post-event state;
- no zero-time retrigger;
- both adaptive and fixed-step integrators if both are supported.

## 8. Data and provenance rules

Use `EngineeringTable` for regular N-D engineering data. Every production-quality table should define:
- axis names and units;
- interpolation method;
- extrapolation policy per axis;
- output units;
- validity envelope;
- uncertainty metadata if known;
- dataset ID/version/source/citation/license/authorship;
- checksum.

Never silently extrapolate beyond an engineering validity envelope. Distinguish:
- mathematical interpolation/extrapolation domain;
- engineering validity domain.

Use `EngineeringDataCatalog` for version-explicit lookup.

## 9. Mission DSL rules

MDL format version is `1.0`. Supported file formats: YAML, TOML, JSON.

Prefer exact dataset/plugin version pinning. Validate before run:
```bash
uniflight-mission validate mission.yaml
uniflight-mission inspect mission.yaml
uniflight-mission run mission.yaml --output report.json
```

Use JSON Pointers for optimization and dispersions. Do not mutate `MissionDocument.raw`; 1.0.2 makes mission data deeply immutable by design. Create overrides/copies through supported mission utilities.

## 10. Optimization rules

A `TrajectoryProblem` is a black-box mapping:
**design vector → deterministic trajectory evaluation → metrics → objective/constraints**.

Keep simulation and optimizer decoupled. Use:
- scaled bounded `DesignVariable`s;
- `MetricObjective`;
- `MetricConstraint`;
- SLSQP for smooth constrained problems;
- COBYLA/derivative-free fallback where appropriate;
- finite-difference Jacobians only with sensible variable scales and deterministic evaluations;
- multiple shooting for long/unstable trajectories.

Cache identical evaluations when objective and constraints request the same design point.

## 11. GNC rules

Sampled-data chronology is mandatory:
1. propagate physical state to next GNC tick;
2. generate sensor measurements;
3. update navigation estimator;
4. run guidance;
5. run controller;
6. update command bus;
7. propagate with held commands to next tick.

Use Joseph-form covariance updates already provided by the EKF. Treat sensor noise/bias seeds as part of reproducibility.

## 12. Multi-vehicle rules

Each vehicle may have its own:
- schema/DOF;
- RHS/models;
- solver;
- environment;
- GNC;
- event set.

The universe advances to the earliest event across active vehicles, synchronizes all vehicles to that physical time, applies ordered mutations, then continues.

Trajectory history is **segment-based**, not one rectangular global matrix, because topology and schemas can change.

For separation:
- daughter masses must sum to parent mass;
- COM offsets must satisfy the parent COM relation;
- verify linear and angular momentum residuals;
- explicitly specify relative separation velocity/impulse and daughter inertias.

## 13. Analysis/HPC rules

All campaigns should reduce to deterministic case records with stable case IDs. Use `SQLiteResultStore` as both result store and checkpoint.

Use:
- `SerialBackend` for debugging;
- `ProcessBackend` for local CPU campaigns;
- `ExternalExecutorBackend` for institutional/distributed executors.

Do not make workers write directly to SQLite; coordinator is the writer.

Restart compatibility requires matching campaign identity and mission SHA-256.

## 13b. MCP rules

The MCP layer is an operations API, not a second physics engine.
- Install with `pip install "uniflight[mcp]"` and run `uniflight-mcp`.
- Prefer the 36 named tools; do not invent `plugin_install`, `shell`, or `python_exec`.
- Use FastMCP background tasks for long simulation/optimization/campaign tools.
- Paginate events, history, catalog, and campaign cases with `page` / cursors. Do not dump entire campaigns.
- Pin datasets/plugins; record mission SHA, solver, seed, and artifact SHA in provenance.
- STDIO is trusted-local. HTTP requires configured tokens (`UNIFLIGHT_MCP_TOKENS`).
- Campaign SQLite writes stay on the coordinator. Workers must not open the store.

## 14. Verification rules

Before changing core numerics, establish the expected independent answer:
- analytical solution;
- conservation law;
- manufactured solution;
- finite-difference derivative;
- convergence theory;
- independent integrator;
- external benchmark.

Run:
```bash
uniflight-verify run --output reports/verification.json --markdown reports/verification.md
```

For external CSV:
```bash
uniflight-verify compare-csv reference.csv actual.csv \
  --time-column time \
  --channels altitude speed \
  --abs-tol 1e-6 \
  --rel-tol 1e-8 \
  --output reports/comparison.json
```

When comparing external sampled data, inspect time-grid precision before interpolation. The NESC Case 04 work demonstrated that tiny stored timestamp representation offsets can manufacture sawtooth residuals if interpreted as physical time shifts.

## 15. Change-management checklist

For any framework modification:
- inspect existing tests covering the touched module;
- add a focused regression reproducing the defect/feature;
- preserve public API where possible;
- update version/docs/schema when contract changes;
- run focused tests;
- run relevant examples;
- run broader regression groups;
- run formal verifier for numerical-core changes;
- build/install wheel for packaging changes;
- smoke-test CLIs;
- record known limitations.

## 16. What not to do

- Do not mutate stateful GNC inside an adaptive RHS.
- Do not use raw numeric state indices when schema access is available.
- Do not assume quaternion direction or axis convention.
- Do not mix units.
- Do not use mission-specific hard-coded data when an `EngineeringTable`/plugin is appropriate.
- Do not claim a benchmark passed without the independent reference data.
- Do not count skipped external cases as passes.
- Do not treat agreement between two solvers as physical validation.
- Do not silently replace dataset/plugin versions.
- Do not hide failed Monte Carlo cases in aggregate statistics.
- Do not parallelize tiny cases blindly; process overhead can dominate.
- Do not run untrusted plugins: plugins are trusted in-process Python code.
- Do not add MCP tools that install plugins, execute shell, or read arbitrary filesystem paths.

## 17. Reference map

Read these files as needed:
- `references/01_architecture.md`
- `references/02_python_api.md`
- `references/03_physics_models.md`
- `references/04_gnc_subsystems.md`
- `references/05_mission_dsl.md`
- `references/06_hybrid_multivehicle.md`
- `references/07_optimization.md`
- `references/08_engineering_data.md`
- `references/09_edl_thermal.md`
- `references/10_plugins.md`
- `references/11_analysis_hpc.md`
- `references/12_verification.md`
- `references/13_benchmarking.md`
- `references/14_testing_debugging.md`
- `references/15_agent_recipes.md`
- `references/16_api_index.md`
- `references/17_limitations.md`

Use the live repository `examples/` and `missions/` directories. Do not rely on bundled copies.
