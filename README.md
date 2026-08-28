# UniFlight

**A planet-agnostic 3-DOF / 6-DOF research flight-dynamics engine with declarative missions, plugins, campaign analysis, and formal numerical verification.**

[![Python 3.11+](https://img.shields.io/badge/python-3.11%2B-blue.svg)](https://www.python.org/downloads/)
[![PyPI](https://img.shields.io/pypi/v/uniflight.svg?style=flat&label=PyPI)](https://pypi.org/project/uniflight/)
[![License: MIT](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)
[![NumPy](https://img.shields.io/badge/numpy-2.0%2B-013243.svg)](https://numpy.org/)
[![SciPy](https://img.shields.io/badge/scipy-1.13%2B-8caae6.svg)](https://scipy.org/)

UniFlight integrates translational and rotational dynamics, atmospheres, aerodynamics, propulsion, GNC, multi-vehicle events, engineering tables, and HPC-style campaigns in one Python package. Bodies, atmospheres, and gravity are supplied by the user — nothing in the kernel is hard-wired to Earth.

It is a research / engineering simulator. It does **not** claim flight heritage, operational-mission validation, certification, or independent IV&V.

---

## What you can do

| Layer | Capabilities |
| --- | --- |
| **Kernel** | Immutable packed state, frame graph, SI metadata, single-owner RHS assembler |
| **3-DOF / 6-DOF** | Point-mass and rigid-body flight, quaternion kinematics, variable mass |
| **Environment** | Spherical bodies, gas mixtures, vacuum or hydrostatic atmospheres, tabulated gravity / terrain / air |
| **Aero & heating** | Continuum, Newtonian hypersonic, free-molecular, regime blending, chemistry corrections, Sutton–Graves / radiative heating, lumped ablating TPS |
| **Propulsion** | Ideal rocket, gimballed 6-DOF engines, tabulated performance, TVC |
| **EDL** | Parachutes, jettison, powered descent, landing-gear contact, hybrid mode switches |
| **GNC** | Sampled-data closed loop, sensors, EKF, quaternion PD, abort limits, Monte Carlo robustness |
| **Subsystems** | Engine transients, modal flexibility, slosh, dynamic gear, scheduled faults |
| **Multi-vehicle** | Event-synchronized universe, 3-DOF ↔ 6-DOF promotion/demotion, rigid separation |
| **Missions** | YAML / TOML Mission Definition Language, JSON Schema, SHA-256 mission identity |
| **Optimization** | Single-variable targeting, constrained SLSQP, multiple shooting, multistart batches |
| **Analysis** | Cartesian / zipped sweeps, Monte Carlo, Saltelli–Sobol, SQLite checkpoint / restart |
| **Plugins** | Entry-point discovery, exact version pins, namespaced capability registration |
| **Verification** | Analytical limits, manufactured solutions, conservation checks, CSV time-history compare |
| **MCP** | Optional FastMCP 3.x server: 36 tools for missions, simulation, data, optimization, campaigns, and verification |

---

## Requirements

- Python 3.11+
- NumPy 2.0+, SciPy 1.13+, PyYAML 6.0+

---

## Install

```bash
python -m pip install uniflight
```

From a clone, for development:

```bash
python -m pip install --no-build-isolation -e ".[dev]"
```

The MCP server is an optional extra:

```bash
python -m pip install "uniflight[mcp]"
```

This installs the library and four console scripts:

| Command | Role |
| --- | --- |
| `uniflight-mission` | Validate, inspect, run, and optimize declarative missions |
| `uniflight-analysis` | Sweeps, Monte Carlo, Sobol, multistart optimization, SQLite stores |
| `uniflight-verify` | Built-in verification suite and external CSV comparison |
| `uniflight-mcp` | FastMCP 3.x agent server (requires the `mcp` extra) |

---

## Quick start

### 1. Propagate a point-mass trajectory

Nothing in this snippet assumes Earth. Gravity and radius are just numbers you own.

```python
import numpy as np
from uniflight import (
    core_3dof_schema,
    PointMassGravity,
    TranslationalKinematics,
    DynamicsAssembler,
    SimulationEngine,
)

mu, radius = 8.0e11, 1.2e6
schema = core_3dof_schema()
y0 = schema.pack({
    "position": np.array([radius, 0.0, 0.0]),
    "velocity": np.array([0.0, 900.0, 300.0]),
    "mass": 1000.0,
})
rhs = DynamicsAssembler(schema, [TranslationalKinematics(PointMassGravity(mu))]).rhs
result = SimulationEngine(rhs).run((0.0, 1200.0), y0)
final = schema.unpack(result.states[-1])
print(np.linalg.norm(final["position"]), np.linalg.norm(final["velocity"]))
```

Or run the bundled example:

```bash
python examples/suborbital_point_mass.py
```

### 2. Fly a coupled 6-DOF vehicle

`examples/sixdof_atmospheric_flight.py` builds a fictional atmosphere, a gimballed rocket, linear-stability aerodynamics, and a rigid-body RHS, then integrates with SciPy DOP853.

```bash
python examples/sixdof_atmospheric_flight.py
```

### 3. Run a declarative mission end to end

```bash
uniflight-mission validate missions/nereid_l.yaml
uniflight-mission inspect  missions/nereid_l.yaml
uniflight-mission run      missions/nereid_l.yaml --output reports/mission.json
```

`validate` parses, resolves references, and compiles. `run` executes the compiled universe and writes a JSON report of requested outputs.

### 4. Verify the numerics

```bash
uniflight-verify run \
  --output reports/verification.json \
  --markdown reports/verification.md
```

Expected internal result: **12 passed, 2 skipped**. The skips are NASA/NESC external-benchmark placeholders; they are not counted as passes until you supply independent reference files.

---

## End-to-end workflows

### Atmospheric ascent and re-entry

| Script | What it exercises |
| --- | --- |
| `examples/atmospheric_ascent.py` | 3-DOF ascent through a hydrostatic atmosphere with rocket mass flow |
| `examples/reentry_6dof.py` | 6-DOF entry: continuum / hypersonic / rarefied blending, heating, TPS |
| `examples/full_edl.py` | Hybrid EDL: parachute inflate → jettison → throttle → gear contact |

```bash
python examples/atmospheric_ascent.py
python examples/reentry_6dof.py
python examples/full_edl.py
```

### Closed-loop GNC and robustness

Sampled-data guidance, sensors, estimation, and abort rules, plus campaign Monte Carlo:

```bash
python examples/gnc_monte_carlo.py
python examples/gnc_monte_carlo_g.py
```

### Targeting and trajectory optimization

Single-variable targeting, then constrained propellant minimization:

```bash
python examples/trajectory_optimization.py
```

Declarative equivalent (design variables and constraints live in the mission file):

```bash
uniflight-mission optimize missions/nereid_l.yaml --output reports/opt.json
```

### Multi-vehicle missions

`examples/multivehicle_mission.py` and `missions/nereid_l_staging.yaml` show event-synchronized vehicles, staging, and 3-DOF ↔ 6-DOF switches.

```bash
uniflight-mission run missions/nereid_l_staging.yaml
python examples/multivehicle_mission.py
```

### Engineering tables

Provenance-aware catalogs (CSV / NPZ) feed aero, atmosphere, gravity, terrain, materials, and propulsion models:

```bash
python examples/engineering_data_system.py
python examples/engineering_subsystems.py
```

Checksums can be required in the mission (`verify_checksum: true`). See `reports/k_datasets/` for the bundled synthetic tables.

### Plugins

Plugins are trusted in-process Python packages discovered via the `uniflight.plugins` entry-point group. A mission pins exact versions; a missing or mismatched plugin aborts compilation.

```bash
python -m pip install --no-build-isolation --no-deps -e demo_plugin
uniflight-mission plugins
uniflight-mission capabilities missions/nereid_m_plugin.yaml
uniflight-mission run          missions/nereid_m_plugin.yaml
```

Capability IDs are namespaced (`demo.nereid:constant-acceleration`). A plugin cannot overwrite a core registration or another plugin’s name. Details: [`PLUGIN_API.md`](PLUGIN_API.md).

---

## Mission Definition Language

Missions are YAML or TOML documents (`format_version: "1.0"`). A typical file declares:

- `mission` — id, time span, default solver, optional seed
- `bodies`, `atmospheres`, `environments`, `solvers`
- `datasets` — catalog entries with optional checksum verification
- `vehicles` — initial DOF/state, phased dynamics, event guards
- `outputs` — altitude, speed, mass, vehicle count, custom plugin metrics
- `optimization` — design pointers, objective, constraints
- `monte_carlo` — dispersions on JSON-pointer paths
- `analysis` — sweeps, Sobol studies, multistart batches, store path
- `plugins` — required third-party capabilities

JSON-pointer overrides (`/vehicles/lander/phases/0/dynamics/ideal_rocket/mass_flow`) are the seam used by optimization, Monte Carlo, and analysis.

```bash
# Editor schema
uniflight-mission schema --output missions/mission-1.0.schema.json

# Sample dispersions without flying trajectories
uniflight-mission sample missions/nereid_l.yaml --cases 32 --output reports/samples.json
```

Bundled missions:

| File | Intent |
| --- | --- |
| `missions/nereid_l.yaml` | Phased 3-DOF → 6-DOF coast, optimization + Monte Carlo |
| `missions/nereid_l_staging.yaml` | Staging / multi-body topology change |
| `missions/nereid_l_minimal.toml` | Smallest TOML mission |
| `missions/nereid_m_plugin.yaml` | Installed-plugin propulsion and outputs |
| `missions/nereid_n_analysis.yaml` | Sweep, Sobol, Monte Carlo, and multistart batch |

---

## Analysis and HPC campaigns

`uniflight-analysis` runs many compiled-mission cases against a transactional SQLite store (WAL). Case IDs are stable: worker count and wall-clock time do not change identity. Re-run the same campaign ID against the same mission SHA-256 to skip completed cases and retry failures.

```bash
uniflight-analysis list missions/nereid_n_analysis.yaml

uniflight-analysis sweep        missions/nereid_n_analysis.yaml propulsion-grid
uniflight-analysis monte-carlo  missions/nereid_n_analysis.yaml --cases 1000
uniflight-analysis sobol        missions/nereid_n_analysis.yaml propulsion-sensitivity
uniflight-analysis optimize-batch missions/nereid_n_analysis.yaml multistart

uniflight-analysis status reports/n_analysis.sqlite nereid-n-analysis.monte_carlo
uniflight-analysis export reports/n_analysis.sqlite \
  nereid-n-analysis.monte_carlo reports/mc.json
```

Backends:

- `serial` — caller process, best for debugging
- `process` — `ProcessPoolExecutor` with `spawn` (`workers: 0` = CPUs minus one)
- `ExternalExecutorBackend` — wrap any `concurrent.futures.Executor` (cluster / cloud). UniFlight does not import Dask, Ray, MPI, or Slurm itself.

For CPU-heavy process campaigns, pin BLAS to one thread per worker:

```bash
set OPENBLAS_NUM_THREADS=1
set OMP_NUM_THREADS=1
set MKL_NUM_THREADS=1
```

Contracts: [`HPC_API.md`](HPC_API.md).

---

## Formal verification

Every scalar check uses an explicit tolerance:

```text
error <= absolute + relative * max(|reference|, scale_floor)
```

There is no hidden global epsilon.

`uniflight-verify run` evaluates twelve internal cases:

1. RK4 manufactured exponential — observed order ≈ 4
2. Adaptive manufactured sine (`y = sin t`)
3. Tsiolkovsky Δv quadrature
4. One-period circular Kepler orbit / energy
5. Point-mass gravity Jacobian vs finite difference
6. Constant-rate quaternion kinematics
7. Axisymmetric torque-free rigid body
8. Hybrid event-root timing
9. DOP853 vs RK4 cross-integrator
10. Rigid two-body separation momentum
11. Frame-graph round trip
12. Long-run quaternion-norm stability

Two NASA/NESC external manifests stay `SKIP` until you obtain reference trajectories independently.

Compare your own time histories:

```bash
uniflight-verify compare-csv reference.csv actual.csv \
  --channels altitude speed \
  --abs-tol 1e-6 --rel-tol 1e-8 \
  --output reports/external_comparison.json
```

Python API:

```python
from uniflight.verification_cases import run_builtin_verification

report = run_builtin_verification()
assert report.failed == 0
assert report.passed == 12
assert report.skipped == 2
```

---

## MCP server

`uniflight-mcp` is a FastMCP 3.x server that calls UniFlight public APIs. Contracts live in `mcp/`; the implementation is `src/uniflight_mcp/`.

```bash
# STDIO (trusted local)
uniflight-mcp --transport stdio --workspace ./workspace

# Streamable HTTP (configure tokens via UNIFLIGHT_MCP_TOKENS)
uniflight-mcp --transport http --host 127.0.0.1 --port 8000 --workspace ./workspace

# Discover the 36-tool contract
fastmcp list mcp/server.py --json --input-schema --output-schema
```

There is no plugin-install or shell tool. Long runs use FastMCP background tasks. Redis/Valkey Docket is optional via `UNIFLIGHT_MCP_DOCKET_URL`.

---

## Python API map

Import from the top-level package. A few composition patterns:

```python
from uniflight import (
    SphericalBody, PlanetaryEnvironment, IsothermalHydrostaticAtmosphere,
    core_6dof_schema, ConstantMassProperties, GimballedRocketEngine,
    ContinuumAerodynamics6DOF, RigidBody6DOFDynamics, QuaternionKinematics,
    DynamicsAssembler, SimulationEngine, ScipyIVPIntegrator, SolverConfig,
    MissionCompiler, load_mission,
    ParameterSweep, MissionCampaignRunner, ProcessBackend, SQLiteResultStore,
    PluginManager,
    run_builtin_verification,
)
```

| Concern | Start here |
| --- | --- |
| State / frames | `StateSchema`, `StateView`, `FrameGraph`, `core_3dof_schema`, `core_6dof_schema` |
| Dynamics | `DynamicsAssembler`, `RigidBody6DOFDynamics`, `IdealRocket`, `SimulationEngine` |
| Integrators | `ScipyIVPIntegrator`, `FixedStepRK4Integrator` |
| Closed loop | `SampledDataClosedLoopEngine`, `LandingGNCController`, `ExtendedKalmanFilter` |
| Universe | `MultiVehicleUniverseEngine`, `VehicleSpec`, `RigidSeparationHandler` |
| Missions | `load_mission`, `MissionCompiler`, `pointer_get` / `pointer_set` |
| Campaigns | `MissionCampaignRunner`, `ParameterSweep`, `MissionMonteCarlo`, `SobolSensitivity` |
| Plugins | `PluginManager`, `PluginDescriptor`, `PLUGIN_API_VERSION` |
| Verification | `TolerancePolicy`, `ReferenceTimeHistory`, `run_builtin_verification` |

---

## Tests

```bash
python -m pytest
```

The suite covers kernel frames, atmospheres, 6-DOF aero/TVC, entry/EDL, GNC robustness, optimization, multi-vehicle events, subsystems, engineering tables, the mission language, plugins, analysis/HPC, verification, and the MCP server.

---

## Repository layout

```text
src/uniflight/     library
src/uniflight_mcp/ FastMCP 3.x server (optional extra)
tests/             pytest suite
examples/          runnable Python demonstrations
missions/          YAML / TOML missions + JSON Schema
reports/           reference JSON / SQLite / synthetic tables
demo_plugin/       separate third-party plugin distribution
skills/            agent skill for UniFlight
mcp/               FastMCP tool/resource contracts and design spec
```

---

## Scope

UniFlight does **not** claim:

- validation against operational flight missions
- flight heritage or NASA endorsement
- certification or independent IV&V
- that bundled tables are flight-validated engineering data

External time-history comparison is verification against a reference you supply, not validation of a flown vehicle.

---

## License

MIT. See [`LICENSE`](LICENSE).
