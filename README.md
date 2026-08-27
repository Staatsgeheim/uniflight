# UniFlight — Milestone M Plugin/API Architecture

UniFlight **0.13.0** adds a stable public plugin system on top of the complete A–L celestial-body-agnostic flight-dynamics stack.

Mission-specific or proprietary models can now live in separately installed Python distributions and be referenced from MDL YAML/TOML/JSON without modifying UniFlight core code.

## New in M

- Plugin API version **1.0**
- `importlib.metadata` discovery via the `uniflight.plugins` entry-point group
- lazy discovery: installed plugin metadata can be listed without importing plugin code
- explicit mission-level plugin requirements
- exact plugin-version pinning
- Plugin API compatibility checks
- namespaced capability IDs: `<plugin-id>:<capability>`
- protected capability ownership; plugins cannot silently replace core or other vendors
- capability provenance/inventory
- stable compiler-facing extension categories:
  - body, atmosphere, environment, solver, dynamics, guard, event action, output, optimizer, dataset loader
- stable reusable model categories:
  - gravity, aero, aerothermal, propulsion, GNC, sensor, actuator, subsystem, terrain, material, chemistry
- mission-level named `models:` declarations
- plugin dynamics can compose declared model objects
- namespaced plugin guards and topology-changing event actions
- plugin scalar output metrics
- plugin optimizers
- plugin dataset-loader hook
- plugin inventory in every mission run report
- new `uniflight-mission plugins` and `capabilities` CLI commands
- separate installable third-party reference distribution in `demo_plugin/`
- **142/142 total verification tests pass in bounded groups**

Package version: **0.13.0**.

## Install core

```bash
python -m pip install -e . --no-build-isolation --no-deps
```

## Install the separate demo plugin

```bash
python -m pip install -e demo_plugin --no-build-isolation --no-deps
```

The plugin is deliberately a separate Python distribution named `uniflight-demo-plugin`.

## Discover installed plugins

```bash
uniflight-mission plugins
```

Reference discovery:

```json
[
  {
    "distribution": "uniflight-demo-plugin",
    "entry_point": "uniflight_demo_plugin:plugin_descriptor",
    "plugin_id": "demo.nereid"
  }
]
```

## Run the installed-plugin mission

```bash
uniflight-mission validate missions/nereid_m_plugin.yaml
uniflight-mission capabilities missions/nereid_m_plugin.yaml
uniflight-mission run missions/nereid_m_plugin.yaml \
  --output reports/m_reference.json
```

The YAML explicitly requires:

```yaml
plugins:
  - id: demo.nereid
    version: "1.0.0"
```

It then uses six plugin capabilities without mission-specific Python:

- `demo.nereid:constant-acceleration` propulsion model
- `demo.nereid:point-mass-propulsion` dynamics
- `demo.nereid:time` phase guard
- `demo.nereid:remove-vehicle` event action
- `demo.nereid:specific-energy` output
- `demo.nereid:grid-search` optimizer

Reference nominal result:

- final altitude: ~1115.209817 m
- final mass: ~99.200000 kg
- specific energy: ~-149656.137791 J/kg
- active vehicles: 1

## Optimize with a plugin optimizer

```bash
uniflight-mission optimize missions/nereid_m_plugin.yaml \
  --output reports/m_optimization.json
```

The external grid-search optimizer performs 31 trajectory evaluations and selects acceleration = **8.0 m/s²** for the declared maximum-altitude objective.

## Generic model declarations

Reusable plugin models are mission-level objects:

```yaml
models:
  main_propulsion:
    category: propulsion
    type: demo.nereid:constant-acceleration
    config:
      acceleration_mps2: 5.0
      mass_flow_kgps: 0.2
```

A plugin dynamics factory receives the compiled model map and can compose proprietary aero, propulsion, GNC, sensor, terrain, subsystem, or other models behind its own verified state equations.

## Capability ownership

```bash
uniflight-mission capabilities missions/nereid_m_plugin.yaml
```

The output records category, fully-qualified type, owner, owner version, and description. Core capabilities remain owned by `core`.

## Security

Plugins are **trusted in-process Python code**. Installing/running a plugin is equivalent to running arbitrary Python code with the current process permissions. MDL documents themselves remain data-only; plugin code executes only after a matching installed package is explicitly required.

## Documents

- `MILESTONE_M.md` — architecture, reproducibility rules, reference acceptance
- `PLUGIN_API.md` — Plugin API 1.0 author contract
- `MILESTONE_L.md` — declarative mission language
- `MILESTONE_K.md` — engineering-data system
- `VERIFICATION.md` — A–M verification record
- `missions/nereid_m_plugin.yaml` — installed-plugin reference mission
- `demo_plugin/` — separate reference third-party distribution
- `reports/m_reference.json` — nominal plugin mission result
- `reports/m_optimization.json` — plugin optimizer result
- `reports/m_capabilities.json` — capability ownership inventory

## Project scope

The project target remains functional/architectural proximity to NASA POST2 while explicitly **not claiming** real-mission validation, flight heritage, or certification/independent-IV&V pedigree.

The next roadmap item is **Milestone N — Integrated Analysis/HPC**: generalized parameter sweeps, optimization campaigns, Monte Carlo/uncertainty propagation, global sensitivity analysis, checkpoint/restart, structured result stores, and local/distributed execution backends.
