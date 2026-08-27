# UniFlight — Milestone L Mission Definition Language

UniFlight **0.12.0** adds a declarative Mission Definition Language (MDL) on top of the complete A–K celestial-body-agnostic flight-dynamics stack.

Mission authors can now describe bodies, exact-version engineering datasets, solver profiles, vehicles, flight phases, 3↔6-DOF transitions, hybrid staging, requested outputs, trajectory-optimization variables/constraints, and Monte Carlo dispersions in **YAML, TOML, or JSON** instead of assembling every mission procedurally in Python.

## New in L

- strict MDL format version **1.0**
- YAML / TOML / JSON loading
- deterministic normalized mission SHA-256
- semantic validation and cross-reference checking
- exact dataset ID/version/provenance verification
- adaptive SciPy and fixed-step RK4 solver profiles
- declarative vehicles and ordered phases
- time / altitude / state-field phase guards
- per-phase solver and dynamics selection
- declarative **3→6 and 6→3 DOF transitions**
- global hybrid events
- declarative momentum-consistent **rigid two-body staging** via vehicle templates
- final state / altitude / speed / time / vehicle-count metrics
- RFC-6901 JSON-pointer mission overrides
- H trajectory optimization directly from mission-file variables/objectives/constraints
- deterministic normal/uniform Monte Carlo dispersion declarations
- strict `MissionRegistry` factory seam for Milestone M plugins
- `uniflight-mission` CLI
- editor-facing mission schema export
- **130/130 total verification tests pass in bounded groups**

Package version: **0.12.0**.

## Install

```bash
python -m pip install -e . --no-build-isolation
```

Dependencies:

- Python >= 3.11
- NumPy >= 2.0
- SciPy >= 1.13
- PyYAML >= 6.0
- pytest >= 8 for development

## Run the YAML reference mission

```bash
uniflight-mission validate missions/nereid_l.yaml
uniflight-mission inspect missions/nereid_l.yaml
uniflight-mission run missions/nereid_l.yaml \
  --output reports/l_reference.json
```

The reference mission starts with a powered 3-DOF phase, switches to 6-DOF coast at 5 s, switches back to 3-DOF at 8 s, and terminates at 12 s.

Reference nominal result:

- altitude: ~961.584818 m
- speed: ~100.787989 m/s
- mass: 95.0 kg

## Optimize directly from YAML

```bash
uniflight-mission optimize missions/nereid_l.yaml \
  --output reports/l_optimization.json
```

The YAML declares the rocket mass-flow design variable, final-mass objective, and final-altitude constraint. No optimization-specific Python evaluator is needed.

Reference optimum:

- mass flow: ~0.633600 kg/s
- final altitude: ~600.000001 m
- final mass: ~96.832000 kg

## Declarative staging

```bash
uniflight-mission run missions/nereid_l_staging.yaml
```

At 2 s the 6-DOF parent stack is replaced by two independently propagated daughters through the same momentum-consistent Milestone-I separation model used by the Python API.

## Monte Carlo declarations

```bash
uniflight-mission sample missions/nereid_l.yaml \
  --cases 100 --seed 20260827 \
  --output reports/l_mc_samples.json
```

L validates and samples the declared dispersions. Existing F.1/N campaign infrastructure remains responsible for large-scale parallel execution.

## Mission schema

```bash
uniflight-mission schema --output missions/mission-1.0.schema.json
```

Runtime semantic validation remains authoritative because it also checks cross references, JSON pointers, dataset provenance, and compilation.

## Documents

- `MILESTONE_L.md` — MDL semantics, hybrid-event model, optimization/MC integration, boundaries
- `MILESTONE_K.md` — engineering data system
- `MILESTONE_J.md` — engineering subsystem dynamics
- `MILESTONE_I.md` — multi-vehicle / multi-DOF runtime
- `MILESTONE_H.md` — targeting and optimization
- `VERIFICATION.md` — full L verification record
- `missions/nereid_l.yaml` — full YAML reference mission
- `missions/nereid_l_staging.yaml` — declarative staging mission
- `missions/nereid_l_minimal.toml` — minimal TOML mission
- `missions/mission-1.0.schema.json` — editor-facing schema
- `reports/l_reference.json` — deterministic nominal run
- `reports/l_optimization.json` — deterministic optimization result

## Project scope

The project target remains functional/architectural proximity to NASA POST2 while explicitly **not claiming** real-mission validation, flight heritage, or certification/independent-IV&V pedigree.

The next roadmap item is **Milestone M — Plugin/API Architecture**: stable public plugin contracts and discovery for mission-specific or proprietary atmosphere, gravity, aero, aerothermal, propulsion, GNC, terrain, sensor, actuator, subsystem, event-action, and optimization models without editing the UniFlight core package.
