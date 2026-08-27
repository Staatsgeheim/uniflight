# Milestone L — Mission Definition Language

UniFlight **0.12.0** adds a declarative Mission Definition Language (MDL) above the A–K simulation/optimization stack. The MDL is an orchestration layer: it compiles mission files into the same `VehicleSpec`, dynamics, integrator, event, engineering-data, multi-vehicle, and optimization objects available through the Python API.

The design goal is POST2-style research workflow configuration without requiring a mission author to write Python for every run.

## 1. Supported source formats

Mission documents may be written as:

- YAML (`.yaml`, `.yml`);
- TOML (`.toml`);
- JSON (`.json`).

All three are normalized into one internal document representation and receive a deterministic SHA-256 fingerprint.

The current MDL format version is:

```text
1.0
```

A mission using another format version is rejected rather than silently interpreted.

## 2. Core document sections

A version-1.0 mission can declare:

- `mission`: identity, mission time span, default solver, seed, descriptive metadata;
- `datasets`: exact engineering dataset IDs, versions, paths, and checksum policy;
- `bodies`: arbitrary spherical bodies parameterized by `mu` or mass, radius, and rotation vector;
- `atmospheres`: built-in vacuum and isothermal/hydrostatic reference atmospheres;
- `environments`: body/atmosphere binding;
- `solvers`: adaptive SciPy or deterministic fixed-step RK4 solver configurations;
- `vehicles`: initial state plus ordered phases;
- `vehicle_templates`: daughter definitions used by topology-changing events;
- `events`: global hybrid actions such as removal or rigid two-body staging;
- `outputs`: named runtime metrics;
- `optimization`: H design variables, objective, and nonlinear constraints;
- `monte_carlo`: deterministic dispersion declarations;
- `metadata`: user/project metadata reserved for provenance and tooling.

Unknown keys in the core sections are rejected. The intent is to catch spelling/configuration errors early rather than accept an ambiguous mission.

## 3. Mission fingerprint and provenance

The normalized document is serialized canonically and hashed:

\[
H_M=\operatorname{SHA256}(\operatorname{canonical}(M)).
\]

Every `MissionRunReport` includes:

- UniFlight package version;
- mission ID;
- mission SHA-256;
- exact dataset inventory `(dataset_id, version, content_sha256)`.

This makes the input configuration and engineering-data identity explicit in every run product.

## 4. Dataset declarations

Datasets are version pinned:

```yaml
datasets:
  - id: vehicle-x.aero
    version: "2026.1"
    path: data/vehicle-x-aero.npz
    verify_checksum: true
```

The compiler loads the K native NPZ dataset and verifies that the file's own provenance matches the declared `(id, version)`. A mismatch is a compilation error.

The MDL therefore cannot silently substitute a different dataset version.

## 5. Vehicle phases

A vehicle contains an initial state and an ordered phase sequence. Each phase selects:

- name;
- DOF (3 or 6);
- solver override;
- dynamics configuration;
- termination guard;
- optional state-transition defaults.

Example:

```yaml
vehicles:
  lander:
    body: nereid
    initial:
      dof: 3
      state:
        position: [1000000, 0, 0]
        velocity: [0, 0, 0]
        mass: 100
    phases:
      - name: powered
        dof: 3
        until: {type: time, value: 5.0, direction: 1}
        dynamics:
          gravity: true
          ideal_rocket:
            exhaust_velocity: 2000
            mass_flow: 1.0
            direction_i: [1, 0, 0]
      - name: coast
        dof: 6
        transition:
          attitude: [1, 0, 0, 0]
          angular_rate: [0, 0, 0]
        dynamics:
          gravity: true
```

The compiler converts phase exits to normal Milestone-I universe events. There is no separate “mission-script” dynamics engine.

## 6. Guards

Version 1.0 supports three guard classes:

### Time

\[
g(t)=t-t_e.
\]

### Altitude

\[
g(\mathbf r)=h(\mathbf r)-h_e.
\]

### State field

\[
g(\mathbf X)=X_k-X_{k,e}.
\]

Each guard may specify direction and priority. The event is passed to the existing root-finding/hybrid event machinery.

## 7. DOF transitions

Phase changes can transition 3→6 or 6→3 DOF.

For 3→6, translational state and mass are inherited while the attitude/angular-rate policy is supplied explicitly (or attitude may be aligned from velocity where supported by the underlying transition function).

For 6→3, rotational state is projected out while position, velocity, and mass are preserved.

The resulting vehicle trajectory remains a sequence of schema-tagged Milestone-I segments.

## 8. Declarative topology changes

Global events expose the multi-vehicle runtime.

### Remove vehicle

`remove_vehicle` deletes the source vehicle at the event.

### Rigid two-body separation

`rigid_separation` invokes the Milestone-I momentum-consistent separation model. The declaration supplies:

- parent inertia;
- retained/detached daughter masses;
- daughter COM offsets;
- daughter inertias;
- optional relative separation velocity;
- angular-momentum conservation policy;
- daughter vehicle templates.

The daughter masses must sum to the parent mass and offsets must satisfy the parent COM condition.

After the jump, each daughter is compiled from its template and propagated as an independent vehicle.

## 9. Solver declarations

Two built-in solver families are exposed:

### Adaptive reference

```yaml
reference:
  type: scipy
  method: DOP853
  rtol: 1e-10
  atol: 1e-12
  max_step: 0.25
```

### Deterministic fixed-step

```yaml
campaign:
  type: rk4
  step: 0.05
  save_every_step: false
```

A mission may define multiple solver profiles, choose a default, and override it per vehicle or phase.

## 10. Requested outputs

The MDL currently supports named final metrics:

- state field;
- altitude;
- speed;
- mission end time;
- active vehicle count.

Outputs become the metric namespace used by the H optimization declaration.

## 11. Optimization declarations

H optimization variables point directly into the mission document using RFC-6901 JSON Pointers.

```yaml
optimization:
  method: SLSQP
  design_variables:
    - name: mass_flow
      pointer: /vehicles/lander/phases/0/dynamics/ideal_rocket/mass_flow
      lower: 0.5
      upper: 1.5
  objective:
    metric: final_mass
    sense: maximize
  constraints:
    - metric: final_altitude
      lower: 600
```

For each design vector, the compiler creates an overridden mission document, recompiles it deterministically, runs it, extracts metrics, and returns them to the existing H `TrajectoryProblem`.

Thus the optimizer operates on declarative mission inputs rather than mission-specific Python callbacks.

## 12. Monte Carlo declarations

Version 1.0 supports normal and uniform scalar dispersions, also addressed by JSON Pointer.

```yaml
monte_carlo:
  cases: 1000
  seed: 20260827
  dispersions:
    - name: thrust_scale
      pointer: /vehicles/lander/phases/0/dynamics/ideal_rocket/mass_flow
      distribution: normal
      mean: 1.0
      std: 0.03
```

Milestone L validates and deterministically samples these declarations. Large-scale execution remains the responsibility of the existing F.1/N-style campaign layer so the MDL does not duplicate multiprocessing infrastructure.

## 13. Registry boundary

`MissionRegistry` is a strict `(category, type) -> factory` mapping. L uses it for built-in body and atmosphere construction and establishes the extension seam for Milestone M.

Unknown types fail with an explicit list of available types. Duplicate registration requires explicit replacement.

Milestone M will turn this seam into a public plugin/entry-point system for arbitrary user atmosphere, gravity, propulsion, aero, thermal, GNC, terrain, sensor, actuator, and optimization models.

## 14. CLI

Installation exposes:

```text
uniflight-mission
```

Commands:

```bash
uniflight-mission validate mission.yaml
uniflight-mission inspect mission.yaml
uniflight-mission run mission.yaml --output report.json
uniflight-mission optimize mission.yaml --output optimum.json
uniflight-mission sample mission.yaml --cases 100 --seed 42 --output samples.json
uniflight-mission schema --output mission-1.0.schema.json
```

`validate` performs parse, semantic validation, cross-reference checks, dataset provenance checks, and runtime compilation without propagating the trajectory.

## 15. Reference missions

### `missions/nereid_l.yaml`

Demonstrates:

- exact K dataset reference;
- YAML input;
- adaptive/reference + RK4 solver profiles;
- powered 3-DOF phase;
- 3→6 DOF transition at 5 s;
- 6→3 DOF transition at 8 s;
- runtime output metrics;
- H optimization declaration;
- Monte Carlo dispersion declaration.

Reference nominal outputs:

- final altitude: ~961.584818 m;
- final speed: ~100.787989 m/s;
- final mass: 95.0 kg.

The embedded optimization maximizes final mass subject to final altitude >= 600 m and converges near:

- mass flow: ~0.633600 kg/s;
- final mass: ~96.832000 kg;
- final altitude: ~600.000001 m.

### `missions/nereid_l_staging.yaml`

Demonstrates a purely declarative 6-DOF stage-separation event at 2 s. The parent stack is replaced by `upper` and `booster` daughters, both of which propagate independently to 4 s.

### `missions/nereid_l_minimal.toml`

Provides a minimal TOML coast mission.

## 16. Deliberate boundaries

Milestone L is not yet the general third-party model/plugin ecosystem. Built-in runtime model declarations are intentionally limited while the configuration semantics are stabilized.

In particular:

- K datasets can be loaded/version-pinned/provenance-recorded, but arbitrary dataset-to-model wiring will be generalized through the Milestone-M plugin factories;
- L samples Monte Carlo declarations but does not introduce a second campaign scheduler;
- rigid two-body separation is built in, but arbitrary user-defined topology actions belong to the plugin layer;
- YAML is data only: no executable Python expressions or unsafe constructors are accepted.

These boundaries preserve deterministic/reviewable mission inputs.

## 17. POST2-parity significance

After L, a substantial UniFlight mission can be described as data rather than code. The mission document can select bodies, datasets, solvers, vehicles, phases, DOF changes, hybrid staging, optimization variables, constraints, dispersions, and outputs while compiling into the already verified A–K implementation.

The next planned milestone is **M — Plugin/API Architecture**: stable public factory interfaces and plugin discovery for mission-specific/proprietary models without editing UniFlight core code.
