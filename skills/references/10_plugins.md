# UniFlight Plugin API 1.0

Milestone M defines a stable public extension boundary for mission-specific and proprietary models. Plugins are **trusted in-process Python code**. UniFlight does not sandbox plugin execution.

## Discovery

An installed distribution exposes one entry point in the `uniflight.plugins` group:

```toml
[project.entry-points."uniflight.plugins"]
"vendor.plugin" = "vendor_package:plugin_descriptor"
```

The entry-point name is the plugin ID. The loaded object must be a `PluginDescriptor`, a callable returning one, or an object exposing `plugin_descriptor`.

```python
from uniflight import PluginDescriptor, PLUGIN_API_VERSION

def plugin_descriptor():
    return PluginDescriptor(
        plugin_id="vendor.plugin",
        version="2.4.1",
        api_version=PLUGIN_API_VERSION,
        register=register,
    )
```

Only plugins explicitly required by a mission are imported during mission compilation.

## Mission pinning

```yaml
plugins:
  - id: vendor.plugin
    version: "2.4.1"
    required: true
```

Plugin versions are exact by design in MDL 1.0. A missing plugin, version mismatch, or Plugin API mismatch aborts compilation before propagation.

## Namespaces and ownership

A plugin registers local capability names through `PluginRegistrar`:

```python
def register(r):
    r.register("aero", "hypersonic-db", build_aero)
    r.register("dynamics", "entry-6dof", build_rhs)
```

Mission-visible IDs become:

```text
vendor.plugin:hypersonic-db
vendor.plugin:entry-6dof
```

A plugin cannot silently overwrite a core registration or another plugin's capability. Registry inventory records owner and owner version.

## Stable capability categories

Compiler-facing categories:

- `body`
- `atmosphere`
- `environment`
- `solver`
- `dynamics`
- `guard`
- `event_action`
- `output`
- `optimizer`
- `dataset_loader`

Reusable model categories:

- `gravity`
- `aero`
- `aerothermal`
- `propulsion`
- `gnc`
- `sensor`
- `actuator`
- `subsystem`
- `terrain`
- `material`
- `chemistry`

Reusable models are declared once in a mission:

```yaml
models:
  main_engine:
    category: propulsion
    type: vendor.plugin:engine
    config:
      chamber_pressure: 12.0e6
```

A plugin dynamics factory may consume these objects through `context["models"]`.

## Factory signature

All capability factories use:

```python
factory(config: Mapping[str, Any], context: Mapping[str, Any]) -> Any
```

For namespaced MDL capabilities, `config` is the mission's `config:` mapping, not the surrounding MDL wrapper. Context contents depend on category and are additive within Plugin API 1.x.

### Common context keys

- `document`: immutable `MissionDocument`
- `catalog`: `EngineeringDataCatalog` when engineering data are available
- `models`: already compiled named mission model objects when applicable
- `compiler`: `MissionCompiler` for advanced event/dynamics composition

### Model factories

Generic model factories receive `name`, bodies, atmospheres, environments, catalog, document, and previously built models when applicable.

### Dynamics

A `dynamics` factory receives:

- `schema`
- `body`
- `environment`
- `models`
- `document`
- `vehicle_id`
- `phase`
- `compiler`

It returns either `rhs(t, y) -> dy` or an object exposing `.rhs`.

### Guards

A `guard` factory receives schema/body/models/document and returns `guard(t, y) -> float`.

### Event actions

An `event_action` factory receives the current vehicle compilation context and returns a handler:

```python
handler(UniverseEventContext) -> UniverseMutation
```

This can remove, replace, spawn, stage, or switch vehicles through the existing Milestone-I topology engine.

### Outputs

An `output` factory receives the completed `UniverseResult`, output spec, bodies, models, and previously resolved outputs. It returns a scalar or zero-argument callable returning a scalar.

### Optimizers

An `optimizer` factory receives a `TrajectoryProblem` and declaration context. It may return:

- an object with `solve(problem)`;
- a callable `callable(problem)`;
- an already-computed `OptimizationResult`-compatible object.

## Reproducibility and provenance

Every `MissionRunReport` records:

- UniFlight version;
- canonical mission SHA-256;
- exact engineering dataset inventory/checksums;
- exact plugin `(id, version, api_version)` inventory.

The `capabilities` CLI command additionally records capability ownership.

## Security boundary

Plugins have the permissions of the Python process. Installing/running a plugin is equivalent to installing/running arbitrary Python code. MDL YAML/TOML/JSON remains non-executable by itself; code execution occurs only through explicitly installed and version-required plugin packages.

## Compatibility policy

`PLUGIN_API_VERSION = "1.0"`.

- API 1.x aims to keep registration and factory contracts backward-compatible.
- Breaking changes require a new major Plugin API version.
- Plugin package version is independent of Plugin API version.
- MDL format version is independent of both.
