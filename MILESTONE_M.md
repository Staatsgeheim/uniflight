# Milestone M — Stable Plugin/API Architecture

UniFlight 0.13.0 turns the strict factory seam introduced by MDL into a public third-party extension system. The goal is POST2-style model replaceability: mission-specific or proprietary models can be installed independently and referenced declaratively without editing the UniFlight package.

## Architecture

The extension chain is:

```text
installed Python distribution
        ↓
importlib.metadata entry point: uniflight.plugins
        ↓
PluginDescriptor (plugin ID, plugin version, API version)
        ↓
PluginRegistrar
        ↓
namespaced MissionRegistry capabilities
        ↓
MDL exact-version requirement + namespaced type references
        ↓
MissionCompiler
        ↓
trusted A–L simulation/optimization runtime
```

Discovery is lazy. `uniflight-mission plugins` enumerates entry-point metadata without importing plugin code. Mission compilation imports only explicitly required plugins.

## Reproducibility contract

MDL requires exact plugin versions:

```yaml
plugins:
  - id: demo.nereid
    version: "1.0.0"
```

A mission may not use `demo.nereid:*` capability IDs without declaring `demo.nereid` in `plugins`. Compilation fails on missing packages, exact-version mismatch, or Plugin API incompatibility.

Run provenance contains:

```text
plugin_id, plugin_version, plugin_api_version
```

alongside mission and engineering-data checksums.

## Capability ownership

Core factories are owned by `core`; plugin registrations are owned by their plugin descriptor. Third-party type names are always `<plugin-id>:<local-name>`. Cross-owner replacement is rejected even when the caller asks for `replace=True`.

This prevents silent monkey-patching of a production mission model.

## Capability namespaces

The API reserves compiler-facing namespaces for bodies, atmospheres, environments, solvers, dynamics, guards, event actions, outputs, optimizers and dataset loaders.

It also reserves reusable engineering-model namespaces for gravity, aerodynamics, aerothermal physics, propulsion, GNC, sensors, actuators, subsystems, terrain, materials and chemistry.

Named model objects are declared at mission scope and may be composed by plugin dynamics:

```yaml
models:
  engine:
    category: propulsion
    type: vendor.engine:main
    config: {...}
```

This means a proprietary aero/propulsion/GNC model does not need a dedicated UniFlight core branch. A plugin can bind its own high-level dynamics or subsystem composition to those reusable model objects.

## Reference third-party distribution

`demo_plugin/` is a separate installable Python package. It is not included as internal `uniflight.*` code and exposes its descriptor only through the standard entry-point mechanism.

`missions/nereid_m_plugin.yaml` exercises all of the following from that installed distribution:

1. a declared propulsion model;
2. plugin 3-DOF dynamics consuming the model;
3. a plugin phase guard;
4. a plugin event action that removes a second vehicle;
5. a plugin specific-energy output;
6. a plugin grid-search optimizer.

No core file is edited for those six capabilities.

## Installed reference result

The independently installed packages reported:

```text
uniflight = 0.13.0
uniflight-demo-plugin = 1.0.0
Plugin API = 1.0
```

Nominal plugin mission outputs:

- final altitude: ~1115.209817 m;
- final mass: ~99.200000 kg;
- specific energy: ~-149656.137791 J/kg;
- active vehicles: 1 after the plugin event removes `discard`.

The plugin grid-search optimizer evaluates 31 candidate accelerations and selects the upper 8 m/s² design bound for the declared maximum-altitude objective.

## Boundaries

- Plugins are trusted in-process code, not a security sandbox.
- API stability applies to the declared Plugin API contracts, not arbitrary private `uniflight` internals.
- Generic engineering-model categories establish stable ownership/discovery. A plugin may presently use a plugin `dynamics` factory to compose them; future core assemblers can consume the same category objects directly without changing plugin packaging.
- Plugins do not confer flight validation, heritage or certification.

## POST2 parity impact

M closes a major workflow gap: user/mission-specific model replacement no longer requires forking the simulator. UniFlight can now host proprietary models and mission-specific logic as separately versioned packages while retaining deterministic mission definitions and provenance.
