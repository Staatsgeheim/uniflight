# Verification Record — UniFlight Milestone M

UniFlight **0.13.0** preserves all 130 A–L verification cases and adds **12 Plugin/API cases**, for **142/142 total tests passed in bounded groups**.

## New M cases (131–142)

131. plugin descriptor and registrar produce namespaced capability IDs and ownership metadata;
132. cross-owner registry replacement is rejected;
133. plugin discovery is lazy and does not import code merely to list entry points;
134. exact-version plugin requirement successfully loads and registers capabilities;
135. plugin package-version mismatch is rejected;
136. Plugin API major/version mismatch is rejected;
137. optional missing plugin requirement is permitted;
138. a namespaced MDL capability without an explicit plugin requirement is rejected;
139. compiler executes plugin-declared model + dynamics + output and records plugin provenance;
140. plugin guard and event action mutate the multi-vehicle topology through `UniverseMutation`;
141. plugin compatibility failures are surfaced as mission compilation failures;
142. registry capability inventory ordering is deterministic.

## Bounded regression execution

```bash
# A–C core / atmospheric / 6-DOF
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src pytest -q \
  tests/test_state_and_frames.py tests/test_gravity.py tests/test_rocket.py \
  tests/test_events.py tests/test_attitude.py tests/test_atmosphere.py \
  tests/test_flow_aero_propulsion.py tests/test_atmospheric_ascent.py \
  tests/test_6dof_flow_and_aero.py tests/test_6dof_dynamics_and_tvc.py \
  tests/test_6dof_atmospheric_flight.py
# 26/26 passed

# D entry/re-entry
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src pytest -q tests/test_entry_reentry.py
# 11/11 passed

# E EDL
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src pytest -q tests/test_edl.py
# 10/10 passed

# F GNC
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src pytest -q tests/test_gnc_robustness.py
# 12/12 passed

# F.1 performance
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src pytest -q tests/test_f1_performance.py
# 4/4 passed

# G terminal robustness
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src pytest -q tests/test_g_terminal_robustness.py
# 4/4 passed

# H + I
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src pytest -q \
  tests/test_h_optimization.py tests/test_i_multivehicle.py
# 21/21 passed

# J + K
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src pytest -q \
  tests/test_j_subsystems.py tests/test_k_engineering_data.py
# 25/25 passed

# L + M
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src pytest -q \
  tests/test_l_mission_language.py tests/test_m_plugins.py
# 29/29 passed
```

Total: **142/142 passed**.

The H process-parallel test may emit Python's standard Linux `fork()` deprecation warning in a multi-threaded parent process; the test itself passes.

## Installed-package plugin acceptance

Core and the reference plugin were installed as separate editable distributions with no source-path override:

```text
uniflight 0.13.0
uniflight-demo-plugin 1.0.0
Plugin API 1.0
```

`uniflight-mission plugins` discovered `demo.nereid` through `importlib.metadata`.

`uniflight-mission validate missions/nereid_m_plugin.yaml` succeeded with mission SHA-256:

```text
2b5ecae9bb0e893e96ffebe752bc454e2fe80ec67415a6effc257e084bad7090
```

The installed mission exercised six plugin-owned capabilities spanning propulsion, dynamics, guard, event action, output, and optimizer.

## Nereid-M nominal reference

`missions/nereid_m_plugin.yaml` uses a plugin propulsion model and plugin dynamics for the first four seconds, transitions using a plugin guard, then coasts under the core point-mass model. A separate marker vehicle is removed by a plugin event action at 2 s.

Reference outputs:

- final altitude: **1115.209817241 m**;
- final mass: **99.200000000 kg**;
- specific energy: **-149656.137790755 J/kg**;
- final active vehicles: **1**.

The run report records:

```text
plugin_id = demo.nereid
plugin_version = 1.0.0
plugin_api_version = 1.0
```

## Plugin optimizer reference

The YAML selects `demo.nereid:grid-search` as its optimizer. The separate plugin evaluates 31 candidates and returns:

- success: true;
- acceleration: **8.0 m/s²**;
- final altitude: **1187.209960779 m**;
- max constraint violation: **0.0**;
- evaluations: **31**.

## M acceptance invariants

- plugin discovery is through the standard Python distribution entry-point mechanism;
- installed plugins are not imported merely by the `plugins` listing command;
- mission files must exact-version-pin every namespaced plugin they reference;
- Plugin API incompatibility aborts compilation before propagation;
- plugin capability names are automatically namespaced by plugin ID;
- plugins cannot overwrite core/other-owner capability IDs;
- model factories are separated from compiler-facing dynamics/event/output factories;
- plugin event actions use the existing I atomic topology-mutation interface;
- plugin optimizers wrap H `TrajectoryProblem` rather than bypassing the simulator;
- run provenance records the exact plugin inventory;
- engineering-data provenance/checksums and canonical mission SHA remain unchanged;
- all A–L regression behavior remains intact.
