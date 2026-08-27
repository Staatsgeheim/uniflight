# Verification Record — UniFlight Milestone L

UniFlight **0.12.0** preserves all 113 A–K verification cases and adds 17 Mission Definition Language cases.

## New L cases (114–130)

114. deterministic YAML load and canonical mission SHA-256;
115. TOML load, compile, and propagation;
116. RFC-6901 pointer get/set and immutable mission override semantics;
117. rejection of unknown core mission keys;
118. cross-reference rejection for undefined celestial bodies;
119. exact-version/checksummed K engineering-data catalog resolution;
120. dataset declaration/file-provenance mismatch rejection;
121. end-to-end YAML phase execution with 3→6→3 DOF transitions;
122. runtime-derived requested output metrics;
123. H trajectory optimization compiled directly from YAML design variables/objective/constraint;
124. deterministic seeded Monte Carlo dispersion sampling;
125. invalid Monte Carlo pointer rejection;
126. strict/extensible `MissionRegistry` semantics;
127. editor-facing MDL schema contract;
128. CLI validate/inspect/run/schema workflow;
129. declarative 6-DOF rigid two-body staging and daughter propagation;
130. JSON mission load/compile/run.

## Bounded regression execution

```bash
# A–E physics
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src pytest -q \
  tests/test_gravity.py tests/test_state_and_frames.py tests/test_rocket.py \
  tests/test_attitude.py tests/test_events.py tests/test_atmosphere.py \
  tests/test_flow_aero_propulsion.py tests/test_atmospheric_ascent.py \
  tests/test_6dof_flow_and_aero.py tests/test_6dof_dynamics_and_tvc.py \
  tests/test_6dof_atmospheric_flight.py tests/test_entry_reentry.py tests/test_edl.py
# 47/47 passed

# F/F.1/G GNC + robustness
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src pytest -q \
  tests/test_gnc_robustness.py tests/test_f1_performance.py tests/test_g_terminal_robustness.py
# 20/20 passed

# H + I + J + K + L
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src pytest -q \
  tests/test_h_optimization.py tests/test_i_multivehicle.py tests/test_j_subsystems.py \
  tests/test_k_engineering_data.py tests/test_l_mission_language.py
# 63/63 passed
```

Total: **130/130 passed in bounded groups**.

The H process-parallel test may emit Python's standard Linux `fork()` deprecation warning in a multi-threaded parent process; the test itself passes.

## Nereid-L nominal reference

`missions/nereid_l.yaml` is compiled without mission-specific Python and executes:

```text
0 s     powered 3-DOF
5 s     -> coast 6-DOF
8 s     -> coast 3-DOF
12 s    mission end
```

Reference outputs:

- final altitude: **961.584818023 m**;
- final speed: **100.787989078 m/s**;
- final mass: **95.000000000 kg**;
- active vehicles: 1.

The report records dataset `nereid-k.atmosphere` version `1.0` and its exact content SHA-256.

## Declarative H optimization

The YAML design variable points at:

```text
/vehicles/lander/phases/0/dynamics/ideal_rocket/mass_flow
```

SLSQP maximizes final mass subject to final altitude >= 600 m.

Reference result:

- success: true;
- mass flow: **0.6336000390 kg/s**;
- final altitude: **600.000001126 m**;
- final mass: **96.831999805 kg**;
- max normalized constraint violation: **0.0**;
- trajectory evaluations: **5**.

## Declarative staging reference

`missions/nereid_l_staging.yaml` starts one 6-DOF 100 kg stack. At exactly 2 s a mission-file `rigid_separation` event invokes the verified I separation law and produces:

- retained `upper`: 60 kg;
- detached `booster`: 40 kg;
- explicit daughter COM offsets satisfying the parent COM relation;
- specified relative separation velocity;
- angular-momentum conservation enabled.

Both daughters propagate independently to 4 s. Final active vehicle count is **2**.

## L acceptance invariants

- YAML uses safe parsing only; mission files cannot execute Python;
- unknown core keys are errors;
- mission format version is explicit and mandatory;
- body/environment/solver/template/output references are checked before propagation;
- design-variable and dispersion JSON pointers must resolve against the original mission document;
- mission SHA-256 is deterministic for normalized equivalent input content;
- declared K dataset ID/version must match native file provenance;
- phase changes use the existing I universe event scheduler;
- 3↔6-DOF transitions use the existing schema mapping layer;
- rigid staging uses the existing I conservation-checked separation model;
- optimization wraps the trusted simulator rather than changing dynamics equations;
- Monte Carlo sampling is deterministic for a fixed seed;
- all A–K regression behavior remains unchanged.
