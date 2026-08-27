# Verification Record — UniFlight Milestone K

UniFlight **0.11.0** preserves all 98 A–J verification cases and adds 15 engineering-data cases.

## New K cases (99–113)

99. arbitrary three-dimensional multilinear interpolation of an exactly linear field;
100. per-axis clamp / extrapolate / error policies plus periodic coordinate wrapping;
101. soft/hard validity envelopes and absolute/relative uncertainty propagation;
102. native NPZ round-trip, deterministic checksum, and explicit-version catalog semantics;
103. N-D aerodynamic database adapter over Mach/alpha/Reynolds coordinates;
104. tabulated atmosphere deriving density and transport properties from a gas mixture;
105. tabulated aerothermal heat-flux model;
106. tabulated propulsion map and 6-DOF engine integration;
107. temperature-dependent material database driving an ablating lumped TPS;
108. radial gravity table, gravity gradient, and `PlanetaryEnvironment` gravity override;
109. Cartesian vector gravity field and numerical Jacobian;
110. periodic latitude/longitude terrain with slope-aware surface normal;
111. deterministic content fingerprints and reproducible catalog inventory;
112. long-form CSV round-trip plus incomplete-grid rejection;
113. end-to-end table-driven atmosphere + gravity + aero + propulsion 6-DOF flight.

## Bounded regression execution

```bash
# A–E physics
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src pytest -q \
  tests/test_6dof_atmospheric_flight.py tests/test_6dof_dynamics_and_tvc.py \
  tests/test_6dof_flow_and_aero.py tests/test_atmosphere.py \
  tests/test_atmospheric_ascent.py tests/test_attitude.py tests/test_edl.py \
  tests/test_entry_reentry.py tests/test_events.py tests/test_flow_aero_propulsion.py \
  tests/test_gravity.py tests/test_rocket.py tests/test_state_and_frames.py
# 47/47 passed

# F/F.1/G GNC + robustness
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src pytest -q \
  tests/test_f1_performance.py tests/test_g_terminal_robustness.py \
  tests/test_gnc_robustness.py
# 20/20 passed

# H + I + J
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src pytest -q \
  tests/test_h_optimization.py tests/test_i_multivehicle.py tests/test_j_subsystems.py
# 31/31 passed

# K engineering data
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src pytest -q tests/test_k_engineering_data.py
# 15/15 passed
```

Total: **113/113 passed in bounded groups**.

The H process-parallel test may emit Python's standard `fork()` deprecation warning in a multi-threaded Linux parent; the test itself passes.

## K reference output

`examples/engineering_data_system.py --output reports/k_reference.json` generated six synthetic, checksummed Nereid-K datasets and reloaded them through the catalog.

Coupled table-driven 6-DOF flight:

- duration: 6.0 s;
- final altitude: ~141.181299 m;
- final speed: ~45.866155 m/s;
- final mass: ~96.880000 kg;
- quaternion norm: 1.0.

Reference data demonstrations:

- six explicit dataset/version/hash records are present in the catalog;
- 900 K TPS material interpolation returns finite thermophysical properties;
- terrain lookup returns a slope-aware normal;
- the reference aerodynamic query at Mach 4 / alpha 9 deg is deliberately outside the declared recommended validity envelope and is flagged without suppressing the interpolation;
- NPZ and CSV versions of all six synthetic datasets are included under `reports/k_datasets/`.

## K acceptance invariants

- table coordinates and output data are finite and shapes match the complete Cartesian grid;
- interpolation-domain behavior is explicit per axis;
- validity is recorded separately from interpolation/extrapolation behavior;
- periodic axes wrap deterministically;
- unit strings are metadata only; kernel numeric values remain coherent SI;
- uncertainty annotations never silently perturb deterministic simulation values;
- catalog lookup never silently selects among multiple versions;
- native NPZ reload verifies deterministic content SHA-256;
- CSV ingestion rejects duplicate or missing Cartesian grid points;
- domain adapters reuse existing dynamics interfaces rather than bypassing the force/moment/state ownership rules;
- all A–J regression behavior remains unchanged.
