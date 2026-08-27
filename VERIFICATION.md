# Verification Record — UniFlight Milestone J

UniFlight 0.10.0 preserves all 88 A–I verification cases and adds 10 engineering-subsystem cases.

## New J cases (89–98)

89. engineering-state schema augmentation is composable with arbitrary base schemas;
90. flexible modal equation and modal-energy calculation;
91. torque-to-mode participation and flexible-station attitude/rate sensing (CSI hook);
92. transverse linear slosh equation plus reaction force/moment signs;
93. second-order engine transient and second-order servo acceleration/position limits;
94. deterministic gain/bias/stuck/dropout fault schedule semantics;
95. stateful landing-gear compression dynamics and contact wrench;
96. wrench-level fault scaling;
97. subsystem-bundle composition preserves derivative/wrench/mass-flow ordering;
98. coupled rigid + engine transient + TVC + flex + slosh + mass-depletion integration.

## Bounded regression execution

The full suite is split to stay inside restricted wall-clock environments.

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
```

Total: **98/98 passed in bounded groups**.

The H process-parallel ordering test may emit Python 3.13's standard `fork()` deprecation warning on Linux when the parent process is multi-threaded; the test itself passes.

## J reference output

`examples/engineering_subsystems.py --output reports/j_reference.json` produced the deterministic Nereid-J reference:

Coupled powered subsystem case:

- duration: 6.0 s;
- final altitude: ~118.158 m;
- final speed: ~4.315 m/s;
- final mass: ~493.602 kg;
- scheduled engine-command degradation reaches ~0.700 normalized power;
- max pitch-gimbal magnitude: ~0.04022 rad;
- max flexible modal displacement: ~0.01543 m;
- max slosh displacement: ~0.00893 m.

Dynamic gear-drop case:

- max strut compression: ~0.09248 m;
- rebound is present by 1.0 s;
- the stateful contact load remains finite and integration completes successfully.

## J acceptance invariants

- engineering states can augment core, entry, EDL, GNC, or future schemas without replacing them;
- each subsystem state field still has one derivative writer;
- subsystem wrenches feed the existing rigid-body force/moment accumulator;
- engine transient output is physically clipped to [0,1] at the propulsion interface;
- second-order actuator output is physically clipped to declared hard stops;
- slosh excitation explicitly selects non-slosh forcing models, preventing a hidden algebraic reaction loop;
- flexible-station sensor effects are measurement-level couplings, not hidden changes to rigid truth state;
- fault schedules are deterministic functions of simulation time;
- dynamic gear compression is a continuous state, not an implicit mutation inside contact evaluation;
- all A–I regression behavior remains unchanged.
