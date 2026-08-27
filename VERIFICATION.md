# Verification Record — UniFlight Milestone H

Milestone H preserves the 67 A–G/F.1 verification cases and adds 10 trajectory-design cases.

## New H cases (68–77)

68. design-space scaling round trip and physical-name mapping
69. bound-aware finite-difference Jacobian
70. generic bounded nonlinear two-variable targeter
71. constrained black-box equality optimization
72. multiple-shooting continuity defects
73. process-parallel candidate evaluation preserves ordering/results
74. explicit-event radial-ascent apogee targeter
75. simulation-based minimum-propellant radial-ascent optimization
76. exact-value trajectory-evaluation cache suppresses duplicate simulation calls
77. nonlinear lower-bound inequality constraint

## H bounded execution

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src pytest -q tests/test_h_optimization.py
# 10/10 passed
```

## H reference numerical result

Nereid-H target apogee: 20,000 m.

Single-variable event targeter at fixed `mdot=5 kg/s`:

- burn time: ~5.918916 s
- normalized residual: < 1e-12

Constrained minimum-propellant optimization:

- optimizer: SLSQP
- mass flow: ~7.9976 kg/s
- burn time: ~3.68857 s
- propellant used: ~29.49975 kg
- apogee: ~20,000.000001 m
- normalized maximum constraint violation: ~4.8e-11

The optimum approaches the upper mass-flow bound because reducing burn duration reduces gravity loss.

## Regression execution strategy

The complete suite is intentionally divisible because atmospheric GNC/re-entry tests can exceed strict sandbox wall-clock limits when invoked in one process.

Sandbox bounded execution completed:

- A–C/non-entry physics: **26/26 passed**
- D/E entry + EDL physics: **21/21 passed**
- F/F.1/G GNC + robustness: **20/20 passed**
- H targeting + optimization: **10/10 passed**

Total: **77/77 passed in bounded groups**.

Recommended reproduction groups:

```bash
# A–E/non-GNC physics
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src pytest -q \
  tests/test_rocket.py tests/test_state_and_frames.py tests/test_attitude.py \
  tests/test_gravity.py tests/test_events.py tests/test_atmosphere.py \
  tests/test_flow_aero_propulsion.py tests/test_atmospheric_ascent.py \
  tests/test_6dof_flow_and_aero.py tests/test_6dof_dynamics_and_tvc.py \
  tests/test_6dof_atmospheric_flight.py tests/test_entry_reentry.py tests/test_edl.py

# F/F.1/G GNC and robustness
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src pytest -q \
  tests/test_gnc_robustness.py tests/test_f1_performance.py \
  tests/test_g_terminal_robustness.py

# H targeting/optimization
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src pytest -q \
  tests/test_h_optimization.py
```

## Acceptance invariants

- optimization never mutates the flight-dynamics kernel
- design variables are physical values with explicit optimizer scaling
- equality and inequality constraints are independent of objective definition
- event targets are exposed through ordinary trajectory metrics
- repeated objective/constraint requests at an identical design point share one cached trajectory evaluation
- finite-difference derivatives respect variable bounds
- derivative-free fallback does not silently relax declared constraints beyond the explicit tolerance
- multiple-shooting defects follow `Phi_i(x_i,p)-x_(i+1)`
- parallel candidate results preserve deterministic input ordering

---

# Milestone I Addendum — UniFlight 0.9.0

Milestone I preserves all 77 A–H cases and adds 11 multi-vehicle/multi-DOF runtime cases.

## New I cases (78–88)

78. 3-DOF -> 6-DOF promotion and exact translational round-trip projection
79. zero-velocity promotion requires explicit attitude
80. concurrent propagation of two vehicles with distinct dynamics/model contexts
81. earliest global event synchronizes a non-triggering vehicle and propagation continues
82. dynamic parent removal + child spawn changes active topology
83. 3-DOF -> 6-DOF runtime schema/configuration switch with stable vehicle ID
84. rigid two-body separation conserves linear and angular momentum
85. rigid-separation universe handler replaces parent with two active 6-DOF daughters
86. simultaneous-event priority suppresses lower-priority guards from a replaced generation
87. 6-DOF -> 3-DOF runtime switch
88. fixed-step RK4 vehicle resynchronization fallback at a global event

## Bounded regression execution

```bash
# A–E physics
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src pytest -q \\
  tests/test_state_and_frames.py tests/test_gravity.py tests/test_rocket.py \\
  tests/test_events.py tests/test_attitude.py tests/test_atmosphere.py \\
  tests/test_flow_aero_propulsion.py tests/test_atmospheric_ascent.py \\
  tests/test_6dof_flow_and_aero.py tests/test_6dof_dynamics_and_tvc.py \\
  tests/test_6dof_atmospheric_flight.py tests/test_entry_reentry.py tests/test_edl.py
# 47/47 passed

# F/F.1/G GNC and robustness
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src pytest -q \\
  tests/test_gnc_robustness.py tests/test_f1_performance.py tests/test_g_terminal_robustness.py
# 20/20 passed

# H + I
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src pytest -q \\
  tests/test_h_optimization.py tests/test_i_multivehicle.py
# 21/21 passed
```

Total: **88/88 passed in bounded groups**.

## I reference output

The Nereid-I reference mission produced:

- `stage_separation` at exactly 5.0 s;
- active set after separation: `booster`, `upper`;
- `upper_demote` at exactly 8.0 s;
- upper final mode: `upper-coast-3dof`;
- booster final mode: `booster-6dof`;
- upper final altitude: ~5775.84 m;
- booster final altitude: ~5771.35 m;
- both propagated through one universe to 20 s.

## Acceptance invariants

- active vehicle IDs are unique;
- every universe event has one global timestamp;
- all vehicles are synchronized to that time before topology mutation;
- trajectory segments retain their own schema and DOF metadata;
- vehicle-local RHS/model closures are never merged into a universal fixed state vector;
- upsert replacement begins a new configuration generation;
- old-generation simultaneous guards do not execute against a replacement generation;
- daughter masses sum to parent mass;
- rigid separation enforces daughter COM consistency;
- reference rigid separation conserves linear and angular momentum to numerical precision;
- both adaptive dense-output and fixed-step no-dense-output vehicles can participate in the same event-synchronization semantics.
