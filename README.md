# UniFlight — Milestone F

Milestone F extends the Milestone E surface-to-space-to-surface physics kernel with **sampled-data guidance, navigation and control plus reproducible mission robustness analysis**. The central architectural rule is that state estimation and control execute only at explicit chronological sample times; commands are zero-order held while the adaptive continuous-time plant integrator advances between samples.

## Implemented in F

- `gnc_edl_6dof_schema()` with actuator states for throttle and two-axis TVC
- deterministic noisy position/velocity sensing
- noisy quaternion/angular-rate sensing with unit-quaternion preservation
- radar-altimeter sensor interface
- generic Extended Kalman Filter with Joseph-form covariance update
- 6-state translational navigation EKF using arbitrary-body gravity
- 3-D vector terminal-landing guidance with local-gravity feedforward
- quaternion PD attitude controller
- sampled-data GNC command bus
- zero-order-held commands between GNC updates
- first-order actuators with position and slew-rate limits
- bounded commanded body torque
- GNC-controlled gimballed rocket engine
- state-limit abort rules and terminal abort events
- deterministic seeded normal/uniform dispersions
- serial Monte Carlo campaign runner
- success-rate and p05/median/p95 mission statistics
- JSON campaign-report export from the reference example
- full regression coverage for Milestones A-E

## Why sampled-data GNC is outside the ODE RHS

An adaptive ODE solver may evaluate its RHS at trial times that are not monotonically increasing. A mutable estimator/controller called from inside that RHS could therefore process measurements out of chronological order. Milestone F avoids that failure mode by integrating the continuous plant to the next GNC sample boundary, performing exactly one sensor/estimator/controller update, holding the resulting command, and repeating.

## Important fidelity note

Milestone F establishes the **GNC and robustness software contracts**. The included sensors, EKF process model, landing guidance, PD attitude controller, actuator models, and dispersion set are engineering reference implementations. They are not flight-qualified navigation filters, fault-tolerant flight software, or a tuned operational landing controller.

## Dependencies

- Python >= 3.11
- NumPy >= 2.0
- SciPy >= 1.13
- pytest >= 8 for development tests

## Install

```bash
python -m pip install -e .
```

Offline, when the build toolchain is already installed:

```bash
python -m pip install -e . --no-build-isolation
```

## Verification

For a deterministic local test environment:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src pytest -q
```

The complete Milestone A-F suite contains **59 tests**.

## Sandbox-friendly acceptance run

The bounded acceptance profile exercises all F-specific unit tests and one full noisy closed-loop landing at the intended 0.5 s GNC cadence:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src pytest -q tests/test_gnc_robustness.py
PYTHONPATH=src python examples/gnc_monte_carlo.py --nominal-only --sample-period 0.5
```

The checked nominal sandbox result is approximately 125.05 s to touchdown, 0.038 m lateral error, 0.040 m/s total touchdown speed, and 370.76 kg final mass. Statistical acceptance is intentionally left to the larger local campaign.

## Full local campaign

The exact same runner scales to larger campaigns:

```bash
PYTHONPATH=src python examples/gnc_monte_carlo.py \
  --cases 1000 \
  --sample-period 0.5 \
  --seed 20260827 \
  --output reports/f1000.json
```

See `FULL_SCALE_VALIDATION.md` for the recommended sequence, success criteria, dispersions, and handoff format.

## Reference GNC loop

```text
truth state
    ↓
sensors + deterministic noise
    ↓
EKF navigation estimate
    ↓
3-D landing guidance
    ↓
quaternion attitude control
    ↓
limited actuator commands
    ↓
zero-order-held plant input
    ↓
adaptive 6-DOF integration to next sample time
```

## Intentionally deferred

Milestone F does not yet implement:

- high-fidelity INS/GNSS/star-tracker/radar measurement geometry
- tightly coupled inertial navigation with IMU bias states
- nonlinear MPC / convex powered-descent guidance
- fault detection, isolation and reconfiguration beyond reference abort rules
- concurrent parallel Monte Carlo execution
- automatic worst-case search / importance sampling
- mission trajectory optimization
- variable-dimension concurrent multi-vehicle propagation

These are compatible with the existing interfaces and can be added without changing the trusted physics kernel.
