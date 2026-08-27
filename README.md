# UniFlight — Milestone E

Milestone E extends the Milestone D planet-agnostic entry/re-entry kernel through **entry, descent, powered terminal descent, touchdown, and first-contact dynamics**. The milestone keeps mission phase discrete while the physical vehicle state remains numeric, and it preserves the one-owner-per-state-derivative rule introduced in Milestone A.

## Implemented in E

- `edl_6dof_schema()` extending the Milestone-D entry state with:
  - parachute deployment fraction
  - landing-gear deployment fraction
- Generic `FirstOrderDeployable`
  - deploy/retract time constants
  - irreversible deployment option
  - callable command support
- 6-DOF `InflatingParachute`
  - deployment-dependent effective area
  - atmosphere-relative drag
  - body attachment point and resulting moment about CG
- `RadialTerrain`
  - arbitrary spherical body
  - constant or callable radial elevation
  - AGL, surface normal, surface point, and local rotating-surface velocity
- `LandingGearContact`
  - multiple gear legs
  - stowed/deployed foot locations
  - spring-damper normal contact
  - regularized Coulomb friction
  - 6-DOF force and moment output
- `VerticalDescentThrottle`
  - local-gravity feedforward
  - altitude-dependent descent-speed schedule
  - arbitrary planetary atmosphere and gravity
- `JettisonJump`
  - hybrid state jump for mass removal
  - optional state resets, e.g. parachute deployment -> 0
- Momentum-conserving `separate_two_body()` utility
  - produces retained and detached daughter initial states
  - supports a prescribed relative separation velocity
- `HybridModeEngine`
  - phase-specific RHS and event sets
  - discrete mode remains outside continuous `X`
  - transition by terminal event name
  - concatenated state/event/mode history
- Full regression coverage for Milestones A-D
- End-to-end fictional-world EDL example:
  - parachute inflation and descent
  - parachute jettison
  - powered terminal descent
  - gear deployment
  - touchdown
  - spring/damper first-contact compression

## Important fidelity note

Milestone E establishes the **EDL software contracts**. Its parachute and contact models are engineering reference closures, not substitutes for validated canopy CFD/FSI, line dynamics, flexible landing-gear models, soil mechanics, crushable structures, or high-fidelity multibody impact simulation.

`separate_two_body()` establishes momentum-consistent daughter initialization. Milestone E does **not** yet provide a single solver that automatically changes the dimension of the global state and concurrently integrates an arbitrary number of newly created daughter vehicles; each daughter can already be initialized and integrated independently with the existing kernel.

## Numerical dependencies

- Python >= 3.11
- NumPy >= 2.0
- SciPy >= 1.13
- pytest >= 8 for development tests

## Install

```bash
python -m pip install -e .
```

For an offline environment with the build toolchain already installed:

```bash
python -m pip install -e . --no-build-isolation
```

## Run verification

```bash
PYTHONPATH=src pytest -q
```

The project currently passes **47 tests**.

## Run the full EDL example

```bash
PYTHONPATH=src python examples/full_edl.py
```

The reference example uses the fictional body **Nereid-E**. The checked configuration begins at 3 km AGL with a 120 m/s downward speed and then executes:

1. parachute inflation and descent
2. parachute jettison / powered-descent transition at 500 m
3. closed-loop powered vertical descent
4. landing-gear deployment
5. first foot contact
6. engine-off landing-gear compression to the first zero-radial-speed point

Representative checked output:

- powered-descent event: ~185.76 s
- touchdown event: ~301.20 s
- first compression stop: ~301.27 s
- final CG altitude: ~1.975 m
- final radial speed: ~0 m/s at maximum first compression
- final vehicle mass: ~364.23 kg
- landing gear deployment: 1.0
- quaternion norm: 1.0 to displayed precision
- gear contact active at termination

The example is intentionally a reference architecture case, not a tuned operational landing guidance law.

## Core Milestone E assembly

```python
chute_deploy = FirstOrderDeployable(
    "parachute_deployment", command=1.0, deploy_time_constant=1.5
)
parachute = InflatingParachute(
    environment, mass_properties,
    maximum_area=80.0,
    drag_coefficient=1.5,
    deployment=chute_deploy,
)

gear_deploy = FirstOrderDeployable(
    "gear_deployment", command=1.0, deploy_time_constant=0.5
)
landing_gear = LandingGearContact(
    terrain, mass_properties, legs=(gear_leg,)
)

guidance = VerticalDescentThrottle(
    environment, terrain,
    exhaust_velocity=2000.0,
    mdot_exhaust=3.0,
)
engine = GimballedRocketEngine(
    environment, mass_properties,
    exhaust_velocity=2000.0,
    mdot_exhaust=3.0,
    base_direction_b=np.array([1.0, 0.0, 0.0]),
    throttle=guidance,
)

modes = {
    "parachute": ModeDefinition("parachute", parachute_rhs, (to_power_event,)),
    "powered": ModeDefinition("powered", powered_rhs, (touchdown_event,)),
    "contact": ModeDefinition("contact", contact_rhs, (compression_stop_event,)),
}
mission = HybridModeEngine(modes, transition_function, integrator)
```

## State and mode ownership

Continuous physical state remains in the packed schema:

```text
[position, velocity, attitude, angular_rate, mass,
 tps_temperature, heat_load, tps_mass,
 parachute_deployment, gear_deployment]
```

Mission mode is intentionally **not** stored as a floating-point state. `HybridModeEngine` owns discrete phase selection and records mode intervals separately. This preserves clear semantics for ODE integration and event root finding.

## Separation convention

For two daughters with masses `m1` and `m2`, parent velocity `V`, and prescribed relative separation velocity

```text
dv = v_detached - v_retained
```

Milestone E initializes

```text
v_retained = V - (m2 / (m1 + m2)) * dv
v_detached = V + (m1 / (m1 + m2)) * dv
```

which conserves parent linear momentum exactly up to floating-point roundoff.

## Contact convention

For a deployed foot below terrain by penetration `delta` and normal relative speed `v_n` (positive outward), the reference normal-force closure is

```text
F_n = max(0, k * delta - c * v_n)
```

with regularized tangential friction bounded by `mu * F_n`. This is a penalty formulation; rigid complementarity contact remains a later high-fidelity option.

## Intentionally deferred

Milestone E does **not** yet implement:

- flexible/parachute canopy multibody or FSI models
- automatic variable-dimension concurrent multi-vehicle integration
- rigid complementarity/impulse contact and wheel/leg kinematic constraints
- deformable terrain / regolith / soil mechanics
- closed-loop attitude guidance for arbitrary 3-D powered landing
- state estimation and sensor models
- Monte Carlo dispersion runner
- trajectory optimization / optimal control
- finite-rate chemistry and higher-fidelity TPS models deferred from D

Those remain compatible with the architecture and are natural later milestones.
