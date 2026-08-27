# UniFlight — Milestone J Engineering Subsystem Dynamics

UniFlight **0.10.0** adds flexible-body modes, propellant slosh, second-order engine/actuator dynamics, control-structure-interaction hooks, stateful landing gear, deterministic fault injection, and explicit subsystem-coupling adapters on top of the complete A–I celestial-body-agnostic flight stack.

Milestone J is the POST2-parity step that moves the vehicle model beyond a perfectly rigid vehicle with ideal subsystem response.

## New in J

- composable `augment_engineering_schema()` for core/entry/EDL/GNC states
- `ModalFlexibleBody` linear structural modes
- torque-to-modal-force participation matrices
- flexible point/station translation and rotation maps
- flexible attitude/rate sensor for low-order control-structure interaction
- `LinearSloshSubsystem` with reaction force/moment feedback
- explicit selected-wrench -> body-specific-force slosh excitation
- second-order normalized `EngineTransient`
- second-order position/rate/acceleration-limited servo actuator
- stateful `DynamicLandingGear` strut compression/rebound
- deterministic gain, bias, stuck, and dropout fault windows
- wrench-level fault injection
- `SubsystemBundle` composition helper
- coupled Nereid-J engineering-subsystem reference simulation
- **98/98 total verification tests pass in bounded groups**

Package version: **0.10.0**.

## Install

```bash
python -m pip install -e . --no-build-isolation
```

Dependencies:

- Python >= 3.11
- NumPy >= 2.0
- SciPy >= 1.13
- pytest >= 8 for development

## Run the J reference cases

```bash
PYTHONPATH=src python examples/engineering_subsystems.py \
  --output reports/j_reference.json
```

The example contains a coupled engine/TVC/flex/slosh/fault case and a separate dynamic landing-gear drop/rebound case on fictional body Nereid-J.

## Minimal engineering-subsystem pattern

```python
schema = augment_engineering_schema(
    core_6dof_schema(),
    flex_modes=2,
    slosh_modes=1,
)

engine_transient = EngineTransient(command=1.0)
engine = GimballedRocketEngine(
    environment,
    mass_properties,
    exhaust_velocity=2400.0,
    mdot_exhaust=1.2,
    throttle=engine_transient,
)

flex = ModalFlexibleBody([3.0, 7.5], [0.02, 0.03])
slosh = LinearSloshSubsystem(...)

rigid = RigidBody6DOFDynamics(
    mass_properties,
    gravity=body.gravity,
    wrench_models=(engine, slosh),
)
```

## Documents

- `MILESTONE_J.md` — subsystem mathematics, coupling semantics, and boundaries
- `MILESTONE_I.md` — multi-vehicle/multi-DOF runtime
- `MILESTONE_H.md` — trajectory targeting and optimization
- `VERIFICATION.md` — complete J regression/acceptance record
- `reports/j_reference.json` — deterministic J reference output

## Project scope

The project target remains functional/architectural proximity to NASA POST2 while explicitly **not claiming** real-mission validation, flight heritage, or certification/independent-IV&V pedigree.

The next roadmap item is **Milestone K — General Engineering Data System**: arbitrary N-dimensional engineering tables, interpolation/extrapolation policies, validity envelopes, uncertainty metadata, aero/aerothermal/propulsion/material datasets, gravity/terrain datasets, and provenance-aware model lookup.
