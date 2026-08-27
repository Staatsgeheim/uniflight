# UniFlight — Milestone A

Reference Python implementation of **Milestone A (Kernel)** from the software architecture specification for the celestial-body-agnostic hybrid flight-dynamics framework.

Implemented in this milestone:

- immutable state schema/views and deterministic packing;
- explicit frame graph and quaternion/rotation utilities;
- SI unit metadata and basic state-domain validation;
- composable RHS/derivative assembly with single-owner state derivatives;
- inertial translational kinematics;
- quaternion attitude kinematics;
- spherical point-mass gravity using arbitrary `mu = G M`;
- ideal constant-exhaust-velocity rocket model for verification;
- SciPy adaptive ODE adapter with dense output;
- guard/event definitions, root isolation, priorities, terminal events, and jump-map plumbing;
- invariant hooks and simulation result/event records;
- verification cases 001–004 and 011–012.

## Install and test

```bash
python -m pip install -e .
pytest
```

## Run example

```bash
python examples/suborbital_point_mass.py
```

All physics-facing quantities are SI internally. Directional state fields carry frame identifiers in their schema metadata. The kernel contains no Earth constants.
