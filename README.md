# UniFlight — Milestone K General Engineering Data System

UniFlight **0.11.0** adds the shared engineering-data infrastructure needed for POST2-class research workflows on top of the full A–J celestial-body-agnostic dynamics stack.

Milestone K turns aerodynamic, aerothermal, propulsion, material, atmosphere, gravity, and terrain data into versioned first-class datasets rather than subsystem-specific hard-coded lookup logic.

## New in K

- arbitrary regular **N-dimensional** `EngineeringTable`
- linear / nearest interpolation
- per-axis `error`, `clamp`, or `extrapolate` policy
- periodic/wrapped axes such as longitude
- explicit validity envelopes, separate from interpolation domain
- coherent-SI axis/output unit metadata
- output uncertainty annotations
- dataset provenance and source metadata
- deterministic table-content SHA-256
- checksummed native NPZ persistence
- human-readable complete-grid long-form CSV import/export
- provenance-aware `EngineeringDataCatalog` with explicit version resolution
- finite-difference table partial derivatives
- table-driven 6-DOF aerodynamic coefficients
- tabulated aerothermal heat flux
- tabulated rocket thrust/mass-flow performance
- table-driven TPS material properties
- tabulated atmospheres
- radial and Cartesian gravity-field adapters
- periodic latitude/longitude terrain with slope-aware normals
- `PlanetaryEnvironment.gravity_model` override
- coupled Nereid-K table-driven reference flight
- **113/113 total verification tests pass in bounded groups**

Package version: **0.11.0**.

## Install

```bash
python -m pip install -e . --no-build-isolation
```

Dependencies:

- Python >= 3.11
- NumPy >= 2.0
- SciPy >= 1.13
- pytest >= 8 for development

## Run the K reference case

```bash
PYTHONPATH=src python examples/engineering_data_system.py \
  --output reports/k_reference.json
```

The example generates six synthetic engineering databases under `reports/k_datasets/`, reloads them through a versioned catalog, and uses the atmosphere, gravity, aero, and propulsion datasets in one coupled 6-DOF flight.

## Minimal N-D table

```python
from uniflight import *
import numpy as np

mach = np.array([0.0, 1.0, 2.0])
alpha = np.deg2rad([-10.0, 0.0, 10.0])
M, A = np.meshgrid(mach, alpha, indexing="ij")

table = EngineeringTable(
    axes=(
        AxisMetadata("mach", mach, "1", extrapolation="clamp"),
        AxisMetadata("alpha", alpha, "rad", extrapolation="clamp"),
    ),
    outputs={
        "cd": 0.4 + 0.1*M + A*A,
        "cl": 2.0*A,
    },
    provenance=DataProvenance("vehicle-x.aero", "2026.1"),
)

q = table.query({"mach": 1.5, "alpha": np.deg2rad(4.0)})
print(q.values)
```

## Native dataset storage

```python
table.to_npz("vehicle-x-aero.npz")
loaded = EngineeringTable.from_npz("vehicle-x-aero.npz")

catalog = EngineeringDataCatalog()
catalog.register(loaded)
model_data = catalog.resolve("vehicle-x.aero", "2026.1")
```

If multiple versions are present, `resolve()` requires an explicit version rather than silently choosing one.

## Documents

- `MILESTONE_K.md` — data mathematics, policies, adapters, and boundaries
- `MILESTONE_J.md` — engineering subsystem dynamics
- `MILESTONE_I.md` — multi-vehicle / multi-DOF runtime
- `MILESTONE_H.md` — targeting and optimization
- `VERIFICATION.md` — complete K verification record
- `reports/k_reference.json` — deterministic K reference result
- `reports/k_datasets/` — synthetic NPZ + CSV reference datasets

## Project scope

The project target remains functional/architectural proximity to NASA POST2 while explicitly **not claiming** real-mission validation, flight heritage, or certification/independent-IV&V pedigree.

The next roadmap item is **Milestone L — Mission Definition Language**: declarative bodies, vehicles, phases, events, datasets, optimization variables, constraints, Monte Carlo dispersions, solver settings, and output requests.
