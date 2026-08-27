# Engineering data system

## EngineeringTable
Represents a regular N-D grid with metadata and policies.

Configure:
- `AxisMetadata`
- `OutputMetadata`
- `InterpolationMethod`
- `ExtrapolationPolicy`
- `ValidityEnvelope`
- `UncertaintyMetadata`
- `DataProvenance`

## Policies
Per-axis extrapolation may error, clamp, or extrapolate. Engineering validity is separate from interpolation behavior. A mathematically computable value can still be outside the model's declared validity.

## Persistence
Use native checksummed NPZ for reproducible runtime data. Long-form CSV exists for human editing/interchange. Validate complete grids and reject duplicates.

## Catalog
`EngineeringDataCatalog` is version explicit. Never silently pick "latest" when multiple versions exist.

## Domain adapters
`data_models.py` includes table-backed:
- aero coefficients;
- aerothermal models;
- rocket performance/engine;
- materials/TPS;
- radial/cartesian gravity;
- spherical terrain;
- atmosphere.

## Data QA
Before accepting a dataset:
- verify monotonic axes;
- verify units;
- plot slices;
- check boundary behavior;
- check interpolation at grid nodes;
- check periodic axes;
- check checksum/provenance;
- test validity-envelope reporting.
