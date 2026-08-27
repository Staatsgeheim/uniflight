# Milestone K — General Engineering Data System

UniFlight **0.11.0** introduces a provenance-aware engineering-data layer that can supply the numerical models used by the existing A–J dynamics stack. The intent is POST2-class *model-data plumbing*: arbitrary regular N-dimensional databases can be supplied without writing one-off interpolation code for every subsystem.

This milestone does **not** claim that the bundled synthetic tables are validated engineering data. It establishes the data contract through which validated or proprietary datasets can later be supplied.

## 1. Core table object

`EngineeringTable` represents a complete rectilinear Cartesian grid

\[
\mathcal D = A_1\times A_2\times\cdots\times A_N
\]

with arbitrary dimensionality \(N\ge1\), and one or more scalar output fields

\[
y_j = f_j(x_1,\ldots,x_N).
\]

Every axis has:

- a unique symbolic name;
- a strictly increasing coordinate vector;
- coherent-SI unit metadata;
- a description;
- an extrapolation policy;
- an optional period for wrapped coordinates such as longitude.

Every output has:

- a unique symbolic name;
- coherent-SI unit metadata;
- a description;
- optional uncertainty metadata.

The table supports linear and nearest-neighbor regular-grid interpolation through SciPy's `RegularGridInterpolator`.

## 2. Extrapolation policy

Extrapolation is declared independently per axis:

- `ERROR`: reject any out-of-domain query;
- `CLAMP`: clamp the effective coordinate to the nearest table boundary;
- `EXTRAPOLATE`: pass the out-of-domain coordinate to the interpolator.

The query result records:

- original coordinates;
- effective coordinates;
- whether the raw query was inside the table grid;
- which axes were adjusted;
- which axes were extrapolated.

This prevents a model from silently clipping or extrapolating without telemetry.

## 3. Periodic axes

`AxisMetadata.period` wraps coordinates before domain processing. A longitude axis can therefore use

\[
\lambda\mapsto \lambda_0 + ((\lambda-\lambda_0)\bmod 2\pi).
\]

The reference terrain table uses this facility to make longitude queries seam-safe.

## 4. Validity envelopes

Interpolation domain and engineering validity are distinct concepts.

A table may physically contain data over a broad rectangular domain but only be recommended over a narrower envelope:

\[
\mathcal V\subseteq\mathcal D.
\]

`ValidityEnvelope` contains independent lower/upper bounds on selected axes. Policy is either:

- `FLAG`: continue evaluation but return explicit validity violations;
- `ERROR`: reject the query.

For example, the reference aerodynamic table contains Mach values through 4 but declares a recommended validity limit of Mach 3.5.

## 5. Uncertainty annotations

An output can declare `UncertaintyMetadata` with absolute and relative one-sigma components. At a query value \(y\), the reported standard uncertainty is

\[
\sigma_y = \sqrt{\sigma_{\rm abs}^2 + (\sigma_{\rm rel}|y|)^2}.
\]

The metadata can additionally name a distribution, confidence level, correlation group, and notes. This is intentionally an annotation layer: Milestone F/F.1/G Monte Carlo remains responsible for sampling policy.

## 6. Provenance and checksums

`DataProvenance` records:

- dataset ID;
- explicit version;
- source;
- authors;
- citation;
- license;
- creation timestamp metadata;
- optional SHA-256 of the original source artifact;
- notes.

Every `EngineeringTable` can compute a deterministic SHA-256 over its manifest and numeric content. Native `.npz` serialization stores that content checksum and verifies it on reload.

## 7. Provenance-aware catalog

`EngineeringDataCatalog` indexes data by

\[
(\text{dataset_id},\text{version}).
\]

The catalog deliberately does **not** silently choose a version. If multiple versions exist, the caller must explicitly request one. Its inventory returns dataset/version/content-hash tuples suitable for simulation provenance records.

## 8. File formats

### Native NPZ

`EngineeringTable.to_npz()` stores:

- all axis arrays;
- all output arrays;
- JSON metadata manifest;
- interpolation/extrapolation policies;
- validity envelope;
- uncertainty annotations;
- provenance;
- content checksum.

`EngineeringTable.from_npz()` can verify that checksum.

### Long-form CSV

`save_long_form_csv()` and `load_long_form_csv()` provide a human-editable interchange format. Each row contains one N-D grid point and its outputs. The loader rejects:

- duplicate grid points;
- incomplete Cartesian products;
- missing required columns.

CSV is intended for exchange/editing; NPZ is the complete provenance-bearing native artifact.

## 9. Domain adapters

### Aerodynamics

`EngineeringTableAeroCoefficients` exposes a generic table as an existing `AeroCoefficientModel6DOF`. Default recognized coordinates include:

\[
M,\alpha,\beta,Re,Kn,q_\infty,V.
\]

Arbitrary extra axes can be supplied by resolver callbacks. Outputs map to

\[
C_D,C_L,C_Y,C_l,C_m,C_n.
\]

### Aerothermal

`TabulatedAerothermalModel` supplies convective/radiative heat flux from N-D tables. Recognized coordinates include altitude, atmospheric state, speed, Mach, Reynolds, Knudsen and dynamic pressure.

### Propulsion

`TabulatedRocketPerformance` maps ambient pressure, throttle, and optional extra operating coordinates to thrust and mass flow. `TabulatedGimballedRocketEngine` wraps that database in the existing 6-DOF wrench/mass-flow interface, preserving mounting-arm and TVC moments.

### Materials/TPS

`TabulatedMaterialProperties` returns arbitrary thermophysical outputs from a material database. `TabulatedMaterialLumpedTPS` demonstrates direct integration into the existing TPS state equations using table-driven specific heat, emissivity, ablation temperature, and effective heat of ablation.

### Atmosphere

`TabulatedAtmosphere` consumes altitude and optional time tables. Temperature and pressure are mandatory. Density, viscosity, speed of sound and mean free path may be tabulated directly or derived from a supplied static `GasMixture`.

### Gravity

Two adapters are supplied:

- `TabulatedRadialGravity`: radial gravity-magnitude database with optional potential and a finite-difference gravity-gradient Jacobian;
- `TabulatedCartesianGravity`: arbitrary Cartesian vector field on x/y/z and optional time axes, suitable for precomputed irregular-body or external gravity fields.

`PlanetaryEnvironment` now accepts an optional `gravity_model` override so these fields can be used by existing environment queries without altering `SphericalBody`.

### Terrain

`TabulatedSphericalTerrain` maps latitude/longitude to elevation over a spherical reference body. Periodic longitude is supported. The adapter can estimate a slope-aware outward normal from table derivatives rather than assuming a purely radial surface.

## 10. Derivatives

`EngineeringTable.derivative()` provides finite-difference partial derivatives while respecting periodic and boundary policies. The initial uses are:

- gravity gradients for navigation filters;
- terrain slopes/normals;
- future table-based sensitivity calculations.

It is not intended as a substitute for analytic or spline derivatives when a high-order model supplies those directly.

## 11. Reference integration

`examples/engineering_data_system.py` builds six synthetic Nereid-K datasets:

1. atmosphere;
2. gravity;
3. Mach/alpha aerodynamics;
4. ambient-pressure/throttle propulsion;
5. TPS material properties;
6. latitude/longitude terrain.

Each table is saved as NPZ and CSV, reloaded into an `EngineeringDataCatalog`, and then used by a coupled 6-DOF powered flight. The reference run completes at 6 s with approximately:

- altitude: **141.181 m**;
- speed: **45.866 m/s**;
- mass: **96.880 kg**;
- quaternion norm: **1.0**.

The example also demonstrates a flagged out-of-validity aerodynamic query and temperature-dependent material lookup.

## 12. Deliberate boundaries

Milestone K is an engineering-data infrastructure milestone, not a high-fidelity dataset release. It does not itself provide:

- validated Earth/Mars/Titan atmosphere databases;
- flight-qualified aero databases;
- CFD/DSMC data generation;
- SPICE kernels;
- high-degree gravity harmonic files;
- global planetary DEM packages;
- proprietary propulsion maps;
- material test databases.

Those can now be loaded or adapted without changing the flight dynamics kernel.

## 13. POST2-parity significance

Before K, several UniFlight models contained their own table/interpolation logic. K centralizes the data contract and establishes reproducible model lookup. That is important for a POST2-class research workflow because trajectory simulation can now be configured around versioned engineering databases rather than code modifications.

The next planned milestone is **L — Mission Definition Language**: a declarative mission/configuration format for bodies, vehicles, model datasets, phases, events, optimization variables, constraints, Monte Carlo dispersions, solver settings, and requested outputs.
