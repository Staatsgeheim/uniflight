# Physics models

## Bodies and gravity
`SphericalBody` defines radius/rotation/body parameters. `PointMassGravity` supplies inverse-square gravity and gravity Jacobian.

If a task needs J2, harmonics, third-body, polyhedral gravity, or relativity, do not fake it with point-mass parameters. Implement a gravity provider/plugin and verify it independently.

## Atmosphere and gas
Built-ins:
- `VacuumAtmosphere`
- `IsothermalHydrostaticAtmosphere`
- `GasSpecies`
- `GasMixture`

Mixture support includes derived thermodynamic/transport properties used by flow calculations.

## Relative flow
Use `compute_flow_state` and `compute_body_flow_state`. Relative velocity must account for atmosphere/body rotation and winds as configured.

## Aerodynamics
3-DOF:
- `ConstantDragCoefficient`
- `MachTableDragCoefficient`
- `ContinuumDrag`

6-DOF:
- `AeroCoefficients`
- `ConstantAeroCoefficients`
- `LinearStabilityAerodynamics`
- `GridAeroCoefficientDatabase`
- `ConstantReferenceGeometry`
- `EllipsoidProjectedGeometry`

Angles follow body convention: +x forward, +y right, +z down. Alpha and beta are derived from body-relative flow.

## Hypersonic/rarefied
- `NewtonianHypersonicCoefficients`
- `MachBlendedAeroCoefficients`
- `FreeMolecularAerodynamics6DOF`
- `RegimeBlendedAerodynamics6DOF`

Regime blending uses Knudsen number with smooth blending. Do not abruptly switch models unless intentionally testing discontinuity behavior.

## Chemistry
`FrozenChemistry` and `ThresholdDissociationCorrection` are engineering corrections, not a general finite-rate reacting-flow solver.

## Propulsion
- `RocketEngine`
- `GimballedRocketEngine`
- `IdealRocket`
- `EngineTransient`
- tabulated propulsion adapters in `data_models.py`

Pressure thrust and mass flow should remain consistent. Couple mass loss through the mass-flow path rather than independently decrementing mass.

## Mass properties
Use constant or affine mass properties. For changing inertia/COM, provide a model rather than freezing initial inertia.

## Numerical checks
For new physics:
- recover a known limiting case;
- test sign/frame convention;
- test units;
- test zero-input behavior;
- test symmetry;
- test conservation where applicable.
