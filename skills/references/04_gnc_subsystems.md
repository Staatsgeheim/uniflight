# GNC and engineering subsystems

## Sampled-data closed loop
Use `SampledDataClosedLoopEngine`. Do not execute stateful estimation/control updates at arbitrary adaptive RHS evaluation times.

## Sensors
- `PositionVelocitySensor`
- `RadarAltimeterSensor`
- `AttitudeRateSensor`
- flexible station sensor perturbation via `FlexibleAttitudeRateSensor`

Specify noise/bias and deterministic RNG seeds.

## Estimation
- `ExtendedKalmanFilter`
- `KinematicProcessModel`
- `TranslationalNavigationEKF`
- `numerical_jacobian`

EKF covariance update uses Joseph form. Preserve covariance symmetry/PSD diagnostics when modifying.

## Guidance/control
- `VectorLandingGuidance`
- `VerticalDescentThrottle`
- `QuaternionPDController`
- `AdaptiveThrustScaleEstimator`
- `LandingGNCController`

## Actuators
- `GNCCommandBus`
- first/second-order limited state actuators
- `CommandedBodyTorque`
- TVC through propulsion/servo models

Respect position/rate/acceleration limits.

## Aborts/faults
- `LimitAbortRule`, `AbortManager`
- `FaultMode`, `FaultWindow`, `ScalarFaultSchedule`
- faulted scalar/wrench providers

Fault injection should be explicit and logged. Do not make a fault indistinguishable from nominal uncertainty.

## Flexibility
`ModalFlexibleBody`, `TorqueToModalForce`, `FlexiblePointKinematics`.
These are low-order modal models, not FEM.

## Slosh
`LinearSloshSubsystem` provides engineering-order slosh states/reaction loads. Keep excitation/reaction paths explicit to avoid algebraic loops.

## Landing gear
`DynamicGearLeg`, `DynamicLandingGear` model stateful strut dynamics. `LandingGearContact` provides compliant contact. This is not a general rigid-contact complementarity solver.

## Subsystem composition
Use `SubsystemBundle` and augment the state schema. A vehicle should carry only subsystem states it actually uses.
