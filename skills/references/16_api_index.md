# Public API index

This index is generated from UniFlight 1.0.4 source-level classes and public top-level functions. Inspect live source/signatures before coding against a different version.

## `aborts.py`
Classes: `LimitAbortRule`, `AbortManager`

## `actuators.py`
Classes: `GNCCommandBus`, `StateFieldProvider`, `BusScalarProvider`, `FirstOrderLimitedStateActuator`, `CommandedBodyTorque`, `SecondOrderLimitedStateActuator`

## `aerodynamics.py`
Classes: `DragCoefficientModel`, `ConstantDragCoefficient`, `MachTableDragCoefficient`, `AeroEvaluation`, `ContinuumDrag`, `AeroCoefficients`, `AeroCoefficientModel6DOF`, `ConstantAeroCoefficients`, `LinearStabilityAerodynamics`, `GridAeroCoefficientDatabase`, `GeometryEvaluation`, `GeometryModel`, `ConstantReferenceGeometry`, `EllipsoidProjectedGeometry`, `Aero6DOFEvaluation`, `ContinuumAerodynamics6DOF`

## `analysis.py`
Classes: `AnalysisCase`, `CampaignExecution`, `MissionCampaignRunner`, `SweepVariable`, `ParameterSweep`, `MonteCarloVariable`, `MissionMonteCarlo`, `SobolVariable`, `SobolIndices`, `SobolSensitivity`, `OptimizationStart`, `OptimizationBatch`
Functions: `summarize_numeric_metrics`

## `analysis_cli.py`
Functions: `main`

## `atmosphere.py`
Classes: `AtmosphereSample`, `AtmosphereModel`, `VacuumAtmosphere`, `IsothermalHydrostaticAtmosphere`

## `bodies.py`
Classes: `SphericalBody`

## `chemistry.py`
Classes: `ThermochemicalCorrection`, `ChemistryCorrectionModel`, `FrozenChemistry`, `ThresholdDissociationCorrection`

## `closed_loop.py`
Classes: `GNCRecord`, `ClosedLoopResult`, `SampledDataClosedLoopEngine`

## `contact.py`
Classes: `GearLeg`, `LegContactEvaluation`, `LandingGearEvaluation`, `LandingGearContact`

## `control.py`
Classes: `ThrustGuidanceCommand`, `VectorLandingGuidance`, `QuaternionPDController`, `AdaptiveThrustScaleEstimator`, `GNCDecision`, `LandingGNCController`
Functions: `quaternion_align_body_x`

## `data_models.py`
Classes: `EngineeringTableAeroCoefficients`, `TabulatedAerothermalModel`, `RocketPerformanceEvaluation`, `TabulatedRocketPerformance`, `TabulatedRocket6DOFEvaluation`, `TabulatedGimballedRocketEngine`, `MaterialEvaluation`, `TabulatedMaterialProperties`, `TabulatedRadialGravity`, `TabulatedCartesianGravity`, `TabulatedSphericalTerrain`, `TabulatedAtmosphere`, `TabulatedMaterialLumpedTPS`

## `deployables.py`
Classes: `FirstOrderDeployable`, `ParachuteEvaluation`, `InflatingParachute`

## `dof.py`
Classes: `DOFTransition`
Functions: `map_state_fields`, `demote_6dof_to_3dof`, `promote_3dof_to_6dof`

## `dynamics.py`
Classes: `DerivativeModel`, `DynamicsAssembler`, `TranslationalKinematics`, `QuaternionKinematics`, `IdealRocket`, `RigidBody6DOFDynamics`

## `engine_dynamics.py`
Classes: `EngineTransient`

## `engineering_data.py`
Classes: `InterpolationMethod`, `ExtrapolationPolicy`, `ValidityPolicy`, `AxisMetadata`, `UncertaintyMetadata`, `OutputMetadata`, `DataProvenance`, `ValidityBound`, `ValidityEnvelope`, `TableQueryResult`, `EngineeringTable`, `EngineeringDataCatalog`
Functions: `load_long_form_csv`, `save_long_form_csv`

## `environment.py`
Classes: `EnvironmentSample`, `PlanetaryEnvironment`

## `estimation.py`
Classes: `EKFUpdate`, `ExtendedKalmanFilter`, `KinematicProcessModel`, `TranslationalNavigationEKF`
Functions: `numerical_jacobian`

## `events.py`
Classes: `EventAction`, `Event`, `EventOccurrence`

## `faults.py`
Classes: `FaultMode`, `FaultWindow`, `ScalarFaultSchedule`, `FaultedScalarProvider`, `FaultedWrenchModel`

## `flexibility.py`
Classes: `ModalFlexibleBody`, `TorqueToModalForce`, `FlexiblePointKinematics`, `FlexibleAttitudeRateSensor`

## `flow.py`
Classes: `FlowState`, `BodyFlowState`
Functions: `compute_flow_state`, `wind_to_body_matrix`, `compute_body_flow_state`

## `frames.py`
Classes: `Transform`, `FrameGraph`
Functions: `quat_normalize`, `quat_conjugate`, `quat_multiply`, `quat_to_matrix`, `matrix_to_quat`, `body_to_inertial_matrix`, `inertial_to_body_matrix`, `rotate_body_to_inertial`, `rotate_inertial_to_body`

## `gases.py`
Classes: `GasSpecies`, `GasMixture`

## `gear_dynamics.py`
Classes: `DynamicGearLeg`, `DynamicLandingGear`

## `gravity.py`
Classes: `PointMassGravity`

## `guidance.py`
Classes: `DescentGuidanceEvaluation`, `VerticalDescentThrottle`

## `heating.py`
Classes: `AerothermalEvaluation`, `AerothermalModel`, `PowerLawRadiativeHeating`, `SuttonGravesHeating`

## `hpc.py`
Classes: `ExecutionBackend`, `SerialBackend`, `ProcessBackend`, `ExternalExecutorBackend`

## `hypersonics.py`
Classes: `NewtonianHypersonicCoefficients`, `MachBlendedAeroCoefficients`

## `integrators.py`
Classes: `SolverConfig`, `ScipyIVPIntegrator`, `FixedStepRK4Config`, `FixedStepSegmentSolution`, `FixedStepRK4Integrator`

## `invariants.py`
Functions: `specific_energy`, `specific_angular_momentum`, `quaternion_norm_error`

## `mass_properties.py`
Classes: `MassProperties`, `MassPropertiesModel`, `ConstantMassProperties`, `AffineMassProperties`

## `massflow.py`
Classes: `MassFlowSource`, `MassFlowAggregator`

## `mission.py`
Classes: `MissionValidationError`, `MissionCompilationError`, `MissionDocument`, `MissionOptimizationDeclaration`, `MissionDispersionDeclaration`, `MissionRunReport`, `CompiledMission`, `MissionRegistry`, `MissionCompiler`
Functions: `mission_sha256`, `pointer_get`, `pointer_set`, `load_mission`, `validate_mission_dict`, `mission_json_schema`, `save_report`

## `mission_cli.py`
Functions: `main`

## `modes.py`
Classes: `ModeDefinition`, `ModeInterval`, `HybridMissionResult`, `HybridModeEngine`

## `montecarlo.py`
Classes: `Dispersion`, `NormalDispersion`, `UniformDispersion`, `MonteCarloCaseResult`, `MetricStatistics`, `MonteCarloSummary`, `MonteCarloRunner`
Functions: `automatic_worker_count`

## `multibody.py`
Classes: `VehicleConfiguration`, `DOFSwitchHandler`, `RigidChildTemplate`, `RigidSeparationHandler`

## `optimization.py`
Classes: `TrajectoryEvaluator`, `DesignVariable`, `DesignSpace`, `MetricObjective`, `MetricConstraint`, `ProblemEvaluation`, `TrajectoryProblem`, `FiniteDifferenceConfig`, `TargetingSettings`, `TargetingResult`, `TrajectoryTargeter`, `OptimizationSettings`, `OptimizationResult`, `TrajectoryOptimizer`, `MultipleShootingTranscription`, `BatchEvaluationResult`
Functions: `finite_difference_jacobian`, `parallel_batch_evaluate`

## `plugins.py`
Classes: `PluginError`, `PluginDiscoveryError`, `PluginCompatibilityError`, `PluginRequirementError`, `PluginDescriptor`, `CapabilityRegistration`, `PluginRegistrar`, `PluginRequirement`, `LoadedPlugin`, `PluginManager`
Functions: `installed_plugin_summary`

## `propulsion.py`
Classes: `RocketEvaluation`, `RocketEngine`, `Rocket6DOFEvaluation`, `GimballedRocketEngine`

## `rarefied.py`
Classes: `FreeMolecularAerodynamics6DOF`, `RegimeAeroEvaluation`, `RegimeBlendedAerodynamics6DOF`

## `result_store.py`
Classes: `StoredCase`, `SQLiteResultStore`

## `sensors.py`
Classes: `SensorMeasurement`, `PositionVelocitySensor`, `RadarAltimeterSensor`, `AttitudeRateMeasurement`, `AttitudeRateSensor`

## `separation.py`
Classes: `SeparatedBodyState`, `TwoBodySeparationResult`, `JettisonJump`, `RigidSeparatedBodyState`, `RigidTwoBodySeparationResult`
Functions: `separate_two_body`, `separate_two_rigid_bodies`

## `simulation.py`
Classes: `SimulationResult`, `SimulationEngine`

## `slosh.py`
Classes: `LinearSloshSubsystem`

## `state.py`
Classes: `StateField`, `StateSchema`, `StateView`
Functions: `core_3dof_schema`, `core_6dof_schema`, `entry_6dof_schema`, `edl_6dof_schema`, `gnc_edl_6dof_schema`, `augment_engineering_schema`, `engineering_6dof_schema`

## `subsystems.py`
Classes: `SubsystemBundle`, `WrenchSpecificForceBodyProvider`

## `terrain.py`
Classes: `TerrainSample`, `TerrainModel`, `RadialTerrain`

## `tps.py`
Classes: `TPSEvaluation`, `LumpedAblatingTPS`

## `units.py`
Classes: `UnitDimension`

## `universe.py`
Classes: `VehicleEvent`, `VehicleSpec`, `VehicleSnapshot`, `UniverseEventContext`, `UniverseMutation`, `VehicleTrajectorySegment`, `UniverseEventOccurrence`, `UniverseResult`, `MultiVehicleUniverseEngine`

## `validation_f.py`
Classes: `FLandingResult`
Functions: `build_f_landing_case`, `run_f_landing`, `f_landing_monte_carlo_case`

## `validation_g.py`
Classes: `GLandingResult`
Functions: `build_g_landing_case`, `run_g_landing`, `g_landing_monte_carlo_case`

## `validation_h.py`
Functions: `evaluate_radial_ascent`, `evaluate_radial_ascent_event`, `build_radial_ascent_targeter`, `build_radial_ascent_optimizer`

## `verification.py`
Classes: `TolerancePolicy`, `VerificationResult`, `VerificationReport`, `RegressionBaseline`, `ReferenceTimeHistory`
Functions: `compare_time_histories`, `observed_order`, `scalar_result`

## `verification_cases.py`
Functions: `case_rk4_order`, `case_adaptive_mms`, `case_tsiolkovsky`, `case_kepler`, `case_gravity_jacobian`, `case_quaternion`, `case_symmetric_torque_free`, `case_event_root`, `case_cross_integrator`, `case_separation`, `case_frame_roundtrip`, `case_quaternion_longrun`, `external_manifests`, `run_builtin_verification`

## `verify_cli.py`
Functions: `main`

## `wrenches.py`
Classes: `Wrench`, `WrenchModel`
