"""UniFlight Milestone I: celestial-body-agnostic multi-vehicle, multi-DOF flight dynamics."""

__version__ = "0.9.0"

from .state import StateField, StateSchema, StateView, core_3dof_schema, core_6dof_schema, entry_6dof_schema, edl_6dof_schema
from .frames import (
    FrameGraph, Transform, quat_normalize, quat_to_matrix, quat_multiply, matrix_to_quat,
    body_to_inertial_matrix, inertial_to_body_matrix,
    rotate_body_to_inertial, rotate_inertial_to_body,
)
from .gravity import PointMassGravity
from .bodies import SphericalBody
from .gases import GasSpecies, GasMixture, R_UNIVERSAL, BOLTZMANN
from .atmosphere import AtmosphereSample, AtmosphereModel, VacuumAtmosphere, IsothermalHydrostaticAtmosphere
from .environment import EnvironmentSample, PlanetaryEnvironment
from .flow import (
    FlowState, BodyFlowState, compute_flow_state, compute_body_flow_state,
    wind_to_body_matrix,
)
from .wrenches import Wrench, WrenchModel
from .mass_properties import MassProperties, MassPropertiesModel, ConstantMassProperties, AffineMassProperties
from .aerodynamics import (
    DragCoefficientModel, ConstantDragCoefficient, MachTableDragCoefficient,
    AeroEvaluation, ContinuumDrag,
    AeroCoefficients, AeroCoefficientModel6DOF, ConstantAeroCoefficients,
    LinearStabilityAerodynamics, GridAeroCoefficientDatabase,
    GeometryEvaluation, GeometryModel, ConstantReferenceGeometry,
    EllipsoidProjectedGeometry, Aero6DOFEvaluation, ContinuumAerodynamics6DOF,
)
from .hypersonics import NewtonianHypersonicCoefficients, MachBlendedAeroCoefficients
from .rarefied import FreeMolecularAerodynamics6DOF, RegimeAeroEvaluation, RegimeBlendedAerodynamics6DOF
from .chemistry import (
    ThermochemicalCorrection, ChemistryCorrectionModel, FrozenChemistry,
    ThresholdDissociationCorrection,
)
from .heating import (
    AerothermalEvaluation, AerothermalModel, PowerLawRadiativeHeating,
    SuttonGravesHeating,
)
from .tps import STEFAN_BOLTZMANN, TPSEvaluation, LumpedAblatingTPS
from .massflow import MassFlowSource, MassFlowAggregator
from .propulsion import (
    RocketEvaluation, RocketEngine, Rocket6DOFEvaluation, GimballedRocketEngine,
)
from .dynamics import (
    DynamicsAssembler, TranslationalKinematics, QuaternionKinematics, IdealRocket,
    RigidBody6DOFDynamics,
)
from .events import Event, EventOccurrence, EventAction
from .integrators import ScipyIVPIntegrator, SolverConfig, FixedStepRK4Config, FixedStepRK4Integrator
from .simulation import SimulationEngine, SimulationResult

from .deployables import FirstOrderDeployable, ParachuteEvaluation, InflatingParachute
from .terrain import TerrainSample, TerrainModel, RadialTerrain
from .contact import GearLeg, LegContactEvaluation, LandingGearEvaluation, LandingGearContact
from .separation import (
    SeparatedBodyState, TwoBodySeparationResult, separate_two_body, JettisonJump,
)
from .guidance import DescentGuidanceEvaluation, VerticalDescentThrottle
from .modes import ModeDefinition, ModeInterval, HybridMissionResult, HybridModeEngine

__all__ = [
    "StateField", "StateSchema", "StateView", "core_3dof_schema", "core_6dof_schema", "entry_6dof_schema", "edl_6dof_schema",
    "FrameGraph", "Transform", "quat_normalize", "quat_to_matrix", "quat_multiply", "matrix_to_quat",
    "body_to_inertial_matrix", "inertial_to_body_matrix", "rotate_body_to_inertial", "rotate_inertial_to_body",
    "PointMassGravity", "SphericalBody",
    "GasSpecies", "GasMixture", "R_UNIVERSAL", "BOLTZMANN",
    "AtmosphereSample", "AtmosphereModel", "VacuumAtmosphere", "IsothermalHydrostaticAtmosphere",
    "EnvironmentSample", "PlanetaryEnvironment",
    "FlowState", "BodyFlowState", "compute_flow_state", "compute_body_flow_state", "wind_to_body_matrix",
    "Wrench", "WrenchModel", "MassProperties", "MassPropertiesModel", "ConstantMassProperties", "AffineMassProperties",
    "DragCoefficientModel", "ConstantDragCoefficient", "MachTableDragCoefficient", "AeroEvaluation", "ContinuumDrag",
    "AeroCoefficients", "AeroCoefficientModel6DOF", "ConstantAeroCoefficients", "LinearStabilityAerodynamics",
    "GridAeroCoefficientDatabase", "GeometryEvaluation", "GeometryModel", "ConstantReferenceGeometry",
    "EllipsoidProjectedGeometry", "Aero6DOFEvaluation", "ContinuumAerodynamics6DOF",
    "NewtonianHypersonicCoefficients", "MachBlendedAeroCoefficients",
    "FreeMolecularAerodynamics6DOF", "RegimeAeroEvaluation", "RegimeBlendedAerodynamics6DOF",
    "ThermochemicalCorrection", "ChemistryCorrectionModel", "FrozenChemistry", "ThresholdDissociationCorrection",
    "AerothermalEvaluation", "AerothermalModel", "PowerLawRadiativeHeating", "SuttonGravesHeating",
    "STEFAN_BOLTZMANN", "TPSEvaluation", "LumpedAblatingTPS", "MassFlowSource", "MassFlowAggregator",
    "RocketEvaluation", "RocketEngine", "Rocket6DOFEvaluation", "GimballedRocketEngine",
    "DynamicsAssembler", "TranslationalKinematics", "QuaternionKinematics", "IdealRocket", "RigidBody6DOFDynamics",
    "Event", "EventOccurrence", "EventAction", "ScipyIVPIntegrator", "SolverConfig", "FixedStepRK4Config", "FixedStepRK4Integrator",
    "SimulationEngine", "SimulationResult",
    "FirstOrderDeployable", "ParachuteEvaluation", "InflatingParachute",
    "TerrainSample", "TerrainModel", "RadialTerrain",
    "GearLeg", "LegContactEvaluation", "LandingGearEvaluation", "LandingGearContact",
    "SeparatedBodyState", "TwoBodySeparationResult", "separate_two_body", "JettisonJump",
    "DescentGuidanceEvaluation", "VerticalDescentThrottle",
    "ModeDefinition", "ModeInterval", "HybridMissionResult", "HybridModeEngine",
]

# Milestone F: sampled-data GNC, sensors, estimation, robustness, and aborts.
from .state import gnc_edl_6dof_schema
from .actuators import (
    GNCCommandBus, StateFieldProvider, BusScalarProvider,
    FirstOrderLimitedStateActuator, CommandedBodyTorque,
)
from .sensors import (
    SensorMeasurement, PositionVelocitySensor, RadarAltimeterSensor,
    AttitudeRateMeasurement, AttitudeRateSensor,
)
from .estimation import (
    numerical_jacobian, EKFUpdate, ExtendedKalmanFilter,
    KinematicProcessModel, TranslationalNavigationEKF,
)
from .control import (
    ThrustGuidanceCommand, VectorLandingGuidance, quaternion_align_body_x,
    QuaternionPDController, AdaptiveThrustScaleEstimator, GNCDecision, LandingGNCController,
)
from .aborts import LimitAbortRule, AbortManager
from .montecarlo import (
    Dispersion, NormalDispersion, UniformDispersion, MonteCarloCaseResult,
    MetricStatistics, MonteCarloSummary, MonteCarloRunner, automatic_worker_count,
)
from .closed_loop import GNCRecord, ClosedLoopResult, SampledDataClosedLoopEngine

__all__ += [
    "gnc_edl_6dof_schema",
    "GNCCommandBus", "StateFieldProvider", "BusScalarProvider",
    "FirstOrderLimitedStateActuator", "CommandedBodyTorque",
    "SensorMeasurement", "PositionVelocitySensor", "RadarAltimeterSensor",
    "AttitudeRateMeasurement", "AttitudeRateSensor",
    "numerical_jacobian", "EKFUpdate", "ExtendedKalmanFilter",
    "KinematicProcessModel", "TranslationalNavigationEKF",
    "ThrustGuidanceCommand", "VectorLandingGuidance", "quaternion_align_body_x",
    "QuaternionPDController", "AdaptiveThrustScaleEstimator", "GNCDecision", "LandingGNCController",
    "LimitAbortRule", "AbortManager",
    "Dispersion", "NormalDispersion", "UniformDispersion", "MonteCarloCaseResult",
    "MetricStatistics", "MonteCarloSummary", "MonteCarloRunner", "automatic_worker_count",
    "GNCRecord", "ClosedLoopResult", "SampledDataClosedLoopEngine",
]

# Milestone H: trajectory targeting and optimization
from .optimization import (
    DesignVariable, DesignSpace, MetricObjective, MetricConstraint,
    TrajectoryProblem, ProblemEvaluation, FiniteDifferenceConfig,
    finite_difference_jacobian, TargetingSettings, TargetingResult,
    TrajectoryTargeter, OptimizationSettings, OptimizationResult,
    TrajectoryOptimizer, MultipleShootingTranscription,
    BatchEvaluationResult, parallel_batch_evaluate,
)
from .validation_h import (
    BODY_H, MASS0_H, VE_H, TARGET_APOGEE_H,
    evaluate_radial_ascent, evaluate_radial_ascent_event, build_radial_ascent_targeter,
    build_radial_ascent_optimizer,
)

__all__ += [
    "DesignVariable", "DesignSpace", "MetricObjective", "MetricConstraint",
    "TrajectoryProblem", "ProblemEvaluation", "FiniteDifferenceConfig",
    "finite_difference_jacobian", "TargetingSettings", "TargetingResult",
    "TrajectoryTargeter", "OptimizationSettings", "OptimizationResult",
    "TrajectoryOptimizer", "MultipleShootingTranscription",
    "BatchEvaluationResult", "parallel_batch_evaluate",
    "BODY_H", "MASS0_H", "VE_H", "TARGET_APOGEE_H",
    "evaluate_radial_ascent", "evaluate_radial_ascent_event",
    "build_radial_ascent_targeter", "build_radial_ascent_optimizer",
]

# Milestone I: true multi-vehicle / multi-DOF runtime
from .dof import map_state_fields, demote_6dof_to_3dof, promote_3dof_to_6dof, DOFTransition
from .universe import (
    VehicleEvent, VehicleSpec, VehicleSnapshot, UniverseEventContext,
    UniverseMutation, VehicleTrajectorySegment, UniverseEventOccurrence,
    UniverseResult, MultiVehicleUniverseEngine,
)
from .multibody import (
    VehicleConfiguration, DOFSwitchHandler, RigidChildTemplate, RigidSeparationHandler,
)
from .separation import (
    RigidSeparatedBodyState, RigidTwoBodySeparationResult, separate_two_rigid_bodies,
)

__all__ += [
    "map_state_fields", "demote_6dof_to_3dof", "promote_3dof_to_6dof", "DOFTransition",
    "VehicleEvent", "VehicleSpec", "VehicleSnapshot", "UniverseEventContext",
    "UniverseMutation", "VehicleTrajectorySegment", "UniverseEventOccurrence",
    "UniverseResult", "MultiVehicleUniverseEngine",
    "VehicleConfiguration", "DOFSwitchHandler", "RigidChildTemplate", "RigidSeparationHandler",
    "RigidSeparatedBodyState", "RigidTwoBodySeparationResult", "separate_two_rigid_bodies",
]
