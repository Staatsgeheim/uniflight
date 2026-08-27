"""UniFlight Milestone C: planet-agnostic 6-DOF atmospheric-flight kernel."""

from .state import StateField, StateSchema, StateView, core_3dof_schema, core_6dof_schema
from .frames import (
    FrameGraph, Transform, quat_normalize, quat_to_matrix, quat_multiply,
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
from .propulsion import (
    RocketEvaluation, RocketEngine, Rocket6DOFEvaluation, GimballedRocketEngine,
)
from .dynamics import (
    DynamicsAssembler, TranslationalKinematics, QuaternionKinematics, IdealRocket,
    RigidBody6DOFDynamics,
)
from .events import Event, EventOccurrence, EventAction
from .integrators import ScipyIVPIntegrator, SolverConfig
from .simulation import SimulationEngine, SimulationResult

__all__ = [
    "StateField", "StateSchema", "StateView", "core_3dof_schema", "core_6dof_schema",
    "FrameGraph", "Transform", "quat_normalize", "quat_to_matrix", "quat_multiply",
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
    "RocketEvaluation", "RocketEngine", "Rocket6DOFEvaluation", "GimballedRocketEngine",
    "DynamicsAssembler", "TranslationalKinematics", "QuaternionKinematics", "IdealRocket", "RigidBody6DOFDynamics",
    "Event", "EventOccurrence", "EventAction", "ScipyIVPIntegrator", "SolverConfig",
    "SimulationEngine", "SimulationResult",
]
