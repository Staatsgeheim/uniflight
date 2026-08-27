"""UniFlight Milestone B atmospheric-flight kernel."""

from .state import StateField, StateSchema, StateView, core_3dof_schema, core_6dof_schema
from .frames import FrameGraph, Transform, quat_normalize, quat_to_matrix, quat_multiply
from .gravity import PointMassGravity
from .bodies import SphericalBody
from .gases import GasSpecies, GasMixture, R_UNIVERSAL, BOLTZMANN
from .atmosphere import AtmosphereSample, AtmosphereModel, VacuumAtmosphere, IsothermalHydrostaticAtmosphere
from .environment import EnvironmentSample, PlanetaryEnvironment
from .flow import FlowState, compute_flow_state
from .aerodynamics import (
    DragCoefficientModel, ConstantDragCoefficient, MachTableDragCoefficient,
    AeroEvaluation, ContinuumDrag,
)
from .propulsion import RocketEvaluation, RocketEngine
from .dynamics import DynamicsAssembler, TranslationalKinematics, QuaternionKinematics, IdealRocket
from .events import Event, EventOccurrence, EventAction
from .integrators import ScipyIVPIntegrator, SolverConfig
from .simulation import SimulationEngine, SimulationResult

__all__ = [
    "StateField", "StateSchema", "StateView", "core_3dof_schema", "core_6dof_schema",
    "FrameGraph", "Transform", "quat_normalize", "quat_to_matrix", "quat_multiply",
    "PointMassGravity", "SphericalBody",
    "GasSpecies", "GasMixture", "R_UNIVERSAL", "BOLTZMANN",
    "AtmosphereSample", "AtmosphereModel", "VacuumAtmosphere", "IsothermalHydrostaticAtmosphere",
    "EnvironmentSample", "PlanetaryEnvironment", "FlowState", "compute_flow_state",
    "DragCoefficientModel", "ConstantDragCoefficient", "MachTableDragCoefficient",
    "AeroEvaluation", "ContinuumDrag", "RocketEvaluation", "RocketEngine",
    "DynamicsAssembler", "TranslationalKinematics", "QuaternionKinematics", "IdealRocket",
    "Event", "EventOccurrence", "EventAction", "ScipyIVPIntegrator", "SolverConfig",
    "SimulationEngine", "SimulationResult",
]
