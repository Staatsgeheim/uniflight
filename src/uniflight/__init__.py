"""UniFlight Milestone A kernel."""

from .state import StateField, StateSchema, StateView, core_3dof_schema, core_6dof_schema
from .frames import FrameGraph, Transform, quat_normalize, quat_to_matrix, quat_multiply
from .gravity import PointMassGravity
from .dynamics import DynamicsAssembler, TranslationalKinematics, QuaternionKinematics, IdealRocket
from .events import Event, EventOccurrence, EventAction
from .integrators import ScipyIVPIntegrator, SolverConfig
from .simulation import SimulationEngine, SimulationResult

__all__ = [
    "StateField", "StateSchema", "StateView", "core_3dof_schema", "core_6dof_schema",
    "FrameGraph", "Transform", "quat_normalize", "quat_to_matrix", "quat_multiply",
    "PointMassGravity", "DynamicsAssembler", "TranslationalKinematics",
    "QuaternionKinematics", "IdealRocket", "Event", "EventOccurrence", "EventAction",
    "ScipyIVPIntegrator", "SolverConfig", "SimulationEngine", "SimulationResult",
]
