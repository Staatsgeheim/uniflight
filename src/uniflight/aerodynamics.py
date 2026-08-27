from __future__ import annotations
from dataclasses import dataclass
from typing import Protocol
import numpy as np

from .environment import PlanetaryEnvironment, EnvironmentSample
from .flow import FlowState, compute_flow_state
from .state import StateView


class DragCoefficientModel(Protocol):
    def __call__(self, flow: FlowState) -> float: ...


@dataclass(frozen=True, slots=True)
class ConstantDragCoefficient:
    cd: float

    def __post_init__(self) -> None:
        if not np.isfinite(self.cd) or self.cd < 0:
            raise ValueError("cd must be finite and non-negative")

    def __call__(self, flow: FlowState) -> float:
        return float(self.cd)


@dataclass(frozen=True, slots=True)
class MachTableDragCoefficient:
    mach: np.ndarray
    cd: np.ndarray

    def __post_init__(self) -> None:
        m = np.asarray(self.mach, dtype=float)
        c = np.asarray(self.cd, dtype=float)
        if m.ndim != 1 or c.ndim != 1 or len(m) < 2 or m.shape != c.shape:
            raise ValueError("mach and cd must be same-length 1-D arrays with at least two points")
        if not np.all(np.isfinite(m)) or not np.all(np.isfinite(c)) or np.any(c < 0):
            raise ValueError("table values must be finite and cd non-negative")
        if np.any(np.diff(m) <= 0):
            raise ValueError("mach table must be strictly increasing")
        object.__setattr__(self, "mach", m.copy())
        object.__setattr__(self, "cd", c.copy())

    def __call__(self, flow: FlowState) -> float:
        return float(np.interp(flow.mach, self.mach, self.cd))


@dataclass(frozen=True, slots=True)
class AeroEvaluation:
    environment: EnvironmentSample
    flow: FlowState
    cd: float
    force_i: np.ndarray


@dataclass(frozen=True, slots=True)
class ContinuumDrag:
    """Milestone-B point-mass continuum drag model."""

    environment: PlanetaryEnvironment
    reference_area: float
    reference_length: float
    coefficient: DragCoefficientModel
    max_knudsen: float | None = None

    def __post_init__(self) -> None:
        if not np.isfinite(self.reference_area) or self.reference_area <= 0:
            raise ValueError("reference_area must be finite and positive")
        if not np.isfinite(self.reference_length) or self.reference_length <= 0:
            raise ValueError("reference_length must be finite and positive")
        if self.max_knudsen is not None and (not np.isfinite(self.max_knudsen) or self.max_knudsen <= 0):
            raise ValueError("max_knudsen must be finite and positive")

    def evaluate(self, state: StateView) -> AeroEvaluation:
        env = self.environment.query(state.get("position"), state.time)
        flow = compute_flow_state(state.get("velocity"), env, self.reference_length)
        if self.max_knudsen is not None and flow.knudsen > self.max_knudsen and flow.dynamic_pressure > 0:
            raise ValueError(
                f"continuum model outside declared Knudsen validity: Kn={flow.knudsen:g} > {self.max_knudsen:g}"
            )
        cd = float(self.coefficient(flow))
        if not np.isfinite(cd) or cd < 0:
            raise ValueError("drag coefficient model returned invalid cd")
        if flow.speed == 0.0 or flow.dynamic_pressure == 0.0 or cd == 0.0:
            force = np.zeros(3)
        else:
            force = -flow.dynamic_pressure * self.reference_area * cd * flow.relative_velocity_i / flow.speed
        return AeroEvaluation(env, flow, cd, force)

    def force_i(self, state: StateView) -> np.ndarray:
        return self.evaluate(state).force_i

    def acceleration(self, state: StateView) -> np.ndarray:
        m = float(state.get("mass"))
        if m <= 0:
            raise ValueError("vehicle mass must remain positive")
        return self.force_i(state) / m
