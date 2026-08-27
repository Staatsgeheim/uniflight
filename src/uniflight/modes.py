from __future__ import annotations
from dataclasses import dataclass
from typing import Callable, Mapping
import numpy as np

from .events import Event, EventOccurrence
from .integrators import ScipyIVPIntegrator
from .simulation import SimulationEngine

TransitionFunction = Callable[[str, float, np.ndarray], str | None]


@dataclass(frozen=True, slots=True)
class ModeDefinition:
    name: str
    rhs: Callable[[float, np.ndarray], np.ndarray]
    events: tuple[Event, ...]


@dataclass(frozen=True, slots=True)
class ModeInterval:
    mode: str
    start_time: float
    end_time: float
    terminal_event: str | None


@dataclass(frozen=True, slots=True)
class HybridMissionResult:
    times: np.ndarray
    states: np.ndarray
    events: tuple[EventOccurrence, ...]
    modes: tuple[ModeInterval, ...]
    final_mode: str
    success: bool
    message: str


class HybridModeEngine:
    """Sequence phase-specific RHS/event sets without embedding mode in X.

    Each mode segment is solved by the trusted ``SimulationEngine``. A terminal
    event name is then passed to ``transition`` to select the next mode. This
    keeps the continuous state numeric and leaves mission logic discrete.
    """

    def __init__(self, modes: Mapping[str, ModeDefinition], transition: TransitionFunction,
                 integrator: ScipyIVPIntegrator | None = None):
        self.modes = dict(modes)
        self.transition = transition
        self.integrator = integrator or ScipyIVPIntegrator()
        if not self.modes:
            raise ValueError("at least one mode is required")
        for key, mode in self.modes.items():
            if key != mode.name:
                raise ValueError("mode mapping keys must equal ModeDefinition.name")

    def run(self, t_span: tuple[float,float], y0: np.ndarray, initial_mode: str,
            max_transitions: int = 100) -> HybridMissionResult:
        if initial_mode not in self.modes:
            raise KeyError(initial_mode)
        t0, tf = map(float, t_span)
        if tf <= t0:
            raise ValueError("t_span must increase")
        mode_name = initial_mode
        t = t0
        y = np.asarray(y0, dtype=float).copy()
        all_t: list[float] = []
        all_y: list[np.ndarray] = []
        all_events: list[EventOccurrence] = []
        intervals: list[ModeInterval] = []

        for _ in range(max_transitions + 1):
            mode = self.modes[mode_name]
            result = SimulationEngine(mode.rhs, self.integrator).run((t,tf), y, mode.events)
            if not result.success:
                return HybridMissionResult(np.asarray(all_t), np.asarray(all_y), tuple(all_events),
                                           tuple(intervals), mode_name, False, result.message)
            seg_t, seg_y = result.times, result.states
            if all_t and len(seg_t) and np.isclose(seg_t[0], all_t[-1], atol=0, rtol=0):
                seg_t = seg_t[1:]; seg_y = seg_y[1:]
            all_t.extend(seg_t.tolist()); all_y.extend([row.copy() for row in seg_y])
            all_events.extend(result.events)
            end_time = float(result.times[-1]) if len(result.times) else t
            intervals.append(ModeInterval(mode_name, t, end_time, result.terminated_by))
            if result.terminated_by is None:
                return HybridMissionResult(np.asarray(all_t), np.asarray(all_y), tuple(all_events),
                                           tuple(intervals), mode_name, True, "final time reached")
            y = result.states[-1].copy()
            next_mode = self.transition(result.terminated_by, end_time, y.copy())
            if next_mode is None:
                return HybridMissionResult(np.asarray(all_t), np.asarray(all_y), tuple(all_events),
                                           tuple(intervals), mode_name, True, "terminal mode event")
            if next_mode not in self.modes:
                raise KeyError(f"transition selected unknown mode {next_mode!r}")
            mode_name = next_mode
            t = np.nextafter(end_time, tf)
            if t >= tf:
                return HybridMissionResult(np.asarray(all_t), np.asarray(all_y), tuple(all_events),
                                           tuple(intervals), mode_name, True, "final time reached")
        raise RuntimeError("maximum mode transitions exceeded")
