from __future__ import annotations
from dataclasses import dataclass
from enum import Enum, auto
from typing import Callable
import numpy as np

Guard = Callable[[float, np.ndarray], float]
JumpMap = Callable[[float, np.ndarray], np.ndarray]

class EventAction(Enum):
    CONTINUE = auto()
    TERMINATE = auto()

@dataclass(frozen=True, slots=True)
class Event:
    name: str
    guard: Guard
    direction: float = 0.0
    priority: int = 0
    action: EventAction = EventAction.CONTINUE
    jump: JumpMap | None = None

@dataclass(frozen=True, slots=True)
class EventOccurrence:
    name: str
    time: float
    state_before: np.ndarray
    state_after: np.ndarray
    priority: int
