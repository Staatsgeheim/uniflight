from __future__ import annotations
from dataclasses import dataclass
from typing import Callable
import numpy as np

from .events import Event, EventAction
from .state import StateSchema, StateView

Metric = Callable[[StateView], float]


@dataclass(frozen=True, slots=True)
class LimitAbortRule:
    name: str
    schema: StateSchema
    metric: Metric
    upper: float | None = None
    lower: float | None = None
    priority: int = 1000

    def __post_init__(self)->None:
        if (self.upper is None)==(self.lower is None):
            raise ValueError("exactly one of upper/lower must be supplied")
        limit=self.upper if self.upper is not None else self.lower
        if not np.isfinite(limit): raise ValueError("abort limit must be finite")

    def violated(self,state:StateView)->bool:
        value=float(self.metric(state))
        return value>=self.upper if self.upper is not None else value<=self.lower

    def event(self)->Event:
        if self.upper is not None:
            guard=lambda t,y: float(self.upper)-float(self.metric(StateView(t,y,self.schema)))
            direction=-1.0
        else:
            guard=lambda t,y: float(self.metric(StateView(t,y,self.schema)))-float(self.lower)
            direction=-1.0
        return Event(self.name,guard,direction=direction,priority=self.priority,action=EventAction.TERMINATE)


@dataclass(frozen=True, slots=True)
class AbortManager:
    rules: tuple[LimitAbortRule,...]

    def events(self)->tuple[Event,...]:
        return tuple(rule.event() for rule in self.rules)

    def violations(self,state:StateView)->tuple[str,...]:
        return tuple(rule.name for rule in self.rules if rule.violated(state))
