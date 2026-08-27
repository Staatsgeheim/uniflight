from __future__ import annotations
from dataclasses import dataclass
from enum import Enum
from typing import Callable
import numpy as np

from .state import StateView
from .wrenches import Wrench


class FaultMode(str, Enum):
    GAIN = "gain"
    BIAS = "bias"
    STUCK = "stuck"
    DROPOUT = "dropout"


@dataclass(frozen=True, slots=True)
class FaultWindow:
    start_time: float
    end_time: float | None
    mode: FaultMode | str
    value: float = 0.0
    priority: int = 0

    def __post_init__(self) -> None:
        if not np.isfinite(self.start_time): raise ValueError("start_time must be finite")
        if self.end_time is not None and (not np.isfinite(self.end_time) or self.end_time <= self.start_time):
            raise ValueError("end_time must exceed start_time")
        object.__setattr__(self,"mode",FaultMode(self.mode))
        if not np.isfinite(self.value): raise ValueError("fault value must be finite")

    def active(self,t:float)->bool:
        return t>=self.start_time and (self.end_time is None or t<self.end_time)


@dataclass(frozen=True, slots=True)
class ScalarFaultSchedule:
    """Deterministic time-window fault transformation for scalar commands."""
    windows: tuple[FaultWindow,...]

    def __post_init__(self)->None:
        object.__setattr__(self,"windows",tuple(sorted(self.windows,key=lambda w:(w.priority,w.start_time))))

    def apply(self,value:float,time:float)->float:
        y=float(value)
        if not np.isfinite(y): raise ValueError("nominal scalar is non-finite")
        active=[w for w in self.windows if w.active(time)]
        for w in active:
            if w.mode is FaultMode.GAIN: y*=w.value
            elif w.mode is FaultMode.BIAS: y+=w.value
            elif w.mode is FaultMode.STUCK: y=w.value
            elif w.mode is FaultMode.DROPOUT: y=0.0
        return float(y)


@dataclass(frozen=True, slots=True)
class FaultedScalarProvider:
    nominal: float | Callable[[StateView],float]
    schedule: ScalarFaultSchedule
    lower: float | None = None
    upper: float | None = None

    def __call__(self,state:StateView)->float:
        v=float(self.nominal(state) if callable(self.nominal) else self.nominal)
        y=self.schedule.apply(v,state.time)
        if self.lower is not None: y=max(float(self.lower),y)
        if self.upper is not None: y=min(float(self.upper),y)
        return float(y)


@dataclass(frozen=True, slots=True)
class FaultedWrenchModel:
    """Scale or disable a wrench contribution according to a scalar schedule."""
    model: object
    schedule: ScalarFaultSchedule
    nominal_scale: float = 1.0
    source: str = "faulted-wrench"

    def wrench(self,state:StateView)->Wrench:
        base=self.model.wrench(state)
        s=self.schedule.apply(self.nominal_scale,state.time)
        return Wrench(s*base.force_i,s*base.moment_b,self.source+":"+base.source)
