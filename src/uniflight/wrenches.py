from __future__ import annotations
from dataclasses import dataclass
from typing import Protocol
import numpy as np

from .state import StateView


@dataclass(frozen=True, slots=True)
class Wrench:
    """Force/moment contribution for one rigid vehicle.

    ``force_i`` is expressed in the inertial frame. ``moment_b`` is expressed
    in the vehicle body frame and is taken about the instantaneous center of
    mass. Mass flow is intentionally not included here; the state owner for
    mass remains a separate derivative model.
    """

    force_i: np.ndarray
    moment_b: np.ndarray
    source: str = "unnamed"

    def __post_init__(self) -> None:
        f = np.asarray(self.force_i, dtype=float)
        m = np.asarray(self.moment_b, dtype=float)
        if f.shape != (3,) or m.shape != (3,):
            raise ValueError("force_i and moment_b must be 3-vectors")
        if not np.all(np.isfinite(f)) or not np.all(np.isfinite(m)):
            raise ValueError("wrench contains non-finite values")
        object.__setattr__(self, "force_i", f.copy())
        object.__setattr__(self, "moment_b", m.copy())

    @staticmethod
    def zero(source: str = "zero") -> "Wrench":
        return Wrench(np.zeros(3), np.zeros(3), source)


class WrenchModel(Protocol):
    def wrench(self, state: StateView) -> Wrench: ...
