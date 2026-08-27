from __future__ import annotations
from dataclasses import dataclass

@dataclass(frozen=True, slots=True)
class UnitDimension:
    """Human-readable SI metadata. Numeric kernel values are always coherent SI."""
    symbol: str
    si_unit: str

DIMENSIONLESS = UnitDimension("1", "1")
LENGTH = UnitDimension("L", "m")
VELOCITY = UnitDimension("L T^-1", "m/s")
ACCELERATION = UnitDimension("L T^-2", "m/s^2")
MASS = UnitDimension("M", "kg")
ANGLE = UnitDimension("1", "rad")
ANGULAR_RATE = UnitDimension("T^-1", "rad/s")
RATE = UnitDimension("T^-1", "1/s")
TEMPERATURE = UnitDimension("Theta", "K")
AREAL_ENERGY = UnitDimension("M T^-2", "J/m^2")
