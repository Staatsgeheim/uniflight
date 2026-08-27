from __future__ import annotations
from dataclasses import dataclass
from hashlib import sha256
from types import MappingProxyType
from typing import Mapping
import json
import numpy as np

from .units import DIMENSIONLESS, LENGTH, VELOCITY, MASS, ANGULAR_RATE, UnitDimension

@dataclass(frozen=True, slots=True)
class StateField:
    key: str
    shape: tuple[int, ...]
    unit: UnitDimension
    frame: str | None = None
    owner: str = "core"
    continuity: str = "CONTINUOUS"

    @property
    def size(self) -> int:
        return int(np.prod(self.shape, dtype=int)) if self.shape else 1

class StateSchema:
    def __init__(self, fields: tuple[StateField, ...] | list[StateField]):
        self.fields = tuple(fields)
        keys = [f.key for f in self.fields]
        if len(keys) != len(set(keys)):
            raise ValueError("State field keys must be unique")
        slices: dict[str, slice] = {}
        offset = 0
        for f in self.fields:
            slices[f.key] = slice(offset, offset + f.size)
            offset += f.size
        self._slices = MappingProxyType(slices)
        self.total_size = offset
        serial = [{"key":f.key,"shape":f.shape,"unit":f.unit.si_unit,"frame":f.frame,
                   "owner":f.owner,"continuity":f.continuity} for f in self.fields]
        self.layout_hash = sha256(json.dumps(serial, sort_keys=True).encode()).hexdigest()

    def field(self, key: str) -> StateField:
        for f in self.fields:
            if f.key == key:
                return f
        raise KeyError(key)

    def sl(self, key: str) -> slice:
        return self._slices[key]

    def pack(self, values: Mapping[str, object]) -> np.ndarray:
        y = np.empty(self.total_size, dtype=float)
        expected = {f.key for f in self.fields}
        missing = expected - values.keys()
        extra = values.keys() - expected
        if missing or extra:
            raise ValueError(f"State keys mismatch; missing={sorted(missing)}, extra={sorted(extra)}")
        for f in self.fields:
            a = np.asarray(values[f.key], dtype=float)
            if f.shape:
                if a.shape != f.shape:
                    raise ValueError(f"{f.key} expected shape {f.shape}, got {a.shape}")
                y[self.sl(f.key)] = a.reshape(-1)
            else:
                if a.size != 1:
                    raise ValueError(f"{f.key} must be scalar")
                y[self.sl(f.key)] = float(a.reshape(-1)[0])
        if not np.all(np.isfinite(y)):
            raise ValueError("State contains non-finite values")
        return y

    def unpack(self, packed: np.ndarray) -> dict[str, object]:
        y = np.asarray(packed, dtype=float)
        if y.shape != (self.total_size,):
            raise ValueError(f"Expected packed shape {(self.total_size,)}, got {y.shape}")
        out: dict[str, object] = {}
        for f in self.fields:
            a = y[self.sl(f.key)]
            out[f.key] = float(a[0]) if not f.shape else a.reshape(f.shape).copy()
        return out

class StateView:
    def __init__(self, time: float, packed: np.ndarray, schema: StateSchema):
        arr = np.asarray(packed, dtype=float)
        if arr.shape != (schema.total_size,):
            raise ValueError("Packed state does not match schema")
        self.time = float(time)
        self.schema = schema
        self._packed = arr.view()
        self._packed.flags.writeable = False

    @property
    def packed(self) -> np.ndarray:
        return self._packed

    def get(self, key: str):
        f = self.schema.field(key)
        a = self._packed[self.schema.sl(key)]
        if not f.shape:
            return float(a[0])
        out = a.reshape(f.shape).view()
        out.flags.writeable = False
        return out


def core_3dof_schema() -> StateSchema:
    return StateSchema([
        StateField("position", (3,), LENGTH, "I", owner="kinematics"),
        StateField("velocity", (3,), VELOCITY, "I", owner="dynamics"),
        StateField("mass", (), MASS, None, owner="mass"),
    ])


def core_6dof_schema() -> StateSchema:
    return StateSchema([
        StateField("position", (3,), LENGTH, "I", owner="kinematics"),
        StateField("velocity", (3,), VELOCITY, "I", owner="dynamics"),
        StateField("attitude", (4,), DIMENSIONLESS, "B<-I", owner="attitude"),
        StateField("angular_rate", (3,), ANGULAR_RATE, "B", owner="rotation"),
        StateField("mass", (), MASS, None, owner="mass"),
    ])
