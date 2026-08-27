from __future__ import annotations
from dataclasses import dataclass
from typing import Mapping
import numpy as np

from .state import StateSchema, core_3dof_schema, core_6dof_schema
from .control import quaternion_align_body_x
from .frames import quat_normalize


def map_state_fields(
    source_schema: StateSchema,
    target_schema: StateSchema,
    source_state: np.ndarray,
    *,
    defaults: Mapping[str, object] | None = None,
) -> np.ndarray:
    """Map common named fields between schemas and fill new fields explicitly.

    Shape compatibility is enforced for every copied field.  This is the
    low-level schema transition primitive used by Milestone-I DOF switches.
    """
    src = source_schema.unpack(np.asarray(source_state, dtype=float))
    defaults = dict(defaults or {})
    target_values: dict[str, object] = {}
    src_fields = {f.key: f for f in source_schema.fields}
    for tf in target_schema.fields:
        if tf.key in src:
            sf = src_fields[tf.key]
            if sf.shape != tf.shape:
                raise ValueError(f"field {tf.key!r} changes shape across schemas")
            target_values[tf.key] = src[tf.key]
        elif tf.key in defaults:
            target_values[tf.key] = defaults[tf.key]
        else:
            raise KeyError(f"target-only field {tf.key!r} requires an explicit default")
    return target_schema.pack(target_values)


def demote_6dof_to_3dof(
    state_6dof: np.ndarray,
    *,
    source_schema: StateSchema | None = None,
    target_schema: StateSchema | None = None,
) -> np.ndarray:
    """Project a 6-DOF state onto translational position/velocity/mass."""
    src = source_schema or core_6dof_schema()
    dst = target_schema or core_3dof_schema()
    return map_state_fields(src, dst, state_6dof)


def promote_3dof_to_6dof(
    state_3dof: np.ndarray,
    *,
    attitude: np.ndarray | None = None,
    angular_rate_b: np.ndarray | None = None,
    source_schema: StateSchema | None = None,
    target_schema: StateSchema | None = None,
    up_hint_i: np.ndarray | None = None,
) -> np.ndarray:
    """Lift a 3-DOF state into 6-DOF with an explicit attitude/rate policy.

    If attitude is omitted, +x_B is aligned with inertial velocity.  A zero
    velocity requires an explicit attitude because no flight-direction frame
    exists in that case.
    """
    src = source_schema or core_3dof_schema()
    dst = target_schema or core_6dof_schema()
    values = src.unpack(np.asarray(state_3dof, dtype=float))
    if attitude is None:
        velocity = np.asarray(values["velocity"], dtype=float)
        if np.linalg.norm(velocity) <= 1e-14:
            raise ValueError("zero-velocity 3-DOF promotion requires explicit attitude")
        q = quaternion_align_body_x(velocity, up_hint_i=up_hint_i)
    else:
        q = quat_normalize(np.asarray(attitude, dtype=float))
    w = np.zeros(3) if angular_rate_b is None else np.asarray(angular_rate_b, dtype=float)
    if w.shape != (3,) or not np.all(np.isfinite(w)):
        raise ValueError("angular_rate_b must be a finite 3-vector")
    return map_state_fields(src, dst, state_3dof, defaults={"attitude": q, "angular_rate": w})


@dataclass(frozen=True, slots=True)
class DOFTransition:
    """Declarative state/schema transition used by universe event handlers."""

    target_schema: StateSchema
    defaults: Mapping[str, object] | None = None

    def apply(self, source_schema: StateSchema, source_state: np.ndarray) -> np.ndarray:
        return map_state_fields(source_schema, self.target_schema, source_state, defaults=self.defaults)
