from __future__ import annotations
from dataclasses import dataclass, field
from typing import Callable, Mapping, Any
import numpy as np

from .dof import map_state_fields
from .separation import RigidSeparatedBodyState, separate_two_rigid_bodies
from .state import StateSchema
from .universe import (
    UniverseEventContext, UniverseMutation, VehicleEvent, VehicleSpec,
)


@dataclass(frozen=True, slots=True)
class VehicleConfiguration:
    """Reusable definition of a vehicle mode independent of its current state."""

    schema: StateSchema
    rhs: Callable[[float, np.ndarray], np.ndarray]
    events: tuple[VehicleEvent, ...] = ()
    integrator: object | None = None
    mode: str = "default"
    dof: int | None = None
    model_context: Mapping[str, Any] = field(default_factory=dict)

    def instantiate(self, vehicle_id: str, state: np.ndarray) -> VehicleSpec:
        return VehicleSpec(
            vehicle_id, self.schema, np.asarray(state, dtype=float), self.rhs,
            tuple(self.events), self.integrator, self.mode, self.dof,
            dict(self.model_context),
        )


@dataclass(frozen=True, slots=True)
class DOFSwitchHandler:
    """Replace one active vehicle with another schema/dynamics configuration."""

    target: VehicleConfiguration
    defaults: Mapping[str, object] | None = None
    state_mapper: Callable[[StateSchema, StateSchema, np.ndarray], np.ndarray] | None = None
    note: str = "DOF/configuration transition"

    def __call__(self, context: UniverseEventContext) -> UniverseMutation:
        src = context.source
        if self.state_mapper is None:
            y = map_state_fields(src.schema, self.target.schema, src.state, defaults=self.defaults)
        else:
            y = np.asarray(self.state_mapper(src.schema, self.target.schema, src.state), dtype=float)
        spec = self.target.instantiate(src.vehicle_id, y)
        return UniverseMutation(upsert=(spec,), note=self.note)


@dataclass(frozen=True, slots=True)
class RigidChildTemplate:
    """Configuration and state defaults for a daughter body after separation."""

    vehicle_id: str
    configuration: VehicleConfiguration
    extra_state_defaults: Mapping[str, object] = field(default_factory=dict)

    def build_state(self, body: RigidSeparatedBodyState) -> np.ndarray:
        core = {
            "position": body.position_i,
            "velocity": body.velocity_i,
            "attitude": body.attitude_bi,
            "angular_rate": body.angular_rate_b,
            "mass": body.mass,
        }
        values: dict[str, object] = {}
        for f in self.configuration.schema.fields:
            if f.key in core:
                values[f.key] = core[f.key]
            elif f.key in self.extra_state_defaults:
                values[f.key] = self.extra_state_defaults[f.key]
            else:
                raise KeyError(
                    f"daughter {self.vehicle_id!r} field {f.key!r} needs an extra_state_default"
                )
        return self.configuration.schema.pack(values)


@dataclass(frozen=True, slots=True)
class RigidSeparationHandler:
    """Universe event handler for momentum-consistent two-body 6-DOF staging."""

    retained: RigidChildTemplate
    detached: RigidChildTemplate
    retained_mass: float
    detached_mass: float
    retained_offset_b: np.ndarray
    detached_offset_b: np.ndarray
    parent_inertia_b: np.ndarray
    retained_inertia_b: np.ndarray
    detached_inertia_b: np.ndarray
    relative_separation_velocity_i: np.ndarray = field(default_factory=lambda: np.zeros(3))
    conserve_angular_momentum: bool = True
    note: str = "rigid two-body separation"

    def __call__(self, context: UniverseEventContext) -> UniverseMutation:
        src = context.source
        values = src.schema.unpack(src.state)
        required = {"position", "velocity", "attitude", "angular_rate", "mass"}
        if not required.issubset(values):
            missing = sorted(required-set(values))
            raise KeyError(f"6-DOF separation source is missing fields {missing}")
        result = separate_two_rigid_bodies(
            parent_mass=float(values["mass"]),
            parent_position_i=np.asarray(values["position"]),
            parent_velocity_i=np.asarray(values["velocity"]),
            parent_attitude_bi=np.asarray(values["attitude"]),
            parent_angular_rate_b=np.asarray(values["angular_rate"]),
            parent_inertia_b=np.asarray(self.parent_inertia_b),
            retained_mass=self.retained_mass,
            detached_mass=self.detached_mass,
            retained_offset_b=np.asarray(self.retained_offset_b),
            detached_offset_b=np.asarray(self.detached_offset_b),
            retained_inertia_b=np.asarray(self.retained_inertia_b),
            detached_inertia_b=np.asarray(self.detached_inertia_b),
            relative_separation_velocity_i=np.asarray(self.relative_separation_velocity_i),
            conserve_angular_momentum=self.conserve_angular_momentum,
        )
        r_spec = self.retained.configuration.instantiate(
            self.retained.vehicle_id, self.retained.build_state(result.retained)
        )
        d_spec = self.detached.configuration.instantiate(
            self.detached.vehicle_id, self.detached.build_state(result.detached)
        )
        upsert = (r_spec, d_spec)
        remove = () if context.vehicle_id in {r_spec.vehicle_id, d_spec.vehicle_id} else (context.vehicle_id,)
        # If the parent ID is reused by one daughter, it is replaced via upsert.
        if context.vehicle_id == r_spec.vehicle_id or context.vehicle_id == d_spec.vehicle_id:
            remove = ()
        return UniverseMutation(remove=remove, upsert=upsert, note=self.note)
