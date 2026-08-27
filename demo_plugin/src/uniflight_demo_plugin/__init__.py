from __future__ import annotations

from dataclasses import dataclass
from types import SimpleNamespace
from typing import Any, Mapping
import numpy as np

from uniflight.plugins import PluginDescriptor, PLUGIN_API_VERSION
from uniflight.universe import UniverseMutation
from uniflight.optimization import OptimizationResult, TrajectoryOptimizer


@dataclass(frozen=True)
class ConstantAccelerationPropulsion:
    acceleration_mps2: float
    mass_flow_kgps: float
    direction_i: np.ndarray

    def __post_init__(self):
        d=np.asarray(self.direction_i,dtype=float); n=float(np.linalg.norm(d))
        if self.acceleration_mps2 < 0 or self.mass_flow_kgps < 0 or n <= 0:
            raise ValueError("invalid propulsion model")
        object.__setattr__(self,"direction_i",d/n)


def _propulsion_factory(spec: Mapping[str,Any], ctx: Mapping[str,Any]):
    return ConstantAccelerationPropulsion(
        float(spec["acceleration_mps2"]), float(spec.get("mass_flow_kgps",0.0)),
        np.asarray(spec.get("direction_i",[1.0,0.0,0.0]),dtype=float),
    )


def _dynamics_factory(spec: Mapping[str,Any], ctx: Mapping[str,Any]):
    schema=ctx["schema"]; body=ctx["body"]; models=ctx["models"]
    model_name=str(spec["propulsion_model"])
    if model_name not in models:
        raise KeyError(f"unknown propulsion model {model_name!r}")
    prop=models[model_name]
    psl=schema.sl("position"); vsl=schema.sl("velocity"); msl=schema.sl("mass")
    use_gravity=bool(spec.get("gravity",True))
    def rhs(t,y):
        y=np.asarray(y,dtype=float); dy=np.zeros_like(y)
        r=np.asarray(y[psl],dtype=float); v=np.asarray(y[vsl],dtype=float)
        dy[psl]=v
        a=np.zeros(3)
        if use_gravity: a += np.asarray(body.gravity.acceleration(r),dtype=float)
        a += prop.acceleration_mps2*prop.direction_i
        dy[vsl]=a
        dy[msl]=-prop.mass_flow_kgps
        return dy
    return rhs


def _time_guard_factory(spec: Mapping[str,Any], ctx: Mapping[str,Any]):
    target=float(spec["time_s"])
    return lambda t,y: float(t-target)


def _remove_action_factory(spec: Mapping[str,Any], ctx: Mapping[str,Any]):
    vehicle_id=str(ctx["vehicle_id"]); note=str(spec.get("note",f"plugin remove {vehicle_id}"))
    def handler(event_ctx):
        return UniverseMutation(remove=(vehicle_id,),note=note)
    return handler


def _specific_energy_output(spec: Mapping[str,Any], ctx: Mapping[str,Any]):
    output_spec=ctx["spec"]; result=ctx["result"]; bodies=ctx["bodies"]
    vid=str(output_spec["vehicle"]); body_name=str(spec.get("body",output_spec.get("body","")))
    snap=result.final_vehicles[vid]; vals=snap.schema.unpack(snap.state)
    body=bodies[body_name or snap.model_context["body"]]
    r=float(np.linalg.norm(vals["position"])); v2=float(np.dot(vals["velocity"],vals["velocity"]))
    return 0.5*v2-float(body.mu)/r


class GridSearchOptimizer:
    def __init__(self, points: int=31): self.points=max(3,int(points))
    def solve(self, problem):
        space=problem.design_space
        if space.size != 1:
            raise ValueError("demo grid optimizer supports exactly one design variable")
        lo,hi=space.bounds_scaled; best=None; nfev=0
        checker=TrajectoryOptimizer()
        for z0 in np.linspace(float(lo[0]),float(hi[0]),self.points):
            ev=problem.evaluate_scaled(np.array([z0])); nfev+=1
            violation=max((checker._violation(c,ev.metrics) for c in problem.constraints),default=0.0)
            feasible=violation <= 1e-6
            score=(0 if feasible else 1, violation, ev.objective)
            if best is None or score < best[0]: best=(score,ev,violation,z0)
        _,ev,violation,z0=best
        return OptimizationResult(
            bool(violation<=1e-6), "demo deterministic grid search", ev.parameters,
            float(ev.objective), ev.metrics, float(violation), nfev, 1,
            "demo.nereid:grid-search", SimpleNamespace(x=np.array([z0])),
        )


def _optimizer_factory(spec: Mapping[str,Any], ctx: Mapping[str,Any]):
    return GridSearchOptimizer(int(spec.get("points",31)))


def _register(registrar):
    registrar.register("propulsion","constant-acceleration",_propulsion_factory,
                       description="Constant inertial acceleration + mass flow reference propulsion model")
    registrar.register("dynamics","point-mass-propulsion",_dynamics_factory,
                       description="3-DOF point-mass dynamics consuming a declared propulsion model")
    registrar.register("guard","time",_time_guard_factory,
                       description="Plugin-defined time guard")
    registrar.register("event_action","remove-vehicle",_remove_action_factory,
                       description="Plugin-defined topology mutation")
    registrar.register("output","specific-energy",_specific_energy_output,
                       description="Specific orbital energy output")
    registrar.register("optimizer","grid-search",_optimizer_factory,
                       description="Deterministic one-variable grid-search optimizer")


def plugin_descriptor():
    return PluginDescriptor(
        plugin_id="demo.nereid", version="1.0.0", api_version=PLUGIN_API_VERSION,
        register=_register, description="UniFlight Milestone M third-party reference plugin",
        homepage="https://example.invalid/uniflight-demo-plugin",
        metadata={"reference_only":True},
    )
