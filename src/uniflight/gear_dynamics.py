from __future__ import annotations
from dataclasses import dataclass
import numpy as np

from .frames import body_to_inertial_matrix
from .mass_properties import MassPropertiesModel
from .state import StateView
from .terrain import TerrainModel
from .wrenches import Wrench


@dataclass(frozen=True, slots=True)
class DynamicGearLeg:
    foot_b: np.ndarray
    compression_axis_b: np.ndarray
    stiffness: float
    damping: float
    effective_mass: float
    max_compression: float
    friction_coefficient: float = 0.0

    def __post_init__(self)->None:
        foot=np.asarray(self.foot_b,dtype=float); axis=np.asarray(self.compression_axis_b,dtype=float)
        if foot.shape!=(3,) or not np.all(np.isfinite(foot)): raise ValueError("foot_b must be finite 3-vector")
        if axis.shape!=(3,) or not np.all(np.isfinite(axis)) or np.linalg.norm(axis)==0: raise ValueError("compression_axis_b must be nonzero finite 3-vector")
        axis=axis/np.linalg.norm(axis)
        for name in ("stiffness","effective_mass","max_compression"):
            v=float(getattr(self,name));
            if not np.isfinite(v) or v<=0: raise ValueError(f"{name} must be positive")
        if not np.isfinite(self.damping) or self.damping<0: raise ValueError("damping must be non-negative")
        if not np.isfinite(self.friction_coefficient) or self.friction_coefficient<0: raise ValueError("friction coefficient must be non-negative")
        object.__setattr__(self,"foot_b",foot.copy()); object.__setattr__(self,"compression_axis_b",axis)


@dataclass(frozen=True, slots=True)
class DynamicLandingGear:
    """Stateful landing-gear struts with terrain-driven compression dynamics."""
    terrain: TerrainModel
    mass_properties: MassPropertiesModel
    legs: tuple[DynamicGearLeg,...]
    compression_key: str = "gear_compression"
    rate_key: str = "gear_compression_rate"
    deployment_key: str = "gear_deployment"
    active_threshold: float = 0.95
    friction_velocity_scale: float = 0.1
    source: str = "dynamic-landing-gear"

    def __post_init__(self)->None:
        if not self.legs: raise ValueError("at least one gear leg required")
        if not 0<=self.active_threshold<=1: raise ValueError("active_threshold must lie in [0,1]")
        if self.friction_velocity_scale<=0: raise ValueError("friction_velocity_scale must be positive")

    @property
    def leg_count(self)->int: return len(self.legs)

    def _nominal_kinematics(self,state:StateView,leg:DynamicGearLeg):
        mp=self.mass_properties.evaluate(state); R=body_to_inertial_matrix(state.get("attitude"))
        arm=leg.foot_b-mp.cg_b
        foot_i=np.asarray(state.get("position"),dtype=float)+R@arm
        omega=np.asarray(state.get("angular_rate"),dtype=float)
        foot_v=np.asarray(state.get("velocity"),dtype=float)+R@np.cross(omega,arm)
        return foot_i,foot_v,arm,R

    def derivatives(self,state:StateView)->dict[str,np.ndarray]:
        c=np.asarray(state.get(self.compression_key),dtype=float); cd=np.asarray(state.get(self.rate_key),dtype=float)
        if c.shape!=(self.leg_count,) or cd.shape!=(self.leg_count,): raise ValueError("gear state shape mismatch")
        deployment=float(state.get(self.deployment_key)); active=deployment>=self.active_threshold
        cdd=np.zeros(self.leg_count)
        for i,leg in enumerate(self.legs):
            foot_i,foot_v,_,_=self._nominal_kinematics(state,leg)
            ts=self.terrain.query(foot_i,state.time)
            target=np.clip(max(0.0,-ts.agl) if active else 0.0,0.0,leg.max_compression)
            wn=np.sqrt(leg.stiffness/leg.effective_mass)
            zeta=leg.damping/(2*np.sqrt(leg.stiffness*leg.effective_mass))
            cdd[i]=wn*wn*(target-c[i])-2*zeta*wn*cd[i]
            if c[i]<=0 and cd[i]<0 and cdd[i]<0: cdd[i]=0
            if c[i]>=leg.max_compression and cd[i]>0 and cdd[i]>0: cdd[i]=0
        c_dot=cd.copy()
        c_dot[(c<=0)&(c_dot<0)]=0; c_dot[(c>=np.array([l.max_compression for l in self.legs]))&(c_dot>0)]=0
        return {self.compression_key:c_dot,self.rate_key:cdd}

    def wrench(self,state:StateView)->Wrench:
        c=np.asarray(state.get(self.compression_key),dtype=float); cd=np.asarray(state.get(self.rate_key),dtype=float)
        deployment=float(state.get(self.deployment_key)); active=deployment>=self.active_threshold
        if not active: return Wrench.zero(self.source)
        total_f=np.zeros(3); total_m=np.zeros(3); mp=self.mass_properties.evaluate(state)
        for i,leg in enumerate(self.legs):
            foot_i0,foot_v0,arm0,R=self._nominal_kinematics(state,leg)
            arm=arm0 + c[i]*leg.compression_axis_b
            foot_i=foot_i0 + R@(c[i]*leg.compression_axis_b)
            foot_v=foot_v0 + R@(cd[i]*leg.compression_axis_b)
            ts=self.terrain.query(foot_i,state.time)
            # No strut load until terrain engagement or residual compression exists.
            if ts.agl>0 and c[i]<=0: continue
            fn=max(0.0,leg.stiffness*c[i]+leg.damping*cd[i])
            force=fn*ts.normal_i
            rel_v=foot_v-ts.surface_velocity_i
            vn=float(np.dot(rel_v,ts.normal_i)); vt=rel_v-vn*ts.normal_i; vm=float(np.linalg.norm(vt))
            if vm>0 and fn>0 and leg.friction_coefficient>0:
                force += -leg.friction_coefficient*fn*np.tanh(vm/self.friction_velocity_scale)*vt/vm
            total_f+=force; total_m+=np.cross(arm,R.T@force)
        return Wrench(total_f,total_m,self.source)
