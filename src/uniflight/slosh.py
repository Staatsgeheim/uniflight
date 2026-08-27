from __future__ import annotations
from dataclasses import dataclass
from typing import Callable
import numpy as np

from .frames import body_to_inertial_matrix
from .mass_properties import MassPropertiesModel
from .state import StateView
from .wrenches import Wrench

AccelerationProvider = Callable[[StateView], np.ndarray]


@dataclass(frozen=True, slots=True)
class LinearSloshSubsystem:
    """Linear transverse slosh modes with reaction wrench on the rigid vehicle.

    Each mode represents a slosh mass moving along a user-specified body axis.
    The relative coordinate obeys

        xdd + 2*zeta*wn*xd + wn^2*x = -a_base dot axis.

    The spring/damper reaction on the tank is returned as a body force and an
    inertial force/moment wrench.  Rotational fluid effects are deliberately
    outside this low-order reference closure.
    """
    slosh_mass: np.ndarray | float
    natural_frequency_hz: np.ndarray | float
    damping_ratio: np.ndarray | float
    axes_b: np.ndarray
    tank_position_b: np.ndarray
    mass_properties: MassPropertiesModel
    base_acceleration_b: np.ndarray | AccelerationProvider | None = None
    displacement_key: str = "slosh_displacement"
    velocity_key: str = "slosh_velocity"
    source: str = "linear-slosh"

    def __post_init__(self) -> None:
        axes = np.asarray(self.axes_b, dtype=float)
        if axes.ndim == 1:
            axes = axes.reshape(1,3)
        if axes.ndim != 2 or axes.shape[1] != 3 or not np.all(np.isfinite(axes)):
            raise ValueError("axes_b must be a finite (n_modes,3) matrix")
        norms = np.linalg.norm(axes, axis=1)
        if np.any(norms <= 0):
            raise ValueError("slosh axes must be nonzero")
        axes = axes / norms[:,None]
        n = axes.shape[0]
        def vec(v, name, positive=False, nonnegative=False):
            a=np.asarray(v,dtype=float)
            if a.ndim==0: a=np.full(n,float(a))
            if a.shape!=(n,) or not np.all(np.isfinite(a)): raise ValueError(f"{name} must be scalar or {n}-vector")
            if positive and np.any(a<=0): raise ValueError(f"{name} must be positive")
            if nonnegative and np.any(a<0): raise ValueError(f"{name} must be non-negative")
            return a
        sm=vec(self.slosh_mass,"slosh_mass",positive=True)
        f=vec(self.natural_frequency_hz,"natural_frequency_hz",positive=True)
        z=vec(self.damping_ratio,"damping_ratio",nonnegative=True)
        pos=np.asarray(self.tank_position_b,dtype=float)
        if pos.shape!=(3,) or not np.all(np.isfinite(pos)): raise ValueError("tank_position_b must be a finite 3-vector")
        if self.base_acceleration_b is not None and not callable(self.base_acceleration_b):
            ba=np.asarray(self.base_acceleration_b,dtype=float)
            if ba.shape!=(3,) or not np.all(np.isfinite(ba)): raise ValueError("base_acceleration_b must be finite 3-vector or callable")
            object.__setattr__(self,"base_acceleration_b",ba.copy())
        object.__setattr__(self,"axes_b",axes)
        object.__setattr__(self,"slosh_mass",sm)
        object.__setattr__(self,"natural_frequency_hz",f)
        object.__setattr__(self,"damping_ratio",z)
        object.__setattr__(self,"tank_position_b",pos.copy())

    @property
    def mode_count(self) -> int: return self.axes_b.shape[0]
    @property
    def omega_n(self) -> np.ndarray: return 2*np.pi*self.natural_frequency_hz

    def _base_accel(self,state:StateView)->np.ndarray:
        if self.base_acceleration_b is None: return np.zeros(3)
        a=self.base_acceleration_b(state) if callable(self.base_acceleration_b) else self.base_acceleration_b
        a=np.asarray(a,dtype=float)
        if a.shape!=(3,) or not np.all(np.isfinite(a)): raise ValueError("base acceleration provider returned invalid vector")
        return a

    def derivatives(self,state:StateView)->dict[str,np.ndarray]:
        x=np.asarray(state.get(self.displacement_key),dtype=float)
        xd=np.asarray(state.get(self.velocity_key),dtype=float)
        if x.shape!=(self.mode_count,) or xd.shape!=(self.mode_count,): raise ValueError("slosh state shape mismatch")
        wn=self.omega_n
        forcing=-(self.axes_b @ self._base_accel(state))
        xdd=forcing-2*self.damping_ratio*wn*xd-wn*wn*x
        return {self.displacement_key:xd,self.velocity_key:xdd}

    def reaction_force_b(self,state:StateView)->np.ndarray:
        x=np.asarray(state.get(self.displacement_key),dtype=float)
        xd=np.asarray(state.get(self.velocity_key),dtype=float)
        wn=self.omega_n
        scalar=self.slosh_mass*(2*self.damping_ratio*wn*xd+wn*wn*x)
        return np.sum(scalar[:,None]*self.axes_b,axis=0)

    def wrench(self,state:StateView)->Wrench:
        fb=self.reaction_force_b(state)
        R=body_to_inertial_matrix(state.get("attitude"))
        mp=self.mass_properties.evaluate(state)
        arm=self.tank_position_b-mp.cg_b
        return Wrench(R@fb,np.cross(arm,fb),self.source)

    def modal_energy(self,state:StateView)->float:
        x=np.asarray(state.get(self.displacement_key),dtype=float)
        xd=np.asarray(state.get(self.velocity_key),dtype=float)
        wn=self.omega_n
        return float(0.5*np.sum(self.slosh_mass*(xd*xd+(wn*x)**2)))
