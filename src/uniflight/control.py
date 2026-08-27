from __future__ import annotations
from dataclasses import dataclass
import numpy as np

from .actuators import GNCCommandBus
from .environment import PlanetaryEnvironment
from .frames import matrix_to_quat, quat_to_matrix
from .sensors import AttitudeRateMeasurement
from .state import StateView


@dataclass(frozen=True, slots=True)
class ThrustGuidanceCommand:
    throttle: float
    direction_i: np.ndarray
    thrust_acceleration_i: np.ndarray
    desired_total_acceleration_i: np.ndarray
    position_error_i: np.ndarray
    velocity_error_i: np.ndarray


@dataclass(frozen=True, slots=True)
class VectorLandingGuidance:
    """Planet-agnostic 3-D terminal guidance with gravity feed-forward.

    ``terminal_sink_rate`` is an optional anti-hover contact mode.  Inside
    ``terminal_zone`` the commanded target velocity acquires a component along
    local gravity (toward the surface), preventing a PD controller with
    multiplicative thrust error from settling into a static hover above the
    touchdown surface.  Defaults preserve the Milestone-F controller exactly.
    """
    environment: PlanetaryEnvironment
    target_position_i: np.ndarray
    kp_position: float
    kd_velocity: float
    max_thrust_acceleration: float
    target_velocity_i: np.ndarray | None = None
    terminal_sink_rate: float = 0.0
    terminal_zone: float = 0.0

    def __post_init__(self) -> None:
        target = np.asarray(self.target_position_i, dtype=float)
        tv = None if self.target_velocity_i is None else np.asarray(self.target_velocity_i, dtype=float)
        if target.shape != (3,) or not np.all(np.isfinite(target)):
            raise ValueError("target_position_i must be finite 3-vector")
        if tv is not None and (tv.shape != (3,) or not np.all(np.isfinite(tv))):
            raise ValueError("target_velocity_i must be finite 3-vector")
        if any((not np.isfinite(v) or v <= 0) for v in (self.kp_position,self.kd_velocity,self.max_thrust_acceleration)):
            raise ValueError("guidance gains and acceleration limit must be positive")
        if (not np.isfinite(self.terminal_sink_rate) or self.terminal_sink_rate < 0 or
                not np.isfinite(self.terminal_zone) or self.terminal_zone < 0):
            raise ValueError("terminal sink parameters must be finite and non-negative")
        if self.terminal_sink_rate > 0 and self.terminal_zone <= 0:
            raise ValueError("terminal_zone must be positive when terminal_sink_rate is enabled")
        object.__setattr__(self,"target_position_i",target.copy())
        if tv is not None: object.__setattr__(self,"target_velocity_i",tv.copy())

    def evaluate(self, estimate_rv: np.ndarray, available_thrust_acceleration: float) -> ThrustGuidanceCommand:
        x = np.asarray(estimate_rv, dtype=float)
        if x.shape != (6,): raise ValueError("estimate_rv must contain [r,v]")
        r,v=x[:3],x[3:]
        tv = self.environment.body.rotational_velocity_i(self.target_position_i) if self.target_velocity_i is None else self.target_velocity_i
        er = self.target_position_i-r
        g = self.environment.query(r,0.0).gravity_i
        gmag = float(np.linalg.norm(g))
        # Local outward direction is opposite gravity.  This remains independent
        # of any Earth-specific radial-axis convention and works for any gravity
        # model that provides a meaningful local vertical.
        if self.terminal_sink_rate > 0 and gmag > 1e-12:
            outward = -g/gmag
            normal_distance = float(np.dot(r-self.target_position_i, outward))
            if normal_distance <= self.terminal_zone:
                tv = np.asarray(tv, dtype=float) - self.terminal_sink_rate*outward
        ev = tv-v
        desired_total = self.kp_position*er + self.kd_velocity*ev
        thrust_accel = desired_total-g
        mag=float(np.linalg.norm(thrust_accel))
        cap=min(self.max_thrust_acceleration,float(available_thrust_acceleration))
        if mag > cap > 0:
            thrust_accel=thrust_accel*(cap/mag); mag=cap
        if mag <= 1e-15:
            direction=np.array([1.,0.,0.]); throttle=0.0
        else:
            direction=thrust_accel/mag
            throttle=0.0 if available_thrust_acceleration<=0 else float(np.clip(mag/available_thrust_acceleration,0,1))
        return ThrustGuidanceCommand(throttle,direction,thrust_accel,desired_total,er,ev)


def quaternion_align_body_x(direction_i: np.ndarray, up_hint_i: np.ndarray | None = None) -> np.ndarray:
    """Construct B->I attitude whose +x_B axis points along ``direction_i``."""
    x = np.asarray(direction_i,dtype=float); nx=np.linalg.norm(x)
    if x.shape!=(3,) or not np.isfinite(nx) or nx==0: raise ValueError("direction_i invalid")
    x=x/nx
    hint=np.array([0.,0.,1.]) if up_hint_i is None else np.asarray(up_hint_i,dtype=float)
    if hint.shape!=(3,) or not np.all(np.isfinite(hint)): raise ValueError("up_hint_i invalid")
    # Pick body +z as close as possible to hint while perpendicular to +x.
    z=hint-np.dot(hint,x)*x
    if np.linalg.norm(z)<1e-9:
        alt=np.array([0.,1.,0.])
        z=alt-np.dot(alt,x)*x
    z=z/np.linalg.norm(z)
    y=np.cross(z,x); y=y/np.linalg.norm(y)
    z=np.cross(x,y)
    R=np.column_stack((x,y,z))
    return matrix_to_quat(R)


@dataclass(frozen=True, slots=True)
class QuaternionPDController:
    kp: float
    kd: float
    max_torque: float | np.ndarray

    def __post_init__(self)->None:
        lim=np.asarray(self.max_torque,dtype=float)
        if lim.ndim==0: lim=np.full(3,float(lim))
        if self.kp<=0 or self.kd<=0 or lim.shape!=(3,) or np.any(lim<=0) or not np.all(np.isfinite(lim)):
            raise ValueError("attitude-controller parameters invalid")
        object.__setattr__(self,"max_torque",lim.copy())

    def command(self, attitude: np.ndarray, angular_rate_b: np.ndarray, desired_attitude: np.ndarray,
                desired_rate_b: np.ndarray | None=None)->np.ndarray:
        R=quat_to_matrix(attitude); Rd=quat_to_matrix(desired_attitude)
        E=0.5*(Rd.T@R-R.T@Rd)
        eR=np.array([E[2,1],E[0,2],E[1,0]])
        wd=np.zeros(3) if desired_rate_b is None else np.asarray(desired_rate_b,dtype=float)
        torque=-self.kp*eR-self.kd*(np.asarray(angular_rate_b,dtype=float)-wd)
        return np.clip(torque,-self.max_torque,self.max_torque)


@dataclass(slots=True)
class AdaptiveThrustScaleEstimator:
    """Scalar online estimate of effective thrust relative to the nominal model.

    The estimate uses sampled navigation velocity increments.  It projects the
    observed non-gravitational acceleration onto the previous commanded thrust
    direction and compares it with the previous nominal commanded thrust
    acceleration.  A bounded exponential update rejects measurement noise.

    The estimator is deliberately lightweight: it is not a replacement for an
    augmented-state propulsion EKF, but it removes persistent multiplicative
    thrust bias without putting mutable logic inside the continuous ODE RHS.
    """
    estimate: float = 1.0
    adaptation_gain: float = 0.08
    minimum: float = 0.8
    maximum: float = 1.2
    min_commanded_acceleration: float = 0.5
    innovation_limit: float = 0.25
    _previous_time: float | None = None
    _previous_velocity_i: np.ndarray | None = None
    _previous_thrust_acceleration_i: np.ndarray | None = None

    def __post_init__(self) -> None:
        vals=(self.estimate,self.adaptation_gain,self.minimum,self.maximum,self.min_commanded_acceleration,self.innovation_limit)
        if not all(np.isfinite(v) for v in vals):
            raise ValueError("thrust estimator parameters must be finite")
        if not (0 < self.adaptation_gain <= 1):
            raise ValueError("adaptation_gain must lie in (0,1]")
        if not (0 < self.minimum <= self.estimate <= self.maximum):
            raise ValueError("invalid thrust-scale bounds/initial estimate")
        if self.min_commanded_acceleration < 0 or self.innovation_limit <= 0:
            raise ValueError("estimator thresholds must be non-negative/positive")

    def observe(self, time: float, estimate_rv: np.ndarray, gravity_i: np.ndarray) -> float:
        t=float(time); x=np.asarray(estimate_rv,dtype=float); g=np.asarray(gravity_i,dtype=float)
        if x.shape!=(6,) or g.shape!=(3,):
            raise ValueError("invalid thrust-estimator observation")
        if self._previous_time is not None and self._previous_velocity_i is not None and self._previous_thrust_acceleration_i is not None:
            dt=t-self._previous_time
            a_cmd=np.asarray(self._previous_thrust_acceleration_i,dtype=float)
            amag=float(np.linalg.norm(a_cmd))
            if dt > 1e-9 and amag >= self.min_commanded_acceleration:
                a_obs=(x[3:]-self._previous_velocity_i)/dt
                direction=a_cmd/amag
                measured=float(np.dot(a_obs-g,direction))/amag
                if np.isfinite(measured) and measured > 0:
                    # Bound each innovation before low-pass adaptation so a
                    # single noisy velocity sample cannot destabilize guidance.
                    error=float(np.clip(measured-self.estimate,-self.innovation_limit,self.innovation_limit))
                    self.estimate=float(np.clip(self.estimate+self.adaptation_gain*error,self.minimum,self.maximum))
        self._previous_time=t
        self._previous_velocity_i=x[3:].copy()
        return self.estimate

    def commit_command(self, thrust_acceleration_i: np.ndarray) -> None:
        a=np.asarray(thrust_acceleration_i,dtype=float)
        if a.shape!=(3,) or not np.all(np.isfinite(a)):
            raise ValueError("thrust acceleration command must be finite 3-vector")
        self._previous_thrust_acceleration_i=a.copy()


@dataclass(frozen=True, slots=True)
class GNCDecision:
    throttle_command: float
    torque_command_b: np.ndarray
    thrust_direction_i: np.ndarray
    thrust_acceleration_i: np.ndarray
    desired_attitude: np.ndarray


@dataclass(frozen=True, slots=True)
class LandingGNCController:
    guidance: VectorLandingGuidance
    attitude_controller: QuaternionPDController
    engine_exhaust_velocity: float
    engine_mdot_exhaust: float
    bus: GNCCommandBus
    thrust_scale_estimator: AdaptiveThrustScaleEstimator | None = None

    def update(self, truth: StateView, estimate_rv: np.ndarray,
               attitude_measurement: AttitudeRateMeasurement) -> GNCDecision:
        mass=float(truth.get("mass"))
        env=self.guidance.environment.query(estimate_rv[:3],truth.time)
        scale=1.0
        if self.thrust_scale_estimator is not None:
            scale=self.thrust_scale_estimator.observe(truth.time, estimate_rv, env.gravity_i)
        max_thrust=max(0.0,self.engine_mdot_exhaust*self.engine_exhaust_velocity*scale)
        amax=max_thrust/mass
        guide=self.guidance.evaluate(estimate_rv,amax)
        desired_q=quaternion_align_body_x(guide.direction_i)
        torque=self.attitude_controller.command(attitude_measurement.attitude,
                                                attitude_measurement.angular_rate,desired_q)
        # Use TVC for the fast translation loop while the attitude loop slews the
        # vehicle toward the same desired thrust direction.  The physical
        # gimbal actuator states apply position/rate limits in the plant.
        R_bi = quat_to_matrix(attitude_measurement.attitude).T
        desired_b = R_bi @ guide.direction_i
        yaw_cmd = float(np.arctan2(desired_b[1], desired_b[0]))
        pitch_cmd = float(np.arctan2(-desired_b[2], np.hypot(desired_b[0], desired_b[1])))
        self.bus.set(throttle=guide.throttle, pitch_gimbal=pitch_cmd, yaw_gimbal=yaw_cmd, torque_b=torque)
        if self.thrust_scale_estimator is not None:
            self.thrust_scale_estimator.commit_command(guide.thrust_acceleration_i)
        return GNCDecision(guide.throttle,torque,guide.direction_i,guide.thrust_acceleration_i,desired_q)
