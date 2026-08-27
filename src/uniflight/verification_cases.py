from __future__ import annotations

import math
import platform
import sys
import numpy as np
from scipy.integrate import solve_ivp

from ._version import __version__
from .events import Event
from .frames import FrameGraph, Transform, quat_normalize
from .gravity import PointMassGravity
from .integrators import FixedStepRK4Config, FixedStepRK4Integrator, ScipyIVPIntegrator, SolverConfig
from .separation import separate_two_rigid_bodies
from .verification import TolerancePolicy, VerificationReport, VerificationResult, observed_order, scalar_result


def _rk4_scalar(rhs, t0: float, y0: float, tf: float, h: float) -> float:
    t, y = float(t0), float(y0)
    while t < tf - 1e-15:
        dt = min(h, tf - t)
        k1 = rhs(t, y)
        k2 = rhs(t + 0.5*dt, y + 0.5*dt*k1)
        k3 = rhs(t + 0.5*dt, y + 0.5*dt*k2)
        k4 = rhs(t + dt, y + dt*k3)
        y += dt*(k1 + 2*k2 + 2*k3 + k4)/6.0
        t += dt
    return float(y)


def case_rk4_order() -> VerificationResult:
    hs = np.array([0.2, 0.1, 0.05, 0.025])
    exact = math.e
    errs = np.array([abs(_rk4_scalar(lambda _t,y:y,0.0,1.0,1.0,h)-exact) for h in hs])
    order = observed_order(hs, errs)
    return scalar_result("O-001", "RK4 manufactured exponential solution", "convergence/MMS", actual=order, reference=4.0, policy=TolerancePolicy(absolute=0.2), details={"steps": hs.tolist(), "errors": errs.tolist()})


def case_adaptive_mms() -> VerificationResult:
    # Manufactured exact y=sin(t), RHS=cos(t).
    sol = solve_ivp(lambda t,y: [math.cos(t)], (0.0, 7.0), [0.0], method="DOP853", rtol=1e-12, atol=1e-14, dense_output=True)
    ts = np.linspace(0,7,101)
    err = float(np.max(np.abs(sol.sol(ts)[0]-np.sin(ts))))
    return scalar_result("O-002", "Adaptive manufactured sine solution", "MMS", actual=err, reference=0.0, policy=TolerancePolicy(absolute=2e-11), details={"nfev": int(sol.nfev)})


def case_tsiolkovsky() -> VerificationResult:
    ve, m0, mf = 3000.0, 500.0, 200.0
    exact = ve*math.log(m0/mf)
    # Independent quadrature of -ve dm/m.
    from scipy.integrate import quad
    numeric = quad(lambda m: -ve/m, m0, mf, epsabs=1e-11, epsrel=1e-13)[0]
    rel = abs(numeric-exact)/exact
    return scalar_result("O-003", "Tsiolkovsky limiting case", "analytical", actual=rel, reference=0.0, policy=TolerancePolicy(absolute=2e-13), details={"delta_v": numeric})


def case_kepler() -> VerificationResult:
    mu = 4.0e13
    r0 = 2.0e6
    v0 = math.sqrt(mu/r0)
    period = 2*math.pi*math.sqrt(r0**3/mu)
    y0 = np.array([r0,0,0,0,v0,0], float)
    def rhs(_t,y):
        r=y[:3]; v=y[3:]; rn=np.linalg.norm(r)
        return np.r_[v,-mu*r/rn**3]
    sol = solve_ivp(rhs,(0,period),y0,method="DOP853",rtol=2e-12,atol=1e-7)
    yf=sol.y[:,-1]
    pos_rel=np.linalg.norm(yf[:3]-y0[:3])/r0
    e0=0.5*np.dot(y0[3:],y0[3:])-mu/np.linalg.norm(y0[:3])
    ef=0.5*np.dot(yf[3:],yf[3:])-mu/np.linalg.norm(yf[:3])
    e_rel=abs(ef-e0)/abs(e0)
    metric=max(pos_rel,e_rel)
    return scalar_result("O-004", "One-period circular Kepler orbit", "analytical/conservation", actual=metric, reference=0.0, policy=TolerancePolicy(absolute=2e-9), details={"position_relative_error":pos_rel,"energy_relative_error":e_rel,"period_s":period})


def case_gravity_jacobian() -> VerificationResult:
    g=PointMassGravity(3.986e14); r=np.array([7.2e6,-1.1e6,0.8e6]); J=g.jacobian(r)
    h=1.0
    Jfd=np.column_stack([(g.acceleration(r+h*np.eye(3)[i])-g.acceleration(r-h*np.eye(3)[i]))/(2*h) for i in range(3)])
    rel=np.linalg.norm(J-Jfd)/np.linalg.norm(J)
    return scalar_result("O-005", "Point-mass gravity Jacobian", "derivative", actual=rel, reference=0.0, policy=TolerancePolicy(absolute=5e-8))


def _quat_rhs(_t,q,omega):
    wx,wy,wz=omega
    O=np.array([[0,-wx,-wy,-wz],[wx,0,wz,-wy],[wy,-wz,0,wx],[wz,wy,-wx,0]],float)
    return 0.5*O@q


def case_quaternion() -> VerificationResult:
    omega=np.array([0,0,0.2]); tf=7.5; q0=np.array([1,0,0,0],float)
    sol=solve_ivp(lambda t,q:_quat_rhs(t,q,omega),(0,tf),q0,method="DOP853",rtol=1e-13,atol=1e-15)
    q=quat_normalize(sol.y[:,-1]); th=omega[2]*tf
    exact=np.array([math.cos(th/2),0,0,math.sin(th/2)])
    if np.dot(q,exact)<0: q=-q
    err=np.linalg.norm(q-exact)
    return scalar_result("O-006", "Constant-rate quaternion kinematics", "analytical", actual=err, reference=0.0, policy=TolerancePolicy(absolute=2e-11))


def case_symmetric_torque_free() -> VerificationResult:
    # Axisymmetric body I1=I2=A, I3=C. wz constant and transverse rate precesses exactly.
    A,C=2.0,1.0; w0=np.array([0.1,0.2,0.3]); tf=10.0
    I=np.diag([A,A,C])
    def rhs(_t,w): return np.linalg.solve(I,-np.cross(w,I@w))
    sol=solve_ivp(rhs,(0,tf),w0,method="DOP853",rtol=1e-12,atol=1e-14)
    nu=(C-A)*w0[2]/A
    z0=w0[0]+1j*w0[1]
    z=z0*np.exp(1j*nu*tf)
    exact=np.array([z.real,z.imag,w0[2]])
    err=np.linalg.norm(sol.y[:,-1]-exact)
    return scalar_result("O-007", "Axisymmetric torque-free rigid body", "analytical 6-DOF", actual=err, reference=0.0, policy=TolerancePolicy(absolute=2e-10))


def case_event_root() -> VerificationResult:
    rhs=lambda _t,y: np.array([2.0])
    event=Event("x=3",lambda _t,y: y[0]-3.0,direction=1.0)
    a=ScipyIVPIntegrator(SolverConfig(rtol=1e-12,atol=1e-14,max_step=0.2)).solve_segment(rhs,(0,5),np.array([0.0]),[event])
    b=FixedStepRK4Integrator(FixedStepRK4Config(step=0.1,event_time_tolerance=1e-10)).solve_segment(rhs,(0,5),np.array([0.0]),[event])
    ta=float(a.t_events[0][0]); tb=float(b.t_events[0][0]); err=max(abs(ta-1.5),abs(tb-1.5),abs(ta-tb))
    return scalar_result("O-008", "Hybrid event-root timing", "hybrid", actual=err, reference=0.0, policy=TolerancePolicy(absolute=2e-8), details={"dop853":ta,"rk4":tb})


def case_cross_integrator() -> VerificationResult:
    rhs=lambda _t,y: np.array([y[1],2.5])
    y0=np.array([1.0,-3.0]); tf=12.0
    a=ScipyIVPIntegrator(SolverConfig(rtol=1e-12,atol=1e-14)).solve_segment(rhs,(0,tf),y0)
    b=FixedStepRK4Integrator(FixedStepRK4Config(step=0.1)).solve_segment(rhs,(0,tf),y0)
    err=np.linalg.norm(a.y[:,-1]-b.y[:,-1],ord=np.inf)
    return scalar_result("O-009", "DOP853/RK4 constant-acceleration comparison", "cross-integrator", actual=err, reference=0.0, policy=TolerancePolicy(absolute=2e-10))


def case_separation() -> VerificationResult:
    m1,m2=60.,40.; M=m1+m2
    r1=np.array([0.8,0,0]); r2=-(m1/m2)*r1
    I1=np.diag([5.,6.,7.]); I2=np.diag([4.,5.,6.])
    def pa(m,r): return m*((r@r)*np.eye(3)-np.outer(r,r))
    I0=I1+I2+pa(m1,r1)+pa(m2,r2)
    res=separate_two_rigid_bodies(parent_mass=M,parent_position_i=np.array([1e6,20,30.]),parent_velocity_i=np.array([10.,20.,30.]),parent_attitude_bi=np.array([1.,0,0,0]),parent_angular_rate_b=np.array([.02,-.03,.04]),parent_inertia_b=I0,retained_mass=m1,detached_mass=m2,retained_offset_b=r1,detached_offset_b=r2,retained_inertia_b=I1,detached_inertia_b=I2,relative_separation_velocity_i=np.array([.7,-.2,.1]))
    err=max(np.linalg.norm(res.linear_momentum_error_i),np.linalg.norm(res.angular_momentum_error_i))
    return scalar_result("O-010", "Rigid two-body separation momentum conservation", "hybrid/conservation", actual=err, reference=0.0, policy=TolerancePolicy(absolute=2e-10))


def case_frame_roundtrip() -> VerificationResult:
    ang=.71; c,s=math.cos(ang),math.sin(ang); R=np.array([[c,-s,0],[s,c,0],[0,0,1.]])
    fg=FrameGraph(); fg.add_transform("A","B",Transform(R,np.array([3.,-2.,.5]),np.array([0,0,.1])))
    p=np.array([1.1,-4.2,.3]); v=np.array([.2,.5,-.7])
    ab=fg.transform("A","B",0); ba=fg.transform("B","A",0)
    err=max(np.linalg.norm(ba.point(ab.point(p))-p),np.linalg.norm(ba.vector(ab.vector(v))-v))
    return scalar_result("O-011", "Frame graph round trip", "coordinate-transform", actual=err, reference=0.0, policy=TolerancePolicy(absolute=2e-13))


def case_quaternion_longrun() -> VerificationResult:
    omega=np.array([.01,-.015,.02]); q0=np.array([1.,0,0,0]); tf=4000.
    sol=solve_ivp(lambda t,q:_quat_rhs(t,q,omega),(0,tf),q0,method="DOP853",rtol=1e-12,atol=1e-14)
    normerr=abs(np.linalg.norm(sol.y[:,-1])-1.0)
    return scalar_result("O-012", "Long-run quaternion norm stability", "numerical-stability", actual=normerr, reference=0.0, policy=TolerancePolicy(absolute=2e-10), details={"duration_s":tf})


def external_manifests() -> tuple[VerificationResult,...]:
    return (
        VerificationResult("NESC-ATM-01","NESC dropped sphere, dragless","external-reference","SKIP",details={"reason":"NASA/NESC reference trajectory not bundled","source":"https://nescacademy.nasa.gov/flightsim/2015"}),
        VerificationResult("NESC-ORB-08B","NESC torque-free rotation, non-zero rate","external-reference","SKIP",details={"reason":"NASA/NESC reference trajectory not bundled","source":"https://nescacademy.nasa.gov/flightsim/2015"}),
    )


def run_builtin_verification() -> VerificationReport:
    cases=(case_rk4_order,case_adaptive_mms,case_tsiolkovsky,case_kepler,case_gravity_jacobian,case_quaternion,case_symmetric_torque_free,case_event_root,case_cross_integrator,case_separation,case_frame_roundtrip,case_quaternion_longrun)
    results=[]
    for fn in cases:
        try: results.append(fn())
        except Exception as exc:
            results.append(VerificationResult(fn.__name__,fn.__name__,"internal","FAIL",details={"exception":repr(exc)}))
    results.extend(external_manifests())
    return VerificationReport(tuple(results),metadata={"uniflight_version":__version__,"python":sys.version.split()[0],"platform":platform.platform(),"suite":"Milestone O formal verification"})
