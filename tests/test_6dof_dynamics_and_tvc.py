import math
import numpy as np

from uniflight import (
    Wrench, ConstantMassProperties, RigidBody6DOFDynamics, QuaternionKinematics,
    DynamicsAssembler, core_6dof_schema, SimulationEngine, ScipyIVPIntegrator, SolverConfig,
    SphericalBody, PlanetaryEnvironment, VacuumAtmosphere, GimballedRocketEngine,
    StateView, body_to_inertial_matrix,
)


class ConstantMoment:
    def __init__(self, moment_b): self.moment_b=np.asarray(moment_b,float)
    def wrench(self,state): return Wrench(np.zeros(3),self.moment_b,"test-moment")


def _initial(schema, omega=np.zeros(3), mass=100.):
    return schema.pack({"position":np.zeros(3),"velocity":np.zeros(3),
                        "attitude":np.array([1.,0,0,0]),"angular_rate":np.asarray(omega,float),"mass":mass})


def test_023_constant_body_torque_matches_euler_equation_from_rest():
    schema=core_6dof_schema(); I=np.diag([4.,5.,10.]); mp=ConstantMassProperties(I)
    dyn=RigidBody6DOFDynamics(mp,wrench_models=(ConstantMoment([0,0,20.]),))
    asm=DynamicsAssembler(schema,[dyn,QuaternionKinematics()])
    res=SimulationEngine(asm.rhs,ScipyIVPIntegrator(SolverConfig(rtol=1e-12,atol=1e-13,max_step=.01))).run((0,2.),_initial(schema))
    omega=schema.unpack(res.states[-1])["angular_rate"]
    np.testing.assert_allclose(omega,[0,0,4.],rtol=2e-12,atol=2e-12)


def test_024_torque_free_rigid_body_conserves_energy_and_inertial_angular_momentum():
    schema=core_6dof_schema(); I=np.diag([3.,5.,8.]); mp=ConstantMassProperties(I)
    w0=np.array([.4,.7,1.1]); y0=_initial(schema,w0)
    dyn=RigidBody6DOFDynamics(mp)
    asm=DynamicsAssembler(schema,[dyn,QuaternionKinematics()])
    integ=ScipyIVPIntegrator(SolverConfig(rtol=2e-12,atol=2e-13,max_step=.005))
    res=SimulationEngine(asm.rhs,integ).run((0,8.),y0)
    s0=schema.unpack(y0); sf=schema.unpack(res.states[-1])
    E0=.5*w0@(I@w0); Ef=.5*sf["angular_rate"]@(I@sf["angular_rate"])
    H0=body_to_inertial_matrix(s0["attitude"])@(I@w0)
    Hf=body_to_inertial_matrix(sf["attitude"])@(I@sf["angular_rate"])
    assert abs(Ef-E0)/E0 < 2e-10
    assert np.linalg.norm(Hf-H0)/np.linalg.norm(H0) < 3e-10
    assert abs(np.linalg.norm(sf["attitude"])-1) < 2e-11


def test_025_gimballed_engine_generates_body_force_inertial_force_and_mount_moment():
    body=SphericalBody(1e10,1e5); env=PlanetaryEnvironment(body,VacuumAtmosphere())
    mp=ConstantMassProperties(np.diag([100.,200.,300.]),np.zeros(3))
    yaw=.1
    engine=GimballedRocketEngine(env,mp,exhaust_velocity=1000.,mdot_exhaust=2.,
                                 mount_position_b=np.array([-2.,0,0]),yaw_gimbal=yaw,dry_mass=10.)
    schema=core_6dof_schema()
    y=schema.pack({"position":np.array([body.radius,0,0]),"velocity":np.zeros(3),
                   "attitude":np.array([1.,0,0,0]),"angular_rate":np.zeros(3),"mass":100.})
    state=StateView(0,y,schema); e=engine.evaluate(state)
    expected=2000*np.array([math.cos(yaw),math.sin(yaw),0.])
    np.testing.assert_allclose(e.force_b,expected,rtol=1e-14,atol=1e-12)
    np.testing.assert_allclose(e.force_i,expected,rtol=1e-14,atol=1e-12)
    np.testing.assert_allclose(e.moment_b_about_cg,np.cross([-2.,0,0],expected),rtol=1e-14,atol=1e-12)
    assert engine.derivatives(state)["mass"] == -2.0
