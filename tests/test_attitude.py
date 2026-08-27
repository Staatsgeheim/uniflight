import numpy as np
from uniflight import core_6dof_schema, QuaternionKinematics, DynamicsAssembler, SimulationEngine, ScipyIVPIntegrator, SolverConfig, quat_normalize

class Static6DOF:
    def derivatives(self, state):
        return {"position": state.get("velocity"), "velocity": np.zeros(3), "mass":0.0, "angular_rate":np.zeros(3)}


def test_004_quaternion_constant_rate():
    schema = core_6dof_schema()
    omega = np.array([0.,0.,0.3]); tf = 7.0
    y0 = schema.pack({"position":np.zeros(3),"velocity":np.zeros(3),"attitude":np.array([1.,0,0,0]),"angular_rate":omega,"mass":1.})
    asm = DynamicsAssembler(schema,[Static6DOF(), QuaternionKinematics()])
    integ = ScipyIVPIntegrator(SolverConfig(rtol=2e-13,atol=1e-14,max_step=.02))
    res = SimulationEngine(asm.rhs,integ).run((0,tf),y0)
    q = quat_normalize(schema.unpack(res.states[-1])["attitude"])
    angle = omega[2]*tf
    expected = np.array([np.cos(angle/2),0,0,np.sin(angle/2)])
    # q and -q represent same attitude.
    err = min(np.linalg.norm(q-expected), np.linalg.norm(q+expected))
    assert err < 5e-12
