import numpy as np
from uniflight import core_3dof_schema, DynamicsAssembler, SimulationEngine, ScipyIVPIntegrator, SolverConfig, Event, EventAction

class ConstantVelocity:
    def derivatives(self,state):
        return {"position":state.get("velocity"),"velocity":np.zeros(3),"mass":0.0}


def test_011_event_root_time():
    schema=core_3dof_schema()
    y0=schema.pack({"position":np.array([0.,0,0]),"velocity":np.array([3.,0,0]),"mass":1.})
    asm=DynamicsAssembler(schema,[ConstantVelocity()])
    sx=schema.sl("position")
    event=Event("x=10",lambda t,y:y[sx][0]-10.0,direction=1,action=EventAction.TERMINATE)
    integ=ScipyIVPIntegrator(SolverConfig(rtol=1e-12,atol=1e-13,max_step=.7))
    res=SimulationEngine(asm.rhs,integ).run((0,10),y0,[event])
    assert abs(res.events[0].time - 10/3) < 1e-12
    assert res.terminated_by == "x=10"


def test_jump_map_plumbing():
    schema=core_3dof_schema()
    y0=schema.pack({"position":np.zeros(3),"velocity":np.array([1.,0,0]),"mass":1.})
    asm=DynamicsAssembler(schema,[ConstantVelocity()]); sx=schema.sl("position"); sv=schema.sl("velocity")
    def bounce(t,y):
        y=y.copy(); y[sv]*=-1; return y
    bounce_event=Event("wall",lambda t,y:y[sx][0]-1,direction=1,priority=10,jump=bounce)
    stop=Event("stop",lambda t,y:t-3,direction=1,action=EventAction.TERMINATE)
    res=SimulationEngine(asm.rhs,ScipyIVPIntegrator(SolverConfig(max_step=.1))).run((0,5),y0,[bounce_event,stop])
    st=schema.unpack(res.states[-1]); assert st["position"][0] < 0.0
