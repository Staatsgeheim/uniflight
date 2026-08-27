import numpy as np
import pytest
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


def _unit_speed_rhs(t, y):
    return np.array([1.0])


def _simultaneous_priority_events():
    return [
        Event(
            "low",
            lambda t, y: y[0] - 1.0,
            direction=1,
            priority=0,
            jump=lambda t, y: y + np.array([1.0]),
        ),
        Event(
            "high",
            lambda t, y: y[0] - 1.0,
            direction=1,
            priority=10,
            jump=lambda t, y: y + np.array([10.0]),
        ),
    ]


def test_adaptive_simultaneous_events_collect_all_ties_and_apply_priority():
    res = SimulationEngine(_unit_speed_rhs, ScipyIVPIntegrator(
        SolverConfig(rtol=1e-12, atol=1e-13, max_step=0.2)
    )).run((0.0, 2.0), np.array([0.0]), _simultaneous_priority_events())
    assert [e.name for e in res.events[:2]] == ["high", "low"]
    assert res.events[0].time == pytest.approx(1.0, abs=1e-11)
    assert res.events[1].time == pytest.approx(1.0, abs=1e-11)
    assert res.states[-1, 0] == pytest.approx(13.0, abs=1e-10)


def test_fixed_step_simultaneous_events_collect_all_ties_and_apply_priority():
    from uniflight import FixedStepRK4Integrator, FixedStepRK4Config
    integ = FixedStepRK4Integrator(FixedStepRK4Config(
        step=0.2, event_time_tolerance=1e-9, event_guard_tolerance=1e-10
    ))
    res = SimulationEngine(_unit_speed_rhs, integ).run(
        (0.0, 2.0), np.array([0.0]), _simultaneous_priority_events()
    )
    assert [e.name for e in res.events[:2]] == ["high", "low"]
    assert res.events[0].time == pytest.approx(1.0, abs=2e-9)
    assert res.states[-1, 0] == pytest.approx(13.0, abs=2e-8)


@pytest.mark.parametrize("integrator", [
    ScipyIVPIntegrator(SolverConfig(rtol=1e-12, atol=1e-13, max_step=0.2)),
    pytest.param("rk4", id="fixed-step-rk4"),
])
def test_continuing_nonjump_state_root_fires_once_without_zero_time_cycle(integrator):
    from uniflight import FixedStepRK4Integrator, FixedStepRK4Config
    if integrator == "rk4":
        integrator = FixedStepRK4Integrator(FixedStepRK4Config(
            step=0.2, event_time_tolerance=1e-9, event_guard_tolerance=1e-10
        ))
    event = Event(
        "cross",
        lambda t, y: y[0] - 1.0,
        direction=1,
        action=EventAction.CONTINUE,
        jump=None,
    )
    res = SimulationEngine(_unit_speed_rhs, integrator).run(
        (0.0, 2.0), np.array([0.0]), [event]
    )
    assert res.success
    assert len(res.events) == 1
    assert res.events[0].time == pytest.approx(1.0, abs=2e-9)
    assert res.states[-1, 0] == pytest.approx(2.0, abs=2e-8)
