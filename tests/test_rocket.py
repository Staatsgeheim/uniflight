import numpy as np
from uniflight import (core_3dof_schema, TranslationalKinematics, DynamicsAssembler,
                       IdealRocket, SimulationEngine, ScipyIVPIntegrator, SolverConfig,
                       Event, EventAction)


def test_001_tsiolkovsky_relative_error_better_than_1e9():
    m0, mf, ve, mdot = 1200.0, 400.0, 3200.0, 8.0
    rocket = IdealRocket(ve, mdot, np.array([1.,0,0]))
    schema = core_3dof_schema()
    y0 = schema.pack({"position":np.zeros(3), "velocity":np.zeros(3), "mass":m0})
    asm = DynamicsAssembler(schema,[TranslationalKinematics(None,(rocket,)), rocket])
    mass_slice = schema.sl("mass")
    burn = Event("burnout", lambda t,y: y[mass_slice][0]-mf, direction=-1,
                 action=EventAction.TERMINATE, priority=100)
    integ = ScipyIVPIntegrator(SolverConfig(rtol=2e-12, atol=1e-12, max_step=0.5))
    res = SimulationEngine(asm.rhs,integ).run((0,1000),y0,[burn])
    vf = schema.unpack(res.states[-1])["velocity"][0]
    expected = ve*np.log(m0/mf)
    assert abs(vf-expected)/expected < 1e-9
    assert res.terminated_by == "burnout"
