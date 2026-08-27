import numpy as np
from uniflight import core_3dof_schema, PointMassGravity, TranslationalKinematics, DynamicsAssembler, SimulationEngine, ScipyIVPIntegrator, SolverConfig
from uniflight.invariants import specific_energy, specific_angular_momentum


def test_002_kepler_two_body_invariants():
    mu = 3.986004418e14  # test datum only; gravity class is body-agnostic
    r0 = 7.0e6
    v0 = np.sqrt(mu/r0)
    period = 2*np.pi*np.sqrt(r0**3/mu)
    schema = core_3dof_schema()
    y0 = schema.pack({"position":np.array([r0,0,0]), "velocity":np.array([0,v0,0]), "mass":1000.})
    asm = DynamicsAssembler(schema, [TranslationalKinematics(PointMassGravity(mu))])
    integ = ScipyIVPIntegrator(SolverConfig(rtol=2e-12, atol=1e-8, max_step=period/300))
    res = SimulationEngine(asm.rhs, integ).run((0, period), y0)
    p0,vv0 = schema.unpack(y0)["position"], schema.unpack(y0)["velocity"]
    pf,vvf = schema.unpack(res.states[-1])["position"], schema.unpack(res.states[-1])["velocity"]
    e0, ef = specific_energy(mu,p0,vv0), specific_energy(mu,pf,vvf)
    h0, hf = specific_angular_momentum(p0,vv0), specific_angular_momentum(pf,vvf)
    assert abs((ef-e0)/e0) < 2e-11
    assert np.linalg.norm(hf-h0)/np.linalg.norm(h0) < 2e-11
    assert np.linalg.norm(pf-p0)/r0 < 5e-10


def test_003_vacuum_radial_free_fall_short_time_against_constant_g_limit():
    # Use a short interval so the analytic local Taylor expansion provides an independent check.
    mu = 4.0e12; r0 = 2.0e6; dt = 0.1
    g0 = mu/r0**2
    schema = core_3dof_schema()
    y0 = schema.pack({"position":np.array([r0,0,0]), "velocity":np.zeros(3), "mass":1.})
    asm = DynamicsAssembler(schema,[TranslationalKinematics(PointMassGravity(mu))])
    integ = ScipyIVPIntegrator(SolverConfig(rtol=1e-13,atol=1e-13,max_step=0.002))
    res = SimulationEngine(asm.rhs,integ).run((0,dt),y0)
    st = schema.unpack(res.states[-1])
    # x = r0 - 1/2 g0 t^2 + O(t^4), v=-g0 t + O(t^3)
    assert abs(st["position"][0] - (r0 - .5*g0*dt**2)) < 2e-8
    assert abs(st["velocity"][0] - (-g0*dt)) < 2e-6
