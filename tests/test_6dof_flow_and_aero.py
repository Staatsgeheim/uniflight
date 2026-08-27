import math
import numpy as np

from uniflight import (
    GasSpecies, GasMixture, SphericalBody, IsothermalHydrostaticAtmosphere,
    PlanetaryEnvironment, VacuumAtmosphere, EnvironmentSample,
    compute_body_flow_state, wind_to_body_matrix,
    FlowState, BodyFlowState,
    AeroCoefficients, ConstantAeroCoefficients, GridAeroCoefficientDatabase,
    ConstantReferenceGeometry, EllipsoidProjectedGeometry, ContinuumAerodynamics6DOF,
    ConstantMassProperties, core_6dof_schema, StateView,
)


def _world():
    gas = GasSpecies("X", 0.030, 30.0, 1.7e-5, 300.0, 120.0, 3.8e-10)
    mix = GasMixture((gas,), (1.0,))
    body = SphericalBody(mu=8e11, radius=5e5, name="Testia")
    atm = IsothermalHydrostaticAtmosphere(50_000.0, 250.0, mix, body.mu, body.radius)
    return body, PlanetaryEnvironment(body, atm)


def _state(body, velocity, attitude=None, mass=1000.0):
    schema = core_6dof_schema()
    q = np.array([1.,0,0,0]) if attitude is None else np.asarray(attitude,float)
    y = schema.pack({"position":np.array([body.radius,0,0.]),"velocity":np.asarray(velocity,float),
                     "attitude":q,"angular_rate":np.zeros(3),"mass":mass})
    return StateView(0.0,y,schema)


def test_018_body_flow_alpha_beta_and_wind_basis():
    body, env = _world()
    V, alpha, beta = 300.0, math.radians(10), math.radians(5)
    vb = V*np.array([math.cos(alpha)*math.cos(beta), math.sin(beta), math.sin(alpha)*math.cos(beta)])
    sample = env.query(np.array([body.radius,0,0.]))
    flow = compute_body_flow_state(vb, np.array([1.,0,0,0]), sample, 2.0)
    assert abs(flow.alpha-alpha) < 1e-14
    assert abs(flow.beta-beta) < 1e-14
    np.testing.assert_allclose(flow.rotation_bw.T @ flow.rotation_bw, np.eye(3), atol=2e-15, rtol=0)
    np.testing.assert_allclose(flow.rotation_bw[:,0], vb/V, atol=2e-15, rtol=0)
    assert np.linalg.det(flow.rotation_bw) > 0.999999999999


def test_019_attitude_maps_inertial_flow_into_body_axes():
    body, env = _world()
    # +90 deg yaw: body +x maps to inertial +y.
    q = np.array([math.cos(math.pi/4),0,0,math.sin(math.pi/4)])
    sample = env.query(np.array([body.radius,0,0.]))
    flow = compute_body_flow_state(np.array([0.,250.,0.]), q, sample, 1.0)
    np.testing.assert_allclose(flow.relative_velocity_b, [250.,0.,0.], atol=2e-13, rtol=0)
    assert abs(flow.alpha) < 1e-14
    assert abs(flow.beta) < 1e-14


def test_020_6dof_aero_wind_force_and_body_moment_scaling():
    body, env = _world()
    mp = ConstantMassProperties(np.diag([10.,20.,30.]))
    geom = ConstantReferenceGeometry(4.0,2.0,6.0,1.5,np.zeros(3))
    coeff = AeroCoefficients(cd=.5, cl=.2, cy=.1, c_roll=.01, c_pitch=-.02, c_yaw=.03)
    aero = ContinuumAerodynamics6DOF(env,geom,ConstantAeroCoefficients(coeff),mp)
    state = _state(body,[200.,0,0])
    ev = aero.evaluate(state)
    qS = ev.flow.dynamic_pressure*4.0
    np.testing.assert_allclose(ev.force_b, qS*np.array([-.5,.1,-.2]), rtol=2e-14, atol=1e-10)
    np.testing.assert_allclose(ev.force_i, ev.force_b, rtol=0, atol=1e-12)
    expected_m = qS*np.array([6*.01,1.5*(-.02),6*.03])
    np.testing.assert_allclose(ev.moment_b_about_cg, expected_m, rtol=2e-14, atol=1e-10)


def test_021_grid_aero_database_trilinear_interpolation():
    mach=np.array([0.,2.]); alpha=np.array([-.2,.2]); beta=np.array([-.1,.1])
    M,A,B=np.meshgrid(mach,alpha,beta,indexing='ij')
    def f(c0,cM,cA,cB): return c0+cM*M+cA*A+cB*B
    db=GridAeroCoefficientDatabase(mach,alpha,beta,
        f(.3,.1,.2,.1), f(0,.0,2.,0), f(0,0,0,1.5),
        f(0,0,0,-.2), f(0,0,-.7,0), f(0,0,0,.3))
    base=FlowState(np.array([1.,0,0]),1.,1.,1.25,1e6,1e-5)
    flow=BodyFlowState(base,np.array([1.,0,0]),.05,-.025,wind_to_body_matrix(.05,-.025))
    c=db(flow)
    assert abs(c.cd-(.3+.1*1.25+.2*.05+.1*(-.025))) < 1e-13
    assert abs(c.cl-2*.05) < 1e-13
    assert abs(c.cy-1.5*(-.025)) < 1e-13
    assert abs(c.c_pitch-(-.7*.05)) < 1e-13


def test_022_ellipsoid_projected_area_changes_with_flow_direction():
    geom=EllipsoidProjectedGeometry(np.array([3.,2.,1.]),6.,4.,3.,np.zeros(3))
    base=FlowState(np.array([1.,0,0]),1.,0.,0.,0.,0.)
    fx=BodyFlowState(base,np.array([1.,0,0]),0.,0.,np.eye(3))
    fz=BodyFlowState(base,np.array([0.,0,1.]),math.pi/2,0.,wind_to_body_matrix(math.pi/2,0.))
    class Dummy: pass
    ax=geom.evaluate(fx,Dummy()).reference_area
    az=geom.evaluate(fz,Dummy()).reference_area
    assert abs(ax-math.pi*2*1) < 1e-13
    assert abs(az-math.pi*3*2) < 1e-13
    assert az > ax
