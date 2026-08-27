import numpy as np

from uniflight import (
    GasSpecies, GasMixture, SphericalBody, IsothermalHydrostaticAtmosphere, PlanetaryEnvironment,
    ConstantMassProperties, GimballedRocketEngine, ConstantReferenceGeometry,
    LinearStabilityAerodynamics, ContinuumAerodynamics6DOF,
    core_6dof_schema, RigidBody6DOFDynamics, QuaternionKinematics, DynamicsAssembler,
    SimulationEngine, ScipyIVPIntegrator, SolverConfig,
)


def test_026_end_to_end_6dof_atmospheric_flight_couples_translation_rotation_and_mass():
    gas=GasSpecies("Q",.029,29.2,1.75e-5,300.,115.,3.7e-10)
    mix=GasMixture((gas,),(1.,))
    body=SphericalBody(mu=5e11,radius=5e5,name="Asteria-C")
    atm=IsothermalHydrostaticAtmosphere(18_000.,245.,mix,body.mu,body.radius,ceiling=120_000.)
    env=PlanetaryEnvironment(body,atm)
    mp=ConstantMassProperties(np.diag([400.,800.,800.]))
    engine=GimballedRocketEngine(env,mp,exhaust_velocity=1500.,mdot_exhaust=2.,
                                 mount_position_b=np.array([-1.5,0,0]),pitch_gimbal=.001,
                                 dry_mass=850.)
    geom=ConstantReferenceGeometry(2.0,3.0,2.0,3.0,np.array([.4,0,0]))
    coeff=LinearStabilityAerodynamics(cd0=.35,cd_alpha2=.8,cl_alpha=1.2,
                                      cy_beta=-.5,c_pitch_alpha=-.8,c_yaw_beta=.4)
    aero=ContinuumAerodynamics6DOF(env,geom,coeff,mp)
    schema=core_6dof_schema()
    y0=schema.pack({"position":np.array([body.radius,0,0]),"velocity":np.zeros(3),
                    "attitude":np.array([1.,0,0,0]),"angular_rate":np.zeros(3),"mass":1000.})
    dyn=RigidBody6DOFDynamics(mp,body.gravity,(engine,aero))
    asm=DynamicsAssembler(schema,[dyn,QuaternionKinematics(),engine])
    integ=ScipyIVPIntegrator(SolverConfig(rtol=3e-10,atol=1e-11,max_step=.02))
    res=SimulationEngine(asm.rhs,integ).run((0,15.),y0)
    sf=schema.unpack(res.states[-1])
    assert res.success
    assert abs(sf["mass"]-970.) < 2e-8
    assert np.linalg.norm(sf["velocity"]) > 5.
    assert np.linalg.norm(sf["position"])-body.radius > 20.
    assert np.linalg.norm(sf["angular_rate"]) > 1e-4
    assert abs(np.linalg.norm(sf["attitude"])-1.) < 2e-8
    assert np.all(np.isfinite(res.states))
