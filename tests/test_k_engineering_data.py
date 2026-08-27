import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from uniflight import (
    AxisMetadata, UncertaintyMetadata, OutputMetadata, DataProvenance,
    ValidityBound, ValidityEnvelope, ValidityPolicy,
    ExtrapolationPolicy, InterpolationMethod, EngineeringTable, EngineeringDataCatalog,
    EngineeringTableAeroCoefficients, AeroCoefficients,
    FlowState, BodyFlowState, wind_to_body_matrix,
    GasSpecies, GasMixture, SphericalBody, PlanetaryEnvironment, VacuumAtmosphere,
    TabulatedAtmosphere, TabulatedAerothermalModel, TabulatedRocketPerformance,
    TabulatedGimballedRocketEngine, ConstantMassProperties,
    TabulatedMaterialProperties, TabulatedMaterialLumpedTPS,
    TabulatedRadialGravity, TabulatedCartesianGravity, TabulatedSphericalTerrain,
    EnvironmentSample, AtmosphereSample, AerothermalEvaluation, FrozenChemistry,
    StateView, core_6dof_schema, entry_6dof_schema,
)


def _prov(name="test-data", version="1"):
    return DataProvenance(name, version, source="synthetic-test")


def test_k001_arbitrary_nd_linear_interpolation():
    x=np.array([0.,1.,2.]); y=np.array([-1.,1.]); z=np.array([10.,20.])
    X,Y,Z=np.meshgrid(x,y,z,indexing="ij")
    f=2*X-3*Y+0.5*Z+7
    table=EngineeringTable(
        (AxisMetadata("x",x,"m"),AxisMetadata("y",y,"rad"),AxisMetadata("z",z,"K")),
        {"f":f}, {"f":OutputMetadata("f","N")}, provenance=_prov(),
    )
    q=table.query({"x":0.25,"y":0.5,"z":13.0})
    assert abs(q.value("f")-(2*.25-3*.5+.5*13+7))<1e-12
    assert q.in_table_domain and q.validity_ok


def test_k002_extrapolation_policies_and_periodic_axis():
    clamp=EngineeringTable((AxisMetadata("x",[0.,1.],extrapolation="clamp"),),{"f":np.array([0.,10.])})
    assert clamp.query({"x":2.}).value("f")==10.0
    assert clamp.query({"x":2.}).adjusted_axes==("x",)
    linear=EngineeringTable((AxisMetadata("x",[0.,1.],extrapolation="extrapolate"),),{"f":np.array([0.,10.])})
    assert abs(linear.query({"x":2.}).value("f")-20.)<1e-12
    assert linear.query({"x":2.}).extrapolated_axes==("x",)
    strict=EngineeringTable((AxisMetadata("x",[0.,1.],extrapolation="error"),),{"f":np.array([0.,10.])})
    with pytest.raises(ValueError): strict.query({"x":2.})

    lon=np.array([-math.pi,0.,math.pi])
    periodic=EngineeringTable((AxisMetadata("longitude",lon,"rad",period=2*math.pi),),{"elevation":np.array([1.,2.,1.])})
    assert abs(periodic.query({"longitude":3*math.pi}).value("elevation")-1.0)<1e-12


def test_k003_validity_envelope_and_uncertainty_metadata():
    table=EngineeringTable(
        (AxisMetadata("mach",[0.,5.,10.]),), {"cd":np.array([.3,.5,.8])},
        {"cd":OutputMetadata("cd","1",uncertainty=UncertaintyMetadata("normal",absolute_sigma=.01,relative_sigma=.02))},
        validity=ValidityEnvelope((ValidityBound("mach",0.,6.),),ValidityPolicy.FLAG),
        provenance=_prov(),
    )
    q=table.query({"mach":8.})
    assert not q.validity_ok and q.validity_violations
    assert abs(q.standard_uncertainty["cd"]-math.hypot(.01,.02*q.value("cd")))<1e-15

    hard=EngineeringTable(
        (AxisMetadata("mach",[0.,10.]),),{"cd":np.array([.3,.8])},
        validity=ValidityEnvelope((ValidityBound("mach",0.,6.),),ValidityPolicy.ERROR),
    )
    with pytest.raises(ValueError): hard.query({"mach":8.})


def test_k004_npz_roundtrip_checksum_and_version_explicit_catalog(tmp_path):
    t=EngineeringTable(
        (AxisMetadata("x",[0.,1.]),),{"y":np.array([2.,4.])},
        {"y":OutputMetadata("y","Pa")},provenance=_prov("catalog-demo","1"),description="demo"
    )
    path=t.to_npz(tmp_path/"demo.npz")
    r=EngineeringTable.from_npz(path)
    assert r.content_sha256()==t.content_sha256()
    assert r.query({"x":.5}).value("y")==3.0
    cat=EngineeringDataCatalog(); cat.register(r)
    assert cat.resolve("catalog-demo").provenance.version=="1"
    t2=EngineeringTable((AxisMetadata("x",[0.,1.]),),{"y":np.array([3.,5.])},provenance=_prov("catalog-demo","2"))
    cat.register(t2)
    with pytest.raises(KeyError): cat.resolve("catalog-demo")
    assert cat.resolve("catalog-demo","2") is t2
    assert len(cat.inventory())==2


def test_k005_aero_adapter_supports_nd_flow_coordinates():
    mach=np.array([0.,2.]); alpha=np.array([-0.2,0.2]); re=np.array([1e5,2e5])
    M,A,R=np.meshgrid(mach,alpha,re,indexing="ij")
    outputs={
        "cd":0.3+0.05*M+0.2*A*A+1e-7*(R-1e5),
        "cl":2*A,
        "cy":np.zeros_like(M),"c_roll":np.zeros_like(M),"c_pitch":-0.5*A,"c_yaw":np.zeros_like(M),
    }
    table=EngineeringTable((AxisMetadata("mach",mach),AxisMetadata("alpha",alpha,"rad"),AxisMetadata("reynolds",re)),outputs)
    model=EngineeringTableAeroCoefficients(table)
    base=FlowState(np.array([1.,0,0]),1.,1.,1.0,1.5e5,0.001)
    flow=BodyFlowState(base,np.array([1.,0,0]),0.1,0.0,wind_to_body_matrix(.1,0.))
    c=model(flow)
    assert isinstance(c,AeroCoefficients)
    assert abs(c.cl-.2)<1e-12 and abs(c.c_pitch+.05)<1e-12
    assert c.cd>0.3


def _simple_world():
    gas=GasSpecies("X",0.03,30.0,1.7e-5,300.,120.,3.8e-10)
    mix=GasMixture((gas,),(1.0,))
    body=SphericalBody(mu=1e10,radius=1e5,name="K-world")
    return gas,mix,body


def _state6(body, speed=100.0, mass=100.0):
    s=core_6dof_schema(); y=s.pack({
        "position":np.array([body.radius,0,0.]),"velocity":np.array([0.,speed,0.]),
        "attitude":np.array([1.,0,0,0]),"angular_rate":np.zeros(3),"mass":mass,
    }); return StateView(0.,y,s)


def test_k006_tabulated_atmosphere_derives_transport_from_mixture():
    _,mix,body=_simple_world()
    alt=np.array([0.,10_000.])
    table=EngineeringTable((AxisMetadata("altitude",alt,"m"),),{
        "temperature":np.array([250.,230.]),"pressure":np.array([20_000.,2_000.]),
    })
    atm=TabulatedAtmosphere(table,mix)
    q=atm.query(5_000.)
    assert q.temperature==240.0 and q.pressure==11_000.0
    assert q.density>0 and q.viscosity>0 and q.speed_of_sound>0 and q.mean_free_path>0


def test_k007_tabulated_aerothermal_model_drives_heat_flux():
    _,mix,body=_simple_world()
    atm_table=EngineeringTable((AxisMetadata("altitude",[0.,1000.]),),{
        "temperature":np.array([250.,250.]),"pressure":np.array([20_000.,20_000.]),
    })
    env=PlanetaryEnvironment(body,TabulatedAtmosphere(atm_table,mix))
    density=env.query(np.array([body.radius,0,0])).atmosphere.density
    speed=np.array([0.,200.])
    D,V=np.meshgrid(np.array([density*.5,density*1.5]),speed,indexing="ij")
    qconv=100*D+2*V
    heat_table=EngineeringTable((AxisMetadata("density",[density*.5,density*1.5]),AxisMetadata("speed",speed)),{
        "convective_heat_flux":qconv,
        "radiative_heat_flux":np.zeros_like(qconv),
    })
    model=TabulatedAerothermalModel(env,1.0,heat_table)
    e=model.evaluate(_state6(body,100.0))
    assert abs(e.convective_heat_flux-(100*density+200.0))<1e-9


def test_k008_tabulated_propulsion_engine_integrates_table_performance():
    _,_,body=_simple_world(); env=PlanetaryEnvironment(body,VacuumAtmosphere())
    p=np.array([0.,100_000.]); u=np.array([0.,1.]); P,U=np.meshgrid(p,u,indexing="ij")
    thrust=1000*U-0.002*P*U; mdot=0.5*U
    table=EngineeringTable((AxisMetadata("ambient_pressure",p,"Pa"),AxisMetadata("throttle",u)),{
        "thrust":thrust,"mass_flow":mdot,
    })
    perf=TabulatedRocketPerformance(table)
    pe=perf.evaluate(50_000.,.5)
    assert abs(pe.thrust-450.)<1e-12 and abs(pe.mass_flow-.25)<1e-12
    engine=TabulatedGimballedRocketEngine(env,ConstantMassProperties(np.eye(3)),perf,throttle=.5)
    e=engine.evaluate(_state6(body,0.,100.))
    assert abs(e.thrust-500.)<1e-12 and abs(engine.mass_rate(_state6(body,0.,100.))+.25)<1e-12


def test_k009_material_table_drives_lumped_tps_properties():
    _,_,body=_simple_world(); env=PlanetaryEnvironment(body,VacuumAtmosphere())
    temp=np.array([300.,1000.])
    material_table=EngineeringTable((AxisMetadata("temperature",temp,"K",extrapolation="clamp"),),{
        "specific_heat":np.array([1000.,1200.]),
        "emissivity":np.array([0.8,0.9]),
        "ablation_temperature":np.array([800.,800.]),
        "effective_heat_of_ablation":np.array([2e6,2e6]),
    })
    material=TabulatedMaterialProperties(material_table)
    class Heating:
        def evaluate(self,state):
            atm=AtmosphereSample(0.,300.,0.,0.,0.,math.inf,math.inf,None)
            es=EnvironmentSample(state.time,state.get("position"),0.,np.zeros(3),np.array([1.,0,0]),atm,np.zeros(3),np.zeros(3))
            flow=FlowState(np.zeros(3),0.,0.,0.,0.,math.inf)
            return AerothermalEvaluation(es,flow,FrozenChemistry().evaluate(es,flow),1e5,0.,1e5)
    tps=TabulatedMaterialLumpedTPS(Heating(),material,heated_area=1.0,thermal_mass=10.0)
    s=entry_6dof_schema(); y=s.pack({
        "position":np.array([body.radius,0,0]),"velocity":np.zeros(3),"attitude":np.array([1.,0,0,0]),
        "angular_rate":np.zeros(3),"mass":100.,"tps_temperature":900.,"heat_load":0.,"tps_mass":5.,
    })
    st=StateView(0.,y,s); e=tps.evaluate(st)
    assert e.ablation_mass_rate>0 and e.temperature_rate==0.0


def test_k010_radial_gravity_table_and_environment_override():
    _,_,body=_simple_world()
    r=np.array([body.radius,body.radius+10_000.])
    g=np.array([4.,3.])
    table=EngineeringTable((AxisMetadata("radius",r,"m",extrapolation="extrapolate"),),{"gravity":g})
    gravity=TabulatedRadialGravity(table)
    a=gravity.acceleration(np.array([body.radius+5000.,0,0]))
    assert np.allclose(a,[-3.5,0,0])
    J=gravity.jacobian(np.array([body.radius+5000.,0,0]))
    assert np.all(np.isfinite(J)) and J.shape==(3,3)
    env=PlanetaryEnvironment(body,gravity_model=gravity)
    assert np.allclose(env.query(np.array([body.radius+5000.,0,0])).gravity_i,a)


def test_k011_cartesian_gravity_table_interpolates_vector_field():
    axis=np.array([-1.,1.]); X,Y,Z=np.meshgrid(axis,axis,axis,indexing="ij")
    table=EngineeringTable((AxisMetadata("x",axis),AxisMetadata("y",axis),AxisMetadata("z",axis)),{
        "gx":-2*X,"gy":-3*Y,"gz":-4*Z,
    })
    grav=TabulatedCartesianGravity(table)
    r=np.array([.25,-.5,.75])
    assert np.allclose(grav.acceleration(r),[-.5,1.5,-3.])
    assert np.allclose(grav.jacobian(r),np.diag([-2.,-3.,-4.]),atol=1e-9)


def test_k012_spherical_terrain_uses_periodic_longitude_and_slope_normal():
    _,_,body=_simple_world()
    lat=np.array([-0.5,0.,0.5]); lon=np.array([-math.pi,0.,math.pi])
    LAT,LON=np.meshgrid(lat,lon,indexing="ij")
    elevation=100.0+20.0*LAT  # slope only in latitude; seam-safe longitude
    table=EngineeringTable((
        AxisMetadata("latitude",lat,"rad",extrapolation="clamp"),
        AxisMetadata("longitude",lon,"rad",period=2*math.pi),
    ),{"elevation":elevation})
    terrain=TabulatedSphericalTerrain(body,table)
    q=terrain.query(np.array([body.radius,0,0.]))
    assert abs(q.elevation-100.0)<1e-12
    assert q.normal_i[2] < 0.0  # rising terrain toward +latitude tilts outward normal southward
    q2=terrain.query(np.array([-body.radius,-1e-6,0.]))
    assert np.isfinite(q2.elevation)


def test_k013_catalog_inventory_is_reproducible_and_content_hash_changes_with_data():
    a=EngineeringTable((AxisMetadata("x",[0.,1.]),),{"f":np.array([0.,1.])},provenance=_prov("hash","1"))
    b=EngineeringTable((AxisMetadata("x",[0.,1.]),),{"f":np.array([0.,2.])},provenance=_prov("hash2","1"))
    assert a.content_sha256()!=b.content_sha256()
    c=EngineeringDataCatalog(); c.register(a); c.register(b)
    inv=c.inventory(); assert inv==tuple(sorted(inv)) and len(inv)==2

def test_k014_long_form_csv_roundtrip_and_incomplete_grid_rejection(tmp_path):
    from uniflight import save_long_form_csv, load_long_form_csv
    x=np.array([0.,1.]); y=np.array([10.,20.,30.]); X,Y=np.meshgrid(x,y,indexing="ij")
    t=EngineeringTable((AxisMetadata("x",x),AxisMetadata("y",y)),{"f":X+2*Y},provenance=_prov("csv","1"))
    p=save_long_form_csv(t,tmp_path/"table.csv")
    r=load_long_form_csv(p,axis_names=("x","y"),output_names=("f",),provenance=_prov("csv","1"))
    assert r.shape==(2,3) and abs(r.query({"x":.5,"y":15.}).value("f")-30.5)<1e-12
    lines=p.read_text().splitlines(); (tmp_path/"bad.csv").write_text("\n".join(lines[:-1])+"\n")
    with pytest.raises(ValueError):
        load_long_form_csv(tmp_path/"bad.csv",axis_names=("x","y"),output_names=("f",))

def test_k015_end_to_end_table_driven_6dof_flight():
    from uniflight import (
        ContinuumAerodynamics6DOF, ConstantReferenceGeometry,
        RigidBody6DOFDynamics, QuaternionKinematics, DynamicsAssembler,
        ScipyIVPIntegrator, SolverConfig, SimulationEngine,
    )
    _,mix,body=_simple_world()
    atm_table=EngineeringTable((AxisMetadata("altitude",[0.,5000.],"m",extrapolation="clamp"),),{
        "temperature":np.array([250.,240.]),"pressure":np.array([20_000.,10_000.]),
    },provenance=_prov("k-atmosphere","1"))
    atmosphere=TabulatedAtmosphere(atm_table,mix)
    gravity_table=EngineeringTable((AxisMetadata("radius",[body.radius,body.radius+5000.],"m",extrapolation="clamp"),),{
        "gravity":np.array([2.0,1.8]),
    },provenance=_prov("k-gravity","1"))
    gravity=TabulatedRadialGravity(gravity_table)
    env=PlanetaryEnvironment(body,atmosphere,gravity_model=gravity)

    aero_table=EngineeringTable((AxisMetadata("mach",[0.,2.],extrapolation="clamp"),),{
        "cd":np.array([0.5,0.5]),"cl":np.zeros(2),"cy":np.zeros(2),
        "c_roll":np.zeros(2),"c_pitch":np.zeros(2),"c_yaw":np.zeros(2),
    },provenance=_prov("k-aero","1"))
    coeff=EngineeringTableAeroCoefficients(aero_table)
    mp=ConstantMassProperties(np.diag([20.,20.,20.]))
    aero=ContinuumAerodynamics6DOF(
        env,ConstantReferenceGeometry(1.0,1.0,1.0,1.0,np.zeros(3)),coeff,mp
    )

    p=np.array([0.,20_000.]); u=np.array([0.,1.]); P,U=np.meshgrid(p,u,indexing="ij")
    propulsion_table=EngineeringTable((AxisMetadata("ambient_pressure",p,"Pa"),AxisMetadata("throttle",u)),{
        "thrust":U*(1200.-0.01*P),"mass_flow":0.5*U,
    },provenance=_prov("k-propulsion","1"))
    engine=TabulatedGimballedRocketEngine(env,mp,TabulatedRocketPerformance(propulsion_table),throttle=1.0)

    schema=core_6dof_schema(); y0=schema.pack({
        "position":np.array([body.radius,0.,0.]),"velocity":np.zeros(3),
        "attitude":np.array([1.,0.,0.,0.]),"angular_rate":np.zeros(3),"mass":100.,
    })
    rigid=RigidBody6DOFDynamics(mp,gravity=gravity,wrench_models=(engine,aero))
    rhs=DynamicsAssembler(schema,[rigid,QuaternionKinematics(),engine]).rhs
    result=SimulationEngine(rhs,ScipyIVPIntegrator(SolverConfig(rtol=1e-9,atol=1e-11,max_step=.05))).run((0.,4.),y0)
    assert result.success and np.all(np.isfinite(result.states))
    final=schema.unpack(result.states[-1])
    assert np.linalg.norm(final["position"])-body.radius > 20.0
    assert abs(final["mass"]-98.0)<1e-6
    assert abs(np.linalg.norm(final["attitude"])-1.0)<1e-10
