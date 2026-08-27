from __future__ import annotations
import numpy as np
import pytest

from uniflight import (
    StateView, core_3dof_schema, core_6dof_schema,
    promote_3dof_to_6dof, demote_6dof_to_3dof,
    VehicleEvent, VehicleSpec, UniverseMutation, MultiVehicleUniverseEngine,
    VehicleConfiguration, DOFSwitchHandler,
    separate_two_rigid_bodies, RigidChildTemplate, RigidSeparationHandler,
)


def rhs3(schema, acceleration=(0.0,0.0,0.0)):
    a=np.asarray(acceleration,float)
    def fn(t,y):
        v=StateView(t,y,schema).get("velocity")
        out=np.zeros_like(y)
        out[schema.sl("position")]=v
        out[schema.sl("velocity")]=a
        return out
    return fn


def rhs6(schema, acceleration=(0.0,0.0,0.0)):
    a=np.asarray(acceleration,float)
    def fn(t,y):
        s=StateView(t,y,schema)
        out=np.zeros_like(y)
        out[schema.sl("position")]=s.get("velocity")
        out[schema.sl("velocity")]=a
        # attitude/rate/mass remain static for these universe tests
        return out
    return fn


def pack3(r=(0,0,0),v=(0,0,0),m=1.0):
    s=core_3dof_schema(); return s,s.pack({"position":r,"velocity":v,"mass":m})


def pack6(r=(0,0,0),v=(1,0,0),m=1.0,q=(1,0,0,0),w=(0,0,0)):
    s=core_6dof_schema(); return s,s.pack({"position":r,"velocity":v,"attitude":q,"angular_rate":w,"mass":m})


def test_promote_and_demote_dof_preserves_translational_state():
    s3,y3=pack3((1,2,3),(4,5,6),7)
    y6=promote_3dof_to_6dof(y3,source_schema=s3,attitude=np.array([1.,0,0,0]),angular_rate_b=np.array([.1,.2,.3]))
    s6=core_6dof_schema(); v=s6.unpack(y6)
    assert np.allclose(v["position"],[1,2,3]); assert np.allclose(v["velocity"],[4,5,6]); assert v["mass"]==7
    assert np.allclose(v["angular_rate"],[.1,.2,.3])
    back=demote_6dof_to_3dof(y6,source_schema=s6,target_schema=s3)
    assert np.allclose(back,y3)


def test_zero_velocity_promotion_requires_attitude():
    s3,y3=pack3(v=(0,0,0))
    with pytest.raises(ValueError):
        promote_3dof_to_6dof(y3,source_schema=s3)


def test_two_vehicle_concurrent_propagation_with_distinct_dynamics():
    s,y0=pack3(v=(1,0,0),m=2)
    a=VehicleSpec("A",s,y0,rhs3(s,(1,0,0)),dof=3,model_context={"environment":"A"})
    b=VehicleSpec("B",s,y0,rhs3(s,(0,2,0)),dof=3,model_context={"environment":"B"})
    result=MultiVehicleUniverseEngine().run((0,2),(a,b))
    A=s.unpack(result.final_vehicles["A"].state); B=s.unpack(result.final_vehicles["B"].state)
    assert np.allclose(A["velocity"],[3,0,0],atol=1e-10)
    assert np.allclose(A["position"],[4,0,0],atol=1e-9)
    assert np.allclose(B["velocity"],[1,4,0],atol=1e-10)
    assert np.allclose(B["position"],[2,4,0],atol=1e-9)
    assert result.final_vehicles["A"].model_context["environment"]=="A"


def test_global_event_synchronizes_other_vehicle_and_continues():
    s,ya=pack3(v=(1,0,0)); _,yb=pack3(v=(0,1,0))
    ev=VehicleEvent("A_tick",lambda t,y:t-1.0,direction=1,handler=lambda ctx:UniverseMutation(note="tick"))
    A=VehicleSpec("A",s,ya,rhs3(s),events=(ev,),dof=3)
    B=VehicleSpec("B",s,yb,rhs3(s),dof=3)
    result=MultiVehicleUniverseEngine().run((0,2),(A,B))
    assert len(result.events)==1 and abs(result.events[0].time-1)<1e-10
    assert len(result.segments["B"])==2
    assert abs(result.segments["B"][0].end_time-1)<1e-10
    assert np.allclose(s.unpack(result.final_vehicles["B"].state)["position"],[0,2,0],atol=1e-9)


def test_spawn_and_remove_changes_universe_topology():
    s,y=pack3(v=(1,0,0),m=10)
    child_state=s.pack({"position":[1,0,0],"velocity":[0,2,0],"mass":3.0})
    child=VehicleSpec("child",s,child_state,rhs3(s),dof=3)
    def handler(ctx): return UniverseMutation(remove=("parent",),upsert=(child,),note="spawn child")
    ev=VehicleEvent("spawn",lambda t,y:t-1,direction=1,handler=handler)
    parent=VehicleSpec("parent",s,y,rhs3(s),events=(ev,),dof=3)
    result=MultiVehicleUniverseEngine().run((0,3),(parent,))
    assert "parent" not in result.final_vehicles and "child" in result.final_vehicles
    assert result.events[0].active_vehicle_ids_after==("child",)
    cv=s.unpack(result.final_vehicles["child"].state)
    assert np.allclose(cv["position"],[1,4,0],atol=1e-8)


def test_3dof_to_6dof_runtime_switch_preserves_id_and_schema_history():
    s3,y=pack3(v=(2,0,0),m=5)
    s6=core_6dof_schema()
    cfg6=VehicleConfiguration(s6,rhs6(s6),mode="six",dof=6)
    handler=DOFSwitchHandler(cfg6,defaults={"attitude":np.array([1.,0,0,0]),"angular_rate":np.zeros(3)})
    ev=VehicleEvent("promote",lambda t,y:t-1,direction=1,handler=handler)
    v=VehicleSpec("ship",s3,y,rhs3(s3),events=(ev,),mode="three",dof=3)
    result=MultiVehicleUniverseEngine().run((0,2),(v,))
    snap=result.final_vehicles["ship"]
    assert snap.dof==6 and snap.schema.total_size==s6.total_size
    assert [seg.dof for seg in result.segments["ship"]]==[3,6]
    vals=s6.unpack(snap.state)
    assert np.allclose(vals["position"],[4,0,0],atol=1e-8)


def test_rigid_separation_conserves_linear_and_angular_momentum():
    m1,m2=6.,4.; M=m1+m2
    r1=np.array([-.4,0,0]); r2=np.array([.6,0,0])
    I1=np.diag([2.,3.,4.]); I2=np.diag([1.,2.,2.5])
    def pa(m,r): return m*((r@r)*np.eye(3)-np.outer(r,r))
    I0=I1+I2+pa(m1,r1)+pa(m2,r2)
    res=separate_two_rigid_bodies(
        parent_mass=M,parent_position_i=np.array([10.,20.,30.]),parent_velocity_i=np.array([3.,-2.,1.]),
        parent_attitude_bi=np.array([1.,0,0,0]),parent_angular_rate_b=np.array([.2,-.1,.3]),parent_inertia_b=I0,
        retained_mass=m1,detached_mass=m2,retained_offset_b=r1,detached_offset_b=r2,
        retained_inertia_b=I1,detached_inertia_b=I2,relative_separation_velocity_i=np.array([.5,.1,-.2]),
    )
    assert np.linalg.norm(res.linear_momentum_error_i)<1e-12
    assert np.linalg.norm(res.angular_momentum_error_i)<1e-11
    assert np.linalg.norm(res.parent_com_error_b)<1e-14
    assert np.linalg.norm(res.composite_inertia_error_b)<1e-12


def test_rigid_separation_handler_spawns_two_6dof_daughters():
    s6,y=pack6(m=10,w=(0,0,.2))
    m1,m2=6.,4.; r1=np.array([-.4,0,0]); r2=np.array([.6,0,0])
    I1=np.diag([2.,3.,4.]); I2=np.diag([1.,2.,2.5])
    def pa(m,r): return m*((r@r)*np.eye(3)-np.outer(r,r))
    I0=I1+I2+pa(m1,r1)+pa(m2,r2)
    cfg=VehicleConfiguration(s6,rhs6(s6),mode="free",dof=6)
    sep=RigidSeparationHandler(
        RigidChildTemplate("core",cfg),RigidChildTemplate("shell",cfg),m1,m2,r1,r2,I0,I1,I2,
        relative_separation_velocity_i=np.array([.4,0,0]),
    )
    ev=VehicleEvent("separate",lambda t,y:t-1,direction=1,handler=sep)
    parent=VehicleSpec("stack",s6,y,rhs6(s6),events=(ev,),mode="stack",dof=6)
    result=MultiVehicleUniverseEngine().run((0,2),(parent,))
    assert set(result.final_vehicles)=={"core","shell"}
    c=s6.unpack(result.final_vehicles["core"].state); sh=s6.unpack(result.final_vehicles["shell"].state)
    assert abs(c["mass"]+sh["mass"]-10)<1e-12
    P=6*np.asarray(c["velocity"])+4*np.asarray(sh["velocity"])
    # Parent was at V=[1,0,0] at separation and no external acceleration.
    assert np.allclose(P,[10,0,0],atol=1e-10)


def test_simultaneous_event_priority_suppresses_old_generation_lower_priority():
    s,y=pack3(v=(1,0,0))
    replacement=VehicleSpec("v",s,y,rhs3(s,(0,1,0)),mode="new",dof=3)
    high=VehicleEvent("high",lambda t,y:t-1,direction=1,priority=10,
                      handler=lambda ctx:UniverseMutation(upsert=(replacement,),note="replace"))
    def should_not_run(ctx):
        raise AssertionError("lower-priority event from replaced generation must not execute")
    low=VehicleEvent("low",lambda t,y:t-1,direction=1,priority=0,handler=should_not_run)
    v=VehicleSpec("v",s,y,rhs3(s),events=(high,low),mode="old",dof=3)
    result=MultiVehicleUniverseEngine().run((0,2),(v,))
    assert len(result.events)==1 and result.events[0].event_name=="high"
    assert result.final_vehicles["v"].mode=="new"


def test_6dof_to_3dof_runtime_switch():
    s6,y=pack6(v=(3,0,0),m=4,w=(.1,.2,.3))
    s3=core_3dof_schema()
    cfg3=VehicleConfiguration(s3,rhs3(s3),mode="coast-3dof",dof=3)
    handler=DOFSwitchHandler(cfg3)
    ev=VehicleEvent("demote",lambda t,y:t-0.5,direction=1,handler=handler)
    v=VehicleSpec("ship",s6,y,rhs6(s6),events=(ev,),mode="six",dof=6)
    result=MultiVehicleUniverseEngine().run((0,1),(v,))
    assert result.final_vehicles["ship"].dof==3
    assert [seg.dof for seg in result.segments["ship"]]==[6,3]
    vals=s3.unpack(result.final_vehicles["ship"].state)
    assert np.allclose(vals["position"],[3,0,0],atol=1e-8)


def test_fixed_step_vehicle_resynchronization_fallback():
    from uniflight import FixedStepRK4Integrator, FixedStepRK4Config
    s,ya=pack3(v=(1,0,0)); _,yb=pack3(v=(0,2,0))
    integ=FixedStepRK4Integrator(FixedStepRK4Config(step=.1,save_every_step=False))
    ev=VehicleEvent("tick",lambda t,y:t-0.75,direction=1,handler=lambda ctx:UniverseMutation(note="tick"))
    A=VehicleSpec("A",s,ya,rhs3(s),events=(ev,),integrator=integ,dof=3)
    B=VehicleSpec("B",s,yb,rhs3(s),integrator=integ,dof=3)
    result=MultiVehicleUniverseEngine(default_integrator=integ).run((0,1.5),(A,B))
    assert abs(result.events[0].time-.75)<2e-8
    assert np.allclose(s.unpack(result.final_vehicles["B"].state)["position"],[0,3,0],atol=1e-7)
