"""Milestone-I reference mission: staging + concurrent propagation + DOF change.

A 6-DOF stack coasts around fictional body Nereid-I, separates into an upper
vehicle and booster at t=5 s, then the upper vehicle demotes from 6-DOF to
3-DOF at t=8 s while the booster remains 6-DOF.  Both daughters continue to be
propagated concurrently in one global event-synchronized universe.
"""
from __future__ import annotations
import json
from pathlib import Path
import numpy as np

from uniflight import (
    SphericalBody, core_3dof_schema, core_6dof_schema,
    ConstantMassProperties, RigidBody6DOFDynamics, QuaternionKinematics,
    TranslationalKinematics, DynamicsAssembler,
    VehicleEvent, VehicleSpec, VehicleConfiguration, DOFSwitchHandler,
    RigidChildTemplate, RigidSeparationHandler, MultiVehicleUniverseEngine,
)

BODY = SphericalBody(mu=1.5e12, radius=1.0e6, name="Nereid-I")
S6 = core_6dof_schema(); S3 = core_3dof_schema()

M_UPPER, M_BOOSTER = 4000.0, 6000.0
R_UPPER_B = np.array([3.0,0.0,0.0])
R_BOOSTER_B = np.array([-2.0,0.0,0.0])
I_UPPER = np.diag([5_000.0,8_000.0,8_000.0])
I_BOOSTER = np.diag([10_000.0,16_000.0,16_000.0])

def pa(m,r):
    return m*((r@r)*np.eye(3)-np.outer(r,r))
I_STACK = I_UPPER+I_BOOSTER+pa(M_UPPER,R_UPPER_B)+pa(M_BOOSTER,R_BOOSTER_B)


def rhs6(inertia):
    dyn = RigidBody6DOFDynamics(ConstantMassProperties(inertia), gravity=BODY.gravity)
    return DynamicsAssembler(S6,[dyn,QuaternionKinematics()]).rhs

RHS_UPPER_6 = rhs6(I_UPPER)
RHS_BOOSTER_6 = rhs6(I_BOOSTER)
RHS_STACK_6 = rhs6(I_STACK)
RHS_UPPER_3 = DynamicsAssembler(S3,[TranslationalKinematics(gravity=BODY.gravity)]).rhs

UPPER_3 = VehicleConfiguration(S3,RHS_UPPER_3,mode="upper-coast-3dof",dof=3,
                               model_context={"body":BODY.name,"role":"upper"})
DEMOTE = DOFSwitchHandler(UPPER_3,note="upper 6DOF -> 3DOF coast")
UPPER_DEMOTE_EVENT = VehicleEvent("upper_demote",lambda t,y:t-8.0,direction=1,priority=20,handler=DEMOTE)
UPPER_6 = VehicleConfiguration(S6,RHS_UPPER_6,events=(UPPER_DEMOTE_EVENT,),mode="upper-6dof",dof=6,
                               model_context={"body":BODY.name,"role":"upper"})
BOOSTER_6 = VehicleConfiguration(S6,RHS_BOOSTER_6,mode="booster-6dof",dof=6,
                                 model_context={"body":BODY.name,"role":"booster"})

SEPARATE = RigidSeparationHandler(
    retained=RigidChildTemplate("upper",UPPER_6),
    detached=RigidChildTemplate("booster",BOOSTER_6),
    retained_mass=M_UPPER,
    detached_mass=M_BOOSTER,
    retained_offset_b=R_UPPER_B,
    detached_offset_b=R_BOOSTER_B,
    parent_inertia_b=I_STACK,
    retained_inertia_b=I_UPPER,
    detached_inertia_b=I_BOOSTER,
    relative_separation_velocity_i=np.array([0.0,2.0,0.0]),
    note="stage separation",
)
STACK_SEP_EVENT = VehicleEvent("stage_separation",lambda t,y:t-5.0,direction=1,priority=100,handler=SEPARATE)

Y0 = S6.pack({
    "position": np.array([BODY.radius+1000.0,0.0,0.0]),
    "velocity": np.array([250.0,600.0,0.0]),
    "attitude": np.array([1.0,0.0,0.0,0.0]),
    "angular_rate": np.array([0.0,0.0,0.02]),
    "mass": M_UPPER+M_BOOSTER,
})
STACK = VehicleSpec("stack",S6,Y0,RHS_STACK_6,events=(STACK_SEP_EVENT,),mode="stack-6dof",dof=6,
                    model_context={"body":BODY.name,"role":"stack"})


def main(output: str | None = None):
    result = MultiVehicleUniverseEngine().run((0.0,20.0),(STACK,))
    final = {}
    for vid,snap in result.final_vehicles.items():
        values=snap.schema.unpack(snap.state)
        final[vid]={
            "mode":snap.mode,
            "dof":snap.dof,
            "altitude_m":BODY.altitude(values["position"]),
            "speed_mps":float(np.linalg.norm(values["velocity"])),
            "mass_kg":float(values["mass"]),
            "segments":len(result.segments[vid]),
        }
    report={
        "body":BODY.name,
        "events":[{"time_s":e.time,"vehicle":e.vehicle_id,"event":e.event_name,
                   "note":e.mutation_note,"active_after":list(e.active_vehicle_ids_after)} for e in result.events],
        "final":final,
    }
    print(json.dumps(report,indent=2))
    if output:
        p=Path(output); p.parent.mkdir(parents=True,exist_ok=True); p.write_text(json.dumps(report,indent=2))
    return report

if __name__ == "__main__":
    import argparse
    ap=argparse.ArgumentParser(); ap.add_argument("--output"); args=ap.parse_args(); main(args.output)
