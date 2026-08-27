from __future__ import annotations

import argparse
import json
from pathlib import Path
import math
import numpy as np

from uniflight import (
    AxisMetadata, UncertaintyMetadata, OutputMetadata, DataProvenance,
    ValidityBound, ValidityEnvelope, ValidityPolicy, EngineeringTable,
    EngineeringDataCatalog, save_long_form_csv,
    GasSpecies, GasMixture, SphericalBody, PlanetaryEnvironment,
    TabulatedAtmosphere, TabulatedRadialGravity, TabulatedSphericalTerrain,
    EngineeringTableAeroCoefficients, ConstantReferenceGeometry,
    ContinuumAerodynamics6DOF, ConstantMassProperties,
    TabulatedRocketPerformance, TabulatedGimballedRocketEngine,
    RigidBody6DOFDynamics, QuaternionKinematics, DynamicsAssembler,
    ScipyIVPIntegrator, SolverConfig, SimulationEngine, core_6dof_schema,
    TabulatedMaterialProperties,
)


def provenance(dataset_id: str) -> DataProvenance:
    return DataProvenance(
        dataset_id=dataset_id,
        version="1.0",
        source="synthetic Nereid-K reference data",
        authors=("UniFlight reference generator",),
        citation="Milestone K synthetic verification/reference dataset",
        notes="Generated analytically; not flight-validated engineering data.",
    )


def build_reference_tables():
    gas = GasSpecies("NereidGas", 0.030, 30.0, 1.7e-5, 300.0, 120.0, 3.8e-10)
    mixture = GasMixture((gas,), (1.0,))
    body = SphericalBody(mu=1.0e10, radius=100_000.0, name="Nereid-K")

    atmosphere = EngineeringTable(
        (AxisMetadata("altitude", np.array([0., 2_500., 5_000.]), "m", extrapolation="clamp"),),
        {
            "temperature": np.array([250., 245., 240.]),
            "pressure": np.array([20_000., 14_000., 10_000.]),
        },
        {
            "temperature": OutputMetadata("temperature", "K"),
            "pressure": OutputMetadata("pressure", "Pa", uncertainty=UncertaintyMetadata("normal", relative_sigma=0.01)),
        },
        provenance=provenance("nereid-k.atmosphere"),
        description="Reference altitude atmosphere table",
    )

    radius = body.radius + np.array([0., 2_500., 5_000.])
    gravity = EngineeringTable(
        (AxisMetadata("radius", radius, "m", extrapolation="clamp"),),
        {"gravity": np.array([2.00, 1.90, 1.80])},
        {"gravity": OutputMetadata("gravity", "m/s^2")},
        provenance=provenance("nereid-k.gravity"),
    )

    mach = np.array([0., 0.5, 1., 2., 4.])
    alpha = np.deg2rad(np.array([-10., 0., 10.]))
    M, A = np.meshgrid(mach, alpha, indexing="ij")
    cd = 0.45 + 0.05*np.exp(-((M-1.0)/0.35)**2) + 0.2*A*A
    cl = 2.2*A
    aero_outputs = {
        "cd": cd,
        "cl": cl,
        "cy": np.zeros_like(cd),
        "c_roll": np.zeros_like(cd),
        "c_pitch": -0.35*A,
        "c_yaw": np.zeros_like(cd),
    }
    aero = EngineeringTable(
        (
            AxisMetadata("mach", mach, "1", extrapolation="clamp"),
            AxisMetadata("alpha", alpha, "rad", extrapolation="clamp"),
        ),
        aero_outputs,
        {name: OutputMetadata(name, "1", uncertainty=UncertaintyMetadata("normal", relative_sigma=0.02)) for name in aero_outputs},
        validity=ValidityEnvelope(
            (ValidityBound("mach", 0., 3.5), ValidityBound("alpha", np.deg2rad(-8), np.deg2rad(8))),
            ValidityPolicy.FLAG,
            "reference model recommended domain",
        ),
        provenance=provenance("nereid-k.aero"),
        description="Synthetic Mach/alpha aerodynamic coefficient database",
    )

    p = np.array([0., 10_000., 20_000.])
    u = np.array([0., .5, 1.])
    P, U = np.meshgrid(p, u, indexing="ij")
    propulsion = EngineeringTable(
        (AxisMetadata("ambient_pressure", p, "Pa", extrapolation="clamp"), AxisMetadata("throttle", u, "1", extrapolation="clamp")),
        {
            "thrust": U*(1_250. - 0.0125*P),
            "mass_flow": 0.52*U,
        },
        {
            "thrust": OutputMetadata("thrust", "N", uncertainty=UncertaintyMetadata("normal", relative_sigma=0.01)),
            "mass_flow": OutputMetadata("mass_flow", "kg/s", uncertainty=UncertaintyMetadata("normal", relative_sigma=0.005)),
        },
        provenance=provenance("nereid-k.propulsion"),
    )

    temperature = np.array([250., 800., 1_300.])
    material = EngineeringTable(
        (AxisMetadata("temperature", temperature, "K", extrapolation="clamp"),),
        {
            "specific_heat": np.array([950., 1_100., 1_250.]),
            "emissivity": np.array([0.78, 0.84, 0.88]),
            "thermal_conductivity": np.array([0.45, 0.52, 0.61]),
            "density": np.array([1_550., 1_500., 1_440.]),
            "ablation_temperature": np.array([1_050., 1_050., 1_050.]),
            "effective_heat_of_ablation": np.array([3.2e6, 3.2e6, 3.2e6]),
        },
        provenance=provenance("nereid-k.tps-material"),
    )

    lat = np.deg2rad(np.array([-30., 0., 30.]))
    lon = np.array([-math.pi, 0., math.pi])
    LAT, LON = np.meshgrid(lat, lon, indexing="ij")
    elevation = 50.0 + 30.0*LAT + 5.0*np.cos(LON)
    terrain = EngineeringTable(
        (
            AxisMetadata("latitude", lat, "rad", extrapolation="clamp"),
            AxisMetadata("longitude", lon, "rad", extrapolation="clamp", period=2*math.pi),
        ),
        {"elevation": elevation},
        {"elevation": OutputMetadata("elevation", "m")},
        provenance=provenance("nereid-k.terrain"),
    )
    return body, mixture, {"atmosphere": atmosphere, "gravity": gravity, "aero": aero, "propulsion": propulsion, "material": material, "terrain": terrain}


def run_reference(output_path: Path) -> dict:
    body, mixture, tables = build_reference_tables()
    data_dir = output_path.parent / "k_datasets"
    data_dir.mkdir(parents=True, exist_ok=True)
    catalog = EngineeringDataCatalog()
    dataset_records = {}
    for name, table in tables.items():
        npz = table.to_npz(data_dir / f"{name}.npz")
        csv = save_long_form_csv(table, data_dir / f"{name}.csv")
        loaded = catalog.load_npz(npz)
        dataset_records[name] = {
            "dataset_id": loaded.provenance.dataset_id,
            "version": loaded.provenance.version,
            "sha256": loaded.content_sha256(),
            "npz": str(npz),
            "csv": str(csv),
            "shape": loaded.shape,
            "axes": loaded.axis_names,
            "outputs": loaded.output_names,
        }

    atmosphere = TabulatedAtmosphere(catalog.resolve("nereid-k.atmosphere", "1.0"), mixture)
    gravity = TabulatedRadialGravity(catalog.resolve("nereid-k.gravity", "1.0"))
    environment = PlanetaryEnvironment(body, atmosphere, gravity_model=gravity)
    mass_properties = ConstantMassProperties(np.diag([20., 24., 24.]))
    aero_model = EngineeringTableAeroCoefficients(catalog.resolve("nereid-k.aero", "1.0"))
    aero = ContinuumAerodynamics6DOF(
        environment,
        ConstantReferenceGeometry(1.1, 1.5, 1.2, 1.0, np.zeros(3)),
        aero_model,
        mass_properties,
    )
    engine = TabulatedGimballedRocketEngine(
        environment,
        mass_properties,
        TabulatedRocketPerformance(catalog.resolve("nereid-k.propulsion", "1.0")),
        throttle=1.0,
        pitch_gimbal=0.0,
        yaw_gimbal=0.0,
    )

    schema = core_6dof_schema()
    y0 = schema.pack({
        "position": np.array([body.radius, 0., 0.]),
        "velocity": np.zeros(3),
        "attitude": np.array([1., 0., 0., 0.]),
        "angular_rate": np.zeros(3),
        "mass": 100.0,
    })
    rhs = DynamicsAssembler(schema, [
        RigidBody6DOFDynamics(mass_properties, gravity=gravity, wrench_models=(engine, aero)),
        QuaternionKinematics(),
        engine,
    ]).rhs
    result = SimulationEngine(
        rhs, ScipyIVPIntegrator(SolverConfig(rtol=1e-9, atol=1e-11, max_step=0.05))
    ).run((0.0, 6.0), y0)
    final = schema.unpack(result.states[-1])

    material = TabulatedMaterialProperties(catalog.resolve("nereid-k.tps-material", "1.0"))
    mat_900 = material.query(temperature=900.0)
    terrain = TabulatedSphericalTerrain(body, catalog.resolve("nereid-k.terrain", "1.0"))
    terrain_eq = terrain.query(np.array([body.radius + 100.0, 0., 0.]))

    aero_query = catalog.resolve("nereid-k.aero", "1.0").query({"mach": 4.0, "alpha": np.deg2rad(9.0)})

    report = {
        "uniflight_version": "0.11.0",
        "body": body.name,
        "catalog_inventory": catalog.inventory(),
        "datasets": dataset_records,
        "flight": {
            "success": bool(result.success),
            "duration_s": float(result.times[-1]),
            "final_altitude_m": float(np.linalg.norm(final["position"]) - body.radius),
            "final_speed_mps": float(np.linalg.norm(final["velocity"])),
            "final_mass_kg": float(final["mass"]),
            "quaternion_norm": float(np.linalg.norm(final["attitude"])),
        },
        "material_at_900K": dict(mat_900.properties),
        "terrain_equator": {
            "elevation_m": terrain_eq.elevation,
            "normal_i": terrain_eq.normal_i.tolist(),
        },
        "validity_demo": {
            "validity_ok": aero_query.validity_ok,
            "violations": list(aero_query.validity_violations),
            "cd": aero_query.value("cd"),
            "cd_sigma": aero_query.standard_uncertainty["cd"],
        },
    }
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description="UniFlight Milestone K engineering-data reference")
    parser.add_argument("--output", type=Path, default=Path("reports/k_reference.json"))
    args = parser.parse_args()
    report = run_reference(args.output)
    print(json.dumps(report["flight"], indent=2))
    print(f"catalog datasets: {len(report['catalog_inventory'])}")
    print(f"reference report: {args.output}")


if __name__ == "__main__":
    main()
