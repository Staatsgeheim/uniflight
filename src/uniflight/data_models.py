from __future__ import annotations

"""Domain adapters from the Milestone-K engineering-data system to flight models."""

from dataclasses import dataclass, field
from typing import Callable, Mapping
import math
import numpy as np

from .engineering_data import EngineeringTable, TableQueryResult
from .aerodynamics import AeroCoefficients
from .atmosphere import AtmosphereSample
from .chemistry import FrozenChemistry
from .environment import PlanetaryEnvironment, EnvironmentSample
from .flow import BodyFlowState, FlowState, compute_flow_state
from .frames import body_to_inertial_matrix
from .heating import AerothermalEvaluation
from .mass_properties import MassPropertiesModel
from .propulsion import _rot_y, _rot_z
from .state import StateView
from .terrain import TerrainSample
from .wrenches import Wrench
from .gases import GasMixture


# ---------------------------------------------------------------------------
# Shared axis resolvers
# ---------------------------------------------------------------------------


def _body_flow_value(name: str, flow: BodyFlowState) -> float:
    aliases = {
        "mach": flow.mach,
        "alpha": flow.alpha,
        "beta": flow.beta,
        "reynolds": flow.reynolds,
        "re": flow.reynolds,
        "knudsen": flow.knudsen,
        "kn": flow.knudsen,
        "dynamic_pressure": flow.dynamic_pressure,
        "q": flow.dynamic_pressure,
        "speed": flow.speed,
    }
    if name not in aliases:
        raise KeyError(f"no default aerodynamic resolver for table axis {name!r}")
    return float(aliases[name])


def _aerothermal_value(name: str, env: EnvironmentSample, flow: FlowState) -> float:
    atm = env.atmosphere
    aliases = {
        "altitude": env.altitude,
        "temperature": atm.temperature,
        "pressure": atm.pressure,
        "density": atm.density,
        "viscosity": atm.viscosity,
        "speed_of_sound": atm.speed_of_sound,
        "mean_free_path": atm.mean_free_path,
        "speed": flow.speed,
        "mach": flow.mach,
        "reynolds": flow.reynolds,
        "re": flow.reynolds,
        "knudsen": flow.knudsen,
        "kn": flow.knudsen,
        "dynamic_pressure": flow.dynamic_pressure,
        "q": flow.dynamic_pressure,
    }
    if name not in aliases:
        raise KeyError(f"no default aerothermal resolver for table axis {name!r}")
    return float(aliases[name])


# ---------------------------------------------------------------------------
# Aerodynamic database adapter
# ---------------------------------------------------------------------------


@dataclass(frozen=True, slots=True)
class EngineeringTableAeroCoefficients:
    """N-D engineering table exposed as a 6-DOF aerodynamic coefficient model.

    Axes may use any subset of Mach, alpha, beta, Reynolds, Knudsen, dynamic
    pressure and speed.  Additional axes can be supplied through
    ``axis_resolvers``.
    """

    table: EngineeringTable
    field_map: Mapping[str, str] = field(default_factory=lambda: {
        "cd": "cd", "cl": "cl", "cy": "cy",
        "c_roll": "c_roll", "c_pitch": "c_pitch", "c_yaw": "c_yaw",
    })
    axis_resolvers: Mapping[str, Callable[[BodyFlowState], float]] = field(default_factory=dict)

    def __post_init__(self) -> None:
        defaults = {"cd", "cl", "cy", "c_roll", "c_pitch", "c_yaw"}
        missing_keys = defaults - set(self.field_map)
        if missing_keys:
            raise ValueError(f"field_map missing coefficient keys {sorted(missing_keys)}")
        missing_outputs = {v for v in self.field_map.values()} - set(self.table.output_names)
        if missing_outputs:
            raise ValueError(f"aero table missing outputs {sorted(missing_outputs)}")

    def query(self, flow: BodyFlowState) -> tuple[AeroCoefficients, TableQueryResult]:
        coordinates = {}
        for axis in self.table.axis_names:
            if axis in self.axis_resolvers:
                coordinates[axis] = float(self.axis_resolvers[axis](flow))
            else:
                coordinates[axis] = _body_flow_value(axis, flow)
        q = self.table.query(coordinates)
        coeff = AeroCoefficients(**{
            name: q.value(output_name) for name, output_name in self.field_map.items()
        })
        return coeff, q

    def __call__(self, flow: BodyFlowState) -> AeroCoefficients:
        return self.query(flow)[0]


# ---------------------------------------------------------------------------
# Aerothermal database adapter
# ---------------------------------------------------------------------------


@dataclass(frozen=True, slots=True)
class TabulatedAerothermalModel:
    environment: PlanetaryEnvironment
    reference_length: float
    table: EngineeringTable
    convective_output: str = "convective_heat_flux"
    radiative_output: str = "radiative_heat_flux"
    axis_resolvers: Mapping[str, Callable[[EnvironmentSample, FlowState], float]] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not np.isfinite(self.reference_length) or self.reference_length <= 0:
            raise ValueError("reference_length must be finite and positive")
        if self.convective_output not in self.table.output_names:
            raise ValueError(f"table missing {self.convective_output!r}")
        # Radiative output is optional; absent means zero radiative heating.

    def evaluate_with_query(self, state: StateView) -> tuple[AerothermalEvaluation, TableQueryResult]:
        env = self.environment.query(state.get("position"), state.time)
        flow = compute_flow_state(state.get("velocity"), env, self.reference_length)
        coords = {}
        for axis in self.table.axis_names:
            if axis in self.axis_resolvers:
                coords[axis] = float(self.axis_resolvers[axis](env, flow))
            else:
                coords[axis] = _aerothermal_value(axis, env, flow)
        q = self.table.query(coords)
        q_conv = max(0.0, q.value(self.convective_output))
        q_rad = max(0.0, q.value(self.radiative_output)) if self.radiative_output in q.values else 0.0
        chemistry = FrozenChemistry().evaluate(env, flow)
        return AerothermalEvaluation(env, flow, chemistry, q_conv, q_rad, q_conv + q_rad), q

    def evaluate(self, state: StateView) -> AerothermalEvaluation:
        return self.evaluate_with_query(state)[0]


# ---------------------------------------------------------------------------
# Propulsion database adapter
# ---------------------------------------------------------------------------


@dataclass(frozen=True, slots=True)
class RocketPerformanceEvaluation:
    thrust: float
    mass_flow: float
    query: TableQueryResult


@dataclass(frozen=True, slots=True)
class TabulatedRocketPerformance:
    """Tabulated propulsion performance as a function of ambient/control state.

    Default recognized axes are ``ambient_pressure`` and ``throttle``.  Extra
    axes (mixture ratio, chamber pressure command, inlet state, etc.) can be
    provided through ``axis_providers`` at evaluation time.
    """

    table: EngineeringTable
    thrust_output: str = "thrust"
    mass_flow_output: str = "mass_flow"

    def __post_init__(self) -> None:
        for name in (self.thrust_output, self.mass_flow_output):
            if name not in self.table.output_names:
                raise ValueError(f"propulsion table missing output {name!r}")

    def evaluate(self, ambient_pressure: float, throttle: float, **coordinates: float) -> RocketPerformanceEvaluation:
        base = {
            "ambient_pressure": float(ambient_pressure),
            "throttle": float(throttle),
        }
        base.update({k: float(v) for k,v in coordinates.items()})
        query = self.table.query({name: base[name] for name in self.table.axis_names})
        thrust = query.value(self.thrust_output)
        mass_flow = query.value(self.mass_flow_output)
        if thrust < 0 or mass_flow < 0:
            raise ValueError("tabulated rocket performance returned negative thrust/mass flow")
        return RocketPerformanceEvaluation(float(thrust), float(mass_flow), query)


@dataclass(frozen=True, slots=True)
class TabulatedRocket6DOFEvaluation:
    ambient_pressure: float
    throttle: float
    mass_flow: float
    thrust: float
    pitch_gimbal: float
    yaw_gimbal: float
    direction_b: np.ndarray
    force_b: np.ndarray
    force_i: np.ndarray
    moment_b_about_cg: np.ndarray
    table_query: TableQueryResult


@dataclass(frozen=True, slots=True)
class TabulatedGimballedRocketEngine:
    environment: PlanetaryEnvironment
    mass_properties: MassPropertiesModel
    performance: TabulatedRocketPerformance
    mount_position_b: np.ndarray = field(default_factory=lambda: np.zeros(3))
    base_direction_b: np.ndarray = field(default_factory=lambda: np.array([1.0,0.0,0.0]))
    throttle: float | Callable[[StateView], float] = 1.0
    pitch_gimbal: float | Callable[[StateView], float] = 0.0
    yaw_gimbal: float | Callable[[StateView], float] = 0.0
    dry_mass: float = 0.0
    extra_coordinates: Mapping[str, float | Callable[[StateView], float]] = field(default_factory=dict)
    source: str = "tabulated-gimballed-rocket"

    def __post_init__(self) -> None:
        mount = np.asarray(self.mount_position_b, dtype=float)
        base = np.asarray(self.base_direction_b, dtype=float)
        if mount.shape != (3,) or not np.all(np.isfinite(mount)):
            raise ValueError("mount_position_b must be a finite 3-vector")
        if base.shape != (3,) or not np.all(np.isfinite(base)) or np.linalg.norm(base) == 0:
            raise ValueError("base_direction_b must be a nonzero finite 3-vector")
        object.__setattr__(self, "mount_position_b", mount.copy())
        object.__setattr__(self, "base_direction_b", base/np.linalg.norm(base))
        if not np.isfinite(self.dry_mass) or self.dry_mass < 0:
            raise ValueError("dry_mass must be finite and non-negative")

    @staticmethod
    def _value(provider, state: StateView) -> float:
        return float(provider(state) if callable(provider) else provider)

    def evaluate(self, state: StateView) -> TabulatedRocket6DOFEvaluation:
        env = self.environment.query(state.get("position"), state.time)
        throttle = self._value(self.throttle, state)
        if float(state.get("mass")) <= self.dry_mass:
            throttle = 0.0
        if not np.isfinite(throttle) or not 0 <= throttle <= 1:
            raise ValueError("throttle must lie in [0,1]")
        pitch = self._value(self.pitch_gimbal, state)
        yaw = self._value(self.yaw_gimbal, state)
        extra = {k: self._value(v, state) for k,v in self.extra_coordinates.items()}
        perf = self.performance.evaluate(env.atmosphere.pressure, throttle, **extra)
        direction_b = _rot_z(yaw) @ _rot_y(pitch) @ self.base_direction_b
        direction_b /= np.linalg.norm(direction_b)
        force_b = perf.thrust * direction_b
        force_i = body_to_inertial_matrix(state.get("attitude")) @ force_b
        mp = self.mass_properties.evaluate(state)
        moment_b = np.cross(self.mount_position_b - mp.cg_b, force_b)
        return TabulatedRocket6DOFEvaluation(
            env.atmosphere.pressure, throttle, perf.mass_flow, perf.thrust,
            pitch, yaw, direction_b, force_b, force_i, moment_b, perf.query,
        )

    def wrench(self, state: StateView) -> Wrench:
        e = self.evaluate(state)
        return Wrench(e.force_i, e.moment_b_about_cg, self.source)

    def mass_rate(self, state: StateView) -> float:
        return -self.evaluate(state).mass_flow

    def derivatives(self, state: StateView) -> dict[str,float]:
        return {"mass": self.mass_rate(state)}


# ---------------------------------------------------------------------------
# Material-property database adapter
# ---------------------------------------------------------------------------


@dataclass(frozen=True, slots=True)
class MaterialEvaluation:
    properties: Mapping[str,float]
    query: TableQueryResult

    def get(self, name: str) -> float:
        return float(self.properties[name])


@dataclass(frozen=True, slots=True)
class TabulatedMaterialProperties:
    table: EngineeringTable

    def query(self, **conditions: float) -> MaterialEvaluation:
        q = self.table.query({name: float(conditions[name]) for name in self.table.axis_names})
        return MaterialEvaluation(dict(q.values), q)


# ---------------------------------------------------------------------------
# Gravity dataset adapters
# ---------------------------------------------------------------------------


@dataclass(frozen=True, slots=True)
class TabulatedRadialGravity:
    """Radially symmetric gravity magnitude from an engineering table."""

    table: EngineeringTable
    gravity_output: str = "gravity"
    radial_axis: str = "radius"   # ``radius`` or ``altitude`` are conventional
    reference_radius: float | None = None
    potential_output: str | None = "potential"

    def __post_init__(self) -> None:
        if self.radial_axis not in self.table.axis_names:
            raise ValueError(f"gravity table missing radial axis {self.radial_axis!r}")
        if self.gravity_output not in self.table.output_names:
            raise ValueError(f"gravity table missing output {self.gravity_output!r}")
        if self.radial_axis == "altitude":
            if self.reference_radius is None or not np.isfinite(self.reference_radius) or self.reference_radius <= 0:
                raise ValueError("altitude-based gravity table requires positive reference_radius")
        if self.potential_output is not None and self.potential_output not in self.table.output_names:
            object.__setattr__(self, "potential_output", None)

    def _coord(self, radius: float) -> dict[str,float]:
        value = radius if self.radial_axis == "radius" else radius - float(self.reference_radius)
        if len(self.table.axis_names) != 1:
            raise ValueError("TabulatedRadialGravity requires a one-dimensional radial table")
        return {self.radial_axis: float(value)}

    def acceleration(self, position_i: np.ndarray, time: float = 0.0) -> np.ndarray:
        r = np.asarray(position_i, dtype=float)
        nrm = float(np.linalg.norm(r))
        if r.shape != (3,) or not np.all(np.isfinite(r)) or nrm <= 0:
            raise ValueError("invalid position for tabulated radial gravity")
        g = self.table.query(self._coord(nrm)).value(self.gravity_output)
        if g < 0:
            raise ValueError("gravity magnitude table returned negative value")
        return -g * r/nrm

    def potential(self, position_i: np.ndarray) -> float:
        if self.potential_output is None:
            raise NotImplementedError("gravity table has no potential output")
        r = float(np.linalg.norm(np.asarray(position_i, dtype=float)))
        return self.table.query(self._coord(r)).value(self.potential_output)

    def jacobian(self, position_i: np.ndarray, time: float = 0.0) -> np.ndarray:
        r = np.asarray(position_i, dtype=float)
        radius = float(np.linalg.norm(r))
        if r.shape != (3,) or not np.all(np.isfinite(r)) or radius <= 0:
            raise ValueError("invalid position for gravity gradient")
        coord = self._coord(radius)
        g = self.table.query(coord).value(self.gravity_output)
        gp = self.table.derivative(self.gravity_output, coord, self.radial_axis)
        n = r/radius
        nn = np.outer(n,n)
        return -gp*nn - (g/radius)*(np.eye(3)-nn)


@dataclass(frozen=True, slots=True)
class TabulatedCartesianGravity:
    """General Cartesian vector gravity field on an N-D table.

    Required axes are x/y/z; an optional time axis is supported.  This is a
    convenient adapter for precomputed irregular-body or multi-source fields.
    """

    table: EngineeringTable
    x_axis: str = "x"
    y_axis: str = "y"
    z_axis: str = "z"
    time_axis: str | None = None
    gx_output: str = "gx"
    gy_output: str = "gy"
    gz_output: str = "gz"

    def __post_init__(self) -> None:
        required_axes = {self.x_axis,self.y_axis,self.z_axis}
        if self.time_axis is not None:
            required_axes.add(self.time_axis)
        if required_axes != set(self.table.axis_names):
            raise ValueError("Cartesian gravity table axes must match x/y/z and optional time exactly")
        required_outputs = {self.gx_output,self.gy_output,self.gz_output}
        if not required_outputs.issubset(self.table.output_names):
            raise ValueError("Cartesian gravity table missing acceleration components")

    def _coords(self, r: np.ndarray, time: float) -> dict[str,float]:
        c = {self.x_axis:float(r[0]), self.y_axis:float(r[1]), self.z_axis:float(r[2])}
        if self.time_axis is not None:
            c[self.time_axis] = float(time)
        return c

    def acceleration(self, position_i: np.ndarray, time: float = 0.0) -> np.ndarray:
        r = np.asarray(position_i, dtype=float)
        if r.shape != (3,) or not np.all(np.isfinite(r)):
            raise ValueError("position_i must be a finite 3-vector")
        q = self.table.query(self._coords(r,time))
        return np.array([q.value(self.gx_output),q.value(self.gy_output),q.value(self.gz_output)])

    def jacobian(self, position_i: np.ndarray, time: float = 0.0) -> np.ndarray:
        r = np.asarray(position_i,dtype=float)
        c = self._coords(r,time)
        outs = (self.gx_output,self.gy_output,self.gz_output)
        axes = (self.x_axis,self.y_axis,self.z_axis)
        return np.array([[self.table.derivative(out,c,ax) for ax in axes] for out in outs],dtype=float)


# ---------------------------------------------------------------------------
# Terrain dataset adapter
# ---------------------------------------------------------------------------


@dataclass(frozen=True, slots=True)
class TabulatedSphericalTerrain:
    body: object
    table: EngineeringTable
    elevation_output: str = "elevation"
    latitude_axis: str = "latitude"
    longitude_axis: str = "longitude"
    slope_aware_normal: bool = True

    def __post_init__(self) -> None:
        if set(self.table.axis_names) != {self.latitude_axis,self.longitude_axis}:
            raise ValueError("spherical terrain table must contain latitude and longitude axes")
        if self.elevation_output not in self.table.output_names:
            raise ValueError("terrain table missing elevation output")
        if not hasattr(self.body,"radius") or not hasattr(self.body,"rotational_velocity_i"):
            raise TypeError("terrain body must expose radius and rotational_velocity_i")

    def _angles(self, r: np.ndarray) -> tuple[float,float]:
        radius = np.linalg.norm(r)
        lat = math.asin(float(np.clip(r[2]/radius,-1.0,1.0)))
        lon = math.atan2(float(r[1]),float(r[0]))
        return lat,lon

    def query(self, position_i: np.ndarray, time: float = 0.0) -> TerrainSample:
        r = np.asarray(position_i,dtype=float)
        radius = float(np.linalg.norm(r))
        if r.shape != (3,) or not np.all(np.isfinite(r)) or radius <= 0:
            raise ValueError("position_i must be a finite nonzero 3-vector")
        lat,lon = self._angles(r)
        coords = {self.latitude_axis:lat,self.longitude_axis:lon}
        elev = self.table.query(coords).value(self.elevation_output)
        rs = float(self.body.radius)+elev
        n_radial = r/radius
        surface_point = rs*n_radial
        normal = n_radial
        if self.slope_aware_normal and abs(math.cos(lat)) > 1e-8:
            dh_dlat = self.table.derivative(self.elevation_output,coords,self.latitude_axis)
            dh_dlon = self.table.derivative(self.elevation_output,coords,self.longitude_axis)
            cl,sl = math.cos(lat),math.sin(lat)
            co,so = math.cos(lon),math.sin(lon)
            n = np.array([cl*co,cl*so,sl])
            dn_lat = np.array([-sl*co,-sl*so,cl])
            dn_lon = np.array([-cl*so,cl*co,0.0])
            t_lat = dh_dlat*n + rs*dn_lat
            t_lon = dh_dlon*n + rs*dn_lon
            normal = np.cross(t_lon,t_lat)
            norm = np.linalg.norm(normal)
            if norm > 0:
                normal /= norm
                if np.dot(normal,n)<0:
                    normal = -normal
            else:
                normal = n_radial
        return TerrainSample(
            r.copy(),float(time),float(elev),float(radius-rs),normal,
            surface_point,self.body.rotational_velocity_i(surface_point),
        )


# ---------------------------------------------------------------------------
# Atmosphere dataset adapter
# ---------------------------------------------------------------------------


@dataclass(frozen=True, slots=True)
class TabulatedAtmosphere:
    """Atmosphere model backed by an N-D engineering table.

    The existing AtmosphereModel interface supplies altitude and time, so those
    are the default axis names.  A static GasMixture can be supplied to derive
    density/transport properties from tabulated pressure and temperature when
    those outputs are absent.
    """

    table: EngineeringTable
    mixture: GasMixture | None = None
    altitude_axis: str = "altitude"
    time_axis: str | None = None

    def __post_init__(self) -> None:
        allowed = {self.altitude_axis}
        if self.time_axis is not None:
            allowed.add(self.time_axis)
        if set(self.table.axis_names) != allowed:
            raise ValueError("TabulatedAtmosphere axes must be altitude and optional time")
        if "temperature" not in self.table.output_names or "pressure" not in self.table.output_names:
            raise ValueError("atmosphere table requires temperature and pressure outputs")

    def query(self, altitude: float, time: float = 0.0) -> AtmosphereSample:
        c = {self.altitude_axis:float(altitude)}
        if self.time_axis is not None:
            c[self.time_axis]=float(time)
        q = self.table.query(c)
        T = q.value("temperature")
        p = q.value("pressure")
        if p <= 0 or T <= 0:
            return AtmosphereSample(float(altitude),max(0.0,T),max(0.0,p),0.0,0.0,math.inf,math.inf,self.mixture)
        if "density" in q.values:
            rho = q.value("density")
        elif self.mixture is not None:
            rho = self.mixture.density(p,T)
        else:
            raise ValueError("atmosphere table needs density output or a GasMixture")
        if "viscosity" in q.values:
            mu = q.value("viscosity")
        elif self.mixture is not None:
            mu = self.mixture.viscosity(T)
        else:
            raise ValueError("atmosphere table needs viscosity output or a GasMixture")
        if "speed_of_sound" in q.values:
            a = q.value("speed_of_sound")
        elif self.mixture is not None:
            a = self.mixture.speed_of_sound(T)
        else:
            raise ValueError("atmosphere table needs speed_of_sound output or a GasMixture")
        if "mean_free_path" in q.values:
            mfp = q.value("mean_free_path")
        elif self.mixture is not None:
            mfp = self.mixture.mean_free_path(p,T)
        else:
            mfp = math.inf
        return AtmosphereSample(float(altitude),float(T),float(p),float(rho),float(mu),float(a),float(mfp),self.mixture)

# ---------------------------------------------------------------------------
# Material-backed TPS adapter
# ---------------------------------------------------------------------------

from .tps import STEFAN_BOLTZMANN, TPSEvaluation


@dataclass(frozen=True, slots=True)
class TabulatedMaterialLumpedTPS:
    """Lumped TPS whose thermophysical properties come from a material table.

    Required material outputs are ``specific_heat``, ``emissivity``,
    ``ablation_temperature`` and ``effective_heat_of_ablation``.  Table axes
    may include ``temperature`` and ``pressure``; extra axes can be provided as
    constants/callables.  This remains a low-order thermal node, but it proves
    that material databases can drive the existing TPS state/mass coupling.
    """

    heating_model: object
    material: TabulatedMaterialProperties
    heated_area: float
    thermal_mass: float
    temperature_key: str = "tps_temperature"
    heat_load_key: str = "heat_load"
    tps_mass_key: str = "tps_mass"
    extra_conditions: Mapping[str, float | Callable[[StateView], float]] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not np.isfinite(self.heated_area) or self.heated_area <= 0:
            raise ValueError("heated_area must be finite and positive")
        if not np.isfinite(self.thermal_mass) or self.thermal_mass <= 0:
            raise ValueError("thermal_mass must be finite and positive")
        required = {"specific_heat","emissivity","ablation_temperature","effective_heat_of_ablation"}
        if not required.issubset(self.material.table.output_names):
            raise ValueError(f"material table missing TPS properties {sorted(required-set(self.material.table.output_names))}")

    @staticmethod
    def _value(provider, state: StateView) -> float:
        return float(provider(state) if callable(provider) else provider)

    def evaluate(self, state: StateView) -> TPSEvaluation:
        aero = self.heating_model.evaluate(state)
        T = float(state.get(self.temperature_key))
        m_tps = float(state.get(self.tps_mass_key))
        conditions = {}
        for axis in self.material.table.axis_names:
            if axis == "temperature":
                conditions[axis] = T
            elif axis == "pressure":
                conditions[axis] = aero.environment.atmosphere.pressure
            elif axis in self.extra_conditions:
                conditions[axis] = self._value(self.extra_conditions[axis],state)
            else:
                raise KeyError(f"no TPS material condition provider for axis {axis!r}")
        mat = self.material.query(**conditions)
        cp = mat.get("specific_heat")
        emissivity = mat.get("emissivity")
        t_ab = mat.get("ablation_temperature")
        h_ab = mat.get("effective_heat_of_ablation")
        if cp <= 0 or h_ab <= 0 or t_ab <= 0 or not 0 <= emissivity <= 1:
            raise ValueError("material table returned invalid TPS properties")
        T_inf = max(0.0,float(aero.environment.atmosphere.temperature))
        incident = aero.total_heat_flux*self.heated_area
        emitted = emissivity*STEFAN_BOLTZMANN*self.heated_area*max(0.0,T**4-T_inf**4)
        net = incident-emitted
        mdot = 0.0
        if T >= t_ab and net > 0 and m_tps > 0:
            mdot = net/h_ab
            dT = 0.0
        else:
            dT = net/(self.thermal_mass*cp)
        return TPSEvaluation(aero,T,m_tps,incident,emitted,net,float(mdot),float(dT))

    def derivatives(self,state:StateView)->dict[str,float]:
        e=self.evaluate(state)
        return {
            self.temperature_key:e.temperature_rate,
            self.heat_load_key:e.aerothermal.total_heat_flux,
            self.tps_mass_key:-e.ablation_mass_rate,
        }

    def mass_rate(self,state:StateView)->float:
        return -self.evaluate(state).ablation_mass_rate
