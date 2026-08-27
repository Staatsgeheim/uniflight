import math
import numpy as np

from uniflight import (
    GasSpecies, GasMixture, SphericalBody,
    IsothermalHydrostaticAtmosphere, VacuumAtmosphere,
    PlanetaryEnvironment,
)


def _species():
    a = GasSpecies("A", 0.028, 29.1, 1.75e-5, 300.0, 110.0, 3.7e-10)
    b = GasSpecies("B", 0.044, 37.1, 1.48e-5, 300.0, 240.0, 4.0e-10)
    return a, b


def test_009_gas_mixture_thermodynamic_closure():
    a, b = _species()
    mix = GasMixture((a, b), (0.7, 0.3))
    expected_m = 0.7 * 0.028 + 0.3 * 0.044
    assert abs(mix.molar_mass - expected_m) < 1e-15
    assert mix.specific_gas_constant > 0
    assert mix.cp_mass > mix.cv_mass > 0
    assert mix.gamma > 1.0
    assert abs(np.sum(mix.mass_fractions) - 1.0) < 1e-15
    assert mix.viscosity(250.0) > 0
    assert mix.mean_free_path(1e5, 250.0) > 0


def test_010_spherical_hydrostatic_pressure_matches_exact_integral():
    a, _ = _species()
    mix = GasMixture((a,), (1.0,))
    mu, radius, T, p0, h = 1.5e12, 8.0e5, 240.0, 80_000.0, 12_000.0
    atm = IsothermalHydrostaticAtmosphere(p0, T, mix, mu, radius)
    sample = atm.query(h)
    exponent = mu / (mix.specific_gas_constant * T) * (1/(radius+h) - 1/radius)
    expected_p = p0 * math.exp(exponent)
    assert abs(sample.pressure - expected_p) / expected_p < 1e-14
    assert abs(sample.density - expected_p/(mix.specific_gas_constant*T)) / sample.density < 1e-14
    assert sample.speed_of_sound > 0
    assert sample.mean_free_path > 0


def test_011_environment_combines_body_rotation_and_wind():
    a, _ = _species()
    mix = GasMixture((a,), (1.0,))
    body = SphericalBody(mu=2e12, radius=1e6, rotation_vector_i=np.array([0., 0., 2e-4]))
    atm = IsothermalHydrostaticAtmosphere(10_000.0, 220.0, mix, body.mu, body.radius)
    wind = lambda r, t, s: np.array([3.0, 4.0, 0.0])
    env = PlanetaryEnvironment(body, atm, wind)
    sample = env.query(np.array([body.radius, 0., 0.]))
    expected = np.array([3.0, body.rotation_vector_i[2]*body.radius + 4.0, 0.0])
    assert np.allclose(sample.fluid_velocity_i, expected, atol=1e-12, rtol=0)
    assert abs(sample.altitude) < 1e-12
    assert np.allclose(sample.surface_normal_i, [1,0,0])


def test_012_atmosphere_ceiling_returns_vacuum():
    a, _ = _species()
    mix = GasMixture((a,), (1.0,))
    atm = IsothermalHydrostaticAtmosphere(50_000.0, 250.0, mix, 1e12, 1e6, ceiling=20_000.0)
    sample = atm.query(25_000.0)
    assert sample.is_vacuum
    assert sample.density == 0.0
    assert math.isinf(sample.mean_free_path)
    assert isinstance(VacuumAtmosphere().query(0), type(sample))
