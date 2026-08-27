import numpy as np

from uniflight import AdaptiveThrustScaleEstimator, SphericalBody, PlanetaryEnvironment
from uniflight.control import VectorLandingGuidance
from uniflight.validation_f import run_f_landing
from uniflight.validation_g import run_g_landing


def test_terminal_sink_mode_commands_downward_velocity_near_surface():
    body = SphericalBody(mu=3.0e12, radius=8.0e5, name="G-test")
    env = PlanetaryEnvironment(body)
    target = np.array([body.radius - 1.0, 0.0, 0.0])
    guide = VectorLandingGuidance(
        env, target, kp_position=0.012, kd_velocity=0.32,
        max_thrust_acceleration=24.0,
        terminal_sink_rate=0.5, terminal_zone=30.0,
    )
    x = np.array([body.radius + 5.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    cmd = guide.evaluate(x, available_thrust_acceleration=30.0)
    # With sink mode active, the radial velocity error must request inward
    # motion even when the estimated vehicle is momentarily stationary.
    assert cmd.velocity_error_i[0] < -0.49


def test_adaptive_thrust_estimator_recovers_synthetic_scale():
    est = AdaptiveThrustScaleEstimator(
        adaptation_gain=1.0, minimum=0.8, maximum=1.2,
        min_commanded_acceleration=0.1, innovation_limit=0.5,
    )
    g = np.array([-2.0, 0.0, 0.0])
    x0 = np.zeros(6)
    est.observe(0.0, x0, g)
    cmd = np.array([10.0, 0.0, 0.0])
    est.commit_command(cmd)
    true_scale = 1.06
    x1 = np.zeros(6)
    x1[3:] = g + true_scale * cmd
    value = est.observe(1.0, x1, g)
    assert np.isclose(value, true_scale, atol=1e-12)


def test_g_eliminates_known_positive_thrust_hover_failure():
    kwargs = dict(
        seed=17, lateral_y=14.0, lateral_z=6.0, radial_speed=-13.0,
        sensor_bias_y=0.8, thrust_scale=1.03,
        integrator_kind="rk4", rk4_step=0.1,
        record_trajectory=False, record_gnc_records=False,
    )
    baseline = run_f_landing(**kwargs)
    robust = run_g_landing(**kwargs)
    assert baseline.metrics["success"] is False
    assert baseline.metrics["touchdown_time"] == 150.0
    assert robust.metrics["success"] is True
    assert robust.metrics["touchdown_time"] < 150.0
    assert robust.metrics["touchdown_speed"] < 3.0


def test_g_remains_safe_for_negative_thrust_dispersion():
    result = run_g_landing(
        seed=17, lateral_y=14.0, lateral_z=6.0, radial_speed=-13.0,
        sensor_bias_y=0.8, thrust_scale=0.95,
        integrator_kind="rk4", rk4_step=0.1,
        record_trajectory=False, record_gnc_records=False,
    )
    assert result.metrics["success"] is True
    assert result.metrics["touchdown_speed"] < 3.0
    assert result.metrics["landing_error"] < 5.0
    assert result.metrics["final_mass"] > 300.0
