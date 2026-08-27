import numpy as np

from uniflight import (
    Event, EventAction, FixedStepRK4Config, FixedStepRK4Integrator,
    MonteCarloRunner, NormalDispersion, PointMassGravity, SimulationEngine,
    automatic_worker_count,
)


def _parallel_case(params, rng):
    return {
        "success": True,
        "metric": float(params["x"] + rng.normal(0.0, 0.01)),
    }


def test_060_fixed_step_rk4_detects_terminal_root():
    integ = FixedStepRK4Integrator(FixedStepRK4Config(step=0.1))
    event = Event(
        "root", lambda t, y: float(y[0]-0.35),
        direction=1.0, action=EventAction.TERMINATE,
    )
    result = SimulationEngine(lambda t, y: np.array([1.0]), integ).run(
        (0.0, 1.0), np.array([0.0]), (event,)
    )
    assert result.success and result.terminated_by == "root"
    assert abs(result.times[-1]-0.35) < 2e-8
    assert abs(result.states[-1, 0]-0.35) < 2e-8


def test_061_point_mass_gravity_analytical_jacobian_matches_finite_difference():
    g = PointMassGravity(3.0e12)
    r = np.array([8.0e5, 2.0e3, -1.0e3])
    J = g.jacobian(r)
    eps = 0.1
    Jfd = np.column_stack([
        (g.acceleration(r + eps*np.eye(3)[i]) - g.acceleration(r - eps*np.eye(3)[i]))/(2*eps)
        for i in range(3)
    ])
    assert np.allclose(J, Jfd, rtol=2e-8, atol=2e-12)


def test_062_parallel_monte_carlo_matches_serial_exactly():
    runner = MonteCarloRunner(_parallel_case, {"x": NormalDispersion(0.0, 1.0)}, 12345)
    serial = runner.run(8, workers=1)
    parallel = runner.run(8, workers=2, chunksize=2)
    assert [r.seed for r in serial.cases] == [r.seed for r in parallel.cases]
    assert [r.parameters for r in serial.cases] == [r.parameters for r in parallel.cases]
    assert np.array_equal(
        np.array([r.metrics["metric"] for r in serial.cases]),
        np.array([r.metrics["metric"] for r in parallel.cases]),
    )
    assert serial.success_rate == parallel.success_rate


def test_063_automatic_worker_count_is_bounded():
    assert automatic_worker_count(1) == 1
    assert 1 <= automatic_worker_count(10) <= 10
