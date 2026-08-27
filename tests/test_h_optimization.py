from __future__ import annotations
import numpy as np

from uniflight.optimization import (
    DesignVariable, DesignSpace, MetricObjective, MetricConstraint,
    TrajectoryProblem, TrajectoryOptimizer, OptimizationSettings,
    TrajectoryTargeter, TargetingSettings, finite_difference_jacobian,
    MultipleShootingTranscription, parallel_batch_evaluate,
)
from uniflight.validation_h import (
    TARGET_APOGEE_H, evaluate_radial_ascent,
    build_radial_ascent_targeter, build_radial_ascent_optimizer,
)


def batch_square(x):
    return float(np.dot(x, x))


def prop_dt1(x0, params):
    # state [position, velocity], dt=1 with constant velocity
    return np.array([x0[0] + x0[1], x0[1]])


def prop_dt2(x0, params):
    dt = float(params.get("dt2", 2.0))
    return np.array([x0[0] + dt*x0[1], x0[1]])


def test_design_space_scaling_roundtrip_and_mapping():
    space = DesignSpace([
        DesignVariable("a", 10.0, 0.0, 20.0, scale=10.0),
        DesignVariable("b", -2.0, -5.0, 5.0, scale=2.0),
    ])
    z = space.initial_scaled
    x = space.to_physical(z)
    assert np.allclose(x, [10.0, -2.0])
    assert space.as_mapping(x) == {"a": 10.0, "b": -2.0}


def test_bound_aware_finite_difference_jacobian():
    def f(x):
        return np.array([x[0]**2 + 3*x[1], np.sin(x[0])])
    x = np.array([0.0, 2.0])
    J = finite_difference_jacobian(f, x, bounds=(np.array([0.0, -5.0]), np.array([10.0, 5.0])))
    expected = np.array([[0.0, 3.0], [1.0, 0.0]])
    assert np.allclose(J, expected, atol=2e-5, rtol=2e-5)


def test_generic_targeter_hits_two_nonlinear_targets():
    space = DesignSpace([
        DesignVariable("x", 1.5, -5, 5, scale=2),
        DesignVariable("y", 0.5, -5, 5, scale=2),
    ])
    def residual(p):
        x, y = p["x"], p["y"]
        return np.array([x+y-3.0, x-y-1.0])
    result = TrajectoryTargeter(space, residual, TargetingSettings(max_nfev=50)).solve()
    assert result.success
    assert result.residual_norm < 1e-8
    assert abs(result.design["x"]-2.0) < 1e-7
    assert abs(result.design["y"]-1.0) < 1e-7


def test_constrained_optimizer_on_analytic_black_box():
    space = DesignSpace([
        DesignVariable("x", 0.5, 0.0, 4.0, scale=2.0),
        DesignVariable("y", 2.5, 0.0, 4.0, scale=2.0),
    ])
    def evaluator(p):
        x, y = p["x"], p["y"]
        return {"cost": (x-1.0)**2 + (y-2.0)**2, "sum": x+y}
    problem = TrajectoryProblem(
        space, evaluator, MetricObjective("cost"),
        (MetricConstraint("sum", lower=3.0, upper=3.0, scale=3.0),),
    )
    out = TrajectoryOptimizer(OptimizationSettings(maxiter=80, fallback_method=None)).solve(problem)
    assert out.success, out.message
    assert out.max_constraint_violation < 1e-7
    assert abs(out.design["x"]-1.0) < 2e-4
    assert abs(out.design["y"]-2.0) < 2e-4


def test_multiple_shooting_defects_are_zero_for_continuous_nodes():
    ms = MultipleShootingTranscription((prop_dt1, prop_dt2), state_size=2)
    nodes = np.array([[0.0, 3.0], [3.0, 3.0], [9.0, 3.0]])
    assert np.allclose(ms.defects(nodes, {"dt2": 2.0}), 0.0)
    packed = ms.flatten_nodes(nodes)
    assert np.allclose(ms.unflatten_nodes(packed), nodes)


def test_parallel_candidate_evaluation_preserves_order():
    candidates = [np.array([i, 2*i], dtype=float) for i in range(6)]
    result = parallel_batch_evaluate(batch_square, candidates, workers=2)
    expected = tuple(float(np.dot(x, x)) for x in candidates)
    assert result.outputs == expected
    assert result.workers == 2


def test_event_based_radial_ascent_targeter_hits_apogee():
    targeter = build_radial_ascent_targeter(mdot=5.0)
    out = targeter.solve()
    assert out.success, out.message
    assert out.residual_norm < 1e-6
    metrics = evaluate_radial_ascent({"mdot": 5.0, "burn_time": out.design["burn_time"]})
    assert abs(metrics["apogee_altitude"]-TARGET_APOGEE_H) < 0.05


def test_simulation_based_ascent_optimizer_meets_target_and_reduces_propellant():
    problem, optimizer = build_radial_ascent_optimizer()
    initial = problem.evaluate_physical(problem.design_space.initial_physical)
    out = optimizer.solve(problem)
    assert out.success, out.message
    assert abs(out.metrics["apogee_altitude"]-TARGET_APOGEE_H) < 0.1
    assert out.metrics["propellant_used"] < initial.metrics["propellant_used"]
    # Minimum-propellant radial burn should favor the upper thrust/mass-flow bound
    # because it minimizes gravity losses.
    assert out.design["mdot"] > 7.5


def test_problem_cache_avoids_duplicate_simulation_evaluations():
    calls = {"n": 0}
    space = DesignSpace([DesignVariable("x", 1.0, 0.0, 2.0)])
    def evaluator(p):
        calls["n"] += 1
        return {"f": p["x"]**2}
    problem = TrajectoryProblem(space, evaluator, MetricObjective("f"))
    problem.evaluate_physical(np.array([1.0]))
    problem.evaluate_physical(np.array([1.0]))
    assert calls["n"] == 1
    assert problem.evaluation_count == 1


def test_lower_bound_inequality_constraint():
    space = DesignSpace([DesignVariable("x", 0.2, 0.0, 4.0)])
    def evaluator(p):
        return {"cost": p["x"]**2, "x": p["x"]}
    problem = TrajectoryProblem(
        space, evaluator, MetricObjective("cost"),
        (MetricConstraint("x", lower=2.0, scale=2.0),),
    )
    out = TrajectoryOptimizer(OptimizationSettings(maxiter=60, fallback_method=None)).solve(problem)
    assert out.success
    assert out.design["x"] >= 2.0 - 1e-6
    assert abs(out.design["x"] - 2.0) < 2e-4
