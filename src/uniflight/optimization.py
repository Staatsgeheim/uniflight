"""Milestone H: generic trajectory targeting and optimization.

The optimization layer deliberately treats the flight simulation as a black-box
mapping from a bounded design vector to named scalar/vector metrics.  It does
not own or mutate the physics kernel.  This separation allows the same design
machinery to wrap ascent, orbit, entry, EDL, GNC, or user-supplied models.
"""
from __future__ import annotations

from concurrent.futures import ProcessPoolExecutor
from collections import OrderedDict
from dataclasses import dataclass, field
from typing import Callable, Mapping, Protocol, Sequence
import math
import os
import numpy as np
from scipy.optimize import least_squares, minimize

ArrayLike = np.ndarray | Sequence[float]
MetricValue = float | np.ndarray
Metrics = Mapping[str, MetricValue]


class TrajectoryEvaluator(Protocol):
    def __call__(self, parameters: Mapping[str, float]) -> Metrics: ...


@dataclass(frozen=True, slots=True)
class DesignVariable:
    """One bounded scalar design variable.

    ``scale`` is used only to nondimensionalize the optimizer coordinate; the
    physical value returned to the simulation is always in the variable's
    native SI-compatible units.
    """

    name: str
    initial: float
    lower: float = -np.inf
    upper: float = np.inf
    scale: float = 1.0

    def __post_init__(self) -> None:
        vals = (self.initial, self.lower, self.upper, self.scale)
        if not all(np.isfinite(v) for v in (self.initial, self.scale)):
            raise ValueError(f"Design variable {self.name!r} has non-finite initial/scale")
        if self.scale <= 0:
            raise ValueError("Design-variable scale must be positive")
        if self.lower > self.upper:
            raise ValueError("Design-variable lower bound exceeds upper bound")
        if not self.lower <= self.initial <= self.upper:
            raise ValueError(f"Initial value for {self.name!r} lies outside bounds")


class DesignSpace:
    def __init__(self, variables: Sequence[DesignVariable]):
        self.variables = tuple(variables)
        if not self.variables:
            raise ValueError("DesignSpace requires at least one variable")
        names = [v.name for v in self.variables]
        if len(names) != len(set(names)):
            raise ValueError("Design-variable names must be unique")

    @property
    def size(self) -> int:
        return len(self.variables)

    @property
    def names(self) -> tuple[str, ...]:
        return tuple(v.name for v in self.variables)

    @property
    def initial_physical(self) -> np.ndarray:
        return np.asarray([v.initial for v in self.variables], dtype=float)

    @property
    def initial_scaled(self) -> np.ndarray:
        return np.asarray([v.initial / v.scale for v in self.variables], dtype=float)

    @property
    def bounds_physical(self) -> tuple[np.ndarray, np.ndarray]:
        lo = np.asarray([v.lower for v in self.variables], dtype=float)
        hi = np.asarray([v.upper for v in self.variables], dtype=float)
        return lo, hi

    @property
    def bounds_scaled(self) -> tuple[np.ndarray, np.ndarray]:
        lo = np.asarray([v.lower / v.scale for v in self.variables], dtype=float)
        hi = np.asarray([v.upper / v.scale for v in self.variables], dtype=float)
        return lo, hi

    def to_scaled(self, physical: ArrayLike) -> np.ndarray:
        x = np.asarray(physical, dtype=float)
        if x.shape != (self.size,):
            raise ValueError("Design vector has wrong shape")
        return x / np.asarray([v.scale for v in self.variables])

    def to_physical(self, scaled: ArrayLike) -> np.ndarray:
        z = np.asarray(scaled, dtype=float)
        if z.shape != (self.size,):
            raise ValueError("Design vector has wrong shape")
        return z * np.asarray([v.scale for v in self.variables])

    def as_mapping(self, physical: ArrayLike) -> dict[str, float]:
        x = np.asarray(physical, dtype=float)
        if x.shape != (self.size,):
            raise ValueError("Design vector has wrong shape")
        return {v.name: float(value) for v, value in zip(self.variables, x, strict=True)}


@dataclass(frozen=True, slots=True)
class MetricObjective:
    """Scalar objective constructed from a named evaluator metric."""

    metric: str
    sense: str = "minimize"  # or maximize
    weight: float = 1.0
    reference: float = 0.0
    scale: float = 1.0

    def __post_init__(self) -> None:
        if self.sense not in {"minimize", "maximize"}:
            raise ValueError("sense must be 'minimize' or 'maximize'")
        if self.scale <= 0 or not np.isfinite(self.scale):
            raise ValueError("objective scale must be finite and positive")

    def value(self, metrics: Metrics) -> float:
        raw = np.asarray(metrics[self.metric], dtype=float)
        if raw.size != 1:
            raise ValueError(f"Objective metric {self.metric!r} must be scalar")
        val = self.weight * (float(raw.reshape(-1)[0]) - self.reference) / self.scale
        return val if self.sense == "minimize" else -val


@dataclass(frozen=True, slots=True)
class MetricConstraint:
    """Bounded nonlinear constraint on a named scalar/vector metric.

    The condition is ``lower <= metric/scale <= upper``. Bounds may be scalar
    or vector and are broadcast to the metric shape.
    """

    metric: str
    lower: float | np.ndarray = -np.inf
    upper: float | np.ndarray = np.inf
    scale: float | np.ndarray = 1.0
    name: str | None = None

    def components(self, metrics: Metrics) -> np.ndarray:
        val = np.atleast_1d(np.asarray(metrics[self.metric], dtype=float))
        scale = np.broadcast_to(np.asarray(self.scale, dtype=float), val.shape)
        if np.any(~np.isfinite(scale)) or np.any(scale <= 0):
            raise ValueError("constraint scale must be finite and positive")
        return val / scale

    def bounds(self, metrics: Metrics) -> tuple[np.ndarray, np.ndarray]:
        val = self.components(metrics)
        scale = np.broadcast_to(np.asarray(self.scale, dtype=float), val.shape)
        lo = np.broadcast_to(np.asarray(self.lower, dtype=float), val.shape) / scale
        hi = np.broadcast_to(np.asarray(self.upper, dtype=float), val.shape) / scale
        if np.any(lo > hi):
            raise ValueError("constraint lower bound exceeds upper bound")
        return lo, hi


@dataclass(frozen=True, slots=True)
class ProblemEvaluation:
    physical_design: np.ndarray
    parameters: Mapping[str, float]
    metrics: Metrics
    objective: float


class TrajectoryProblem:
    """Black-box simulation optimization problem.

    The evaluator must be deterministic for a fixed parameter mapping whenever
    gradient-based optimization is requested.  Stochastic campaigns should be
    wrapped in an outer statistical objective explicitly rather than silently
    sampling inside a single evaluation.
    """

    def __init__(self, design_space: DesignSpace, evaluator: TrajectoryEvaluator,
                 objective: MetricObjective | Callable[[Metrics], float],
                 constraints: Sequence[MetricConstraint] = (), *, cache_size: int = 128):
        self.design_space = design_space
        self.evaluator = evaluator
        self.objective_spec = objective
        self.constraints = tuple(constraints)
        if cache_size < 0:
            raise ValueError("cache_size must be nonnegative")
        self.cache_size = int(cache_size)
        self._cache: OrderedDict[bytes, ProblemEvaluation] = OrderedDict()
        self.evaluation_count = 0

    def clear_cache(self) -> None:
        self._cache.clear()

    def evaluate_physical(self, physical: ArrayLike) -> ProblemEvaluation:
        x = np.asarray(physical, dtype=float)
        lo, hi = self.design_space.bounds_physical
        if x.shape != (self.design_space.size,):
            raise ValueError("Design vector has wrong shape")
        if np.any(x < lo) or np.any(x > hi):
            raise ValueError("Design vector outside declared bounds")
        key = np.ascontiguousarray(x, dtype=np.float64).tobytes()
        if self.cache_size and key in self._cache:
            ev = self._cache.pop(key)
            self._cache[key] = ev
            return ev
        params = self.design_space.as_mapping(x)
        metrics = self.evaluator(params)
        self.evaluation_count += 1
        if isinstance(self.objective_spec, MetricObjective):
            obj = self.objective_spec.value(metrics)
        else:
            obj = float(self.objective_spec(metrics))
        if not np.isfinite(obj):
            raise FloatingPointError("Objective returned a non-finite value")
        ev = ProblemEvaluation(x.copy(), params, metrics, obj)
        if self.cache_size:
            self._cache[key] = ev
            while len(self._cache) > self.cache_size:
                self._cache.popitem(last=False)
        return ev

    def evaluate_scaled(self, scaled: ArrayLike) -> ProblemEvaluation:
        return self.evaluate_physical(self.design_space.to_physical(scaled))

    def constraint_values(self, scaled: ArrayLike) -> list[tuple[MetricConstraint, np.ndarray, np.ndarray, np.ndarray]]:
        ev = self.evaluate_scaled(scaled)
        out = []
        for c in self.constraints:
            values = c.components(ev.metrics)
            lo, hi = c.bounds(ev.metrics)
            out.append((c, values, lo, hi))
        return out


@dataclass(frozen=True, slots=True)
class FiniteDifferenceConfig:
    relative_step: float = 1e-5
    absolute_step: float = 1e-8

    def __post_init__(self) -> None:
        if self.relative_step <= 0 or self.absolute_step <= 0:
            raise ValueError("finite-difference steps must be positive")


def finite_difference_jacobian(fun: Callable[[np.ndarray], ArrayLike], x: ArrayLike,
                               *, bounds: tuple[ArrayLike, ArrayLike] | None = None,
                               config: FiniteDifferenceConfig | None = None) -> np.ndarray:
    """Bound-aware central finite-difference Jacobian.

    Near a bound, a one-sided second-order stencil is used when possible; if a
    second step would leave the interval, a first-order one-sided difference is
    used. The routine accepts scalar or vector outputs.
    """
    cfg = config or FiniteDifferenceConfig()
    x = np.asarray(x, dtype=float)
    f0 = np.atleast_1d(np.asarray(fun(x), dtype=float))
    m, n = f0.size, x.size
    jac = np.empty((m, n), dtype=float)
    if bounds is None:
        lo = np.full(n, -np.inf); hi = np.full(n, np.inf)
    else:
        lo = np.broadcast_to(np.asarray(bounds[0], dtype=float), x.shape)
        hi = np.broadcast_to(np.asarray(bounds[1], dtype=float), x.shape)

    for j in range(n):
        h = max(cfg.absolute_step, cfg.relative_step * max(1.0, abs(x[j])))
        can_plus = x[j] + h <= hi[j]
        can_minus = x[j] - h >= lo[j]
        if can_plus and can_minus:
            xp = x.copy(); xm = x.copy()
            xp[j] += h; xm[j] -= h
            jac[:, j] = (np.atleast_1d(fun(xp)) - np.atleast_1d(fun(xm))) / (2*h)
        elif can_plus:
            xp = x.copy(); xp[j] += h
            if x[j] + 2*h <= hi[j]:
                xpp = x.copy(); xpp[j] += 2*h
                jac[:, j] = (-3*f0 + 4*np.atleast_1d(fun(xp)) - np.atleast_1d(fun(xpp))) / (2*h)
            else:
                jac[:, j] = (np.atleast_1d(fun(xp)) - f0) / h
        elif can_minus:
            xm = x.copy(); xm[j] -= h
            if x[j] - 2*h >= lo[j]:
                xmm = x.copy(); xmm[j] -= 2*h
                jac[:, j] = (3*f0 - 4*np.atleast_1d(fun(xm)) + np.atleast_1d(fun(xmm))) / (2*h)
            else:
                jac[:, j] = (f0 - np.atleast_1d(fun(xm))) / h
        else:
            raise ValueError(f"No finite-difference step available for coordinate {j}")
    return jac


@dataclass(frozen=True, slots=True)
class TargetingSettings:
    xtol: float = 1e-10
    ftol: float = 1e-10
    gtol: float = 1e-10
    max_nfev: int = 200
    fd: FiniteDifferenceConfig = field(default_factory=FiniteDifferenceConfig)


@dataclass(frozen=True, slots=True)
class TargetingResult:
    success: bool
    message: str
    design: Mapping[str, float]
    residual: np.ndarray
    residual_norm: float
    nfev: int
    raw_result: object


class TrajectoryTargeter:
    """Bounded nonlinear least-squares targeter.

    ``residual`` receives the physical design mapping and must return a vector
    whose zero corresponds to the desired terminal/event targets.
    """

    def __init__(self, design_space: DesignSpace,
                 residual: Callable[[Mapping[str, float]], ArrayLike],
                 settings: TargetingSettings | None = None):
        self.design_space = design_space
        self.residual = residual
        self.settings = settings or TargetingSettings()

    def solve(self) -> TargetingResult:
        lo, hi = self.design_space.bounds_scaled

        def fun(z: np.ndarray) -> np.ndarray:
            p = self.design_space.as_mapping(self.design_space.to_physical(z))
            r = np.atleast_1d(np.asarray(self.residual(p), dtype=float))
            if not np.all(np.isfinite(r)):
                raise FloatingPointError("Target residual is non-finite")
            return r

        def jac(z: np.ndarray) -> np.ndarray:
            return finite_difference_jacobian(fun, z, bounds=(lo, hi), config=self.settings.fd)

        res = least_squares(
            fun, self.design_space.initial_scaled, jac=jac, bounds=(lo, hi),
            xtol=self.settings.xtol, ftol=self.settings.ftol, gtol=self.settings.gtol,
            max_nfev=self.settings.max_nfev,
        )
        physical = self.design_space.to_physical(res.x)
        design = self.design_space.as_mapping(physical)
        residual = np.asarray(res.fun, dtype=float)
        return TargetingResult(bool(res.success), str(res.message), design, residual,
                               float(np.linalg.norm(residual)), int(res.nfev), res)


@dataclass(frozen=True, slots=True)
class OptimizationSettings:
    method: str = "SLSQP"
    maxiter: int = 200
    ftol: float = 1e-9
    constraint_tolerance: float = 1e-7
    use_finite_difference_jacobian: bool = True
    fallback_method: str | None = "COBYLA"
    fd: FiniteDifferenceConfig = field(default_factory=FiniteDifferenceConfig)


@dataclass(frozen=True, slots=True)
class OptimizationResult:
    success: bool
    message: str
    design: Mapping[str, float]
    objective: float
    metrics: Metrics
    max_constraint_violation: float
    nfev: int
    nit: int | None
    method: str
    raw_result: object


class TrajectoryOptimizer:
    def __init__(self, settings: OptimizationSettings | None = None):
        self.settings = settings or OptimizationSettings()

    @staticmethod
    def _violation(c: MetricConstraint, metrics: Metrics) -> float:
        values = c.components(metrics); lo, hi = c.bounds(metrics)
        low_v = np.maximum(lo-values, 0.0)
        high_v = np.maximum(values-hi, 0.0)
        return float(np.max(np.maximum(low_v, high_v))) if values.size else 0.0

    def _scipy_constraints(self, problem: TrajectoryProblem):
        constraints: list[dict[str, object]] = []
        tol = self.settings.constraint_tolerance

        # Each bound becomes a SciPy inequality g(z)>=0. Exact equality bounds
        # become an equality h(z)=0 for SLSQP. COBYLA fallback uses a tolerance
        # band around equalities below.
        for index, spec in enumerate(problem.constraints):
            # Probe shape once at initial point.
            probe = problem.evaluate_scaled(problem.design_space.initial_scaled)
            vals = spec.components(probe.metrics); lo, hi = spec.bounds(probe.metrics)
            for k in range(vals.size):
                lk, hk = float(lo[k]), float(hi[k])
                if np.isfinite(lk) and np.isfinite(hk) and abs(hk-lk) <= tol:
                    target = 0.5*(lk+hk)
                    def eq_fun(z, s=spec, kk=k, target=target):
                        ev = problem.evaluate_scaled(z)
                        return float(s.components(ev.metrics)[kk] - target)
                    constraints.append({"type": "eq", "fun": eq_fun})
                else:
                    if np.isfinite(lk):
                        def lo_fun(z, s=spec, kk=k, lower=lk):
                            ev = problem.evaluate_scaled(z)
                            return float(s.components(ev.metrics)[kk] - lower)
                        constraints.append({"type": "ineq", "fun": lo_fun})
                    if np.isfinite(hk):
                        def hi_fun(z, s=spec, kk=k, upper=hk):
                            ev = problem.evaluate_scaled(z)
                            return float(upper - s.components(ev.metrics)[kk])
                        constraints.append({"type": "ineq", "fun": hi_fun})
        return constraints

    def _fallback_constraints(self, problem: TrajectoryProblem):
        constraints: list[dict[str, object]] = []
        tol = self.settings.constraint_tolerance
        probe = problem.evaluate_scaled(problem.design_space.initial_scaled)
        for spec in problem.constraints:
            vals = spec.components(probe.metrics); lo, hi = spec.bounds(probe.metrics)
            for k in range(vals.size):
                lk, hk = float(lo[k]), float(hi[k])
                if np.isfinite(lk):
                    def lf(z, s=spec, kk=k, lower=lk):
                        ev = problem.evaluate_scaled(z)
                        return float(s.components(ev.metrics)[kk] - lower + tol)
                    constraints.append({"type": "ineq", "fun": lf})
                if np.isfinite(hk):
                    def hf(z, s=spec, kk=k, upper=hk):
                        ev = problem.evaluate_scaled(z)
                        return float(upper - s.components(ev.metrics)[kk] + tol)
                    constraints.append({"type": "ineq", "fun": hf})
        return constraints

    def solve(self, problem: TrajectoryProblem) -> OptimizationResult:
        space = problem.design_space
        lo, hi = space.bounds_scaled
        bounds = list(zip(lo, hi, strict=True))

        def obj(z: np.ndarray) -> float:
            return problem.evaluate_scaled(z).objective

        jac = None
        if self.settings.use_finite_difference_jacobian:
            def jac(z: np.ndarray) -> np.ndarray:
                return finite_difference_jacobian(lambda zz: np.array([obj(zz)]), z,
                                                  bounds=(lo, hi), config=self.settings.fd)[0]

        primary_constraints = self._scipy_constraints(problem)
        res = minimize(
            obj, space.initial_scaled, method=self.settings.method, jac=jac,
            bounds=bounds, constraints=primary_constraints,
            options={"maxiter": self.settings.maxiter, "ftol": self.settings.ftol, "disp": False},
        )
        used_method = self.settings.method

        if (not res.success) and self.settings.fallback_method:
            fallback = self.settings.fallback_method
            # COBYLA does not accept equality constraints and older SciPy
            # versions do not use Bounds consistently; encode both variable
            # bounds and nonlinear constraints as inequalities.
            cons = self._fallback_constraints(problem)
            for j in range(space.size):
                if np.isfinite(lo[j]):
                    cons.append({"type": "ineq", "fun": lambda z, jj=j, l=lo[j]: float(z[jj]-l)})
                if np.isfinite(hi[j]):
                    cons.append({"type": "ineq", "fun": lambda z, jj=j, h=hi[j]: float(h-z[jj])})
            res2 = minimize(
                obj, np.asarray(res.x, dtype=float), method=fallback,
                constraints=cons,
                options={"maxiter": self.settings.maxiter, "tol": self.settings.ftol, "disp": False},
            )
            if res2.success or res2.fun < res.fun:
                res = res2
                used_method = fallback

        z = np.clip(np.asarray(res.x, dtype=float), lo, hi)
        final = problem.evaluate_scaled(z)
        violation = 0.0
        for c in problem.constraints:
            violation = max(violation, self._violation(c, final.metrics))
        success = bool(res.success and violation <= max(self.settings.constraint_tolerance, 10*self.settings.ftol))
        return OptimizationResult(
            success, str(res.message), final.parameters, float(final.objective), final.metrics,
            violation, int(getattr(res, "nfev", -1)),
            int(getattr(res, "nit", -1)) if getattr(res, "nit", None) is not None else None,
            used_method, res,
        )


@dataclass(frozen=True, slots=True)
class MultipleShootingTranscription:
    """Continuity-defect generator for a multiple-shooting trajectory.

    ``propagators[i](x_i, parameters)`` must return the end state of segment i.
    The supplied node matrix has shape ``(n_segments+1, state_size)``.  Defects
    are flattened as ``propagate(x_i)-x_{i+1}``.
    """

    propagators: tuple[Callable[[np.ndarray, Mapping[str, float]], np.ndarray], ...]
    state_size: int

    def __post_init__(self) -> None:
        if not self.propagators or self.state_size <= 0:
            raise ValueError("multiple shooting requires segments and positive state size")

    @property
    def segment_count(self) -> int:
        return len(self.propagators)

    def defects(self, nodes: np.ndarray, parameters: Mapping[str, float]) -> np.ndarray:
        n = np.asarray(nodes, dtype=float)
        expected = (self.segment_count + 1, self.state_size)
        if n.shape != expected:
            raise ValueError(f"nodes must have shape {expected}")
        defects = []
        for i, prop in enumerate(self.propagators):
            end = np.asarray(prop(n[i].copy(), parameters), dtype=float)
            if end.shape != (self.state_size,):
                raise ValueError("segment propagator returned wrong state shape")
            defects.append(end - n[i+1])
        return np.concatenate(defects)

    def flatten_nodes(self, nodes: np.ndarray) -> np.ndarray:
        n = np.asarray(nodes, dtype=float)
        if n.shape != (self.segment_count + 1, self.state_size):
            raise ValueError("invalid node matrix")
        return n.reshape(-1).copy()

    def unflatten_nodes(self, packed: ArrayLike) -> np.ndarray:
        p = np.asarray(packed, dtype=float)
        expected = (self.segment_count+1)*self.state_size
        if p.shape != (expected,):
            raise ValueError("invalid packed node vector")
        return p.reshape(self.segment_count+1, self.state_size).copy()


@dataclass(frozen=True, slots=True)
class BatchEvaluationResult:
    inputs: tuple[np.ndarray, ...]
    outputs: tuple[object, ...]
    workers: int


def _batch_call(payload):
    fn, vector = payload
    return fn(np.asarray(vector, dtype=float))


def parallel_batch_evaluate(function: Callable[[np.ndarray], object], candidates: Sequence[ArrayLike],
                            *, workers: int = 0, chunksize: int = 1) -> BatchEvaluationResult:
    """Evaluate independent optimization candidates in separate processes.

    The callable must be pickleable under spawn-based platforms.  ``workers=0``
    uses logical CPU count minus one, matching the Monte Carlo convention.
    Output order always matches input order.
    """
    xs = tuple(np.asarray(x, dtype=float).copy() for x in candidates)
    if not xs:
        return BatchEvaluationResult((), (), 0)
    if chunksize <= 0:
        raise ValueError("chunksize must be positive")
    actual = (max(1, (os.cpu_count() or 1)-1) if workers == 0 else int(workers))
    if actual <= 0:
        raise ValueError("workers must be >=0")
    if actual == 1 or len(xs) == 1:
        ys = tuple(function(x) for x in xs)
        return BatchEvaluationResult(xs, ys, 1)
    with ProcessPoolExecutor(max_workers=actual) as pool:
        ys = tuple(pool.map(_batch_call, ((function, x) for x in xs), chunksize=chunksize))
    return BatchEvaluationResult(xs, ys, actual)
