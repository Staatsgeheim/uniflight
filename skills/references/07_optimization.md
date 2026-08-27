# Targeting and trajectory optimization

## Core objects
- `DesignVariable`
- `DesignSpace`
- `MetricObjective`
- `MetricConstraint`
- `TrajectoryProblem`
- `TrajectoryTargeter`
- `TrajectoryOptimizer`
- `FiniteDifferenceConfig`
- `MultipleShootingTranscription`
- `parallel_batch_evaluate`

## Pattern
Create one deterministic evaluator:
```python
def evaluate(design_values):
    # configure/run mission
    return {"apogee": ..., "propellant": ..., "max_q": ...}
```
Then expose metrics to objectives/constraints.

Example:
```python
problem = TrajectoryProblem(
    design_space,
    evaluator=evaluate,
    objective=MetricObjective("propellant", "minimize"),
    constraints=(
        MetricConstraint("apogee", lower=20000.0, upper=20000.0),
        MetricConstraint("max_q", upper=50000.0),
    ),
)
result = TrajectoryOptimizer().solve(problem)
```

## Numerical discipline
- scale variables;
- scale constraints;
- make evaluator deterministic;
- cache identical evaluations;
- use bounds;
- verify final constraints independently;
- compare optimizer result with nearby perturbations;
- use multiple starts for nonconvex problems.

## Multiple shooting
Use when long trajectories make single shooting ill-conditioned. Continuity defects become constraints between segments.

## MDL optimization
MDL variables point into the mission with JSON Pointer. Prefer this for reproducible configuration-level studies. Use Python evaluator when design variables affect custom objects not exposed by MDL.
