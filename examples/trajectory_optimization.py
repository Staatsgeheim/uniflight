"""Milestone H trajectory-targeting and constrained-optimization example.

A radial launch from fictional airless body Nereid-H is propagated with the
UniFlight dynamics kernel.  The targeter first solves for burn duration at a
fixed mass flow.  The optimizer then minimizes propellant while constraining
apogee to exactly 20 km using mass flow and burn duration as design variables.
"""
from __future__ import annotations
import argparse
import json
from pathlib import Path

from uniflight.validation_h import (
    TARGET_APOGEE_H, build_radial_ascent_optimizer,
    build_radial_ascent_targeter, evaluate_radial_ascent,
)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()

    target = build_radial_ascent_targeter(mdot=5.0).solve()
    target_metrics = evaluate_radial_ascent({"mdot": 5.0, "burn_time": target.design["burn_time"]})

    problem, optimizer = build_radial_ascent_optimizer()
    initial = problem.evaluate_physical(problem.design_space.initial_physical)
    optimized = optimizer.solve(problem)

    report = {
        "target_apogee_m": TARGET_APOGEE_H,
        "single_variable_targeting": {
            "success": target.success,
            "design": dict(target.design),
            "residual_norm": target.residual_norm,
            "metrics": {k: float(v) for k, v in target_metrics.items()},
        },
        "constrained_optimization": {
            "success": optimized.success,
            "method": optimized.method,
            "message": optimized.message,
            "design": dict(optimized.design),
            "objective": optimized.objective,
            "max_constraint_violation": optimized.max_constraint_violation,
            "metrics": {k: float(v) for k, v in optimized.metrics.items()},
            "actual_simulation_evaluations": problem.evaluation_count,
            "initial_metrics": {k: float(v) for k, v in initial.metrics.items()},
        },
    }

    print("UniFlight Milestone H — Nereid-H radial trajectory design")
    print(f"Targeter: burn_time={target.design['burn_time']:.6f} s at mdot=5 kg/s")
    print(f"  apogee={target_metrics['apogee_altitude']:.6f} m")
    print("Optimizer:")
    print(f"  success={optimized.success} method={optimized.method}")
    print(f"  mdot={optimized.design['mdot']:.6f} kg/s")
    print(f"  burn_time={optimized.design['burn_time']:.6f} s")
    print(f"  propellant={optimized.metrics['propellant_used']:.6f} kg")
    print(f"  apogee={optimized.metrics['apogee_altitude']:.6f} m")
    print(f"  constraint violation={optimized.max_constraint_violation:.3e}")
    print(f"  actual simulation evaluations={problem.evaluation_count}")

    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2), encoding="utf-8")
        print(f"Wrote {args.output}")


if __name__ == "__main__":
    main()
