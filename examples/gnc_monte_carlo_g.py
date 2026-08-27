"""Milestone G: robust terminal-GNC Monte Carlo campaign.

Default campaign mode uses a fixed-step RK4 integrator and multiprocessing.
Use ``--backend scipy --workers 1`` to reproduce the high-accuracy reference
execution path.  Every case remains deterministically seeded independent of
worker count.
"""
from __future__ import annotations

import argparse
from functools import partial
import json
from pathlib import Path
import platform
import sys
import time

import numpy as np

import uniflight
from uniflight import MonteCarloRunner, NormalDispersion, automatic_worker_count
from uniflight.validation_g import g_landing_monte_carlo_case, run_g_landing


def dispersions():
    return {
        "lateral_y": NormalDispersion(10.0, 4.0),
        "lateral_z": NormalDispersion(0.0, 4.0),
        "radial_speed": NormalDispersion(-12.0, 0.8),
        "thrust_scale": NormalDispersion(1.0, 0.015),
        "sensor_bias_y": NormalDispersion(0.0, 0.4),
    }


def _metric_dict(result):
    return dict(result.metrics)


def _summary_to_json(mc, *, cases: int, base_seed: int, sample_period: float,
                     backend: str, rk4_step: float, requested_workers: int,
                     reference_nominal: dict):
    return {
        "metadata": {
            "uniflight_version": getattr(uniflight, "__version__", "unknown"),
            "python": platform.python_version(),
            "numpy": np.__version__,
            "platform": platform.platform(),
            "logical_cpus": __import__("os").cpu_count(),
            "cases": cases,
            "base_seed": base_seed,
            "sample_period_s": sample_period,
            "campaign_backend": backend,
            "rk4_step_s": rk4_step if backend == "rk4" else None,
            "requested_workers": requested_workers,
            "actual_workers": mc.workers,
            "elapsed_seconds": mc.elapsed_seconds,
            "cases_per_second": cases / mc.elapsed_seconds if mc.elapsed_seconds > 0 else None,
            "controller": {
                "terminal_sink_rate_mps": 0.5,
                "terminal_zone_m": 30.0,
                "adaptive_thrust_scale": True,
            },
            "success_criteria": {
                "touchdown_event": True,
                "lateral_error_m_lt": 5.0,
                "touchdown_speed_mps_lt": 3.0,
                "final_mass_kg_gt": 300.0,
            },
        },
        "reference_nominal_scipy": reference_nominal,
        "summary": {
            "success_rate": mc.success_rate,
            "statistics": {
                name: {
                    "mean": s.mean, "std": s.std, "minimum": s.minimum,
                    "maximum": s.maximum, "p05": s.p05,
                    "median": s.median, "p95": s.p95,
                }
                for name, s in mc.statistics.items()
            },
        },
        "case_results": [
            {
                "index": r.index,
                "seed": r.seed,
                "parameters": dict(r.parameters),
                "metrics": dict(r.metrics),
            }
            for r in mc.cases
        ],
    }


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--cases", type=int, default=8,
                   help="Monte Carlo cases (default: 8)")
    p.add_argument("--seed", type=int, default=20260827,
                   help="deterministic campaign base seed")
    p.add_argument("--sample-period", type=float, default=0.5,
                   help="sampled GNC update period in seconds")
    p.add_argument("--backend", choices=("rk4", "scipy"), default="rk4",
                   help="campaign integration backend (default: rk4)")
    p.add_argument("--rk4-step", type=float, default=0.1,
                   help="fixed RK4 step for campaign backend")
    p.add_argument("--workers", type=int, default=0,
                   help="process workers; 0=auto, 1=serial (default: auto)")
    p.add_argument("--chunksize", type=int, default=1,
                   help="multiprocessing map chunksize")
    p.add_argument("--output", type=Path, help="optional JSON report path")
    p.add_argument("--nominal-only", action="store_true",
                   help="run only the adaptive SciPy reference nominal")
    p.add_argument("--skip-reference", action="store_true",
                   help="skip the adaptive nominal before the campaign")
    p.add_argument("--no-progress", action="store_true",
                   help="disable periodic progress output")
    return p.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    if args.cases <= 0:
        raise SystemExit("--cases must be positive")
    if args.sample_period <= 0 or args.rk4_step <= 0:
        raise SystemExit("sample periods/steps must be positive")
    if args.workers < 0:
        raise SystemExit("--workers must be >= 0")

    nominal = {}
    if not args.skip_reference or args.nominal_only:
        print("Adaptive reference nominal (SciPy DOP853)")
        t0 = time.perf_counter()
        nominal_result = run_g_landing(
            seed=7, lateral_y=10.0, sample_period=args.sample_period,
            integrator_kind="scipy", record_trajectory=False, record_gnc_records=True,
        )
        nominal_wall = time.perf_counter()-t0
        nominal = _metric_dict(nominal_result)
        for k, v in nominal.items():
            print(f"  {k:>18s}: {v}")
        print(f"  {'GNC updates':>18s}: {len(nominal_result.result.gnc_records)}")
        print(f"  {'wall time [s]':>18s}: {nominal_wall:.3f}")
        if args.nominal_only:
            return 0
    elif args.nominal_only:
        raise SystemExit("--nominal-only cannot be combined with --skip-reference")

    workers = args.workers
    resolved = automatic_worker_count(args.cases) if workers == 0 else min(workers, args.cases)
    case = partial(
        g_landing_monte_carlo_case,
        sample_period=args.sample_period,
        integrator_kind=args.backend,
        rk4_step=args.rk4_step,
    )
    runner = MonteCarloRunner(case, dispersions(), base_seed=args.seed)

    last_print = [0.0]
    def progress(done: int, total: int):
        if args.no_progress:
            return
        now = time.perf_counter()
        if done == total or done == 1 or now-last_print[0] >= 2.0:
            print(f"  progress: {done}/{total} ({100.0*done/total:.1f}%)", flush=True)
            last_print[0] = now

    print(
        f"\nDeterministic {args.cases}-case campaign: backend={args.backend}, "
        f"workers={resolved}, sample={args.sample_period:g}s"
        + (f", rk4_step={args.rk4_step:g}s" if args.backend == "rk4" else "")
    )
    mc = runner.run(
        args.cases, workers=workers, chunksize=args.chunksize,
        progress=progress,
    )
    rate = args.cases/mc.elapsed_seconds if mc.elapsed_seconds > 0 else float("inf")
    print(f"  success rate: {mc.success_rate:.1%}")
    print(f"  elapsed:      {mc.elapsed_seconds:.3f} s")
    print(f"  throughput:   {rate:.3f} cases/s")
    for name in ("landing_error", "touchdown_speed", "final_mass", "touchdown_time", "estimated_thrust_scale"):
        if name in mc.statistics:
            s = mc.statistics[name]
            print(f"  {name:>18s}: mean={s.mean:.3f}, p05={s.p05:.3f}, p95={s.p95:.3f}")

    if args.output:
        report = _summary_to_json(
            mc, cases=args.cases, base_seed=args.seed,
            sample_period=args.sample_period, backend=args.backend,
            rk4_step=args.rk4_step, requested_workers=args.workers,
            reference_nominal=nominal,
        )
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2), encoding="utf-8")
        print(f"\nJSON report: {args.output}")
    return 0


if __name__ == "__main__":
    # Required for multiprocessing spawn semantics on Windows/macOS.
    sys.exit(main())
