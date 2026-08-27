from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

from .analysis import (
    MissionCampaignRunner, ParameterSweep, SweepVariable,
    MissionMonteCarlo, MonteCarloVariable,
    SobolSensitivity, SobolVariable,
    OptimizationBatch, OptimizationStart,
    mission_case_worker, optimization_case_worker,
    summarize_numeric_metrics,
)
from .montecarlo import NormalDispersion, UniformDispersion
from .hpc import SerialBackend, ProcessBackend
from .mission import load_mission, MissionCompiler
from .result_store import SQLiteResultStore


def _find(entries, analysis_id: str, section: str):
    for entry in entries:
        if str(entry.get("id")) == analysis_id:
            return entry
    raise SystemExit(f"unknown {section} analysis ID {analysis_id!r}")


def _defaults(doc, args):
    analysis = doc.raw.get("analysis") or {}
    execution = analysis.get("execution") or {}
    backend_name = args.backend or str(execution.get("backend", "process"))
    workers = args.workers if args.workers is not None else int(execution.get("workers", 0))
    chunksize = args.chunksize if args.chunksize is not None else int(execution.get("chunksize", 1))
    if backend_name == "serial":
        backend = SerialBackend()
    else:
        backend = ProcessBackend(max_workers=workers, chunksize=chunksize)
    if args.store:
        store_path = Path(args.store)
    elif execution.get("store"):
        store_path = doc.base_directory / str(execution["store"])
    else:
        store_path = doc.base_directory / "reports" / f"{doc.mission_id}_analysis.sqlite"
    return backend, store_path


def _add_exec(parser):
    parser.add_argument("--backend", choices=("serial", "process"))
    parser.add_argument("--workers", type=int)
    parser.add_argument("--chunksize", type=int)
    parser.add_argument("--store")
    parser.add_argument("--campaign-id")
    parser.add_argument("--export")
    parser.add_argument("--quiet", action="store_true")


def _progress(quiet):
    if quiet:
        return None
    def report(done, total, result):
        step = max(1, total // 20)
        if done == 1 or done == total or done % step == 0:
            print(f"[{done}/{total}] {result.status} {result.case_id}", file=sys.stderr, flush=True)
    return report


def _finish(store, campaign_id, execution, export=None, extra=None):
    cases = store.cases(campaign_id)
    payload = {
        "execution": {
            "campaign_id": execution.campaign_id,
            "kind": execution.kind,
            "requested_cases": execution.requested_cases,
            "executed_cases": execution.executed_cases,
            "resumed_cases": execution.resumed_cases,
            "failed_cases": execution.failed_cases,
            "elapsed_seconds": execution.elapsed_seconds,
            "workers": execution.workers,
            "store_path": execution.store_path,
        },
        "store": dict(store.summary(campaign_id)),
        "statistics": summarize_numeric_metrics(cases),
    }
    if extra:
        payload.update(extra)
    if export:
        store.export_json(campaign_id, export)
        payload["export"] = str(Path(export).resolve())
    print(json.dumps(payload, indent=2, sort_keys=True, allow_nan=False))


def main(argv=None) -> int:
    p = argparse.ArgumentParser(prog="uniflight-analysis", description="UniFlight integrated analysis/HPC runner")
    sub = p.add_subparsers(dest="command", required=True)

    ps = sub.add_parser("sweep", help="execute a named declarative parameter sweep")
    ps.add_argument("mission"); ps.add_argument("analysis_id"); _add_exec(ps)
    pm = sub.add_parser("monte-carlo", help="execute the mission monte_carlo declaration")
    pm.add_argument("mission"); pm.add_argument("--cases", type=int); pm.add_argument("--seed", type=int); _add_exec(pm)
    pb = sub.add_parser("sobol", help="execute a named Sobol global-sensitivity study")
    pb.add_argument("mission"); pb.add_argument("analysis_id"); _add_exec(pb)
    po = sub.add_parser("optimize-batch", help="execute a named multistart optimization batch")
    po.add_argument("mission"); po.add_argument("analysis_id"); _add_exec(po)
    pl = sub.add_parser("list", help="list analysis declarations in a mission")
    pl.add_argument("mission")
    pst = sub.add_parser("status", help="show checkpoint/result-store status")
    pst.add_argument("store"); pst.add_argument("campaign_id")
    pex = sub.add_parser("export", help="export one campaign from SQLite to portable JSON")
    pex.add_argument("store"); pex.add_argument("campaign_id"); pex.add_argument("output")
    args = p.parse_args(argv)

    if args.command == "status":
        with SQLiteResultStore(args.store) as store:
            print(json.dumps(store.summary(args.campaign_id), indent=2, sort_keys=True))
        return 0
    if args.command == "export":
        with SQLiteResultStore(args.store) as store:
            path = store.export_json(args.campaign_id, args.output)
            print(str(path))
        return 0

    doc = load_mission(args.mission)
    analysis = doc.raw.get("analysis") or {}
    if args.command == "list":
        payload = {
            "mission_id": doc.mission_id,
            "sweeps": [str(x["id"]) for x in analysis.get("sweeps", [])],
            "sobol": [str(x["id"]) for x in analysis.get("sobol", [])],
            "optimization_batches": [str(x["id"]) for x in analysis.get("optimization_batches", [])],
            "monte_carlo": doc.raw.get("monte_carlo") is not None,
        }
        print(json.dumps(payload, indent=2, sort_keys=True)); return 0

    backend, store_path = _defaults(doc, args)
    store = SQLiteResultStore(store_path)
    try:
        runner = MissionCampaignRunner(args.mission, backend=backend, store=store)
        progress = _progress(args.quiet)
        if args.command == "sweep":
            spec = _find(analysis.get("sweeps", []), args.analysis_id, "sweep")
            sweep = ParameterSweep([
                SweepVariable(str(v["name"]), str(v["pointer"]), tuple(float(x) for x in v["values"]))
                for v in spec["variables"]
            ], mode=str(spec.get("mode", "cartesian")))
            cid = args.campaign_id or f"{doc.mission_id}.sweep.{args.analysis_id}"
            ex = runner.run_cases(sweep.cases(), campaign_id=cid, kind="sweep", progress=progress,
                                  metadata={"analysis_id": args.analysis_id})
            _finish(store, cid, ex, args.export); return 0 if ex.failed_cases == 0 else 2

        if args.command == "monte-carlo":
            mc = doc.raw.get("monte_carlo")
            if not mc:
                raise SystemExit("mission has no monte_carlo declaration")
            variables=[]
            for d in mc.get("dispersions", []):
                disp = NormalDispersion(float(d["mean"]), float(d["std"])) if d["distribution"] == "normal" else UniformDispersion(float(d["low"]), float(d["high"]))
                variables.append(MonteCarloVariable(str(d["name"]), str(d["pointer"]), disp))
            campaign = MissionMonteCarlo(
                variables, cases=int(args.cases or mc.get("cases", 1)),
                seed=int(args.seed if args.seed is not None else mc.get("seed", 0)),
                mission_seed_pointer="/mission/seed" if "seed" in doc.raw.get("mission", {}) else None,
            )
            cid = args.campaign_id or f"{doc.mission_id}.monte_carlo"
            ex = runner.run_cases(campaign.cases(), campaign_id=cid, kind="monte_carlo", progress=progress,
                                  metadata={"seed": campaign.seed})
            _finish(store, cid, ex, args.export); return 0 if ex.failed_cases == 0 else 2

        if args.command == "sobol":
            spec = _find(analysis.get("sobol", []), args.analysis_id, "sobol")
            study = SobolSensitivity([
                SobolVariable(str(v["name"]), str(v["pointer"]), float(v["lower"]), float(v["upper"]))
                for v in spec["variables"]
            ], base_samples=int(spec.get("base_samples", 128)), seed=int(spec.get("seed", 0)))
            cid = args.campaign_id or f"{doc.mission_id}.sobol.{args.analysis_id}"
            ex = runner.run_cases(study.cases(), campaign_id=cid, kind="sobol", progress=progress,
                                  metadata={"analysis_id": args.analysis_id, "metric": spec["metric"], "base_samples": study.base_samples})
            extra = {}
            if ex.failed_cases == 0:
                indices = study.analyze(store.cases(cid), str(spec["metric"]))
                extra["sobol"] = indices.to_json_dict()
            _finish(store, cid, ex, args.export, extra); return 0 if ex.failed_cases == 0 else 2

        if args.command == "optimize-batch":
            spec = _find(analysis.get("optimization_batches", []), args.analysis_id, "optimization batch")
            batch = OptimizationBatch([
                OptimizationStart(str(s["name"]), {str(k): float(v) for k,v in s["values"].items()})
                for s in spec["starts"]
            ])
            cid = args.campaign_id or f"{doc.mission_id}.optbatch.{args.analysis_id}"
            ex = runner.run_cases(batch.cases(doc), campaign_id=cid, kind="optimization_batch",
                                  worker=optimization_case_worker, progress=progress,
                                  metadata={"analysis_id": args.analysis_id})
            _finish(store, cid, ex, args.export); return 0 if ex.failed_cases == 0 else 2
    finally:
        store.close()
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
