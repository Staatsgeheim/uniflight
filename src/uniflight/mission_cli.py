from __future__ import annotations
import argparse
import json
from pathlib import Path
import sys

from .mission import load_mission, MissionCompiler, mission_json_schema, save_report
from .plugins import PluginManager, installed_plugin_summary


def _summary(compiled):
    return {
        "mission_id": compiled.document.mission_id,
        "mission_sha256": compiled.document.digest_sha256,
        "t_span": list(compiled.t_span),
        "vehicles": [v.vehicle_id for v in compiled.vehicles],
        "bodies": list(compiled.bodies),
        "environments": list(compiled.environments),
        "datasets": [list(x) for x in compiled.data_catalog.inventory()],
        "plugins": [list(x) for x in compiled.plugin_inventory],
        "models": list(compiled.models),
        "optimization": compiled.optimization is not None,
        "dispersions": len(compiled.dispersions),
    }


def main(argv=None) -> int:
    parser=argparse.ArgumentParser(prog="uniflight-mission",description="UniFlight declarative mission runner")
    sub=parser.add_subparsers(dest="command",required=True)
    pval=sub.add_parser("validate",help="parse, validate, resolve references, and compile a mission")
    pval.add_argument("mission")
    pins=sub.add_parser("inspect",help="print a compiled mission summary")
    pins.add_argument("mission")
    prun=sub.add_parser("run",help="execute a declarative mission")
    prun.add_argument("mission"); prun.add_argument("--output")
    popt=sub.add_parser("optimize",help="execute the mission's H optimization declaration")
    popt.add_argument("mission"); popt.add_argument("--output")
    pmc=sub.add_parser("sample",help="sample declared Monte Carlo dispersions without running trajectories")
    pmc.add_argument("mission"); pmc.add_argument("--cases",type=int); pmc.add_argument("--seed",type=int); pmc.add_argument("--output")
    psch=sub.add_parser("schema",help="emit the Mission Definition Language 1.0 editor schema")
    psch.add_argument("--output")
    ppl=sub.add_parser("plugins",help="list installed UniFlight plugin entry points without importing them")
    pcap=sub.add_parser("capabilities",help="compile a mission and list core/plugin capability ownership")
    pcap.add_argument("mission")
    args=parser.parse_args(argv)

    if args.command=="plugins":
        print(json.dumps([dict(x) for x in installed_plugin_summary()],indent=2,sort_keys=True)); return 0

    if args.command=="schema":
        text=json.dumps(mission_json_schema(),indent=2,sort_keys=True)
        if args.output: Path(args.output).write_text(text+"\n",encoding="utf-8")
        else: print(text)
        return 0

    doc=load_mission(args.mission); compiler=MissionCompiler(); compiled=compiler.compile(doc)
    if args.command=="capabilities":
        payload={"plugins":[list(x) for x in compiled.plugin_inventory],
                 "capabilities":[dict(x) for x in compiler.registry.inventory()]}
        print(json.dumps(payload,indent=2,sort_keys=True)); return 0
    if args.command=="validate":
        print(f"VALID {doc.mission_id} {doc.digest_sha256}")
        return 0
    if args.command=="inspect":
        print(json.dumps(_summary(compiled),indent=2,sort_keys=True)); return 0
    if args.command=="run":
        report=compiled.run()
        if args.output: save_report(report,args.output)
        print(json.dumps(report.to_json_dict(),indent=2,sort_keys=True,allow_nan=False))
        return 0 if report.success else 2
    if args.command=="optimize":
        result=compiler.optimize(doc)
        payload={"success":bool(result.success),"message":result.message,"objective":result.objective,
                 "design":dict(result.design),"metrics":dict(result.metrics),
                 "max_constraint_violation":result.max_constraint_violation,
                 "evaluations":result.nfev,"iterations":result.nit,"method":result.method}
        text=json.dumps(payload,indent=2,sort_keys=True)
        if args.output: Path(args.output).write_text(text+"\n",encoding="utf-8")
        print(text); return 0 if result.success else 3
    if args.command=="sample":
        samples=compiler.sample_monte_carlo(doc,args.cases,args.seed)
        payload=[{"index":s["index"],"values":dict(s["values"]),"overrides":dict(s["overrides"])} for s in samples]
        text=json.dumps(payload,indent=2,sort_keys=True)
        if args.output: Path(args.output).write_text(text+"\n",encoding="utf-8")
        else: print(text)
        return 0
    return 1

if __name__ == "__main__":
    raise SystemExit(main())
