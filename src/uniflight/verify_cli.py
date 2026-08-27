from __future__ import annotations
import argparse, json
from pathlib import Path
from .verification import ReferenceTimeHistory, TolerancePolicy, VerificationReport, compare_time_histories
from .verification_cases import run_builtin_verification


def _cmd_run(args) -> int:
    report=run_builtin_verification()
    if args.output: report.write_json(args.output)
    if args.markdown: report.write_markdown(args.markdown)
    print(json.dumps(report.to_dict()["summary"],indent=2))
    return 0 if report.success else 1


def _cmd_compare(args) -> int:
    ref=ReferenceTimeHistory.from_csv(args.reference,time_column=args.time_column,channels=args.channels)
    act=ReferenceTimeHistory.from_csv(args.actual,time_column=args.time_column,channels=args.channels)
    results=compare_time_histories(ref,act,channels=args.channels,tolerance=TolerancePolicy(args.abs_tol,args.rel_tol,args.scale_floor))
    report=VerificationReport(results,metadata={"reference":str(args.reference),"actual":str(args.actual)})
    if args.output: report.write_json(args.output)
    print(json.dumps(report.to_dict()["summary"],indent=2))
    return 0 if report.success else 1


def main(argv=None) -> int:
    p=argparse.ArgumentParser(prog="uniflight-verify",description="UniFlight formal verification tools")
    sp=p.add_subparsers(dest="command",required=True)
    r=sp.add_parser("run",help="run built-in formal verification suite")
    r.add_argument("--output",type=Path); r.add_argument("--markdown",type=Path); r.set_defaults(func=_cmd_run)
    c=sp.add_parser("compare-csv",help="compare scalar time-history CSV channels")
    c.add_argument("reference",type=Path); c.add_argument("actual",type=Path); c.add_argument("--time-column",default="time"); c.add_argument("--channels",nargs="+"); c.add_argument("--abs-tol",type=float,default=1e-9); c.add_argument("--rel-tol",type=float,default=1e-9); c.add_argument("--scale-floor",type=float,default=0.0); c.add_argument("--output",type=Path); c.set_defaults(func=_cmd_compare)
    a=p.parse_args(argv); return a.func(a)

if __name__ == "__main__": raise SystemExit(main())
