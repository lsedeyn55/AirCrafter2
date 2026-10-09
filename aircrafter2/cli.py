"""Command-line interface.

    aircrafter2 run configs/step7_full.yaml [--out outputs/run1] [--analyse] [--no-figures]
                [--design-space] [--prop 11x7E]
    aircrafter2 plugins          # list scoring plug-ins
    aircrafter2 ui               # start the Streamlit app
"""

import argparse
import sys
from pathlib import Path


def main(argv=None):
    ap = argparse.ArgumentParser(prog="aircrafter2", description="AirCrafter 2.0: DBF MDAO framework")
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run", help="optimise (or analyse) a design from a YAML config")
    r.add_argument("config")
    r.add_argument("--out", default=None, help="output folder (default outputs/<config name>)")
    r.add_argument("--analyse", action="store_true", help="run the coupled analysis only, no optimisation")
    r.add_argument("--no-figures", action="store_true")
    r.add_argument("--design-space", action="store_true", help="add a 7x7 b-c design-space sweep (slow)")
    r.add_argument("--prop", default=None, help="override the APC prop (e.g. 10x7E)")
    sub.add_parser("plugins", help="list the registered scoring plug-ins")
    sub.add_parser("ui", help="start the Streamlit UI")
    args = ap.parse_args(argv)

    if args.cmd == "plugins":
        from .scoring import available

        print("\n".join(available()))
        return 0
    if args.cmd == "ui":
        import subprocess

        app = Path(__file__).parent / "ui" / "app.py"
        return subprocess.call([sys.executable, "-m", "streamlit", "run", str(app)])

    from .api import run_design
    from .problems.full_mdo import load_full_config

    cfg = load_full_config(args.config)
    if args.prop:
        cfg.propulsion.prop = args.prop
    out = args.out or str(Path("outputs") / Path(args.config).stem)
    res = run_design(cfg, optimise=not args.analyse, out_dir=out, figures=not args.no_figures,
                     design_space=args.design_space, progress=lambda s: print(f"[aircrafter2] {s} ...", flush=True))
    print(f"success = {res.success}, iterations = {res.iterations}, score = {res.values['score']:.4f}")
    print(f"outputs in {out}/ (summary.md, results.json, n2.html, figures/)")
    return 0 if res.success else 1


if __name__ == "__main__":
    sys.exit(main())
