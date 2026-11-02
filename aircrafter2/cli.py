"""Command-line interface.

aircrafter2 run configs/full_mdo.yaml [--out outputs/run1] [--analyse] [--no-figures]
            [--design-space] [--prop 11x7E] [--n-motors {1,2}] [--battery-topology {per_motor,shared}]
            [--robust | --no-robust]   # robust design optimisation (configs/full_mdo_rdo.yaml has it on)
                             # the motor Kv is a design variable in configs/full_mdo*.yaml; *_fixed_kv.yaml: fixed
aircrafter2 vsp outputs/<run>/ [--out DIR] [--no-figures] [--incidence {zero,cruise}]
                             # OpenVSP mock-up from the run's geometry.json (optional OpenVSP)
aircrafter2 plugins          # list scoring plug-ins
aircrafter2 ui               # start the Streamlit app
"""

import argparse
import sys
from pathlib import Path


def main(argv=None):
    """The aircrafter2 command.

    Parameters
    ----------
    argv : list of str, optional
        Default: sys.argv.

    Returns
    -------
    int
        Exit code.
    """
    # --- arguments ---
    ap = argparse.ArgumentParser(prog="aircrafter2", description="AirCrafter 2.0: DBF MDAO framework")
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run", help="optimise (or analyse) a design from a YAML config")
    r.add_argument("config")
    r.add_argument("--out", default=None, help="output folder (default outputs/<config name>)")
    r.add_argument("--analyse", action="store_true", help="run the coupled analysis only, no optimisation")
    r.add_argument("--no-figures", action="store_true")
    r.add_argument("--design-space", action="store_true", help="add a 7x7 b-c design-space sweep (slow)")
    r.add_argument("--prop", default=None, help="override the APC prop (e.g. 10x7E)")
    r.add_argument(
        "--n-motors",
        type=int,
        choices=[1, 2],
        default=None,
        help="1 = nose tractor, 2 = wing-mounted pair (y_motor defaults to the Woombah station; "
        "configs/full_mdo_twin.yaml has the full twin defaults)",
    )
    r.add_argument(
        "--battery-topology",
        choices=["per_motor", "shared"],
        default=None,
        help="one pack per motor (default) or one shared pack; capacity is per pack",
    )
    r.add_argument("--vsp", action="store_true", help="build the OpenVSP mock-up after the run (needs OpenVSP)")
    rob = r.add_mutually_exclusive_group()
    rob.add_argument(
        "--robust",
        action="store_true",
        help="robust design optimisation (needs a robust: block with parameters, e.g. configs/full_mdo_rdo.yaml)",
    )
    rob.add_argument("--no-robust", action="store_true", help="deterministic, even if the config enables robust")
    v = sub.add_parser("vsp", help="build the OpenVSP mock-up of a saved run (from its geometry.json)")
    v.add_argument("run_dir", help="run folder holding geometry.json (or the file)")
    v.add_argument("--out", default=None, help="output folder (default <run_dir>/vsp)")
    v.add_argument("--no-figures", action="store_true")
    v.add_argument(
        "--incidence",
        choices=["zero", "cruise"],
        default=None,
        help="tilt the model to the cruise AoA (default: the config's mockup.wing_incidence)",
    )
    sub.add_parser("plugins", help="list the registered scoring plug-ins")
    sub.add_parser("ui", help="start the Streamlit UI")
    args = ap.parse_args(argv)

    # --- the commands other than run ---
    if args.cmd == "plugins":
        from .scoring import available

        print("\n".join(available()))
        return 0
    if args.cmd == "vsp":
        return _vsp(args.run_dir, args.out, not args.no_figures, args.incidence)
    if args.cmd == "ui":
        import subprocess

        app = Path(__file__).parent / "ui" / "app.py"
        return subprocess.call([sys.executable, "-m", "streamlit", "run", str(app)])

    if args.vsp and not _vsp_available():
        return 2
    from .api import run_design
    from .problems.full_mdo import WOOMBAH_Y_M, PropulsionInputs, load_full_config

    # --- run: config overrides ---
    cfg = load_full_config(args.config)
    if args.prop:
        cfg.propulsion.prop = args.prop
    if args.n_motors is not None or args.battery_topology is not None:
        pr = cfg.propulsion.model_dump()
        if args.n_motors is not None:
            pr["n_motors"] = args.n_motors
            if args.n_motors == 2 and pr["y_motor"] is None and pr["y_motor_m"] is None:
                pr["y_motor_m"] = WOOMBAH_Y_M
                print(f"[aircrafter2] n_motors = 2: y_motor_m = {WOOMBAH_Y_M:.4f} m (Woombah station)")
        if args.battery_topology is not None:
            pr["battery_topology"] = args.battery_topology
        cfg.propulsion = PropulsionInputs(**pr)  # re-validated
    if args.robust or args.no_robust:
        if args.robust and not cfg.robust.parameters:
            print(
                "[aircrafter2] --robust needs a robust: block with uncertain parameters "
                "(see configs/full_mdo_rdo.yaml)",
                file=sys.stderr,
            )
            return 2
        cfg.robust.enabled = bool(args.robust)
    # --- run: the design ---
    out = args.out or str(Path("outputs") / Path(args.config).stem)
    res = run_design(
        cfg,
        optimise=not args.analyse,
        out_dir=out,
        figures=not args.no_figures,
        design_space=args.design_space,
        progress=lambda s: print(f"[aircrafter2] {s} ...", flush=True),
    )
    print(
        f"success = {res.success}, iterations = {res.iterations}, score = {res.values['score']:.4f}"
        + (" (nominal)" if "robust" in res.values else "")
    )
    if "robust" in res.values:
        st = res.values["robust"]["stats"]["score"]
        print(
            f"robust: score mu = {st['mu']:.4f}, sigma = {st['sigma']:.4f}, mu + k sigma = {st['robust']:.4f}"
            + (f"; {res.values['robust']['error']}" if res.values["robust"].get("error") else "")
        )
    print(f"outputs in {out}/ (summary.md, results.json, geometry.json, n2.html, figures/)")
    if args.vsp:
        rc = _vsp(out, None, not args.no_figures, None)
        if rc:
            return rc
    return 0 if res.success else 1


def _vsp_available():
    """True if OpenVSP is installed; otherwise print the install hint."""
    from .vsp import INSTALL_HINT, have_openvsp

    if not have_openvsp():
        print(f"[aircrafter2] {INSTALL_HINT}", file=sys.stderr)
        return False
    return True


def _vsp(run_dir, out, figures, incidence):
    """Build the OpenVSP mock-up of a saved run. Returns the exit code."""
    if not _vsp_available():
        return 2
    from .vsp.pipeline import build_run

    res = build_run(run_dir, out_dir=out, figures=figures, incidence=incidence)
    flagged = res["check"]["flagged"]
    print(f"[aircrafter2] OpenVSP model built in {res['build_s']:.1f} s: {res['files']['vsp3']}")
    print(f"[aircrafter2] consistency check: {res['files']['check']} ({len(flagged)} item(s) over 5 %)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
