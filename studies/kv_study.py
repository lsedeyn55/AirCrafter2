"""Motor Kv as a design variable: study (docs/studies/kv_study.md). Each part writes outputs/kv/<part>.json.

    .venv/bin/python studies/kv_study.py corners     # MDA at the Kv bounds (start point, fixed-Kv optimum)
    .venv/bin/python studies/kv_study.py compare     # single 10x7E 4S: fixed Kv = 900 vs Kv a design variable
    .venv/bin/python studies/kv_study.py sweep       # Kv-DV optimum against the current limit I_max
    .venv/bin/python studies/kv_study.py robust      # configs/full_mdo_rdo.yaml (Kv a DV) vs the fixed-Kv RDO result
    .venv/bin/python studies/kv_study.py report      # docs/studies/kv_study.md, kv_study_results.json, figures

Threads: OMP_NUM_THREADS (default 3, as for the RDO study's deterministic runs).
"""

import os

os.environ.setdefault("OMP_NUM_THREADS", "3")

import argparse  # noqa: E402
import json  # noqa: E402
import subprocess  # noqa: E402
import time  # noqa: E402
import warnings  # noqa: E402
from pathlib import Path  # noqa: E402

import numpy as np  # noqa: E402

from aircrafter2.api import motor_report  # noqa: E402
from aircrafter2.problems.full_mdo import build_full_problem, full_constraints, load_full_config  # noqa: E402

warnings.filterwarnings("ignore")
ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "outputs" / "kv"
PROP = "10x7E"
I_MAX_SWEEP = [40.0, 50.0, 60.0, 70.0]
REPORT = ["ThreeLapTime", "score", "m_total", "v", "I_static", "I_margin", "rpm_margin", "energy_margin",
          "thrust_margin_static", "T_static", "capacity", "Kv", "Rm", "I0", "laps_frac"]


def v(p, k):
    return float(np.ravel(p.get_val(k))[0])


def dump(name, data):
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / f"{name}.json").write_text(json.dumps(data, indent=1, default=lambda o: o.item() if hasattr(o, "item")
                                                 else o.tolist()))


def load(name):
    return json.loads((OUT / f"{name}.json").read_text())


def git_hash():
    return subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=ROOT, capture_output=True,
                          text=True).stdout.strip()


def config(name, I_max=None):
    cfg = load_full_config(ROOT / "configs" / name)
    cfg.propulsion.prop = PROP
    if I_max is not None:
        cfg.propulsion.I_max = I_max
    return cfg


def active_constraints(prob, cfg, tol=1e-4):
    """Constraints within tol (scaled by ref) of a bound."""
    out = []
    for name, kw in full_constraints(cfg):
        x = v(prob, name)
        ref = kw.get("ref") or 1.0
        for side in ("lower", "upper"):
            b = kw.get(side)
            if b is not None and abs(x - b) / ref < tol:
                out.append(f"{name} ({side})")
    return out


def active_bounds(prob, cfg, tol=1e-4):
    return [f"{k} ({side})" for k, bd in cfg.design_variables.items()
            for side, b in (("lower", bd.lower), ("upper", bd.upper))
            if abs(v(prob, k) - b) <= tol * (bd.upper - bd.lower)]


def optimise(cfg, label):
    prob = build_full_problem(cfg)
    prob.driver.options["disp"] = False
    t = time.perf_counter()
    res = prob.run_driver()
    wall = time.perf_counter() - t
    out = dict(label=label, ok=bool(res.success), its=int(res.iter_count), time=wall,
               dv={k: v(prob, k) for k in cfg.design_variables}, values={k: v(prob, k) for k in REPORT},
               motor=motor_report(prob, cfg), active_constraints=active_constraints(prob, cfg),
               active_bounds=active_bounds(prob, cfg), I_max=cfg.propulsion.I_max, prop=cfg.propulsion.prop,
               threads=os.environ.get("OMP_NUM_THREADS"))
    print(f"{label}: ok={out['ok']} its={out['its']} {wall:.0f}s t3={out['values']['ThreeLapTime']:.3f} "
          f"Kv={out['values']['Kv']:.1f} set by {out['motor']['set_by']}", flush=True)
    return out


def corners():
    """The coupled analysis at Kv = 500 / 900 / 1500, other DVs at the start point and at the fixed-Kv optimum."""
    from openmdao.core.analysis_error import AnalysisError

    cfg = config("full_mdo.yaml")
    rows = []
    det = ROOT / "outputs" / "rdo" / "det_single_10x7E.json"  # fixed-Kv 10x7E optimum (RDO study), if present
    points = {"start": {}}
    if det.exists():
        points["fixed-Kv optimum"] = json.loads(det.read_text())["dv"]
    for label, dv in points.items():
        for kv in (500.0, 900.0, 1500.0):
            prob = build_full_problem(cfg, driver=False)
            for k, x in dv.items():
                prob.set_val(k, x)
            prob.set_val("Kv", kv)
            try:
                prob.run_model()
                row = dict(point=label, converged=True, **{k: v(prob, k) for k in REPORT})
            except AnalysisError as exc:
                row = dict(point=label, Kv=kv, converged=False, error=str(exc)[:200])
            rows.append(row)
            print({k: (round(x, 3) if isinstance(x, float) else x) for k, x in row.items()
                   if k in ("point", "Kv", "converged", "ThreeLapTime", "I_static", "rpm_margin", "v")}, flush=True)
    dump("corners", dict(rows=rows, git=git_hash()))


def compare():
    fixed = optimise(config("full_mdo_fixed_kv.yaml"), "fixed Kv = 900")
    kv = optimise(config("full_mdo.yaml"), "Kv a design variable")
    dump("compare", dict(fixed=fixed, kv=kv, git=git_hash()))


def sweep():
    rows = [optimise(config("full_mdo.yaml", I_max=i), f"I_max = {i:g} A") for i in I_MAX_SWEEP]
    dump("sweep", dict(rows=rows, git=git_hash()))


def robust():
    from openmdao.core.analysis_error import AnalysisError

    from aircrafter2.robustness.rdo_problem import build_robust_problem, node_report, robust_summary

    cfg = load_full_config(ROOT / "configs" / "full_mdo_rdo.yaml")
    prob = build_robust_problem(cfg)
    prob.driver.options["disp"] = False
    t = time.perf_counter()
    err = None
    try:
        res = prob.run_driver()
        ok, its = bool(res.success), int(res.iter_count)
    except AnalysisError as exc:
        ok, its, err = False, int(prob.driver.iter_count), f"MDA non-convergence: {str(exc)[:200]}"
    wall = time.perf_counter() - t
    summ = robust_summary(prob)
    out = dict(ok=ok, its=its, error=err, time=wall, N=prob.rdo["N"], active=[d.name for d in prob.rdo["active"]],
               dv={k: v(prob, k) for k in cfg.design_variables},
               summary={q: {kk: vv for kk, vv in d.items() if kk != "nodes"} for q, d in summ.items()},
               reference_node={k: v(prob, f"nodes.n0.{k}") for k in REPORT}, nodes=node_report(prob),
               k=cfg.robust.k, beta=cfg.robust.beta, threads=os.environ.get("OMP_NUM_THREADS"), git=git_hash())
    dump("robust", out)
    print(f"robust: ok={ok} its={its} {wall:.0f}s mu={out['summary']['score']['mu']:.3f} "
          f"sigma={out['summary']['score']['sigma']:.3f} Kv={out['dv']['Kv']:.1f} {err or ''}", flush=True)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("part", choices=["corners", "compare", "sweep", "robust", "report"])
    globals()[ap.parse_args().part]()
