"""2024-25 results: multipoint optimisation, bookends, enumeration, RDO, common evaluation
(docs/studies/results/results_2025.md). Each run writes outputs/results/runs/<tag>.json.

    .venv/bin/python studies/results_2025.py optimise --tag mp_6S_10x7E_10x7E [--cells 6] [--m2 10x7E] [--m3 10x7E]
                                                       [--w2 1 --w3 1] [--R2 0.2 --R3 27] [--start TAG]
    .venv/bin/python studies/results_2025.py bookends --case mp_6S_10x7E_10x7E      # M2-only and M3-only optima
    .venv/bin/python studies/results_2025.py report

Bookends: the same multipoint problem (both missions flown, every constraint at both points) with the objective
reduced to one mission's score (dbf2025 weights w2, w3), so the difference to the multipoint optimum is what the
shared airframe gives up on each mission.
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

warnings.filterwarnings("ignore")
ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "outputs" / "results"
RUNS = OUT / "runs"
CFG = ROOT / "configs" / "dbf2025_multipoint.yaml"
POINT_OUT = ["m_total", "m_empty", "m_pay", "m_wing", "m_ht", "m_vt", "m_fus", "m_gear", "m_sys", "m_batt", "v",
             "ThreeLapTime", "laps_frac", "I_static", "I_pack_static", "rpm_margin", "I_margin", "SM", "CD0",
             "CD_stores", "energy_margin", "E_nom_total", "thrust_margin_static", "v_fit_margin", "stall_margin",
             "cruise.L_over_D", "Rm", "I0"]


def v(p, k):
    return float(np.ravel(p.get_val(k))[0])


def git_hash():
    return subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=ROOT, capture_output=True,
                          text=True).stdout.strip()


def dump(tag, data):
    RUNS.mkdir(parents=True, exist_ok=True)
    (RUNS / f"{tag}.json").write_text(json.dumps(data, indent=1, default=lambda o: o.item() if hasattr(o, "item")
                                                 else o.tolist()))


def load(tag):
    return json.loads((RUNS / f"{tag}.json").read_text())


def config(cells=None, m2=None, m3=None, scoring=None, cfg=CFG):
    from aircrafter2.problems.multipoint import load_multipoint_config

    mcfg = load_multipoint_config(cfg)
    if cells is not None:
        mcfg.base.propulsion.n_cells = float(cells)
    if m2:
        mcfg.mp.points["M2"].prop = m2
    if m3:
        mcfg.mp.points["M3"].prop = m3
    if scoring:
        mcfg.mp.scoring.update(scoring)
    return mcfg


def active(p, mcfg, tol=1e-4):
    from aircrafter2.problems.multipoint import multipoint_constraints

    out = []
    for name, kw in multipoint_constraints(mcfg):
        x = v(p, name)
        ref = kw.get("ref") or 1.0
        for side in ("lower", "upper"):
            b = kw.get(side)
            if b is not None and abs(x - b) / ref < tol:
                out.append(f"{name} ({side})")
    bounds = []
    for name, bd in mcfg.all_dvs().items():
        up = min(bd.upper, mcfg.mp.rules.span_max_ft * 0.3048) if name == "b" else bd.upper
        for side, b in (("lower", bd.lower), ("upper", up)):
            if abs(v(p, name) - b) <= tol * max(up - bd.lower, 1e-12):
                bounds.append(f"{name} ({side})")
    return out, bounds


def summarise(p, mcfg):
    from aircrafter2.problems.multipoint import design_values, m1_check

    out = dict(dv=design_values(p, mcfg), score={k: v(p, k) for k in ["score", "M1", "M2", "M3", "flight_score",
                                                                      "n_pre"]},
               props={pt: mcfg.mp.points[pt].prop for pt in ("M2", "M3")}, cells=mcfg.base.propulsion.n_cells,
               scoring=dict(mcfg.mp.scoring))
    for pt in ("M2", "M3"):
        out[pt] = {k: v(p, f"points.{pt}.{k}") for k in POINT_OUT}
    out["fuel_lb"] = out["dv"]["fuel_mass"] / 0.45359237
    out["active_constraints"], out["active_bounds"] = active(p, mcfg)
    kv_set = [c for c in out["active_constraints"] if "rpm_margin" in c or "I_margin" in c or "fuse" in c]
    kv_set += [b for b in out["active_bounds"] if b.startswith("Kv")]
    out["kv_set_by"] = ", ".join(kv_set) if kv_set else "interior"
    out["M1_check"] = m1_check(mcfg, p)
    return out


def optimise(tag, cells=None, m2=None, m3=None, scoring=None, start=None, x0=None, cfg=CFG):
    from openmdao.core.analysis_error import AnalysisError

    from aircrafter2.problems.multipoint import build_multipoint_problem

    mcfg = config(cells, m2, m3, scoring, cfg)
    p = build_multipoint_problem(mcfg, driver=True)
    p.driver.options["disp"] = False
    if start:
        for k, x in load(start)["dv"].items():
            p.set_val(k, x)
    for k, x in (x0 or {}).items():
        p.set_val(k, x)
    t = time.perf_counter()
    err = None
    try:
        res = p.run_driver()
        ok, its = bool(res.success), int(res.iter_count)
    except AnalysisError as exc:
        ok, its, err = False, int(p.driver.iter_count), f"MDA non-convergence: {str(exc)[:200]}"
    wall = time.perf_counter() - t
    out = dict(tag=tag, ok=ok, its=its, error=err, time=wall, start=start, x0=x0, git=git_hash(),
               threads=os.environ.get("OMP_NUM_THREADS"), config=str(Path(cfg).relative_to(ROOT)))
    if err is None:
        out.update(summarise(p, mcfg))
    dump(tag, out)
    sc = out.get("score", {})
    print(f"{tag}: ok={ok} its={its} {wall:.0f}s F={sc.get('flight_score', float('nan')):.4f} "
          f"M2={sc.get('M2', float('nan')):.4f} M3={sc.get('M3', float('nan')):.4f} {err or ''}", flush=True)
    return out


def bookends(case):
    d = load(case)
    kw = dict(cells=d["cells"], m2=d["props"]["M2"], m3=d["props"]["M3"])
    optimise(f"{case}_M2only", scoring={"w2": 1.0, "w3": 0.0}, start=case, **kw)
    optimise(f"{case}_M3only", scoring={"w2": 0.0, "w3": 1.0}, start=case, **kw)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("part", choices=["optimise", "bookends", "report"])
    ap.add_argument("--tag")
    ap.add_argument("--case")
    ap.add_argument("--cells", type=float)
    ap.add_argument("--m2")
    ap.add_argument("--m3")
    ap.add_argument("--w2", type=float)
    ap.add_argument("--w3", type=float)
    ap.add_argument("--R2", type=float)
    ap.add_argument("--R3", type=float)
    ap.add_argument("--start")
    a = ap.parse_args()
    if a.part == "optimise":
        sc = {k: getattr(a, k) for k in ("w2", "w3", "R2", "R3") if getattr(a, k) is not None}
        optimise(a.tag, a.cells, a.m2, a.m3, sc or None, a.start)
    elif a.part == "bookends":
        bookends(a.case)
    else:
        raise SystemExit("report: checkpoint 4")
