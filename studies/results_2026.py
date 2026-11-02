"""2025-26 results (Project Humpback's ruleset): multipoint optimisation, payload rounding, bookends, enumeration, RDO,
common evaluation (docs/studies/results_2026/results_2026.md). Each run writes outputs/results_2026/runs/<tag>.json.

    .venv/bin/python studies/results_2026.py optimise --tag mp_8S_19x12E_19x12E [--cells 8] [--m2 19x12E] [--m3 19x12E]
                                                       [--w2 1 --w3 1] [--NI 2800 --B 1500] [--start TAG]
                                                       [--old-constraints] [--takeoff-off] [--fix n_p=42 n_c=14]
    .venv/bin/python studies/results_2026.py integer --case C      # payload counts rounded, re-optimised (§4.2)

Bookends: the same multipoint problem with the objective reduced to one mission's score (dbf2026 weights w2, w3).
The model and its decisions: docs/RESULTS_2026_PLAN.md.
"""

import os

os.environ.setdefault("OMP_NUM_THREADS", "3")

import argparse  # noqa: E402
import json  # noqa: E402
import math  # noqa: E402
import subprocess  # noqa: E402
import time  # noqa: E402
import warnings  # noqa: E402
from pathlib import Path  # noqa: E402

import numpy as np  # noqa: E402

warnings.filterwarnings("ignore")
ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "outputs" / "results_2026"
RUNS = OUT / "runs"
LB_ = 0.45359237
CFG = ROOT / "configs" / "dbf2026_multipoint.yaml"
RDO_CFG = ROOT / "configs" / "dbf2026_multipoint_rdo.yaml"
POINT_OUT = ["m_total", "m_empty", "m_pay", "m_wing", "m_ht", "m_vt", "m_fus", "m_gear", "m_sys", "m_batt", "v",
             "ThreeLapTime", "Lap1time", "otherLapTime", "laps_frac", "I_static", "I_pack_static", "rpm_margin",
             "I_margin", "SM", "V_h", "V_v", "CD0", "CD_stores", "energy_margin", "E_nom_total",
             "thrust_margin_static", "v_fit_margin", "stall_margin", "cruise.L_over_D", "Rm", "I0", "l_cb",
             "t_takeoff", "s_ground_roll", "x_cg", "x_np"]
SCORE_OUT = ["score", "M1", "M2", "M3", "flight_score", "net_income", "m3_term", "RAC", "EF"]
# the old constraint set (the sensitivity of decision 3): AirCrafter 1.0's rpm convention and the Q9 bands
OLD_CONSTRAINTS = {"propulsion.rpm_fraction": 0.85, "stability.SM_max": 0.20, "stability.V_h_max": 0.65,
                   "stability.V_v_max": 0.06}


def v(p, k):
    return float(np.ravel(p.get_val(k))[0])


def git_hash():
    """Short HEAD commit, with '-dirty' when tracked files differ from it."""
    head = subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=ROOT, capture_output=True,
                          text=True).stdout.strip()
    dirty = subprocess.run(["git", "diff", "--quiet", "HEAD"], cwd=ROOT).returncode != 0
    return head + ("-dirty" if dirty else "")


def dump(tag, data):
    RUNS.mkdir(parents=True, exist_ok=True)
    (RUNS / f"{tag}.json").write_text(json.dumps(data, indent=1, default=lambda o: o.item() if hasattr(o, "item")
                                                 else o.tolist()))


def load(tag):
    return json.loads((RUNS / f"{tag}.json").read_text())


def config(cells=None, m2=None, m3=None, scoring=None, cfg=CFG, fixed=None, old_constraints=False, takeoff=True):
    """The 2025-26 multipoint config with overrides; fixed = {design variable: value} pins DVs (equal bounds)."""
    from aircrafter2.problems.multipoint import load_multipoint_config
    from aircrafter2.problems.wing_aero import Bounds

    mcfg = load_multipoint_config(cfg)
    for name, val in (fixed or {}).items():
        if name in mcfg.mp.payload:
            mcfg.mp.payload[name] = Bounds(lower=val, upper=val)
            mcfg.mp.payload_initial[name] = val
        elif name in mcfg.base.design_variables:
            mcfg.base.design_variables[name] = Bounds(lower=val, upper=val)
            mcfg.base.initial[name] = val
        else:
            raise ValueError(f"cannot fix {name!r}: not a shared or payload design variable")
    if cells is not None:
        mcfg.base.propulsion.n_cells = float(cells)
    if m2:
        mcfg.mp.points["M2"].prop = m2
    if m3:
        mcfg.mp.points["M3"].prop = m3
    if scoring:
        mcfg.mp.scoring.update(scoring)
    if old_constraints:
        mcfg.base.propulsion.rpm_fraction = OLD_CONSTRAINTS["propulsion.rpm_fraction"]
        for k in ("SM_max", "V_h_max", "V_v_max"):
            setattr(mcfg.base.stability, k, OLD_CONSTRAINTS[f"stability.{k}"])
    if not takeoff:
        mcfg.base.course.takeoff.in_mission_time = False
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


def violated(p, mcfg, tol=1e-6):
    from aircrafter2.problems.multipoint import multipoint_constraints

    out = []
    for name, kw in multipoint_constraints(mcfg):
        x = v(p, name)
        ref = kw.get("ref") or 1.0
        if (kw.get("lower") is not None and (x - kw["lower"]) / ref < -tol) or \
                (kw.get("upper") is not None and (x - kw["upper"]) / ref > tol):
            out.append(f"{name} = {x:.4g}")
    return out


def floored_score(p, mcfg):
    """The score as the judges would record it: laps floored to completed laps, the banner rounded down to 1/4 in, the
    span to the nearest inch, the payload counts rounded (dbf2026.floored)."""
    from aircrafter2.scoring.dbf2026 import DBF2026

    plug = DBF2026(**mcfg.mp.scoring)
    x = dict(n_p=v(p, "n_p"), n_c=v(p, "n_c"), laps_M2=v(p, "points.M2.laps_frac"), E_M2=v(p, "points.M2.E_nom_total"),
             laps_M3=v(p, "points.M3.laps_frac"), L_banner=v(p, "L_banner"), b=v(p, "b"))
    return {k: float(np.real(val)) for k, val in plug.floored(x).items()}


def summarise(p, mcfg):
    from aircrafter2.problems.multipoint import design_values, m1_check

    out = dict(dv=design_values(p, mcfg), score={k: v(p, k) for k in SCORE_OUT},
               props={pt: mcfg.mp.points[pt].prop for pt in ("M2", "M3")}, cells=mcfg.base.propulsion.n_cells,
               scoring=dict(mcfg.mp.scoring))
    for pt in ("M2", "M3"):
        out[pt] = {k: v(p, f"points.{pt}.{k}") for k in POINT_OUT}
    out["M3"]["m_banner"] = v(p, "points.M3.m_banner")
    out["M3"]["CD_banner"] = v(p, "points.M3.CD_banner")
    out["floored"] = floored_score(p, mcfg)
    out["active_constraints"], out["active_bounds"] = active(p, mcfg)
    out["violated"] = violated(p, mcfg)
    kv_set = [c for c in out["active_constraints"] if "rpm_margin" in c or "I_margin" in c or "fuse" in c]
    kv_set += [b for b in out["active_bounds"] if b.startswith("Kv")]
    out["kv_set_by"] = ", ".join(kv_set) if kv_set else "interior"
    out["M1_check"] = m1_check(mcfg, p)
    return out


def initial_values(mcfg):
    """{design variable: start value} of a multipoint config, as set_dv_defaults and apply_multipoint_values set them."""
    from aircrafter2.problems.multipoint import POINTS

    base = mcfg.base
    out = {}
    for name in mcfg.shared_dvs:
        bd = base.design_variables[name]
        out[name] = base.initial.get(name, base.propulsion.Kv if name == "Kv" else 0.5 * (bd.lower + bd.upper))
    for pt in POINTS:
        for name, bd in mcfg.point_dvs(pt).items():
            k = name[: -len(pt) - 1]
            out[name] = mcfg.mp.per_point_initial.get(name, base.initial.get(k, 0.5 * (bd.lower + bd.upper)))
    out.update(mcfg.extra_initial())
    return out


def mda_fallbacks(p):
    """Solves that needed the continuation (RetryNewton robust mode), per point MDA."""
    out = {}
    for pt in ("M2", "M3"):
        solver = p.model._get_subsystem(f"points.{pt}.mda").nonlinear_solver
        log = getattr(solver, "log", [])
        out[pt] = dict(solves=getattr(solver, "n_solve", 0),
                       continuation=len({e[0] for e in log if e[1].startswith("continuation")}),
                       failed_attempts=len([e for e in log if not e[2]]))
    return out


def optimise(tag, cells=None, m2=None, m3=None, scoring=None, start=None, x0=None, cfg=CFG, maxiter=None, fixed=None,
             old_constraints=False, takeoff=True):
    from openmdao.core.analysis_error import AnalysisError

    from aircrafter2.problems.multipoint import build_multipoint_problem

    mcfg = config(cells, m2, m3, scoring, cfg, fixed, old_constraints, takeoff)
    if maxiter:
        mcfg.base.maxiter = maxiter
    p = build_multipoint_problem(mcfg, driver=True, mda_retry=True)
    p.driver.options["disp"] = False
    warm = bool(mcfg.base.mda_robust and (start or x0))
    t = time.perf_counter()
    err = None
    if warm:  # converge at the config's (unfixed) start first, so the driver's first evaluation is warm
        for k, x in initial_values(config(cells, m2, m3, scoring, cfg, None, old_constraints, takeoff)).items():
            p.set_val(k, x)
        try:
            p.run_model()
        except AnalysisError as exc:
            err = f"warm-path pre-run: MDA non-convergence: {str(exc)[:200]}"
    if start:
        for k, x in load(start)["dv"].items():
            if not (fixed and k in fixed):
                p.set_val(k, x)
    for k, x in (x0 or {}).items():
        p.set_val(k, x)
    for k, x in (fixed or {}).items():
        p.set_val(k, x)
    try:
        if err is not None:
            raise AnalysisError(err)
        res = p.run_driver()
        ok, its = bool(res.success), int(res.iter_count)
    except AnalysisError as exc:
        ok, its = False, int(p.driver.iter_count)
        err = err or f"MDA non-convergence: {str(exc)[:200]}"
    wall = time.perf_counter() - t
    out = dict(tag=tag, ok=ok, its=its, error=err, time=wall, start=start, x0=x0, warm_path=warm,
               mda_fallbacks=mda_fallbacks(p), git=git_hash(),
               maxiter=mcfg.base.maxiter, fixed=fixed, old_constraints=old_constraints, takeoff=takeoff,
               threads=os.environ.get("OMP_NUM_THREADS"), config=str(Path(cfg).relative_to(ROOT)))
    if err is None:
        out.update(summarise(p, mcfg))
    dump(tag, out)
    sc = out.get("score", {})
    print(f"{tag}: ok={ok} its={its} {wall:.0f}s F={sc.get('flight_score', float('nan')):.4f} "
          f"M2={sc.get('M2', float('nan')):.4f} M3={sc.get('M3', float('nan')):.4f} {err or ''}", flush=True)
    return out


def rounding_check(d, cands, tol=1e-9):
    """The payload-count constraints of each rounded candidate (n_p, n_c) at the continuous optimum d (one analysis
    each): fits = fuselage_margin >= 0 and pax_margin >= 0. With the span at its bound, the fuselage length is set by
    the counts alone, so a candidate that does not fit here is infeasible for every design (docs/RESULTS_2026_PLAN.md
    §4.2, 2026-11-02)."""
    from openmdao.core.analysis_error import AnalysisError

    from aircrafter2.problems.multipoint import build_multipoint_problem

    args = (d["cells"], d["props"]["M2"], d["props"]["M3"], d["scoring"], CFG, None, d.get("old_constraints", False),
            d.get("takeoff", True))
    mcfg = config(*args)
    p = build_multipoint_problem(mcfg, driver=False, mda_retry=True)
    out = {}
    try:
        p.run_model()  # converge at the config's start, then jump (warm)
        for k, x in d["dv"].items():
            p.set_val(k, x)
        for npx, nc in cands:
            p.set_val("n_p", float(npx))
            p.set_val("n_c", float(nc))
            p.run_model()
            fus, pax = v(p, "rules.fuselage_margin"), v(p, "rules.pax_margin")
            out[f"{npx}p{nc}c"] = dict(n_p=npx, n_c=nc, fuselage_margin=fus, pax_margin=pax,
                                       fits=bool(fus >= -tol and pax >= -tol))
    except AnalysisError as exc:
        return dict(error=f"MDA non-convergence: {str(exc)[:200]}")
    return out


def integer(case, maxiter=None):
    """Payload rounding (docs/RESULTS_2026_PLAN.md §4.2): n_c down and up, n_p = max(3 n_c, round(n_p)), plus both
    counts rounded down (n_p = max(3 n_c, floor(n_p))); the candidates that fit the payload fuselage and the passenger
    ratio at the continuous optimum (rounding_check) are each re-optimised with the counts fixed; the better one is
    the result. If none fits, every candidate is run."""
    d = load(case)
    n_p, n_c = d["dv"]["n_p"], d["dv"]["n_c"]
    cands = []
    for nc in sorted({math.floor(n_c), math.ceil(n_c)}):
        nc = max(nc, 1)
        for npx in (max(3 * nc, round(n_p)), max(3 * nc, math.floor(n_p))):
            if (npx, nc) not in cands:
                cands.append((npx, nc))
    check = rounding_check(d, cands)
    keep = [c for c in cands if check.get(f"{c[0]}p{c[1]}c", {}).get("fits")] or cands
    dump(f"{case}_int_check", dict(case=case, candidates=cands, check=check, run=keep, git=git_hash()))
    print(f"{case}: rounding candidates {cands}, run {keep}", flush=True)
    outs = []
    for npx, nc in keep:
        outs.append(optimise(f"{case}_int_{npx}p{nc}c", d["cells"], d["props"]["M2"], d["props"]["M3"],
                             scoring=d["scoring"], start=case, fixed={"n_p": float(npx), "n_c": float(nc)},
                             old_constraints=d.get("old_constraints", False), takeoff=d.get("takeoff", True),
                             maxiter=maxiter))
    return outs


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("part", choices=["optimise", "integer"])
    ap.add_argument("--tag")
    ap.add_argument("--case")
    ap.add_argument("--cells", type=float)
    ap.add_argument("--m2")
    ap.add_argument("--m3")
    ap.add_argument("--w2", type=float)
    ap.add_argument("--w3", type=float)
    ap.add_argument("--NI", type=float, help="NI_max")
    ap.add_argument("--B", type=float, help="B_max")
    ap.add_argument("--start")
    ap.add_argument("--maxiter", type=int)
    ap.add_argument("--old-constraints", action="store_true")
    ap.add_argument("--takeoff-off", action="store_true")
    ap.add_argument("--fix", nargs="*", default=[], help="name=value pins")
    a = ap.parse_args()
    if a.part == "optimise":
        sc = {k: val for k, val in (("w2", a.w2), ("w3", a.w3), ("NI_max", a.NI), ("B_max", a.B)) if val is not None}
        fixed = {s.split("=")[0]: float(s.split("=")[1]) for s in a.fix} or None
        optimise(a.tag, a.cells, a.m2, a.m3, sc or None, a.start, maxiter=a.maxiter, fixed=fixed,
                 old_constraints=a.old_constraints, takeoff=not a.takeoff_off)
    elif a.part == "integer":
        integer(a.case)
