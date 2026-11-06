"""2025-26 results (Project Humpback's ruleset): the run parts of the final thesis campaign. Each run writes
outputs/results_2026/runs/<tag>.json (the Monte Carlo arrays: outputs/results_2026/mc/<tag>.npz). The order and the
resume logic: studies/results_2026_campaign.py; the pages: studies/results_2026_report.py.

    .venv/bin/python studies/results_2026.py optimise --tag mp_8S_19x12E_19x12E [--cells 8] [--m2 19x12E] [--m3 19x12E]
                                                       [--w2 1 --w3 1] [--NI 2800 --B 1500] [--start TAG]
                                                       [--old-constraints] [--takeoff-off] [--fix n_p=42 n_c=14]
                                                       [--mean-wind] [--override stability.V_h_min=0.5] [--bound c.upper=0.6]
    .venv/bin/python studies/results_2026.py integer --case C [--maxiter N]  # payload counts rounded, re-optimised
    .venv/bin/python studies/results_2026.py rdo --tag T --case C [--start TAG] [--free] [--maxiter N]  # robust optimisation
    .venv/bin/python studies/results_2026.py rdo_integer --case RDOTAG [--maxiter 15]  # robust payload rounding
    .venv/bin/python studies/results_2026.py multistart --case C --index I           # Sobol' start I in the design box
    .venv/bin/python studies/results_2026.py common --label L     # the common evaluation: nominal, mean wind, details
    .venv/bin/python studies/results_2026.py mc --label L --set in|out [--workers 3]  # 1024 scrambled Sobol' points
    .venv/bin/python studies/results_2026.py propagation            # UDR, Smolyak level 2, Sobol' at the robust optimum
    .venv/bin/python studies/results_2026.py wind_dir --label L     # F against direction at the mean wind speed
    .venv/bin/python studies/results_2026.py wind_env --label L     # wind limit against direction (bisection)
    .venv/bin/python studies/results_2026.py screen --label L       # first-order and 1-D contributions (extended list)
    .venv/bin/python studies/results_2026.py node_check             # every UDR node at the optimum, start, 6 Sobol' designs
    .venv/bin/python studies/results_2026.py reanalysis --label L   # C_D,b and rpm-limit re-analyses
    .venv/bin/python studies/results_2026.py placeholders           # re-analysis of every placeholder over its range

Design labels (`--label`): humpback, humpback_loo, ac1_direct, ac1_completed, mdo, mdo_wind, rdo (designs()).
Bookends: the same multipoint problem with the objective reduced to one mission's score (dbf2026 weights w2, w3).
The model and its decisions: docs/development/RESULTS_2026_PLAN.md.
"""

import os

os.environ.setdefault("OMP_NUM_THREADS", "3")

import argparse  # noqa: E402
import json  # noqa: E402
import math  # noqa: E402
import resource  # noqa: E402
import subprocess  # noqa: E402
import sys  # noqa: E402
import time  # noqa: E402
import warnings  # noqa: E402
from pathlib import Path  # noqa: E402

import numpy as np  # noqa: E402

warnings.filterwarnings("ignore")
ROOT = Path(__file__).resolve().parents[1]
OUT = Path(os.environ.get("RESULTS_2026_OUT", ROOT / "outputs" / "results_2026"))  # the variable: smoke runs and tests
RUNS = OUT / "runs"
LB_ = 0.45359237
CFG = ROOT / "configs" / "dbf2026_multipoint.yaml"
RDO_CFG = ROOT / "configs" / "dbf2026_multipoint_rdo.yaml"
FREEZE_TAG = "results-2026-final"
MC_N = 1024
MC_DIR = OUT / "mc"
MEAN_WIND = {"lap.wind": 5.0, "lap.wind_dir": 0.0}  # the mean of the uncertain set (configs/rulesets/dbf_2025_26.yaml)
# The out-of-set list: configs/uncertainty.yaml's parameters that act on the 2025-26 problem (Kv a design variable, m_pay
# computed by the stores; every spread a PLACEHOLDER, Q32), plus the areal tail factors a_ht, a_vt at the ±30 % that list
# gives f_ht and f_vt (which don't act under the areal tail model).
EXTRA_TAILS = [dict(name="a_ht", dist="uniform", spread=0.30, relative=True, source="PLACEHOLDER"),
               dict(name="a_vt", dist="uniform", spread=0.30, relative=True, source="PLACEHOLDER")]
NOT_IN_2026 = {"f_ht", "f_vt", "m_pay"}
DIRECTIONS = [-30.0, -22.5, -15.0, -7.5, 0.0, 7.5, 15.0, 22.5, 30.0]  # deg: 9 over the set's ±30 (Q59)
CD_BANNER_CASES = {"0.019 ([HB] Table 41, low)": 0.019, "0.028 ([HB] Table 41, high)": 0.028,
                   "0.03 (AirCrafter 1.0)": 0.03, "0.034 (Table 29 deployed, on L x H)": 0.034}
POINT_OUT = ["m_total", "m_empty", "m_pay", "m_wing", "m_ht", "m_vt", "m_fus", "m_gear", "m_sys", "m_batt", "v",
             "ThreeLapTime", "Lap1time", "otherLapTime", "laps_frac", "I_static", "I_pack_static", "rpm_margin",
             "I_margin", "SM", "V_h", "V_v", "CD0", "CD_stores", "energy_margin", "E_nom_total",
             "thrust_margin_static", "v_fit_margin", "stall_margin", "cruise.L_over_D", "Rm", "I0", "l_cb",
             "t_takeoff", "s_ground_roll", "x_cg", "x_np", "cruise.nf_confidence_min", "x_motor", "x_pay",
             "batt_fwd_margin", "batt_aft_margin", "tail_margin", "x_cb_aft"]
SCORE_OUT = ["score", "M1", "M2", "M3", "flight_score", "net_income", "m3_term", "RAC", "EF", "r_M2", "r_M3"]
# the old constraint set (the sensitivity of decision 3): AirCrafter 1.0's rpm convention and the Q9 bands
# SM_max: the old 0.20 shifted into the model frame by the same +0.20 c as SM_min (Q9, Q62; 0.20 itself sits below the
# 2025-26 SM_min 0.25, so the old set could never be feasible). SM_min stays 0.25.
OLD_CONSTRAINTS = {"propulsion.rpm_fraction": 0.85, "stability.SM_max": 0.40, "stability.V_h_max": 0.65,
                   "stability.V_v_max": 0.06}


def initial_values(mcfg):
    """problems.multipoint.initial_values (moved there; kept here for this module's callers)."""
    from aircrafter2.problems.multipoint import initial_values as _iv

    return _iv(mcfg)


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
    (RUNS / f"{tag}.json").write_text(json.dumps(data, indent=1, default=lambda o: o.tolist()))


def load(tag):
    return json.loads((RUNS / f"{tag}.json").read_text())


def set_path(mcfg, path, val):
    """Set one dotted config path on a loaded multipoint config, as the layer files name it: `multipoint.a.b` on mcfg.mp,
    any other on mcfg.base (pydantic sections and the mass inputs' dict)."""
    parts = path.split(".")
    obj = mcfg.mp if parts[0] == "multipoint" else mcfg.base
    parts = parts[1:] if parts[0] == "multipoint" else parts
    for k in parts[:-1]:
        obj = obj[k] if isinstance(obj, dict) else getattr(obj, k)
    if isinstance(obj, dict):
        if parts[-1] not in obj:
            raise KeyError(path)
        obj[parts[-1]] = val
    else:
        if not hasattr(obj, parts[-1]):
            raise KeyError(path)
        setattr(obj, parts[-1], val)


def config(cells=None, m2=None, m3=None, scoring=None, cfg=CFG, fixed=None, old_constraints=False, takeoff=True,
           overrides=None, bounds=None):
    """The 2025-26 multipoint config with overrides; fixed = {design variable: value} pins DVs (equal bounds);
    overrides = {dotted config path: value}; bounds = {shared design variable: {lower, upper}} replaces bounds (the bound
    sensitivities: study overrides on the loaded config, no config file changes)."""
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
    for path, val in (overrides or {}).items():
        set_path(mcfg, path, val)
    for name, new in (bounds or {}).items():
        bd = mcfg.base.design_variables[name]
        mcfg.base.design_variables[name] = Bounds(lower=new.get("lower", bd.lower), upper=new.get("upper", bd.upper))
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


def kv_set_by(active_constraints, active_bounds):
    """What sets Kv: the active rpm, current or fuse limits (rules.fuse_margin*, not rules.fuselage_margin), or its
    bound; else interior."""
    kv_set = [c for c in active_constraints if "rpm_margin" in c or "I_margin" in c or c.startswith("rules.fuse_margin")]
    kv_set += [b for b in active_bounds if b.startswith("Kv")]
    return ", ".join(kv_set) if kv_set else "interior"


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
    from aircrafter2.scoring.dbf2026 import DBF2026

    out["cap"] = dict(delta=DBF2026(**mcfg.mp.scoring).options["cap_delta"], r_M2=out["score"]["r_M2"],
                      r_M3=out["score"]["r_M3"], M2_capped=out["score"]["r_M2"] >= 1.0,
                      M3_capped=out["score"]["r_M3"] >= 1.0)
    out["active_constraints"], out["active_bounds"] = active(p, mcfg)
    out["violated"] = violated(p, mcfg)
    out["kv_set_by"] = kv_set_by(out["active_constraints"], out["active_bounds"])
    out["M1_check"] = m1_check(mcfg, p)
    out["details"] = details(p, mcfg)
    return out


GEOM_INPUTS = ["AR_ht", "AR_vt", "boom_offset", "l_tb", "r_tb", "w_cb", "h_cb"]
MASS_DETAIL = ["m_tube", "m_rib", "m_le", "m_te", "m_film", "m_wing_bare", "m_tb", "m_cb", "m_motor_group", "m_avionics"]
DRAG_DETAIL = ["cruise.vlm.wing_perf.CDi", "cruise.CDp", "parasite.CD_ht", "parasite.CD_vt", "parasite.CD_cb", "parasite.CD_boom", "CD_fus", "CD_tail",
               "CD_areas", "CD_stores", "CD_banner", "dCD_stowed", "cruise.vlm.wing_perf.CL", "alpha_cruise", "Vstall",
               "T_static", "turn_speed_margin", "D_prop", "Kv"]


def _opt(p, k):
    """A scalar the point may not have (None then)."""
    try:
        return v(p, k)
    except KeyError:
        return None


def details(p, mcfg):
    """Everything the pages' geometry, mass and drag tables and the planform figures read, plus each constraint with its
    margin; not used by the optimiser."""
    from aircrafter2.problems.multipoint import multipoint_constraints

    dv = {k: v(p, k) for k in mcfg.all_dvs()}
    geo = {k: _opt(p, f"points.M2.{k}") for k in GEOM_INPUTS}
    geo.update({k: dv[k] for k in ("b", "c", "l_ht", "S_ht", "S_vt")})
    geo.update(l_cb=v(p, "points.M2.l_cb"), x_cb=v(p, "points.M2.stations.x_cb"), x_motor=v(p, "points.M2.x_motor"),
               x_cb_aft=v(p, "points.M2.x_cb_aft"))
    geo["D_prop"] = max(v(p, f"points.{pt}.D_prop") for pt in ("M2", "M3"))
    out = dict(geometry=geo, constraints=[])
    for pt in ("M2", "M3"):
        out[pt] = {k: _opt(p, f"points.{pt}.{k}") for k in MASS_DETAIL + DRAG_DETAIL}
    for name, kw in multipoint_constraints(mcfg):
        x = v(p, name)
        margins = []
        if kw.get("lower") is not None:
            margins.append(x - kw["lower"])
        if kw.get("upper") is not None:
            margins.append(kw["upper"] - x)
        out["constraints"].append(dict(name=name, value=x, lower=kw.get("lower"), upper=kw.get("upper"),
                                       ref=kw.get("ref"), margin=min(margins), units=kw.get("units")))
    return out


def instrument(prob):
    """Counters on every Newton solver, linear solver and the driver (instance-attribute wrappers, as
    compute_performance.py): MDA solves, Newton iterations, linear solves, compute_totals calls, model runs."""
    import openmdao.api as om

    count = dict(mda_solves=0, newton_iterations=0, linear_solves=0, compute_totals=0, model_runs=0)
    seen = set()

    def wrap(obj, attr, key):
        f = getattr(obj, attr)

        def g(*a, **k):
            count[key] += 1
            return f(*a, **k)

        setattr(obj, attr, g)

    for sys_ in prob.model.system_iter(include_self=True, recurse=True):
        ns = sys_.nonlinear_solver
        if isinstance(ns, om.NewtonSolver) and id(ns) not in seen:
            seen.add(id(ns))
            wrap(ns, "solve", "mda_solves")
            wrap(ns, "_single_iteration", "newton_iterations")
        for ls in (sys_.linear_solver, getattr(ns, "linear_solver", None)):
            if ls is not None and not isinstance(ls, om.LinearRunOnce) and id(ls) not in seen:
                seen.add(id(ls))
                wrap(ls, "solve", "linear_solves")
    wrap(prob.model, "run_solve_nonlinear", "model_runs")
    if prob.driver is not None and hasattr(prob.driver, "_compute_totals"):
        wrap(prob.driver, "_compute_totals", "compute_totals")
    return count


def slsqp_nit(driver):
    """SLSQP major iterations (scipy's nit); driver.iter_count counts objective evaluations."""
    res = getattr(driver, "_scipy_optimize_result", None)
    return int(res.nit) if res is not None and hasattr(res, "nit") else None


def peak_rss_gb():
    """This process's peak resident memory [GB]."""
    return resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1e6


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
             old_constraints=False, takeoff=True, wind=None, overrides=None, bounds=None):
    """One deterministic multipoint optimisation. wind = {lap.wind: m/s, lap.wind_dir: deg} flies both points at that
    steady wind (the mean-wind run; the M1 check stays in calm air). overrides, bounds: see config()."""
    from openmdao.core.analysis_error import AnalysisError

    from aircrafter2.problems.multipoint import build_multipoint_problem, run_warm
    from aircrafter2.robustness.rdo_multipoint import param_inputs

    mcfg = config(cells, m2, m3, scoring, cfg, fixed, old_constraints, takeoff, overrides, bounds)
    if maxiter:
        mcfg.base.maxiter = maxiter
    p = build_multipoint_problem(mcfg, driver=True, mda_retry=True)
    p.driver.options["disp"] = False
    for name, val in (wind or {}).items():
        for inp in param_inputs(name):
            p.set_val(inp, val)
    counters = instrument(p)
    warm = bool(mcfg.base.mda_robust and (start or x0))
    t = time.perf_counter()
    err = None
    if warm:  # converge at the config's (unfixed) start first, so the driver's first evaluation is warm
        try:
            run_warm(p, config(cells, m2, m3, scoring, cfg, None, old_constraints, takeoff), {}, run=False)
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
    out = dict(tag=tag, ok=ok, its=its, nit=slsqp_nit(p.driver), error=err, time=wall, start=start, x0=x0,
               warm_path=warm, mda_fallbacks=mda_fallbacks(p), counters=dict(counters), peak_rss_gb=peak_rss_gb(),
               git=git_hash(), maxiter=mcfg.base.maxiter, fixed=fixed, old_constraints=old_constraints, takeoff=takeoff,
               wind=wind, overrides=overrides, bounds=bounds,
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
    the counts alone, so a candidate that does not fit here is infeasible for every design (docs/development/RESULTS_2026_PLAN.md
    §4.2, 2026-11-02)."""
    from openmdao.core.analysis_error import AnalysisError

    from aircrafter2.problems.multipoint import build_multipoint_problem, run_warm

    args = (d["cells"], d["props"]["M2"], d["props"]["M3"], d["scoring"], CFG, None, d.get("old_constraints", False),
            d.get("takeoff", True))
    mcfg = config(*args)
    p = build_multipoint_problem(mcfg, driver=False, mda_retry=True)
    out = {}
    try:
        run_warm(p, mcfg, d["dv"], run=False)  # converge at the config's start, then jump (warm)
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


def _finished(tag):
    """The run exists at this commit (the resume rule of the campaign)."""
    f = RUNS / f"{tag}.json"
    return f.exists() and json.loads(f.read_text()).get("git") == git_hash()


def integer(case, maxiter=None):
    """Payload rounding (docs/development/RESULTS_2026_PLAN.md §4.2): n_c down and up, n_p = max(3 n_c, round(n_p)), plus both
    counts rounded down (n_p = max(3 n_c, floor(n_p))); the candidates that fit the payload fuselage and the passenger
    ratio at the continuous optimum (rounding_check) are each re-optimised with the counts fixed; the better one is
    the result. If none fits, every candidate is run."""
    d = load(case)
    cands = rounded_candidates(d["dv"]["n_p"], d["dv"]["n_c"])
    check = rounding_check(d, cands)
    keep = [c for c in cands if check.get(f"{c[0]}p{c[1]}c", {}).get("fits")] or cands
    dump(f"{case}_int_check", dict(case=case, candidates=cands, check=check, run=keep, git=git_hash()))
    print(f"{case}: rounding candidates {cands}, run {keep}", flush=True)
    outs = []
    for npx, nc in keep:
        if _finished(f"{case}_int_{npx}p{nc}c"):
            outs.append(load(f"{case}_int_{npx}p{nc}c"))
            continue
        outs.append(optimise(f"{case}_int_{npx}p{nc}c", d["cells"], d["props"]["M2"], d["props"]["M3"],
                             scoring=d["scoring"], start=case, fixed={"n_p": float(npx), "n_c": float(nc)},
                             old_constraints=d.get("old_constraints", False), takeoff=d.get("takeoff", True),
                             maxiter=maxiter, wind=d.get("wind")))
    return outs


def rdo(tag, case, maxiter=None, start=None, free=False, fixed=None):
    """One robust optimisation (configs/dbf2026_multipoint_rdo.yaml, docs/development/RESULTS_2026_PLAN.md §7) at the cells, props and
    scoring of run `case`, from the design of run `start` (default `case`). free: the payload counts are design variables
    (else they stay as `case` fixed them); fixed: pin these instead. The warm path: converge the robust problem at the
    config's (unfixed) start, set the design, then run the driver."""
    from openmdao.core.analysis_error import AnalysisError

    from aircrafter2.problems.multipoint import design_values, run_warm
    from aircrafter2.robustness.rdo_multipoint import build_robust_multipoint, node_report_mp, robust_summary_mp

    run = load(case)
    start_dv = (load(start) if start and (RUNS / f"{start}.json").exists() else _start_design(start, run))["dv"]
    args = (run["cells"], run["props"]["M2"], run["props"]["M3"], run["scoring"], RDO_CFG)
    rest = (run.get("old_constraints", False), run.get("takeoff", True))
    pins = fixed if fixed is not None else (None if free else run.get("fixed"))
    mcfg = config(*args, pins, *rest)
    if maxiter:
        mcfg.base.maxiter = maxiter
    p = build_robust_multipoint(mcfg, driver=True)
    p.driver.options["disp"] = False
    counters = instrument(p)
    t = time.perf_counter()
    err, ok, its = None, False, 0
    try:
        try:
            run_warm(p, config(*args, None, *rest), {**start_dv, **(pins or {})}, run=False)
        except AnalysisError as exc:
            raise AnalysisError(f"warm-path pre-run: {exc}") from exc
        res = p.run_driver()
        ok, its = bool(res.success), int(res.iter_count)
    except AnalysisError as exc:
        its = int(p.driver.iter_count)
        kind = "wind limit (lap_invalid)" if "non-physical lap" in str(exc) else "MDA non-convergence"
        err = f"{kind}: {str(exc)[:200]}"
    wall = time.perf_counter() - t
    try:
        summary = robust_summary_mp(p)
    except Exception:  # noqa: BLE001 - after a failure the stats may be unset
        summary = None
    fallbacks = [dict(point=pt, node=j, continuation=len({e[0] for e in s.log if e[1].startswith("continuation")}),
                      failed_attempts=len([e for e in s.log if not e[2]]))
                 for pt, ss in p.rdo["solvers"].items() for j, s in enumerate(ss)]
    out = dict(tag=tag, case=case, start=start or case, ok=ok, its=its, nit=slsqp_nit(p.driver), error=err, time=wall,
               N=p.rdo["N"], active=[d.name for d in p.rdo["active"]], dv=design_values(p, mcfg), summary=summary,
               nodes=node_report_mp(p), fallbacks=fallbacks, warm_path=True, props=run["props"], cells=run["cells"],
               scoring=run["scoring"], fixed=pins, maxiter=mcfg.base.maxiter, k=mcfg.base.robust.k,
               beta=mcfg.base.robust.beta, counters=dict(counters), peak_rss_gb=peak_rss_gb(),
               git=git_hash(), threads=os.environ.get("OMP_NUM_THREADS"), config=str(RDO_CFG.relative_to(ROOT)))
    dump(tag, out)
    fs = (summary or {}).get("flight_score", {})
    print(f"{tag}: ok={ok} its={its} {wall:.0f}s mu={fs.get('mu', float('nan')):.4f} "
          f"sigma={fs.get('sigma', float('nan')):.4f} {err or ''}", flush=True)
    return out


def _start_design(start, run):
    """The start of an RDO: `humpback` = the config's start design (Humpback's), with `run`'s other choices."""
    if start == "humpback":
        from aircrafter2.problems.multipoint import initial_values as _iv

        return dict(dv=_iv(config(run["cells"], run["props"]["M2"], run["props"]["M3"])))
    return run


def rounded_candidates(n_p, n_c):
    """The payload-rounding candidates of integer(): n_c down and up, n_p = max(3 n_c, round / floor of n_p)."""
    cands = []
    for nc in sorted({math.floor(n_c), math.ceil(n_c)}):
        nc = max(nc, 1)
        for npx in (max(3 * nc, round(n_p)), max(3 * nc, math.floor(n_p))):
            if (npx, nc) not in cands:
                cands.append((npx, nc))
    return cands


def rdo_integer(case, maxiter=15):
    """Round the best RDO start's payload counts with the robust objective: the candidates that fit (rounding_check),
    each a warm robust re-optimisation with the counts fixed (one run per candidate: rdo_int_one)."""
    d = load(case)
    src = dict(d, scoring=d["scoring"])
    cands = rounded_candidates(d["dv"]["n_p"], d["dv"]["n_c"])
    check = rounding_check(src, cands)
    keep = [c for c in cands if check.get(f"{c[0]}p{c[1]}c", {}).get("fits")] or cands
    dump(f"{case}_int_check", dict(case=case, candidates=cands, check=check, run=keep, git=git_hash()))
    print(f"{case}: rounding candidates {cands}, run {keep}", flush=True)
    return [load(f"{case}_int_{npx}p{nc}c") if _finished(f"{case}_int_{npx}p{nc}c") else
            rdo(f"{case}_int_{npx}p{nc}c", d["case"], maxiter=maxiter, start=case,
                fixed={"n_p": float(npx), "n_c": float(nc)}) for npx, nc in keep]


# ------------------------------------------------------------------------------------------------ the campaign's designs
def clean_runs(prefix, exclude=()):
    """Every finished deterministic run <prefix>*: SLSQP converged, nothing violated, no MDA failure."""
    out = []
    for f in sorted(RUNS.glob(f"{prefix}*.json")):
        if any(x in f.stem for x in exclude):
            continue
        d = json.loads(f.read_text())
        if d.get("ok") and "score" in d and not d.get("violated"):
            out.append(d)
    return sorted(out, key=lambda d: -d["score"]["flight_score"])


def enumeration_tag(cells, m2, m3):
    return f"fr_{cells:g}S_{m2}_{m3}"


M2_PROPS, M3_PROPS = ("18x10E", "19x10E", "19x12E"), ("18x12E", "19x12E", "22x12E")  # [HB Table 27 p. 29] (plan §4.4)


def enumeration_runs():
    """The propulsion-enumeration runs that exist, 8S first."""
    tags = [enumeration_tag(8, m2, m3) for m2 in M2_PROPS for m3 in M3_PROPS]
    tags += [f.stem for f in RUNS.glob("fr_6S_*.json") if "_int_" not in f.stem]
    return [load(t) for t in tags if (RUNS / f"{t}.json").exists()]


def ranked_cases(cells=None):
    """The clean enumeration runs, best F first; cells: only that cell count."""
    runs = [d for d in enumeration_runs() if d.get("ok") and "score" in d and not d.get("violated")]
    runs = [d for d in runs if cells is None or d["cells"] == cells]
    return sorted(runs, key=lambda d: -d["score"]["flight_score"])


def best_integer(case):
    """The payload-rounded result of a deterministic case: the best clean <case>_int_*p*c run."""
    runs = clean_runs(f"{case}_int_", exclude=("_int_check",))
    return runs[0] if runs else None


def rdo_runs(prefix="fr_rdo_"):
    return [json.loads(f.read_text()) for f in sorted(RUNS.glob(f"{prefix}*.json")) if not f.stem.endswith("_int_check")]


def best_rdo(rounded=True):
    """The best converged robust run (highest mu - k sigma): the rounded runs if there are any and rounded is set, else
    the continuous starts."""
    runs = [r for r in rdo_runs() if r.get("ok") and r.get("summary")]
    pool = [r for r in runs if "_int_" in r["tag"]] if rounded else []
    pool = pool or [r for r in runs if "_int_" not in r["tag"]]
    return max(pool, key=lambda r: r["summary"]["flight_score"]["robust"]) if pool else None


def rdo_mcfg(run):
    """A design's config for the robust analyses: its cells, props and scoring with the RDO's parameter list."""
    return config(run["cells"], run["props"]["M2"], run["props"]["M3"], run.get("scoring"), RDO_CFG,
                  None, run.get("old_constraints", False), run.get("takeoff", True))


def humpback_mcfg(mass="cal"):
    """Humpback as built (configs/humpback_2026.yaml, calibrated factors; 'loo': fitted without Humpback), with the
    RDO's uncertain set."""
    sys.path.insert(0, str(ROOT / "studies"))
    from validation_2026 import humpback_config

    from aircrafter2.problems.multipoint import load_multipoint_config

    mcfg = humpback_config(mass)
    mcfg.base.robust = load_multipoint_config(RDO_CFG).base.robust
    return mcfg


def designs(label):
    """A design of the common evaluation: (MultipointConfig with the RDO's parameter list, dv, source tag)."""
    from aircrafter2.problems.multipoint import initial_values as _iv

    if label in ("humpback", "humpback_loo"):
        mcfg = humpback_mcfg("cal" if label == "humpback" else "loo")
        return mcfg, _iv(mcfg), "configs/humpback_2026.yaml"
    if label in ("ac1_direct", "ac1_completed"):
        run = load(f"fr_{label}")
    elif label == "mdo":
        run = best_integer(ranked_cases()[0]["tag"])
    elif label == "mdo_wind":
        run = best_integer("fr_wind")
    elif label == "rdo":
        run = best_rdo()
    else:
        raise ValueError(label)
    return rdo_mcfg(run), run["dv"], run["tag"]


def analyse(mcfg, dv, inputs=None):
    """The problem at a design through the warm path (RetryNewton's robust mode). inputs = {uncertain parameter: value},
    at both mission points."""
    from aircrafter2.problems.multipoint import build_multipoint_problem, run_warm
    from aircrafter2.robustness.rdo_multipoint import param_inputs

    p = build_multipoint_problem(mcfg, driver=False, mda_retry=True)
    run_warm(p, mcfg, dv, run=False)
    for name, val in (inputs or {}).items():
        for inp in param_inputs(name):
            p.set_val(inp, val)
    p.run_model()
    return p


def analyse_summary(mcfg, dv, inputs=None):
    """summarise(analyse()) as a dict, with the analysis failure as `error` (never a constraint violation)."""
    from openmdao.core.analysis_error import AnalysisError

    try:
        p = analyse(mcfg, dv, inputs)
    except (AnalysisError, RuntimeError) as exc:
        kind = "wind limit (lap_invalid)" if "non-physical lap" in str(exc) else "MDA non-convergence"
        return dict(ok=False, error=f"{kind}: {str(exc)[:200]}")
    return dict(ok=True, error=None, mda_fallbacks=mda_fallbacks(p), **summarise(p, mcfg))


def common(label):
    """The common evaluation at nominal inputs (calm air, every factor at its config value) and at the set's means (the
    mean wind): scores, constraints with margins, the M1 check, MDA fallbacks, geometry, mass and drag."""
    mcfg, dv, source = designs(label)
    t = time.perf_counter()
    out = dict(tag=f"fr_common_{label}", label=label, source=source, nominal=analyse_summary(mcfg, dv),
               mean_wind=analyse_summary(mcfg, dv, MEAN_WIND), props={pt: mcfg.mp.points[pt].prop for pt in ("M2", "M3")},
               cells=mcfg.base.propulsion.n_cells, time=time.perf_counter() - t, git=git_hash(),
               threads=os.environ.get("OMP_NUM_THREADS"))
    dump(out["tag"], out)
    sc = out["nominal"].get("score", {})
    print(f"common {label}: F={sc.get('flight_score')} M2={sc.get('M2')} M3={sc.get('M3')} {out['nominal'].get('error')}",
          flush=True)
    return out


def extended_config(mcfg):
    """mcfg with the out-of-set parameters added to its parameter list (uncertain sets: EXTRA_TAILS, NOT_IN_2026)."""
    from aircrafter2.robustness.uncertainty import UncertainParam, for_kv_mode, load_uncertainty

    have = {q.name for q in mcfg.base.robust.parameters}
    extra = [q for q in for_kv_mode(load_uncertainty(ROOT / "configs" / "uncertainty.yaml"), True)
             if q.name not in have and q.name not in NOT_IN_2026]
    extra += [UncertainParam(**e) for e in EXTRA_TAILS]
    mcfg.base.robust.parameters = list(mcfg.base.robust.parameters) + extra
    mcfg.base.robust.active = None
    return mcfg


def _dists(mcfg, dv):
    """(all distributions, active) resolved at the design's config values (the relative spreads' nominals)."""
    from aircrafter2.problems.multipoint import build_multipoint_problem
    from aircrafter2.robustness.rdo_multipoint import resolved_dists_mp

    p = build_multipoint_problem(mcfg, driver=False)
    return resolved_dists_mp(mcfg, lambda nm: v(p, f"points.M2.{nm}"))


def sobol_rows(act, n=MC_N, seed=0):
    """(u, rows): n scrambled Sobol' points over the active distributions (common random numbers: the same u for every
    design with the same parameter order)."""
    from scipy.stats import qmc

    u = qmc.Sobol(len(act), scramble=True, seed=seed).random(n)
    return u, [{d.name: float(d.ppf(ui)) for d, ui in zip(act, row)} for row in u]


def any_violation(values, outputs, constraints, rtol=1e-6):
    """P(any constraint violated) over the converged rows; the same tolerance as sampling.violation_probabilities."""
    idx = {o: j for j, o in enumerate(outputs)}
    ok = ~np.isnan(values).any(axis=1)
    bad = np.zeros(int(ok.sum()), bool)
    for name, kw in constraints:
        if name not in idx:
            continue
        col = values[ok, idx[name]]
        for side, sgn in (("lower", -1.0), ("upper", 1.0)):
            bound = kw.get(side)
            if bound is not None:
                tol = rtol * max(abs(bound), abs(kw.get("ref") or 0.0), 1.0)
                bad |= sgn * (col - bound) > tol
    return float(bad.mean()) if len(bad) else float("nan")


def mc(label, which, workers=3, n=MC_N):
    """The Monte Carlo of one design: `in` = the RDO's five parameters, `out` = the extended list. Analysis failures are
    NaN rows, counted apart (MDA failures, wind limits), never as constraint violations."""
    from aircrafter2.problems.multipoint import multipoint_constraints
    from aircrafter2.robustness.sampling import evaluate, violation_probabilities

    mcfg, dv, source = designs(label)
    if which == "out":
        mcfg = extended_config(mcfg)
    dists, act = _dists(mcfg, dv)
    u, rows = sobol_rows(act, n)
    cons = multipoint_constraints(mcfg)
    outputs = ["flight_score", "M2", "M3", "r_M2", "r_M3", "points.M2.laps_frac", "points.M3.laps_frac"] + \
        [c for c, _ in cons]
    ev = evaluate(mcfg, dv, rows, outputs, workers)
    V = ev["values"]
    MC_DIR.mkdir(parents=True, exist_ok=True)
    tag = f"fr_mc_{which}_{label}"
    np.savez(MC_DIR / f"{tag}.npz", values=V, outputs=np.array(outputs), u=u)
    ok = ~np.isnan(V).any(axis=1)
    stats = {}
    for j, o in enumerate(outputs[:7]):
        col = V[ok, j]
        stats[o] = dict(mu=float(col.mean()), sigma=float(col.std(ddof=1)), p5=float(np.percentile(col, 5)),
                        p95=float(np.percentile(col, 95)), min=float(col.min()), max=float(col.max()))
    k = mcfg.base.robust.k
    f = stats["flight_score"]
    f["robust"] = f["mu"] - k * f["sigma"]
    pv = violation_probabilities(V, outputs, cons)
    out = dict(tag=tag, label=label, set=which, source=source, N=n, converged=int(ok.sum()),
               mda_failed=len(ev["failed"]) - len(ev["lap_invalid"]), wind_limit=len(ev["lap_invalid"]),
               retried=len(ev["retried"]), wall=ev["wall"], workers=workers, stats=stats, p_violation=pv,
               p_any=any_violation(V, outputs, cons), params=[dict(name=d.name, mean=d.mean, std=d.std, kind=d.kind)
                                                              for d in act],
               sources={q.name: q.source for q in mcfg.base.robust.parameters}, k=k,
               dv=dv, git=git_hash(), threads=os.environ.get("OMP_NUM_THREADS"), peak_rss_gb=peak_rss_gb())
    dump(tag, out)
    print(f"MC {tag}: conv {out['converged']}/{n}, mda {out['mda_failed']}, wind {out['wind_limit']}, mu {f['mu']:.4f} "
          f"sigma {f['sigma']:.4f}, P(any) {out['p_any']:.3f}, {ev['wall']:.0f} s", flush=True)
    return out


def propagation(workers=3):
    """UDR (the RDO's estimate), Smolyak level 2 and the in-set Sobol' Monte Carlo at the robust optimum, for the mean and
    sigma of F and the robust constraints."""
    from aircrafter2.robustness.quadrature import make_rule
    from aircrafter2.robustness.rdo_multipoint import robust_names
    from aircrafter2.robustness.sampling import evaluate

    mcfg, dv, source = designs("rdo")
    rob = mcfg.base.robust
    _, act = _dists(mcfg, dv)
    names = ["flight_score"] + robust_names(mcfg)
    out = dict(tag="fr_propagation", label="rdo", source=source, git=git_hash(), params=[d.name for d in act], methods={},
               k=rob.k, beta=rob.beta, threads=os.environ.get("OMP_NUM_THREADS"))
    for meth in ("udr", "smolyak"):
        rule = make_rule(rob.model_copy(update={"quadrature": meth}), act)
        rows = [{d.name: float(x) for d, x in zip(act, row)} for row in rule.nodes]
        ev = evaluate(mcfg, dv, rows, names, workers)
        res = dict(N=rule.n, failed=len(ev["failed"]), wind_limit=len(ev["lap_invalid"]), wall=ev["wall"], stats={},
                   psd=bool(rule.psd), rows=rows, values=ev["values"])
        if not ev["failed"]:
            for j, o in enumerate(names):
                mu, var = rule.moments(ev["values"][:, j])
                res["stats"][o] = dict(mu=float(mu), sigma=float(np.sqrt(max(var, 0.0))))
        out["methods"][meth] = res
        print(f"propagation {meth}: N {rule.n}, failed {len(ev['failed'])}, {ev['wall']:.0f} s", flush=True)
    f = MC_DIR / "fr_mc_in_rdo.npz"
    if f.exists():
        z = np.load(f)
        out_names = [str(x) for x in z["outputs"]]
        V = z["values"]
        ok = ~np.isnan(V).any(axis=1)
        st = {o: dict(mu=float(V[ok, out_names.index(o)].mean()), sigma=float(V[ok, out_names.index(o)].std(ddof=1)))
              for o in names if o in out_names}
        # sampling error of the Sobol' estimates: the plain Monte Carlo bound, conservative for a low-discrepancy set
        n_ok = int(ok.sum())
        for o, x in st.items():
            x["se_mu"] = x["sigma"] / np.sqrt(n_ok)
            x["se_sigma"] = x["sigma"] / np.sqrt(2.0 * (n_ok - 1))
        out["methods"]["sobol"] = dict(N=int(len(V)), converged=n_ok, stats=st)
    dump(out["tag"], out)
    return out


def wind_dir(label, workers=3):
    """F against the wind direction at the mean wind speed (DIRECTIONS)."""
    from aircrafter2.robustness.sampling import evaluate

    mcfg, dv, source = designs(label)
    names = ["flight_score", "M2", "M3", "r_M2", "r_M3", "points.M2.laps_frac", "points.M3.laps_frac"]
    rows = [{"lap.wind": MEAN_WIND["lap.wind"], "lap.wind_dir": th} for th in DIRECTIONS]
    ev = evaluate(mcfg, dv, rows, names, workers)
    out = dict(tag=f"fr_wind_dir_{label}", label=label, source=source, wind=MEAN_WIND["lap.wind"], directions=DIRECTIONS,
               outputs=names, values=ev["values"], failed=ev["failed"], wind_limit=ev["lap_invalid"], wall=ev["wall"],
               git=git_hash(), threads=os.environ.get("OMP_NUM_THREADS"))
    dump(out["tag"], out)
    print(f"wind dir {label}: F {np.round(ev['values'][:, 0], 4)}, failed {ev['failed']}", flush=True)
    return out


def wind_limit_at(run_at, theta, tol=0.05, hi_max=40.0):
    """The wind limit at direction theta by bisection: the largest speed [m/s] whose analysis succeeds (within tol), and
    what stops it. run_at(wind, theta) -> how ('warm', 'nominal', 'lap_invalid' or 'failed'); the bracket is grown from
    10 m/s in 5 m/s steps."""
    lo, hi, kind, calls = 0.0, None, None, 0
    w = 10.0
    while w <= hi_max:
        how = run_at(w, theta)
        calls += 1
        if how in ("lap_invalid", "failed"):
            hi, kind = w, how
            break
        lo = w
        w += 5.0
    if hi is None:
        return dict(theta=theta, limit=None, above=lo, stopped_by=None, calls=calls)
    while hi - lo > tol:
        mid = 0.5 * (lo + hi)
        how = run_at(mid, theta)
        calls += 1
        if how in ("lap_invalid", "failed"):
            hi, kind = mid, how
        else:
            lo = mid
    return dict(theta=theta, limit=lo, stopped_by=kind, calls=calls)


def wind_env(label):
    """The wind-limit envelope: the wind limit against direction, 9 directions, by bisection (one process, warm)."""
    from aircrafter2.robustness import sampling

    mcfg, dv, source = designs(label)
    t = time.perf_counter()
    sampling._init(mcfg, dv, ["flight_score"])
    if sampling._W.get("prob") is None:
        out = dict(tag=f"fr_wind_env_{label}", label=label, error="the nominal point did not converge", git=git_hash())
        dump(out["tag"], out)
        return out

    def run_at(wind, theta):
        return sampling._run((0, {"lap.wind": wind, "lap.wind_dir": theta}))[2]

    rows = []
    for th in DIRECTIONS:
        sampling._restore_nominal()
        rows.append(wind_limit_at(run_at, th))
        print(f"wind envelope {label}: {rows[-1]}", flush=True)
    out = dict(tag=f"fr_wind_env_{label}", label=label, source=source, rows=rows, time=time.perf_counter() - t,
               git=git_hash(), threads=os.environ.get("OMP_NUM_THREADS"))
    dump(out["tag"], out)
    return out


def screen(label):
    """Each parameter's contribution to the variance of F and the robust constraints, over the extended list (f_tb
    included), at the set's means: first order, (dq/dp sigma)^2 from compute_totals, and the 3-point Gauss variance of
    each parameter alone (the UDR's per-dimension term, which also sees the cap and the curvature). With r_M2 inside the
    cap's blend, the first-order M2 terms are damped by the cap: that is the score the design is judged by."""
    from aircrafter2.problems.multipoint import build_multipoint_problem, run_warm
    from aircrafter2.robustness.quadrature import gauss_1d
    from aircrafter2.robustness.rdo_multipoint import first_order_contributions_mp, param_inputs, robust_names
    from aircrafter2.robustness.sampling import evaluate
    from aircrafter2.robustness.uncertainty import select_parameters

    mcfg, dv, source = designs(label)
    active = list(mcfg.base.robust.active)
    mcfg = extended_config(mcfg)
    dists, _ = _dists(mcfg, dv)
    p = build_multipoint_problem(mcfg, driver=False, mda_retry=True)
    run_warm(p, mcfg, dv, run=False)
    for d in dists:
        for inp in param_inputs(d.name):
            p.set_val(inp, d.mean)
    p.run_model()
    of = ["flight_score", "M2", "M3"] + robust_names(mcfg)
    contrib = first_order_contributions_mp(p, dists, of)
    rob = mcfg.base.robust
    sel, cov, reached = select_parameters({k: x for k, x in contrib.items() if k not in ("M2", "M3")},
                                          rob.screening_threshold, rob.max_dim)
    base = {d.name: d.mean for d in dists}
    rows, index = [dict(base)], {}
    for d in dists:
        xi, w = gauss_1d(d.kind, 3)
        index[d.name] = (len(rows), [float(x) for x in w], [float(x) for x in xi])
        rows += [dict(base, **{d.name: float(d.mean + d.scale * x)}) for x in xi]
    ev = evaluate(mcfg, dv, rows, ["flight_score", "M2", "M3"], 3)
    oned = {}
    for name, (i0, w, _) in index.items():
        col = ev["values"][i0:i0 + 3, 0]
        if np.isnan(col).any():
            oned[name] = None
        else:
            mean = float(np.dot(w, col))
            oned[name] = float(np.dot(w, (col - mean) ** 2))
    out = dict(tag=f"fr_screen_{label}", label=label, source=source, active=active, selected=sel, coverage=cov,
               reached=reached, threshold=rob.screening_threshold, max_dim=rob.max_dim, contrib=contrib, oned=oned,
               values={q: v(p, q) for q in of}, means=base, std={d.name: d.std for d in dists},
               sources={q.name: q.source for q in rob.parameters}, oned_failed=ev["failed"], git=git_hash(),
               threads=os.environ.get("OMP_NUM_THREADS"))
    dump(out["tag"], out)
    print(f"screen {label}: selected {sel}, reached {reached}", flush=True)
    return out


def box_designs(mcfg, n, seed=0):
    """n scrambled Sobol' designs in the design box. The payload counts are drawn inside what the fuselage cap allows at
    the largest span (the config's 500 / 200 count bounds are nominal, 'no bound in practice', Q55): a start the cap
    rules out at every span would only test the cap."""
    from scipy.stats import qmc

    dvs = mcfg.all_dvs()
    names = list(dvs)
    span = min(dvs["b"].upper, mcfg.mp.rules.span_max_ft * 0.3048)
    l_max = mcfg.mp.rules.fuselage_span_ratio_max * span
    up = {k: b.upper for k, b in dvs.items()}
    up["b"] = span
    up["n_p"] = l_max / mcfg.mp.stores2026.pax_pitch
    up["n_c"] = up["n_p"] / mcfg.mp.rules.pax_per_cargo_min
    u = qmc.Sobol(len(names), scramble=True, seed=seed).random(n)
    return [{k: dvs[k].lower + ui * (up[k] - dvs[k].lower) for k, ui in zip(names, row)} for row in u]


def multistart(case, index, n=10):
    """Start `index` of n scrambled Sobol' designs (seed 0) in the design box, the same case."""
    d = load(case)
    x0 = box_designs(config(d["cells"], d["props"]["M2"], d["props"]["M3"]), n)[index]
    return optimise(f"fr_ms{index}", d["cells"], d["props"]["M2"], d["props"]["M3"], x0=x0)


def node_check(workers=3):
    """Every UDR node of the set at the optimum, the config's start and 6 Sobol' designs (both points)."""
    from aircrafter2.problems.multipoint import initial_values as _iv
    from aircrafter2.robustness.quadrature import make_rule
    from aircrafter2.robustness.sampling import evaluate

    mcfg, dv, source = designs("rdo")
    _, act = _dists(mcfg, dv)
    rule = make_rule(mcfg.base.robust, act)
    rows = [{d.name: float(x) for d, x in zip(act, r)} for r in rule.nodes]
    res = []
    for lab, x in [("rdo optimum", dv), ("start (Humpback)", _iv(mcfg))] + [(f"sobol{i}", d) for i, d in
                                                                           enumerate(box_designs(mcfg, 6))]:
        ev = evaluate(mcfg, x, rows, ["flight_score"], workers)
        res.append(dict(design=lab, failed=ev["failed"], wind_limit=ev["lap_invalid"], retried=ev["retried"], wall=ev["wall"]))
        print(f"node check {lab}: failed {ev['failed']} (wind limit {ev['lap_invalid']}) {ev['wall']:.0f} s", flush=True)
    out = dict(tag="fr_node_check", source=source, N=rule.n, d=len(act), params=[d.name for d in act], results=res,
               git=git_hash(), threads=os.environ.get("OMP_NUM_THREADS"))
    dump(out["tag"], out)
    return out


def reanalysis(label):
    """Re-analyses at a design (no re-optimisation): the banner C_D,b cases and the rpm limit at 0.85."""
    from aircrafter2.stores2026 import Store2026Config  # noqa: F401 (documents where cd_banner lives)

    mcfg, dv, source = designs(label)
    cd0 = mcfg.mp.stores2026.cd_banner
    cases = {}
    for name, cd in CD_BANNER_CASES.items():
        cases[name] = dict(cd_banner=cd, factor=cd / cd0, **_pick(analyse_summary(mcfg, dv, {"banner_drag_factor": cd / cd0})))
        # the pilot's answer to more drag: the same design at full M3 throttle (the level speed back above the stall margin)
        cases[f"{name}, M3 full throttle"] = dict(cd_banner=cd, factor=cd / cd0, throttle_M3=1.0, **_pick(
            analyse_summary(mcfg, {**dv, "throttle_M3": 1.0}, {"banner_drag_factor": cd / cd0})))
    m85, dv85, _ = designs(label)
    m85.base.propulsion.rpm_fraction = 0.85
    cases["rpm limit 0.85"] = dict(rpm_fraction=0.85, **_pick(analyse_summary(m85, dv85)))
    out = dict(tag=f"fr_reanalysis_{label}", label=label, source=source, cd_nominal=cd0, cases=cases, git=git_hash(),
               threads=os.environ.get("OMP_NUM_THREADS"))
    dump(out["tag"], out)
    for k, x in cases.items():
        print(f"reanalysis {label} {k}: F {x.get('F')} violated {x.get('violated')}", flush=True)
    return out


def _pick(s):
    """The short form of an analyse_summary for the sensitivity tables."""
    if not s.get("ok"):
        return dict(ok=False, error=s.get("error"))
    sc = s["score"]
    return dict(ok=True, F=sc["flight_score"], M2=sc["M2"], M3=sc["M3"], r_M2=sc["r_M2"], r_M3=sc["r_M3"],
                F_uncapped=sc["M1"] + 1.0 + sc["r_M2"] + 2.0 + sc["r_M3"], floored=s["floored"]["flight_score"],
                laps_M2=s["M2"]["laps_frac"], laps_M3=s["M3"]["laps_frac"], violated=s["violated"],
                rpm_margin_M2=s["M2"]["rpm_margin"], rpm_margin_M3=s["M3"]["rpm_margin"])


# The placeholders that act on the 2025-26 analysis, each with its plausible range (a source or the assumption, stated on
# the page). Layer-file status PLACEHOLDER, or cited with a PLACEHOLDER scaling (Q63). Re-analysis at a fixed design.
PLACEHOLDER_RANGES = [
    ("Q23 tail dynamic-pressure ratio eta_h", "stability.eta_h", 0.81, 0.99, "±10 % (configs/uncertainty.yaml, Q32)"),
    ("Q23 wing CG / chord", "stability.wing_cg_frac", 0.35, 0.45, "±0.05 c, assumed"),
    ("Q23 avionics station", "stability.x_avionics", -0.20, 0.0, "±0.10 m, assumed"),
    ("Q23 gear station", "stability.x_gear", 0.0, 0.10, "±0.05 m, assumed"),
    ("Q8 cell resistance", "propulsion.R_cell", 0.0035, 0.0065, "±30 % (configs/uncertainty.yaml, Q32)"),
    ("Q8 cell nominal voltage", "propulsion.V_cell", 3.6, 3.85, "LiPo 3.6-3.85 V, assumed"),
    ("Q16 n_crit", "aero.n_crit", 7.0, 11.0, "XFoil: 9 average wind tunnel; ±2, assumed"),
    ("Q19 tail boom radius", "mass.inputs.r_tb", 0.006, 0.010, "12-20 mm tube, assumed"),
    ("Q19 boom start offset", "geometry.boom_offset", 0.10, 0.20, "±0.05 m, assumed"),
    ("Q4 ground-test hold point x_gd", "mass.inputs.x_gd", 0.20, 0.30, "±0.05 c, assumed"),
    ("Q5 tube clearance", "mass.inputs.clearance", 0.0005, 0.002, "×0.5 / ×2, assumed"),
    ("Q6 boom density", "mass.inputs.rho_tb", 1280.0, 1920.0, "±20 %, assumed"),
    ("Q6 boom strength", "mass.inputs.sigma_tb", 5.76e8, 8.64e8, "±20 %, assumed"),
    ("Q6 boom modulus", "mass.inputs.E_tb", 6.72e10, 1.008e11, "±20 %, assumed"),
    ("Q12 tip-deflection ratio", "mass.inputs.delta_tip_ratio", 0.05, 0.15, "±50 %, assumed"),
    ("Q61 avionics mass", "propulsion.m_avionics", 0.1887, 0.2553, "±15 % (Q32)"),
    ("no Q pilot reaction time per lap", "course.pilot_reaction_time_s", 1.0, 5.0, "±2 s, assumed"),
    ("Q41 lift-off factor k_lof", "course.takeoff.k_lof", 1.05, 1.20, "1.2 = Eq. 3.3's rotation speed (Q41)"),
    ("Q40 ground-roll C_L", "course.takeoff.CL_ground", 0.0, 1.0, "Q40's study range"),
    ("Q43 start offset (ft)", "course.takeoff.start_offset_ft", 0.0, 50.0, "0-50 ft, assumed"),
    ("Q54 restraint per seat (kg)", "multipoint.stores2026.m_seat", 0.0, 0.010, "×0 / ×2, assumed"),
    ("Q54 restraint per slot (kg)", "multipoint.stores2026.m_slot", 0.0, 0.020, "×0 / ×2, assumed"),
    ("Q55 passenger pitch (m)", "multipoint.stores2026.pax_pitch", 0.0202, 0.0260,
     "33.4 in box ([HB] Fig. 9) to +10 %"),
    ("Q55 cargo pitch (m)", "multipoint.stores2026.car_pitch", 0.0384, 0.0469, "±10 %, assumed"),
    ("Q55 fuselage width (m)", "multipoint.stores2026.w_cb", 0.178, 0.218, "±10 %, assumed"),
    ("Q55 fuselage height (m)", "multipoint.stores2026.h_cb", 0.126, 0.154, "±10 %, assumed"),
    ("Q63 gear drag area (m²)", "aero.cd_area_gear", 0.0014, 0.0056, "×0.5 / ×2, scaling PLACEHOLDER"),
    ("Q63 leakage drag area (m²)", "aero.cd_area_lp", 0.00112, 0.00448, "×0.5 / ×2, scaling PLACEHOLDER"),
    ("Q56 stowed banner ΔC_D0", "multipoint.stores2026.dcd_stowed", 0.002, 0.008, "×0.5 / ×2, assumed"),
    ("Q56 banner mechanism (kg)", "multipoint.stores2026.m_banner_mech", 0.038, 0.153, "×0.5 / ×2, assumed"),
]


def _placeholder_job(job):
    """One placeholder at one end of its range, at a fixed design; an override the problem can't take is an error row."""
    name, path, side, val, label = job
    warnings.filterwarnings("ignore")
    mcfg, dv, _ = designs(label)
    t = time.perf_counter()
    try:
        set_path(mcfg, path, val)
        res = _pick(analyse_summary(mcfg, dv))
    except Exception as exc:  # noqa: BLE001 - an unusable override is reported per row
        res = dict(ok=False, error=f"{type(exc).__name__}: {str(exc)[:200]}")
    return dict(name=name, path=path, side=side, value=val, wall=time.perf_counter() - t, **res)


def placeholders(label="mdo", workers=3):
    """Re-analysis of every placeholder of PLACEHOLDER_RANGES at the design, at both ends of its range."""
    from multiprocessing import get_context

    mcfg, dv, source = designs(label)
    nominal = _pick(analyse_summary(mcfg, dv))
    jobs = [(n, path, side, val, label) for n, path, lo, hi, _ in PLACEHOLDER_RANGES
            for side, val in (("lo", lo), ("hi", hi))]
    with get_context("forkserver").Pool(workers) as pool:
        rows = list(pool.imap(_placeholder_job, jobs))
    out = dict(tag=f"fr_placeholders_{label}", label=label, source=source, nominal=nominal, rows=rows,
               ranges=[dict(name=n, path=path, lo=lo, hi=hi, why=why) for n, path, lo, hi, why in PLACEHOLDER_RANGES],
               nominal_values={path: _value_at(mcfg, path) for _, path, *_ in PLACEHOLDER_RANGES},
               git=git_hash(), threads=os.environ.get("OMP_NUM_THREADS"))
    dump(out["tag"], out)
    bad = [r["name"] for r in rows if not r.get("ok")]
    print(f"placeholders {label}: {len(rows)} rows, failed {bad}", flush=True)
    return out


def _value_at(mcfg, path):
    parts = path.split(".")
    obj = mcfg.mp if parts[0] == "multipoint" else mcfg.base
    for k in parts[1:] if parts[0] == "multipoint" else parts:
        obj = obj[k] if isinstance(obj, dict) else getattr(obj, k)
    return float(obj)


def _pairs(items, cast=float):
    """name=value strings to a dict."""
    return {x.split("=")[0]: cast(x.split("=")[1]) for x in items}


def _bounds(items):
    """c.upper=0.6 strings to {c: {upper: 0.6}}."""
    out = {}
    for x in items:
        key, val = x.split("=")
        name, side = key.rsplit(".", 1)
        out.setdefault(name, {})[side] = float(val)
    return out


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("part", choices=["optimise", "integer", "rdo", "rdo_integer", "multistart", "common", "mc", "propagation",
                                     "wind_dir", "wind_env", "screen", "node_check", "reanalysis", "placeholders"])
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
    ap.add_argument("--mean-wind", action="store_true", help="fly both points at the uncertain set's mean wind")
    ap.add_argument("--fix", nargs="*", default=[], help="name=value pins")
    ap.add_argument("--override", nargs="*", default=[], help="dotted.config.path=value")
    ap.add_argument("--bound", nargs="*", default=[], help="dv.upper=value or dv.lower=value")
    ap.add_argument("--free", action="store_true", help="rdo: the payload counts stay design variables")
    ap.add_argument("--index", type=int)
    ap.add_argument("--label")
    ap.add_argument("--set", choices=["in", "out"], default="in")
    ap.add_argument("--workers", type=int, default=3)
    a = ap.parse_args()
    if a.part == "optimise":
        sc = {k: val for k, val in (("w2", a.w2), ("w3", a.w3), ("NI_max", a.NI), ("B_max", a.B)) if val is not None}
        optimise(a.tag, a.cells, a.m2, a.m3, sc or None, a.start, maxiter=a.maxiter, fixed=_pairs(a.fix) or None,
                 old_constraints=a.old_constraints, takeoff=not a.takeoff_off, wind=MEAN_WIND if a.mean_wind else None,
                 overrides=_pairs(a.override) or None, bounds=_bounds(a.bound) or None)
    elif a.part == "integer":
        integer(a.case, maxiter=a.maxiter)
    elif a.part == "rdo":
        rdo(a.tag, a.case, maxiter=a.maxiter, start=a.start, free=a.free)
    elif a.part == "rdo_integer":
        rdo_integer(a.case, maxiter=a.maxiter or 15)
    elif a.part == "multistart":
        multistart(a.case, a.index)
    elif a.part == "common":
        common(a.label)
    elif a.part == "mc":
        mc(a.label, a.set, a.workers)
    elif a.part == "propagation":
        propagation(a.workers)
    elif a.part == "wind_dir":
        wind_dir(a.label, a.workers)
    elif a.part == "wind_env":
        wind_env(a.label)
    elif a.part == "screen":
        screen(a.label)
    elif a.part == "node_check":
        node_check(a.workers)
    elif a.part == "reanalysis":
        reanalysis(a.label)
    elif a.part == "placeholders":
        placeholders(a.label or "mdo", a.workers)
