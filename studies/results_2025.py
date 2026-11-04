"""2024-25 results: multipoint optimisation, bookends, enumeration, RDO, common evaluation
(docs/studies/results/results_2025.md). Each run writes outputs/results/runs/<tag>.json.

    .venv/bin/python studies/results_2025.py optimise --tag mp_6S_10x7E_10x7E [--cells 6] [--m2 10x7E] [--m3 10x7E]
                                                       [--w2 1 --w3 1] [--R2 0.2 --R3 27] [--start TAG]
    .venv/bin/python studies/results_2025.py bookends --case mp_6S_10x7E_10x7E      # M2-only and M3-only optima
    .venv/bin/python studies/results_2025.py screen | node_check | rdo --tag T [--index i] | multistart --case C --index i
    .venv/bin/python studies/results_2025.py normalisation --case C --f2 2 --f3 1
    .venv/bin/python studies/results_2025.py mc | common | robust_eval    # common evaluation, UDR at every design
    .venv/bin/python studies/results_2025.py report                        # results_2025.{md,json} + figures

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
LB_ = 0.45359237
CFG = ROOT / "configs" / "dbf2025_multipoint.yaml"
WOOMBAH_CFG = ROOT / "configs" / "woombah_2025.yaml"
AC1_JSON = ROOT / "docs" / "studies" / "results" / "aircrafter1_2025.json"  # the reused AirCrafter 1.0 run
BATCH = 240  # Monte Carlo samples per pool of fresh workers: no worker gets near the ~250-evaluation solver-state defect
POINT_OUT = ["m_total", "m_empty", "m_pay", "m_wing", "m_ht", "m_vt", "m_fus", "m_gear", "m_sys", "m_batt", "v",
             "ThreeLapTime", "laps_frac", "I_static", "I_pack_static", "rpm_margin", "I_margin", "SM", "CD0",
             "CD_stores", "energy_margin", "E_nom_total", "thrust_margin_static", "v_fit_margin", "stall_margin",
             "cruise.L_over_D", "Rm", "I0"]


def v(p, k):
    return float(np.ravel(p.get_val(k))[0])


def git_hash():
    """Short HEAD commit, with '-dirty' when tracked files differ from it."""
    head = subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=ROOT, capture_output=True,
                          text=True).stdout.strip()
    dirty = subprocess.run(["git", "diff", "--quiet", "HEAD"], cwd=ROOT).returncode != 0
    return head + ("-dirty" if dirty else "")


def provenance():
    """The commit, the clean-tree flag and the model package this process imported (the venv's editable install points
    at another checkout, so a run that imports that one must be visible)."""
    import aircrafter2

    g = git_hash()
    return dict(git=g, git_clean=not g.endswith("-dirty"), model_path=str(Path(aircrafter2.__file__).parent))


def use_paths(out=None, config=None, rdo_config=None, woombah_config=None, docs=None):
    """Point the study at another output folder and other configs (the re-run at the final code: RERUN_2025_LOG.md)."""
    global OUT, RUNS, CFG, DOCS, WOOMBAH_CFG
    if out:
        OUT = ROOT / out
        RUNS = OUT / "runs"
    if config:
        CFG = ROOT / config
    if woombah_config:
        WOOMBAH_CFG = ROOT / woombah_config
    if docs:
        DOCS = ROOT / docs
    if rdo_config:
        UQ_SETS["final"] = dict(name="final", cfg=ROOT / rdo_config, out=OUT / "reduced_final", prefix="rdof_", select=False)


def dump(tag, data):
    RUNS.mkdir(parents=True, exist_ok=True)
    (RUNS / f"{tag}.json").write_text(json.dumps({**data, **provenance()}, indent=1,
                                                 default=lambda o: o.item() if hasattr(o, "item") else o.tolist()))


def peak_rss_gb():
    import resource

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


def _kind(exc):
    return "wind limit (lap_invalid)" if "non-physical lap" in str(exc) else "MDA non-convergence"


def load(tag):
    return json.loads((RUNS / f"{tag}.json").read_text())


def config(cells=None, m2=None, m3=None, scoring=None, cfg=None, fixed=None, bottle=None):
    """The multipoint config with overrides; fixed = {design variable: value} pins DVs (equal bounds);
    bottle = a key of stores.BOTTLES for the external tanks."""
    from aircrafter2.problems.multipoint import load_multipoint_config
    from aircrafter2.problems.wing_aero import Bounds
    from aircrafter2.stores import BOTTLES, Bottle

    mcfg = load_multipoint_config(cfg or CFG)
    if bottle:
        mcfg.mp.stores.bottle = Bottle(**BOTTLES[bottle])
    for name, val in (fixed or {}).items():
        if name == "fuel_mass":
            mcfg.mp.fuel = Bounds(lower=val, upper=val)
            mcfg.mp.fuel_initial = val
        elif name in mcfg.base.design_variables:
            mcfg.base.design_variables[name] = Bounds(lower=val, upper=val)
            mcfg.base.initial[name] = val
        else:
            raise ValueError(f"cannot fix {name!r}: not a shared design variable or fuel_mass")
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


def optimise(tag, cells=None, m2=None, m3=None, scoring=None, start=None, x0=None, cfg=None, maxiter=None, fixed=None,
             bottle=None):
    from openmdao.core.analysis_error import AnalysisError

    from aircrafter2.problems.multipoint import build_multipoint_problem, run_warm

    cfg = cfg or CFG
    mcfg = config(cells, m2, m3, scoring, cfg, fixed, bottle)
    if maxiter:
        mcfg.base.maxiter = maxiter
    p = build_multipoint_problem(mcfg, driver=True, mda_retry=True)  # warm-start fallbacks (RDO's RetryNewton)
    p.driver.options["disp"] = False
    warm = bool(mcfg.base.mda_robust and (start or x0))
    t = time.perf_counter()
    err = None
    if warm:  # as results_2026.optimise: converge at the config's (unfixed) start, so the first driver evaluation is warm
        try:
            run_warm(p, config(cells, m2, m3, scoring, cfg, None, bottle), {}, run=False)
        except AnalysisError as exc:
            err = f"warm-path pre-run: {_kind(exc)}: {str(exc)[:200]}"
    if start:
        for k, x in load(start)["dv"].items():
            if not (fixed and k in fixed):
                p.set_val(k, x)
    for k, x in (x0 or {}).items():
        p.set_val(k, x)
    for k, x in (fixed or {}).items():
        p.set_val(k, x)
    ok, its = False, 0
    if err is None:
        try:
            res = p.run_driver()
            ok, its = bool(res.success), int(res.iter_count)
        except AnalysisError as exc:
            ok, its, err = False, int(p.driver.iter_count), f"{_kind(exc)}: {str(exc)[:200]}"
    wall = time.perf_counter() - t
    out = dict(tag=tag, ok=ok, its=its, error=err, time=wall, start=start, x0=x0, git=git_hash(), maxiter=mcfg.base.maxiter,
               fixed=fixed, bottle=bottle or "1L", warm_path=warm, mda_fallbacks=mda_fallbacks(p), peak_rss_gb=peak_rss_gb(),
               threads=os.environ.get("OMP_NUM_THREADS"), config=str(Path(cfg).relative_to(ROOT)))
    if err is None:
        out.update(summarise(p, mcfg))
    dump(tag, out)
    sc = out.get("score", {})
    print(f"{tag}: ok={ok} its={its} {wall:.0f}s F={sc.get('flight_score', float('nan')):.4f} "
          f"M2={sc.get('M2', float('nan')):.4f} M3={sc.get('M3', float('nan')):.4f} {err or ''}", flush=True)
    return out


RDO_CFG = ROOT / "configs" / "dbf2025_multipoint_rdo.yaml"  # the reduced mass-and-wind set (docs/RDO_REDUCED_PLAN.md)
RDO_FULL_CFG = ROOT / "configs" / "dbf2025_multipoint_rdo_full.yaml"  # the published 15-parameter set
# The uncertain set the robust parts (screen, node_check, rdo, mc, robust_eval, propagation) work on (--set): the full
# set's outputs stay where they were published (outputs/results/, runs rdo_*), the reduced set's go to
# outputs/results/reduced/ (runs rdor_*).
UQ_SETS = {"full": dict(name="full", cfg=RDO_FULL_CFG, out=OUT, prefix="rdo_", select=True),
           "reduced": dict(name="reduced", cfg=RDO_CFG, out=OUT / "reduced", prefix="rdor_", select=False),
           # f_build one-sided on [1.0, 1.3] (sensitivity) and f_build calibrated on Woombah (calibrate())
           "onesided": dict(name="onesided", cfg=ROOT / "configs" / "dbf2025_multipoint_rdo_onesided.yaml",
                            out=OUT / "reduced_onesided", prefix="rdoro_", select=False),
           "calibrated": dict(name="calibrated", cfg=ROOT / "configs" / "dbf2025_multipoint_rdo_calibrated.yaml",
                              out=OUT / "reduced_calibrated", prefix="rdorc_", select=False),
           # the reduced set on the USYD-calibrated mass model (docs/studies/mass_calibration_usyd.md), from the
           # calibrated deterministic optimum (--case mpcal_6S_11x8E_10x7E); the published rdor_* runs stay as they are
           "usyd": dict(name="usyd", cfg=RDO_CFG, out=OUT / "reduced_usyd", prefix="rdoru_", select=False),
           # the full 15-parameter set on the calibrated model (gg tails, the config's pin; screened as published)
           "full_usyd": dict(name="full_usyd", cfg=RDO_FULL_CFG, out=OUT / "full_usyd", prefix="rdofu_", select=True)}
CALIBRATED_SETS = ("usyd", "full_usyd")  # their common evaluation: the calibrated optima only (mc_designs)
UQ = dict(UQ_SETS["reduced"])


def use_set(name):
    UQ.clear()
    UQ.update(UQ_SETS[name])
    UQ["out"].mkdir(parents=True, exist_ok=True)
MC_N = 1024
# the full 15-parameter set on the calibrated model: its RDO, Monte Carlo and common evaluation
FULL_NOTE = "docs/studies/mass_calibration_usyd.md §11–12"


def calibrate():
    """f_build such that Woombah as built (configs/woombah_2025.yaml at the logged-current design) has Table 34's
    structure, solved through the closure (the heavier structure resizes the spar and the gear): wing + HT + VT +
    fuselage and nose + undercarriage = 6.05 lb [W Table 34 p. 45]. Writes configs/uncertainty_2025_reduced_calibrated.yaml
    (f_build centred on it, +/-30 % about it) and outputs/results/reduced_calibrated/calibration.json."""
    from scipy.optimize import brentq

    from aircrafter2.problems.multipoint import build_multipoint_problem, load_multipoint_config

    table34 = {"wing": 3.41, "HT": 0.41, "VT": 0.19, "fuselage + nose": 1.23 + 0.15, "undercarriage": 0.66}  # lb
    target = sum(table34.values()) * LB_
    w = json.loads((OUT / "validation_woombah.json").read_text())["cases"]["logged_current"]["dv"]
    p = build_multipoint_problem(load_multipoint_config(ROOT / "configs" / "woombah_2025.yaml"), driver=False,
                                 mda_retry=True)
    for k, x in w.items():
        p.set_val(k, x)
    parts = ["m_wing", "m_ht", "m_vt", "m_fus", "m_gear"]

    def structure(fb):
        for pt in ("M2", "M3"):
            p.set_val(f"points.{pt}.f_build", fb)
        p.run_model()
        return sum(v(p, f"points.M2.{k}") for k in parts)

    s1 = structure(1.0)
    breakdown1 = {k: v(p, f"points.M2.{k}") for k in parts}
    fb = brentq(lambda f: structure(f) - target, 1.0, 6.0, xtol=1e-10)
    breakdown = {k: v(p, f"points.M2.{k}") for k in parts}
    out = dict(target_kg=target, table34_lb=table34, structure_at_1_kg=s1, ratio_at_1=target / s1, f_build=fb,
               breakdown_at_1=breakdown1, breakdown_calibrated=breakdown, git=git_hash())
    use_set("calibrated")
    (UQ["out"] / "calibration.json").write_text(json.dumps(out, indent=1, default=float))
    # only the f_build override: the other three spreads are the shared ones (placeholders.yaml, the ruleset)
    new = (f"  - {{name: f_build,    dist: uniform, nominal: {fb:.6f}, spread: 0.30, relative: true, "
           "source: 'Woombah Table 34 calibration'}  # Q39: centre calibrated, +/-30 % PLACEHOLDER")
    head = ("# GENERATED by studies/results_2025.py calibrate (do not edit): the reduced set with f_build centred on "
            "Woombah's\n# built/predicted structure, solved through the closure: f_build = "
            f"{fb:.6f} (Table 34 structure {sum(table34.values()):.2f} lb [W p. 45];\n# the uncalibrated model's "
            f"{s1 / LB_:.2f} lb, ratio {target / s1:.3f} before the closure resizes the spar and gear). "
            "Spread +/-30 % about it.\n")
    body = ("# Reduced uncertain set for the 2024-25 RDO: mass and wind only (docs/RDO_REDUCED_PLAN.md). Only what differs "
            "from the\n# shared spreads (configs/placeholders.yaml: f_build, f_tb, m_avionics; configs/rulesets/"
            "dbf_2024_25.yaml: lap.wind) is\n# here, merged by name. Evidence: docs/RDO_REDUCED_PLAN.md section 2 and "
            "OPEN_QUESTIONS Q39; all PLACEHOLDERS.\nparameters:\n  # build-mass factor on the whole structure (wing, "
            "tails, boom, cargo bay, gear): aircrafter2/mass/garcia_gamboa/masses.py\n")
    (ROOT / "configs" / "uncertainty_2025_reduced_calibrated.yaml").write_text(head + body + new + "\n")
    print(f"calibrate: structure at f_build = 1 {s1 / LB_:.3f} lb, target {target / LB_:.2f} lb, ratio {target / s1:.4f}, "
          f"f_build = {fb:.4f}", flush=True)
    return out


def best_case(prefix="mp_"):
    """The best feasible deterministic multipoint run (highest flight score) among outputs/results/runs/<prefix>*."""
    best = None
    for f in RUNS.glob(f"{prefix}*.json"):
        if any(s in f.stem for s in ("_M2only", "_M3only", "_ms", "_norm", "rdo", "_bottle")):
            continue
        d = json.loads(f.read_text())
        if d.get("ok") and "score" in d and (best is None or d["score"]["flight_score"] > best["score"]["flight_score"]):
            best = d
    return best


def _mcfg_for(run, cfg=None):
    return config(run["cells"], run["props"]["M2"], run["props"]["M3"], cfg=cfg or UQ["cfg"],
                  bottle=run.get("bottle"))


def screen(case=None):
    """First-order variance screening of the flight score and the robust constraints at the deterministic optimum."""
    from aircrafter2.problems.multipoint import build_multipoint_problem
    from aircrafter2.robustness.rdo_multipoint import first_order_contributions_mp, resolved_dists_mp, robust_names
    from aircrafter2.robustness.uncertainty import select_parameters

    run = load(case) if case else best_case()
    mcfg = _mcfg_for(run)
    p = build_multipoint_problem(mcfg, driver=False, mda_retry=True)
    for k, x in run["dv"].items():
        p.set_val(k, x)
    dists, _ = resolved_dists_mp(mcfg, lambda n: v(p, f"points.M2.{n}"))
    for d in dists:
        for inp in (f"points.M2.{d.name}", f"points.M3.{d.name}"):
            p.set_val(inp, d.mean)
    p.run_model()
    of = ["flight_score"] + robust_names(mcfg, extra=("points.M2.laps_frac",))
    contrib = first_order_contributions_mp(p, dists, of)
    rob = mcfg.base.robust
    sel, cov, reached = select_parameters(contrib, rob.screening_threshold, rob.max_dim)
    if not UQ["select"]:  # the reduced set: every parameter active, the contributions are a table, not a selection
        sel = [d.name for d in dists]
        tot = {q: sum(c.values()) for q, c in contrib.items()}
        cov, reached = {q: 1.0 for q in contrib if tot[q] > 0}, True
    out = dict(case=run["tag"], set=UQ["name"], selected=sel, coverage=cov, reached=reached, d=len(sel),
               git=git_hash(), contrib=contrib, values={q: v(p, q) for q in of},
               means={d.name: d.mean for d in dists}, std={d.name: d.std for d in dists})
    (UQ["out"] / "screen.json").write_text(json.dumps(out, indent=1, default=float))
    print(f"screen {run['tag']}: d = {len(sel)} {sel} reached={reached} min coverage "
          f"{min(cov.values()) if cov else float('nan'):.3f}", flush=True)
    return out


def _sobol_designs(mcfg, n, seed=0):
    from scipy.stats import qmc

    dvs = mcfg.all_dvs()
    names = list(dvs)
    up = {k: (min(b.upper, mcfg.mp.rules.span_max_ft * 0.3048) if k == "b" else b.upper) for k, b in dvs.items()}
    u = qmc.Sobol(len(names), scramble=True, seed=seed).random(n)
    return [{k: dvs[k].lower + ui * (up[k] - dvs[k].lower) for k, ui in zip(names, row)} for row in u]


def node_check(case=None, workers=5):
    """Every UDR node of the screened set at the optimum, the start point and 6 Sobol designs (both points)."""
    from aircrafter2.problems.multipoint import build_multipoint_problem
    from aircrafter2.robustness.quadrature import make_rule
    from aircrafter2.robustness.rdo_multipoint import resolved_dists_mp
    from aircrafter2.robustness.sampling import evaluate

    sc = json.loads((UQ["out"] / "screen.json").read_text())
    run = load(case or sc["case"])
    mcfg = _mcfg_for(run)
    mcfg.base.robust.active = sc["selected"]
    p = build_multipoint_problem(mcfg, driver=False)
    dists, act = resolved_dists_mp(mcfg, lambda n: v(p, f"points.M2.{n}"))
    rule = make_rule(mcfg.base.robust, act)
    base = {d.name: d.mean for d in dists}
    rows = [dict(base, **{d.name: float(x) for d, x in zip(act, r)}) for r in rule.nodes]
    start = {k: float(np.ravel(p.get_val(k))[0]) for k in mcfg.all_dvs()}
    designs = [("optimum", run["dv"]), ("start", start)] + [(f"sobol{i}", d) for i, d in
                                                            enumerate(_sobol_designs(mcfg, 6))]
    res = []
    for label, dv in designs:
        ev = evaluate(mcfg, dv, rows, ["flight_score"], workers)
        res.append(dict(design=label, failed=ev["failed"], retried=ev["retried"], wall=ev["wall"]))
        print(f"node check {label}: failed {ev['failed']} retried {ev['retried']} {ev['wall']:.0f}s", flush=True)
    (UQ["out"] / "node_check.json").write_text(json.dumps(dict(case=run["tag"], N=len(rows), d=len(act), results=res,
                                                         git=git_hash()), indent=1, default=float))


def rdo(tag, case=None, start=None, x0=None, maxiter=None, index=None):
    """One robust optimisation of the screened case; index: start from that Sobol' design (multistart)."""
    from openmdao.core.analysis_error import AnalysisError

    from aircrafter2.robustness.rdo_multipoint import build_robust_multipoint, node_report_mp, robust_summary_mp

    sc = json.loads((UQ["out"] / "screen.json").read_text())
    run = load(case or sc["case"])
    mcfg = _mcfg_for(run)
    mcfg.base.robust.active = sc["selected"]
    if maxiter:
        mcfg.base.maxiter = maxiter
    if index is not None:
        x0 = _sobol_designs(mcfg, 10)[index]
    p = build_robust_multipoint(mcfg, driver=True, extra_robust=("points.M2.laps_frac",))
    p.driver.options["disp"] = False
    from aircrafter2.problems.multipoint import design_values, run_warm

    start_dv = {**(load(start)["dv"] if start else run["dv"]), **(x0 or {})}
    t = time.perf_counter()
    err = None
    ok, its = False, 0
    try:
        if mcfg.base.mda_robust:  # as results_2026.rdo: the warm path to the start design
            try:
                run_warm(p, _mcfg_for(run), start_dv, run=False)
            except AnalysisError as exc:
                raise AnalysisError(f"warm-path pre-run: {exc}") from exc
        else:
            for k, x in start_dv.items():
                p.set_val(k, x)
        res = p.run_driver()
        ok, its = bool(res.success), int(res.iter_count)
    except AnalysisError as exc:
        ok, its, err = False, int(p.driver.iter_count), f"{_kind(exc)}: {str(exc)[:200]}"
    wall = time.perf_counter() - t

    fallbacks = [dict(point=pt, node=j, continuation=len({e[0] for e in s_.log if e[1].startswith("continuation")}),
                      failed_attempts=len([e for e in s_.log if not e[2]]))
                 for pt, ss in p.rdo["solvers"].items() for j, s_ in enumerate(ss)] if "solvers" in p.rdo else None
    out = dict(tag=tag, case=run["tag"], ok=ok, its=its, error=err, time=wall, N=p.rdo["N"], fallbacks=fallbacks,
               peak_rss_gb=peak_rss_gb(),
               active=[d.name for d in p.rdo["active"]], dv=design_values(p, mcfg),
               start=start or (f"sobol{index}" if index is not None else run["tag"]),
               summary=robust_summary_mp_safe(p), nodes=node_report_mp(p), props=run["props"], cells=run["cells"],
               k=mcfg.base.robust.k, beta=mcfg.base.robust.beta, git=git_hash(),
               threads=os.environ.get("OMP_NUM_THREADS"))
    dump(tag, out)
    fs = (out["summary"] or {}).get("flight_score", {})
    print(f"{tag}: ok={ok} its={its} {wall:.0f}s mu={fs.get('mu', float('nan')):.4f} "
          f"sigma={fs.get('sigma', float('nan')):.4f} {err or ''}", flush=True)


def robust_summary_mp_safe(p):
    try:
        from aircrafter2.robustness.rdo_multipoint import robust_summary_mp

        return robust_summary_mp(p)
    except Exception:  # noqa: BLE001  (timing children: a summary is a nice-to-have)
        return None


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
    return float(bad.mean()) if bad.size else float("nan")


def _longest(flags):
    best = run_ = 0
    for x in flags:
        run_ = run_ + 1 if x else 0
        best = max(best, run_)
    return best


def evaluate_batched(mcfg, dv, rows, outputs, workers, checkpoint=None):
    """sampling.evaluate over consecutive batches of at most BATCH rows, each on a fresh pool (the worker defect:
    RESULTS_LOG.md Decision 13; the frozen evaluate keeps one pool). checkpoint: a stem; each finished batch is saved
    there, so a restart resumes at the next batch."""
    from aircrafter2.robustness.sampling import evaluate

    parts = []
    for b, i in enumerate(range(0, len(rows), BATCH)):
        f = Path(f"{checkpoint}.part{b}.npz") if checkpoint else None
        if f is not None and f.exists():
            z = np.load(f)
            parts.append(dict(values=z["values"], failed=list(z["failed"]), lap_invalid=list(z["lap_invalid"]),
                              retried=list(z["retried"]), wall=float(z["wall"])))
            continue
        ev = evaluate(mcfg, dv, rows[i:i + BATCH], outputs, workers)
        parts.append(ev)
        if f is not None:
            np.savez(f, values=ev["values"], failed=np.array(ev["failed"], int), lap_invalid=np.array(ev["lap_invalid"], int),
                     retried=np.array(ev["retried"], int), wall=ev["wall"])
        print(f"  batch {b}: {len(ev['failed'])} failed, {ev['wall']:.0f} s", flush=True)
    shift = lambda key: sorted(int(b * BATCH + j) for b, p_ in enumerate(parts) for j in p_[key])  # noqa: E731
    return dict(values=np.vstack([p_["values"] for p_ in parts]), failed=shift("failed"), lap_invalid=shift("lap_invalid"),
                retried=shift("retried"), wall=sum(p_["wall"] for p_ in parts), workers=workers)


def failure_check(mcfg, dv, rows, outputs, ev, workers):
    """Are the failures the worker defect (blocks at the end of a batch) or the sample? The longest run of consecutive
    failures, where they fall within their batches, and whether the MDA failures converge on a fresh pool (a report: the
    NaN rows stay NaN)."""
    from aircrafter2.robustness.sampling import evaluate

    n = len(rows)
    flags = np.zeros(n, bool)
    flags[ev["failed"]] = True
    pos = [(i % BATCH) / BATCH for i in ev["failed"]]
    quart = [sum(int(4 * x) == q for x in pos) for q in range(4)]
    mda = sorted(set(ev["failed"]) - set(ev["lap_invalid"]))
    out = dict(longest_failed_run=_longest(flags), failed_by_quarter_of_batch=quart, batch=BATCH, mda_failed=mda)
    if mda:
        again = evaluate(mcfg, dv, [rows[i] for i in mda], outputs, workers)
        out["mda_failed_recovered_on_fresh_pool"] = len(mda) - len(again["failed"])
    return out


def monte_carlo(designs, n=MC_N, workers=5):
    """1024 scrambled Sobol' points (seed 0) over all uncertain parameters, the same points for every design.
    designs: [(label, MultipointConfig, dv)]. Writes outputs/results/mc_<label>.npz and mc.json."""
    from scipy.stats import qmc

    from aircrafter2.problems.multipoint import build_multipoint_problem, multipoint_constraints
    from aircrafter2.robustness.rdo_multipoint import resolved_dists_mp
    from aircrafter2.robustness.sampling import violation_probabilities

    summary = json.loads((UQ["out"] / "mc.json").read_text()) if (UQ["out"] / "mc.json").exists() else {}
    for label, mcfg, dv in designs:
        p = build_multipoint_problem(mcfg, driver=False)
        for k, x in dv.items():
            p.set_val(k, x)
        dists, _ = resolved_dists_mp(mcfg, lambda nm: v(p, f"points.M2.{nm}"))
        u = qmc.Sobol(len(dists), scramble=True, seed=0).random(n)  # common random numbers: the same u for all
        rows = [{d.name: float(d.ppf(ui)) for d, ui in zip(dists, row)} for row in u]
        cons = multipoint_constraints(mcfg)
        outputs = ["flight_score", "M2", "M3"] + [c for c, _ in cons]
        ev = evaluate_batched(mcfg, dv, rows, outputs, workers, checkpoint=UQ["out"] / f"mc_{label}")
        V = ev["values"]
        np.savez(UQ["out"] / f"mc_{label}.npz", values=V, outputs=np.array(outputs), u=u)
        fcheck = failure_check(mcfg, dv, rows, outputs, ev, workers)
        ok = ~np.isnan(V).any(axis=1)
        stats = {}
        for j, o in enumerate(outputs[:3]):
            col = V[ok, j]
            stats[o] = dict(mu=float(col.mean()), sigma=float(col.std(ddof=1)), p5=float(np.percentile(col, 5)))
        summary[label] = dict(N=n, failed=len(ev["failed"]), lap_invalid=len(ev.get("lap_invalid", [])),
                              retried=len(ev["retried"]), wall=ev["wall"],
                              stats=stats, p_violation=violation_probabilities(V, outputs, cons),
                              p_any=any_violation(V, outputs, cons), failure_check=fcheck, batch=BATCH,
                              params=[d.name for d in dists], **provenance())
        (UQ["out"] / "mc.json").write_text(json.dumps(summary, indent=1, default=float))
        (UQ["out"] / f"mc_{label}.json").write_text(json.dumps(summary[label], indent=1, default=float))
        print(f"MC {label}: failed {len(ev['failed'])}, F mu {stats['flight_score']['mu']:.4f} "
              f"sigma {stats['flight_score']['sigma']:.4f} p5 {stats['flight_score']['p5']:.4f}", flush=True)


def classify_mc_failures(workers=5):
    """Re-run the failed Monte Carlo samples of this set's designs (written before failures were classified) to split
    them into MDA non-convergence and the wind limit (a headwind near M3's V_stall: no flight); updates mc.json.
    Skips (with a message) when the set has no Monte Carlo yet."""
    if not (UQ["out"] / "mc.json").exists():
        print(f"classify: no {UQ['out'].relative_to(ROOT)}/mc.json (no Monte Carlo of the {UQ['name']} set): skipped",
              flush=True)
        return
    summary = json.loads((UQ["out"] / "mc.json").read_text())
    designs = {lab: (mcfg, dv) for lab, mcfg, dv in mc_designs()}
    for label, m in summary.items():
        if not m["failed"] or "lap_invalid" in m or label not in designs:
            continue
        z = np.load(UQ["out"] / f"mc_{label}.npz")
        bad = np.where(np.isnan(z["values"]).any(axis=1))[0]
        mcfg, dv = designs[label]
        from aircrafter2.robustness.rdo_multipoint import resolved_dists_mp
        from aircrafter2.robustness.sampling import evaluate

        from aircrafter2.problems.multipoint import build_multipoint_problem

        p = build_multipoint_problem(mcfg, driver=False)
        dists, _ = resolved_dists_mp(mcfg, lambda nm: v(p, f"points.M2.{nm}"))
        rows = [{d.name: float(d.ppf(ui)) for d, ui in zip(dists, z["u"][i])} for i in bad]
        ev = evaluate(mcfg, dv, rows, ["flight_score"], workers)
        m["lap_invalid"] = len(ev["lap_invalid"])
        m["failed_rerun_still_failed"] = len(ev["failed"])
        print(f"classify {label}: {len(bad)} failed, {len(ev['lap_invalid'])} wind limit, "
              f"{len(ev['failed']) - len(ev['lap_invalid'])} MDA, {len(bad) - len(ev['failed'])} converged on rerun",
              flush=True)
    (UQ["out"] / "mc.json").write_text(json.dumps(summary, indent=1, default=float))


def mc_designs(labels=None):
    """The common-evaluation designs: [(label, MultipointConfig with the RDO parameter list, dv)]."""
    from aircrafter2.problems.multipoint import load_multipoint_config

    if UQ["name"] in CALIBRATED_SETS:  # the calibrated deterministic optimum (the screened case) and the set's RDO
        sc = json.loads((UQ["out"] / "screen.json").read_text())
        out = [("multipoint", _mcfg_for(load(sc["case"])), load(sc["case"])["dv"])]
        r = best_rdo()
        if r:
            out.append(("rdo", _mcfg_for(load(r["case"])), r["dv"]))
        return [d for d in out if labels is None or d[0] in labels]
    ucfg = load_multipoint_config(UQ["cfg"])
    rob = ucfg.base.robust
    out = []
    # Woombah as built, at the throttles it was flown at (Table 38 currents), on the calibrated model (in-sample: it is
    # one of the three calibration aircraft; studies/validation_2025.py woombah --mass cal)
    w = json.loads((OUT / "validation_woombah_cal.json").read_text())
    if "logged_current" in w["cases"]:
        wm = load_multipoint_config(WOOMBAH_CFG)
        wm.base.robust = rob.model_copy(deep=True)
        # the set's tail form: the full set pins gg tails (its f_ht / f_vt spreads), as every other design here
        wm.base.mass.options = wm.base.mass.options.model_copy(
            update={"tail_mass_model": ucfg.base.mass.options.tail_mass_model})
        out.append(("woombah", wm, w["cases"]["logged_current"]["dv"]))
    for tag in ("ac1_direct", "ac1_completed"):
        if (RUNS / f"{tag}.json").exists():
            d = load(tag)
            out.append((tag, config(d["cells"], d["props"]["M2"], d["props"]["M3"], cfg=UQ["cfg"],
                                    bottle=d.get("bottle")), d["dv"]))
    b = best_case()
    out.append(("multipoint", _mcfg_for(b), b["dv"]))
    if (RUNS / f"{b['tag']}_bottle2L.json").exists():  # the same case with 2 L bottles (AirCrafter 1.0 (b)'s tanks)
        d = load(f"{b['tag']}_bottle2L")
        if d.get("ok"):
            out.append(("multipoint_2L", _mcfg_for(d), d["dv"]))
    r = best_rdo()
    if r:
        out.append(("rdo", _mcfg_for(load(r["case"])), r["dv"]))
    return [d for d in out if labels is None or d[0] in labels]


def multistart(case, index, n=10):
    """Start `index` of n scrambled Sobol' points (seed 0) in the design-variable box, same case as `case`."""
    d = load(case)
    mcfg = config(d["cells"], d["props"]["M2"], d["props"]["M3"])
    x0 = _sobol_designs(mcfg, n)[index]
    optimise(f"{case}_ms{index}", d["cells"], d["props"]["M2"], d["props"]["M3"], x0=x0)


def normalisation(case, f2, f3):
    """The case re-optimised with R2 x f2 and R3 x f3 (from its own optimum)."""
    from aircrafter2.problems.multipoint import load_multipoint_config

    d = load(case)
    sc = load_multipoint_config(CFG).mp.scoring
    optimise(f"{case}_norm_{f2:g}_{f3:g}", d["cells"], d["props"]["M2"], d["props"]["M3"],
             scoring={"R2": sc["R2"] * f2, "R3": sc["R3"] * f3}, start=case)


def bookends(case):
    d = load(case)
    kw = dict(cells=d["cells"], m2=d["props"]["M2"], m3=d["props"]["M3"])
    optimise(f"{case}_M2only", scoring={"w2": 1.0, "w3": 0.0}, start=case, **kw)
    # M3's laps are capped by the 100 Wh rule, so its objective is nearly flat in the airframe: more iterations
    optimise(f"{case}_M3only", scoring={"w2": 0.0, "w3": 1.0}, start=case, maxiter=300, **kw)

DOCS = ROOT / "docs" / "studies" / "results"
EXCLUDE = ("_M2only", "_M3only", "_ms", "_norm", "rdo", "_bottle")


def deterministic_cases():
    """The propulsion-enumeration runs (every outputs/results/runs/mp_* that is not a bookend/multistart/grid run)."""
    return sorted((json.loads(f.read_text()) for f in RUNS.glob("mp_*.json") if not any(s in f.stem for s in EXCLUDE)),
                  key=lambda d: d["tag"])


def best_rdo():
    """The best converged RDO run (highest mu - k sigma) of the current uncertain set: outputs/results/runs/<prefix>*
    (rdo_ full, rdor_ reduced, rdorc_ calibrated, rdoro_ one-sided)."""
    runs = [json.loads(f.read_text()) for f in RUNS.glob(f"{UQ['prefix']}*.json")]
    ok = [r for r in runs if r.get("ok") or (r.get("error") is None and r.get("summary"))]
    return max(ok, key=lambda r: r["summary"]["flight_score"]["robust"]) if ok else None


def common(labels=None):
    """Common evaluation 2: every design in the AirCrafter 2.0 multipoint analysis at nominal inputs (summarise())."""
    from openmdao.core.analysis_error import AnalysisError

    from aircrafter2.problems.multipoint import build_multipoint_problem, run_warm

    out = json.loads((UQ["out"] / "common_nominal.json").read_text()) if (UQ["out"] / "common_nominal.json").exists() else {}
    for label, mcfg, dv in mc_designs(labels):
        p = build_multipoint_problem(mcfg, driver=False, mda_retry=True)
        try:
            if mcfg.base.mda_robust:
                run_warm(p, mcfg, dv)
            else:
                for k, x in dv.items():
                    p.set_val(k, x)
                p.run_model()
            out[label] = dict(ok=True, **summarise(p, mcfg), **provenance())
        except AnalysisError as exc:
            out[label] = dict(ok=False, error=f"{_kind(exc)}: {str(exc)[:200]}", **provenance())
        print(f"common {label}: {out[label].get('score', {}).get('flight_score', out[label].get('error'))}",
              flush=True)
    (UQ["out"] / "common_nominal.json").write_text(json.dumps(out, indent=1, default=float))


def continuity():
    """The six designs of the published 2024-25 page (docs/studies/results/results_2025.json, git 34e4834), unchanged, at the
    final code: take-off off, then take-off on. Separates what the code change moved from what the take-off moved, and
    reports whether the old designs still satisfy the constraints in the new model."""
    from openmdao.core.analysis_error import AnalysisError

    from aircrafter2.problems.multipoint import build_multipoint_problem, multipoint_constraints, run_warm
    from aircrafter2.robustness.sampling import violation_probabilities

    old = json.loads((ROOT / "docs" / "studies" / "results" / "results_2025.json").read_text())["uq"]["reduced"]["nominal"]
    res = {}
    for label, o in old.items():
        res[label] = dict(old_F=o["score"]["flight_score"], old_M2=o["score"]["M2"], old_M3=o["score"]["M3"],
                          old_t3=o["M2"]["ThreeLapTime"])
        bottle = "2L" if o["fuel_lb"] > 13.3 else None
        wb = label == "woombah"
        for name, base, wcfg in (("takeoff_off", ROOT / "configs" / "dbf2025_multipoint.yaml", ROOT / "configs" / "woombah_2025.yaml"),
                                 ("takeoff_on", CFG, WOOMBAH_CFG)):
            mcfg = config(cfg=wcfg) if wb else config(o["cells"], o["props"]["M2"], o["props"]["M3"], cfg=base, bottle=bottle)
            p = build_multipoint_problem(mcfg, driver=False, mda_retry=True)
            try:
                run_warm(p, mcfg, o["dv"])
                outs = [c for c, _ in multipoint_constraints(mcfg)]
                V = np.array([[v(p, c) for c in outs]])
                res[label][name] = dict(F=v(p, "flight_score"), M2=v(p, "M2"), M3=v(p, "M3"),
                                        t3=v(p, "points.M2.ThreeLapTime"), m2_total=v(p, "points.M2.m_total"),
                                        violated=[c for c, x in violation_probabilities(V, outs, multipoint_constraints(mcfg)).items() if x > 0])
            except AnalysisError as exc:
                res[label][name] = dict(error=f"{_kind(exc)}: {str(exc)[:200]}")
        print("continuity", label, {k: x for k, x in res[label].items()}, flush=True)
    dump("continuity", dict(tag="continuity", designs=res))


def robust_eval(workers=5):
    """UDR moments over the screened set (the RDO's nodes) of the flight score and the robust constraints at every
    deterministic optimum and every common-evaluation design: does the propulsion ranking change under uncertainty?"""
    from aircrafter2.problems.multipoint import build_multipoint_problem, multipoint_constraints
    from aircrafter2.robustness.quadrature import make_rule
    from aircrafter2.robustness.rdo_multipoint import resolved_dists_mp, robust_names
    from aircrafter2.robustness.sampling import evaluate

    sc = json.loads((UQ["out"] / "screen.json").read_text())
    if UQ["name"] in CALIBRATED_SETS:  # the calibrated deterministic optimum and the set's robust optimum only
        designs = [(d["tag"], _mcfg_for(d), d["dv"]) for d in (load(sc["case"]), best_rdo()) if d]
    else:
        designs = [(d["tag"], _mcfg_for(d), d["dv"]) for d in deterministic_cases() if d.get("ok")]
        designs += [d for d in mc_designs() if d[0] != "multipoint"]
    out = {}
    for label, mcfg, dv in designs:
        mcfg.base.robust.active = sc["selected"]
        rob = mcfg.base.robust
        p = build_multipoint_problem(mcfg, driver=False)
        nominal = None
        if UQ["name"] in CALIBRATED_SETS:  # the nominal (calm-air) score, for the cost of robustness
            for k, x in dv.items():
                p.set_val(k, x)
            p.run_model()
            nominal = v(p, "flight_score")
        dists, act = resolved_dists_mp(mcfg, lambda n: v(p, f"points.M2.{n}"))
        rule = make_rule(rob, act)
        base = {d.name: d.mean for d in dists}
        rows = [dict(base, **{d.name: float(x) for d, x in zip(act, r)}) for r in rule.nodes]
        cons = dict(multipoint_constraints(mcfg))
        rn = robust_names(mcfg, extra=("points.M2.laps_frac",))
        ev = evaluate(mcfg, dv, rows, ["flight_score"] + rn, workers)
        res = dict(N=rule.n, failed=len(ev["failed"]), robust_violated=[])
        if nominal is not None:
            res["nominal"] = nominal
        if not ev["failed"]:
            mu, var = rule.moments(ev["values"][:, 0])
            sig = float(np.sqrt(max(var, 0.0)))
            res.update(mu=mu, sigma=sig, robust=mu - rob.k * sig)
            for j, n in enumerate(rn, start=1):
                m_, var_ = rule.moments(ev["values"][:, j])
                s_ = np.sqrt(max(var_, 0.0))
                kw = cons[n]
                tol = 1e-6 * max(abs(kw.get("lower") or kw.get("upper") or 0.0), kw.get("ref") or 0.0, 1.0)
                if (kw.get("lower") is not None and m_ - rob.beta * s_ < kw["lower"] - tol) or \
                        (kw.get("upper") is not None and m_ + rob.beta * s_ > kw["upper"] + tol):
                    res["robust_violated"].append(n)
        out[label] = res
        print(f"robust eval {label}: {res}", flush=True)
    (UQ["out"] / "robust_eval.json").write_text(json.dumps(dict(selected=sc["selected"], results=out, git=git_hash()),
                                                     indent=1, default=float))


def propagation(workers=5, n=MC_N):
    """Run 4: at the set's robust optimum, the flight score and the robust constraints propagated by UDR (the RDO's
    rule), Smolyak level 2 and a scrambled Sobol' Monte Carlo (seed 0) over the active parameters."""
    from scipy.stats import qmc

    from aircrafter2.problems.multipoint import build_multipoint_problem, multipoint_constraints
    from aircrafter2.robustness.quadrature import make_rule
    from aircrafter2.robustness.rdo_multipoint import resolved_dists_mp, robust_names
    from aircrafter2.robustness.sampling import evaluate, violation_probabilities

    r = best_rdo()
    mcfg = _mcfg_for(load(r["case"]))
    mc_file = UQ["out"] / "mc_rdo.npz"
    rob = mcfg.base.robust
    p = build_multipoint_problem(mcfg, driver=False)
    dists, act = resolved_dists_mp(mcfg, lambda nm: v(p, f"points.M2.{nm}"))
    base = {d.name: d.mean for d in dists}
    cons = multipoint_constraints(mcfg)
    rn = robust_names(mcfg, extra=("points.M2.laps_frac",))
    outputs = ["flight_score"] + rn
    out = dict(design=r["tag"], params=[d.name for d in act], git=git_hash(), methods={})
    for meth in ("udr", "smolyak"):
        rule = make_rule(rob.model_copy(update={"quadrature": meth}), act)
        rows = [dict(base, **{d.name: float(x) for d, x in zip(act, row)}) for row in rule.nodes]
        ev = evaluate(mcfg, r["dv"], rows, outputs, workers)
        res = dict(N=rule.n, failed=len(ev["failed"]), wall=ev["wall"], stats={})
        if not ev["failed"]:
            for j, o in enumerate(outputs):
                mu, var = rule.moments(ev["values"][:, j])
                res["stats"][o] = dict(mu=float(mu), sigma=float(np.sqrt(max(var, 0.0))))
        out["methods"][meth] = res
        print(f"propagation {meth}: N = {rule.n}, failed {len(ev['failed'])}, {ev['wall']:.0f} s", flush=True)
    u = qmc.Sobol(len(act), scramble=True, seed=0).random(n)
    rows = [dict(base, **{d.name: float(d.ppf(ui)) for d, ui in zip(act, row)}) for row in u]
    allout = outputs + [c for c, _ in cons if c not in outputs]
    z = np.load(mc_file) if mc_file.exists() else None
    if z is not None and len(act) == len(dists) and np.array_equal(z["u"], u):  # every parameter active: the same rows
        names = [str(x) for x in z["outputs"]]  # as the design's in-set Monte Carlo, so its array is the Sobol' estimate
        V = z["values"][:, [names.index(o) for o in allout]]
        mc_summary = json.loads((UQ["out"] / "mc_rdo.json").read_text())
        ev = dict(values=V, failed=[0] * mc_summary["failed"], wall=0.0)
        out["sobol_source"] = "the in-set Monte Carlo of this design (same rows, batched)"
    else:
        ev = evaluate_batched(mcfg, r["dv"], rows, allout, workers)
        V = ev["values"]
    ok = ~np.isnan(V).any(axis=1)
    out["methods"]["sobol"] = dict(N=n, failed=len(ev["failed"]), wall=ev["wall"],
                                   stats={o: dict(mu=float(V[ok, j].mean()), sigma=float(V[ok, j].std(ddof=1)))
                                          for j, o in enumerate(outputs)},
                                   p_violation=violation_probabilities(V, allout, cons))
    print(f"propagation sobol: N = {n}, failed {len(ev['failed'])}, {ev['wall']:.0f} s", flush=True)
    (UQ["out"] / "propagation.json").write_text(json.dumps(out, indent=1, default=float))


# ---------------------------------------------------------------------------------------------------------- report


def _own_claims():
    """Common evaluation 1: what each method claims in its own model."""
    from aircrafter2.scoring.dbf2025 import m2_verbatim, m3_verbatim
    import sys

    sys.path.insert(0, str(ROOT / "studies"))
    from validation_2025 import TABLE39

    sc = load(best_case()["tag"])["scoring"]
    out = {}
    d = TABLE39["M2"]["design"]
    t3 = 3 * d["avg_lap_s"]
    m2 = m2_verbatim(d["fuel_lb"], t3, sc["R2"])
    m3 = m3_verbatim(TABLE39["M3"]["design"]["laps"] - 1, 0.0, TABLE39["M3"]["design"]["x1_lb"], sc["R3"])
    out["woombah"] = dict(source="Table 39 'Design' column [W p. 59]; M2 time = 3 x the 33 s average lap", fuel_lb=
                          d["fuel_lb"], t3=t3, laps=TABLE39["M3"]["design"]["laps"], M2=m2, M3=m3, F=1 + m2 + m3)
    a = json.loads(AC1_JSON.read_text())["aircrafter1_2025"]["DP"]
    ac = load("ac1_direct")["ac1"]
    for tag in ("ac1_direct", "ac1_completed"):
        fuel = ac["fuel_kg"] / LB_
        out[tag] = dict(source="AirCrafter 1.0 '2025' mode, its selected design point (the same for both mappings); "
                        "t3 from its M2 score", fuel_lb=fuel, t3=fuel / ((a["M2score"] - 1.0) * sc["R2"]),
                        laps=a["M3_LapCount"],
                        M2=a["M2score"], M3=a["M3score"], F=1 + a["M2score"] + a["M3score"])
    b = best_case()
    out["multipoint"] = dict(source="AirCrafter 2.0 nominal (its own model)", fuel_lb=b["fuel_lb"],
                             t3=b["M2"]["ThreeLapTime"], laps=b["M3"]["laps_frac"], M2=b["score"]["M2"],
                             M3=b["score"]["M3"], F=b["score"]["flight_score"])
    return out


def _own_rdo():
    r = best_rdo()
    if not r:
        return None
    s = r["summary"]["flight_score"]
    return dict(source="AirCrafter 2.0 RDO: UDR mean, sigma and mu - k sigma of the flight score at its own "
                "uncertain set", fuel_lb=r["dv"]["fuel_mass"] / LB_, F=s["mu"], sigma=s["sigma"], robust=s["robust"])


def _set_block():
    """Everything of the current uncertain set (use_set) the report needs."""
    o = UQ["out"]

    def rd(f):
        return json.loads((o / f).read_text()) if (o / f).exists() else None

    runs = []
    for f in sorted(RUNS.glob(f"{UQ['prefix']}*.json")):
        d = json.loads(f.read_text())
        runs.append({k: d.get(k) for k in ("tag", "ok", "its", "error", "time", "N", "active", "start", "dv",
                                          "summary", "nodes", "k", "beta", "case")})
    br = best_rdo()
    main = UQ["name"] == "reduced"  # the full set's common evaluation is not on these pages (FULL_NOTE)
    return dict(name=UQ["name"], cfg=str(UQ["cfg"].relative_to(ROOT)), out=str(o.relative_to(ROOT)),
                screen=rd("screen.json"), node_check=rd("node_check.json"), rdo=runs,
                rdo_best=br["tag"] if br else None, mc=rd("mc.json") or {}, robust_eval=rd("robust_eval.json"),
                nominal=rd("common_nominal.json") or {}, propagation=rd("propagation.json"),
                calibration=rd("calibration.json"), own_rdo=_own_rdo(),
                designs=[d[0] for d in mc_designs()] if main else [])


def _fig(name, fn):
    from aircrafter2.plotting.style import figure, save

    fdir = DOCS / "figures"
    fdir.mkdir(parents=True, exist_ok=True)
    fig, ax = figure(*fn.size) if hasattr(fn, "size") else figure(0.8, 0.6)
    fn(fig, ax)
    save(fig, fdir / f"{name}.pdf")
    return f"figures/{name}.pdf"


def _figures(R):
    from aircrafter2.plotting.style import PALETTE

    files = []
    enum = [c for c in R["enumeration"] if c["ok"]]

    def f_enum(fig, ax):
        lab = [f"{c['cells']:g}S {c['props']['M2']}/{c['props']['M3']}" for c in enum]
        ax.barh(range(len(enum)), [c["F"] for c in enum], color=PALETTE[0])
        ax.set_yticks(range(len(enum)), lab)
        lo = min(c["F"] for c in enum)
        ax.set_xlim(lo - 0.05, max(c["F"] for c in enum) + 0.02)
        ax.set_xlabel("flight score F = M1 + M2 + M3 (nominal)")
        ax.set_ylabel("cells, M2 prop / M3 prop")
    files.append(_fig("results_enumeration", f_enum))

    g = R["normalisation"]["grid"]
    if g:
        def f_norm(fig, ax):
            fs = sorted({r["f2"] for r in g})
            gs = sorted({r["f3"] for r in g})
            Z = np.full((len(fs), len(gs)), np.nan)
            for r in g:
                Z[fs.index(r["f2"]), gs.index(r["f3"])] = r["fuel_lb"]
            im = ax.imshow(Z, origin="lower", cmap="Blues")
            for r in g:
                ax.text(gs.index(r["f3"]), fs.index(r["f2"]), f"{r['fuel_lb']:.1f} lb\n{r['laps']:.2f} laps",
                        ha="center", va="center", fontsize=7)
            ax.set_xticks(range(len(gs)), [f"R3 x{x:g}" for x in gs])
            ax.set_yticks(range(len(fs)), [f"R2 x{x:g}" for x in fs])
            ax.grid(False)
            fig.colorbar(im, ax=ax, label="M2 fuel weight (lb)")
        f_norm.size = (0.7, 0.75)
        files.append(_fig("results_normalisation", f_norm))

    files += _figures_set(R, R["uq"]["reduced"], "")
    return files


def _own_of(R, B):
    own = dict(R["common"]["own"])
    if B.get("own_rdo"):
        own["rdo"] = B["own_rdo"]
    return own


def _figures_set(R, B, suffix):
    from aircrafter2.plotting.style import PALETTE

    files = []
    mc, own = B["mc"], _own_of(R, B)
    if mc:
        def f_mc(fig, ax):
            for i, lab in enumerate(mc):
                f = ROOT / B["out"] / f"mc_{lab}.npz"
                if not f.exists():
                    continue
                z = np.load(f)
                col = z["values"][:, 0]
                col = np.sort(col[~np.isnan(col)])
                ax.plot(col, np.arange(1, len(col) + 1) / len(col), color=PALETTE[i % len(PALETTE)],
                        label=lab.replace("_", " "))
            ax.set_xlabel(f"flight score F (Monte Carlo over the {B['name']} set, 1024 Sobol' points, converged)")
            ax.set_ylabel("cumulative probability")
            ax.legend(frameon=False)
        files.append(_fig("results_mc_cdf" + suffix, f_mc))

    def f_common(fig, ax):
        labs = list(B["designs"])
        cols = [("own model", lambda d: own.get(d, {}).get("F")),
                ("AC2 nominal", lambda d: B["nominal"].get(d, {}).get("score", {}).get("flight_score")),
                ("MC mean", lambda d: mc.get(d, {}).get("stats", {}).get("flight_score", {}).get("mu")),
                ("MC 5th pct.", lambda d: mc.get(d, {}).get("stats", {}).get("flight_score", {}).get("p5"))]
        w = 0.2
        for j, (name, fn) in enumerate(cols):
            ys = [fn(d) for d in labs]
            ax.bar([i + (j - 1.5) * w for i in range(len(labs))], [y if y is not None else 0 for y in ys], w,
                   label=name, color=PALETTE[j])
        ax.set_xticks(range(len(labs)), [d.replace("_", "\n") for d in labs])
        ax.set_ylim(4.0, None)
        ax.set_ylabel("flight score F")
        ax.legend(frameon=False, ncol=2)
    if B["designs"]:
        files.append(_fig("results_common_evaluation" + suffix, f_common))
    return files


def _fmt(x, nd=3):
    if x is None:
        return "–"
    if isinstance(x, str):
        return x
    if isinstance(x, (bool, np.bool_)):
        return "yes" if x else "no"
    if not np.isfinite(x):
        return "nan"
    return f"{x:.{nd}f}"


def _table(header, rows):
    s = "| " + " | ".join(header) + " |\n|" + "|".join("---" if i == 0 else "---:" for i in range(len(header))) + "|\n"
    return s + "".join("| " + " | ".join(str(c) for c in r) + " |\n" for r in rows)


def collect():
    """Everything the report needs, from outputs/results (JSON only: the numbers in the markdown come from here)."""
    from aircrafter2.problems.multipoint import load_multipoint_config, multipoint_constraints

    R = dict(git=git_hash())
    base = best_case()
    R["best_case"] = base["tag"]
    R["scoring"] = base["scoring"]
    R["enumeration"] = []
    for d in deterministic_cases():
        row = dict(tag=d["tag"], cells=d["cells"], props=d["props"], ok=d["ok"], its=d["its"], time=d["time"],
                   error=d.get("error"))
        if "score" in d:
            row.update(F=d["score"]["flight_score"], M2=d["score"]["M2"], M3=d["score"]["M3"], fuel_lb=d["fuel_lb"],
                       t3=d["M2"]["ThreeLapTime"], laps=d["M3"]["laps_frac"], Kv=d["dv"]["Kv"], kv_set_by=d["kv_set_by"],
                       active=d["active_constraints"], bounds=d["active_bounds"], M1_passed=d["M1_check"]["passed"],
                       M1_violated=d["M1_check"].get("violated"))
        R["enumeration"].append(row)
    R["bottles"] = []
    for tag, bottle in ((f"{base['tag']}_bottle0.5L", "0.5L"), (base["tag"], "1L"), (f"{base['tag']}_bottle2L", "2L")):
        f = RUNS / f"{tag}.json"
        if f.exists():
            d = json.loads(f.read_text())
            row = dict(tag=tag, bottle=bottle, ok=d["ok"], its=d["its"], error=d.get("error"))
            if "score" in d:
                row.update(F=d["score"]["flight_score"], M2=d["score"]["M2"], M3=d["score"]["M3"],
                           fuel_lb=d["fuel_lb"], t3=d["M2"]["ThreeLapTime"], laps=d["M3"]["laps_frac"],
                           m2_total=d["M2"]["m_total"], cd_stores=d["M2"]["CD_stores"], bounds=d["active_bounds"],
                           M1_passed=d["M1_check"]["passed"])
            R["bottles"].append(row)

    ms = []
    for i in range(10):
        f = RUNS / f"{base['tag']}_ms{i}.json"
        if f.exists():
            d = json.loads(f.read_text())
            ms.append(dict(index=i, ok=d["ok"], its=d["its"], time=d["time"], error=d.get("error"),
                           F=d.get("score", {}).get("flight_score")))
    R["multistart"] = dict(default=base["score"]["flight_score"], runs=ms)

    bk = {}
    for s in ("M2only", "M3only"):
        f = RUNS / f"{base['tag']}_{s}.json"
        if f.exists():
            d = json.loads(f.read_text())
            bk[s] = dict(ok=d["ok"], its=d["its"], maxiter=d.get("maxiter"), error=d.get("error"),
                         **({k: d["score"][k] for k in ("flight_score", "M2", "M3")} if "score" in d else {}),
                         **(dict(fuel_lb=d["fuel_lb"], laps=d["M3"]["laps_frac"], t3=d["M2"]["ThreeLapTime"],
                                 dv=d["dv"], active=d["active_constraints"], bounds=d["active_bounds"])
                            if "score" in d else {}))
    R["bookends"] = bk

    grid = [dict(f2=1.0, f3=1.0, tag=base["tag"], ok=True, its=base["its"], F=base["score"]["flight_score"],
                 M2=base["score"]["M2"], M3=base["score"]["M3"], fuel_lb=base["fuel_lb"], t3=base["M2"]["ThreeLapTime"],
                 laps=base["M3"]["laps_frac"], dv=base["dv"])]
    for f in RUNS.glob(f"{base['tag']}_norm_*.json"):
        d = json.loads(f.read_text())
        f2, f3 = (float(x) for x in f.stem.split("_norm_")[1].split("_"))
        row = dict(f2=f2, f3=f3, tag=d["tag"], ok=d["ok"], its=d["its"], error=d.get("error"))
        if "score" in d:
            row.update(F=d["score"]["flight_score"], M2=d["score"]["M2"], M3=d["score"]["M3"], fuel_lb=d["fuel_lb"],
                       t3=d["M2"]["ThreeLapTime"], laps=d["M3"]["laps_frac"], dv=d["dv"])
        grid.append(row)
    R["normalisation"] = dict(grid=sorted(grid, key=lambda r: (r["f2"], r["f3"])))

    R["uq"] = {}
    for name in ("reduced", "full", "onesided"):  # "calibrated" (Woombah-only f_build) dropped on the calibrated model
        use_set(name)
        R["uq"][name] = _set_block()
    use_set("reduced")
    R["common"] = dict(own=_own_claims())
    mcfg = load_multipoint_config(CFG)
    R["constraints"] = {n: kw for n, kw in multipoint_constraints(mcfg)}
    x1 = mcfg.mp.scoring.get("x1_weight_lb", 0.37)
    R["bonus"] = dict(B=2.5, x1_lb=x1, dM3=2.5 / x1 / R["scoring"]["R3"])
    return R


def report():
    R = collect()
    R["figures"] = _figures(R)
    DOCS.mkdir(parents=True, exist_ok=True)
    (DOCS / "results_2025.json").write_text(json.dumps(R, indent=1, default=float))
    (DOCS / "results_2025.md").write_text(render(R))
    print("wrote docs/studies/results/results_2025.{md,json}", flush=True)


GEOM = [("b", "span b (m)"), ("c", "chord c (m)"), ("tc_scale", "t/c scale"), ("camber_scale", "camber scale"),
        ("l_ht", "tail arm l_ht (m)"), ("S_ht", "S_ht (m²)"), ("S_vt", "S_vt (m²)"), ("x_cb_frac", "x_cb / l"),
        ("Kv", "Kv (rpm/V)"), ("capacity_M2", "M2 pack (Ah)"), ("capacity_M3", "M3 pack (Ah)"),
        ("throttle_M2", "M2 throttle"), ("throttle_M3", "M3 throttle"), ("x_batt_M2", "M2 battery station (m)"),
        ("x_batt_M3", "M3 battery station (m)")]
MASSES = ["m_total", "m_empty", "m_wing", "m_ht", "m_vt", "m_fus", "m_gear", "m_sys", "m_batt", "m_pay"]
PERF = [("v", "cruise speed (m/s)", 1), ("ThreeLapTime", "3-lap time (s)", 1), ("laps_frac", "laps in 10 min", 2),
        ("CD0", "C_D0 (with stores)", 4), ("CD_stores", "C_D stores", 4), ("cruise.L_over_D", "cruise L/D", 1),
        ("I_static", "static current (A)", 1), ("rpm_margin", "rpm margin", 0), ("I_margin", "current margin (A)", 1),
        ("SM", "static margin", 3), ("energy_margin", "energy margin", 0), ("stall_margin", "stall margin (m/s)", 1)]


def _rdo_findings(R, base):
    F = []
    B, Bf = R["uq"]["reduced"], R["uq"]["full"]
    nom = B["nominal"]
    rb = B["rdo_best"]
    if rb:
        r = next(x for x in B["rdo"] if x["tag"] == rb)
        s = r["summary"]["flight_score"]
        re_ = (B["robust_eval"] or {}).get("results", {}).get(base, {})
        F.append(f"RDO, reduced set ({', '.join(r['active'])}; d = {len(r['active'])}, N = {r['N']} nodes x 2 "
                 f"points; {rb}): mu - k sigma = {s['robust']:.4f} (mu {s['mu']:.4f}, sigma {s['sigma']:.4f}). The "
                 f"deterministic optimum has mu - k sigma {_fmt(re_.get('robust'), 4)} at the same nodes"
                 + (f" and violates {len(re_['robust_violated'])} robust constraints there"
                    if re_.get("robust_violated") else "") + ". Nominal (calm-air) cost of robustness: "
                 f"{_fmt(nom.get('multipoint', {}).get('score', {}).get('flight_score', np.nan) - nom.get('rdo', {}).get('score', {}).get('flight_score', np.nan), 4)}"
                 " in F.")
        mc = B["mc"]
        if "multipoint" in mc and "rdo" in mc:
            pa, pb = (max(mc[k]["p_violation"].values()) for k in ("multipoint", "rdo"))
            rob = [x["summary"]["flight_score"]["robust"] for x in B["rdo"] if x.get("ok")]
            same = (f"{len(rob)} converged RDO starts, spread of mu - k sigma {max(rob) - min(rob):.1e}. "
                    if len(rob) > 1 else "")
            okr = [x for x in B["rdo"] if x.get("ok")]
            if len(okr) > 1:
                dvs = okr[0]["dv"].keys()
                spread = {k: (min(x["dv"][k] for x in okr), max(x["dv"][k] for x in okr)) for k in dvs}
                moved = [f"{k} {lo:.4g}–{hi:.4g}" for k, (lo, hi) in spread.items()
                         if hi - lo > 0.01 * max(abs(hi), abs(lo), 1e-9)]
                if moved:
                    kt = [x["dv"]["Kv"] * x["dv"]["throttle_M2"] for x in okr] if "Kv" in dvs else []
                    same += ("The designs are not identical: they differ in " + ", ".join(moved)
                             + (f" (Kv x M2 throttle {min(kt):.0f}–{max(kt):.0f}: a flat direction of the "
                                "objective, the motor's no-load speed at the M2 throttle is what matters)" if kt else "")
                             + ". ")
            sc_ = B["screen"] or {}
            lap0 = (sc_.get("values") or {}).get("points.M2.laps_frac")
            pl = mc["multipoint"]["p_violation"].get("points.M2.laps_frac")
            if lap0 is not None and pl is not None:
                F.append(f"**Wind is what robustness buys here.** The deterministic optimum flies M2's 3 laps exactly in "
                         f"calm air (the laps constraint is active), so any headwind costs it laps: at the mean wind "
                         f"({sc_['means'].get('lap.wind', float('nan')):g} m/s) it flies {lap0:.3f} laps, and in "
                         f"{100 * pl:.0f} % of the Monte Carlo samples it misses M2's 3-lap requirement. The RDO "
                         "design keeps the laps margin (mu - 2 sigma >= 3) for almost no score.")
            F.append(same + f"Monte Carlo over the reduced set: the largest probability of violating a constraint "
                     f"goes from {pa:.3f} (multipoint) to {pb:.3f} (RDO); the 5th percentile of F from "
                     f"{mc['multipoint']['stats']['flight_score']['p5']:.4f} to "
                     f"{mc['rdo']['stats']['flight_score']['p5']:.4f}.")
        if "rdo_full15" in mc:
            m = mc["rdo_full15"]
            F.append(f"Out of set: the reduced-set RDO design under the full 15-parameter set has F mu "
                     f"{m['stats']['flight_score']['mu']:.4f}, sigma {m['stats']['flight_score']['sigma']:.4f} and a "
                     f"largest P(violation) of {max(m['p_violation'].values()):.3f} ("
                     + ", ".join(f"{k.replace('points.', '')} {x:.2f}" for k, x in sorted(
                         m["p_violation"].items(), key=lambda t: -t[1])[:3] if x > 0) + "): what the reduced set "
                     "leaves unprotected.")
    if Bf["rdo_best"]:
        r = next(x for x in Bf["rdo"] if x["tag"] == Bf["rdo_best"])
        s = r["summary"]["flight_score"]
        F.append(f"Full 15-parameter set on the calibrated model: RDO mu - k sigma {s['robust']:.4f} (mu "
                 f"{s['mu']:.4f}, sigma {s['sigma']:.4f}) over the {len(r['active'])} screened parameters; its Monte "
                 f"Carlo and common evaluation are in {FULL_NOTE}, not on this page.")
    return F


def _sec_common(w, R, B, suffix):
    nom, own, mc, designs = B["nominal"], _own_of(R, B), B["mc"], B["designs"]
    params = next((m.get("params") for m in mc.values() if m.get("params")), [])
    w("1 = in its own model (what each method claims); 2 = the AirCrafter 2.0 multipoint analysis at nominal inputs "
      "(calm air, every factor at its config value; **favours the AirCrafter 2.0 designs**); 3 = Monte Carlo in "
      f"AirCrafter 2.0, 1024 scrambled Sobol' points (seed 0) over the {len(params)} uncertain parameters of this set "
      f"({', '.join(params)}), the same points for every design (**favours RDO by construction**). Analysis failures "
      "are excluded from the statistics and counted in their own column: MDA non-convergence, or **wind limit (no "
      "flight)**: the headwind is close to the light M3 aircraft's V_stall, so it cannot make progress into the wind "
      "after lift-off and would not fly (the aircraft's wind limit, a property of the design, docs/modules/mission.md)."
      "\n")
    rows = []
    for d in designs + [k for k in mc if k not in designs]:
        o, n_, m = own.get(d, {}) if d in designs else {}, nom.get(d, {}), mc.get(d, {})
        st = m.get("stats", {})
        rows.append([d, _fmt(o.get("F"), 4), _fmt(n_.get("score", {}).get("flight_score"), 4),
                     _fmt(st.get("flight_score", {}).get("mu"), 4), _fmt(st.get("flight_score", {}).get("sigma"), 4),
                     _fmt(st.get("flight_score", {}).get("p5"), 4), _fmt(st.get("M2", {}).get("mu"), 4),
                     _fmt(st.get("M2", {}).get("p5"), 4), _fmt(st.get("M3", {}).get("mu"), 4),
                     _fmt(st.get("M3", {}).get("p5"), 4),
                     (f"{m['failed']}/{m['N']}" + (f" ({m['lap_invalid']} wind limit)" if m.get("lap_invalid")
                                                   else "")) if m else "–"])
    w(_table(["design", "F own (1)", "F nominal (2)", "F mu (3)", "F sigma", "F p5", "M2 mu", "M2 p5", "M3 mu",
              "M3 p5", "analysis failures"], rows))
    if "rdo_full15" in mc:
        w("\n`rdo_full15` = this set's RDO design in a Monte Carlo over the **full 15-parameter set** (the same 1024-point "
          "rule over 15 dimensions): an out-of-set check of what the reduced set leaves unprotected.\n")
    w("\nOwn-model sources: " + "; ".join(f"{d}: {own[d]['source']}" for d in own if d in designs) + ".\n")
    for d in ("ac1_direct", "ac1_completed"):
        if d in own:
            w(f"\nAirCrafter 1.0's claim ({own[d]['fuel_lb']:.1f} lb, {own[d]['laps']:g} M3 laps) has no store drag "
              "(it has no way to represent it; a limitation of AirCrafter 1.0, not added).\n")
            break
    if mc:
        names = sorted({k for m in mc.values() for k, p in m["p_violation"].items() if p > 0})
        w("\n**Probability of violating each constraint** (Monte Carlo, converged samples; constraints never violated "
          "by any design are omitted):\n\n")
        w(_table(["constraint"] + list(mc), [[n] + [_fmt(mc[d]["p_violation"].get(n), 3) for d in mc] for n in names])
          if names else "No constraint is violated in any converged sample.\n")
    w(f"\n![common evaluation](figures/results_common_evaluation{suffix}.png)\n\n"
      f"![MC](figures/results_mc_cdf{suffix}.png)\n")


def _sec_robust(w, R, B, ok):
    s_ = B["screen"]
    full = R["uq"]["full"]["screen"]
    if s_ and s_.get("set", "full") == "full":
        w(f"**Screening** at {s_['case']} (first-order variance, threshold 90 %, max d = 7): d = {s_['d']}, "
          f"{', '.join(s_['selected'])}. "
          + (f"90 % reached for every quantity. " if s_['reached'] else
             f"90 % not reached for every quantity within d = 7 (below 90 %: "
             + ", ".join(f"{k} {x:.3f}" for k, x in s_['coverage'].items() if x < 0.9) + "). ")
          + f"Flight-score coverage {s_['coverage']['flight_score']:.3f}. UDR with 3 points per "
          f"parameter gives N = 2d + 1 = {2 * s_['d'] + 1} nodes, each a full M2 + M3 analysis.\n")
    elif s_:
        w(f"**Contributions** at {s_['case']} (first-order variance (dq/dp sigma)^2 at the means; every parameter is "
          f"active, d = {s_['d']}, N = {2 * s_['d'] + 1} UDR nodes, so this is a table, not a selection). Means: "
          + ", ".join(f"{k} {x:g}" for k, x in s_["means"].items()) + ". sigma(4) and sigma(15) are the root of the "
          "summed variance over this set and over the full set's 15 (its screening at the same case); 'share of 15' = their variance "
          "ratio; the last columns are each parameter's share of this set's variance.\n\n")
        rows = []
        for q, c in s_["contrib"].items():
            tn = sum(c.values())
            to = sum(full["contrib"].get(q, {}).values()) if full else 0.0
            if tn == 0 and to == 0:
                continue
            rows.append([q.replace("points.", ""), _fmt(s_["values"].get(q), 4), f"{tn ** 0.5:.3g}",
                         f"{to ** 0.5:.3g}" if full else "–", _fmt(tn / to if to else None, 3)]
                        + [_fmt(c[k] / tn, 3) if tn else "–" for k in s_["selected"]])
        w(_table(["quantity", "value at the means", "sigma (this set)", "sigma (15)", "share of 15"]
                 + s_["selected"], rows))
    nc = B["node_check"]
    if nc:
        w("\n**Node check** (every node, both mission points; MDA failures with the warm-start fallbacks): "
          + "; ".join(f"{r['design']} {len(r['failed'])}/{nc['N']} failed" for r in nc["results"]) + ".")
        bad = [r for r in nc["results"] if len(r["failed"]) == nc["N"]]
        if bad:
            w(f" At {', '.join(r['design'] for r in bad)} the design's own nominal point does not converge (every "
              "node fails because the start state does): a design outside the analysis' convergence region, like the "
              "failed multistarts, not a node problem.")
        w("\n\n")
    rows = []
    for r in B["rdo"]:
        s = (r.get("summary") or {}).get("flight_score", {})
        fails = sum(x["failed_attempts"] for x in (r.get("nodes") or []))
        rows.append([r["tag"], r["start"], _fmt(r["ok"]), r["its"], _fmt((r["time"] or 0) / 3600, 2),
                     _fmt(s.get("mu"), 4), _fmt(s.get("sigma"), 4), _fmt(s.get("robust"), 4), fails,
                     (r.get("error") or "")[:60]])
    if rows:
        w(_table(["run", "start", "ok", "its", "wall (h)", "mu F", "sigma F", "mu - k sigma", "failed MDA attempts "
                  "(recovered or not)", "error"], rows))
    nom, designs, rb, mc = B["nominal"], B["designs"], B["rdo_best"], B["mc"]
    re_ = (B["robust_eval"] or {}).get("results", {})
    if re_:
        w("\n**The same UDR statistics at every design** (the RDO's nodes; robust constraints mu - beta sigma >= lower,"
          " mu + beta sigma <= upper, beta = 2):\n\n")
        rows = []
        det = {c["tag"]: c for c in ok}
        lab_nom = {**{t: c["F"] for t, c in det.items()},
                   **{d: nom.get(d, {}).get("score", {}).get("flight_score") for d in designs}}
        items = [(k, x) for k, x in re_.items() if "mu" in x]
        rank_nom = sorted((k for k, _ in items if k in det), key=lambda k: -det[k]["F"])
        rank_rob = sorted((k for k, _ in items if k in det), key=lambda k: -re_[k]["robust"])
        for k, x in items:
            rows.append([k, _fmt(lab_nom.get(k), 4), _fmt(x["mu"], 4), _fmt(x["sigma"], 4), _fmt(x["robust"], 4),
                         rank_nom.index(k) + 1 if k in rank_nom else "–",
                         rank_rob.index(k) + 1 if k in rank_rob else "–",
                         ", ".join(n.replace("points.", "") for n in x["robust_violated"]) or "none"])
        for k, x in re_.items():
            if "mu" not in x:
                rows.append([k, "", f"{x['failed']}/{x['N']} nodes failed", "", "", "", "", ""])
        if rb:
            r = next(x for x in B["rdo"] if x["tag"] == rb)
            s = r["summary"]["flight_score"]
            rows.append([f"**{rb}**", _fmt(nom.get("rdo", {}).get("score", {}).get("flight_score"), 4),
                         _fmt(s["mu"], 4), _fmt(s["sigma"], 4), _fmt(s["robust"], 4), "–", "–",
                         "none (constrained)" if r["ok"] else "see run"])
        w(_table(["design", "F nominal", "mu F", "sigma F", "mu - k sigma", "rank nominal", "rank robust",
                  "robust constraints violated"], rows))
        moved = [f"{k} {rank_nom.index(k) + 1} → {rank_rob.index(k) + 1}" for k in rank_nom
                 if rank_nom.index(k) != rank_rob.index(k)]
        w("\n**Propulsion ranking under uncertainty: " + ("unchanged" if not moved else
          f"the best case stays first ({rank_rob[0]}); swaps: {'; '.join(moved)}" if rank_rob[0] == rank_nom[0]
          else f"the best case changes to {rank_rob[0]}") + "** (ranked by mu - k sigma at each case's deterministic "
          "optimum; an RDO per case was not run).\n")
    if rb and "multipoint" in mc and "rdo" in mc:
        a, b2 = mc["multipoint"], mc["rdo"]
        w(f"\nMonte Carlo (this set): multipoint F mu {a['stats']['flight_score']['mu']:.4f} sigma "
          f"{a['stats']['flight_score']['sigma']:.4f} p5 {a['stats']['flight_score']['p5']:.4f}, max P(violation) "
          f"{max(a['p_violation'].values()):.3f}; RDO mu {b2['stats']['flight_score']['mu']:.4f} sigma "
          f"{b2['stats']['flight_score']['sigma']:.4f} p5 {b2['stats']['flight_score']['p5']:.4f}, max P(violation) "
          f"{max(b2['p_violation'].values()):.3f}.\n")


def _sec_variants(w, R):
    B = R["uq"]["reduced"]
    pr = B["propagation"]
    if pr:
        w(f"\n### 7a. Propagation check at the robust optimum ({pr['design']})\n\n")
        meths = pr["methods"]
        qs = list(meths["udr"]["stats"])
        rows = []
        for q in qs:
            rows.append([q.replace("points.", "")] + [
                f"{meths[m]['stats'][q]['mu']:.4g} / {meths[m]['stats'][q]['sigma']:.3g}" if q in meths[m]["stats"]
                else "–" for m in ("udr", "smolyak", "sobol")])
        w(_table(["quantity (mu / sigma)", f"UDR (N = {meths['udr']['N']})", f"Smolyak level 2 (N = "
                  f"{meths['smolyak']['N']})", f"Sobol' MC (N = {meths['sobol']['N']})"], rows))
        w("\nMDA failures: " + ", ".join(f"{m} {meths[m]['failed']}/{meths[m]['N']}" for m in meths) + ". Largest "
          "Monte Carlo P(violation): " + (", ".join(f"{k.replace('points.', '')} {x:.3f}" for k, x in sorted(
              meths["sobol"]["p_violation"].items(), key=lambda t: -t[1])[:4] if x > 0) or "none") + ".\n")
    w("\n### 7b. Calibrated build mass (dropped)\n\nThe Woombah-only f_build calibration (2.74, ±30 % about it) of the "
      "uncalibrated pages would count the build bias twice on the calibrated model, whose correction factors carry it "
      "(docs/studies/mass_calibration_usyd.md); it is in the pre-calibration pages (git tag `results-uncalibrated`).\n")
    for name, title, what in (("onesided", "7c. One-sided build mass (sensitivity)",
                               "f_build on [1.0, 1.3] (nominal 1.15 +/- 0.15: the reference node sits at a 15 % "
                               "heavier structure)"),):
        V = R["uq"][name]
        if not V["rdo"] and not V["screen"]:
            continue
        w(f"\n### {title}\n\n{what}; config `{V['cfg']}`.")
        if V["calibration"]:
            c = V["calibration"]
            w(f" Calibration: Woombah's modelled structure at f_build = 1 is {c['structure_at_1_kg'] / LB_:.2f} lb "
              f"against Table 34's {c['target_kg'] / LB_:.2f} lb (ratio {c['ratio_at_1']:.3f}); through the closure "
              f"(the spar and gear resize) f_build = **{c['f_build']:.4f}** reproduces it.")
        w("\n\n")
        rows = []
        for r in V["rdo"]:
            s = (r.get("summary") or {}).get("flight_score", {})
            rows.append([r["tag"], r["start"], _fmt(r["ok"]), r["its"], _fmt((r["time"] or 0) / 3600, 2),
                         _fmt(s.get("mu"), 4), _fmt(s.get("sigma"), 4), _fmt(s.get("robust"), 4),
                         (r.get("error") or "")[:60]])
        if rows:
            w(_table(["run", "start", "ok", "its", "wall (h)", "mu F", "sigma F", "mu - k sigma", "error"], rows))
            main = next((x for x in B["rdo"] if x["tag"] == B["rdo_best"]), None)
            r = next((x for x in V["rdo"] if x.get("ok")), None)
            if main and r:
                ch = [f"{k} {main['dv'][k]:.4g} → {r['dv'][k]:.4g}" for k in main["dv"]
                      if abs(r["dv"][k] - main["dv"][k]) > 1e-3 * max(abs(main["dv"][k]), 1e-6)]
                w("\nDesign change against the main robust optimum: " + ("; ".join(ch) or "none") + ".\n")


def render(R):
    L = []
    w = L.append
    sc = R["scoring"]
    base = R["best_case"]
    enum = R["enumeration"]
    ok = [c for c in enum if c["ok"]]
    best = max(ok, key=lambda c: c["F"])
    B = R["uq"]["reduced"]  # the main uncertain set; the published 15-parameter set is R["uq"]["full"]
    nom, own, mc = B["nominal"], _own_of(R, B), B["mc"]
    designs = B["designs"]
    w("# 2024-25 results: AirCrafter 1.0 vs multipoint vs RDO\n")
    w(f"Generated by `studies/results_2025.py report` (git `{R['git']}`) from `outputs/results/` (runs, screening, node "
      "check, Monte Carlo); every number below is read from `docs/studies/results/results_2025.json`. Configs: "
      "`configs/dbf2025_multipoint.yaml` (deterministic), `configs/dbf2025_multipoint_rdo.yaml` (robust: the reduced "
      "mass-and-wind set, f_build, f_tb, m_avionics and wind, docs/RDO_REDUCED_PLAN.md; the published 15-parameter set "
      f"is `configs/dbf2025_multipoint_rdo_full.yaml`, on the calibrated model in {FULL_NOTE}), "
      "`configs/woombah_2025.yaml` (Woombah as built). Scoring `dbf2025`: F = M1 + M2 + M3, "
      f"M2 = 1 + (W_fuel/t_3laps)/R2, M3 = 2 + (n_pre + B/W_X1)/R3 with **R2 = {sc['R2']:g} lb/s and R3 = "
      f"{sc['R3']:g} (PLACEHOLDERS, OPEN_QUESTIONS Q35)**, B = {sc.get('bonus', 0):g}. Validation of Woombah: "
      "`validation_2025.md`; AirCrafter 1.0's run: `aircrafter1_2025.json`.\n")
    w("**Biases stated up front.** (i) The lap model has no take-off, so every 3-lap time is fast and every M2 high "
      "(validation_2025.md). (ii) The common evaluation in the AirCrafter 2.0 analysis favours the AirCrafter 2.0 "
      "designs (they were optimised in it); the Monte Carlo favours RDO by construction (it optimised a statistic of "
      "the same model). (iii) MDA non-convergence is counted separately from constraint violation everywhere.\n")

    # ------------------------------------------------------------------ headline
    w("## Findings\n")
    F = []
    F.append(f"**Best propulsion case: {best['cells']:g}S, {best['props']['M2']} (M2) / {best['props']['M3']} (M3), "
             f"F = {best['F']:.4f}** (M2 {best['M2']:.4f}, M3 {best['M3']:.4f}, {best['fuel_lb']:.1f} lb of fuel, "
             f"{best['laps']:.2f} M3 laps). Kv is set by {best['kv_set_by']}.")
    capped = [c for c in ok if "fuel_mass (upper)" in c["bounds"]]
    if capped:
        F.append(f"**The fuel weight sits at its upper bound in {len(capped)} of {len(ok)} cases**: "
                 f"{best['fuel_lb']:.1f} lb, the capacity of the two 1 L bottles (OPEN_QUESTIONS Q36). M2 is "
                 "tank-limited, so the bottle choice (kept at Woombah's 1 L) is an active design decision: a larger "
                 "bottle buys M2 at the cost of tank drag and mass. The main results keep 1 L (decision); the bottle "
                 "sensitivity (section 1b) gives " + ", ".join(f"{b['bottle']} F = {b['F']:.4f} ({b['fuel_lb']:.1f} lb)"
                                                               for b in R["bottles"] if "F" in b) + ".")
    cells = sorted((c for c in ok if c["props"] == best["props"]), key=lambda c: c["cells"])
    if len(cells) > 1:
        F.append("Cell count barely matters: " + ", ".join(f"{c['cells']:g}S {c['F']:.4f}" for c in cells) +
                 f" (spread {max(c['F'] for c in cells) - min(c['F'] for c in cells):.4f}); Kv absorbs the voltage.")
    s6 = [c for c in ok if c["cells"] == best["cells"]]
    m2best = {}
    for c in s6:
        m2best.setdefault(c["props"]["M2"], []).append(c["F"])
    spread_m3 = max(max(v_) - min(v_) for v_ in m2best.values())
    spread_m2 = max(max(v_) for v_ in m2best.values()) - min(max(v_) for v_ in m2best.values())
    F.append(f"The M2 prop decides the ranking (best-per-M2-prop spread {spread_m2:.3f}); the M3 prop hardly moves F "
             f"(at most {spread_m3:.4f} for a given M2 prop): M3's laps are capped by the 100 Wh pack rule, not by "
             "the prop.")
    msr = R["multistart"]["runs"]
    if msr:
        good = [r for r in msr if r["ok"]]
        F.append(f"Multistart ({len(msr)} scrambled Sobol' starts): {len(good)}/{len(msr)} converge, all to "
                 f"F = {min(r['F'] for r in good):.4f}–{max(r['F'] for r in good):.4f} (default start "
                 f"{R['multistart']['default']:.4f}); the {len(msr) - len(good)} failures are MDA non-convergence "
                 "at a random start design, not optimiser failures.")
    bk = R["bookends"]
    if "M2only" in bk and "M3only" in bk and bk["M3only"].get("M2") is not None:
        F.append(f"Bookends: the M2-only optimum gains {bk['M2only']['M2'] - best['M2']:+.4f} in M2 and the "
                 f"M3-only optimum {bk['M3only']['M3'] - best['M3']:+.4f} in M3 over the multipoint design; the "
                 f"M3-only design gives up {best['M2'] - bk['M3only']['M2']:.3f} in M2 for it. **The multipoint "
                 "optimum is effectively the M2 optimum**: M2 (fuel/time) is the only mission with real leverage "
                 "at these normalisations.")
    F += _rdo_findings(R, base)
    w("".join(f"{i + 1}. {s}\n" for i, s in enumerate(F)))
    w("\n")

    # ------------------------------------------------------------------ enumeration
    w("## 1. Propulsion enumeration (deterministic multipoint)\n")
    w("Every run from the same start, SLSQP, warm-start MDA fallbacks. `ok` = SLSQP converged; a failed MDA is "
      "reported as such, never as infeasible. `its` = OpenMDAO driver iterations, i.e. objective evaluations "
      "(line-search steps included), here and in every table below.\n")
    rows = []
    for c in enum:
        if "F" in c:
            rows.append([c["tag"], f"{c['cells']:g}", c["props"]["M2"], c["props"]["M3"], _fmt(c["ok"]), c["its"],
                         _fmt(c["F"], 4), _fmt(c["M2"], 4), _fmt(c["M3"], 4), _fmt(c["fuel_lb"], 2), _fmt(c["t3"], 1),
                         _fmt(c["laps"], 2), _fmt(c["Kv"], 0), c["kv_set_by"], _fmt(c["M1_passed"])])
        else:
            rows.append([c["tag"], f"{c['cells']:g}", c["props"]["M2"], c["props"]["M3"], "no", c["its"],
                         c.get("error", "")[:60], "", "", "", "", "", "", "", ""])
    w(_table(["run", "cells", "M2 prop", "M3 prop", "ok", "its", "F", "M2", "M3", "fuel (lb)", "t3 (s)", "M3 laps",
              "Kv", "Kv set by", "M1 check"], rows))
    w("\nActive constraints at the best case: " + ", ".join(best["active"]) + "; at bounds: " +
      ", ".join(best["bounds"]) + ".\n")
    m1f = [c for c in ok if not c["M1_passed"]]
    if m1f:
        w(f"\n**Flag: the M1 check (no payload, M3's prop/pack/throttle) fails at {len(m1f)} of {len(ok)} optima.** "
          "Violated: " + "; ".join(sorted({", ".join(c["M1_violated"] or []) for c in m1f})) + ". M1 is scored 1 "
          "regardless (analysis-only check, RESULTS_PLAN §9); v_fit_margin is a model-validity limit (the light M1 "
          "aircraft outruns the propeller fit range at M3's throttle), and an SM just above its upper bound without "
          "payload is fixed by ballast or the battery station on the day.\n")
    w("\n![enumeration](figures/results_enumeration.png)\n")
    if R["bottles"]:
        w("\n### 1b. Store option: the external bottle (best propulsion case)\n")
        w("Same case, re-optimised from the 1 L optimum with each candidate bottle (dimensions and empty masses are "
          "PLACEHOLDERS, OPEN_QUESTIONS Q36). Sensitivity only: the main results use Woombah's 1 L.\n\n")
        rows = [[b["bottle"], _fmt(b["ok"]), b["its"], _fmt(b.get("F"), 4), _fmt(b.get("M2"), 4), _fmt(b.get("M3"), 4),
                 _fmt(b.get("fuel_lb"), 2), _fmt(b.get("t3"), 1), _fmt(b.get("laps"), 2), _fmt(b.get("m2_total"), 2),
                 _fmt(b.get("cd_stores"), 4), ", ".join(b.get("bounds", [])) or (b.get("error") or "")[:50],
                 _fmt(b.get("M1_passed"))] for b in R["bottles"]]
        w(_table(["bottle", "ok", "its", "F", "M2", "M3", "fuel (lb)", "t3 (s)", "M3 laps", "M2 mass (kg)",
                  "M2 C_D stores", "at bounds", "M1 check"], rows))
        capped_b = [b for b in R["bottles"] if "fuel_mass (upper)" in b.get("bounds", [])]
        if len(capped_b) == len([b for b in R["bottles"] if "F" in b]):
            w("\n**The fuel fills the tanks for every bottle**: within the model, M2 keeps rising with tank capacity "
              "(fuel weight grows faster than the 3-lap time), and nothing in the 2024-25 rules but the 55 lb TOGW "
              "and the span bounds it (no take-off distance limit, RESULTS_PLAN §2). The tank size, and the number of "
              "tanks (fixed at 2 here), are the largest levers in the problem; the 1 L result is a constrained "
              "optimum, not the problem's.\n")

    # ------------------------------------------------------------------ method comparison
    w("## 2. Method comparison (the AirCrafter 2.0 multipoint analysis at nominal inputs)\n")
    w("Woombah: as built, at the throttles that reproduce its logged currents, on the calibrated mass model (in-sample: "
      "Woombah is one of the three calibration aircraft; the out-of-sample check is validation_2025.md). AirCrafter 1.0 "
      "direct = mapping (a), completed = mapping (b) (span, chord and fuel fixed at AirCrafter 1.0's values, the rest "
      f"optimised). Multipoint = {base}. RDO = {B['rdo_best'] or 'not run'} (reduced set; the 15-parameter RDO is in "
      f"{FULL_NOTE})."
      + (f" **AirCrafter 1.0's fuel ({own['ac1_direct']['fuel_lb']:.1f} lb) does not fit in two 1 L bottles, so both "
         "mappings fly 2 L bottles** "
         "(smallest bottle that holds it); the like-for-like AirCrafter 2.0 design is multipoint_2L (the same case "
         "re-optimised with 2 L bottles, section 1b)." if "multipoint_2L" in designs else "") + "\n")
    labs = [d for d in designs if nom.get(d, {}).get("ok")]
    w("### Design variables and geometry\n")
    rows = [[name] + [_fmt(nom[d]["dv"].get(k), 3) for d in labs] for k, name in GEOM]
    rows.append(["aspect ratio b/c"] + [_fmt(nom[d]["dv"]["b"] / nom[d]["dv"]["c"], 2) for d in labs])
    rows.append(["fuel (lb)"] + [_fmt(nom[d]["fuel_lb"], 2) for d in labs])
    rows.append(["cells"] + [f"{nom[d]['cells']:g}" for d in labs])
    rows.append(["M2 / M3 prop"] + [f"{nom[d]['props']['M2']} / {nom[d]['props']['M3']}" for d in labs])
    w(_table(["quantity"] + labs, rows))
    w("\n### Mass breakdown (kg)\n")
    rows = [[f"{pt} {k}"] + [_fmt(nom[d][pt].get(k), 3) for d in labs] for pt in ("M2", "M3") for k in MASSES]
    w(_table(["mass"] + labs, rows))
    w("\n### Performance, scores and margins\n")
    rows = [[f"{pt} {name}"] + [_fmt(nom[d][pt].get(k), nd) for d in labs] for pt in ("M2", "M3")
            for k, name, nd in PERF]
    rows += [[k] + [_fmt(nom[d]["score"][k], 4) for d in labs] for k in ("M2", "M3", "flight_score")]
    fixed_kv = {"woombah": "fixed (as built)", "ac1_direct": "fixed (AirCrafter 1.0)",
                "rdo": "robust M2 rpm_margin (mu - 2 sigma)"}
    rows.append(["Kv set by"] + [fixed_kv.get(d, nom[d]["kv_set_by"]) for d in labs])
    rows.append(["M1 check"] + [("pass" if nom[d]["M1_check"]["passed"] else
                                 "fail: " + ", ".join(nom[d]["M1_check"].get("violated") or
                                                      [nom[d]["M1_check"].get("error", "")])) for d in labs])
    w(_table(["quantity"] + labs, rows))
    w("\n### Constraint status (active within 1e-4 of ref, or violated)\n")
    cons = R["constraints"]
    rows = []
    for d in labs:
        viol = []
        for n, kw in cons.items():
            x = nom[d]["M2" if n.startswith("points.M2") else "M3"].get(n.split(".", 2)[-1]) \
                if n.startswith("points.") else None
            if x is None:
                continue
            ref = kw.get("ref") or 1.0
            if (kw.get("lower") is not None and (x - kw["lower"]) / ref < -1e-4) or \
                    (kw.get("upper") is not None and (x - kw["upper"]) / ref > 1e-4):
                viol.append(f"{n} = {x:.3g}")
        rows.append([d, "; ".join(nom[d]["active_constraints"]) or "–", "; ".join(nom[d]["active_bounds"]) or "–",
                     "; ".join(viol) or "none (of the reported outputs)"])
    w(_table(["design", "active constraints", "at bounds", "violated"], rows))
    w("\nViolations are checked here only for the outputs recorded per point (the table above); the Monte Carlo "
      "(section 3) checks every constraint.\n")

    # ------------------------------------------------------------------ common evaluation
    w("## 3. Common evaluation\n")
    _sec_common(w, R, R["uq"]["reduced"], "")
    w(f"\nThe common evaluation over the full 15-parameter set (the published section 3b) is not repeated here: on the "
      f"calibrated model it covers the multipoint and RDO designs only, in {FULL_NOTE}.\n")
    # ------------------------------------------------------------------ bookends
    w("## 4. Mission bookends\n")
    w("The same multipoint problem (both missions flown, every constraint at both points) with the objective reduced "
      "to one mission's score.\n")
    rows = [["multipoint", "yes", R["enumeration"][[c["tag"] for c in enum].index(base)]["its"], _fmt(best["F"], 4),
             _fmt(best["M2"], 4), _fmt(best["M3"], 4), _fmt(best["fuel_lb"], 2), _fmt(best["t3"], 1),
             _fmt(best["laps"], 2)]]
    for s, d in bk.items():
        rows.append([s, _fmt(d["ok"]), d["its"], _fmt(d.get("flight_score"), 4), _fmt(d.get("M2"), 4),
                     _fmt(d.get("M3"), 4), _fmt(d.get("fuel_lb"), 2), _fmt(d.get("t3"), 1), _fmt(d.get("laps"), 2)])
    w(_table(["objective", "ok", "its", "F", "M2", "M3", "fuel (lb)", "t3 (s)", "M3 laps"], rows))
    if "M3only" in bk and not bk["M3only"]["ok"]:
        w(f"\nThe M3-only run stopped at its iteration limit ({bk['M3only']['maxiter']}) without SLSQP's convergence "
          "flag: M3's objective is nearly flat in the airframe (the laps are capped by the 100 Wh rule), so the "
          "optimiser wanders on a plateau. Its M3 is what it reached, a lower bound on the M3-only optimum.\n")
    if "M3only" in bk and bk["M3only"].get("dv"):
        d3 = bk["M3only"]["dv"]
        w("\nM3-only design vs multipoint: " + ", ".join(
            f"{k} {best_dv:.3g} → {d3[k]:.3g}" for k, best_dv in load(base)["dv"].items()
            if abs(d3[k] - best_dv) > 0.02 * max(abs(best_dv), 1e-3)) + ".\n")

    # ------------------------------------------------------------------ normalisation
    w("## 5. Normalisation sensitivity\n")
    w(f"R2 and R3 scaled by 0.5, 1 and 2 (R2 {0.5 * sc['R2']:g}–{2 * sc['R2']:g} lb/s, R3 {0.5 * sc['R3']:g}–"
      f"{2 * sc['R3']:g}); each re-optimised from the centre optimum, deterministic multipoint, {base}. A larger R "
      "makes that mission worth less.\n")
    g = R["normalisation"]["grid"]
    c0 = next(r for r in g if r["f2"] == 1 and r["f3"] == 1)
    keys = ["b", "c", "Kv", "capacity_M2", "capacity_M3", "throttle_M2", "throttle_M3"]
    rows = []
    for r in g:
        if "F" not in r:
            rows.append([f"x{r['f2']:g}", f"x{r['f3']:g}", "no", r["its"], r.get("error", "")[:50]] + [""] * (5 + len(keys)))
            continue
        rows.append([f"x{r['f2']:g}", f"x{r['f3']:g}", _fmt(r["ok"]), r["its"], _fmt(r["F"], 4), _fmt(r["M2"], 4),
                     _fmt(r["M3"], 4), _fmt(r["fuel_lb"], 2), _fmt(r["t3"], 1), _fmt(r["laps"], 2)]
                    + [_fmt(r["dv"][k], 3) for k in keys])
    w(_table(["R2", "R3", "ok", "its", "F", "M2", "M3", "fuel (lb)", "t3 (s)", "M3 laps"] + keys, rows))
    moves = []
    for r in g:
        if "F" not in r or (r["f2"] == r["f3"]):
            continue
        dfuel = r["fuel_lb"] - c0["fuel_lb"]
        dlaps = r["laps"] - c0["laps"]
        moves.append((r["f2"], r["f3"], dfuel, dlaps, r["t3"] - c0["t3"]))
    same = [r for r in g if "F" in r and r["f2"] == r["f3"] and r["f2"] != 1]
    w("\n" + (f"Equal scaling of R2 and R3 ({', '.join(f'x{r['f2']:g}' for r in same)}) scales the whole variable "
              "part of F, so the optimum cannot move; those runs stopped in "
              f"{', '.join(str(r['its']) for r in same)} iterations at the centre design, as they should. " if same
              else ""))
    mx = max(moves, key=lambda m: abs(m[2]) + abs(m[3])) if moves else None
    if moves:
        w("Off-diagonal: " + "; ".join(f"R2 x{a:g} / R3 x{b:g}: fuel {d:+.2f} lb, M3 laps {l_:+.2f}, t3 {t:+.1f} s"
                                      for a, b, d, l_, t in moves) +
          f". **Largest move: R2 x{mx[0]:g} / R3 x{mx[1]:g}.** "
          + ("The design barely moves: the M2 and M3 levers are nearly decoupled (fuel weight sets M2; M3's laps "
             "are capped by the energy rule), so the normalisation re-weights the score but not the aircraft."
             if max(abs(m[2]) for m in moves) < 0.5 and max(abs(m[3]) for m in moves) < 0.2 else
             "Making M3 relatively worth more (R3 smaller or R2 larger) moves the design towards M3 (less fuel, "
             "more laps) and vice versa; see the table for the size of each move.") + "\n")
    w("\n![normalisation](figures/results_normalisation.png)\n")

    # ------------------------------------------------------------------ bonus
    b_ = R["bonus"]
    w("## 6. Bonus scenario\n")
    w(f"B = {b_['B']:g} at W_X1 = {b_['x1_lb']:g} lb adds B/W_X1/R3 = {b_['dM3']:.4f} to M3 for every design. W_X1 is "
      "fixed (the X-1 is not a design variable here), so the term is a constant: **the optimum does not change**, "
      "only the totals.\n")
    rows = []
    for d in designs:
        f0 = nom.get(d, {}).get("score", {}).get("flight_score")
        rows.append([d, _fmt(f0, 4), _fmt(f0 + b_["dM3"] if f0 is not None else None, 4)])
    w(_table(["design", "F (B = 0)", "F (B = 2.5)"], rows))
    w("\n")

    # ------------------------------------------------------------------ robust
    w("## 7. Robust vs deterministic\n")
    _sec_robust(w, R, R["uq"]["reduced"], ok)
    _sec_variants(w, R)
    w(f"\nThe RDO over the full 15-parameter set (the published section 7d) on the calibrated model: {FULL_NOTE}.\n")
    w("\n## Caveats\n")
    w("- R2, R3 are PLACEHOLDERS (Q35); section 5 shows what moves with them.\n"
      "- No take-off in the lap model (switch `course.takeoff.in_mission_time`, off here). The bias is not uniform: it favours heavy designs (uncalibrated model, not rerun: re-analysis with take-off on: ΔF −0.008 for the 0.5 L optimum to −0.053 for 2 L, −0.020 multipoint, −0.021 RDO; no ranking change at nominal inputs). With take-off on, every optimum here violates M2's energy constraint and the RDO design violates two robust constraints. Re-optimised with take-off on, the best case is feasible again at F 5.1200 and the RDO design meets every robust constraint and stays ahead in mu − kσ (docs/studies/takeoff_study.md §9).\n"
      "- Bottle dimensions and masses, the store interference factors and the store drag spread are PLACEHOLDERS "
      "(Q36, Q37); every uncertainty spread is a PLACEHOLDER (configs/uncertainty.yaml).\n"
      "- The mass model is the USYD-calibrated one (docs/studies/mass_calibration_usyd.md). Woombah is one of its three "
      "calibration aircraft, so its column here is in-sample; the out-of-sample check (factors fitted without Woombah) "
      "is validation_2025.md.\n"
      "- The full ranked list of placeholders affecting the results is in INDEX.md.\n")
    return "\n".join(L)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("part", choices=["continuity", "calibrate", "propagation", "classify", "optimise", "bookends", "screen", "node_check", "rdo", "multistart",
                                     "normalisation", "mc", "common", "robust_eval", "report"])
    ap.add_argument("--set", choices=[*UQ_SETS, "final"], default="reduced",
                    help="uncertain set of the robust parts (default the reduced mass-and-wind set)")
    ap.add_argument("--config", help="deterministic config, relative to the repo root (default dbf2025_multipoint.yaml)")
    ap.add_argument("--rdo-config", help="robust config of the 'final' uncertain set")
    ap.add_argument("--woombah-config", help="Woombah as built (default woombah_2025.yaml)")
    ap.add_argument("--out", help="output folder, relative to the repo root (default outputs/results)")
    ap.add_argument("--docs", help="page folder, relative to the repo root")
    ap.add_argument("--labels", nargs="*")
    ap.add_argument("--index", type=int)
    ap.add_argument("--f2", type=float)
    ap.add_argument("--f3", type=float)
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
    ap.add_argument("--workers", type=int, default=5, help="processes of the sampled evaluations (mc, propagation)")
    ap.add_argument("--bottle", help="external tank bottle: a key of stores.BOTTLES (default the config's 1 L)")
    a = ap.parse_args()
    use_paths(a.out, a.config, a.rdo_config, a.woombah_config, a.docs)
    use_set(a.set)
    if a.part == "continuity":
        continuity()
    elif a.part == "calibrate":
        calibrate()
    elif a.part == "optimise":
        sc = {k: getattr(a, k) for k in ("w2", "w3", "R2", "R3") if getattr(a, k) is not None}
        optimise(a.tag, a.cells, a.m2, a.m3, sc or None, a.start, bottle=a.bottle)
    elif a.part == "bookends":
        bookends(a.case)
    elif a.part == "screen":
        screen(a.case)
    elif a.part == "node_check":
        node_check(a.case)
    elif a.part == "rdo":
        rdo(a.tag, a.case, a.start, index=a.index)
    elif a.part == "mc":
        monte_carlo(mc_designs(a.labels), workers=a.workers)
        r = best_rdo()
        if UQ["name"] not in ("full", "full_usyd") and r and (a.labels is None or "rdo_full15" in a.labels):
            # out-of-set check: the new robust design under the full 15-parameter set
            monte_carlo([("rdo_full15", _mcfg_for(load(r["case"]), cfg=RDO_FULL_CFG), r["dv"])], workers=a.workers)
    elif a.part == "propagation":
        propagation(workers=a.workers)
    elif a.part == "classify":
        classify_mc_failures()
    elif a.part == "multistart":
        multistart(a.case, a.index)
    elif a.part == "normalisation":
        normalisation(a.case, a.f2, a.f3)
    elif a.part == "common":
        common(a.labels)
    elif a.part == "robust_eval":
        robust_eval(workers=a.workers)
    else:
        report()
