"""Computational performance of AirCrafter 1.0 (MATLAB grid sweep) and AirCrafter 2.0 (multipoint MDF + SLSQP, RDO)
on the 2024-25 problem (docs/studies/results/compute_performance.md).

    .venv/bin/python studies/compute_performance.py run        # every timed measurement (resumable), ~4-5 h
    .venv/bin/python studies/compute_performance.py refine     # AirCrafter 1.0 local grid refinement (~25 min)
    .venv/bin/python studies/compute_performance.py report

Protocol (docs/RESULTS_PLAN.md, the brief's "Computational performance protocol"): one machine, nothing else running,
on a clean committed tree, after the results runs. Each measurement is its own process: single-threaded
(OMP/OPENBLAS/MKL_NUM_THREADS = 1, MATLAB -singleCompThread) for the main table, plus one row per method at default
threading. Median and range of 5 repeats for runs under 10 min, 3 for longer. AirCrafter 1.0's per-point progress
print is suppressed for the timed runs (opts.quiet). AirCrafter 2.0 counts MDA solves, Newton iterations, linear
solves and factorisations, compute_totals calls, model runs and optimiser iterations with counters on the solver and
driver objects (not estimates).

Raw measurements: outputs/results/perf/<name>_<repeat>.json; the report reads them and the results runs
(outputs/results/runs: multistart reliability, the enumeration's iteration counts).
"""

import argparse
import json
import os
import platform
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PERF = ROOT / "outputs" / "results" / "perf"
RUNS = ROOT / "outputs" / "results" / "runs"
DOCS = ROOT / "docs" / "studies" / "results"
CASE = "mp_6S_11x8E_10x7E"     # the best deterministic case (results_2025.md)
THREAD_VARS = ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS")
DV_SETS = {4: ["c", "Kv", "throttle_M2", "capacity_M3"],
           7: ["c", "Kv", "throttle_M2", "capacity_M3", "tc_scale", "S_ht", "throttle_M3"]}
RDO_D = (3, 5, 7)
RDO_MAXITER = 3                # SLSQP iterations per timed RDO run (cost per iteration; a full run is ~hours)


# ------------------------------------------------------------------------------------------------ child process
def _instrument(prob):
    """Counters on every Newton solver, linear solver and the driver (instance-attribute wrappers)."""
    import openmdao.api as om

    C = dict(mda_solves=0, newton_iterations=0, linear_solves=0, factorisations=0, compute_totals=0, model_runs=0)
    seen = set()

    def wrap(obj, attr, key):
        f = getattr(obj, attr)

        def g(*a, **k):
            C[key] += 1
            return f(*a, **k)
        setattr(obj, attr, g)

    for s in prob.model.system_iter(include_self=True, recurse=True):
        ns = s.nonlinear_solver
        if isinstance(ns, om.NewtonSolver) and id(ns) not in seen:
            seen.add(id(ns))
            wrap(ns, "solve", "mda_solves")
            wrap(ns, "_single_iteration", "newton_iterations")
        for ls in (s.linear_solver, getattr(ns, "linear_solver", None)):
            if ls is not None and not isinstance(ls, om.LinearRunOnce) and id(ls) not in seen:
                seen.add(id(ls))
                wrap(ls, "solve", "linear_solves")
                if hasattr(ls, "_linearize"):
                    wrap(ls, "_linearize", "factorisations")
    wrap(prob.model, "run_solve_nonlinear", "model_runs")
    if prob.driver is not None and hasattr(prob.driver, "_compute_totals"):
        wrap(prob.driver, "_compute_totals", "compute_totals")
    return C


def _nit(driver):
    """SLSQP major iterations (scipy's nit); driver.iter_count counts objective evaluations."""
    r = getattr(driver, "_scipy_optimize_result", None)
    return int(r.nit) if r is not None and hasattr(r, "nit") else None


def child_det(dvs=None, maxiter=None):
    """The best case's deterministic multipoint optimisation from the config's default start."""
    sys.path.insert(0, str(ROOT / "studies"))
    import openmdao.api as om
    from openmdao.core.analysis_error import AnalysisError
    from results_2025 import config, load

    from aircrafter2.problems.multipoint import (add_multipoint_driver, apply_multipoint_values,
                                                 build_multipoint_model)

    run = load(CASE)
    mcfg = config(run["cells"], run["props"]["M2"], run["props"]["M3"])
    if maxiter:
        mcfg.base.maxiter = maxiter
    only = DV_SETS[dvs] if dvs else None
    t0 = time.perf_counter()
    prob = om.Problem(reports=False)
    build_multipoint_model(prob.model, mcfg, mda_retry=True)
    add_multipoint_driver(prob, mcfg, only=only)
    prob.driver.options["disp"] = False
    prob.setup()
    apply_multipoint_values(prob, mcfg)
    if only:  # the other design variables pinned at the full optimum
        for k, x in run["dv"].items():
            if k not in only:
                prob.set_val(k, x)
    t_setup = time.perf_counter() - t0
    C = _instrument(prob)
    t0 = time.perf_counter()
    err = None
    try:
        res = prob.run_driver()
        ok, its = bool(res.success), int(res.iter_count)
    except AnalysisError as exc:
        ok, its, err = False, int(prob.driver.iter_count), str(exc)[:200]
    wall = time.perf_counter() - t0
    return dict(ok=ok, iterations=_nit(prob.driver), function_evaluations=its, error=err, wall_s=wall, setup_s=t_setup,
                n_dv=len(only or mcfg.all_dvs()),
                F=float(prob.get_val("flight_score")[0]), **C)


def child_rdo(d, maxiter=RDO_MAXITER, uqset="full"):
    """RDO from the deterministic optimum with the first d screened parameters of an uncertain set (results_2025.py
    --set: the published full set screened to 7, or the reduced mass-and-wind set of 4), `maxiter` SLSQP iterations
    (0: the config's maxiter, i.e. a complete run)."""
    sys.path.insert(0, str(ROOT / "studies"))
    from openmdao.core.analysis_error import AnalysisError
    from results_2025 import UQ, _mcfg_for, load, use_set

    from aircrafter2.robustness.rdo_multipoint import build_robust_multipoint

    use_set(uqset)
    sc = json.loads((UQ["out"] / "screen.json").read_text())
    run = load(sc["case"])
    mcfg = _mcfg_for(run)
    mcfg.base.robust.active = sc["selected"][:d]
    if maxiter:
        mcfg.base.maxiter = maxiter
    t0 = time.perf_counter()
    p = build_robust_multipoint(mcfg, driver=True, extra_robust=("points.M2.laps_frac",))
    p.driver.options["disp"] = False
    for k, x in run["dv"].items():
        p.set_val(k, x)
    t_setup = time.perf_counter() - t0
    C = _instrument(p)
    t0 = time.perf_counter()
    err = None
    try:
        res = p.run_driver()
        its = int(res.iter_count)
    except AnalysisError as exc:
        its, err = int(p.driver.iter_count), str(exc)[:200]
    wall = time.perf_counter() - t0
    from results_2025 import robust_summary_mp_safe

    return dict(d=d, set=uqset, N=p.rdo["N"], iterations=_nit(p.driver), function_evaluations=its, error=err,
                wall_s=wall, setup_s=t_setup, robust=robust_summary_mp_safe(p), **C)


def child_quad(rule):
    """UDR (the RDO's rule) or Smolyak level 2 over the screened set at the deterministic optimum: node count,
    moments of the flight score, wall time (1 worker: single-threaded sampling)."""
    import numpy as np

    sys.path.insert(0, str(ROOT / "studies"))
    from results_2025 import OUT, RDO_FULL_CFG, _mcfg_for, load

    from aircrafter2.problems.multipoint import build_multipoint_problem
    from aircrafter2.robustness.quadrature import make_rule
    from aircrafter2.robustness.rdo_multipoint import resolved_dists_mp
    from aircrafter2.robustness.sampling import evaluate

    sc = json.loads((OUT / "screen.json").read_text())  # the published full set (the d = 7 screening)
    run = load(sc["case"])
    mcfg = _mcfg_for(run, cfg=RDO_FULL_CFG)
    rob = mcfg.base.robust
    rob.active = sc["selected"]
    rob.quadrature = rule
    if rule == "smolyak":
        rob.smolyak_level = 2
    p = build_multipoint_problem(mcfg, driver=False)
    dists, act = resolved_dists_mp(mcfg, lambda n: float(np.ravel(p.get_val(f"points.M2.{n}"))[0]))
    r = make_rule(rob, act)
    base = {d_.name: d_.mean for d_ in dists}
    rows = [dict(base, **{d_.name: float(x) for d_, x in zip(act, row)}) for row in r.nodes]
    ev = evaluate(mcfg, run["dv"], rows, ["flight_score"], workers=1)
    ok = ~np.isnan(ev["values"][:, 0])
    mu, var = r.moments(np.where(ok, ev["values"][:, 0], 0.0)) if ok.all() else (float("nan"), float("nan"))
    return dict(rule=rule, N=r.n, d=len(act), failed=len(ev["failed"]), wall_s=ev["wall"], mu=mu,
                sigma=float(np.sqrt(max(var, 0.0))) if np.isfinite(var) else float("nan"), psd=bool(r.psd))


# ------------------------------------------------------------------------------------------------ orchestration
def _env(single):
    env = dict(os.environ)
    for k in THREAD_VARS:
        if single:
            env[k] = "1"
        else:
            env.pop(k, None)
    return env


def _measure(name, repeat, argv, single=True, timeout=6 * 3600):
    """One measurement in a fresh process; outputs/results/perf/<name>_<repeat>.json (skipped if present)."""
    PERF.mkdir(parents=True, exist_ok=True)
    f = PERF / f"{name}_{repeat}.json"
    if f.exists():
        return json.loads(f.read_text())
    t0 = time.perf_counter()
    out = subprocess.run([sys.executable, __file__, "child", *argv], cwd=ROOT, env=_env(single), capture_output=True,
                         text=True, timeout=timeout)
    line = [x for x in out.stdout.splitlines() if x.startswith("RESULT ")]
    if not line:
        raise RuntimeError(f"{name}: no result\n{out.stderr[-2000:]}")
    res = json.loads(line[-1][7:])
    res.update(process_s=time.perf_counter() - t0, single_thread=single, name=name, repeat=repeat)
    f.write_text(json.dumps(res, indent=1))
    print(f"{name} #{repeat}: {res.get('wall_s', 0):.1f} s", flush=True)
    return res


def _ac1(name, repeat, refine=1, single=True, box=None):
    PERF.mkdir(parents=True, exist_ok=True)
    f = PERF / f"{name}_{repeat}.json"
    if f.exists():
        return json.loads(f.read_text())
    env = dict(os.environ, AC1_REFINE=str(refine), AC1_TIME_OUT=str(f))
    if box:
        env["AC1_BOX"] = ",".join(f"{x:g}" for x in box)
    cmd = ["matlab"] + (["-singleCompThread"] if single else []) + ["-batch",
                                                                    "run('reference/aircrafter1/time_2025.m')"]
    t0 = time.perf_counter()
    subprocess.run(cmd, cwd=ROOT, env=env, check=True, capture_output=True, timeout=48 * 3600)
    res = json.loads(f.read_text())
    res.update(process_s=time.perf_counter() - t0, single_thread=single, name=name, repeat=repeat)
    f.write_text(json.dumps(res, indent=1))
    print(f"{name} #{repeat}: sweep {res['time_sweep_s']:.0f} s", flush=True)
    return res


def refine_boxes():
    """Local grid refinement boxes (the full grid at 2x is 16x the points, ~6.5 h; at 4x 256x). 'top20': the extents
    of AirCrafter 1.0's 20 best design points plus one default step (mass 0.5 kg, span 0.1 m, AR 0.5), every speed;
    'dp': the selected point +/- 2 default steps, speeds 28-50 m/s (both missions' best speeds of the top 20 inside)."""
    top = json.loads((ROOT / "docs/studies/results/aircrafter1_2025.json").read_text())["aircrafter1_2025"]["top"]
    dp = top[0]
    ext = lambda k: (min(t[k] for t in top), max(t[k] for t in top))
    (m0, m1), (b0, b1), (a0, a1) = ext("M2Mass"), ext("Wingspan"), ext("AspectRatio")
    v = (min(min(t["M2Speed"], t["M3Speed"]) for t in top), max(max(t["M2Speed"], t["M3Speed"]) for t in top))
    clip = lambda lo, hi, glo, ghi: (max(lo, glo), min(hi, ghi))
    top20 = (*clip(m0 - 0.5, m1 + 0.5, 5, 24.5), *clip(b0 - 0.1, b1 + 0.1, 0.5, 1.8),
             *clip(a0 - 0.5, a1 + 0.5, 3, 12), 10, 50)
    dpb = (*clip(dp["M2Mass"] - 1, dp["M2Mass"] + 1, 5, 24.5), *clip(dp["Wingspan"] - 0.2, dp["Wingspan"] + 0.2, 0.5, 1.8),
           *clip(dp["AspectRatio"] - 1, dp["AspectRatio"] + 1, 3, 12), *clip(v[0] - 2, v[1] + 3, 10, 50))
    return {"top20": (top20, (1, 2)), "dp": (dpb, (1, 2, 4))}


def run_refine():
    for tag, (box, levels) in refine_boxes().items():
        for r in levels:
            _ac1(f"ac1_box_{tag}_r{r}", 0, refine=r, box=box)


def _repeats(first_wall):
    return 5 if first_wall < 600 else 3


def run_reduced():
    """The reduced mass-and-wind set (docs/RDO_REDUCED_PLAN.md): RDO at d = 4 (RDO_MAXITER SLSQP iterations, 5
    repeats) for the cost-against-d table, and one complete RDO from the deterministic optimum (the main run, clean)."""
    r = _measure("rdor_d4", 0, ["rdo", "--d", "4", "--set", "reduced"])
    for i in range(1, _repeats(r["wall_s"])):
        _measure("rdor_d4", i, ["rdo", "--d", "4", "--set", "reduced"])
    _measure("rdor_main", 0, ["rdo", "--d", "4", "--set", "reduced", "--maxiter", "0"])


def run_all():
    """Every measurement of the main table, the threading rows and the scaling studies (resumable)."""
    r = _measure("ac2_det", 0, ["det"])
    for i in range(1, _repeats(r["wall_s"])):
        _measure("ac2_det", i, ["det"])
    _measure("ac2_det_default_threads", 0, ["det"], single=False)
    for n in DV_SETS:
        r = _measure(f"ac2_det_{n}dv", 0, ["det", "--dvs", str(n)])
        for i in range(1, _repeats(r["wall_s"])):
            _measure(f"ac2_det_{n}dv", i, ["det", "--dvs", str(n)])
    for d in RDO_D:
        r = _measure(f"rdo_d{d}", 0, ["rdo", "--d", str(d)])
        for i in range(1, _repeats(r["wall_s"])):
            _measure(f"rdo_d{d}", i, ["rdo", "--d", str(d)])
    _measure("rdo_d7_default_threads", 0, ["rdo", "--d", "7"], single=False)
    for rule in ("udr", "smolyak"):
        _measure(f"quad_{rule}", 0, ["quad", "--rule", rule])
    r = _ac1("ac1", 0)
    for i in range(1, _repeats(r["time_sweep_s"])):
        _ac1("ac1", i)
    _ac1("ac1_default_threads", 0, single=False)


def machine():
    import numpy
    import openmdao
    import scipy

    def sh(cmd):
        try:
            return subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=60).stdout.strip()
        except Exception:
            return ""

    ac1 = sorted(PERF.glob("ac1_*.json"))
    return dict(cpu=sh("lscpu | grep 'Model name' | sed 's/.*: *//'"), cores=os.cpu_count(),
                ram_GB=round(os.sysconf("SC_PAGE_SIZE") * os.sysconf("SC_PHYS_PAGES") / 1e9, 1),
                os=f"{platform.system()} {platform.release()}", python=platform.python_version(),
                openmdao=openmdao.__version__, numpy=numpy.__version__, scipy=scipy.__version__,
                blas=sh(f"{sys.executable} -c \"import numpy; numpy.show_config()\" | grep -i -m1 'name'"),
                matlab=json.loads(ac1[0].read_text()).get("matlab") if ac1 else None,
                git=sh(f"git -C {ROOT} rev-parse --short HEAD"),
                tree_clean=sh(f"git -C {ROOT} status --porcelain --untracked-files=no") == "",
                uncommitted=[ln.split(maxsplit=1)[-1] for ln in sh(f"git -C {ROOT} status --porcelain --untracked-files=no").splitlines()])


# ------------------------------------------------------------------------------------------------ report
def _load(name):
    import re

    pat = re.compile(rf"{re.escape(name)}_\d+\.json")  # ac2_det must not pick up ac2_det_4dv_*
    return [json.loads(f.read_text()) for f in sorted(PERF.glob(f"{name}_*.json")) if pat.fullmatch(f.name)]


def _stat(rows, key):
    import numpy as np

    x = np.array([r[key] for r in rows if r.get(key) is not None], float)
    if not len(x):
        return None
    return dict(median=float(np.median(x)), min=float(x.min()), max=float(x.max()), n=len(x))


def collect():
    R = dict(machine=machine())
    names = ["ac2_det", "ac2_det_default_threads", "ac1", "ac1_default_threads", "quad_udr",
             "quad_smolyak"] + [f"ac2_det_{n}dv" for n in DV_SETS] + [f"rdo_d{d}" for d in RDO_D] + \
        ["rdo_d7_default_threads", "rdor_d4", "rdor_main"]
    R["raw"] = {n: _load(n) for n in names}
    S = {}
    for n, rows in R["raw"].items():
        if not rows:
            continue
        keys = [k for k, x in rows[0].items() if isinstance(x, (int, float)) and not isinstance(x, bool)]
        S[n] = {k: _stat(rows, k) for k in keys}
    R["stats"] = S

    # reliability and the complete study, from the results runs
    def rl(tag):
        f = RUNS / f"{tag}.json"
        return json.loads(f.read_text()) if f.exists() else None

    base = rl(CASE)
    ms = [rl(f"{CASE}_ms{i}") for i in range(10)]
    ms = [m for m in ms if m]
    R["reliability_det"] = dict(default=base["score"]["flight_score"], n=len(ms),
                                ok=[m["ok"] for m in ms], F=[m.get("score", {}).get("flight_score") for m in ms],
                                its=[m["its"] for m in ms], errors=[m.get("error") for m in ms])
    rdo = [rl(t) for t in ("rdor_best", "rdor_ms0", "rdor_ms1", "rdo_best", "rdo_ms0", "rdo_ms1")]
    R["reliability_rdo"] = [dict(tag=r["tag"], start=r["start"], ok=r["ok"], its=r["its"], time=r["time"],
                                 robust=(r.get("summary") or {}).get("flight_score", {}).get("robust"),
                                 error=r.get("error")) for r in rdo if r]
    enum = [json.loads(f.read_text()) for f in RUNS.glob("mp_*.json")]
    R["study_runs"] = [dict(tag=d["tag"], ok=d["ok"], its=d["its"], time=d["time"]) for d in enum]
    return R


def _f(x, nd=1):
    return "–" if x is None else f"{x:.{nd}f}"


def _ms(s, nd=1):
    if not s:
        return "–"
    if s["n"] == 1:
        return _f(s["median"], nd)
    return f"{_f(s['median'], nd)} [{_f(s['min'], nd)}–{_f(s['max'], nd)}] (n={s['n']})"


def render(R):
    m, S = R["machine"], R["stats"]
    L = []
    w = L.append
    w("# Computational performance: AirCrafter 1.0 vs AirCrafter 2.0 (2024-25 problem)\n")
    w('> **Uncalibrated (2026-10-26).** The results runs this page times were computed on the uncalibrated (G&G) mass model, and the page has not been rerun on the calibrated one. The 2025-26 pages (`docs/studies/results_2026/`, take-off on, calibrated mass model) supersede it for the thesis.\n\n')
    w(f"Generated by `studies/compute_performance.py report` from `outputs/results/perf/` (git `{m['git']}`, tree "
      f"clean: {m['tree_clean']}"
      + (f"; uncommitted when the report was generated: {', '.join(m['uncommitted'])}. The main-table "
         "and scaling runs used the committed code at this hash; the MATLAB changes are the box option of the local "
         "grid refinement (section 3), the script changes its runner and this report" if m.get("uncommitted") else "") + f"); numbers in `compute_performance.json`. **Machine:** {m['cpu']}, {m['cores']} "
      f"logical cores, {m['ram_GB']} GB RAM, {m['os']}; Python {m['python']}, OpenMDAO {m['openmdao']}, numpy "
      f"{m['numpy']}, scipy {m['scipy']}; MATLAB {m['matlab']}. No other computation during the timed runs (the desktop session, an idle MATLAB "
      "window and an idle VM stayed open); each "
      "measurement its own process. Main table single-threaded (OMP/OPENBLAS/MKL_NUM_THREADS = 1, MATLAB "
      "-singleCompThread); median [range] over 5 repeats (runs < 10 min) or 3. AirCrafter 1.0's per-point progress "
      "print was suppressed (opts.quiet) for every timed run. Problem: the best case, "
      f"{CASE} (6S, 11x8E M2 / 10x7E M3).\n")
    w("## 1. Main table (single-threaded)\n")
    a1, d2 = S.get("ac1", {}), S.get("ac2_det", {})
    rows = []
    if a1:
        n = a1["n_points"]["median"]
        rows.append(["AirCrafter 1.0 grid sweep + selection", _ms(a1["time_sweep_s"]), _ms(a1["time_selection_s"], 2),
                     f"{n:.0f} design points ({a1['n_performance_calls']['median']:.0f} mission evaluations)",
                     f"{1e3 * a1['time_sweep_s']['median'] / a1['n_performance_calls']['median']:.2f} ms / mission "
                     "evaluation", "–"])
    if d2:
        rows.append(["AirCrafter 2.0 multipoint (MDF, SLSQP)", _ms(d2["wall_s"]), _ms(d2["setup_s"], 2),
                     f"{d2['model_runs']['median']:.0f} model runs = {d2['mda_solves']['median']:.0f} MDA solves "
                     f"({d2['newton_iterations']['median']:.0f} Newton its), {d2['compute_totals']['median']:.0f} "
                     f"compute_totals, {d2['linear_solves']['median']:.0f} linear solves, "
                     f"{d2['factorisations']['median']:.0f} factorisations",
                     f"{d2['wall_s']['median'] / d2['mda_solves']['median']:.3f} s / MDA solve (incl. derivatives)",
                     f"{d2['iterations']['median']:.0f} SLSQP its, {d2['function_evaluations']['median']:.0f} "
                     f"objective evaluations; {d2['wall_s']['median'] / d2['iterations']['median']:.1f} s / SLSQP "
                     "iteration"])
    w("| method | wall (s) | setup / selection (s) | evaluations | cost per evaluation | optimiser |\n"
      "|---|---:|---:|---|---|---|\n" + "".join("| " + " | ".join(r) + " |\n" for r in rows))
    w("\nMATLAB vs Python: the two codes evaluate different things (AirCrafter 1.0: a closed-form mission estimate "
      "per grid point; AirCrafter 2.0: a coupled aero-structure-propulsion MDA per evaluation plus its analytic "
      "derivatives), so both the wall time and the evaluation count are given, never a single speed-up.\n")
    if a1 and d2:
        w(f"\nAirCrafter 1.0 evaluates {a1['n_performance_calls']['median'] / d2['mda_solves']['median']:.0f}x as many "
          f"points as AirCrafter 2.0 solves MDAs; an AirCrafter 2.0 MDA solve (with its derivatives) costs "
          f"{(d2['wall_s']['median'] / d2['mda_solves']['median']) / (a1['time_sweep_s']['median'] / a1['n_performance_calls']['median']):.0f}x "
          f"an AirCrafter 1.0 mission evaluation; in total AirCrafter 1.0 takes "
          f"{a1['time_sweep_s']['median'] / d2['wall_s']['median']:.1f}x the wall time of one AirCrafter 2.0 "
          "optimisation.\n")
    w("\n## 2. Default threading (one run each)\n")
    rows = []
    for n, lab in (("ac1_default_threads", "AirCrafter 1.0"), ("ac2_det_default_threads", "AirCrafter 2.0 multipoint"),
                   ("rdo_d7_default_threads", f"RDO d = 7, {RDO_MAXITER} SLSQP iterations")):
        s = S.get(n)
        k = "time_sweep_s" if n.startswith("ac1") else "wall_s"
        single = S.get(n.replace("_default_threads", ""), {}).get(k)
        if s:
            rows.append([lab, _f(s[k]["median"]), _f(single["median"]) if single else "–",
                         _f(single["median"] / s[k]["median"], 2) if single else "–"])
    w("| method | default threads (s) | single thread (s) | speed-up |\n|---|---:|---:|---:|\n"
      + "".join("| " + " | ".join(r) + " |\n" for r in rows))
    w("\n## 3. Scaling\n")
    w("### AirCrafter 2.0: cost against the number of design variables\n")
    w("The free subsets start from the config's default start; the other design variables are pinned at the full "
      "optimum.\n\n")
    rows = []
    for n in list(DV_SETS) + ["all"]:
        s = S.get(f"ac2_det_{n}dv") if n != "all" else d2
        if s:
            rows.append([str(n if n != "all" else int(s["n_dv"]["median"])), _ms(s["wall_s"]),
                         f"{s['iterations']['median']:.0f} / {s['function_evaluations']['median']:.0f}",
                         f"{s['mda_solves']['median']:.0f}",
                         f"{s['compute_totals']['median']:.0f}",
                         _f(s["wall_s"]["median"] / max(s["compute_totals"]["median"], 1), 2),
                         _f(s["F"]["median"], 4)])
    w("| design variables | wall (s) | SLSQP its / evaluations | MDA solves | compute_totals | s / gradient iteration | F reached |\n"
      "|---:|---:|---:|---:|---:|---:|---:|\n" + "".join("| " + " | ".join(r) + " |\n" for r in rows))
    w(f"\n### RDO: cost against d ({RDO_MAXITER} SLSQP iterations from the deterministic optimum)\n\n")
    rows = []
    for d, name, lab in sorted([(d, f"rdo_d{d}", "full (first d screened)") for d in RDO_D]
                               + [(4, "rdor_d4", "reduced (mass and wind)")]):
        s = S.get(name)
        if s:
            rows.append([str(d), lab, f"{s['N']['median']:.0f}", _ms(s["setup_s"]), _ms(s["wall_s"]),
                         f"{s['model_runs']['median']:.0f}", f"{s['mda_solves']['median']:.0f}",
                         f"{s['compute_totals']['median']:.0f}",
                         _f(s["wall_s"]["median"] / max(s["iterations"]["median"] or 1, 1), 0),
                         _f(s["wall_s"]["median"] / max(s["mda_solves"]["median"], 1), 2)])
    w("| d | set | N nodes | setup (s) | wall (s) | model runs | MDA solves | compute_totals | s / SLSQP iteration | "
      "s / MDA solve |\n|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|\n"
      + "".join("| " + " | ".join(r) + " |\n" for r in rows))
    w("\nThe cost per MDA solve (derivatives included) is about constant across d and sets; the wall time follows the "
      "number of solves, i.e. N nodes x 2 points x the model runs the line search needed (the reduced set's d = 4 "
      "needed fewer than the full set's d = 3, so it is cheaper per run despite more nodes).\n")
    rm = S.get("rdor_main")
    if rm:
        rs_ = (R["raw"]["rdor_main"][0].get("robust") or {}).get("flight_score", {})
        w(f"\n**Main reduced-set RDO, clean** (d = 4, N = 9, from the deterministic optimum to convergence, "
          f"single-threaded, one run): {rm['wall_s']['median']:.0f} s, {rm['iterations']['median']:.0f} SLSQP "
          f"iterations, {rm['function_evaluations']['median']:.0f} objective evaluations, "
          f"{rm['mda_solves']['median']:.0f} MDA solves, {rm['compute_totals']['median']:.0f} compute_totals"
          + (f"; mu - k sigma {rs_['robust']:.4f}" if rs_.get("robust") is not None else "") + ".\n")
    qu, qs = S.get("quad_udr"), S.get("quad_smolyak")
    if qu and qs:
        w(f"\n**UDR vs Smolyak level 2** at the deterministic optimum (d = {qu['d']['median']:.0f}, values only, 1 "
          f"worker): UDR N = {qu['N']['median']:.0f}, {qu['wall_s']['median']:.0f} s, mu {qu['mu']['median']:.4f}, "
          f"sigma {qu['sigma']['median']:.4f}; Smolyak N = {qs['N']['median']:.0f}, {qs['wall_s']['median']:.0f} s, "
          f"mu {qs['mu']['median']:.4f}, sigma {qs['sigma']['median']:.4f}. RDO cost scales with N (every node is a "
          "full M2 + M3 MDA with its derivatives), so Smolyak would cost "
          f"{qs['N']['median'] / qu['N']['median']:.1f}x per iteration.\n")
    w("\n### AirCrafter 1.0: grid refinement\n\n")
    boxes = refine_boxes()
    R["refine"] = {}
    for tag, (box, levels) in boxes.items():
        R["refine"][tag] = dict(box=box, runs={r: (_load(f"ac1_box_{tag}_r{r}") or [None])[0] for r in levels})
    if a1:
        w(f"The full grid at 2x the points per dimension (four gridded dimensions: mass, span, AR, speed) is 16x the "
          f"points, about {16 * a1['time_sweep_s']['median'] / 3600:.1f} h single-threaded; at 4x it is 256x, about "
          f"{256 * a1['time_sweep_s']['median'] / 86400:.1f} days. A full 2x run was started and stopped after 1 h (too "
          "slow for what it answers). Instead the grid is refined locally, on the same grid points as the full grid "
          "(the box keeps the full grid's points inside it): **top20** = the extents of AirCrafter 1.0's 20 best design "
          "points plus one default step, every speed; **dp** = the selected point +/- 2 default steps in mass, span and "
          "AR, speeds covering every top-20 point's M2 and M3 speed. A local refinement cannot find a better optimum "
          f"outside its box. The box option (`ac1_run.m` opts.box) was added after the main timing commit and is committed with "
          "this page; it only selects grid points, the per-point model is unchanged. The cost per point inside the "
          "boxes is higher than the whole grid's average: the per-point cost varies across the grid (not "
          f"investigated). Default grid (whole): best score {a1['score']['median']:.4f} (M2 + M3).\n\n")
    rows = []
    for tag, b in R["refine"].items():
        box = b["box"]
        base = b["runs"].get(1)
        for r, x in b["runs"].items():
            if not x:
                rows.append([tag, str(r), "not run", "", "", "", "", "", "", ""])
                continue
            dp = x["DP"]
            edge = [n for n, val, lo, hi, glo, ghi in (("mass", dp["M2_mass"], box[0], box[1], 5, 24.5),
                                                       ("span", dp["b"], box[2], box[3], 0.5, 1.8),
                                                       ("AR", dp["AR"], box[4], box[5], 3, 12),
                                                       ("M2 speed", dp["M2_V"], box[6], box[7], 10, 50),
                                                       ("M3 speed", dp["M3_V"], box[6], box[7], 10, 50))
                    if (abs(val - lo) < 1e-9 and lo > glo + 1e-9) or (abs(val - hi) < 1e-9 and hi < ghi - 1e-9)]
            rows.append([tag, str(r), f"{x['n_points']:.0f}", _f(x["time_sweep_s"], 0), _f(x["score"], 4),
                         f"{100 * (x['score'] / base['score'] - 1):+.2f} %" if base else "–",
                         f"m {dp['M2_mass']:g} kg, b {dp['b']:g} m, AR {dp['AR']:g}, V {dp['M2_V']:g} / {dp['M3_V']:g} m/s",
                         f"{dp['M2_LapCount']:g} / {dp['M3_LapCount']:g}", ", ".join(edge) or "no",
                         _f(1e3 * x["time_sweep_s"] / x["n_points"], 2)])
    w("| box | refine | points | sweep (s) | best score | change vs refine 1 | selected point | M2 / M3 laps | "
      "on a box edge (not a grid bound) | ms / point |\n|---|---:|---:|---:|---:|---:|---|---:|---|---:|\n"
      + "".join("| " + " | ".join(r) + " |\n" for r in rows))
    for tag, b in R["refine"].items():
        runs = [x for x in b["runs"].values() if x]
        if len(runs) > 1:
            ch = 100 * (runs[-1]["score"] / runs[0]["score"] - 1)
            w(f"\n{tag}: refine {list(b['runs'])[-1]} changes the best score by {ch:+.2f} % against the default "
              f"spacing ({'within' if abs(ch) < 0.5 else 'outside'} the 0.5 % criterion).")
    w("\n")
    w("\n## 4. Reliability of the search\n")
    rd = R["reliability_det"]
    good = [f for f, o in zip(rd["F"], rd["ok"]) if o]
    w(f"Multipoint, {rd['n']} scrambled Sobol' starts (seed 0) in the design-variable box: {len(good)}/{rd['n']} "
      f"converge; final F {min(good):.4f}–{max(good):.4f} (spread {max(good) - min(good):.1e}); default start "
      f"{rd['default']:.4f}. Failures: " + "; ".join(f"start {i}: {e[:90]}" for i, (o, e) in
                                                   enumerate(zip(rd["ok"], rd["errors"])) if not o) + ".\n")
    if R["reliability_rdo"]:
        w("\nRDO, 3 starts per uncertain set (the deterministic optimum and Sobol' starts 0 and 1), run in parallel "
          "with 3 threads each (wall times not clean). `rdor_*` = the reduced mass-and-wind set (d = 4, the main "
          "results); `rdo_*` = the published 15-parameter set screened to d = 7:\n\n"
          "| run | start | ok | its | wall (h) | mu - k sigma | error |\n"
          "|---|---|---|---:|---:|---:|---|\n" + "".join(
              f"| {r['tag']} | {r['start']} | {r['ok']} | {r['its']} | {r['time'] / 3600:.2f} | {_f(r['robust'], 4)} "
              f"| {(r['error'] or '')[:60]} |\n" for r in R["reliability_rdo"]))
        red = [r for r in R["reliability_rdo"] if r["tag"].startswith("rdor_") and r["ok"]]
        if len(red) > 1:
            rb = [r["robust"] for r in red]
            w(f"\nReduced set: {len(red)}/3 converge, spread of mu - k sigma {max(rb) - min(rb):.1e}; the designs "
              "differ along Kv x M2 throttle (a flat direction; results_2025.md, Findings).\n")
    w("\n## 5. Cost of a complete design study\n")
    if a1 and d2:
        sr = R["study_runs"]
        its = sum(r["its"] for r in sr)
        per_it = d2["wall_s"]["median"] / d2["function_evaluations"]["median"]
        w(f"AirCrafter 1.0: one sweep covers every motor/prop/mass/span/AR/speed combination of its grid: "
          f"{a1['time_sweep_s']['median'] / 60:.0f} min single-threaded.\n\n"
          f"AirCrafter 2.0 multipoint: {len(sr)} runs in outputs/results/runs (the 9 prop pairs at 6S, 4S/5S on the "
          f"best pair, 10 multistarts, bookends, the normalisation grid, the bottle sensitivity), {its} driver "
          f"iterations (objective evaluations) in total; at the clean single-thread {per_it:.1f} s per evaluation "
          f"(gradients included, averaged) that is {its * per_it / 3600:.1f} h (estimate: clean cost per evaluation x "
          "recorded evaluations). The recorded wall "
          f"times of those runs (3 in parallel, 3 threads each) sum to {sum(r['time'] for r in sr) / 3600:.1f} h. "
          "The minimum a team needs is the enumeration plus multistart on the best case: "
          f"{sum(r['its'] for r in sr if '_norm' not in r['tag'] and 'only' not in r['tag'] and 'bottle' not in r['tag']) * per_it / 3600:.1f} h.\n")
        rs = S.get("rdo_d7")
        if rs and R["reliability_rdo"]:
            pi = rs["wall_s"]["median"] / max(rs["function_evaluations"]["median"], 1)
            ri = sum(r["its"] for r in R["reliability_rdo"])
            ri = sum(r["its"] for r in R["reliability_rdo"] if r["tag"].startswith("rdo_"))
            w(f"\nRDO, full set (d = 7): {pi:.0f} s per driver iteration clean; the 3 starts took {ri} driver iterations, "
              f"about {ri * pi / 3600:.1f} h serial single-threaded (estimate), on one propulsion case.\n")
        rm = S.get("rdor_main")
        red = [r for r in R["reliability_rdo"] if r["tag"].startswith("rdor_")]
        if rm and red:
            pi4 = rm["wall_s"]["median"] / max(rm["function_evaluations"]["median"], 1)
            ri4 = sum(r["its"] for r in red)
            w(f"\nRDO, reduced set (d = 4): {pi4:.0f} s per driver iteration clean (the clean main run: "
              f"{rm['wall_s']['median'] / 60:.0f} min to convergence); the 3 starts took {ri4} driver iterations, about "
              f"{ri4 * pi4 / 3600:.1f} h serial single-threaded (estimate).\n")
    return "\n".join(L)


def report():
    R = collect()
    DOCS.mkdir(parents=True, exist_ok=True)
    md = render(R)  # render() adds the local grid refinement to R
    (DOCS / "compute_performance.json").write_text(json.dumps(R, indent=1, default=float))
    (DOCS / "compute_performance.md").write_text(md)
    print("wrote docs/studies/results/compute_performance.{md,json}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("part", choices=["run", "run_reduced", "refine", "report", "child"])
    ap.add_argument("what", nargs="?")
    ap.add_argument("--dvs", type=int)
    ap.add_argument("--d", type=int)
    ap.add_argument("--rule")
    ap.add_argument("--maxiter", type=int)
    ap.add_argument("--set", default="full", help="child rdo: the uncertain set (results_2025.py --set)")
    a = ap.parse_args()
    if a.part == "child":
        import warnings

        warnings.filterwarnings("ignore")
        res = {"det": lambda: child_det(a.dvs, a.maxiter),
               "rdo": lambda: child_rdo(a.d, RDO_MAXITER if a.maxiter is None else a.maxiter, a.set),
               "quad": lambda: child_quad(a.rule)}[a.what]()
        print("RESULT " + json.dumps(res, default=float), flush=True)
    elif a.part == "run":
        run_all()
    elif a.part == "run_reduced":
        run_reduced()
    elif a.part == "refine":
        run_refine()
    else:
        report()
