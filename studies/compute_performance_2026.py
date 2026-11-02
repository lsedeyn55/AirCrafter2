"""Computational performance on the 2025-26 problem: AirCrafter 1.0's sweep, a deterministic multipoint optimisation, an
analysis, a total-derivative evaluation, a robust evaluation and an RDO, all single-threaded on a clean committed tree with
nothing else running (run last). The page: studies/compute_performance_2026_report.py.

    .venv/bin/python studies/compute_performance_2026.py run      # every measurement (resumes), hours
    .venv/bin/python studies/compute_performance_2026.py child det | eval | robust --d D | rdo   # one measurement

Each measurement is its own process (OMP, OPENBLAS and MKL threads = 1; MATLAB -singleCompThread): the counters are on the
solver and driver objects (results_2026.instrument), not estimates. Raw: outputs/results_2026/perf/<name>_<repeat>.json.
"""

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "studies"))
import results_2026 as R  # noqa: E402

PERF = R.OUT / "perf"
THREAD_VARS = ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS")
DV_SETS = {4: ["c", "Kv", "throttle_M2", "capacity_M3"],
           8: ["c", "Kv", "throttle_M2", "capacity_M3", "tc_scale", "S_ht", "throttle_M3", "L_banner"],
           13: ["c", "Kv", "throttle_M2", "capacity_M3", "tc_scale", "S_ht", "throttle_M3", "L_banner", "l_ht", "S_vt",
                "x_cb_frac", "capacity_M2", "x_batt_M2"]}
REPEATS = 3


def best_case():
    return R.ranked_cases()[0]


def child_det(dvs=None):
    """The best case's deterministic multipoint optimisation, cold from Humpback (the config's start). dvs: only that
    many design variables free, the others pinned at the mdo design (the cost-against-DVs study)."""
    import openmdao.api as om
    from openmdao.core.analysis_error import AnalysisError

    from aircrafter2.problems.multipoint import (add_multipoint_driver, apply_multipoint_values, build_multipoint_model,
                                                 run_warm)

    run = best_case()
    mcfg = R.config(run["cells"], run["props"]["M2"], run["props"]["M3"])
    only = DV_SETS[dvs] if dvs else None
    t0 = time.perf_counter()
    p = om.Problem(reports=False)
    build_multipoint_model(p.model, mcfg, mda_retry=True)
    add_multipoint_driver(p, mcfg, only=only)
    p.driver.options["disp"] = False
    p.setup()
    apply_multipoint_values(p, mcfg)
    t_setup = time.perf_counter() - t0
    if only:
        pinned = {k: x for k, x in R.best_integer(run["tag"])["dv"].items() if k not in only}
        run_warm(p, mcfg, pinned, run=False)
    C = R.instrument(p)
    t0 = time.perf_counter()
    err = None
    try:
        res = p.run_driver()
        ok, its = bool(res.success), int(res.iter_count)
    except AnalysisError as exc:
        ok, its, err = False, int(p.driver.iter_count), str(exc)[:200]
    return dict(ok=ok, nit=R.slsqp_nit(p.driver), its=its, error=err, wall_s=time.perf_counter() - t0, setup_s=t_setup,
                n_dv=len(only or mcfg.all_dvs()), F=R.v(p, "flight_score"), peak_rss_gb=R.peak_rss_gb(), **C)


def child_eval():
    """At the mdo design: one analysis (both points, warm from Humpback's converged state, as the warm path moves), one
    analysis at the converged state, and one total-derivative evaluation (the driver's objective and constraints with
    respect to every design variable)."""
    from aircrafter2.problems.multipoint import build_multipoint_problem, run_warm

    run = best_case()
    mdo = R.best_integer(run["tag"])
    mcfg = R.config(run["cells"], run["props"]["M2"], run["props"]["M3"])
    t0 = time.perf_counter()
    p = build_multipoint_problem(mcfg, driver=True, mda_retry=True)
    t_setup = time.perf_counter() - t0
    run_warm(p, mcfg, mdo["dv"], run=False)
    p.final_setup()
    t0 = time.perf_counter()
    p.run_model()
    t_analysis = time.perf_counter() - t0
    t0 = time.perf_counter()
    p.run_model()
    t_converged = time.perf_counter() - t0
    t0 = time.perf_counter()
    p.driver._compute_totals(of=list(p.driver._responses), wrt=list(p.driver._designvars), return_format="array")
    t_totals = time.perf_counter() - t0
    return dict(setup_s=t_setup, analysis_s=t_analysis, analysis_converged_s=t_converged, totals_s=t_totals,
                n_of=len(p.driver._responses), n_wrt=len(p.driver._designvars), F=R.v(p, "flight_score"),
                peak_rss_gb=R.peak_rss_gb())


def child_robust(d):
    """One robust evaluation at the mdo design over the first d parameters of the set (the screening's order at the
    mdo design): model run + total derivatives; N = 2d + 1 nodes x 2 points."""
    from aircrafter2.problems.multipoint import run_warm
    from aircrafter2.robustness.rdo_multipoint import build_robust_multipoint

    run = best_case()
    mdo = R.best_integer(run["tag"])
    mcfg = R.rdo_mcfg(run)
    order = _set_order(mcfg)
    mcfg.base.robust.active = order[:d]
    t0 = time.perf_counter()
    p = build_robust_multipoint(mcfg, driver=True)
    p.driver.options["disp"] = False
    t_setup = time.perf_counter() - t0
    run_warm(p, mcfg, mdo["dv"], run=False)
    C = R.instrument(p)
    t0 = time.perf_counter()
    p.run_model()
    t_eval = time.perf_counter() - t0
    solves = C["mda_solves"]
    t0 = time.perf_counter()
    p.driver._compute_totals(of=list(p.driver._responses), wrt=list(p.driver._designvars), return_format="array")
    t_totals = time.perf_counter() - t0
    return dict(d=d, active=order[:d], N=p.rdo["N"], setup_s=t_setup, robust_eval_s=t_eval, robust_totals_s=t_totals,
                mda_solves=solves, peak_rss_gb=R.peak_rss_gb())


def _set_order(mcfg):
    """The RDO's five parameters by their 1-D variance at the mdo design, largest first (the screening's order)."""
    active = list(mcfg.base.robust.active)
    f = R.RUNS / "fr_screen_mdo.json"
    if f.exists():
        oned = json.loads(f.read_text())["oned"]
        return sorted(active, key=lambda n: -(oned.get(n) or 0.0))
    return active


def child_rdo():
    """RDO to convergence from the mdo design, the same run as fr_rdo_mdo but single-threaded and alone."""
    run = best_case()
    mdo = R.best_integer(run["tag"])
    out = R.rdo("fr_perf_rdo", run["tag"], start=mdo["tag"], free=True)
    return dict(ok=out["ok"], nit=out["nit"], its=out["its"], error=out["error"], wall_s=out["time"],
                robust=(out.get("summary") or {}).get("flight_score", {}).get("robust"), counters=out["counters"],
                peak_rss_gb=out["peak_rss_gb"])


def _env():
    env = dict(os.environ, AIRCRAFTER2_APC_DIR="/home/lucas/University/THESIS/PERFILES2")
    for k in THREAD_VARS:
        env[k] = "1"
    return env


def measure(name, repeat, argv):
    """One measurement in a fresh process; the JSON (skipped if present at this commit) records the commit."""
    PERF.mkdir(parents=True, exist_ok=True)
    f = PERF / f"{name}_{repeat}.json"
    if f.exists():
        return json.loads(f.read_text())
    t0 = time.perf_counter()
    out = subprocess.run([sys.executable, __file__, "child", *argv], cwd=ROOT, env=_env(), capture_output=True, text=True,
                         timeout=24 * 3600)
    line = [x for x in out.stdout.splitlines() if x.startswith("RESULT ")]
    if not line:
        raise RuntimeError(f"{name}: no result\n{out.stderr[-2000:]}")
    res = json.loads(line[-1][7:])
    res.update(process_s=time.perf_counter() - t0, name=name, repeat=repeat, git=R.git_hash(), threads=1)
    f.write_text(json.dumps(res, indent=1))
    print(f"{time.strftime('%F %T')} {name} #{repeat}: {res.get('wall_s') or res.get('analysis_s') or res.get('robust_eval_s'):.1f} s",
          flush=True)
    return res


def measure_ac1(repeat):
    """One timed AirCrafter 1.0 sweep at the shipped steps (MATLAB -singleCompThread, no sweep cache)."""
    PERF.mkdir(parents=True, exist_ok=True)
    f = PERF / f"ac1_{repeat}.json"
    if f.exists():
        return json.loads(f.read_text())
    env = dict(_env(), AC1_TIME_OUT=str(f))
    t0 = time.perf_counter()
    subprocess.run(["matlab", "-singleCompThread", "-batch", "run('reference/aircrafter1/time_2026.m')"], cwd=ROOT, env=env,
                   check=True, capture_output=True, timeout=12 * 3600)
    res = json.loads(f.read_text())
    res.update(process_s=time.perf_counter() - t0, name="ac1", repeat=repeat, git=R.git_hash(), threads=1)
    f.write_text(json.dumps(res, indent=1))
    print(f"{time.strftime('%F %T')} ac1 #{repeat}: sweep {res['time_sweep_s']:.0f} s", flush=True)
    return res


def run_all(with_rdo=True):
    """Cheapest first, so a cut loses the least: the per-evaluation repeats, the optimisations, the sweeps, the scaling,
    and the single-threaded RDO last."""
    for i in range(REPEATS):
        measure("eval", i, ["eval"])
    for i in range(REPEATS):
        measure("det", i, ["det"])
    for i in range(REPEATS):
        measure("robust_d5", i, ["robust", "--d", "5"])
    for d in (1, 3):
        measure(f"robust_d{d}", 0, ["robust", "--d", str(d)])
    for i in range(REPEATS):
        measure_ac1(i)
    for n in DV_SETS:
        measure(f"det_{n}dv", 0, ["det", "--dvs", str(n)])
    if with_rdo:
        measure("rdo", 0, ["rdo"])


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("part", choices=["run", "child"])
    ap.add_argument("what", nargs="?")
    ap.add_argument("--dvs", type=int)
    ap.add_argument("--d", type=int)
    ap.add_argument("--no-rdo", action="store_true")
    a = ap.parse_args()
    if a.part == "child":
        import warnings

        warnings.filterwarnings("ignore")
        res = {"det": lambda: child_det(a.dvs), "eval": child_eval, "robust": lambda: child_robust(a.d), "rdo": child_rdo}[a.what]()
        print("RESULT " + json.dumps(res, default=float), flush=True)
    else:
        run_all(not a.no_rdo)
