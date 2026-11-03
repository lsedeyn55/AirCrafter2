"""docs/studies/results_2026/compute_performance_2026.{md,json}: the timings of compute_performance_2026.py and the campaign's
run JSONs (already measured). Every number comes from those files; extrapolations are labelled.

    .venv/bin/python studies/compute_performance_2026_report.py
"""

import json
import platform
import subprocess
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "studies"))
import results_2026 as S  # noqa: E402

PERF = S.OUT / "perf"
DOCS = ROOT / "docs" / "studies" / "results_2026"
LABELS_MC = ["humpback", "humpback_loo", "ac1_direct", "ac1_completed", "mdo", "mdo_wind", "rdo"]


def sh(cmd):
    return subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=60).stdout.strip()


def load(name):
    return [json.loads(f.read_text()) for f in sorted(PERF.glob(f"{name}_*.json")) if f.stem[len(name) + 1:].isdigit()]


def stat(rows, key):
    x = np.array([r[key] for r in rows if r.get(key) is not None], float)
    if not len(x):
        return None
    return dict(mean=float(x.mean()), min=float(x.min()), max=float(x.max()), n=len(x), spread=float(x.max() - x.min()))


def machine():
    import numpy
    import openmdao
    import scipy

    ac1 = load("ac1")
    return dict(cpu=sh("lscpu | grep 'Model name' | sed 's/.*: *//'"), cores=sh("nproc --all"), ram_GB=round(float(sh("free -g | awk 'NR==2{print $2}'") or 0)),
                os=f"{platform.system()} {platform.release()}", python=platform.python_version(), openmdao=openmdao.__version__, numpy=numpy.__version__,
                scipy=scipy.__version__, matlab=ac1[0].get("matlab") if ac1 else None, power_profile=sh("powerprofilesctl get"))


def smolyak_nodes(d):
    from aircrafter2.robustness.quadrature import make_rule
    from aircrafter2.robustness.uncertainty import Dist

    rob = S.config(cfg=S.RDO_CFG).base.robust
    dists = [Dist(f"p{i}", "uniform", 1.0, 0.1) for i in range(d)]
    return make_rule(rob.model_copy(update={"quadrature": "smolyak"}), dists).n


def collect():
    R = dict(page_git=S.git_hash(), machine=machine(), raw={})
    for n in ("ac1", "det", "eval", "robust_d1", "robust_d3", "robust_d5", "rdo", "det_4dv", "det_8dv", "det_13dv"):
        R["raw"][n] = load(n)
    R["git_runs"] = sorted({r.get("git") for rows in R["raw"].values() for r in rows})
    raw = R["raw"]
    ac1, det, ev, r5, rdo = raw["ac1"], raw["det"], raw["eval"], raw["robust_d5"], raw["rdo"]
    main = []
    if ac1:
        s = stat(ac1, "time_sweep_s")
        main.append(dict(name="AirCrafter 1.0 sweep, shipped steps", mean=s["mean"], stat=s, text=f"{ac1[0]['n_points']:.0f} points, {ac1[0]['n_performance_calls']:.0f} calls"))
    if det:
        s = stat(det, "wall_s")
        main.append(dict(name="AirCrafter 2.0 multipoint, cold", mean=s["mean"], stat=s, text=f"{det[0]['nit']} SLSQP iterations, {det[0]['its']} evaluations"))
    if ev:
        for key, name in (("analysis_s", "one analysis, warm from Humpback (M2 + M3)"), ("analysis_converged_s", "one analysis at the converged state"), ("totals_s", "one total-derivative evaluation")):
            s = stat(ev, key)
            main.append(dict(name=name, mean=s["mean"], stat=s, text=""))
    if r5:
        for key, name in (("robust_eval_s", "one robust evaluation, 11 nodes x 2 points"), ("robust_totals_s", "its total derivatives")):
            s = stat(r5, key)
            main.append(dict(name=name, mean=s["mean"], stat=s, text=""))
    if rdo:
        s = stat(rdo, "wall_s")
        main.append(dict(name="RDO to convergence, one run", mean=s["mean"], stat=s, text=f"{rdo[0].get('nit')} SLSQP iterations, mean - k sigma {rdo[0].get('robust'):.5f}" if rdo[0].get("ok") else f"not converged: {rdo[0].get('error')}"))
    R["main_rows"] = main
    # scaling
    sc = {}
    rows = []
    for d, name in ((1, "robust_d1"), (3, "robust_d3"), (5, "robust_d5")):
        m = raw[name]
        if m:
            t = stat(m, "robust_eval_s")["mean"]
            N = m[0]["N"]
            per = t / N
            rows.append(dict(d=d, active=m[0]["active"], udr_nodes=2 * d + 1, N=N, smolyak_nodes=smolyak_nodes(d), mc_nodes=1024, udr_s=t, per_node_s=per, smolyak_s=per * smolyak_nodes(d), mc_s=per * 1024,
                             totals_s=stat(m, "robust_totals_s")["mean"]))
    sc["rdo_vs_d"] = rows
    if ac1 and det:
        calls = ac1[0]["n_performance_calls"]
        per_call = stat(ac1, "time_sweep_s")["mean"] / calls
        k = calls ** (1.0 / 6.0)  # the sweep's six axes: mass, span, AR, speed, battery fraction, banner
        grad = []
        for n, name in ((4, "det_4dv"), (8, "det_8dv"), (13, "det_13dv")):
            if raw[name]:
                grad.append(dict(n=n, s=raw[name][0]["wall_s"], nit=raw[name][0].get("nit"), its=raw[name][0].get("its"), F=raw[name][0].get("F"), ok=raw[name][0].get("ok")))
        grad.append(dict(n=det[0]["n_dv"], s=stat(det, "wall_s")["mean"], nit=det[0]["nit"], its=det[0]["its"], F=det[0]["F"], ok=det[0]["ok"]))
        ns = [2, 4, 6, 8, 10, 13, det[0]["n_dv"]]
        sc["grid_vs_gradient"] = dict(per_call_s=per_call, k=k, gradient=grad, grid=[dict(n=n, points=k ** n, s=per_call * k ** n, extrapolated=n != 6) for n in ns])
    R["scaling"] = sc
    # the campaign's runs
    camp = []
    for f in sorted(S.RUNS.glob("fr_*.json"), key=lambda p: p.stat().st_mtime):
        d = json.loads(f.read_text())
        if "counters" not in d:
            continue
        c = d["counters"]
        fb = d.get("mda_fallbacks") or {}
        cont = sum(x["continuation"] for x in fb.values()) if fb else sum(x["continuation"] for x in d.get("fallbacks", []))
        camp.append(dict(tag=d["tag"], wall=d.get("time"), nit=d.get("nit"), its=d.get("its"), ok=d.get("ok"), grads=c.get("compute_totals"), model_runs=c.get("model_runs"), mda=c.get("mda_solves"), newton=c.get("newton_iterations"),
                         continuation=cont, threads=d.get("threads"), rss=d.get("peak_rss_gb"), git=d.get("git"), error=(d.get("error") or "")[:50]))
    R["campaign"] = camp
    mc = {}
    conv = fail = wind = n = 0
    for w in ("in", "out"):
        for lab in LABELS_MC:
            f = S.RUNS / f"fr_mc_{w}_{lab}.json"
            if f.exists():
                d = json.loads(f.read_text())
                mc[f"{w}_{lab}"] = dict(wall=d["wall"], converged=d["converged"], N=d["N"], mda_failed=d["mda_failed"], wind_limit=d["wind_limit"], retried=d["retried"], workers=d["workers"], rss=d["peak_rss_gb"])
                conv += d["converged"]
                fail += d["mda_failed"]
                wind += d["wind_limit"]
                n += d["N"]
    R["mc"] = mc
    R["reliability"] = reliability(R, n, conv, fail, wind)
    return R


def _group(tags):
    rows = [json.loads((S.RUNS / f"{t}.json").read_text()) for t in tags if (S.RUNS / f"{t}.json").exists()]
    return dict(n=len(rows), converged=sum(1 for r in rows if r.get("ok")), analysis_failed=sum(1 for r in rows if r.get("error")), tags_failed=[r["tag"] for r in rows if not r.get("ok")],
                errors={r["tag"]: r.get("error") for r in rows if r.get("error")})


def reliability(R, n, conv, fail, wind):
    enum = [S.enumeration_tag(8, a, b) for a in S.M2_PROPS for b in S.M3_PROPS] + [f.stem for f in S.RUNS.glob("fr_6S_*.json")]
    other = ["fr_check", "fr_wind", "fr_M2only", "fr_M3only", "fr_norm_NI0.5", "fr_norm_NI2", "fr_norm_B0.5", "fr_norm_B2", "fr_bound_i", "fr_bound_ii", "fr_bound_iii", "fr_bound_banner", "fr_old_constraints",
             "fr_takeoff_off", "fr_ac1_completed"] + [f.stem for f in S.RUNS.glob("fr_*_int_*p*c.json") if not f.stem.startswith("fr_rdo")]
    rdo = ["fr_rdo_mdo", "fr_rdo_second", "fr_rdo_humpback"]
    rnd = [f.stem for f in S.RUNS.glob("fr_rdo_*_int_*p*c*.json")]
    return dict(enumeration=_group(enum), other_deterministic=_group(other), multistart=_group([f"fr_ms{i}" for i in range(10)]), rdo_starts=_group(rdo), rdo_rounding=_group(rnd),
                mc=dict(samples=n, converged=conv, mda_failed=fail, wind_limit=wind))


def num(x):
    """Seconds in a table: whole numbers from 100 up, never exponents."""
    return f"{x:.0f}" if x >= 100 else f"{x:.3g}"


def f(x, nd=1):
    return "–" if x is None else f"{x:.{nd}f}"


def tbl(h, rows):
    return "| " + " | ".join(h) + " |\n|" + "|".join("---" if i == 0 else "---:" for i in range(len(h))) + "|\n" + "".join("| " + " | ".join(str(c) for c in r) + " |\n" for r in rows)


def render(R):
    m = R["machine"]
    L = []
    w = L.append
    w("# Computational performance: AirCrafter 1.0 and AirCrafter 2.0 (2025-26 problem)\n")
    w(f"Generated by `studies/compute_performance_2026_report.py` (page commit `{R['page_git']}`); timings at {', '.join(R['git_runs']) or '–'}, on a clean committed tree, from `outputs/results_2026/perf/`; every number is read from `compute_performance_2026.json`. "
      f"**Machine:** {m['cpu']}, {m['cores']} logical cores, {m['ram_GB']} GB RAM, {m['os']}, power profile {m['power_profile']}; Python {m['python']}, OpenMDAO {m['openmdao']}, numpy {m['numpy']}, scipy {m['scipy']}; MATLAB {m['matlab']}. "
      "The main table is single-threaded (OMP, OPENBLAS and MKL threads = 1; MATLAB -singleCompThread), each measurement its own process, nothing else computing (the desktop session stayed open). Mean [min-max] over the repeats. The session was paused between measurements once, at the user's request, and the interrupted repeat was discarded (RESULTS_LOG.md). "
      "**The two codes evaluate different things:** AirCrafter 1.0 a closed-form mission estimate per grid point, AirCrafter 2.0 a coupled analysis with analytic derivatives. So wall time and evaluation count are given side by side, never one speed-up.\n")
    w("## 1. Main table (single-threaded)\n")
    w(tbl(["measurement", "wall (s)", "repeats", "notes"], [[r["name"], f"{num(r['stat']['mean'])} [{num(r['stat']['min'])}-{num(r['stat']['max'])}]", r["stat"]["n"], r["text"]] for r in R["main_rows"]]))
    raw = R["raw"]
    if raw["ac1"] and raw["det"]:
        a, d = raw["ac1"], raw["det"]
        pc = R["scaling"]["grid_vs_gradient"]["per_call_s"]
        tot = sum(x.get("mda_solves", 0) for x in d) / len(d)
        w(f"\nAirCrafter 1.0 costs {1e3 * pc:.2f} ms per mission evaluation (the plan's 0.43 ms at checkpoint 2, re-measured). One AirCrafter 2.0 optimisation takes {np.mean([x['wall_s'] for x in d]) / 60:.1f} min and {tot:.0f} MDA solves. "
          f"The sweep takes {np.mean([x['time_sweep_s'] for x in a]) / 60:.1f} min. Peak memory {max(x['peak_rss_gb'] for x in d):.2f} GB.\n")
    if raw["rdo"]:
        w(f"\nRDO, one clean run: {f(raw['rdo'][0].get('wall_s') / 3600, 2)} h, peak {f(raw['rdo'][0].get('peak_rss_gb'), 2)} GB (same start as `fr_rdo_mdo`).\n")
    else:
        w("\nRDO to convergence single-threaded was not measured (see RESULTS_LOG.md, Cuts).\n")
    w("\n## 2. Every campaign run\n\nFrom the run JSONs, measured during the campaign: 3 threads, up to 3 jobs at once, so the wall times are not clean. nit = SLSQP iterations, its = objective evaluations, grads = total-derivative evaluations.\n\n")
    w(tbl(["run", "converged", "wall (s)", "nit", "its", "grads", "model runs", "MDA solves", "Newton its", "continuations", "threads", "peak (GB)", "error"],
          [[c["tag"].replace("fr_", ""), c["ok"], f(c["wall"], 0), c["nit"], c["its"], c["grads"], c["model_runs"], c["mda"], c["newton"], c["continuation"], c["threads"], f(c["rss"], 2), c["error"]] for c in R["campaign"]]))
    w("\n**Monte Carlo runs** (in batches of 384 samples on fresh workers; a first run in one pool lost 17-30 % of its samples to a worker failure, not a response to the sample: RESULTS_LOG.md, Decision 13):\n\n" + tbl(["run", "wall (min)", "workers", "converged", "MDA failures", "wind limits", "recovered", "peak (GB, parent)"],
                                         [[k, f(x["wall"] / 60, 1), x["workers"], f"{x['converged']}/{x['N']}", x["mda_failed"], x["wind_limit"], x["retried"], f(x["rss"], 2)] for k, x in R["mc"].items()]))
    w("\n## 3. Scaling\n\n### RDO cost against d\n\nOne robust evaluation at the MDO design (a first pass moves every node from Humpback's converged state). UDR is measured; Smolyak and Monte Carlo are the measured cost per node times their node counts (scaled, not measured). A node is an M2 + M3 pair.\n\n")
    rows = R["scaling"]["rdo_vs_d"]
    w(tbl(["d", "active", "UDR nodes", "Smolyak level 2 nodes", "MC nodes", "UDR wall (s, measured)", "s per node", "Smolyak (s, scaled)", "1024-point MC (s, scaled)", "UDR derivatives (s, measured)"],
          [[r["d"], ", ".join(r["active"]), r["udr_nodes"], r["smolyak_nodes"], r["mc_nodes"], f(r["udr_s"]), f(r["per_node_s"], 1), f(r["smolyak_s"], 0), f(r["mc_s"], 0), f(r["totals_s"])] for r in rows]))
    g = R["scaling"].get("grid_vs_gradient")
    if g:
        w(f"\n### A grid against the gradient\n\nAirCrafter 1.0's grid: points = the product of the steps. At its measured {1e3 * g['per_call_s']:.2f} ms per call and the shipped grid's geometric mean of {g['k']:.1f} steps per axis, a grid in n variables costs {g['k']:.1f}^n calls. **Only n = 6 is measured; every other row is an extrapolation.**\n\n")
        w(tbl(["n", "calls", "grid wall (s)", "status"], [[r["n"], f"{r['points']:.3g}", f"{r['s']:.3g}", "extrapolated" if r["extrapolated"] else "measured"] for r in g["grid"]]))
        w("\nAirCrafter 2.0, with the other design variables pinned at the MDO design (cold in the free variables), measured:\n\n")
        w(tbl(["free design variables", "wall (s)", "nit", "its", "F reached", "converged"], [[r["n"], f(r["s"]), r["nit"], r["its"], f(r["F"], 4), r["ok"]] for r in g["gradient"]]))
        for r in g["gradient"]:
            if not r["ok"]:
                w(f"The {r['n']}-variable run did not report convergence after {r['nit']} iterations ({r['its']} evaluations, F {r['F']:.4f}), so its wall time isn't a converged cost and this scaling isn't monotonic: the count of free variables isn't the only thing that sets the cost.\n")
    w("\n![Main](figures/compute_main.png)\n\n![Scaling](figures/compute_scaling.png)\n")
    w("\n## 4. Reliability\n\nConverged = SLSQP reports success. An analysis failure is the MDA failing or the lap model stopping, counted apart; it's not infeasibility.\n\n")
    rel = R["reliability"]
    rows = []
    for k, name in (("enumeration", "enumeration"), ("other_deterministic", "other deterministic runs"), ("multistart", "multistart"), ("rdo_starts", "RDO starts"), ("rdo_rounding", "robust rounding (stopped at maxiter 15 counts as not converged)")):
        x = rel[k]
        rows.append([name, f"{x['converged']}/{x['n']}", x["analysis_failed"], "; ".join(f"{a.replace('fr_', '')}: {b[:60]}" for a, b in x["errors"].items()) or "–"])
    c = rel["mc"]
    rows.append(["Monte Carlo samples", f"{c['converged']}/{c['samples']}", f"{c['mda_failed']} MDA, {c['wind_limit']} lap-model limit", "counted apart from constraint violations"])
    w(tbl(["kind of run", "converged", "analysis failures", "which"], rows))
    return "\n".join(L) + "\n"


def report():
    R = collect()
    DOCS.mkdir(parents=True, exist_ok=True)
    (DOCS / "compute_performance_2026.json").write_text(json.dumps(R, indent=1, default=float))
    (DOCS / "compute_performance_2026.md").write_text(render(R))
    print("wrote compute_performance_2026.{md,json}")
    return R


if __name__ == "__main__":
    report()
