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


def validate(n=1024, workers=5):
    """1024-point Sobol' Monte Carlo over all 15 parameters at the Kv-DV robust design (as rdo_study validate)."""
    from aircrafter2.robustness.quadrature import sobol
    from aircrafter2.robustness.rdo_problem import resolved_dists
    from aircrafter2.robustness.sampling import evaluate, violation_probabilities

    rb = load("robust")
    cfg = load_full_config(ROOT / "configs" / "full_mdo_rdo.yaml")
    prob = build_full_problem(cfg, driver=False)
    dists, _ = resolved_dists(cfg, lambda name: v(prob, name))
    del prob
    rule = sobol(dists, n, seed=0)
    rows = [{d.name: float(x) for d, x in zip(dists, node)} for node in rule.nodes[1:]]  # drop the reference node
    outputs = ["ThreeLapTime"] + [nm for nm, _ in full_constraints(cfg)]
    ev = evaluate(cfg, rb["dv"], rows, outputs, workers)
    V = ev["values"]
    ok = ~np.isnan(V).any(axis=1)
    t = V[ok, 0]
    rb["mc"] = dict(N=n, failed=len(ev["failed"]), wall=ev["wall"], mu=float(t.mean()), sigma=float(t.std(ddof=1)),
                    p_violation=violation_probabilities(V, outputs, full_constraints(cfg)))
    dump("robust", rb)
    print(f"MC: failed {len(ev['failed'])}, mu {t.mean():.3f} sigma {t.std(ddof=1):.3f}, "
          f"P(viol) {rb['mc']['p_violation']}", flush=True)


def _f(x, n=4):
    return f"{x:.{n}g}"


def figures():
    from aircrafter2.plotting.style import PALETTE, figure, save

    files = []
    if not (OUT / "sweep.json").exists():
        return files
    rows = [r for r in load("sweep")["rows"]]
    fdir = OUT / "figures"
    fdir.mkdir(parents=True, exist_ok=True)
    fig, (a1, a2) = figure(1.0, 0.4, ncols=2, layout="constrained")
    I = [r["I_max"] for r in rows]
    a1.plot(I, [r["values"]["Kv"] for r in rows], "o-", color=PALETTE[0])
    a1.set_xlabel("current limit I_max (A)")
    a1.set_ylabel("optimal Kv (rpm/V)")
    a2.plot(I, [r["values"]["ThreeLapTime"] for r in rows], "o-", color=PALETTE[1])
    a2.set_xlabel("current limit I_max (A)")
    a2.set_ylabel("3-lap time at the optimum (s)")
    for r in rows:
        a1.annotate("I" if "I_margin" in r["motor"]["set_by"] else "rpm", (r["I_max"], r["values"]["Kv"]),
                    textcoords="offset points", xytext=(4, 4), fontsize=7)
    files.append(save(fig, fdir / "kv_vs_imax.pdf"))
    return files


def report():
    L = []
    w = L.append
    keep = {}
    w("# Motor Kv as a design variable: study\n")
    w("Generated by `studies/kv_study.py report` from `outputs/kv/*.json` (not in git; a compact copy is in "
      "`docs/studies/kv_study_results.json`). Plan: `docs/KV_PLAN.md`. Model: `docs/modules/propulsion.md` (motor "
      "rewind). Single 10x7E 4S, the config's placeholder motor as the reference motor (Kv_ref 900 rpm/V, Rm_ref "
      "0.05 ohm, I0_ref 1 A, 0.15 kg), Kv bounds 500-1500 rpm/V: **all PLACEHOLDERS** (OPEN_QUESTIONS Q7, Q34). "
      "Objective: the placeholder 3-lap time.\n")
    cmp_ = load("compare") if (OUT / "compare.json").exists() else None
    sw = load("sweep") if (OUT / "sweep.json").exists() else None
    rb = load("robust") if (OUT / "robust.json").exists() else None
    co = load("corners") if (OUT / "corners.json").exists() else None
    if cmp_:
        fx, kv = cmp_["fixed"], cmp_["kv"]
        w("## Summary\n")
        w(f"- With Kv free the optimiser picks **Kv = {kv['values']['Kv']:.0f} rpm/V**, set by "
          f"**{kv['motor']['set_by']}**, and the 3-lap time drops from {fx['values']['ThreeLapTime']:.3f} s (Kv fixed "
          f"at 900) to {kv['values']['ThreeLapTime']:.3f} s ({kv['values']['ThreeLapTime'] - fx['values']['ThreeLapTime']:+.3f} s).")
        if sw:
            setby = {r["I_max"]: r["motor"]["set_by"] for r in sw["rows"]}
            w("- Current-limit sweep: " + "; ".join(f"I_max {i:g} A: Kv {r['values']['Kv']:.0f}, set by "
                                                     f"{setby[i]}" for i, r in ((r['I_max'], r) for r in sw['rows'])) + ".")
        if rb:
            w(f"- Robust (μ + σ, Kv a DV, Kv_factor / Rm_ref uncertain): Kv = {rb['dv']['Kv']:.0f} rpm/V, μ = "
              f"{rb['summary']['score']['mu']:.3f} s, σ = {rb['summary']['score']['sigma']:.3f} s (UDR; fixed-Kv "
              "robust: 80.072 / 0.692 s)"
              + (f". Kv rides the nominal rpm limit, which is not a robust constraint, so it is exceeded in "
                 f"{100 * rb['mc']['p_violation']['rpm_margin']:.0f} % of the Monte Carlo samples (§4, Q33)"
                 if "mc" in rb else "") + ".")
        w("")
    if co:
        w("## 1. Model checks\n")
        w("Hand check: static current, static thrust and rpm with the rewound Rm and I0 match an independent root "
          "solve at Kv = 700 and 1200 to 1e-9 (`tests/test_propulsion.py`). Kv = Kv_ref reproduces the fixed motor "
          "exactly, and the fixed-Kv configs reproduce the pre-Kv reference bit for bit (`tests/test_kv_dv.py`, "
          "`tests/test_rdo.py`).\n")
        w("Coupled analysis at the Kv bounds (other DVs fixed):\n")
        w("| Point | Kv (rpm/V) | MDA | 3-lap time (s) | level speed v (m/s) | I_static (A) | rpm_margin (rpm) |")
        w("|---|---:|---|---:|---:|---:|---:|")
        for r in co["rows"]:
            if r["converged"]:
                w(f"| {r['point']} | {r['Kv']:g} | converged | {r['ThreeLapTime']:.2f} | {r['v']:.2f} | "
                  f"{r['I_static']:.1f} | {r['rpm_margin']:.0f} |")
            else:
                w(f"| {r['point']} | {r['Kv']:g} | **not converged** | - | - | - | - |")
        w("\nAt Kv = 1500 the MDA does not converge: the full-throttle level speed passes `v_top` = 50 m/s, the top "
          "of the T(v) fit range and the upper bound of the speed balance (at the fixed-Kv optimum the limit is "
          "near Kv ≈ 1205, from a bisection). The optimum (§2) is far below it, but an SLSQP line search that "
          "steps there stops the run as MDA non-convergence (OpenMDAO's SLSQP driver re-raises the analysis "
          "error). The bound is a placeholder (Q34); raising `v_top` or lowering the upper Kv bound would remove "
          "the failing region. Neither is done here.\n")
    if cmp_:
        w("## 2. Fixed Kv = 900 vs Kv a design variable (single 10x7E 4S)\n")
        w(f"Both from the config start point, SLSQP, {fx['threads']} BLAS threads, energy reserve 20 %.\n")
        w("| Quantity | fixed Kv | Kv a DV |")
        w("|---|---:|---:|")
        for k in [k for k in kv["dv"] if k != "Kv"]:  # Kv: in the motor rows below
            w(f"| {k} | {_f(fx['dv'][k])} | {_f(kv['dv'][k])} |")
        units = dict(ThreeLapTime="s", score="s", m_total="kg", v="m/s", I_static="A", I_margin="A", rpm_margin="rpm",
                     energy_margin="J", thrust_margin_static="N", T_static="N", capacity="Ah", Kv="rpm/V", Rm="ohm",
                     I0="A", laps_frac="-")
        for k in ["Kv", "Rm", "I0", "ThreeLapTime", "m_total", "v", "I_static", "I_margin", "rpm_margin",
                  "energy_margin", "T_static", "thrust_margin_static"]:
            w(f"| {k} ({units[k]}) | {_f(fx['values'][k], 5)} | {_f(kv['values'][k], 5)} |")
        w(f"| SLSQP iterations | {fx['its']} | {kv['its']} |")
        w(f"| wall time (s) | {fx['time']:.0f} | {kv['time']:.0f} |")
        w(f"| active constraints | {', '.join(fx['active_constraints'])} | {', '.join(kv['active_constraints'])} |")
        w(f"| DVs at a bound | {', '.join(fx['active_bounds'])} | {', '.join(kv['active_bounds'])} |")
        w(f"| Kv set by | - | {kv['motor']['set_by']} |")
        w("")
        w(f"**Reading.** Kv rides the rpm limit: `rpm_margin` = 0 at Kv = {kv['values']['Kv']:.0f}, while the "
          f"static current ({kv['values']['I_static']:.1f} A) is well under I_max = {kv['I_max']:g} A. The rpm "
          "limit is 0.85 × APC's 150000/D[in] at the top (full-throttle level) speed: a higher Kv spins the prop "
          "faster at the top speed than that limit allows. The faster motor raises the level speed "
          f"({fx['values']['v']:.2f} → {kv['values']['v']:.2f} m/s) and the battery grows to keep the 3-lap energy "
          f"({fx['values']['capacity']:.3f} → {kv['values']['capacity']:.3f} Ah). The static-margin upper bound is "
          "no longer active.\n")
        keep["compare"] = cmp_
    if sw:
        w("## 3. Dependence on the current limit\n")
        w("Kv a DV, single 10x7E 4S, from the config start point; only I_max changes.\n")
        w("| I_max (A) | optimal Kv (rpm/V) | 3-lap time (s) | I_static (A) | I_margin (A) | rpm_margin (rpm) | "
          "Kv set by | converged / its |")
        w("|---:|---:|---:|---:|---:|---:|---|---|")
        for r in sw["rows"]:
            x = r["values"]
            w(f"| {r['I_max']:g} | {x['Kv']:.1f} | {x['ThreeLapTime']:.3f} | {x['I_static']:.1f} | {x['I_margin']:.2f} "
              f"| {x['rpm_margin']:.1f} | {r['motor']['set_by']} | {r['ok']} / {r['its']} |")
        by_i = [r for r in sw["rows"] if "I_margin" in r["motor"]["set_by"]]
        by_rpm = [r for r in sw["rows"] if "rpm_margin" in r["motor"]["set_by"]]
        w("")
        w("**Reading.** " + (f"The current limit sets Kv only at I_max = {', '.join(f'{r['I_max']:g}' for r in by_i)} A"
                             if by_i else "The current limit never sets Kv in this range")
          + (f"; at I_max = {', '.join(f'{r['I_max']:g}' for r in by_rpm)} A the rpm limit does, and the optimum "
             "does not move with I_max" if by_rpm else "") + ". The expectation in the brief, that the optimum sits "
          "at I_static = I_max, does not hold for this prop and cell count: the rpm limit binds first, at a static "
          f"current of {min(r['values']['I_static'] for r in sw['rows']):.1f} A, so I_max would only set Kv below "
          "that. The optimal Kv is set by the APC rpm limit (and its 0.85 factor, `propulsion.rpm_fraction`), not by "
          "the I_max placeholder; both are placeholders for a real motor/ESC (Q7).\n")
        keep["sweep"] = sw
    if rb:
        old = json.loads((ROOT / "docs" / "studies" / "rdo_study_results.json").read_text()).get("robust_single_10x7E")
        w("## 4. Robust design with Kv a design variable\n")
        w(f"`configs/full_mdo_rdo.yaml`: UDR over {', '.join(rb['active'])} (N = {rb['N']}), minimise μ + "
          f"{rb['k']:g}σ, β = {rb['beta']:g}, as in docs/studies/rdo_study.md but with Kv a DV (Kv_factor and "
          "Rm_ref replace Kv and Rm). Compared with the fixed-Kv robust optimum of that study (UDR estimates for "
          "both).\n")
        w("| Quantity | fixed Kv (rdo_study) | Kv a DV |")
        w("|---|---:|---:|")
        s_new = rb["summary"]["score"]
        if old:
            s_old = old["udr_at_optimum"]["score"]
            w(f"| μ (s) | {s_old['mu']:.3f} | {s_new['mu']:.3f} |")
            w(f"| σ (s) | {s_old['sigma']:.3f} | {s_new['sigma']:.3f} |")
            w(f"| μ + kσ (s) | {s_old['mu'] + rb['k'] * s_old['sigma']:.3f} | {s_new['mu'] + rb['k'] * s_new['sigma']:.3f} |")
            w(f"| Kv (rpm/V) | 900 (fixed) | {rb['dv']['Kv']:.1f} |")
            w(f"| capacity (Ah) | {old['dv']['capacity']:.3f} | {rb['dv']['capacity']:.3f} |")
            w(f"| SLSQP iterations | {old['its']} | {rb['its']} |")
            w(f"| wall time (min) | {old['time'] / 60:.1f} | {rb['time'] / 60:.1f} |")
        w(f"| converged | {old['ok'] if old else '-'} | {rb['ok']}{' (' + rb['error'] + ')' if rb.get('error') else ''} |")
        ref = rb["reference_node"]
        w(f"\nAt the reference node (all parameters at their means): rpm_margin = {ref['rpm_margin']:.1f} rpm, "
          f"I_margin = {ref['I_margin']:.1f} A.\n")
        if "mc" in rb:
            mc = rb["mc"]
            pv = mc["p_violation"]
            old_pv = (old or {}).get("mc", {}).get("all", {}).get("p_violation", {})
            w(f"Monte Carlo over all 15 parameters ({mc['N']} Sobol' points, seed 0; {mc['failed']} MDA failures): "
              f"μ = {mc['mu']:.3f} s, σ = {mc['sigma']:.3f} s.\n")
            w("| P(violated) | fixed Kv (rdo_study) | Kv a DV |")
            w("|---|---:|---:|")
            for k in pv:
                if pv[k] > 0 or old_pv.get(k, 0) > 0:
                    w(f"| {k} | {100 * old_pv.get(k, float('nan')):.1f} % | {100 * pv[k]:.1f} % |")
            w(f"\n**Reading.** Kv again rides `rpm_margin`, which is not a robust constraint (it is enforced at the "
              f"reference node only, OPEN_QUESTIONS Q33). With the winding tolerance (Kv_factor ±5 %) moving the rotor "
              f"speed, the rpm limit is exceeded in {100 * pv.get('rpm_margin', 0):.0f} % of the samples. Adding "
              "`rpm_margin` to `robust.robust_constraints` would bound it; not done here, so the settings stay those "
              "of the RDO study.\n")
        keep["robust"] = {k: rb[k] for k in rb if k != "nodes"}
    files = figures()
    if files:
        w("## Figures\n")
        w("![kv_vs_imax](../../outputs/kv/figures/kv_vs_imax.png)\n")
        w("*Optimal Kv and 3-lap time against the current limit I_max (labels: the constraint that set Kv).*\n")
    w("## Caveats\n")
    w("- The reference motor, the Kv bounds, I_max and the rpm fraction are placeholders (Q7, Q34); with real motor "
      "data the binding limit can change.")
    w("- The rewind scaling assumes the same frame and copper fill (docs/modules/propulsion.md; citation needed).")
    w("- Kv is continuous; `api.reanalyse_with_motor` re-analyses a design with a catalogue motor.")
    if co:
        keep["corners"] = co
    (ROOT / "docs" / "studies" / "kv_study.md").write_text("\n".join(L) + "\n")
    (ROOT / "docs" / "studies" / "kv_study_results.json").write_text(
        json.dumps(keep, indent=1, default=lambda o: o.item() if hasattr(o, "item") else o.tolist()))
    print("\n".join(L))


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("part", choices=["corners", "compare", "sweep", "robust", "validate", "report"])
    globals()[ap.parse_args().part]()
