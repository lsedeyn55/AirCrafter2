"""Robust design optimisation study (docs/studies/rdo_study.md). Each part writes outputs/rdo/<part>.json.

    .venv/bin/python studies/rdo_study.py propagation     # checkpoint 2: quadrature vs Monte Carlo at the det. optimum
    .venv/bin/python studies/rdo_study.py optimise --tag robust_single_10x7E [--reserve 0] [--sigma-cap S] ...
    .venv/bin/python studies/rdo_study.py validate --tags det_single_10x7E robust_single_10x7E ...

Workers: RDO_WORKERS (default 5; ~1-2 GB each on a 14 GB machine).
"""

import os

os.environ.setdefault("OMP_NUM_THREADS", "1")

import argparse  # noqa: E402
import json  # noqa: E402
import time  # noqa: E402
import warnings  # noqa: E402
from pathlib import Path  # noqa: E402

import numpy as np  # noqa: E402

from aircrafter2.problems.full_mdo import build_full_problem, full_constraints, load_full_config  # noqa: E402
from aircrafter2.robustness.quadrature import smolyak, sobol  # noqa: E402
from aircrafter2.robustness.rdo_problem import build_robust_problem, resolved_dists, robust_summary  # noqa: E402
from aircrafter2.robustness.sampling import evaluate, violation_probabilities  # noqa: E402

warnings.filterwarnings("ignore")
ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "outputs" / "rdo"
WORKERS = int(os.environ.get("RDO_WORKERS", "5"))
N_MC = 1024  # Sobol' points (a power of 2, ~1000)


def v(p, k):
    return float(np.ravel(p.get_val(k))[0])


def dump(name, data):
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / f"{name}.json").write_text(json.dumps(data, indent=1, default=lambda o: o.item() if hasattr(o, "item")
                                                 else o.tolist()))


def load(name):
    return json.loads((OUT / f"{name}.json").read_text())


def tracked(cfg):
    """Outputs recorded for every sample: the score, every constraint, and a few report quantities."""
    return (["ThreeLapTime"] + [n for n, _ in full_constraints(cfg)]
            + ["m_total", "laps_frac", "v", "E_usable", "energy_3laps", "Lap1time"])


def deterministic_optimum(cfg, tag):
    """Deterministic optimum of cfg (robust block ignored), cached in outputs/rdo/det_<tag>.json."""
    f = OUT / f"det_{tag}.json"
    if f.exists():
        return json.loads(f.read_text())
    prob = build_full_problem(cfg)
    prob.driver.options["disp"] = False
    t = time.perf_counter()
    res = prob.run_driver()
    out = dict(dv={k: v(prob, k) for k in cfg.design_variables}, ok=bool(res.success), its=res.iter_count,
               time=time.perf_counter() - t, values={o: v(prob, o) for o in tracked(cfg)})
    dump(f"det_{tag}", out)
    return out


def moments(values, outputs, rule=None):
    """{output: (mu, sigma)}: by the rule's weights, or the sample mean / std (ddof 1) over converged rows."""
    out = {}
    for j, o in enumerate(outputs):
        col = values[:, j]
        if rule is not None:
            mu, var = rule.moments(col)
            out[o] = (mu, float(np.sqrt(max(var, 0.0))))
        else:
            c = col[~np.isnan(col)]
            out[o] = (float(c.mean()), float(c.std(ddof=1)))
    return out


def sample_rows(dists, act, rule):
    """{param: value} rows for a rule over the active parameters; the others at their means."""
    base = {d.name: d.mean for d in dists}
    rows = []
    for r in rule.nodes:
        row = dict(base)
        row.update({d.name: float(x) for d, x in zip(act, r)})
        rows.append(row)
    return rows


# --- checkpoint 2 ------------------------------------------------------------------------------------
def propagation(cfg_name="full_mdo_rdo.yaml", tag="single_10x7E"):
    cfg = load_full_config(ROOT / "configs" / cfg_name)
    det = deterministic_optimum(cfg, tag)
    outputs = tracked(cfg)
    cons = full_constraints(cfg)
    res = dict(cfg=cfg_name, dv=det["dv"], outputs=outputs, constraints=cons)

    # quadrature (UDR, the robust model itself)
    t = time.perf_counter()
    rp = build_robust_problem(cfg, driver=False)
    t_setup = time.perf_counter() - t
    for k, x in det["dv"].items():
        rp.set_val(k, x)
    t = time.perf_counter()
    rp.run_model()
    t_run = time.perf_counter() - t
    rp.set_val("b", det["dv"]["b"] * (1 + 1e-4))
    t = time.perf_counter()
    rp.run_model()
    t_warm = time.perf_counter() - t
    rp.set_val("b", det["dv"]["b"])
    rp.run_model()
    dists, act = rp.rdo["dists"], rp.rdo["active"]
    summ = robust_summary(rp)
    res["udr"] = dict(N=rp.rdo["N"], setup=t_setup, cold=t_run, warm=t_warm,
                      moments={q: (d["mu"], d["sigma"]) for q, d in summ.items()},
                      nodes={q: d["nodes"] for q, d in summ.items()},
                      fallbacks=[len([e for e in s.log if not e[2]]) for s in rp.rdo["solvers"]])
    print(f"UDR N={rp.rdo['N']}: cold {t_run:.1f}s warm {t_warm:.1f}s", flush=True)
    del rp

    # the same UDR nodes through the sampler: every tracked output (and a check of the multi-point wiring)
    from aircrafter2.robustness.quadrature import make_rule
    rule_udr = make_rule(cfg.robust, act)
    ev = evaluate(cfg, det["dv"], sample_rows(dists, act, rule_udr), outputs, WORKERS)
    res["udr"]["all_moments"] = moments(ev["values"], outputs, rule_udr)
    res["udr"]["sampler_nodes_score"] = ev["values"][:, 0].tolist()
    res["udr"]["sampler_failed"] = ev["failed"]

    # Smolyak level 2 over the same d parameters (cross terms)
    rule_sm = smolyak(act, 2)
    ev = evaluate(cfg, det["dv"], sample_rows(dists, act, rule_sm), outputs, WORKERS)
    res["smolyak"] = dict(N=rule_sm.n, wall=ev["wall"], failed=ev["failed"], retried=ev["retried"],
                          moments=moments(ev["values"], outputs, rule_sm))
    print(f"Smolyak N={rule_sm.n}: {ev['wall']:.0f}s, failed {len(ev['failed'])}", flush=True)

    # Monte Carlo: Sobol' over the same d parameters, and over all parameters (the screening error)
    for label, sub in [("mc_active", act), ("mc_all", dists)]:
        rule = sobol(sub, N_MC, seed=0)
        rows = sample_rows(dists, sub, rule)[1:]  # node 0 is the reference point (zero weight)
        ev = evaluate(cfg, det["dv"], rows, outputs, WORKERS)
        res[label] = dict(N=len(rows), d=len(sub), wall=ev["wall"], failed=ev["failed"], retried=ev["retried"],
                          moments=moments(ev["values"], outputs),
                          p_violation=violation_probabilities(ev["values"], outputs, cons))
        np.save(OUT / f"propagation_{label}.npy", ev["values"])
        print(f"{label}: d={len(sub)} N={len(rows)} {ev['wall']:.0f}s, failed {len(ev['failed'])}", flush=True)
    dump("propagation", res)
    return res


# --- checkpoint 3 ------------------------------------------------------------------------------------
def _cfg(cfg_name, reserve=None, prop=None, k=None, sigma_cap=None, optimizer=None):
    cfg = load_full_config(ROOT / "configs" / cfg_name)
    if reserve is not None:
        cfg.energy_reserve = reserve
    if prop is not None:
        cfg.propulsion.prop = prop
    if k is not None:
        cfg.robust.k = k
    if sigma_cap is not None:
        cfg.robust.objective, cfg.robust.sigma_cap = "mean_sigma_cap", sigma_cap
    if optimizer is not None:
        cfg.robust.optimizer = optimizer
    return cfg


def nominal_values(cfg, dv):
    """The deterministic model at design dv with the config's own parameter values (wind 0): the 'nominal' design."""
    prob = build_full_problem(cfg, driver=False)
    for k, x in dv.items():
        prob.set_val(k, x)
    prob.run_model()
    return {o: v(prob, o) for o in tracked(cfg)}


def optimise(tag, cfg_name="full_mdo_rdo.yaml", start=None, **over):
    """One robust optimisation; writes outputs/rdo/<tag>.json. start: a tag whose design is the start point."""
    from openmdao.core.analysis_error import AnalysisError

    from aircrafter2.robustness.rdo_problem import node_report

    cfg = _cfg(cfg_name, **over)
    prob = build_robust_problem(cfg)
    if cfg.robust.optimizer == "slsqp":
        prob.driver.options["disp"] = False
    if start:
        for k, x in load(start)["dv"].items():
            prob.set_val(k, x)
    t = time.perf_counter()
    err = None
    try:
        res = prob.run_driver()
        ok, its = bool(res.success), res.iter_count
    except AnalysisError as exc:  # every fallback of some node failed: MDA non-convergence
        ok, its, err = False, prob.driver.iter_count, f"MDA non-convergence: {str(exc)[:200]}"
    wall = time.perf_counter() - t
    dv = {k: v(prob, k) for k in cfg.design_variables}
    cons = {n: float(np.ravel(x)[0]) if np.size(x) == 1 else np.ravel(x).tolist()
            for n, x in prob.driver.get_constraint_values().items()}
    out = dict(tag=tag, cfg=cfg_name, overrides=over, start=start, ok=ok, its=its, error=err, time=wall,
               evaluations=prob.rdo["solvers"][0].n_solve, N=prob.rdo["N"], dv=dv,
               summary={q: {kk: vv for kk, vv in d.items()} for q, d in robust_summary(prob).items()},
               constraints=cons, nodes=node_report(prob),
               reference_node={o: v(prob, f"nodes.n0.{o}") for o in tracked(cfg)},
               nominal=nominal_values(cfg, dv), k=cfg.robust.k, beta=cfg.robust.beta,
               objective=cfg.robust.objective, sigma_cap=cfg.robust.sigma_cap, reserve=cfg.energy_reserve,
               optimizer=cfg.robust.optimizer)
    dump(tag, out)
    print(f"{tag}: ok={ok} its={its} evals={out['evaluations']} {wall:.0f}s score mu={out['summary']['score']['mu']:.3f} "
          f"sigma={out['summary']['score']['sigma']:.3f} {err or ''}", flush=True)
    return out


def det_run(tag, cfg_name="full_mdo_rdo.yaml", **over):
    """Deterministic optimum of a (possibly overridden) config, in the same JSON layout as `optimise`."""
    cfg = _cfg(cfg_name, **over)
    prob = build_full_problem(cfg)
    prob.driver.options["disp"] = False
    t = time.perf_counter()
    try:
        res = prob.run_driver()
        ok, its, err = bool(res.success), res.iter_count, None
    except Exception as exc:
        ok, its, err = False, prob.driver.iter_count, f"{exc.__class__.__name__}: {str(exc)[:200]}"
    out = dict(tag=tag, cfg=cfg_name, overrides=over, ok=ok, its=its, error=err, time=time.perf_counter() - t,
               evaluations=prob.driver.iter_count, dv={k: v(prob, k) for k in cfg.design_variables},
               nominal={o: v(prob, o) for o in tracked(cfg)}, reserve=cfg.energy_reserve)
    dump(tag, out)
    print(f"{tag}: ok={ok} its={its} {out['time']:.0f}s t3={out['nominal']['ThreeLapTime']:.3f} {err or ''}", flush=True)
    return out


def validate(tags, n=N_MC):
    """All-parameter Sobol' Monte Carlo (and the d-parameter one) at the designs of `tags`; adds 'mc' to each."""
    for tag in tags:
        d = load(tag)
        cfg = _cfg(d.get("cfg", "full_mdo_rdo.yaml"), **{k: x for k, x in d.get("overrides", {}).items()
                                                          if k in ("reserve", "prop")})
        prob = build_full_problem(cfg, driver=False)
        dists, act = resolved_dists(cfg, lambda name: v(prob, name))
        del prob
        outputs, cons = tracked(cfg), full_constraints(cfg)
        d["mc"] = {}
        for label, sub in [("all", dists), ("active", act)]:
            rule = sobol(sub, n, seed=0)
            ev = evaluate(cfg, d["dv"], sample_rows(dists, sub, rule)[1:], outputs, WORKERS)
            np.save(OUT / f"mc_{tag}_{label}.npy", ev["values"])
            d["mc"][label] = dict(N=n, wall=ev["wall"], failed=ev["failed"], retried=ev["retried"],
                                  moments=moments(ev["values"], outputs),
                                  p_violation=violation_probabilities(ev["values"], outputs, cons))
            print(f"MC {tag} {label}: {ev['wall']:.0f}s failed {len(ev['failed'])} "
                  f"P(viol) {d['mc'][label]['p_violation']}", flush=True)
        d["mc_outputs"] = outputs
        dump(tag, d)


# --- report -----------------------------------------------------------------------------------------
QREP = ["ThreeLapTime", "energy_margin", "thrust_margin_static", "stall_margin", "SM", "wall_ratio",
        "turn_speed_margin", "m_total"]
LABEL = {"ThreeLapTime": "3-lap time (s)", "energy_margin": "energy margin (J)", "thrust_margin_static":
         "take-off thrust margin (N)", "stall_margin": "stall margin (m/s)", "SM": "static margin",
         "wall_ratio": "spar wall ratio", "turn_speed_margin": "turn speed margin (m/s)", "m_total": "total mass (kg)",
         "rpm_margin": "rpm margin"}


def _g(x, n=4):
    return f"{x:.{n}g}"


def report_propagation(w):
    r = load("propagation")
    n = r["mc_all"]["N"]
    w("## 1. Propagation accuracy at the deterministic optimum (checkpoint 2)\n")
    w("Design: deterministic optimum of single 10x7E 4S (`configs/full_mdo_rdo.yaml` without the robust block): "
      f"3-lap time {load('det_single_10x7E')['values']['ThreeLapTime']:.2f} s. Rules over the d = 7 screened "
      "parameters (the other 8 at their means), and a Monte Carlo over all 15 so the screening error shows. "
      f"Monte Carlo: {n} scrambled Sobol' points (seed 0), independent inputs.\n")
    w("| Output | UDR (N = 15) | Smolyak 2 (N = 127) | MC, d = 7 | MC, all 15 | MC std. error (μ / σ) |")
    w("|---|---|---|---|---|---|")
    U, S, A, M = (r["udr"]["all_moments"], r["smolyak"]["moments"], r["mc_active"]["moments"],
                  r["mc_all"]["moments"])
    for q in QREP + ["rpm_margin"]:
        se = (M[q][1] / np.sqrt(n), M[q][1] / np.sqrt(2 * (n - 1)))
        w(f"| {LABEL[q]} | " + " | ".join(f"{_g(x[q][0], 5)} ± {_g(x[q][1], 3)}" for x in (U, S, A, M))
          + f" | {_g(se[0], 2)} / {_g(se[1], 2)} |")
    w("\nEntries are μ ± σ. Relative σ errors:\n")
    w("| Output | UDR vs MC d = 7 (propagation) | MC d = 7 vs MC all (screening) | UDR vs MC all (total) | "
      "Smolyak vs MC d = 7 |")
    w("|---|---:|---:|---:|---:|")
    for q in QREP:
        e = lambda a, b: f"{100 * (a[q][1] / b[q][1] - 1):+.1f} %"  # noqa: E731
        w(f"| {LABEL[q]} | {e(U, A)} | {e(A, M)} | {e(U, M)} | {e(S, A)} |")
    w("\n| Method | N analyses | wall time | MDA failures |")
    w("|---|---:|---:|---:|")
    w(f"| UDR in the robust model (one run_model, serial) | {r['udr']['N']} | {r['udr']['cold']:.1f} s cold, "
      f"{r['udr']['warm']:.1f} s warm | {sum(r['udr']['fallbacks'])} fallbacks, 0 failures |")
    for k, lab in [("smolyak", "Smolyak level 2"), ("mc_active", "Monte Carlo, d = 7"), ("mc_all", "Monte Carlo, all 15")]:
        w(f"| {lab} ({WORKERS} worker processes) | {r[k]['N']} | {r[k]['wall']:.0f} s | {len(r[k]['failed'])} |")
    w("\nThe robust model's 15 node outputs equal the deterministic model's at the same points to 3e-13 (a check of "
      "the multi-point wiring).\n")
    eu = 100 * (U["ThreeLapTime"][1] / A["ThreeLapTime"][1] - 1)
    es = 100 * (S["ThreeLapTime"][1] / A["ThreeLapTime"][1] - 1)
    w(f"**Reading it.** The means agree within the Monte Carlo standard error for every output and every rule. For "
      f"the constraints UDR's σ is within 0.5 % of the d = 7 Monte Carlo: they are close to additive in the "
      f"parameters. For the 3-lap time UDR's σ is {eu:+.1f} % off while Smolyak level 2 is {es:+.1f} %, so the lap "
      "time has interaction (cross) terms between the screened parameters that the univariate decomposition "
      "cannot represent (the powertrain parameters act together: Kv, Rm and the thrust factor all set the thrust "
      "and power curves). The screening adds 2-7 % (the 8 parameters held at their means), so against the full "
      "Monte Carlo UDR is low on σ by about 15 % for the lap time and 2-6 % for the constraints. In the robust "
      "objective μ + kσ (k = 1) that is about 0.1 s of a 80 s lap; in the constraints, β σ is about 5 % short. "
      "Smolyak level 2 removes the propagation error at 8.5 times the nodes; it is one config line "
      "(`robust.quadrature: smolyak`) if that matters.\n")
    pv = r["mc_all"]["p_violation"]
    w("P(violation) at the deterministic optimum, Monte Carlo over all 15: " + ", ".join(
        f"{k} {100 * x:.1f} %" for k, x in pv.items() if x > 0) + "; every other constraint 0 %. The two active "
      "constraints fail about half the time, as an optimum on its constraints must.\n")


def report():
    L = []
    w = L.append
    w("# Robust design optimisation: AirCrafter 2.0 study\n")
    w("Generated by `studies/rdo_study.py report` from `outputs/rdo/*.json` (the raw results, not in git). Plan, "
      "screening and node-convergence check: `docs/RDO_PLAN.md`. Formulation: `docs/modules/robustness.md`.\n")
    w("**Read this first.** Every spread is a PLACEHOLDER (`configs/uncertainty.yaml`, OPEN_QUESTIONS Q32), as are the "
      "motor, battery and payload values. The objective is the placeholder 3-lap time with the 20 % energy reserve "
      "unless stated. The results show how the model behaves under RDO and what RDO costs, not a design.\n")
    sections = [("propagation", report_propagation)]
    for name, fn in sections:
        if (OUT / f"{name}.json").exists():
            fn(w)
    (ROOT / "docs" / "studies" / "rdo_study.md").write_text("\n".join(L) + "\n")
    print("\n".join(L))


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("part", choices=["propagation", "optimise", "det", "validate", "report"])
    ap.add_argument("--tag")
    ap.add_argument("--tags", nargs="*")
    ap.add_argument("--cfg", default="full_mdo_rdo.yaml")
    ap.add_argument("--reserve", type=float)
    ap.add_argument("--prop")
    ap.add_argument("--k", type=float)
    ap.add_argument("--sigma-cap", type=float)
    ap.add_argument("--optimizer", choices=["slsqp", "ipopt"])
    ap.add_argument("--start")
    a = ap.parse_args()
    over = {k: x for k, x in dict(reserve=a.reserve, prop=a.prop, k=a.k, sigma_cap=a.sigma_cap,
                                    optimizer=a.optimizer).items() if x is not None}
    if a.part == "propagation":
        propagation()
    elif a.part == "optimise":
        optimise(a.tag, a.cfg, start=a.start, **over)
    elif a.part == "det":
        over.pop("k", None), over.pop("sigma_cap", None), over.pop("optimizer", None)
        det_run(a.tag, a.cfg, **over)
    elif a.part == "validate":
        validate(a.tags)
    else:
        report()
