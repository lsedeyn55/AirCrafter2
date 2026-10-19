"""RDO plan checks: dimension screening at the best deterministic design (single 10x7E 4S).

1. Deterministic optimum of configs/full_mdo.yaml with prop 10x7E (the current best single design).
2. First-order variance contributions (df/dp_i sigma_i)^2 from compute_totals at the reference point (every
   uncertain parameter at its mean), for the objective and the robust constraints; the smallest set (<= 5)
   covering 90 % of the objective and the active robust constraints.
3. Check of the first-order screen: the 3-point Gauss variance of each parameter alone (one-dimensional, the
   UDR's per-dimension term), which also sees curvature (e.g. the wind, whose effect is nearly even).

Writes outputs/rdo/screening.json. Run: .venv/bin/python studies/rdo_screening.py
"""

import json
import time
import warnings
from pathlib import Path

import numpy as np

from aircrafter2.problems.full_mdo import build_full_problem, load_full_config
from aircrafter2.robustness.quadrature import gauss_1d
from aircrafter2.robustness.uncertainty import DEFAULT_ROBUST, load_uncertainty, resolve, screen

warnings.filterwarnings("ignore")
ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "outputs" / "rdo"
QUANT = ["ThreeLapTime"] + DEFAULT_ROBUST + ["turn_speed_margin", "rpm_margin", "V_h", "V_v",
                                             "cruise.cl_margin_sec", "v_fit_margin"]


def v(p, k):
    return float(np.ravel(p.get_val(k))[0])


def active_constraints(prob, tol=1e-4):
    act = []
    for name, meta in prob.driver._cons.items():
        val = v(prob, name)
        for side in ("lower", "upper"):
            bd = meta[side]
            if bd is not None and abs(bd) < 1e29:
                scale = max(abs(bd), 1.0) if meta["ref"] is None else abs(meta["ref"])
                if abs(val - bd) <= tol * scale:
                    act.append((meta["name"] if "name" in meta else name, side))
    return act


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    cfg = load_full_config(ROOT / "configs" / "full_mdo.yaml")
    cfg.propulsion.prop = "10x7E"
    prob = build_full_problem(cfg)
    prob.driver.options["disp"] = False
    t = time.perf_counter()
    res = prob.run_driver()
    t_opt = time.perf_counter() - t
    dv = {k: v(prob, k) for k in cfg.design_variables}
    active = active_constraints(prob)
    print("det opt", res.success, res.iter_count, f"{t_opt:.0f}s", v(prob, "ThreeLapTime"), active, flush=True)
    det_vals = {q: v(prob, q) for q in QUANT}

    dists = resolve(load_uncertainty(ROOT / "configs" / "uncertainty.yaml"), lambda n: v(prob, n))
    nominal_cfg = {d.name: v(prob, d.name) for d in dists}
    t = time.perf_counter()
    sc = screen(prob, dists, QUANT, threshold=cfg.robust.screening_threshold, max_dim=cfg.robust.max_dim)
    t_screen = time.perf_counter() - t
    robust_active = [q for q, _ in active if q in DEFAULT_ROBUST]
    from aircrafter2.robustness.uncertainty import select_parameters
    contrib_sel = {q: sc["contrib"][q] for q in ["ThreeLapTime"] + robust_active}
    selected, coverage, reached = select_parameters(contrib_sel, cfg.robust.screening_threshold, cfg.robust.max_dim)
    print("selected", selected, coverage, reached, flush=True)

    # one-dimensional 3-point Gauss variance per parameter (the UDR's per-dimension term)
    t = time.perf_counter()
    oned = {q: {} for q in QUANT}
    failed = []
    for d in dists:
        xi, w = gauss_1d(d.kind, 3)
        rows = []
        for x in xi:
            prob.set_val(d.name, d.mean + d.scale * x)
            try:
                prob.run_model()
                rows.append([v(prob, q) for q in QUANT])
            except Exception as exc:  # MDA non-convergence
                failed.append((d.name, float(x), exc.__class__.__name__))
                rows.append([np.nan] * len(QUANT))
        prob.set_val(d.name, d.mean)
        F = np.array(rows)
        for j, q in enumerate(QUANT):
            mu = w @ F[:, j]
            oned[q][d.name] = float(w @ (F[:, j] - mu) ** 2)
    prob.run_model()
    t_oned = time.perf_counter() - t

    out = dict(dv=dv, t_opt=t_opt, its=res.iter_count, ok=bool(res.success), det_vals=det_vals,
               active=active, robust_active=robust_active,
               dists=[dict(name=d.name, kind=d.kind, mean=d.mean, scale=d.scale, std=d.std,
                           config_value=nominal_cfg[d.name]) for d in dists],
               ref_vals=sc["values"], contrib=sc["contrib"], oned=oned, selected=selected, coverage=coverage,
               reached=reached, selected_all_robust=sc["selected"], coverage_all_robust=sc["coverage"],
               t_screen=t_screen, t_oned=t_oned, oned_failed=failed)
    (OUT / "screening.json").write_text(json.dumps(out, indent=1, default=lambda o: o.item()))
    for q in ["ThreeLapTime"] + DEFAULT_ROBUST:
        tot = sum(sc["contrib"][q].values())
        tot1 = sum(oned[q].values())
        top = sorted(sc["contrib"][q].items(), key=lambda kv: -kv[1])[:6]
        print(f"{q}: sigma1 = {np.sqrt(tot):.4g}, sigma_1d = {np.sqrt(tot1):.4g}; "
              + ", ".join(f"{n} {100 * c / tot:.1f}%" if tot else n for n, c in top))
    print(f"screen {t_screen:.1f}s, 1-D checks {t_oned:.1f}s, failures {failed}")


if __name__ == "__main__":
    main()
