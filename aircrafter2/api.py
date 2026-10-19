"""High-level API used by the CLI and the Streamlit UI.

    from aircrafter2.api import run_design
    result = run_design("configs/full_mdo.yaml", optimise=True, out_dir="outputs/demo")

Writes into out_dir: summary.md, results.json, geometry.json (3D rebuild data), cases.sql (driver history), n2.html, xdsm.tex,
xdsm.pdf and figures/*.pdf.

With an enabled ``robust:`` block (e.g. configs/full_mdo_rdo.yaml) the robust problem is optimised instead
(aircrafter2.robustness.rdo_problem); the summary adds the robust statistics (mu, sigma, mu -/+ beta sigma) and the
per-node MDA report, and the geometry, N2 and figures describe the robust design analysed at the config's nominal
parameter values.
"""

import json
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import openmdao.api as om

from .problems.full_mdo import FullMDOConfig, build_full_problem, check_fixed_limits, load_full_config

SUMMARY_KEYS = [
    ("score", "-", "objective (scoring plug-in)"), ("ThreeLapTime", "s", "3-lap time"),
    ("laps_frac", "-", "laps in the mission window"), ("b", "m", "span"), ("c", "m", "chord"),
    ("tc_max", "-", "t/c"), ("camber_scale", "-", "camber scale"), ("capacity", "Ah", "battery capacity per pack"),
    ("l_ht", "m", "tail arm"), ("S_ht", "m²", "HT area"), ("S_vt", "m²", "VT area"),
    ("x_batt", "m", "battery station"), ("x_cb_frac", "-", "cargo station / c"), ("v", "m/s", "cruise speed"),
    ("m_total", "kg", "total mass"), ("m_empty", "kg", "empty mass"), ("cruise.L_over_D", "-", "cruise L/D"),
    ("T_static", "N", "static thrust (all motors)"), ("E_nom_total", "W*h", "nominal battery energy (all packs)"),
    ("thrust_margin_static", "N", "take-off thrust margin"),
    ("energy_margin", "J", "3-lap energy margin (after reserve)"), ("stall_margin", "m/s", "V - 1.2 V_stall"),
    ("SM", "-", "static margin"), ("V_h", "-", "HT volume coeff."), ("V_v", "-", "VT volume coeff."),
    ("wall_ratio", "-", "spar t/r"), ("r_tube", "m", "spar radius"), ("I_margin", "A", "current margin"),
    ("rpm_margin", "rpm", "RPM margin at top speed"),
]


@dataclass
class RunResult:
    prob: om.Problem
    cfg: FullMDOConfig
    success: bool
    iterations: int
    values: dict = field(default_factory=dict)
    files: list = field(default_factory=list)


def _val(p, k):
    return float(np.ravel(p.get_val(k))[0])


def summary_table(p):
    return [(k, unit, desc, _val(p, k)) for k, unit, desc in SUMMARY_KEYS]


def _run_robust(cfg, optimise, progress):
    """Robust problem: returns (deterministic problem at the robust design, success, iterations, robust values)."""
    import time

    from openmdao.core.analysis_error import AnalysisError

    from .robustness.rdo_problem import build_robust_problem, node_report, robust_summary

    rp = build_robust_problem(cfg, driver=optimise)
    if progress:
        progress(f"robust problem: {rp.rdo['N']} nodes, {cfg.robust.quadrature}, d = {len(rp.rdo['active'])}")
    t = time.perf_counter()
    error = None
    if optimise:
        if cfg.robust.optimizer == "slsqp":
            rp.driver.options["disp"] = False
        try:
            res = rp.run_driver()
            success, its = bool(res.success), int(res.iter_count)
        except AnalysisError as exc:  # every fallback of some node failed
            success, its, error = False, int(rp.driver.iter_count), f"MDA non-convergence: {str(exc)[:300]}"
    else:
        rp.run_model()
        success, its = True, 0
    robust = dict(N=rp.rdo["N"], rule=rp.rdo["rule"].name, active=[d.name for d in rp.rdo["active"]],
                  k=cfg.robust.k, beta=cfg.robust.beta, objective=cfg.robust.objective,
                  sigma_cap=cfg.robust.sigma_cap, time_s=time.perf_counter() - t, error=error,
                  stats={q: {k: v for k, v in d.items()} for q, d in robust_summary(rp).items()},
                  nodes=node_report(rp))
    if optimise and not error:
        robust["objective_value"] = float(np.ravel(rp.get_val("stats.score_robust" if cfg.robust.objective ==
                                                            "mean_plus_k_sigma" else "stats.score_mu"))[0])
    prob = build_full_problem(cfg, driver=False)  # the robust design, at the config's nominal parameter values
    for name in cfg.design_variables:
        prob.set_val(name, rp.get_val(name))
    prob.run_model()
    return prob, success, its, robust


def run_design(config, optimise=True, out_dir=None, figures=True, sensitivities=True, design_space=False,
               record=True, n2=True, progress=None):
    cfg = config if isinstance(config, FullMDOConfig) else load_full_config(config)
    out = Path(out_dir) if out_dir else None
    if out:
        out.mkdir(parents=True, exist_ok=True)
    if cfg.robust.enabled:
        if progress:
            progress("solving (robust)")
        prob, success, its, robust = _run_robust(cfg, optimise, progress)
        return _finish(prob, cfg, config, success, its, out, figures, sensitivities, design_space, None, n2,
                       progress, optimise, robust)
    prob = build_full_problem(cfg, driver=optimise)
    case_file = None
    if optimise and record and out:
        case_file = out / "cases.sql"
        rec = om.SqliteRecorder(str(case_file))
        prob.driver.add_recorder(rec)
        prob.driver.recording_options["includes"] = []
        prob.final_setup()
    if progress:
        progress("solving")
    if optimise:
        prob.driver.options["disp"] = False
        res = prob.run_driver()
        success, its = bool(res.success), int(res.iter_count)
        prob.cleanup()
    else:
        prob.run_model()
        success, its = True, 0
    return _finish(prob, cfg, config, success, its, out, figures, sensitivities, design_space, case_file, n2,
                   progress, optimise)


def _finish(prob, cfg, config, success, its, out, figures, sensitivities, design_space, case_file, n2, progress,
            optimise, robust=None):
    values = {k: v for k, _, _, v in summary_table(prob)}
    if robust is not None:
        values["robust"] = robust
    values["limits"] = check_fixed_limits(prob)
    from .stability import oei_check

    values["oei"] = oei_check(prob, cfg)
    result = RunResult(prob=prob, cfg=cfg, success=success, iterations=its, values=values)
    if not out:
        return result

    sens = None
    dvs = list(cfg.design_variables)
    if sensitivities:
        from .robustness.suite import sensitivities as sens_fn

        if progress:
            progress("sensitivities")
        extra = []
        if cfg.propulsion.n_motors == 2:  # twin-layout assumptions
            extra = ["m_nacelle", "Q_nac", "y_m" if cfg.propulsion.station_mode == "absolute" else "y_motor"]
        sens = sens_fn(prob, "score", dvs + ["m_pay", "Kv", "Rm", "R_cell", "e_spec", "f_tb", "f_cb", "m_avionics"]
                       + [k for k in extra if k not in dvs])
        values["sensitivities"] = sens
    (out / "results.json").write_text(json.dumps(values, indent=2, default=float))
    from .geometry.export import export_geometry, write_geometry

    geo = export_geometry(prob, cfg, config_path=None if isinstance(config, FullMDOConfig) else config,
                          optimised=optimise, success=success, case_id=out.name)
    result.files.append(write_geometry(geo, out / "geometry.json"))
    lines = [f"# AirCrafter 2.0 result\n", f"Config: scoring = `{cfg.scoring.plugin}`, prop = "
             f"`{cfg.propulsion.prop}`, {cfg.propulsion.n_motors} motor(s), {cfg.propulsion.battery_topology} "
             f"battery. {'Optimised' if optimise else 'Analysed'}: success = {success}, "
             f"iterations = {its}.\n", "| Quantity | Value | Unit |", "|---|---:|---|"]
    if robust is not None:
        lines[1] = lines[1].replace("Optimised", "Robust-optimised")
        lines[2:2] = _robust_lines(robust)
        lines.insert(len(_robust_lines(robust)) + 2, "Design analysed at the config's nominal parameter values:\n")
    lines += [f"| {desc} (`{k}`) | {v:.5g} | {unit} |" for k, unit, desc, v in summary_table(prob)]
    (out / "summary.md").write_text("\n".join(lines) + "\n")
    result.files += [out / "results.json", out / "summary.md"]
    if n2:
        om.n2(prob, outfile=str(out / "n2.html"), display_in_notebook=False, show_browser=False)
        result.files.append(out / "n2.html")
    if figures:
        from .airfoil import load_seed
        from .plotting import figures as F
        from .plotting import xdsm

        if progress:
            progress("figures")
        ds = None
        if design_space:  # sweep +/-20 % around the optimum, clipped to the DV bounds
            rng = {}
            for name in ("b", "c"):
                x0, bd = _val(prob, name), cfg.design_variables[name]
                rng[name] = (max(bd.lower, 0.8 * x0), min(bd.upper, 1.2 * x0))
            ds = dict(x1="b", x2="c", n=7, out="score", x1_range=rng["b"], x2_range=rng["c"])
        result.files += F.make_all(prob, out / "figures", seed=load_seed(cfg.airfoil.seed), case_file=case_file,
                                   sens=sens, dvs=dvs, design_space_args=ds)
        result.files += [xdsm.write_tex(out / "xdsm.tex"), xdsm.draw_pdf(out / "xdsm.pdf")]
    return result


def _robust_lines(robust):
    L = [f"Robust statistics: {robust['rule']} over {', '.join(robust['active'])} ({robust['N']} nodes), "
         f"objective {robust['objective']} (k = {robust['k']:g}"
         + (f", sigma cap {robust['sigma_cap']:g}" if robust['sigma_cap'] else "")
         + f"), beta = {robust['beta']:g}; {robust['time_s']:.0f} s."
         + (f" **{robust['error']}**" if robust.get("error") else "") + "\n",
         "| Quantity | mu | sigma | mu - beta sigma | mu + beta sigma |", "|---|---:|---:|---:|---:|"]
    L += [f"| `{q}` | {d['mu']:.5g} | {d['sigma']:.4g} | {d['lo']:.5g} | {d['hi']:.5g} |"
          for q, d in robust["stats"].items()]
    fails = sum(n["failed_attempts"] for n in robust["nodes"])
    L.append(f"\nNode MDAs: {fails} failed attempt(s) over {sum(n['attempts'] for n in robust['nodes'])}, all "
             "recovered by a warm-start fallback.\n" if fails else "\nNode MDAs: every solve converged at the first "
             "attempt.\n")
    return L
