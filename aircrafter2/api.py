"""High-level API used by the CLI and the Streamlit UI: run a single-point design and write its outputs.

    from aircrafter2.api import run_design
    result = run_design("configs/full_mdo.yaml", optimise=True, out_dir="outputs/demo")

Builds the problem from problems/full_mdo.py (or robustness/rdo_problem.py with an enabled robust: block), runs it, and
writes summary.md, results.json, constraints.json (the driver's constraints: bounds and values), geometry.json (3D
rebuild data), cases.sql (driver history), n2.html, xdsm.tex, xdsm.pdf and figures/*.pdf. With robust:, the summary adds
the robust statistics and the per-node MDA report, and the geometry, N2 and figures describe the robust design analysed
at the config's nominal parameter values.
"""

import json
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import openmdao.api as om

from .problems.full_mdo import (
    FullMDOConfig,
    build_full_problem,
    check_fixed_limits,
    full_constraints,
    kv_is_dv,
    load_full_config,
)

SUMMARY_KEYS = [
    ("score", "-", "objective (scoring plug-in)"),
    ("ThreeLapTime", "s", "3-lap time"),
    ("laps_frac", "-", "laps in the mission window"),
    ("b", "m", "span"),
    ("c", "m", "chord"),
    ("tc_max", "-", "t/c"),
    ("camber_scale", "-", "camber scale"),
    ("capacity", "Ah", "battery capacity per pack"),
    ("l_ht", "m", "tail arm"),
    ("S_ht", "m²", "HT area"),
    ("S_vt", "m²", "VT area"),
    ("x_batt", "m", "battery station"),
    ("x_cb_frac", "-", "cargo station / c"),
    ("v", "m/s", "cruise speed"),
    ("m_total", "kg", "total mass"),
    ("m_empty", "kg", "empty mass"),
    ("cruise.L_over_D", "-", "cruise L/D"),
    ("T_static", "N", "static thrust (all motors)"),
    ("E_nom_total", "W*h", "nominal battery energy (all packs)"),
    ("thrust_margin_static", "N", "take-off thrust margin"),
    ("energy_margin", "J", "3-lap energy margin (after reserve)"),
    ("stall_margin", "m/s", "V - 1.2 V_stall"),
    ("SM", "-", "static margin"),
    ("V_h", "-", "HT volume coeff."),
    ("V_v", "-", "VT volume coeff."),
    ("wall_ratio", "-", "spar t/r"),
    ("r_tube", "m", "spar radius"),
    ("I_margin", "A", "current margin"),
    ("rpm_margin", "rpm", "RPM margin at top speed"),
    # take-off in the mission time (course.takeoff.in_mission_time; all zero when it is off)
    ("t_takeoff", "s", "take-off ground roll time"),
    ("s_ground_roll", "m", "ground roll distance"),
    ("V_lof", "m/s", "lift-off speed"),
    ("E_takeoff", "J", "ground roll battery energy"),
    ("takeoff_accel_margin", "m/s²", "smallest ground-roll acceleration"),
    ("takeoff_straight_margin", "m", "first straight - ground roll"),
]


@dataclass
class RunResult:
    """What a run returns.

    Attributes
    ----------
    prob : om.Problem
    cfg : FullMDOConfig
    success : bool
    iterations : int
    values : dict
        The results written to results.json.
    files : list
        The files written.
    """

    prob: om.Problem
    cfg: FullMDOConfig
    success: bool
    iterations: int
    values: dict = field(default_factory=dict)
    files: list = field(default_factory=list)


def _val(p, k):
    """One scalar output of the problem."""
    return float(np.ravel(p.get_val(k))[0])


def summary_table(p):
    """The summary rows of a run problem.

    Parameters
    ----------
    p : om.Problem

    Returns
    -------
    list of tuple
        (name, unit, description, value) for each of SUMMARY_KEYS.
    """
    return [(k, unit, desc, _val(p, k)) for k, unit, desc in SUMMARY_KEYS]


def leg_breakdown(p, path="lap"):
    """Per-leg table of lap 1 and a lap 2+ of a run problem (SmoothLapModel.leg_breakdown), or None.

    Parameters
    ----------
    p : om.Problem
    path : str
        The lap model's path.

    Returns
    -------
    list of dict or None
    """
    try:
        return p.model._get_subsystem(path).leg_breakdown()
    except AttributeError:
        return None


def leg_lines(rows):
    """Markdown table of a leg breakdown.

    Parameters
    ----------
    rows : list of dict or None

    Returns
    -------
    list of str
    """
    if not rows:
        return []
    out = [
        "\n## Leg breakdown\n",
        "| lap | leg | kind | size | wind | v entry (m/s) | v exit (m/s) | time (s) | energy (kJ) |",
        "|---:|---:|---|---:|---|---:|---:|---:|---:|",
    ]
    for r in rows:
        unit = "deg" if r["kind"] == "turn" else "ft"
        out.append(
            f"| {'1' if r['lap'] == 1 else '2+'} | {r['leg']} | {r['kind']} | {r['size']:.0f} {unit} | "
            f"{r['wind']} | {r['v_entry']:.2f} | {r['v_exit']:.2f} | {r['time']:.2f} | {r['energy'] / 1e3:.2f} |"
        )
    return out


# placeholder model assumptions for the sensitivity report, by motor mode (design variables are never repeated here)
ASSUMPTIONS = ["m_pay", "Kv", "Rm", "R_cell", "e_spec", "f_tb", "f_cb", "m_avionics"]
ASSUMPTIONS_KV_DV = ["m_pay", "Kv_factor", "Rm_ref", "R_cell", "e_spec", "f_tb", "f_cb", "m_avionics"]


def sensitivity_assumptions(cfg, extra=()):
    """The assumption list for a config's motor mode, without its design variables or duplicates.

    Parameters
    ----------
    cfg : FullMDOConfig
    extra : iterable of str
        More names.

    Returns
    -------
    list of str
    """
    base = ASSUMPTIONS_KV_DV if kv_is_dv(cfg) else ASSUMPTIONS
    out = []
    for k in list(base) + list(extra):
        if k not in cfg.design_variables and k not in out:
            out.append(k)
    return out


def motor_report(prob, cfg, tol=1e-4):
    """Kv, the (scaled) Rm and I0, the static current against its limit, and what set Kv.

    'set by' names a Kv bound within tol (relative to the bound range) or the current, pack-current or rpm constraints
    that are active (|margin| / ref < tol); otherwise 'interior'.

    Parameters
    ----------
    prob : om.Problem
    cfg : FullMDOConfig
    tol : float

    Returns
    -------
    dict
    """
    v = lambda k: _val(prob, k)  # noqa: E731
    pr = cfg.propulsion
    out = dict(
        kv_dv=kv_is_dv(cfg),
        Kv=v("Kv"),
        Rm=v("Rm"),
        I0=v("I0"),
        I_static=v("I_static"),
        I_max=pr.I_max,
        I_margin=v("I_margin"),
        rpm_margin=v("rpm_margin"),
        Kv_ref=pr.Kv,
        Rm_ref=pr.Rm,
        I0_ref=pr.I0,
    )
    if not out["kv_dv"]:
        out["set_by"] = "fixed (not a design variable)"
        return out
    bd = cfg.design_variables["Kv"]
    span = bd.upper - bd.lower
    why = [
        f"{side} bound ({b:g} rpm/V)"
        for side, b in (("lower", bd.lower), ("upper", bd.upper))
        if abs(out["Kv"] - b) <= tol * span
    ]
    refs = dict(full_constraints(cfg))
    for name in ("I_margin", "I_pack_margin", "rpm_margin"):
        if name in refs and abs(v(name)) / (refs[name].get("ref") or 1.0) < tol:
            why.append(f"{name} = 0 (active)")
    out["set_by"] = ", ".join(why) if why else "interior (no Kv bound or current/rpm constraint active)"
    return out


def switched_off(cfg):
    """The constraints this run leaves out: disabled_constraints, and energy_margin without require_3lap_energy.

    Parameters
    ----------
    cfg : FullMDOConfig

    Returns
    -------
    list of str
    """
    return list(cfg.disabled_constraints) + ([] if cfg.require_3lap_energy else ["energy_margin"])


def constraint_values(prob, cons):
    """The value of each constraint on a run problem.

    Parameters
    ----------
    prob : om.Problem
    cons : list of (str, dict)
        (name, add_constraint kwargs).

    Returns
    -------
    list of dict
        name, lower, upper, equals, ref, units, value, values. value is the element nearest its bound (vector
        constraints, e.g. cruise.cl_margin_sec).
    """
    out = []
    for name, kw in cons:
        vals = np.ravel(prob.get_val(name, units=kw.get("units"))).astype(float)
        lo, hi, eq = kw.get("lower"), kw.get("upper"), kw.get("equals")
        slack = np.full(vals.shape, np.inf)
        if lo is not None:
            slack = np.minimum(slack, vals - lo)
        if hi is not None:
            slack = np.minimum(slack, hi - vals)
        if eq is not None:
            slack = np.minimum(slack, -np.abs(vals - eq))
        out.append(
            dict(
                name=name,
                lower=lo,
                upper=hi,
                equals=eq,
                ref=kw.get("ref"),
                units=kw.get("units"),
                value=float(vals[int(np.argmin(slack))]),
                values=vals.tolist(),
            )
        )
    return out


KV_INPUT_RANGE = (100.0, 5000.0)  # rpm/V, the range the UI accepts for any Kv entry


def validate_kv_dv(lower, upper, start, allowed=KV_INPUT_RANGE):
    """Messages for invalid Kv design-variable settings.

    Parameters
    ----------
    lower, upper, start : float
        rpm/V.
    allowed : (float, float)
        The range the bounds must lie within.

    Returns
    -------
    list of str
        Empty if valid.
    """
    errs = []
    if not lower < upper:
        errs.append(f"Kv lower bound ({lower:g}) must be below the upper bound ({upper:g}).")
    if not lower <= start <= upper:
        errs.append(f"Kv start ({start:g}) must lie within the bounds [{lower:g}, {upper:g}].")
    if lower < allowed[0] or upper > allowed[1]:
        errs.append(f"Kv bounds must lie within {allowed[0]:g}-{allowed[1]:g} rpm/V.")
    return errs


def reanalyse_with_motor(cfg, dv, Kv, Rm, I0, m_motor, reference=None):
    """Re-analyse a design (no optimiser) with a real motor instead of the continuous-Kv one.

    Builds the fixed-motor problem and runs the coupled analysis. No motor catalogue is built in: the caller supplies
    the motor (placeholder, docs/OPEN_QUESTIONS.md Q7).

    Parameters
    ----------
    cfg : FullMDOConfig
        The config the design came from.
    dv : dict
        {design variable: value}. Kv, if present, is replaced by the real motor.
    Kv, Rm, I0, m_motor : float
        rpm/V, ohm, A, kg.
    reference : dict, optional
        {output: value} of the original design.

    Returns
    -------
    dict
        values, motor, limits, and change if reference is given.
    """
    c = cfg.model_copy(deep=True)
    c.design_variables.pop("Kv", None)
    c.initial.pop("Kv", None)
    c.robust.enabled = False
    c.robust.active = None
    c.propulsion.Kv, c.propulsion.Rm, c.propulsion.I0, c.propulsion.m_motor = Kv, Rm, I0, m_motor
    prob = build_full_problem(c, driver=False)
    for name, val in dv.items():
        if name != "Kv":
            prob.set_val(name, val)
    prob.run_model()
    names = [n for n, _ in full_constraints(cfg)] + ["I_margin"]
    vals = {"score": _val(prob, "score"), "ThreeLapTime": _val(prob, "ThreeLapTime"), "m_total": _val(prob, "m_total")}
    vals.update({n: _val(prob, n) for n in dict.fromkeys(names)})
    out = dict(values=vals, motor=dict(Kv=Kv, Rm=Rm, I0=I0, m_motor=m_motor), limits=check_fixed_limits(prob))
    if reference:
        out["change"] = {k: vals[k] - reference[k] for k in vals if k in reference}
    return out


def _run_robust(cfg, optimise, progress):
    """Robust problem: (deterministic problem at the robust design, success, iterations, robust values, constraints)."""
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
    robust = dict(
        N=rp.rdo["N"],
        rule=rp.rdo["rule"].name,
        active=[d.name for d in rp.rdo["active"]],
        k=cfg.robust.k,
        beta=cfg.robust.beta,
        objective=cfg.robust.objective,
        sigma_cap=cfg.robust.sigma_cap,
        time_s=time.perf_counter() - t,
        error=error,
        stats={q: {k: v for k, v in d.items()} for q, d in robust_summary(rp).items()},
        nodes=node_report(rp),
    )
    if optimise and not error:
        robust["objective_value"] = float(
            np.ravel(
                rp.get_val("stats.score_robust" if cfg.robust.objective == "mean_plus_k_sigma" else "stats.score_mu")
            )[0]
        )
    from .robustness.rdo_problem import robust_constraints
    from .scoring import get_plugin

    plugin = get_plugin(cfg.scoring.plugin, **cfg.scoring.options)
    cons = constraint_values(rp, robust_constraints(cfg, full_constraints(cfg), rp.rdo["quantities"][1:], plugin))
    prob = build_full_problem(cfg, driver=False)  # the robust design, at the config's nominal parameter values
    for name in cfg.design_variables:
        prob.set_val(name, rp.get_val(name))
    prob.run_model()
    return prob, success, its, robust, cons


def run_design(
    config,
    optimise=True,
    out_dir=None,
    figures=True,
    sensitivities=True,
    design_space=False,
    record=True,
    n2=True,
    progress=None,
):
    """Optimise or analyse a config, and write its outputs.

    Parameters
    ----------
    config : str, Path or FullMDOConfig
    optimise : bool
        Run the optimiser. Otherwise one analysis.
    out_dir : str or Path, optional
        Where to write. Without it, nothing is written.
    figures, sensitivities, design_space, record, n2 : bool
        What to produce. design_space sweeps +/-20 % around the optimum (slow).
    progress : callable, optional
        progress(message).

    Returns
    -------
    RunResult
    """
    cfg = config if isinstance(config, FullMDOConfig) else load_full_config(config)
    out = Path(out_dir) if out_dir else None
    if out:
        out.mkdir(parents=True, exist_ok=True)
    if cfg.robust.enabled:
        if progress:
            progress("solving (robust)")
        prob, success, its, robust, cons = _run_robust(cfg, optimise, progress)
        return _finish(
            prob,
            cfg,
            config,
            success,
            its,
            out,
            figures,
            sensitivities,
            design_space,
            None,
            n2,
            progress,
            optimise,
            robust,
            cons,
        )
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
    return _finish(
        prob, cfg, config, success, its, out, figures, sensitivities, design_space, case_file, n2, progress, optimise
    )


def _finish(
    prob,
    cfg,
    config,
    success,
    its,
    out,
    figures,
    sensitivities,
    design_space,
    case_file,
    n2,
    progress,
    optimise,
    robust=None,
    cons=None,
):
    """Collect a run's values and write its outputs."""
    if cons is None:  # deterministic: add_full_driver's list (the plug-in's constraints, then full_constraints)
        from .scoring import get_plugin

        plugin = get_plugin(cfg.scoring.plugin, **cfg.scoring.options)
        cons = constraint_values(prob, plugin.constraints() + full_constraints(cfg))
    values = {k: v for k, _, _, v in summary_table(prob)}
    if robust is not None:
        values["robust"] = robust
    values["limits"] = check_fixed_limits(prob)
    values["motor"] = motor_report(prob, cfg)
    from .stability import oei_check

    values["oei"] = oei_check(prob, cfg)
    values["course"] = cfg.course.summary()
    legs = leg_breakdown(prob)
    if legs is not None:
        values["legs"] = legs
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
        sens = sens_fn(prob, "score", dvs + sensitivity_assumptions(cfg, extra))
        values["sensitivities"] = sens
    (out / "results.json").write_text(json.dumps(values, indent=2, default=float))
    (out / "constraints.json").write_text(
        json.dumps(
            dict(disabled=switched_off(cfg), tol=cfg.tol, robust=robust is not None, constraints=cons),
            indent=2,
            default=float,
        )
    )
    from .geometry.export import export_geometry, write_geometry

    geo = export_geometry(
        prob,
        cfg,
        config_path=None if isinstance(config, FullMDOConfig) else config,
        optimised=optimise,
        success=success,
        case_id=out.name,
    )
    result.files.append(write_geometry(geo, out / "geometry.json"))
    lines = [
        "# AirCrafter 2.0 result\n",
        f"Config: scoring = `{cfg.scoring.plugin}`, prop = "
        f"`{cfg.propulsion.prop}`, {cfg.propulsion.n_motors} motor(s), {cfg.propulsion.battery_topology} "
        f"battery. {'Optimised' if optimise else 'Analysed'}: success = {success}, "
        f"iterations = {its}.\n",
        "| Quantity | Value | Unit |",
        "|---|---:|---|",
    ]
    off = switched_off(cfg)
    if off:  # the UI's Constraints tab switched constraints off: say so first
        lines.insert(1, f"**Constraints switched off for this run: {', '.join(f'`{n}`' for n in off)}.**\n")
    if robust is not None:
        lines[1 + bool(off)] = lines[1 + bool(off)].replace("Optimised", "Robust-optimised")
        top = 2 + bool(off)
        lines[top:top] = _robust_lines(robust)
        lines.insert(len(_robust_lines(robust)) + top, "Design analysed at the config's nominal parameter values:\n")
    lines += [f"| {desc} (`{k}`) | {v:.5g} | {unit} |" for k, unit, desc, v in summary_table(prob)]
    lines += _motor_lines(values["motor"])
    co = values["course"]
    lines += [
        f"\nCourse: {co['preset'] or 'custom'} ({co['lap_length_ft']:.0f} ft of straights per lap), pilot reaction "
        f"time {co['pilot_reaction_time_s']:g} s, take-off in the mission time: "
        f"{'on' if co['takeoff']['in_mission_time'] else 'off'}."
    ]
    lines += leg_lines(values.get("legs"))
    (out / "summary.md").write_text("\n".join(lines) + "\n")
    result.files += [out / "results.json", out / "constraints.json", out / "summary.md"]
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
        result.files += F.make_all(
            prob,
            out / "figures",
            seed=load_seed(cfg.airfoil.seed),
            case_file=case_file,
            sens=sens,
            dvs=dvs,
            design_space_args=ds,
        )
        result.files += [xdsm.write_tex(out / "xdsm.tex"), xdsm.draw_pdf(out / "xdsm.pdf")]
    return result


def _motor_lines(mr):
    """The summary's motor section, as Markdown lines."""
    head = (
        "Motor Kv (design variable; the reference motor rewound, docs/modules/propulsion.md)"
        if mr["kv_dv"]
        else "Motor (fixed)"
    )
    L = [
        f"\n**{head}.** Kv = {mr['Kv']:.1f} rpm/V, set by: {mr['set_by']}.\n",
        "| Quantity | Value | Unit |",
        "|---|---:|---|",
        f"| Kv | {mr['Kv']:.5g} | rpm/V |",
        f"| Rm | {mr['Rm']:.5g} | ohm |",
        f"| I0 | {mr['I0']:.5g} | A |",
        f"| static current per motor `I_static` | {mr['I_static']:.4g} | A |",
        f"| current limit `I_max` | {mr['I_max']:.4g} | A |",
        f"| `rpm_margin` | {mr['rpm_margin']:.4g} | rpm |",
    ]
    if mr["kv_dv"]:
        L.append(
            f"| reference motor (Kv, Rm, I0) | {mr['Kv_ref']:g}, {mr['Rm_ref']:g}, {mr['I0_ref']:g} | rpm/V, ohm, A |"
        )
    return L


def _robust_lines(robust):
    """The summary's robust statistics, as Markdown lines."""
    L = [
        f"Robust statistics: {robust['rule']} over {', '.join(robust['active'])} ({robust['N']} nodes), "
        f"objective {robust['objective']} (k = {robust['k']:g}"
        + (f", sigma cap {robust['sigma_cap']:g}" if robust["sigma_cap"] else "")
        + f"), beta = {robust['beta']:g}; {robust['time_s']:.0f} s."
        + (f" **{robust['error']}**" if robust.get("error") else "")
        + "\n",
        "| Quantity | mu | sigma | mu - beta sigma | mu + beta sigma |",
        "|---|---:|---:|---:|---:|",
    ]
    L += [
        f"| `{q}` | {d['mu']:.5g} | {d['sigma']:.4g} | {d['lo']:.5g} | {d['hi']:.5g} |"
        for q, d in robust["stats"].items()
    ]
    fails = sum(n["failed_attempts"] for n in robust["nodes"])
    L.append(
        f"\nNode MDAs: {fails} failed attempt(s) over {sum(n['attempts'] for n in robust['nodes'])}, all "
        "recovered by a warm-start fallback.\n"
        if fails
        else "\nNode MDAs: every solve converged at the first attempt.\n"
    )
    return L
