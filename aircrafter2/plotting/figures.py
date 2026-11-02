"""Thesis figures from a solved full-model problem (problems/full_mdo.py), called by api.py and the study scripts.

Every function takes the problem (already run), draws one figure and saves it as vector PDF. make_all writes the
standard set into a folder.
Abbreviations: MDA = multidisciplinary analysis.
"""

from pathlib import Path

import numpy as np
from matplotlib.patches import Circle, Rectangle

from ..airfoil import cst
from .style import PALETTE, figure, save


def v(p, k):
    """One scalar output of the problem.

    Parameters
    ----------
    p : om.Problem
    k : str

    Returns
    -------
    float
    """
    return float(np.ravel(p.get_val(k))[0])


def _has(p, k):
    """True if the problem has this variable."""
    try:
        p.get_val(k)
        return True
    except KeyError:
        return False


def _n_motors(p):
    """The number of motors, from the motor masses."""
    return int(round(v(p, "m_motor_group") / v(p, "m_motor_unit")))


def planform(p, path, underlay=None):
    """Top view of the design. underlay(ax), if given, draws first (e.g. the OpenVSP mock-up's top view)."""
    b, c, l_ht = v(p, "b"), v(p, "c"), v(p, "l_ht")
    Sh, Sv, ARh = v(p, "S_ht"), v(p, "S_vt"), v(p, "AR_ht")
    bh, ch = np.sqrt(Sh * ARh), np.sqrt(Sh / ARh)
    lcb, wcb = v(p, "l_cb"), v(p, "w_cb")
    fig, ax = figure(0.8, 0.75)
    if underlay is not None:
        underlay(ax)
    ax.add_patch(Rectangle((-b / 2, 0), b, c, fc=PALETTE[0], alpha=0.25, ec=PALETTE[0], label="wing"))
    x_tail = 0.25 * c + l_ht - 0.25 * ch
    ax.add_patch(
        Rectangle((-bh / 2, x_tail), bh, ch, fc=PALETTE[2], alpha=0.25, ec=PALETTE[2], label="horizontal tail")
    )
    x0 = 0.25 * c + v(p, "boom_offset")
    ax.plot([0, 0], [x0, 0.25 * c + l_ht], color="k", lw=2, label="tail boom")
    x_cb = v(p, "stations.x_cb")
    ax.add_patch(Rectangle((-wcb / 2, x_cb - lcb / 2), wcb, lcb, fc="none", ec=PALETTE[3], ls="--", label="cargo bay"))
    ax.plot(0, v(p, "x_cg"), "o", color=PALETTE[1], label=f"CG (SM = {v(p, 'SM'):.2f})")
    ax.plot(0, v(p, "x_np"), "^", color="k", label="neutral point")
    xm, Dp = v(p, "x_motor"), v(p, "D_prop")
    ys = [-v(p, "y_m"), v(p, "y_m")] if _has(p, "y_m") else [0.0]
    for i, y in enumerate(ys):
        ax.plot(y, xm, "s", color=PALETTE[5], label=f"motor{'s' if len(ys) > 1 else ''}" if i == 0 else None)
        ax.plot(
            [y - Dp / 2, y + Dp / 2],
            [xm - 0.01, xm - 0.01],
            color=PALETTE[5],
            lw=1.5,
            label=f"prop disc (D = {Dp * 39.37:.0f} in)" if i == 0 else None,
        )
    ax.set_aspect("equal")
    ax.invert_yaxis()
    ax.set_xlabel("y (m)")
    ax.set_ylabel("x (m), from wing root LE")
    ax.set_xlim(-b / 2 * 1.1, b / 2 * 1.1)
    ax.legend(loc="lower right", fontsize=7)
    ax.set_title(f"b = {b:.2f} m, c = {c:.3f} m, S_v = {Sv:.3f} m²")
    return save(fig, path)


def airfoil(p, path, seed=None):
    """The airfoil section, and the seed, if given.

    Parameters
    ----------
    p : om.Problem
    path : str or Path
    seed : dict, optional

    Returns
    -------
    str or Path
        path.
    """
    up, lo = p.get_val("upper_weights"), p.get_val("lower_weights")
    le, te = v(p, "le_weight"), v(p, "te_thickness")
    xy = cst.coordinates(up, lo, le, te, 151)
    fig, ax = figure(1.0, 0.32)
    if seed is not None:
        s = cst.coordinates(
            seed["upper_weights"], seed["lower_weights"], seed["leading_edge_weight"], seed["TE_thickness"], 151
        )
        ax.plot(s[:, 0], s[:, 1], "--", color=PALETTE[5], lw=1, label="seed")
    ax.plot(xy[:, 0], xy[:, 1], color=PALETTE[0], label=f"optimum (t/c = {v(p, 'tc_max'):.3f})")
    xs = v(p, "x_spar")
    yu, yl = cst.surfaces(np.array([xs]), up, lo, le, te)
    r = v(p, "r_tube") / v(p, "c_root")
    ax.add_patch(Circle((xs, 0.5 * (yu[0] + yl[0])), r, fc="none", ec=PALETTE[1], label="spar tube"))
    ax.set_aspect("equal")
    ax.set_xlabel("x / c")
    ax.set_ylabel("y / c")
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.45), ncol=3, fontsize=7, frameon=False)
    return save(fig, path)


def spanwise_lift(p, path):
    """Section c_l along the span at the cruise and turn points, with the section c_l,max.

    Parameters
    ----------
    p : om.Problem
    path : str or Path

    Returns
    -------
    str or Path
        path.
    """
    fig, ax = figure(1.0, 0.45)
    for name, col in [("cruise", PALETTE[0]), ("turn", PALETTE[1])]:
        y = p.get_val(f"{name}.mesh.mesh")[0, :, 1]
        ym = -0.5 * (y[1:] + y[:-1])
        cl = p.get_val(f"{name}.vlm.wing_perf.Cl")
        ax.plot(ym, cl, "-o", ms=2, color=col, label=f"{name}: $C_L$ = {v(p, name + '.vlm.wing_perf.CL'):.2f}")
    ym = -0.5 * (y[1:] + y[:-1])
    ax.plot(ym, p.get_val("turn.Clmax_sec"), ":", color="k", label=r"section $c_{l,max}$ (NeuralFoil)")
    ax.set_xlabel("spanwise position y (m), root to tip")
    ax.set_ylabel(r"section $c_l$")
    ax.legend(fontsize=7)
    return save(fig, path)


def drag_breakdown(p, path):
    """Drag breakdown.

    Parameters
    ----------
    p : om.Problem
    path : str or Path

    Returns
    -------
    str or Path
        path.
    """
    parts = {
        "induced (VLM)": v(p, "cruise.vlm.wing_perf.CDi"),
        "wing profile (NeuralFoil)": v(p, "cruise.CDp"),
        "tails": v(p, "CD_tail"),
        "fuselage": v(p, "CD_fus"),
    }
    if v(p, "CD_nac") > 0:
        parts["nacelles"] = v(p, "CD_nac")
    fig, ax = figure(0.8, 0.5)
    names, vals = list(parts), np.array(list(parts.values()))
    ax.barh(names, vals * 1e4, color=PALETTE[: len(names)])
    for i, val in enumerate(vals):
        ax.text(val * 1e4, i, f"  {val * 1e4:.0f} ({100 * val / vals.sum():.0f} %)", va="center", fontsize=7)
    ax.set_xlabel("drag coefficient at cruise (counts, 1e-4)")
    ax.set_title(f"cruise V = {v(p, 'v'):.1f} m/s, $C_D$ = {vals.sum():.4f}")
    ax.invert_yaxis()
    return save(fig, path)


def mass_breakdown(p, path):
    """Mass breakdown.

    Parameters
    ----------
    p : om.Problem
    path : str or Path

    Returns
    -------
    str or Path
        path.
    """
    groups = {
        "wing spar tube": v(p, "m_tube") * _wing_factor(p),
        "wing ribs, LE/TE, film": (v(p, "m_rib") + v(p, "m_le") + v(p, "m_te") + v(p, "m_film")) * _wing_factor(p),
        "tails": v(p, "m_ht") + v(p, "m_vt"),
        "tail boom / fuselage": v(p, "m_tb"),
        "cargo bay": v(p, "m_cb"),
        "landing gear": v(p, "m_gear"),
        f"motors, ESCs, props, nacelles (n = {_n_motors(p)})": v(p, "m_motor_group"),
        "battery": v(p, "m_batt"),
        "avionics": v(p, "m_avionics"),
        "payload": v(p, "m_pay"),
    }
    fig, (a1, a2) = figure(1.0, 0.42, ncols=2, gridspec_kw=dict(width_ratios=[1, 1.6], wspace=0.85))
    vals = np.array(list(groups.values()))
    cols = (PALETTE * 2)[: len(vals)]
    a1.pie(
        vals,
        colors=cols,
        startangle=90,
        counterclock=False,
        wedgeprops=dict(linewidth=0.5, edgecolor="white"),
        autopct=lambda pct: f"{pct:.0f} %" if pct >= 5 else "",
        pctdistance=0.72,
        textprops=dict(fontsize=6, color="white"),
    )
    a1.set_title(f"total {vals.sum():.2f} kg (empty {vals.sum() - v(p, 'm_pay'):.2f} kg)")
    a2.barh(list(groups), vals, color=cols)
    for i, val in enumerate(vals):
        a2.text(val, i, f" {1000 * val:.0f} g", va="center", fontsize=7)
    a2.invert_yaxis()
    a2.set_xlabel("mass (kg)")
    a2.grid(axis="y", visible=False)
    return save(fig, path)


def _wing_factor(p):
    """m_wing / m_wing_bare, or 1."""
    bare = v(p, "m_wing_bare")
    return v(p, "m_wing") / bare if bare else 1.0


def convergence(case_file, path, objective="score"):
    """Optimiser convergence from a case file.

    Parameters
    ----------
    case_file : path
    path : str or Path
    objective : str

    Returns
    -------
    str or Path
        path.
    """
    import openmdao.api as om

    cr = om.CaseReader(str(case_file))
    cases = cr.list_cases("driver", recurse=False, out_stream=None)
    obj, viol = [], []
    for cid in cases:
        case = cr.get_case(cid)
        obj.append(float(np.ravel(case.get_objectives()[objective])[0]))
        worst = 0.0
        cons = case.get_constraints()
        for name, val in cons.items():
            meta = cr.problem_metadata["variables"].get(name, {})
            lo, up = meta.get("lower"), meta.get("upper")
            val = np.ravel(val)
            if lo is not None and lo > -1e29:
                worst = max(worst, float(np.max(np.maximum(lo - val, 0) / max(abs(lo), 1.0))))
            if up is not None and up < 1e29:
                worst = max(worst, float(np.max(np.maximum(val - up, 0) / max(abs(up), 1.0))))
        viol.append(worst)
    fig, ax = figure(1.0, 0.45)
    ax.plot(range(len(obj)), obj, color=PALETTE[0], label="objective")
    ax.set_xlabel("driver iteration")
    ax.set_ylabel("objective")
    ax2 = ax.twinx()
    ax2.semilogy(range(len(viol)), np.maximum(viol, 1e-12), color=PALETTE[1], lw=1, label="max constraint violation")
    ax2.set_ylabel("max relative violation")
    ax2.grid(False)
    fig.legend(loc="upper right", fontsize=7)
    return save(fig, path)


def sensitivity_bars(sens, path, dvs=()):
    """Bar chart of the normalised sensitivities.

    Parameters
    ----------
    sens : dict
        {name: sensitivity}.
    path : str or Path
    dvs : iterable of str
        Design variables, coloured apart from the assumptions.

    Returns
    -------
    str or Path
        path.
    """
    items = sorted(sens.items(), key=lambda kv: abs(kv[1]))
    fig, ax = figure(1.0, 0.55)
    cols = [PALETTE[0] if k in dvs else PALETTE[3] for k, _ in items]
    ax.barh([k for k, _ in items], [s for _, s in items], color=cols)
    ax.axvline(0, color="k", lw=0.6)
    ax.set_xlabel("normalised sensitivity (x / f) df/dx")
    ax.set_title("blue: design variables, amber: assumptions")
    return save(fig, path)


def design_space(p, x1, x2, n, out, path, x1_range=None, x2_range=None):
    """Sweep two inputs around the current point (others fixed), full MDA per point; contour of `out`."""
    c1, c2 = v(p, x1), v(p, x2)
    r1 = x1_range or (0.8 * c1, 1.2 * c1)
    r2 = x2_range or (0.8 * c2, 1.2 * c2)
    g1, g2 = np.linspace(*r1, n), np.linspace(*r2, n)
    Z = np.full((n, n), np.nan)
    for i, a in enumerate(g1):
        for j, b in enumerate(g2):
            p.set_val(x1, a)
            p.set_val(x2, b)
            try:
                p.run_model()
                Z[j, i] = v(p, out)
            except Exception:
                pass
    p.set_val(x1, c1)
    p.set_val(x2, c2)
    p.run_model()
    fig, ax = figure(0.8, 0.75)
    cs = ax.contourf(g1, g2, Z, levels=14, cmap="viridis")
    ax.contour(g1, g2, Z, levels=14, colors="k", linewidths=0.3)
    fig.colorbar(cs, ax=ax, label=out)
    ax.plot(c1, c2, "*", color="w", ms=10, mec="k", label="optimum")
    units = {"b": "m", "c": "m", "l_ht": "m", "S_ht": "m²", "S_vt": "m²", "capacity": "Ah"}
    ax.set_xlabel(f"{x1} ({units[x1]})" if x1 in units else x1)
    ax.set_ylabel(f"{x2} ({units[x2]})" if x2 in units else x2)
    ax.set_title("other design variables at the optimum; constraints not shown", fontsize=7)
    ax.legend(fontsize=7)
    return save(fig, path)


def make_all(p, out_dir, seed=None, case_file=None, sens=None, dvs=(), design_space_args=None):
    """Write the standard figure set into a folder.

    Parameters
    ----------
    p : om.Problem
    out_dir : path
    seed : dict, optional
    case_file : path, optional
    sens : dict, optional
    dvs : iterable of str
    design_space_args : dict, optional
        x1, x2, n, out, x1_range, x2_range of design_space.

    Returns
    -------
    list
        The files written.
    """
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    files = [
        planform(p, out / "planform.pdf"),
        airfoil(p, out / "airfoil.pdf", seed),
        spanwise_lift(p, out / "spanwise_lift.pdf"),
        drag_breakdown(p, out / "drag_breakdown.pdf"),
        mass_breakdown(p, out / "mass_breakdown.pdf"),
    ]
    if case_file is not None:
        files.append(convergence(case_file, out / "convergence.pdf"))
    if sens:
        files.append(sensitivity_bars(sens, out / "sensitivities.pdf", dvs))
    if design_space_args:
        files.append(design_space(p, **design_space_args, path=out / "design_space.pdf"))
    return files
