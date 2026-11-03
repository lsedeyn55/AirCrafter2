"""The thesis exhibits of the 2025-26 results: docs/studies/results_2026/figures/*.pdf (+ .png) and tables/*.tex (booktabs) +
.csv, all from the JSON, in one style (aircrafter2/plotting/style.py), sized for a single-column A4 page.

    .venv/bin/python studies/results_2026_exhibits.py        # after results_2026_report.py and the two other pages

Every figure reads in greyscale: line styles, marker shapes or hatches carry each series' identity, not only colour.
"""

import csv
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "studies"))
import results_2026_report as P  # noqa: E402
from results_2026_report import LABELS, MAIN, FOUR, nominal  # noqa: E402

from aircrafter2.plotting.figures import OVERLAY_STYLES, planform_overlay  # noqa: E402
from aircrafter2.plotting.style import PALETTE, figure, save  # noqa: E402

DOCS = P.DOCS
FIG = DOCS / "figures"
TAB = DOCS / "tables"
HATCH = ["", "///", "\\\\\\", "xxx", "...", "+++", "ooo", "---", "|||", "***"]
MARK = ["o", "s", "^", "D", "v", "P"]
SHORT = {"humpback": "Humpback", "humpback_loo": "Humpback LOO", "ac1_direct": "AC1 (a)", "ac1_completed": "AC1 (b)", "mdo": "MDO", "mdo_wind": "MDO wind", "rdo": "RDO"}
COLOR = {lab: PALETTE[i] for i, lab in enumerate(["humpback", "ac1_direct", "ac1_completed", "mdo", "mdo_wind", "rdo", "humpback_loo"])}
STYLE = {lab: OVERLAY_STYLES[i % len(OVERLAY_STYLES)] for i, lab in enumerate(["humpback", "ac1_direct", "ac1_completed", "mdo", "mdo_wind", "rdo"])}
STYLE["humpback_loo"] = (0, (2, 2))


def _save(fig, name):
    FIG.mkdir(parents=True, exist_ok=True)
    save(fig, FIG / f"{name}.pdf")
    return f"figures/{name}.pdf"


def _design(R, lab):
    n = nominal(R, lab)
    return dict(label=LABELS[lab], F=n["score"]["flight_score"], geometry=n["details"]["geometry"], x_cg=n["M2"]["x_cg"], x_np=n["M2"]["x_np"])


def f_overlays(R):
    out = []
    four = [_design(R, x) for x in FOUR if nominal(R, x)]
    six = [_design(R, x) for x in MAIN if nominal(R, x)]
    if four:
        planform_overlay(four, FIG / "planform_overlay.pdf")
        planform_overlay(four, FIG / "planform_overlay_motor.pdf", datum="motor")
        out += ["figures/planform_overlay.pdf", "figures/planform_overlay_motor.pdf"]
    if six:
        planform_overlay(six, FIG / "planform_overlay_all.pdf")
        planform_overlay(six, FIG / "planform_overlay_all_motor.pdf", datum="motor")
        out += ["figures/planform_overlay_all.pdf", "figures/planform_overlay_all_motor.pdf"]
    return out


def f_method(R):
    labs = [x for x in MAIN if nominal(R, x)]
    fig, ax = figure(1.0, 0.55)
    y = np.arange(len(labs))[::-1]
    for yi, lab in zip(y, labs):
        n = nominal(R, lab)
        st = P.stat(R, lab)
        if st:
            ax.plot([st["p5"], st["p95"]], [yi, yi], color="0.6", lw=3, solid_capstyle="butt", label="MC 5th to 95th percentile" if yi == y[0] else None)
            ax.plot(st["mu"], yi, "D", mfc="white", mec="k", ms=6, label="MC mean" if yi == y[0] else None)
        ax.plot(n["score"]["flight_score"], yi, "o", color=PALETTE[0], ms=6, label="F nominal, capped" if yi == y[0] else None)
        ax.plot(n["floored"]["flight_score"], yi, "s", mfc="none", mec=PALETTE[1], ms=6, label="F floored" if yi == y[0] else None)
    ax.set_yticks(y, [LABELS[x] for x in labs])
    ax.axvline(6.0, color="k", lw=0.6, ls=":")
    ax.text(6.0, y[0] + 0.45, "cap", ha="center", fontsize=7)
    ax.set_xlabel("flight score F = M1 + M2 + M3 (AirCrafter 2.0's model)")
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.17), ncol=4, frameon=False, fontsize=7)
    ax.grid(axis="y", visible=False)
    return _save(fig, "method_comparison")


def f_validation(R):
    V = R.get("validation")
    if not V or "loo" not in V.get("variants", {}):
        return []
    sys.path.insert(0, str(ROOT / "studies"))
    from validation_2026 import LB, TABLE36, TABLE42, humpback_config
    from validation_2026_report import _model_rows

    mcfg = humpback_config()
    rows = {m: _model_rows(V["variants"][m]["cases"]["as_built_full_throttle"], mcfg) for m in ("gg", "loo") if m in V["variants"]}
    keys = ["wing", "HT", "VT", "fuselage", "M2", "M3"]
    fig, ax = figure(1.0, 0.45)
    x = np.arange(len(keys))
    w = 0.27
    ax.bar(x - w, [TABLE36[k] for k in keys], w, color="0.75", edgecolor="k", lw=0.4, label="[HB] Table 36")
    if "gg" in rows:
        ax.bar(x, [rows["gg"][k] / LB for k in keys], w, color=PALETTE[3], hatch="///", edgecolor="w", lw=0, label="model, G&G factors")
    ax.bar(x + w, [rows["loo"][k] / LB for k in keys], w, color=PALETTE[0], hatch="xxx", edgecolor="w", lw=0, label="model, USYD factors without Humpback")
    ax.set_xticks(x, ["wing", "HT", "VT", "'fuselage' row", "M2 take-off", "M3 take-off"])
    ax.set_ylabel("mass (lb)")
    ax.legend(frameon=False, fontsize=7)
    ax.grid(axis="x", visible=False)
    out = [_save(fig, "humpback_validation_mass")]
    cases = V["variants"]["loo"]["cases"]
    names = [k for k in TABLE42 if k in cases and cases[k].get("snapshot")]
    errs = [100 * (cases[k]["snapshot"][TABLE42[k]["point"]]["otherLapTime"] / TABLE42[k]["lap_s"] - 1) for k in names]
    fig, ax = figure(0.8, 0.5)
    y = np.arange(len(names))[::-1]
    ax.barh(y, errs, color=PALETTE[0], height=0.55)
    ax.axvspan(-10, 10, color="0.85", zorder=0, label="±10 %, the §3.7 criterion")
    ax.axvline(0, color="k", lw=0.6)
    for yi, e in zip(y, errs):
        ax.text(e - 1, yi, f"{e:+.0f} %", va="center", ha="right", fontsize=7)
    ax.set_yticks(y, [f"{k.replace('_test', ' test ')} ({TABLE42[k].get('banner_in', '36/12')}{' in' if 'banner_in' in TABLE42[k] else ''})" for k in names])
    ax.set_xlabel("lap time, model against [HB] Table 42 (%)")
    ax.set_xlim(min(errs) - 12, 15)
    ax.legend(frameon=False, fontsize=7, loc="lower left")
    ax.grid(axis="y", visible=False)
    out.append(_save(fig, "humpback_validation_flight"))
    return out


def f_parity():
    d = json.loads((ROOT / "docs" / "studies" / "mass_calibration_usyd.json").read_text())
    fits = d["fits"]["areal"]
    fig, ax = figure(0.75, 1.0)
    names = {"OH": "[OH]", "W": "[W]", "HB": "[HB]"}
    lim = [0.03, 3.0]
    x = np.array(lim)
    ax.fill_between(x, x * 0.7, x * 1.3, color="0.88", zorder=0, label="±30 %")
    ax.plot(x, x, color="k", lw=0.7)
    rows = []
    for i, (ac, nm) in enumerate(names.items()):
        for mode, errs, mfc in (("in sample", fits["combined"]["errors"][ac], PALETTE[i]), ("leave-one-out", fits["loo"][ac]["errors"], "none")):
            xs = [d["real"][ac][k] for k in errs]
            ys = [d["real"][ac][k] * (1 + e) for k, e in errs.items()]
            ax.plot(xs, ys, MARK[i], mfc=mfc, mec=PALETTE[i], ms=5, ls="none", label=f"{nm} {mode}")
            rows += [[nm, k, mode, d["real"][ac][k], d["real"][ac][k] * (1 + e), e] for k, e in errs.items()]
    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.set_xlim(lim)
    ax.set_ylim(lim)
    ax.set_aspect("equal")
    ax.set_xlabel("component mass in the report (kg)")
    ax.set_ylabel("predicted component mass (kg)")
    ax.legend(frameon=False, fontsize=7, loc="upper left")
    out = _save(fig, "mass_parity")
    TAB.mkdir(parents=True, exist_ok=True)
    write_table(P.T("mass_parity", "The calibration's parity data: predicted against real component masses; docs/studies/mass_calibration_usyd.json, not refitted.", ["aircraft", "component", "fit", "real (kg)", "predicted (kg)", "error"],
                    [[a, b, c, f"{r:.3f}", f"{p:.3f}", f"{100 * e:+.1f} %"] for a, b, c, r, p, e in rows]))
    return out


def f_mass(R):
    labs = [x for x in MAIN if nominal(R, x)]
    br = {x: P.mass_breakdown(R, x) for x in labs}
    fig, ax = figure(1.0, 0.62)
    x = np.arange(len(labs))
    bottom = np.zeros(len(labs))
    for i, (name, _) in enumerate(P.MASS_ORDER):
        vals = np.array([br[lab][i][1] for lab in labs])
        ax.bar(x, vals, 0.62, bottom=bottom, color=(PALETTE * 2)[i], hatch=HATCH[i], edgecolor="w", lw=0.5, label=name)
        bottom += vals
    ax.set_xticks(x, [SHORT[lab] for lab in labs])
    ax.set_ylabel("M2 take-off mass (kg)")
    ax.legend(loc="center left", bbox_to_anchor=(1.0, 0.5), frameon=False, fontsize=7, reverse=True)
    ax.grid(axis="x", visible=False)
    return _save(fig, "mass_breakdown")


def f_score(R):
    labs = [x for x in MAIN if nominal(R, x)]
    fig, ax = figure(1.0, 0.5)
    x = np.arange(len(labs))
    w = 0.36
    s = [P.score_of(nominal(R, lab)) for lab in labs]
    ax.bar(x - w / 2, [d["M2"] - 1 for d in s], w, color=PALETTE[0], label="M2 - 1 (capped)")
    ax.bar(x + w / 2, [d["M3"] - 2 for d in s], w, color=PALETTE[2], hatch="///", edgecolor="w", lw=0, label="M3 - 2 (capped)")
    ax.plot(x - w / 2, [d["r_M2"] for d in s], "_", color="k", ms=14, mew=1.4, label="uncapped ratio")
    ax.plot(x + w / 2, [d["r_M3"] for d in s], "_", color="k", ms=14, mew=1.4)
    ax.axhline(1.0, color="k", lw=0.8, ls="--", label="the cap (ratio = 1, M1 = 1 for all)")
    ax.set_xticks(x, [SHORT[lab] for lab in labs])
    ax.set_ylabel("mission ratio (score above its floor)")
    ax.set_ylim(0, 1.15)
    ax.legend(frameon=False, fontsize=7, ncol=2, loc="upper center", bbox_to_anchor=(0.5, -0.1))
    ax.grid(axis="x", visible=False)
    return _save(fig, "score_split")


def f_drag(R):
    labs = [x for x in MAIN if nominal(R, x)]
    fig, axs = figure(1.0, 0.5, ncols=2)
    for ax, pt in zip(axs, ("M2", "M3")):
        x = np.arange(len(labs))
        bottom = np.zeros(len(labs))
        for i, (name, _) in enumerate(P.DRAG_PARTS):
            if pt == "M2" and name.startswith("banner"):
                continue
            vals = np.array([P.drag_parts(R, lab, pt)[i][1] for lab in labs])
            ax.bar(x, vals, 0.65, bottom=bottom, color=PALETTE[i], hatch=HATCH[i], edgecolor="w", lw=0.5, label=name)
            bottom += vals
        ax.set_xticks(x, [SHORT[lab] for lab in labs], rotation=40, ha="right", fontsize=7)
        ax.set_title(f"{pt}{' (banner deployed)' if pt == 'M3' else ''}")
        ax.grid(axis="x", visible=False)
    axs[0].set_ylabel("drag coefficient at cruise, on the wing area")
    axs[1].legend(frameon=False, fontsize=7, loc="upper center", bbox_to_anchor=(-0.1, -0.28), ncol=4)
    return _save(fig, "drag_breakdown")


def f_enum(R):
    runs = [d for d in R["enumeration"]]
    if not runs:
        return []
    runs = sorted(runs, key=lambda d: (d.get("F") or 0.0))
    fig, ax = figure(1.0, 0.62)
    y = np.arange(len(runs))
    for yi, d in zip(y, runs):
        if d.get("F") is None:
            ax.text(0.02, yi, "analysis failed at the start design: no optimisation", transform=ax.get_yaxis_transform(), fontsize=6.5, va="center")
            continue
        conv = d.get("ok")
        ax.plot(d["F"], yi, "o" if conv else "x", color=PALETTE[0] if d["cells"] == 8 else PALETTE[1], ms=6)
    good = [d["F"] for d in runs if d.get("F") is not None]
    ax.set_xlim(min(good) - 0.01, max(good) + 0.01 + 0.45 * (max(good) - min(good) + 0.01))
    for yi, d in zip(y, runs):
        if d.get("F") is not None:
            act = d.get("active_constraints") or []
            names = [("M3 at 100 Wh" if "M3.energy_margin_Wh" in c else "M2 at 100 Wh" if "M2.energy_margin_Wh" in c else "3 passengers per cargo" if "pax_margin" in c else c.replace("points.", "")) for c in act if "V_h" not in c and "V_v" not in c]
            ax.text(max(good) + 0.012, yi, ", ".join(names), fontsize=5.5, va="center")
    ax.set_ylim(-0.7, len(runs) - 0.3)
    ax.set_yticks(y, [f"{d['cells']:g}S {d['props']['M2']} / {d['props']['M3']}" + (", Kv 270 start" if "kv270" in d["tag"] else "") if d.get("props") else d["tag"].replace("fr_", "").replace("_", " ").replace("6S 19x12E 22x12E", "6S 19x12E / 22x12E, Kv 245 start") for d in runs], fontsize=7)
    ax.set_xlabel("F, capped, continuous counts (text: active constraints; every case also has V_h, V_v at their minimums)")
    ax.xaxis.label.set_size(7.5)
    ax.grid(axis="y", visible=False)
    return _save(fig, "propulsion_enumeration")


def _mc_col(tag, j=0):
    f = P.S.MC_DIR / f"{tag}.npz"
    if not f.exists():
        return None
    v = np.load(f)["values"][:, j]
    return np.sort(v[~np.isnan(v)])


def f_cdf(R):
    out = []
    for which, name, labs in (("in", "mc_cdf_in_set", MAIN), ("out", "mc_cdf_out_of_set", ["humpback", "mdo", "mdo_wind", "rdo"])):
        fig, ax = figure(1.0, 0.55)
        any_ = False
        for lab in labs:
            col = _mc_col(f"fr_mc_{which}_{lab}")
            if col is None:
                continue
            ax.plot(col, np.arange(1, len(col) + 1) / len(col), color="k" if lab == "rdo" else COLOR[lab], ls=(0, (8, 2)) if lab == "rdo" else STYLE[lab], lw=1.6 if lab == "rdo" else None, label=SHORT[lab])
            any_ = True
        if not any_:
            import matplotlib.pyplot as plt

            plt.close(fig)
            continue
        ax.axvline(6.0, color="k", lw=0.6, ls=":")
        ax.set_xlabel(f"flight score F ({'five active parameters' if which == 'in' else 'the 17 parameters of the extended list'}, 1024 Sobol' points, converged)")
        ax.set_ylabel("cumulative probability")
        ax.legend(frameon=False, fontsize=8, loc="upper left")
        out.append(_save(fig, name))
    return out


def f_pv(R):
    out = []
    for key, name in (("mc_in", "p_violation_in_set"), ("mc_out", "p_violation_out_of_set")):
        labs = [x for x in MAIN if R["designs"][x].get(key)]
        if not labs:
            continue
        cons = sorted({k for x in labs for k, p in R["designs"][x][key]["p_violation"].items() if p > 0}, key=lambda k: -max(R["designs"][x][key]["p_violation"].get(k, 0) for x in labs))[:10]
        rows = ["any constraint"] + cons
        fig, ax = figure(1.0, 0.15 + 0.095 * len(rows) * max(1, len(labs) / 3))
        h = 0.8 / len(labs)
        y = np.arange(len(rows))[::-1]
        for i, x in enumerate(labs):
            m = R["designs"][x][key]
            vals = [m["p_any"]] + [m["p_violation"].get(k, 0.0) for k in cons]
            ax.barh(y + 0.4 - (i + 0.5) * h, vals, h, color=COLOR[x], hatch=HATCH[i], edgecolor="w", lw=0, label=SHORT[x])
        ax.set_yticks(y, [r.replace("points.", "") for r in rows], fontsize=7)
        ax.set_xlabel("P(violation), converged samples (analysis failures counted apart)")
        top = max(R["designs"][x][key]["p_any"] for x in labs)
        ax.set_xlim(0, top * 1.15)
        zero = [SHORT[x] for x in labs if R["designs"][x][key]["p_any"] == 0]
        ax.legend(frameon=False, fontsize=7, loc="lower right", title=f"no violation: {', '.join(zero)}" if zero else None, title_fontsize=6.5)
        ax.grid(axis="y", visible=False)
        out.append(_save(fig, name))
    return out


def f_decomp(R):
    d = R.get("decomposition") or P.decomposition(R)
    if not d:
        return []
    fig, axs = figure(1.0, 0.42, ncols=3, gridspec_kw=dict(wspace=0.55))
    groups = [("F nominal and MC mean", [("nominal", "F nominal"), ("mu", "MC mean")]), ("MC sigma of F", [("sigma", "")]), ("P(any violation)", [("p_any", "")])]
    for ax, (title, items) in zip(axs, groups):
        x = np.arange(len(items))
        w = 0.36
        ax.bar(x - w / 2, [d[k]["wind"] for k, _ in items], w, color=PALETTE[3], hatch="///", edgecolor="w", lw=0, label="designing at the mean wind")
        ax.bar(x + w / 2, [d[k]["robust"] for k, _ in items], w, color=PALETTE[5], hatch="xxx", edgecolor="w", lw=0, label="robustness itself")
        ax.axhline(0, color="k", lw=0.6)
        ax.set_xticks(x, [n for _, n in items], fontsize=7)
        ax.set_title(title, fontsize=8)
        ax.grid(axis="x", visible=False)
    axs[0].set_ylabel("change")
    axs[1].legend(frameon=False, fontsize=7, loc="upper center", bbox_to_anchor=(0.5, -0.12), ncol=2)
    return _save(fig, "robustness_decomposition")


def f_multistart(R):
    ms = R["multistart"]
    if not ms:
        return []
    fig, axs = figure(1.0, 0.45, ncols=2, gridspec_kw=dict(width_ratios=[1, 1.5], wspace=0.28))
    ax = axs[0]
    good = [d.get("F") for d in ms if d.get("F") is not None]
    lo = min(good) - 0.002 if good else 0
    for i, d in enumerate(ms):
        if d.get("F") is None:
            ax.plot(i, lo, "x", color=PALETTE[1], ms=7, label="analysis failed" if i == next(j for j, q in enumerate(ms) if q.get("F") is None) else None)
        else:
            ax.plot(i, d["F"], "o" if d.get("ok") else "s", color=PALETTE[0], mfc=PALETTE[0] if d.get("ok") else "none", ms=5)
    base = next((d for d in R["enumeration"] if d["tag"] == R["best_case"]), None)
    if base:
        ax.axhline(base["F"], color="k", lw=0.7, ls="--", label="default start")
    ax.set_xlabel("start")
    ax.set_ylabel("F")
    ax.legend(frameon=False, fontsize=7)
    ax = axs[1]
    box = P.S.config().all_dvs()
    names = list(box)
    conv = [d for d in ms if d.get("ok") and d.get("dv")]
    for d in conv:
        ys = [(d["dv"][k] - box[k].lower) / (box[k].upper - box[k].lower) for k in names]
        ax.plot(range(len(names)), ys, color=PALETTE[0], lw=0.8, alpha=0.7)
    ax.set_xticks(range(len(names)), names, rotation=70, ha="right", fontsize=6)
    ax.set_ylabel("design variable, fraction of its box")
    return _save(fig, "multistart_spread")


def f_sens(R):
    rows = []
    for x in P.bound_lines(R):
        if "missing" not in x:
            rows.append((f"bound {x['key']}", x["dF"]))
    mdo = P.rd(R["mdo_tag"]) if R["mdo_tag"] else None
    F0 = mdo["score"]["flight_score"] if mdo else None
    for k, nm in (("old_constraints", "old constraint set"), ("takeoff_off", "take-off off")):
        d = R["others"].get(k)
        if d and d.get("F") and F0:
            rows.append((nm, d["F"] - F0))
    re_ = (R["designs"]["mdo"] or {}).get("reanalysis")
    if re_ and F0:
        for name, c in re_["cases"].items():
            if c.get("F") and "throttle" not in name and "rpm" not in name:
                rows.append((f"C_D,b {c['cd_banner']:g}", c["F"] - F0))
    base = next((d for d in R["enumeration"] if d["tag"] == R["best_case"]), None)
    for k, d in R["normalisation"].items():
        if d and d.get("F") and base:
            rows.append((f"{k.replace('NI', 'NI_max x').replace('B', 'B_max x').replace('NI_max x_max', 'NI_max')}", d["F"] - base["F"]))
    if not rows:
        return []
    failed = [f"C_D,b {c['cd_banner']:g}" for name, c in (re_["cases"].items() if re_ else []) if not c.get("F") and "throttle" not in name and "rpm" not in name]
    rows = sorted(rows, key=lambda r: abs(r[1]))
    fig, ax = figure(1.0, 0.1 + 0.05 * len(rows))
    y = np.arange(len(rows))
    ax.barh(y, [r[1] for r in rows], color=[PALETTE[0] if r[1] >= 0 else PALETTE[1] for r in rows], height=0.6)
    ax.axvline(0, color="k", lw=0.6)
    for yi, r in zip(y, rows):
        ax.text(r[1], yi, f" {r[1]:+.3f} ", va="center", ha="left" if r[1] >= 0 else "right", fontsize=6.5)
    ax.set_yticks(y, [r[0] for r in rows], fontsize=7)
    ax.margins(x=0.15)
    ax.set_xlabel("ΔF against MDO (re-optimised, or re-analysed for C_D,b)" + (f"; no result at {', '.join(failed)}: lap-model limit" if failed else ""))
    ax.grid(axis="y", visible=False)
    return _save(fig, "sensitivities")


def f_ranking(R):
    r = P.ranking(R)[:18]
    if not r:
        return []
    r = r[::-1]
    fig, ax = figure(1.0, 0.75)
    y = np.arange(len(r))
    ax.barh(y, [x["effect"] for x in r], color=PALETTE[0], height=0.6)
    ax.set_yticks(y, [(" ".join(x["name"].split()[:2]) + ", tail volumes" if x["name"].startswith("bound (i") and "V_h" in x["name"] else " ".join(x["name"].split()[:2]) + ", chord and camber" if x["name"].startswith("bound (i") else x["name"]) for x in r], fontsize=6.5)
    ax.set_xlabel("effect on F (measure by row: see the ranking table)")
    ax.grid(axis="y", visible=False)
    return _save(fig, "placeholder_ranking")


def f_wind(R):
    out = []
    labs = [x for x in ("mdo", "rdo") if (R["designs"][x].get("wind_env") or {}).get("rows")]
    if labs:
        fig, ax = figure(0.8, 0.55)
        for x in labs:
            rows = R["designs"][x]["wind_env"]["rows"]
            ax.plot([r["theta"] for r in rows], [r["limit"] if r["limit"] is not None else np.nan for r in rows], color=COLOR[x], ls=STYLE[x], marker=MARK[labs.index(x)], ms=4, label=SHORT[x])
        ax.axhline(10.0, color="k", lw=0.6, ls=":", label="top of the uncertain range (10 m/s)")
        ax.set_xlabel("wind direction from the course axis (deg)")
        ax.set_ylabel("wind limit (m/s)")
        ax.legend(frameon=False, fontsize=7)
        out.append(_save(fig, "wind_envelope"))
    labs = [x for x in MAIN if R["designs"][x].get("wind_dir")]
    if labs:
        fig, ax = figure(1.0, 0.5)
        for i, x in enumerate(labs):
            w = R["designs"][x]["wind_dir"]
            ax.plot(w["directions"], [row[0] for row in w["values"]], color=COLOR[x], ls=STYLE[x], marker=MARK[i], ms=3.5, label=SHORT[x])
        ax.set_xlabel("wind direction from the course axis (deg), at the mean speed 5 m/s")
        ax.set_ylabel("F")
        ax.legend(frameon=False, fontsize=7, ncol=3, loc="upper center", bbox_to_anchor=(0.5, -0.17))
        out.append(_save(fig, "f_against_direction"))
    return out


def f_compute():
    f = DOCS / "compute_performance_2026.json"
    if not f.exists():
        return []
    C = json.loads(f.read_text())
    out = []
    rows = C.get("main_rows") or []
    if rows:
        fig, ax = figure(1.0, 0.45)
        rows = rows[::-1]
        y = np.arange(len(rows))
        ax.barh(y, [r["mean"] for r in rows], color=PALETTE[0], height=0.6)
        ax.set_xscale("log")
        for yi, r in zip(y, rows):
            ax.text(r["mean"] * 1.15, yi, r["text"], va="center", fontsize=7)
        ax.set_yticks(y, [r["name"] for r in rows], fontsize=7)
        ax.margins(x=0.25)
        ax.set_xlabel("wall time, single-threaded (s, log scale)")
        ax.grid(axis="y", visible=False)
        out.append(_save(fig, "compute_main"))
    sc = C.get("scaling") or {}
    if sc.get("grid_vs_gradient"):
        fig, axs = figure(1.0, 0.5, ncols=2, gridspec_kw=dict(wspace=0.3))
        ax = axs[0]
        g = sc["grid_vs_gradient"]
        ax.semilogy([r["n"] for r in g["grid"]], [r["s"] for r in g["grid"]], color=PALETTE[1], ls="--", marker="s", ms=4, label="grid, AC1's cost per call (extrapolated)")
        ax.semilogy([r["n"] for r in g["gradient"]], [r["s"] for r in g["gradient"]], color=PALETTE[0], marker="o", ms=4, label="AC2 SLSQP (measured)")
        bad = [r for r in g["gradient"] if not r["ok"]]
        if bad:
            ax.plot([r["n"] for r in bad], [r["s"] for r in bad], ls="", marker="o", ms=4, mfc="w", color=PALETTE[0], label="AC2, not converged")
        ax.set_xlabel("design variables")
        ax.set_ylabel("wall time (s, log scale)")
        ax.legend(frameon=False, fontsize=6, loc="upper center", bbox_to_anchor=(0.5, -0.22), ncol=1)
        ax = axs[1]
        r = sc.get("rdo_vs_d") or []
        ds = [x["d"] for x in r]
        ax.semilogy(ds, [x["udr_s"] for x in r], color=PALETTE[0], marker="o", ms=4, label="UDR (measured)")
        ax.semilogy(ds, [x["smolyak_s"] for x in r], color=PALETTE[2], ls="--", marker="^", ms=4, label="Smolyak 2 (scaled by nodes)")
        ax.semilogy(ds, [x["mc_s"] for x in r], color=PALETTE[1], ls=":", marker="s", ms=4, label="1024-point MC (scaled by nodes)")
        ax.set_xlabel("uncertain parameters d")
        ax.set_ylabel("one robust evaluation (s, log scale)")
        ax.legend(frameon=False, fontsize=6, loc="upper center", bbox_to_anchor=(0.5, -0.22), ncol=1)
        out.append(_save(fig, "compute_scaling"))
    return out


# -------------------------------------------------------------------------------------------------------------- tables
TEX = {"%": r"\%", "_": r"\_", "&": r"\&", "#": r"\#", "$": r"\$", "μ": r"$\mu$", "σ": r"$\sigma$", "Δ": r"$\Delta$", "²": r"$^2$", "±": r"$\pm$", "×": r"$\times$", "≥": r"$\geq$", "≤": r"$\leq$", "–": "--", "~": r"\textasciitilde{}", "<=": r"$\leq$", ">=": r"$\geq$"}


def tex_escape(x):
    s = str(x).replace("**", "")
    for a in ("<=", ">="):
        s = s.replace(a, TEX[a])
    return "".join(TEX.get(c, c) for c in s) if "$" not in s else _esc_keep_math(s)


def _esc_keep_math(s):
    out, inside = [], False
    for c in s:
        if c == "$":
            inside = not inside
            out.append(c)
        else:
            out.append(c if inside else TEX.get(c, c))
    return "".join(out)


def write_table(t):
    TAB.mkdir(parents=True, exist_ok=True)
    with (TAB / f"{t.name}.csv").open("w", newline="") as fh:
        wr = csv.writer(fh)
        wr.writerow(t.header)
        wr.writerows([[str(c).replace("**", "") for c in r] for r in t.rows])
    align = "l" + "r" * (len(t.header) - 1)
    lines = [f"% Generated by studies/results_2026_exhibits.py from docs/studies/results_2026/results_2026.json", r"\begin{table}[htbp]", r"\centering", r"\footnotesize",
             rf"\caption{{{tex_escape(t.caption)}}}", rf"\label{{tab:r26_{t.name}}}", r"\resizebox{\textwidth}{!}{%", rf"\begin{{tabular}}{{{align}}}", r"\toprule",
             " & ".join(tex_escape(h) for h in t.header) + r" \\", r"\midrule"]
    lines += [" & ".join(tex_escape(c) for c in r) + r" \\" for r in t.rows]
    lines += [r"\bottomrule", r"\end{tabular}}", r"\end{table}", ""]
    (TAB / f"{t.name}.tex").write_text("\n".join(lines))


def main():
    R, ts = P.report()
    files = []
    FIG.mkdir(parents=True, exist_ok=True)
    for fn in (f_overlays, f_method, f_validation, f_mass, f_score, f_drag, f_enum, f_cdf, f_pv, f_decomp, f_multistart, f_sens, f_ranking, f_wind):
        res = fn(R)
        files += res if isinstance(res, list) else [res]
    files.append(f_parity())
    files += f_compute()
    for t in ts.values():
        write_table(t)
    write_table(P.limitations(R))
    print(f"{len(files)} figures, {len(ts) + 2} tables")
    return files


if __name__ == "__main__":
    main()
