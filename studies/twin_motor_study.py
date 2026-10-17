"""Single vs twin motor study (twin-motor brief, checkpoint 3) for the AIAC paper.

Outer enumeration (D4, D8): n_motors in {1, 2} x props {10x7E, 11x7E, 12x8E}, per_motor packs, placeholder
objective ThreeLapTime. Plus:
  - matched installed power: twins with smaller props / fewer cells, so the total static thrust (or static
    electrical power) is about that of the single-motor optimum;
  - motor inertia relief on vs off, and the Raymer nacelle form factor, at the best twins;
  - OEI post-check, sensitivities, mass and drag breakdowns, timings.

Writes docs/studies/twin_motor_study.md, docs/studies/twin_motor_study_results.json and figures (PDF + PNG) in
outputs/twin_study/. Run:

    .venv/bin/python studies/twin_motor_study.py [--workers 4] [--reuse]

--reuse rebuilds the doc and figures from docs/studies/twin_motor_study_results.json without optimising.
"""

import os

os.environ.setdefault("OMP_NUM_THREADS", "4")  # several optimisations run in parallel
os.environ.setdefault("OPENBLAS_NUM_THREADS", "4")

import argparse  # noqa: E402
import json  # noqa: E402
import time  # noqa: E402
import warnings  # noqa: E402
from concurrent.futures import ProcessPoolExecutor, as_completed  # noqa: E402
from pathlib import Path  # noqa: E402

import numpy as np  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "outputs" / "twin_study"
DOC = ROOT / "docs" / "studies" / "twin_motor_study.md"
CACHE = ROOT / "docs" / "studies" / "twin_motor_study_results.json"
CFG = {1: "full_mdo.yaml", 2: "full_mdo_twin.yaml"}
PROPS = ["10x7E", "11x7E", "12x8E"]
# matched installed power, from a powertrain-only screen at the single 10x7E / 4S unit (21.4 N, 494 W static):
# static thrust within ~10 % (3S) and static electrical power within ~5 % (10x7E 3S, 8x6E 4S)
MATCHED = [("9x75E", 3), ("10x5E", 3), ("9x6E", 3), ("10x7E", 3), ("8x6E", 4)]

KEYS = ["ThreeLapTime", "b", "c", "tc_max", "camber_scale", "capacity", "l_ht", "S_ht", "S_vt", "x_batt",
        "x_cb_frac", "v", "S", "m_total", "m_empty", "m_wing", "m_tube", "t_tube", "r_tube", "wall_ratio", "Mx",
        "My", "M_fl", "T_fl", "m_sys", "m_batt", "m_motor_group", "E_nom_total", "T_static", "T_static_unit",
        "I_static", "CD_extra", "CD_nac", "CD_fus", "CD_tail", "cruise.CDp", "cruise.vlm.wing_perf.CDi",
        "cruise.CD", "cruise.L_over_D", "CD0", "SM", "V_h", "V_v", "energy_margin", "thrust_margin_static",
        "stall_margin", "turn_speed_margin", "rpm_margin", "m_ht", "m_vt", "m_tb", "m_cb", "m_gear",
        "m_avionics", "m_pay", "Lap1time"]
TWIN_KEYS = ["y_m", "fus_clear_margin", "tip_clear_margin", "prop_clear_margin"]
ASSUMPTIONS = ["m_pay", "Kv", "Rm", "R_cell", "e_spec", "m_motor", "m_avionics", "f_tb", "f_cb", "rho"]
TWIN_ASSUMPTIONS = ["m_nacelle", "Q_nac", "l_nac", "d_nac", "y_m", "c_clear"]


# --- one optimisation (runs in a worker process) ------------------------------------------------------
def run_case(case):
    warnings.filterwarnings("ignore")
    from aircrafter2.plotting import figures as F
    from aircrafter2.problems.full_mdo import build_full_problem, check_fixed_limits, load_full_config
    from aircrafter2.stability import oei_check

    cfg = load_full_config(ROOT / "configs" / CFG[case["n"]])
    cfg.propulsion.prop = case["prop"]
    cfg.propulsion.n_cells = float(case["cells"])
    cfg.mass.options.motor_inertia_relief = case.get("relief", False)
    if case.get("maxiter"):  # smoke tests
        cfg.maxiter = case["maxiter"]
    if case.get("nacelle_ff"):
        cfg.propulsion.nacelle_ff = case["nacelle_ff"]
    out = dict(case)
    prob = build_full_problem(cfg)
    prob.driver.options["disp"] = False
    t = time.perf_counter()
    try:
        res = prob.run_driver()
        out.update(success=bool(res.success), iterations=int(res.iter_count))
    except Exception as exc:  # MDA failure inside the optimiser
        out.update(success=False, iterations=-1, error=f"{exc.__class__.__name__}: {str(exc)[:120]}")
    out["time_s"] = time.perf_counter() - t

    def val(k):
        return float(np.ravel(prob.get_val(k))[0])

    for k in KEYS + (TWIN_KEYS if case["n"] == 2 else []):
        out[k] = val(k)
    out["P_static"] = float(prob.get_val("propulsion.P_nodes")[0])
    out["limits"] = check_fixed_limits(prob)
    out["feasible"] = bool(out["success"] and out["limits"]["ok"])
    out["oei"] = oei_check(prob, cfg)
    out["active"] = _active(prob)
    if out["success"]:
        wrt = list(cfg.design_variables) + ASSUMPTIONS + (TWIN_ASSUMPTIONS if case["n"] == 2 else [])
        J = prob.compute_totals(of=["ThreeLapTime"], wrt=wrt, return_format="dict")["ThreeLapTime"]
        f = out["ThreeLapTime"]
        out["dT_dx"] = {w: float(np.ravel(J[w])[0]) for w in wrt}
        out["sens"] = {w: out["dT_dx"][w] * val(w) / f for w in wrt}
        (OUT / "planforms").mkdir(parents=True, exist_ok=True)
        F.planform(prob, OUT / "planforms" / f"planform_{case['id']}.pdf")
    return out


def _active(prob, tol=1e-4):
    act = []
    cons = prob.driver.get_constraint_values(driver_scaling=True)
    for name, meta in prob.driver._cons.items():
        x = np.ravel(cons[name])
        for bound in ("lower", "upper"):
            b = meta[bound]
            if b is not None and np.all(np.abs(np.ravel(b)) < 1e20) and np.min(np.abs(x - np.ravel(b))) < tol:
                act.append(f"{meta['source'].split('.')[-1]} ({bound})")
    dvs = prob.driver.get_design_var_values(driver_scaling=True)
    for name, meta in prob.driver._designvars.items():
        for bound in ("lower", "upper"):
            if np.min(np.abs(np.ravel(dvs[name]) - np.ravel(meta[bound]))) < 1e-6:
                act.append(f"DV {name} = {bound}")
    return act


def timing_twin_analysis():
    warnings.filterwarnings("ignore")
    from aircrafter2.problems.full_mdo import build_full_problem, load_full_config

    cfg = load_full_config(ROOT / "configs" / CFG[2])
    t0 = time.perf_counter()
    prob = build_full_problem(cfg, driver=False)
    t1 = time.perf_counter()
    prob.run_model()
    t2 = time.perf_counter()
    prob.run_model()  # warm (outputs from the first solve)
    t3 = time.perf_counter()
    return {"build_s": t1 - t0, "analysis_cold_s": t2 - t1, "analysis_warm_s": t3 - t2}


def _run_all(cases, workers):
    results = {}
    with ProcessPoolExecutor(max_workers=workers) as ex:
        futs = {ex.submit(run_case, c): c["id"] for c in cases}
        for fut in as_completed(futs):
            r = fut.result()
            results[r["id"]] = r
            print(f"{r['id']:28s} success={r['success']} feasible={r['feasible']} its={r['iterations']} "
                  f"t={r['time_s']:.0f}s 3-lap={r['ThreeLapTime']:.2f}", flush=True)
    return results


def run(workers):
    cases = [dict(id=f"n{n}_{p}_4S", n=n, prop=p, cells=4, group="enum") for n in (1, 2) for p in PROPS]
    cases += [dict(id=f"n2_{p}_{c}S_matched", n=2, prop=p, cells=c, group="matched") for p, c in MATCHED]
    results = _run_all(cases, workers)
    best2 = _best(results, "enum", 2)
    bestm = _best(results, "matched", 2)
    extra = []
    for r in (best2, bestm):
        if r is None:
            continue
        base = {k: r[k] for k in ("n", "prop", "cells")}
        extra += [dict(id=r["id"] + "_relief", relief=True, group="relief", of=r["id"], **base),
                  dict(id=r["id"] + "_ffnacelle", nacelle_ff="nacelle", group="ff", of=r["id"], **base)]
    results.update(_run_all(extra, workers))
    results["_timing"] = timing_twin_analysis()
    CACHE.write_text(json.dumps(results, indent=1, default=str))
    return results


def _why(r):
    """Short reason a case is infeasible."""
    if r["success"] and not r["limits"]["ok"]:
        return "infeasible (current)" if r["limits"]["I_margin_A"] < 0 else "infeasible (RPM)"
    if r["limits"]["rpm_margin"] < -1.0:
        return "not converged (RPM)"
    return "not converged"


def _best(results, group, n):
    c = [r for k, r in results.items() if not k.startswith("_") and r["group"] == group and r["n"] == n
         and r["feasible"]]
    return min(c, key=lambda r: r["ThreeLapTime"]) if c else None


# --- figures ---------------------------------------------------------------------------------------------
SERIES = {"single": ("1 motor (nose)", 0, ""), "twin": ("2 motors (wing)", 1, "///"),
          "matched": ("2 motors, matched thrust or power", 2, "...")}


def make_figures(R):
    from aircrafter2.plotting.style import PALETTE, figure, save

    OUT.mkdir(parents=True, exist_ok=True)
    files = []
    col = {k: PALETTE[i] for k, (_, i, _) in SERIES.items()}
    hatch = {k: h for k, (_, _, h) in SERIES.items()}
    lab = {k: name for k, (name, _, _) in SERIES.items()}

    # 1. optimum 3-lap time per prop and layout (dot plot: no zero baseline needed; hollow = infeasible)
    fig, ax = figure(0.75, 0.55)
    mk = {1: "o", 2: "s"}
    for j, n in enumerate((1, 2)):
        key = "single" if n == 1 else "twin"
        for i, p in enumerate(PROPS):
            r = R[f"n{n}_{p}_4S"]
            x = i + (j - 0.5) * 0.3
            face = col[key] if r["feasible"] else "white"
            ax.plot(x, r["ThreeLapTime"], mk[n], ms=8, mfc=face, mec=col[key], mew=1.5,
                    label=lab[key] if i == 0 else None)
            txt = f"{r['ThreeLapTime']:.1f}" if r["feasible"] else _why(r)
            ax.annotate(txt, (x, r["ThreeLapTime"]), textcoords="offset points",
                        xytext=(0, 8) if r["feasible"] else (0, -15), ha="center", fontsize=7, color="0.25")
    ax.plot([], [], "o", ms=8, mfc="white", mec="0.4", mew=1.5, label="infeasible (hollow)")
    ax.set_xticks(range(len(PROPS)), PROPS)
    ax.set_xlim(-0.6, len(PROPS) - 0.4)
    y0, y1 = ax.get_ylim()
    ax.set_ylim(y0 - 0.8, y1 + 0.5)
    ax.set_xlabel("APC propeller (4S, placeholder motor)")
    ax.set_ylabel("optimum 3-lap time (s)")
    ax.legend(loc="upper left", frameon=False)
    files.append(save(fig, OUT / "enumeration_lap_time.pdf"))

    # 2. lap time vs installed static thrust
    fig, ax = figure(0.75, 0.6)
    pts = {"single": [], "twin": [], "matched": []}
    for k, r in R.items():
        if k.startswith("_") or r["group"] not in ("enum", "matched") or not r["feasible"]:
            continue
        key = "single" if r["n"] == 1 else ("twin" if r["group"] == "enum" else "matched")
        pts[key].append((r["T_static"], r["ThreeLapTime"], f"{r['prop']} {r['cells']}S"))
    markers = {"single": "o", "twin": "s", "matched": "D"}
    for key, xs in pts.items():
        if not xs:
            continue
        a = np.array([(x, y) for x, y, _ in xs])
        ax.plot(a[:, 0], a[:, 1], markers[key], ms=6, color=col[key], label=lab[key], mec="white", mew=1.0)
        for x, y, t in xs:
            ax.annotate(t, (x, y), textcoords="offset points", xytext=(5, 3), fontsize=6.5, color="0.25")
    ax.set_xlabel("installed static thrust, all motors (N)")
    ax.set_ylabel("optimum 3-lap time (s)")
    ax.legend(frameon=False, loc="upper right")
    files.append(save(fig, OUT / "lap_time_vs_static_thrust.pdf"))

    trio = {"single": _best(R, "enum", 1), "twin": _best(R, "enum", 2), "matched": _best(R, "matched", 2)}
    trio = {k: v for k, v in trio.items() if v is not None}

    # 3. mass breakdown, side by side
    groups = [("wing", lambda r: r["m_wing"]), ("tails", lambda r: r["m_ht"] + r["m_vt"]),
              ("tail boom", lambda r: r["m_tb"]), ("cargo bay", lambda r: r["m_cb"]),
              ("landing gear", lambda r: r["m_gear"]), ("motors, ESCs, props, nacelles", lambda r: r["m_motor_group"]),
              ("battery packs", lambda r: r["m_batt"]), ("avionics", lambda r: r["m_avionics"])]
    files.append(_grouped_barh(trio, groups, "mass (g)", 1000.0, "mass_breakdown_compare.pdf", col, hatch, lab,
                               figure, save, total_fmt="empty mass {:.0f} g"))
    # 4. drag area breakdown at each design's cruise point (D/q = C_D S, so different wings compare directly)
    dgroups = [("induced (VLM)", lambda r: r["cruise.vlm.wing_perf.CDi"] * r["S"]),
               ("wing profile", lambda r: r["cruise.CDp"] * r["S"]), ("tails", lambda r: r["CD_tail"] * r["S"]),
               ("fuselage", lambda r: r["CD_fus"] * r["S"]), ("nacelles", lambda r: r["CD_nac"] * r["S"])]
    files.append(_grouped_barh(trio, dgroups, r"drag area $C_D S$ at cruise (cm$^2$)", 1e4,
                               "drag_breakdown_compare.pdf", col, hatch, lab, figure, save,
                               total_fmt="total {:.0f} cm²"))

    # 5. sensitivities, best single vs best twin
    a, b = trio.get("single"), trio.get("twin")
    if a and b and "sens" in a and "sens" in b:
        names = sorted(set(a["sens"]) | set(b["sens"]),
                       key=lambda k: -max(abs(a["sens"].get(k, 0.0)), abs(b["sens"].get(k, 0.0))))[:14]
        fig, ax = figure(0.8, 0.75)
        y = np.arange(len(names))
        for j, (key, r) in enumerate((("single", a), ("twin", b))):
            vals = [r["sens"].get(k, np.nan) for k in names]
            ax.barh(y + (j - 0.5) * 0.38, vals, 0.36, color=col[key], hatch=hatch[key], edgecolor="white",
                    linewidth=0, label=lab[key])
        ax.set_yticks(y, names)
        ax.invert_yaxis()
        ax.axvline(0, color="0.3", lw=0.6)
        ax.set_xlabel(r"normalised sensitivity $(x/t)\,\partial t/\partial x$ of the 3-lap time")
        ax.legend(frameon=False, loc="lower right")
        ax.grid(axis="y", visible=False)
        files.append(save(fig, OUT / "sensitivities_compare.pdf"))
    return files


def _grouped_barh(trio, groups, xlabel, scale, name, col, hatch, lab, figure, save, total_fmt="total {:.0f}"):
    fig, ax = figure(0.85, 0.62)
    keys = list(trio)
    h = 0.8 / len(keys)
    y = np.arange(len(groups))
    for j, key in enumerate(keys):
        vals = np.array([f(trio[key]) for _, f in groups]) * scale
        yy = y - 0.4 + h * (j + 0.5)
        ax.barh(yy, vals, h * 0.9, color=col[key], hatch=hatch[key], edgecolor="white", linewidth=0,
                label=f"{lab[key]}: {trio[key]['prop']} {trio[key]['cells']}S ({total_fmt.format(vals.sum())})")
        for v_, yv in zip(vals, yy):
            if v_ > 0:
                ax.text(v_, yv, f" {v_:.0f}", va="center", fontsize=6.5, color="0.25")
    ax.set_yticks(y, [g for g, _ in groups])
    ax.invert_yaxis()
    ax.set_xlabel(xlabel)
    ax.grid(axis="y", visible=False)
    ax.set_xlim(0, ax.get_xlim()[1] * 1.08)
    ax.legend(frameon=False, loc="upper center", bbox_to_anchor=(0.5, -0.16), fontsize=7, ncol=1)
    return save(fig, OUT / name)


# --- report -----------------------------------------------------------------------------------------------
def fmt(x, nd=4):
    return f"{x:.{nd}g}" if isinstance(x, (int, float)) else str(x)


def write_doc(R):
    L = []
    w = L.append
    t = R["_timing"]
    enum = {k: R[k] for k in [f"n{n}_{p}_4S" for n in (1, 2) for p in PROPS]}
    b1, b2, bm = _best(R, "enum", 1), _best(R, "enum", 2), _best(R, "matched", 2)
    w("# Single vs twin motors: AirCrafter 2.0 study (twin-motor brief, checkpoint 3)\n")
    w("Generated by `studies/twin_motor_study.py`. Raw results are in `docs/studies/twin_motor_study_results.json`.")
    w("Figures (PDF + PNG) are in `outputs/twin_study/`, which is regenerated by the script and is not in git.\n")
    w("**Read this first.** Every motor, battery, ESC and prop mass and every rudder value is a PLACEHOLDER "
      "(OPEN_QUESTIONS Q7, Q8, Q23, Q25, Q29). Both layouts use the **same placeholder unit**: Kv 900, 0.15 kg, "
      "60 A, 4S. So at the same prop and cells, the twin has **twice the installed power**. "
      "The objective is the placeholder 3-lap time with a 20% energy reserve. The numbers show how the trade "
      "behaves in this model; they are not a design recommendation.\n")
    w("## Setup\n")
    w("- **Configs:**\n"
      "  - `configs/full_mdo.yaml`: n = 1, a nose tractor at x = −0.30 m.\n"
      "  - `configs/full_mdo_twin.yaml`: n = 2 at y_m = 0.2515 m (Woombah 9.9\") and x = −5.5\" (Woombah "
      "firewall), one pack per motor, Q_nac = 1.3 (Woombah), nacelle 0.15 × 0.05 m (placeholder), m_nacelle = 0 "
      "(placeholder, Q25), `nacelle_torsion` on, `motor_inertia_relief` off, body nacelle form factor.\n"
      "  - The two configs are otherwise identical: same DVs, bounds, tails, payload and mass model.")
    w("- **Outer enumeration (D4, D8):** n ∈ {1, 2} × prop ∈ {10x7E, 11x7E, 12x8E} at 4S. Then:\n"
      "  - a matched-power set of twins;\n"
      "  - relief on and the Raymer nacelle form factor, at the best twins.")
    w("- **Feasible** means SLSQP converged *and* the fixed current and RPM post-checks pass.\n")

    w("## 1. Optimum per layout and propeller\n")
    cols = list(enum)
    hdr = [f"{'1' if R[c]['n'] == 1 else '2'} × {R[c]['prop']}" for c in cols]
    w("| Quantity | " + " | ".join(hdr) + " |")
    w("|---|" + "---:|" * len(cols))
    rows = ["ThreeLapTime", "Lap1time", "b", "c", "tc_max", "camber_scale", "capacity", "E_nom_total", "l_ht",
            "S_ht", "S_vt", "x_batt", "x_cb_frac", "v", "m_total", "m_empty", "m_wing", "m_motor_group", "m_batt",
            "T_static", "P_static", "I_static", "cruise.L_over_D", "CD0", "CD_nac", "SM", "V_h", "V_v",
            "energy_margin", "thrust_margin_static", "rpm_margin"]
    for k in rows:
        w(f"| {k} | " + " | ".join(fmt(R[c].get(k, float('nan'))) for c in cols) + " |")
    w("| fus / tip prop clearance (mm) | " + " | ".join(
        "–" if R[c]["n"] == 1 else f"{1000 * R[c]['fus_clear_margin']:+.0f} / {1000 * R[c]['tip_clear_margin']:+.0f}"
        for c in cols) + " |")
    w("| status | " + " | ".join("feasible" if R[c]["feasible"] else _why(R[c]) for c in cols) + " |")
    w("| converged / iterations / time | " + " | ".join(
        f"{R[c]['success']} / {R[c]['iterations']} / {R[c]['time_s']:.0f} s" for c in cols) + " |")
    w("| current & RPM post-check | " + " | ".join(
        f"{'ok' if R[c]['limits']['ok'] else 'VIOLATED'} ({R[c]['limits']['I_margin_A']:+.0f} A, "
        f"{R[c]['limits']['rpm_margin']:+.0f} rpm)" for c in cols) + " |")
    w("")
    w("**Active constraints and DVs at bounds:**\n")
    for c in cols:
        w(f"- {hdr[cols.index(c)]}: {', '.join(R[c]['active']) or '–'}")
    w("")
    if b1 and b2:
        w(f"**Best feasible per layout:**\n"
          f"- single: **{b1['prop']}**, {b1['ThreeLapTime']:.2f} s;\n"
          f"- twin: **{b2['prop']}**, {b2['ThreeLapTime']:.2f} s ({b2['ThreeLapTime'] - b1['ThreeLapTime']:+.2f} s, "
          f"{100 * (b2['ThreeLapTime'] / b1['ThreeLapTime'] - 1):+.1f}%).\n\n"
          f"Installed static thrust: {b1['T_static']:.1f} N single, {b2['T_static']:.1f} N twin. "
          f"Static battery power: {b1['P_static']:.0f} W single, {b2['P_static']:.0f} W twin.\n")
    w("![enumeration](../../outputs/twin_study/enumeration_lap_time.png)\n")

    w("## 2. Matched installed power\n")
    w("The twin in §1 has twice the installed power, so §1 mostly says \"more power wins\". To separate layout from "
      "power, the twins below use smaller props and/or fewer cells. They were chosen from a powertrain-only screen "
      "at the single 10x7E/4S unit (21.4 N, 494 W static):\n"
      "- **matched static thrust (±10%):** 9x75E, 10x5E and 9x6E, all at 3S;\n"
      "- **matched static electrical power (±5%):** 10x7E/3S and 8x6E/4S.\n")
    mids = [k for k in R if not k.startswith("_") and R[k]["group"] == "matched"]
    w("| Twin | T_static (N) | P_static (W) | 3-lap (s) | Δ vs best single | b | c | m_total | v | feasible | "
      "active |")
    w("|---|---:|---:|---:|---:|---:|---:|---:|---:|---|---|")
    for k in sorted(mids, key=lambda k: R[k]["T_static"]):
        r = R[k]
        d = f"{r['ThreeLapTime'] - b1['ThreeLapTime']:+.2f} s" if b1 else "–"
        w(f"| {r['prop']} {r['cells']}S | {r['T_static']:.1f} | {r['P_static']:.0f} | {r['ThreeLapTime']:.2f} | {d} | "
          f"{r['b']:.3f} | {r['c']:.3f} | {r['m_total']:.3f} | {r['v']:.1f} | {r['feasible']} | "
          f"{', '.join(r['active'])} |")
    if b1:
        w(f"| *single {b1['prop']} 4S (reference)* | {b1['T_static']:.1f} | {b1['P_static']:.0f} | "
          f"{b1['ThreeLapTime']:.2f} | – | {b1['b']:.3f} | {b1['c']:.3f} | {b1['m_total']:.3f} | {b1['v']:.1f} | "
          f"{b1['feasible']} | {', '.join(b1['active'])} |")
    w("")
    w("![lap time vs thrust](../../outputs/twin_study/lap_time_vs_static_thrust.png)\n")

    w("## 3. Mass and drag breakdowns\n")
    trio = [(n, r) for n, r in (("best single", b1), ("best twin", b2), ("best matched twin", bm)) if r]
    w("| Item | " + " | ".join(f"{n} ({r['prop']} {r['cells']}S)" for n, r in trio) + " |")
    w("|---|" + "---:|" * len(trio))
    for lab_, f in [("wing (g)", lambda r: 1e3 * r["m_wing"]), ("tails (g)", lambda r: 1e3 * (r["m_ht"] + r["m_vt"])),
                    ("tail boom (g)", lambda r: 1e3 * r["m_tb"]), ("cargo bay (g)", lambda r: 1e3 * r["m_cb"]),
                    ("landing gear (g)", lambda r: 1e3 * r["m_gear"]),
                    ("motors, ESCs, props, nacelles (g)", lambda r: 1e3 * r["m_motor_group"]),
                    ("battery packs (g)", lambda r: 1e3 * r["m_batt"]),
                    ("avionics (g)", lambda r: 1e3 * r["m_avionics"]), ("payload (g)", lambda r: 1e3 * r["m_pay"]),
                    ("**total (g)**", lambda r: 1e3 * r["m_total"]),
                    ("drag area: induced (cm²)", lambda r: 1e4 * r["cruise.vlm.wing_perf.CDi"] * r["S"]),
                    ("drag area: wing profile (cm²)", lambda r: 1e4 * r["cruise.CDp"] * r["S"]),
                    ("drag area: tails (cm²)", lambda r: 1e4 * r["CD_tail"] * r["S"]),
                    ("drag area: fuselage (cm²)", lambda r: 1e4 * r["CD_fus"] * r["S"]),
                    ("drag area: nacelles (cm²)", lambda r: 1e4 * r["CD_nac"] * r["S"]),
                    ("**drag area: total (cm²)**", lambda r: 1e4 * r["cruise.CD"] * r["S"]),
                    ("cruise speed (m/s)", lambda r: r["v"]), ("wing area S (m²)", lambda r: r["S"])]:
        w(f"| {lab_} | " + " | ".join(f"{f(r):.1f}" if abs(f(r)) >= 10 else f"{f(r):.3f}" for _, r in trio) + " |")
    w("\nThe drag areas (C_D S) are each design's own cruise point. They compare directly across wings of different "
      "area.\n")
    w("![mass](../../outputs/twin_study/mass_breakdown_compare.png)\n")
    w("![drag](../../outputs/twin_study/drag_breakdown_compare.png)\n")

    w("## 4. Spar: motor inertia relief on vs off\n")
    w("| Design | relief | M_x root (N·m) | M_y root (N·m) | t_tube (mm) | r_tube (mm) | m_tube (g) | m_wing (g) | "
      "3-lap (s) |")
    w("|---|---|---:|---:|---:|---:|---:|---:|---:|")
    for k in [k for k in R if not k.startswith("_") and R[k]["group"] == "relief"]:
        for r in (R[R[k]["of"]], R[k]):
            w(f"| {r['prop']} {r['cells']}S | {'on' if r.get('relief') else 'off'} | {r['Mx']:.2f} | {r['My']:.2f} | "
              f"{1e3 * r['t_tube']:.3f} | {1e3 * r['r_tube']:.1f} | {1e3 * r['m_tube']:.1f} | {1e3 * r['m_wing']:.1f} | "
              f"{r['ThreeLapTime']:.3f} |")
    w("\nWhere t_tube = 0.500 mm, the wall is at the minimum-gauge placeholder (`t_tube_min`, Q3). Relief then lowers "
      "the root moment but not the mass, and the conservative default costs nothing. Where the loads size the wall "
      "(t_tube > 0.5 mm), relief saves what the table shows. In this study that is under 1 g of spar and a few "
      "milliseconds of lap time: the motor units are light compared with W, and y_m is small compared with b/2.\n")

    w("## 5. One engine inoperative (report only)\n")
    w("This is a post-check; the optimiser doesn't see it. The live unit runs at static thrust, the check speed is "
      "1.2 V_stall, and the rudder model is C_n,δr = η_v V_v a_v τ_r. η_v = 0.9, τ_r = 0.5, δ_max = 25° and the "
      "speed factor are PLACEHOLDERS. DBF has no OEI requirement that we know of, and the Woombah report has none.\n")
    w("| Twin | N = T_unit y_m (N·m) | V check (m/s) | C_n,req | V_v | rudder needed (deg) |")
    w("|---|---:|---:|---:|---:|---:|")
    for k, r in R.items():
        if k.startswith("_") or r["n"] != 2 or r["group"] not in ("enum", "matched") or not r.get("oei") \
                or not r["feasible"]:
            continue
        o = r["oei"]
        w(f"| {r['prop']} {r['cells']}S | {o['N_yaw_Nm']:.2f} | {o['V_check_mps']:.1f} | {o['Cn_req']:.3f} | "
          f"{r['V_v']:.4f} | {o['rudder_req_deg']:.0f} |")
    w("\nThe rudder can't hold any of these designs at full static thrust and low speed. A twin has to cut or "
      "reduce the live motor after a failure, for example with a throttle-cut on failsafe. The optimiser drives "
      "S_vt to its lower bound because nothing in the model rewards fin area, and adding an OEI constraint would "
      "change that. Not added, as briefed.\n")

    w("## 6. Sensitivities\n")
    w("Normalised sensitivities (x/t) ∂t/∂x of the 3-lap time at the best design of each layout. For the twin "
      "placeholders the raw derivative ∂t/∂x is given too, because m_nacelle = 0 makes its normalised value "
      "zero.\n")
    if b1 and b2:
        names = sorted(set(b1.get("sens", {})) | set(b2.get("sens", {})),
                       key=lambda k: -max(abs(b1["sens"].get(k, 0.0)), abs(b2["sens"].get(k, 0.0))))
        w("| Variable | single | twin |")
        w("|---|---:|---:|")
        for k in names:
            a_, b_ = b1["sens"].get(k), b2["sens"].get(k)
            w(f"| {k} | {'–' if a_ is None else f'{a_:+.4f}'} | {'–' if b_ is None else f'{b_:+.4f}'} |")
        w("\n| Twin placeholder (raw) | ∂t/∂x |")
        w("|---|---:|")
        units = {"m_nacelle": "s/kg (per unit)", "Q_nac": "s per unit Q", "l_nac": "s/m", "d_nac": "s/m",
                 "y_m": "s/m", "c_clear": "s/m"}
        for k in TWIN_ASSUMPTIONS:
            if k in b2.get("dT_dx", {}):
                w(f"| {k} | {b2['dT_dx'][k]:+.3f} {units[k]} |")
        w("")
        w("![sensitivities](../../outputs/twin_study/sensitivities_compare.png)\n")

    ffs = [k for k in R if not k.startswith("_") and R[k]["group"] == "ff"]
    if ffs:
        w("## 7. Nacelle form factor (PLAN §6h T6)\n")
        w("| Twin | body FF (as briefed): 3-lap (s), CD_nac | Raymer nacelle FF: 3-lap (s), CD_nac | Δ 3-lap |")
        w("|---|---|---|---:|")
        for k in ffs:
            r, r0 = R[k], R[R[k]["of"]]
            status = "" if r["success"] else f" (not converged, {r['iterations']} iterations)"
            w(f"| {r['prop']} {r['cells']}S | {r0['ThreeLapTime']:.2f}, {r0['CD_nac']:.4f} | "
              f"{r['ThreeLapTime']:.2f}, {r['CD_nac']:.4f}{status} | {r['ThreeLapTime'] - r0['ThreeLapTime']:+.2f} s |")
        w("")

    w("## 8. Wall-clock time\n")
    tw = [R[k]["time_s"] for k in R if not k.startswith("_") and R[k]["n"] == 2 and R[k]["success"]]
    w(f"- **One twin analysis** (`configs/full_mdo_twin.yaml`, initial point; num_y = 21, NeuralFoil xlarge):\n"
      f"  - build and setup: {t['build_s']:.1f} s;\n"
      f"  - coupled MDA, cold start: {t['analysis_cold_s']:.1f} s;\n"
      f"  - re-solve, warm: {t['analysis_warm_s']:.1f} s.")
    w(f"- **One twin optimisation** (SLSQP to convergence): median {np.median(tw):.0f} s, range {min(tw):.0f}–"
      f"{max(tw):.0f} s over the {len(tw)} converged twin runs. These ran 4 at a time on a 16-core laptop with 4 BLAS threads each; a "
      f"run on its own is ≈ 145 s for 10x7E (checkpoint 2).\n")

    w("## 9. What the model can and cannot say about this trade\n")
    w(_discussion(R, b1, b2, bm))
    w("\n## Figures\n")
    for f in ["enumeration_lap_time", "lap_time_vs_static_thrust", "mass_breakdown_compare", "drag_breakdown_compare",
              "sensitivities_compare"]:
        w(f"- `outputs/twin_study/{f}.pdf` (+ `.png`)")
    w("- `outputs/twin_study/planforms/planform_<case>.pdf`: the planform of every converged case")
    DOC.write_text("\n".join(L) + "\n")


def _discussion(R, b1, b2, bm):
    s = []
    if b1 and b2:
        s.append(f"- **Same unit, twice the power.** The best twin is {b2['ThreeLapTime'] - b1['ThreeLapTime']:+.2f} s "
                 f"against the best single, but it installs {b2['T_static'] / b1['T_static']:.2f}× the static thrust "
                 f"and {b2['P_static'] / b1['P_static']:.2f}× the static power. That is a property of the placeholder "
                 "powertrain, not of the layout.")
    if bm and b1:
        s.append(f"- **Matched power.** At about the single's static electrical power, the best twin "
                 f"({bm['prop']} {bm['cells']}S, {bm['T_static']:.1f} N, {bm['P_static']:.0f} W) is "
                 f"{bm['ThreeLapTime'] - b1['ThreeLapTime']:+.2f} s against the single. Here the layout effects are "
                 "visible:\n"
                 f"  - nacelle drag: C_D S ≈ {1e4 * bm['CD_nac'] * bm['S']:.0f} cm² with Q = 1.3 and the body form "
                 "factor;\n"
                 "  - the second motor, ESC and prop;\n"
                 "  - the CG shift from the motors at the wing;\n"
                 "  - smaller props, which run at a different advance ratio and efficiency;\n"
                 f"  - two packs: at the optimum each pack is at {bm['capacity']:.2f} Ah"
                 + (" (the capacity lower bound, a placeholder). So one pack per motor forces surplus energy, and its "
                    "mass, that the 3-lap objective doesn't need;\n" if bm["capacity"] <= 0.5001 else ";\n")
                 + "  - The match is only as close as the discrete prop and cell choices allow: ±10% on static thrust "
                 "or ±5% on static power.")
        thr = [r for k, r in R.items() if not k.startswith("_") and r["group"] == "matched" and r["cells"] == 3
               and r["feasible"]]
        if thr:
            s.append(f"- **Matching static thrust is not enough.** The 3S twins match the single's static thrust "
                     f"but are {min(r['ThreeLapTime'] for r in thr) - b1['ThreeLapTime']:+.0f} to "
                     f"{max(r['ThreeLapTime'] for r in thr) - b1['ThreeLapTime']:+.0f} s slower. With the same "
                     "Kv, a lower pack voltage lowers the no-load rotor speed, so the thrust falls off sooner with "
                     "speed and the full-throttle level speed drops. Lap time is set by thrust at speed, i.e. by "
                     "power and pitch speed. The comparison that means something is therefore the matched static "
                     "*power* one (4S).")
    act = [a for r in (b1, b2, bm) if r for a in r["active"]]
    to_active = any(a.startswith(("thrust_margin_static", "turn_speed_margin", "stall_margin")) for a in act)
    s.append("- **Slipstream is not modelled (PLAN §6h).** It is not neutral between the layouts:\n"
             "  - the twin washes about 2D of clean wing, the nose prop mostly the fuselage;\n"
             "  - leaving it out under-predicts the twin's take-off and turn C_L, and slightly under-predicts its "
             "washed-strip profile drag in cruise (a few percent of q there);\n"
             "  - it also omits the single's fuselage scrubbing drag.\n"
             + ("  - At the best designs here, a take-off, stall or turn constraint *is* active, so leaving out "
                "slipstream penalises the twin." if to_active else
                "  - At the best designs here, none of the take-off-thrust, stall or turn-speed constraints is "
                "active (§1, §2). The active ones are energy, stability, the tail volumes and DV bounds. So the "
                "omission is second order for these lap times; it would matter for a take-off-limited design."))
    s.append("- **Interference.** Q_nac = 1.3 is a single number from Woombah (after Raymer). Wing–nacelle "
             "interference depends on the nacelle's position and shape. The nacelle form factor is uncertain by "
             "2–3×: Raymer's body form against the nacelle form, §7 and PLAN §6h T6. That uncertainty is worth about "
             "0.4 s here, and the briefed body form is the pessimistic one for the twin. The nacelle mass is 0 "
             f"(placeholder, Q25), which is optimistic for the twin: about {bm['dT_dx']['m_nacelle'] if bm and 'dT_dx' in bm else float('nan'):.1f} s "
             "per kg per nacelle.")
    s.append("- **Structure.** The twin carries the nacelle torsion load (conservative default). Bending relief "
             "is off by default. At the best twin the spar is at minimum gauge; at the matched twin it is "
             "load-sized, but relief saves under 1 g (§4). So the twin's structural benefit is negligible at this "
             "scale and load factor (n = 5). Thrust and motor-torque loads on the wing are not modelled.")
    s.append("- **Not modelled at all:**\n"
             "  - the reasons Woombah chose a twin: clean flow at the nose for the glider and the airspeed sensor, "
             "and payload integration;\n"
             "  - redundancy;\n"
             "  - asymmetric thrust and handling, beyond the OEI post-check (§5);\n"
             "  - counter-rotation;\n"
             "  - ground clearance of the props on a taildragger.\n"
             "  - Those are requirements and constraints, not lap-time effects. They belong in the scoring plug-in "
             "or the constraints of a specific year.")
    s.append("- **What the model can say:** at a given installed thrust and power, how much nacelle drag and motor "
             "mass at the wing cost in lap time, and how the optimum wing and tail respond. That is the sensitivity "
             "the AIAC paper can report. A verdict that one layout is faster in general would not be supported, "
             "because it rests on the placeholders above.")
    return "\n".join(s)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--reuse", action="store_true", help="rebuild the doc and figures from the cached results")
    args = ap.parse_args()
    warnings.filterwarnings("ignore")
    OUT.mkdir(parents=True, exist_ok=True)
    R = json.loads(CACHE.read_text()) if args.reuse else run(args.workers)
    make_figures(R)
    write_doc(R)
    print("wrote", DOC, "and", OUT)


if __name__ == "__main__":
    main()
