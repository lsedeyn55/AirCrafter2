"""2024-25 validation: Project Woombah as built, analysis only (docs/studies/results/validation_2025.md).

    .venv/bin/python studies/validation_2025.py corners      # MDA over the Kv / throttle / prop corners (v_top check)
    .venv/bin/python studies/validation_2025.py woombah      # Woombah in AirCrafter 2.0: full throttle and logged currents
    .venv/bin/python studies/validation_2025.py calibrate    # mass calibration on Woombah (configs/mass_usyd_woombah.yaml)
    .venv/bin/python studies/validation_2025.py report       # docs/studies/results/validation_2025.{md,json} + figures

Each part writes outputs/results/validation_<part>.json; report collects them. No value is tuned to the flight data:
the logged-current case sets the throttle from the recorded average current (an input condition of the flight),
everything else is the uncalibrated model.
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

warnings.filterwarnings("ignore")
ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "outputs" / "results"
DOCS = ROOT / "docs" / "studies" / "results"
LB, FT, MPH = 0.45359237, 0.3048, 0.44704
LAP_FT = 3000.0  # 500 + 1000 + 500 ft of straights + turns: Woombah's 3000 ft lap distance [W p. 25]

# Woombah as reported [W]: Table 34 (p. 45) masses in lb; Tables 38-39 (p. 59) flight results
TABLE34 = {"wing": 3.41, "fuselage_nose": 0.15, "fuselage": 1.23, "undercarriage": 0.66, "HT": 0.41, "VT": 0.19,
           "ESC_x2": 0.56, "receiver": 0.19, "receiver_battery": 0.30, "motor_x2": 0.50, "propeller_x2": 0.08,
           "payload_structure": 0.44, "bottle_x2": 13.5, "glider_structure": 0.22, "glider": 0.33, "battery_x2": 1.64,
           "M1": 9.32, "M2": 26.02, "M3": 10.09, "M3_empty": 9.76}
TABLE38 = {"M2_test3": dict(togw_lb=26.9, fuel_lb=13.23, current_A=34.0, speed_mph=51.1, flight_time_s=138.1,
                            avg_lap_s=37.4),
           "M2_test2": dict(togw_lb=26.9, fuel_lb=13.23, current_A=30.0, speed_mph=52.8, flight_time_s=152.3,
                            avg_lap_s=43.5),
           "M3_test2": dict(togw_lb=13.7, laps=7, current_A=17.5, speed_mph=52.3, flight_time_s=260.5, avg_lap_s=19.1),
           "M3_test1": dict(togw_lb=13.7, laps=6, current_A=17.0, speed_mph=52.5, flight_time_s=285.3, avg_lap_s=36.0)}
TABLE39 = {"M2": dict(design=dict(fuel_lb=13.0, speed_mph=87.0, avg_lap_s=33.0, flight_time_s=100.0, reserve=0.30),
                      actual=dict(fuel_lb=13.2, speed_mph=51.1, avg_lap_s=37.4, flight_time_s=138.1, reserve=0.33)),
           "M3": dict(design=dict(laps=8, speed_mph=80.0, avg_lap_s=28.0, flight_time_s=224.0, reserve=0.30,
                                  x1_lb=0.33),
                      actual=dict(laps=7, speed_mph=52.3, avg_lap_s=19.1, flight_time_s=260.5, reserve=0.14,
                                  x1_lb=0.37))}


def v(p, k):
    return float(np.ravel(p.get_val(k))[0])


def dump(name, data):
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / f"validation_{name}.json").write_text(json.dumps(data, indent=1, default=lambda o: o.item()
                                                            if hasattr(o, "item") else o.tolist()))


def load(name):
    return json.loads((OUT / f"validation_{name}.json").read_text())


def git_hash():
    return subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=ROOT, capture_output=True,
                          text=True).stdout.strip()


def woombah_config():
    from aircrafter2.problems.multipoint import load_multipoint_config

    return load_multipoint_config(ROOT / "configs" / "woombah_2025.yaml")


POINT_OUT = ["m_total", "m_empty", "m_pay", "m_wing", "m_ht", "m_vt", "m_fus", "m_tb", "m_cb", "m_gear", "m_sys",
             "m_batt", "v", "ThreeLapTime", "Lap1time", "otherLapTime", "laps_frac", "I_static", "I_pack_static",
             "rpm_margin", "SM", "CD_extra", "CD_stores", "CD_tanks", "CD_pylons", "CD_x1", "CD0", "energy_3laps",
             "E_usable", "E_nom_total", "energy_margin", "T_static", "thrust_margin_static", "v_fit_max",
             "stall_margin", "Vstall", "alpha_cruise", "cruise.L_over_D"]


def snapshot(p, mcfg):
    from aircrafter2.problems.multipoint import design_values

    out = dict(dv=design_values(p, mcfg), score={k: v(p, k) for k in ["score", "M1", "M2", "M3", "flight_score",
                                                                      "n_pre"]})
    for pt in ("M2", "M3"):
        out[pt] = {k: v(p, f"points.{pt}.{k}") for k in POINT_OUT}
        o = out[pt]
        n_packs = mcfg.base.propulsion.n_motors
        voc = mcfg.base.propulsion.n_cells * mcfg.base.propulsion.V_cell
        o["I_avg_pack_A"] = o["energy_3laps"] / o["ThreeLapTime"] / voc / n_packs  # 3-lap average battery current
        o["avg_speed_mph_3laps"] = 3 * LAP_FT * FT / o["ThreeLapTime"] / MPH
        o["reserve_after_3laps"] = 1.0 - o["energy_3laps"] / (o["E_nom_total"] * 3600.0)
    return out


def corners():
    """MDA convergence over Kv x throttle x prop at the Woombah airframe, fuel at its cap (heaviest M2)."""
    from openmdao.core.analysis_error import AnalysisError

    from aircrafter2.problems.multipoint import build_multipoint_problem

    rows = []
    for prop in ("9x6E", "10x7E", "11x8E"):
        for kv in (400.0, 1050.0):
            for th in (0.3, 1.0):
                mcfg = woombah_config()
                for pt in ("M2", "M3"):
                    mcfg.mp.points[pt].prop = prop
                p = build_multipoint_problem(mcfg, driver=False)
                p.set_val("Kv", kv)
                p.set_val("throttle_M2", th)
                p.set_val("throttle_M3", th)
                t = time.perf_counter()
                try:
                    p.run_model()
                    row = dict(prop=prop, Kv=kv, throttle=th, converged=True, time=time.perf_counter() - t)
                    for pt in ("M2", "M3"):
                        row[pt] = {k: v(p, f"points.{pt}.{k}") for k in ("v", "v_fit_max", "I_static", "rpm_margin",
                                                                         "laps_frac", "ThreeLapTime")}
                except AnalysisError as exc:
                    row = dict(prop=prop, Kv=kv, throttle=th, converged=False, error=str(exc)[:200])
                rows.append(row)
                print({k: x for k, x in row.items() if k in ("prop", "Kv", "throttle", "converged")},
                      {pt: round(row[pt]["v"], 1) for pt in ("M2", "M3")} if row["converged"] else "", flush=True)
    v_top = woombah_config().base.propulsion.v_top
    conv = [r for r in rows if r["converged"]]
    vmax = max(max(r["M2"]["v"], r["M3"]["v"]) for r in conv) if conv else float("nan")
    dump("corners", dict(rows=rows, v_top=v_top, v_max_level=vmax, n_failed=len(rows) - len(conv), git=git_hash()))
    print(f"max level speed {vmax:.1f} m/s vs v_top {v_top}; failures {len(rows) - len(conv)}")


def _run(th2, th3, ballast=None):
    """Woombah as built at throttles (th2, th3), on a fresh problem (a failed solve leaves NaN states behind).
    ballast: {point: kg} added through the mass model's m_ballast hook (to fly the logged take-off masses)."""
    from openmdao.core.analysis_error import AnalysisError

    from aircrafter2.problems.multipoint import build_multipoint_problem

    mcfg = woombah_config()
    p = build_multipoint_problem(mcfg, driver=False)
    p.set_val("throttle_M2", th2)
    p.set_val("throttle_M3", th3)
    for pt, kg in (ballast or {}).items():
        p.set_val(f"points.{pt}.m_ballast", kg)
    try:
        p.run_model()
    except AnalysisError:
        return None, mcfg
    return p, mcfg


def _pack_current(p, mcfg, pt):
    pr = mcfg.base.propulsion
    return v(p, f"points.{pt}.energy_3laps") / v(p, f"points.{pt}.ThreeLapTime") / (pr.n_cells * pr.V_cell
                                                                                   * pr.n_motors)


def _solve_throttle(pt, target_A, other=1.0):
    """Throttle of point pt giving the logged average pack current (3-lap average battery power / (V_oc n_packs)).
    Brackets downward from full throttle in steps of 0.05 (below some throttle the aircraft cannot fly level and
    the MDA has no solution), then bisects."""
    from scipy.optimize import brentq

    def cur(th):
        p, mcfg = _run(th, other) if pt == "M2" else _run(other, th)
        return None if p is None else _pack_current(p, mcfg, pt) - target_A

    hi, f_hi = 1.0, cur(1.0)
    lo = hi
    while True:
        lo = round(lo - 0.05, 4)
        f_lo = cur(lo) if lo >= 0.2 else None
        if f_lo is None:
            return dict(throttle=None, note=f"no level flight below throttle {lo + 0.05:.2f} at the logged current")
        if f_lo <= 0.0 <= f_hi:
            break
        hi, f_hi = lo, f_lo
    return dict(throttle=brentq(lambda th: cur(th), lo, hi, xtol=1e-4), note="")


def woombah():
    from aircrafter2.problems.multipoint import m1_check

    out = dict(git=git_hash(), config="configs/woombah_2025.yaml", cases={})
    p, mcfg = _run(1.0, 1.0)
    out["cases"]["full_throttle"] = snapshot(p, mcfg)
    out["cases"]["full_throttle"]["M1"] = m1_check(mcfg, p)
    print("full throttle:", {pt: {k: round(out["cases"]["full_throttle"][pt][k], 2) for k in
                                  ("m_total", "v", "ThreeLapTime", "laps_frac", "I_avg_pack_A")} for pt in ("M2", "M3")},
          flush=True)
    # logged currents (Table 38): M2 Test 3 34 A, M3 Test 2 17.5 A; read as the average current per pack
    s2 = _solve_throttle("M2", TABLE38["M2_test3"]["current_A"])
    s3 = _solve_throttle("M3", TABLE38["M3_test2"]["current_A"], other=s2["throttle"] or 1.0)
    out["logged_throttle"] = dict(M2=s2, M3=s3)
    print("logged throttles:", s2, s3, flush=True)
    if s2["throttle"] and s3["throttle"]:
        p, mcfg = _run(s2["throttle"], s3["throttle"])
        out["cases"]["logged_current"] = snapshot(p, mcfg)
        print("logged current:", {pt: {k: round(out["cases"]["logged_current"][pt][k], 2) for k in
                                       ("m_total", "v", "ThreeLapTime", "laps_frac", "I_avg_pack_A")}
                                  for pt in ("M2", "M3")}, flush=True)
        # + the logged take-off masses (Table 38: 26.9 lb in M2, 13.7 lb in M3), via ballast; two passes because the
        # M2 ballast also re-sizes the structure. An input condition of the flights, like the current.
        target = {"M2": TABLE38["M2_test3"]["togw_lb"] * LB, "M3": TABLE38["M3_test2"]["togw_lb"] * LB}
        ballast = {pt: 0.0 for pt in target}
        for _ in range(3):
            lc = out["cases"]["logged_current"] if not any(ballast.values()) else snap
            ballast = {pt: ballast[pt] + target[pt] - lc[pt]["m_total"] for pt in target}
            p, mcfg = _run(s2["throttle"], s3["throttle"], ballast)
            snap = snapshot(p, mcfg)
        snap["ballast_kg"] = ballast
        out["cases"]["logged_current_and_mass"] = snap
        print("logged current + mass:", {pt: {k: round(snap[pt][k], 2) for k in
                                              ("m_total", "v", "ThreeLapTime", "laps_frac", "I_avg_pack_A")}
                                         for pt in ("M2", "M3")}, flush=True)
    dump("woombah", out)


def calibrate():
    """Refit the G&G correction factors on Woombah's component masses (Eq. 127, one aircraft)."""
    import yaml

    from aircrafter2.config.mass import MassConfig
    from aircrafter2.mass.calibration import fit_correction_factors
    from aircrafter2.mass.garcia_gamboa.inputs import INPUTS
    from aircrafter2.problems.multipoint import build_multipoint_problem

    mcfg = woombah_config()
    p = build_multipoint_problem(mcfg, driver=False)
    p.run_model()
    grp = p.model.points.M2.mda.mass
    vals = {}
    for name, meta in grp.list_inputs(out_stream=None, prom_name=True, units=True):
        prom = meta["prom_name"].split(".")[-1]
        if prom in INPUTS and prom not in vals:
            vals[prom] = float(np.ravel(p.get_val(f"{grp.pathname}.{name}", units=INPUTS[prom][0]))[0])
    real = {"m_wing": TABLE34["wing"] * LB, "m_ht": TABLE34["HT"] * LB, "m_vt": TABLE34["VT"] * LB,
            "m_fus": (TABLE34["fuselage"] + TABLE34["fuselage_nose"]) * LB, "m_gear": TABLE34["undercarriage"] * LB}
    res = {}
    for n_panels in (1, 3):
        opts = mcfg.base.mass.options.model_copy(update={"n_panels": n_panels, "standalone_solver": True,
                                                        "n_motor_pairs": 1})
        mc = MassConfig(options=opts, inputs={**mcfg.base.mass.inputs, **vals})
        factors = ["f_wing", "f_ht", "f_vt", "f_cb", "f_gear"]
        r = fit_correction_factors([(mc, real)], factors)
        res[f"n_panels_{n_panels}"] = dict(factors=r.factors, objective=r.objective, errors=r.relative_errors,
                                           success=r.success, message=r.message)
        print(n_panels, r.factors, r.relative_errors, flush=True)
    chosen = res["n_panels_3"]["factors"]
    text = ("# Garcia & Gamboa correction factors refitted on ONE aircraft, Project Woombah (USYD DBF 2025), with the\n"
            "# Eq. 127 hook (aircrafter2/mass/calibration.py; studies/validation_2025.py calibrate).\n"
            "# One aircraft is one data point, and five factors against five component masses is an exact fit, not\n"
            "# evidence: use for the validation comparison only, never in the optimisations.\n"
            "# Real masses: 2025 USYD DBF Design Report, Table 34 (p. 45). n_panels = 3 (Woombah's three-piece wing,\n"
            "# Design Report p. 20): with n_panels = 1 the f_wing correction vanishes (f_wing^(n_panels-1), PLAN G5)\n"
            "# and the wing stays 75 % below Table 34 (docs/studies/results/validation_2025.md).\n"
            "extends: mass_garcia_gamboa_2022.yaml\noptions:\n  n_panels: 3\ninputs:\n"
            + "".join(f"  {k}: {float(x):.5g}\n" for k, x in chosen.items()))
    (ROOT / "configs" / "mass_usyd_woombah.yaml").write_text(text)
    dump("calibrate", dict(results=res, real_kg=real, git=git_hash()))
    del yaml


def figures(w):
    from aircrafter2.plotting.style import PALETTE, figure, save

    fdir = DOCS / "figures"
    fdir.mkdir(parents=True, exist_ok=True)
    files = []
    cases = [c for c in ("full_throttle", "logged_current", "logged_current_and_mass") if c in w["cases"]]
    fig, ax = figure(0.75, 0.6)
    labels = ["design (Table 39)", "flown (Table 39)"] + [f"AirCrafter 2.0, {c.replace('_', ' ')}" for c in cases]
    vals = [TABLE39["M2"]["design"]["speed_mph"], TABLE39["M2"]["actual"]["speed_mph"]]
    vals += [w["cases"][c]["M2"]["avg_speed_mph_3laps"] for c in cases]
    ax.barh(range(len(vals)), vals, color=[PALETTE[3], PALETTE[1]] + [PALETTE[0]] * len(cases))
    ax.set_yticks(range(len(vals)), labels)
    ax.set_xlabel("M2 average speed over 3 laps (mph)")
    files.append(save(fig, fdir / "woombah_m2_speed.pdf"))
    return files


def report():
    L = []
    w_ = L.append
    out = dict(git=git_hash())
    w_("# 2024-25 validation: Project Woombah as built\n")
    w_("Generated by `studies/validation_2025.py report` from `outputs/results/validation_*.json` (copied to "
       "`docs/studies/results/validation_2025.json`). Config: `configs/woombah_2025.yaml` (extends "
       "`configs/dbf2025_multipoint.yaml`). Sources: [W] 2025 USYD DBF Design Report; [R] Rules_2025.pdf. Analysis "
       "only, uncalibrated mass model unless a column says otherwise; **nothing is tuned to the flight data**.\n")
    w_("**Modelling differences (known before running):** rectangular-equivalent wing (same span and area; Woombah "
       "tapers 0.8); one spar tube for two; the model's fuselage is a cargo box plus a tail boom (Woombah: an 82 in "
       "fuselage); the lap model has no take-off, while the rules time M2 from the first throttle advance [R p. 11]; "
       "first-order motor model, APC surrogate props; stores drag from Woombah's own CFD bottle value [W p. 43].\n")
    if (OUT / "validation_corners.json").exists():
        c = load("corners")
        out["corners"] = c
        w_("## 1. Analysis envelope (Kv x throttle x prop)\n")
        w_(f"Woombah airframe, both points on the same prop, 6S. v_top = {c['v_top']:g} m/s; highest level speed of "
           f"any converged corner: {c['v_max_level']:.1f} m/s, so v_top is above every speed inside the Kv bounds "
           f"(400-1050 rpm/V).\n")
        w_("| Prop | Kv (rpm/V) | throttle | MDA | M2 v (m/s) | M3 v (m/s) |")
        w_("|---|---:|---:|---|---:|---:|")
        for r in c["rows"]:
            if r["converged"]:
                w_(f"| {r['prop']} | {r['Kv']:g} | {r['throttle']:g} | converged | {r['M2']['v']:.1f} | "
                   f"{r['M3']['v']:.1f} |")
            else:
                w_(f"| {r['prop']} | {r['Kv']:g} | {r['throttle']:g} | **not converged** | - | - |")
        w_(f"\n{c['n_failed']} corners do not converge. In each, thrust is below drag at every speed (throttle 0.3; "
           "and 400 KV with the 9x6E): the aircraft cannot hold level flight, so the speed balance has no solution. "
           "That is a physically infeasible design, not a speed above v_top; an optimiser step into it stops the run "
           "as MDA non-convergence (reported separately from constraint violation).\n")
    if (OUT / "validation_woombah.json").exists():
        wb = load("woombah")
        out["woombah"] = wb
        ft = wb["cases"]["full_throttle"]
        w_("## 2. Mass against Table 34\n")
        cal = load("calibrate") if (OUT / "validation_calibrate.json").exists() else None
        if cal:
            out["calibrate"] = cal
        rows = [("wing", "m_wing", TABLE34["wing"]), ("HT", "m_ht", TABLE34["HT"]), ("VT", "m_vt", TABLE34["VT"]),
                ("fuselage + nose", "m_fus", TABLE34["fuselage"] + TABLE34["fuselage_nose"]),
                ("undercarriage", "m_gear", TABLE34["undercarriage"])]
        hdr = "| Component | Table 34 (lb) | AirCrafter 2.0, uncalibrated (lb) | error |"
        if cal:
            hdr += " refitted, n_panels 1 (lb) | refitted, n_panels 3 (lb) |"
        w_(hdr)
        w_("|---|---:|---:|---:|" + ("---:|---:|" if cal else ""))
        for lab, key, real in rows:
            m = ft["M2"][key] / LB
            line = f"| {lab} | {real:.2f} | {m:.2f} | {100 * (m / real - 1):+.0f} % |"
            if cal:
                for npan in (1, 3):
                    err = cal["results"][f"n_panels_{npan}"]["errors"][0].get(key)
                    line += f" {real * (1 + err):.2f} |" if err is not None else " - |"
            w_(line)
        for lab, key, real in [("M1 (no payload)", None, TABLE34["M1"]), ("M2", "M2", TABLE34["M2"]),
                               ("M3", "M3", TABLE34["M3"])]:
            if key:
                w_(f"| **{lab} take-off mass** | {real:.2f} (Table 38: {TABLE38['M2_test3']['togw_lb'] if key == 'M2' else TABLE38['M3_test2']['togw_lb']}) "
                   f"| {ft[key]['m_total'] / LB:.2f} | {100 * (ft[key]['m_total'] / LB / real - 1):+.0f} % |"
                   + (" | |" if cal else ""))
        if "M1" in ft:
            w_(f"| **M1 take-off mass** | {TABLE34['M1']:.2f} | {ft['M1'].get('m_total', float('nan')) / LB:.2f} | "
               f"{100 * (ft['M1'].get('m_total', float('nan')) / LB / TABLE34['M1'] - 1):+.0f} % |" + (" | |" if cal else ""))
        w_("\nTable 34's M2 total (26.02 lb) is 2.21 lb above the sum of its own rows, and its M3 total omits the empty "
           "bottles the rules require; the flight-test take-off masses (Table 38: 26.9 and 13.7 lb) include ballast "
           "added after P1 [W p. 60].")
        if cal:
            f1, f3 = cal["results"]["n_panels_1"]["factors"], cal["results"]["n_panels_3"]["factors"]
            w_(f"\n**Calibration** (Eq. 127, one aircraft; `configs/mass_usyd_woombah.yaml`): factors "
               + ", ".join(f"{k} = {x:.3g}" for k, x in f1.items()) + " with n_panels = 1. With n_panels = 3 "
               "(Woombah's three-piece wing) f_wing enters as f_wing^2 (PLAN G5): " + ", ".join(
                   f"{k} = {x:.3g}" for k, x in f3.items()) + " (saved). With n_panels = 1 the wing cannot be corrected "
               "(f_wing drops out) and stays far below Table 34; with n_panels = 3 five factors fit five masses "
               "exactly, which is interpolation, not evidence. **One aircraft is one data point**: the refitted "
               "factors are a comparison column only, never used in the optimisations.\n")
        w_("## 3. Flight performance against Table 39\n")
        w_("Table 39 is not internally consistent (docs/RESULTS_PLAN.md §6), so each quantity is compared with each "
           "reading. M2 'time' readings: 112.2 s (3 × the 37.4 s average lap), 138.1 s (the total flight time, take-off "
           "included). Average speeds are over 3 × 3000 ft.\n")
        cases = [c for c in ("full_throttle", "logged_current", "logged_current_and_mass") if c in wb["cases"]]
        w_("| Quantity | Table 39 design | Table 39 actual | " + " | ".join(f"AirCrafter 2.0, {c.replace('_', ' ')}"
                                                                    for c in cases) + " |")
        w_("|---|---:|---:|" + "---:|" * len(cases))
        d2, a2, d3, a3 = (TABLE39["M2"]["design"], TABLE39["M2"]["actual"], TABLE39["M3"]["design"],
                          TABLE39["M3"]["actual"])
        cs = [wb["cases"][c] for c in cases]
        w_(f"| M2 average speed (mph) | {d2['speed_mph']:g} | {a2['speed_mph']:g} | "
           + " | ".join(f"{x['M2']['avg_speed_mph_3laps']:.1f}" for x in cs) + " |")
        w_(f"| M2 3-lap time (s) | {d2['flight_time_s']:g} | 112.2 or 138.1 | "
           + " | ".join(f"{x['M2']['ThreeLapTime']:.1f} (no take-off)" for x in cs) + " |")
        w_(f"| M2 average lap (s) | {d2['avg_lap_s']:g} | {a2['avg_lap_s']:g} | "
           + " | ".join(f"{x['M2']['otherLapTime']:.1f} (lap 2+)" for x in cs) + " |")
        w_(f"| M2 battery reserve after 3 laps | {100 * d2['reserve']:.0f} % | {100 * a2['reserve']:.0f} % | "
           + " | ".join(f"{100 * x['M2']['reserve_after_3laps']:.0f} %" for x in cs) + " |")
        w_(f"| M2 average current per pack (A) | 60 [W p. 26] | 34 (Test 3) | "
           + " | ".join(f"{x['M2']['I_avg_pack_A']:.1f}" for x in cs) + " |")
        w_(f"| M3 laps in the window (pre-launch) | {d3['laps']} ({d3['laps'] - 1}) | {a3['laps']} ({a3['laps'] - 1}) | "
           + " | ".join(f"{x['M3']['laps_frac']:.2f} ({x['M3']['laps_frac'] - 1:.2f})" for x in cs) + " |")
        w_(f"| M3 average speed (mph) | {d3['speed_mph']:g} | {a3['speed_mph']:g} | "
           + " | ".join(f"{x['M3']['avg_speed_mph_3laps']:.1f}" for x in cs) + " |")
        w_(f"| M3 average current per pack (A) | 50 [W p. 26] | 17.5 (Test 2) | "
           + " | ".join(f"{x['M3']['I_avg_pack_A']:.1f}" for x in cs) + " |")
        w_(f"| flight score M1 + M2 + M3 (B = 0) | | | " + " | ".join(f"{x['score']['flight_score']:.3f}" for x in cs)
           + " |")
        lt = wb.get("logged_throttle", {})
        if lt:
            w_("\nLogged-current case: throttles " + ", ".join(
                f"{k} {x['throttle']:.3f}" if x.get("throttle") else f"{k}: {x.get('note')}" for k, x in lt.items())
               + " (the throttle at which the 3-lap average pack current equals Table 38's average current, read as "
               "per pack; an input condition of the flight, not a fit).\n")
        w_("**Reading.** " + _reading(wb) + "\n")
    files = figures(out["woombah"]) if "woombah" in out else []
    if files:
        w_("![woombah_m2_speed](figures/woombah_m2_speed.png)\n")
        w_("*M2 average speed: Woombah's design target and flight result (Table 39) against AirCrafter 2.0.*\n")
    DOCS.mkdir(parents=True, exist_ok=True)
    (DOCS / "validation_2025.md").write_text("\n".join(L) + "\n")
    (DOCS / "validation_2025.json").write_text(json.dumps(out, indent=1, default=lambda o: o.item()
                                                          if hasattr(o, "item") else o.tolist()))
    print("\n".join(L))


def _reading(wb):
    c = wb["cases"]
    ft = c["full_throttle"]
    v_ft = ft["M2"]["avg_speed_mph_3laps"]
    s = (f"At full throttle the model flies M2 at {v_ft:.0f} mph average over 3 laps, drawing "
         f"{ft['M2']['I_avg_pack_A']:.0f} A per pack: above the ESC's 60 A and the APC rpm limit (rpm margin "
         f"{ft['M2']['rpm_margin']:.0f} rpm), so that design point is not flyable as built. ")
    flown = TABLE39["M2"]["actual"]["speed_mph"]
    if "logged_current" in c:
        v_lc = c["logged_current"]["M2"]["avg_speed_mph_3laps"]
        gap = v_ft - flown
        s += (f"At the logged current (34 A per pack) it gives {v_lc:.0f} mph: the operating point explains "
              f"{v_ft - v_lc:.0f} of the {gap:.0f} mph between full throttle and the flown {flown:g} mph "
              f"({100 * (v_ft - v_lc) / gap:.0f} %). ")
        if "logged_current_and_mass" in c:
            lm = c["logged_current_and_mass"]
            v_lm = lm["M2"]["avg_speed_mph_3laps"]
            s += (f"Flying the logged take-off masses as well (the uncalibrated model's M2 take-off mass is "
                  f"{c['logged_current']['M2']['m_total'] / LB:.1f} lb against 26.9 lb flown, i.e. "
                  f"{lm['ballast_kg']['M2'] / LB:.1f} lb added) gives {v_lm:.0f} mph: mass explains another "
                  f"{v_lc - v_lm:.0f} mph ({100 * (v_lc - v_lm) / gap:.0f} %). The remaining {v_lm - flown:.0f} mph "
                  f"({100 * (v_lm - flown) / gap:.0f} %) is what the model does not represent: take-off (in the "
                  "flown time, not in the model), the drag of the as-built airframe (model M2 C_D0 "
                  f"{lm['M2']['CD0']:.4f} with stores against Woombah's FlightStream C_D,min 0.0452 [W Table 26]), "
                  "the propulsion model at part throttle, and the pilot's line. ")
            s += (f"M3: {lm['M3']['laps_frac']:.1f} laps with the logged current and mass against 7 flown "
                  f"({c['logged_current']['M3']['laps_frac']:.1f} at the model's own mass). ")
    s += "No parameter was adjusted to the flight data; the current and mass are the flights' recorded conditions."
    return s


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("part", choices=["corners", "woombah", "calibrate", "report"])
    globals()[ap.parse_args().part]()
