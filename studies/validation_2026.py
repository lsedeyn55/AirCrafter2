"""2025-26 validation: Project Humpback as built, analysis only (docs/studies/results_2026/validation_2026.md).

    .venv/bin/python studies/validation_2026.py envelope          # MDA over Kv x throttle x prop (the analysis envelope)
    .venv/bin/python studies/validation_2026.py humpback --mass loo   # USYD factors fitted WITHOUT Humpback (out-of-sample)
    .venv/bin/python studies/validation_2026.py humpback --mass gg    # G&G factors (uncalibrated)
    .venv/bin/python studies/validation_2026.py humpback --mass cal   # the calibrated factors (in-sample: the common
                                                                      # evaluation of results_2026 uses this one)
    .venv/bin/python studies/validation_2026.py report            # docs/studies/results_2026/validation_2026.{md,json}

Each part writes outputs/results_2026/validation_<part>.json. Nothing is tuned to the flight data: the flight cases set
the throttle from the logged average current and the take-off mass from the logged take-off weight ([HB] Table 42,
p. 58), both input conditions of the flights. Wind calm, take-off in the mission time (the ruleset), course
dbf_2025_360_mid. [HB] = 2026 USYD DBF Design Report (Project Humpback), printed page ("Page N of 60").
"""

import os

os.environ.setdefault("OMP_NUM_THREADS", "1")

import argparse  # noqa: E402
import json  # noqa: E402
import subprocess  # noqa: E402
import time  # noqa: E402
import warnings  # noqa: E402
from multiprocessing import get_context  # noqa: E402
from pathlib import Path  # noqa: E402

import numpy as np  # noqa: E402

warnings.filterwarnings("ignore")
ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "outputs" / "results_2026"
DOCS = ROOT / "docs" / "studies" / "results_2026"
LB, FT, IN = 0.45359237, 0.3048, 0.0254
LAP_FT = 2000.0 + 0.0  # 4 x 500 ft of straights per lap (the turns' distance is not counted)

# [HB] Table 36, p. 44 (lb); Q47 issues stand (the 'Mission 1' row, M2 18.54 vs 18.24 from the rows)
TABLE36 = {"fuselage": 6.14, "HT": 0.21 + 0.21, "VT": 0.28, "wing": 0.79 + 0.79, "motor": 1.27, "ESC": 0.24,
           "empty": 9.93, "battery": 1.22, "passengers_42": 1.84, "cargo_14": 5.25, "banner_with_deployment": 1.21,
           "M2": 18.54, "M3": 12.36}
# [HB] Table 42, p. 58: flight tests (take-off weight lb, prop, payload, average current A, average lap s)
TABLE42 = {"M2_test1": dict(point="M2", tow_lb=17.9, prop="19x10E", n_p=36, n_c=12, current_A=28.0, lap_s=38.8),
           "M2_test2": dict(point="M2", tow_lb=17.9, prop="19x12E", n_p=36, n_c=12, current_A=31.0, lap_s=33.2),
           "M3_test1": dict(point="M3", tow_lb=12.2, prop="19x10E", banner_in=118, current_A=32.0, lap_s=62.0),
           "M3_test2": dict(point="M3", tow_lb=12.4, prop="19x10E", banner_in=157, current_A=32.0, lap_s=56.0),
           "M3_test3": dict(point="M3", tow_lb=12.8, prop="19x10E", banner_in=197, current_A=35.0, lap_s=55.0)}
# [HB] Table 43, p. 59 (design / actual) and Table 1, p. 4 (actual)
TABLE43 = {"M2": dict(design=dict(n_p=30, n_c=10, laps=9, speed_fts=92.3, reserve=0.30),
                      actual=dict(n_p=42, n_c=14, laps=8, speed_fts=101.2, reserve=0.20)),
           "M3": dict(design=dict(banner_in=275, laps=5, speed_fts=49.9, reserve=0.30),
                      actual=dict(banner_in=197, laps=4, speed_fts=73.3, reserve=0.15)),
           "M1": dict(design=dict(lap_s=30, speed_fts=102.7), actual=dict(lap_s=28, speed_fts=108.5))}
TABLE1 = {"M1": dict(togw_lb=11.7, speed_fts=108, laps=3, time_s=84), "M2": dict(togw_lb=17.5, speed_fts=101, laps=9),
          "M3": dict(togw_lb=13.0, speed_fts=49.9, laps=5, banner_in=197)}
MASS_VARIANTS = {"gg": "G&G factors (uncalibrated)", "loo": "USYD factors without Humpback (out-of-sample)",
                 "cal": "USYD factors (calibrated, in-sample)"}
REPORT_VARIANTS = ("gg", "loo")
POINT_OUT = ["m_total", "m_empty", "m_pay", "m_wing", "m_ht", "m_vt", "m_fus", "m_tb", "m_cb", "m_gear", "m_sys",
             "m_batt", "v", "ThreeLapTime", "Lap1time", "otherLapTime", "laps_frac", "I_static", "rpm_margin", "SM",
             "x_cg", "x_np", "V_h", "V_v", "CD_extra", "CD_stores", "CD0", "energy_3laps", "E_usable", "E_nom_total",
             "energy_margin", "T_static", "thrust_margin_static", "v_fit_max", "v_fit_margin", "stall_margin", "Vstall",
             "t_takeoff", "s_ground_roll", "cruise.L_over_D", "l_cb"]


def v(p, k):
    return float(np.ravel(p.get_val(k))[0])


def git_hash():
    head = subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=ROOT, capture_output=True,
                          text=True).stdout.strip()
    dirty = subprocess.run(["git", "diff", "--quiet", "HEAD"], cwd=ROOT).returncode != 0
    return head + ("-dirty" if dirty else "")


def dump(name, data):
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / f"validation_{name}.json").write_text(json.dumps(data, indent=1, default=float))


def load(name):
    f = OUT / f"validation_{name}.json"
    return json.loads(f.read_text()) if f.exists() else None


def mass_override(variant):
    """(options, inputs) applied over the loaded config's mass model (above placeholders.yaml)."""
    from aircrafter2.config import load_yaml

    if variant == "gg":
        inp = load_yaml(ROOT / "configs" / "mass_garcia_gamboa_2022.yaml")["inputs"]
        return {"tail_mass_model": "gg"}, {**{k: inp[k] for k in ("f_wing", "f_ht", "f_vt", "f_cb", "f_tb", "f_gear")},
                                           "k_wing": 1.0}
    if variant == "loo":
        d = load_yaml(ROOT / "configs" / "mass_usyd_loo_humpback.yaml")
        return d.get("options", {}), d["inputs"]
    if variant == "cal":
        return {}, {}
    raise ValueError(variant)


def humpback_config(mass=None, m2_prop=None, m3_prop=None, payload=None):
    from aircrafter2.problems.multipoint import load_multipoint_config

    mcfg = load_multipoint_config(ROOT / "configs" / "humpback_2026.yaml")
    if mass:
        opts, inp = mass_override(mass)
        mcfg.base.mass.options = mcfg.base.mass.options.model_copy(update=opts)
        mcfg.base.mass.inputs.update(inp)
    if m2_prop:
        mcfg.mp.points["M2"].prop = m2_prop
    if m3_prop:
        mcfg.mp.points["M3"].prop = m3_prop
    mcfg.mp.payload_initial.update(payload or {})
    return mcfg


def snapshot(p, mcfg):
    from aircrafter2.problems.multipoint import design_values

    out = dict(dv=design_values(p, mcfg), score={k: v(p, k) for k in ["score", "M1", "M2", "M3", "flight_score",
                                                                      "net_income", "m3_term", "RAC", "EF"]})
    pr = mcfg.base.propulsion
    for pt in ("M2", "M3"):
        out[pt] = {k: v(p, f"points.{pt}.{k}") for k in POINT_OUT}
        o = out[pt]
        o["I_avg_pack_A"] = o["energy_3laps"] / o["ThreeLapTime"] / (pr.n_cells * pr.V_cell)  # 3-lap average current
        o["speed_fts_other_lap"] = LAP_FT / o["otherLapTime"]  # straights / lap time (as a lap-averaged speed)
    out["M3"]["m_banner"] = v(p, "points.M3.m_banner")
    out["M3"]["CD_banner"] = v(p, "points.M3.CD_banner")
    return out


def _run(mass, th2, th3, m2_prop=None, m3_prop=None, payload=None, ballast=None):
    """Humpback at throttles (th2, th3) on a fresh problem; ballast {point: kg} through the mass model's hook."""
    from openmdao.core.analysis_error import AnalysisError

    from aircrafter2.problems.multipoint import build_multipoint_problem

    mcfg = humpback_config(mass, m2_prop, m3_prop, payload)
    p = build_multipoint_problem(mcfg, driver=False, mda_retry=True)
    p.set_val("throttle_M2", 1.0)
    p.set_val("throttle_M3", 1.0)
    try:
        p.run_model()  # converge at full throttle first (a cold part-throttle M3 can fail), then move
        p.set_val("throttle_M2", th2)
        p.set_val("throttle_M3", th3)
        for pt, kg in (ballast or {}).items():
            p.set_val(f"points.{pt}.m_ballast", kg)
        p.run_model()
    except AnalysisError:
        return None, mcfg
    return p, mcfg


def _current(p, mcfg, pt):
    pr = mcfg.base.propulsion
    return v(p, f"points.{pt}.energy_3laps") / v(p, f"points.{pt}.ThreeLapTime") / (pr.n_cells * pr.V_cell)


def flight_case(job):
    """One [HB] Table 42 flight: the throttle that gives the logged average current, at the logged take-off mass
    (ballast through the mass model's hook), with the logged prop, payload or banner. Returns (name, result)."""
    from scipy.optimize import brentq

    mass, name = job
    t = TABLE42[name]
    pt = t["point"]
    payload = {"n_p": float(t["n_p"]), "n_c": float(t["n_c"])} if pt == "M2" else {"L_banner": float(t["banner_in"])}
    props = dict(m2_prop=t["prop"]) if pt == "M2" else dict(m3_prop=t["prop"])
    target = t["tow_lb"] * LB
    ballast = {pt: 0.0}
    res = dict(test=name, table42=t)

    def solve(bal):
        def f(th):
            p, mcfg = _run(mass, th if pt == "M2" else 1.0, th if pt == "M3" else 1.0, payload=payload, ballast=bal,
                           **props)
            return None if p is None else _current(p, mcfg, pt) - t["current_A"]
        hi, f_hi = 1.0, f(1.0)
        if f_hi is None or f_hi < 0:
            return None, "logged current above the full-throttle current" if f_hi is not None else "MDA failed"
        lo = hi
        while True:
            lo = round(lo - 0.05, 4)
            f_lo = f(lo) if lo >= 0.2 else None
            if f_lo is None:
                return None, f"no level flight below throttle {lo + 0.05:.2f}"
            if f_lo <= 0.0 <= f_hi:
                break
            hi, f_hi = lo, f_lo
        return brentq(f, lo, hi, xtol=1e-4), ""

    th, note = None, ""
    for _ in range(3):  # the ballast moves the current (heavier: more power), so iterate mass and throttle
        th, note = solve(ballast)
        if th is None:
            break
        p, mcfg = _run(mass, th if pt == "M2" else 1.0, th if pt == "M3" else 1.0, payload=payload, ballast=ballast,
                       **props)
        ballast = {pt: ballast[pt] + target - v(p, f"points.{pt}.m_total")}
    if th is not None:
        p, mcfg = _run(mass, th if pt == "M2" else 1.0, th if pt == "M3" else 1.0, payload=payload, ballast=ballast,
                       **props)
        res.update(throttle=th, ballast_kg=ballast[pt], snapshot=snapshot(p, mcfg))
    res["note"] = note
    return name, res


def humpback(mass, workers=3):
    from aircrafter2.problems.multipoint import m1_check

    out = dict(git=git_hash(), config="configs/humpback_2026.yaml", mass=mass, mass_label=MASS_VARIANTS[mass], cases={})
    p, mcfg = _run(mass, 1.0, 1.0)
    out["cases"]["as_built_full_throttle"] = snapshot(p, mcfg)
    out["cases"]["as_built_full_throttle"]["M1"] = m1_check(mcfg, p)
    print(mass, "as built:", {pt: {k: round(out["cases"]["as_built_full_throttle"][pt][k], 3) for k in
                                   ("m_total", "m_empty", "v", "laps_frac", "SM")} for pt in ("M2", "M3")}, flush=True)
    jobs = [(mass, name) for name in TABLE42]
    with get_context("forkserver").Pool(workers) as pool:
        for name, r in pool.imap_unordered(flight_case, jobs):
            out["cases"][name] = r
            s = r.get("snapshot", {}).get(r["table42"]["point"], {})
            print(mass, name, "throttle", r.get("throttle"), r.get("note"), {k: round(s[k], 2) for k in
                  ("m_total", "v", "otherLapTime", "laps_frac", "I_avg_pack_A") if k in s}, flush=True)
    dump(f"humpback_{mass}", out)


def envelope(workers=3):
    """MDA convergence and the level speed against the T(v) fit over the props x Kv bounds x throttle, Humpback's
    airframe (calibrated mass), as built: the analysis envelope of the 2025-26 problem."""
    jobs = [(m2, m3, kv, th) for m2, m3 in (("18x10E", "18x12E"), ("19x10E", "19x12E"), ("19x12E", "22x12E"))
            for kv in (200.0, 270.0) for th in (0.5, 1.0)]
    rows = []
    with get_context("forkserver").Pool(workers) as pool:
        for row in pool.imap_unordered(_envelope_job, jobs):
            rows.append(row)
            print(row["props"], row["Kv"], row["throttle"], row["converged"], flush=True)
    dump("envelope", dict(rows=rows, git=git_hash()))


def _envelope_job(job):
    from openmdao.core.analysis_error import AnalysisError

    from aircrafter2.problems.multipoint import build_multipoint_problem

    m2, m3, kv, th = job
    mcfg = humpback_config("cal", m2, m3)
    p = build_multipoint_problem(mcfg, driver=False, mda_retry=True)
    row = dict(props=f"{m2}/{m3}", Kv=kv, throttle=th)
    t = time.perf_counter()
    try:
        p.set_val("Kv", kv)
        p.run_model()
        p.set_val("throttle_M2", th)
        p.set_val("throttle_M3", th)
        p.run_model()
        row.update(converged=True, time=time.perf_counter() - t,
                   **{pt: {k: v(p, f"points.{pt}.{k}") for k in ("v", "v_fit_max", "v_fit_margin", "I_static",
                                                                  "rpm_margin", "laps_frac", "stall_margin")}
                      for pt in ("M2", "M3")})
    except AnalysisError as exc:
        row.update(converged=False, error=str(exc)[:200])
    return row


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("part", choices=["envelope", "humpback", "report"])
    ap.add_argument("--mass", choices=list(MASS_VARIANTS), default="loo")
    ap.add_argument("--workers", type=int, default=3)
    a = ap.parse_args()
    if a.part == "envelope":
        envelope(a.workers)
    elif a.part == "humpback":
        humpback(a.mass, a.workers)
    else:
        from validation_2026_report import report

        report()
