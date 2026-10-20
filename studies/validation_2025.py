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


def _solve_throttle(p, pt, target_A, voc_np, lo=0.3, hi=1.0):
    """Throttle of point pt giving the logged average pack current: the 3-lap average battery power / (V_oc n_packs)."""
    from scipy.optimize import brentq

    def f(th):
        p.set_val(f"throttle_{pt}", th)
        p.run_model()
        return v(p, f"points.{pt}.energy_3laps") / v(p, f"points.{pt}.ThreeLapTime") / voc_np - target_A

    return brentq(f, lo, hi, xtol=1e-4)


def woombah():
    from aircrafter2.problems.multipoint import build_multipoint_problem, m1_check

    mcfg = woombah_config()
    pr = mcfg.base.propulsion
    out = dict(git=git_hash(), config="configs/woombah_2025.yaml", cases={})
    p = build_multipoint_problem(mcfg, driver=False)
    voc_np = pr.n_cells * pr.V_cell * pr.n_motors
    p.run_model()
    out["cases"]["full_throttle"] = snapshot(p, mcfg)
    out["cases"]["full_throttle"]["M1"] = m1_check(mcfg, p)
    print("full throttle:", {pt: {k: round(out["cases"]["full_throttle"][pt][k], 2) for k in
                                  ("m_total", "v", "ThreeLapTime", "laps_frac", "I_avg_pack_A")} for pt in ("M2", "M3")},
          flush=True)
    # logged currents (Table 38): M2 Test 3 34 A, M3 Test 2 17.5 A; read as the average current per pack
    th2 = _solve_throttle(p, "M2", TABLE38["M2_test3"]["current_A"], voc_np)
    th3 = _solve_throttle(p, "M3", TABLE38["M3_test2"]["current_A"], voc_np)
    p.set_val("throttle_M2", th2)
    p.set_val("throttle_M3", th3)
    p.run_model()
    out["cases"]["logged_current"] = snapshot(p, mcfg)
    out["cases"]["logged_current"]["throttle"] = dict(M2=th2, M3=th3)
    print("logged current:", {pt: {k: round(out["cases"]["logged_current"][pt][k], 2) for k in
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
            vals[prom] = float(np.ravel(p.get_val(name, units=INPUTS[prom][0]))[0])
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
    chosen = res["n_panels_1"]["factors"]
    text = ("# Garcia & Gamboa correction factors refitted on ONE aircraft, Project Woombah (USYD DBF 2025), with the\n"
            "# Eq. 127 hook (aircrafter2/mass/calibration.py; studies/validation_2025.py calibrate). One aircraft is\n"
            "# one data point: use for the validation comparison only, never in the optimisations.\n"
            "# Real masses: 2025 USYD DBF Design Report, Table 34 (p. 45). n_panels = 1 (rectangular equivalent).\n"
            "extends: mass_garcia_gamboa_2022.yaml\ninputs:\n"
            + "".join(f"  {k}: {float(x):.5g}\n" for k, x in chosen.items()))
    (ROOT / "configs" / "mass_usyd_woombah.yaml").write_text(text)
    dump("calibrate", dict(results=res, real_kg=real, git=git_hash()))
    del yaml


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("part", choices=["corners", "woombah", "calibrate", "report"])
    globals()[ap.parse_args().part]()
