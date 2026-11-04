"""AirCrafter 1.0 on the 2024-25 ruleset, and its design turned into AirCrafter 2.0 designs two ways.

    .venv/bin/python studies/aircrafter1_2025.py run [--refine 1]   # MATLAB '2025' mode sweep (reference/aircrafter1)
    .venv/bin/python studies/aircrafter1_2025.py direct             # mapping (a): fill the rest by fixed rules, analyse
    .venv/bin/python studies/aircrafter1_2025.py completed          # mapping (b): span, AR, fuel fixed; optimise the rest

AirCrafter 1.0 outputs a partial design: M2 and M3 mass, span, AR, a cruise speed per mission, and a prop per
mission (propSelector's minimum-power row). The fixed rules of mapping (a), all stated in the report:
    chord          c = b / AR (rectangular wing), the config's seed airfoil (Clark Y) unscaled
    tails          S_ht = V_h S c / l_ht and S_vt = V_v S b / l_ht with V_h, V_v at the middle of the config's bands
                   (0.50, 0.04) and l_ht = the config's start (0.9 m): AirCrafter 1.0 has no tail sizing
    battery        AirCrafter 1.0's energy figure: the full 100 Wh, i.e. 50 Wh per pack, in both missions
    motor          the reference motor's Kv (1050 rpm/V, the V3115 Woombah flew)
    throttle       per mission, the throttle at which AirCrafter 2.0's level speed equals AirCrafter 1.0's chosen cruise
                   speed (the pilot flies the target speed), within the throttle bounds
    fuel           AirCrafter 1.0's M2 payload minus the stores (its payload fraction x mass is all payload), in the
                   smallest candidate bottle (stores.BOTTLES) whose pair holds it (any bottle size is legal, rules p. 8)
    stations       Woombah's battery and fuselage stations (Design Report Table 34)
"""

import os

os.environ.setdefault("OMP_NUM_THREADS", "3")

import argparse  # noqa: E402
import json  # noqa: E402
import subprocess  # noqa: E402
import sys  # noqa: E402
import warnings  # noqa: E402
from pathlib import Path  # noqa: E402

warnings.filterwarnings("ignore")
ROOT = Path(__file__).resolve().parents[1]
REF = ROOT / "reference" / "aircrafter1"
sys.path.insert(0, str(ROOT / "studies"))
import results_2025 as R  # noqa: E402

LB = 0.45359237
V_H, V_V = 0.50, 0.04  # middles of the config's tail-volume bands (V_h 0.35-0.65, V_v 0.02-0.06)


def run(refine=1):
    env = dict(os.environ, AC1_REFINE=str(refine))
    subprocess.run(["matlab", "-batch", "run('reference/aircrafter1/run_2025.m')"], cwd=ROOT, env=env, check=True)


def ac1(refine=1):
    return json.loads((REF / "runs" / f"ac1_2025_r{refine}.json").read_text())


def prop_name(row):
    return f"{row['Diameter']:g}x{row['Pitch']:g}E"


def partial_design(refine=1):
    a = ac1(refine)
    dp = a["DP"]
    props, notes = {}, {}
    for m in ("M2", "M3"):  # propSelector's min-power prop; if it selects none, Woombah's 10x7E (stated)
        sel = a.get(f"{m}_selected_prop")
        props[m] = prop_name(sel) if sel else "10x7E"
        notes[m] = "" if sel else "AirCrafter 1.0 selected no prop; Woombah's 10x7E used"
    fuel_kg = dp["M2_mass"] * dp["PayloadFraction"] - a["opts"]["m_stores_kg"]
    return dict(m2=dp["M2_mass"], m3=dp["M3_mass"], b=dp["b"], AR=dp["AR"], V2=dp["M2_V"], V3=dp["M3_V"],
                props=props, prop_notes=notes, fuel_kg=fuel_kg, laps_M3=dp["M3_LapCount"], score_M2=dp["M2score"],
                score_M3=dp["M3score"], other_lap_M2=dp["M2_OtherLapTime"])


def bottle_for(fuel_kg):
    from aircrafter2.stores import BOTTLES, Bottle, StoreConfig

    for key in ("1L", "2L"):
        if StoreConfig.from_ruleset(bottle=Bottle(**BOTTLES[key])).fuel_capacity >= fuel_kg:
            return key
    return "2L"


def _speed_throttle(fixed, props, pt, v_target, other, bottle):
    """Throttle at which point pt's level speed is v_target (bounded to the config's throttle bounds)."""
    from scipy.optimize import brentq

    from aircrafter2.problems.multipoint import build_multipoint_problem
    from openmdao.core.analysis_error import AnalysisError

    def speed(th):
        mcfg = R.config(m2=props["M2"], m3=props["M3"], bottle=bottle)   # analysis: the values are set below
        p = build_multipoint_problem(mcfg, driver=False)
        for k, x in fixed.items():
            p.set_val(k, x)
        p.set_val(f"throttle_{pt}", th)
        p.set_val(f"throttle_{'M3' if pt == 'M2' else 'M2'}", other)
        try:
            p.run_model()
        except AnalysisError:
            return None
        return R.v(p, f"points.{pt}.v")

    lo, hi = 0.5, 1.0
    s_lo, s_hi = speed(lo), speed(hi)
    if s_hi is not None and s_hi <= v_target:
        return hi, f"AirCrafter 2.0 cannot reach {v_target:g} m/s: full throttle ({s_hi:.1f} m/s)"
    if s_lo is not None and s_lo >= v_target:
        return lo, f"{v_target:g} m/s is below the speed at the lowest throttle ({s_lo:.1f} m/s)"
    if s_lo is None:
        return hi, "no level flight at the lowest throttle; full throttle used"
    return brentq(lambda th: speed(th) - v_target, lo, hi, xtol=1e-3), ""


def direct(refine=1):
    """Mapping (a): fill the rest by fixed rules, then analyse in AirCrafter 2.0 (nominal)."""
    from aircrafter2.problems.multipoint import build_multipoint_problem

    d = partial_design(refine)
    bottle = bottle_for(d["fuel_kg"])
    mcfg = R.config(m2=d["props"]["M2"], m3=d["props"]["M3"], bottle=bottle)
    b, c = d["b"], d["b"] / d["AR"]
    S = b * c
    l_ht = mcfg.base.initial.get("l_ht", 0.9)
    cap = 100.0 / mcfg.base.propulsion.n_motors / (mcfg.base.propulsion.n_cells * mcfg.base.propulsion.V_cell)
    fixed = dict(b=b, c=c, tc_scale=1.0, camber_scale=1.0, l_ht=l_ht, S_ht=V_H * S * c / l_ht,
                 S_vt=V_V * S * b / l_ht, x_cb_frac=0.438, Kv=mcfg.base.propulsion.Kv, fuel_mass=max(d["fuel_kg"], 0.0),
                 capacity_M2=cap, capacity_M3=cap, x_batt_M2=-0.0211, x_batt_M3=-0.746)
    th2, note2 = _speed_throttle(fixed, d["props"], "M2", d["V2"], 1.0, bottle)
    th3, note3 = _speed_throttle(fixed, d["props"], "M3", d["V3"], th2, bottle)
    fixed.update(throttle_M2=th2, throttle_M3=th3)
    p = build_multipoint_problem(mcfg, driver=True)
    for k, x in fixed.items():
        p.set_val(k, x)
    err = None
    try:
        p.run_model()
    except Exception as exc:  # MDA non-convergence
        err = f"MDA non-convergence: {str(exc)[:200]}"
    out = dict(tag="ac1_direct", mapping="a", ac1=d, bottle=bottle, rules=dict(V_h=V_H, V_v=V_V, l_ht=l_ht, capacity=cap,
                                                                throttle_notes=dict(M2=note2, M3=note3)),
               error=err, git=R.git_hash(), refine=refine)
    if err is None:
        out.update(R.summarise(p, mcfg))
    R.dump("ac1_direct", out)
    print("ac1_direct:", err or {k: round(x, 4) for k, x in out["score"].items()}, flush=True)


def completed(refine=1):
    """Mapping (b): span, AR and fuel at AirCrafter 1.0's values; the multipoint optimiser chooses the rest."""
    d = partial_design(refine)
    R.optimise("ac1_completed", m2=d["props"]["M2"], m3=d["props"]["M3"], bottle=bottle_for(d["fuel_kg"]),
               fixed=dict(b=d["b"], c=d["b"] / d["AR"], fuel_mass=max(d["fuel_kg"], 0.0)))


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("part", choices=["run", "direct", "completed"])
    ap.add_argument("--refine", type=int, default=1)
    ap.add_argument("--config", help="deterministic config, relative to the repo root")
    ap.add_argument("--out", help="output folder, relative to the repo root")
    a = ap.parse_args()
    R.use_paths(a.out, a.config)
    globals()[a.part](a.refine)
