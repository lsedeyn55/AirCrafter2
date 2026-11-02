"""AirCrafter 1.0 on the 2025-26 ruleset, and its selected design turned into AirCrafter 2.0 designs two ways.

    .venv/bin/python studies/aircrafter1_2026.py direct      # mapping (a): fill the rest by fixed rules, analyse
    .venv/bin/python studies/aircrafter1_2026.py completed   # mapping (b): span, chord, counts, banner fixed; optimise the rest

The sweep itself: reference/aircrafter1/run_2026.m at the shipped steps, run by studies/results_2026_campaign.py into
outputs/results_2026/ac1/ (the stored ac1_2026_r1.json, on the coarse grid, is the fallback, labelled with its date).

AirCrafter 1.0 outputs a partial design: M2 and M3 mass, span, AR, a cruise speed per mission, a prop per mission
(propSelector's minimum-power row), the payload bundles of 3 ducks + 1 puck, the banner length and the M2 battery fraction.
The fixed rules of mapping (a), all stated on the page (MAPPING_TABLE):
    chord          c = b / AR (rectangular wing), the config's seed airfoil (S3016) unscaled
    tails          S_ht = V_h S c / l_ht and S_vt = V_v S b / l_ht with V_h, V_v at the middle of the 2025-26 bands
                   (0.575, 0.05) and l_ht = the config's start (Humpback's 1.1 m): AirCrafter 1.0 has no tail sizing
    battery        the M2 pack from the battery fraction (f x 100 Wh), the M3 pack the full 100 Wh, at 8S
    motor          the reference motor's Kv (245 rpm/V, the AT7215 Humpback flew)
    throttle       per mission, the throttle at which AirCrafter 2.0's level speed equals AirCrafter 1.0's cruise speed
                   (the pilot flies the target speed), within the throttle bounds
    stations       Humpback's battery and fuselage stations (the config's start)
"""

import os

os.environ.setdefault("OMP_NUM_THREADS", "3")

import argparse  # noqa: E402
import json  # noqa: E402
import sys  # noqa: E402
import warnings  # noqa: E402
from pathlib import Path  # noqa: E402

warnings.filterwarnings("ignore")
ROOT = Path(__file__).resolve().parents[1]
REF = ROOT / "reference" / "aircrafter1"
sys.path.insert(0, str(ROOT / "studies"))
import results_2026 as R  # noqa: E402

AC1_TAG = "fr_ac1_2026_shipped"
STORED = REF / "runs" / "ac1_2026_r1.json"
V_H, V_V = 0.575, 0.05  # middles of the 2025-26 tail-volume bands (V_h 0.35-0.80, V_v 0.02-0.08)
FALLBACK_PROPS = {"M2": "19x12E", "M3": "19x10E"}  # Humpback's, if propSelector selects none
MAPPING_TABLE = [
    ("span b", "AirCrafter 1.0", "AirCrafter 1.0"),
    ("chord c", "AirCrafter 1.0 (b / AR)", "AirCrafter 1.0 (b / AR)"),
    ("payload counts n_p, n_c", "AirCrafter 1.0 (bundles of 3 + 1)", "AirCrafter 1.0 (bundles of 3 + 1)"),
    ("banner length", "AirCrafter 1.0", "AirCrafter 1.0"),
    ("props per mission", "AirCrafter 1.0 (propSelector's minimum-power row)", "AirCrafter 1.0 (the same)"),
    ("M2 battery", "AirCrafter 1.0 (fraction x 100 Wh)", "AirCrafter 2.0 optimises capacity_M2"),
    ("M3 battery", "the full 100 Wh (AirCrafter 1.0's M3)", "AirCrafter 2.0 optimises capacity_M3"),
    ("throttle per mission", "matches AirCrafter 1.0's cruise speed (within 0.5-1)", "AirCrafter 2.0 optimises"),
    ("airfoil", "seed S3016, unscaled", "AirCrafter 2.0 optimises tc_scale, camber_scale"),
    ("tails S_ht, S_vt, arm l_ht", f"rule: V_h {V_H}, V_v {V_V} (band middles), l_ht 1.1 m", "AirCrafter 2.0 optimises"),
    ("motor Kv", "245 rpm/V (AT7215 as flown)", "AirCrafter 2.0 optimises"),
    ("wing, battery stations", "Humpback's", "AirCrafter 2.0 optimises"),
    ("masses", "AirCrafter 2.0's model (AirCrafter 1.0's mass is not imposed)", "AirCrafter 2.0's model"),
]


def ac1_file():
    """The sweep at the shipped steps if it exists, else the stored Oct 26 run (coarse grid; its label says so)."""
    f = R.OUT / "ac1" / f"{AC1_TAG}.json"
    return (f, "this campaign, shipped steps") if f.exists() else (STORED, "stored run of 2026-10-26, coarse grid")


def prop_name(row):
    return f"{row['Diameter']:g}x{row['Pitch']:g}E"


def partial_design():
    """AirCrafter 1.0's selected design from its run JSON: the part the mappings read."""
    f, label = ac1_file()
    a = json.loads(f.read_text())
    dp = a["DP"]
    props, notes = {}, {}
    for m in ("M2", "M3"):
        sel = a.get(f"{m}_selected_prop")
        props[m] = prop_name(sel) if sel else FALLBACK_PROPS[m]
        notes[m] = "" if sel else "AirCrafter 1.0 selected no prop; Humpback's used"
    return dict(source=str(f.relative_to(ROOT)), source_label=label, b=dp["b"], AR=dp["AR"], c=dp["b"] / dp["AR"],
                m2=dp["M2_mass"], m3=dp["M3_mass"], V2=dp["M2_V"], V3=dp["M3_V"], f=dp["battery_fraction"], n_p=dp["n_p"],
                n_c=dp["n_c"], banner_in=dp["banner_in"], props=props, prop_notes=notes, score=dp["score"],
                M2score=dp["M2score"], M3score=dp["M3score"], laps_M2=dp["M2_LapCount"], laps_M3=dp["M3_LapCount"],
                git=a.get("git"), grid=a.get("grid"), wall_s=a.get("wall_s"), n_points=a.get("n_points"),
                n_calls=a.get("n_performance_calls"), n_feasible=a.get("n_feasible"))


def fixed_rules(d, mcfg):
    """Mapping (a): every design variable from AirCrafter 1.0's partial design and the fixed rules (no throttles)."""
    b, c = d["b"], d["c"]
    S = b * c
    l_ht = mcfg.base.initial["l_ht"]
    pr = mcfg.base.propulsion
    cap_full = mcfg.mp.rules.energy_total_max_Wh / (pr.n_cells * pr.V_cell)
    return dict(b=b, c=c, tc_scale=1.0, camber_scale=1.0, l_ht=l_ht, S_ht=V_H * S * c / l_ht, S_vt=V_V * S * b / l_ht,
                x_cb_frac=mcfg.base.initial["x_cb_frac"], Kv=pr.Kv, capacity_M2=d["f"] * cap_full, capacity_M3=cap_full,
                x_batt_M2=mcfg.base.initial["x_batt"], x_batt_M3=mcfg.base.initial["x_batt"], n_p=float(d["n_p"]),
                n_c=float(d["n_c"]), L_banner=float(d["banner_in"]))


def speed_throttle(p, pt, v_target):
    """The throttle at which point pt's level speed is v_target, within 0.5-1 (the bounds); the note says when it can't."""
    from openmdao.core.analysis_error import AnalysisError
    from scipy.optimize import brentq

    def speed(th):
        p.set_val(f"throttle_{pt}", th)
        try:
            p.run_model()
        except AnalysisError:
            return None
        return R.v(p, f"points.{pt}.v")

    hi, s_hi = 1.0, speed(1.0)
    if s_hi is None:
        return hi, "MDA failed at full throttle"
    if s_hi <= v_target:
        return hi, f"AirCrafter 2.0 cannot reach {v_target:g} m/s: full throttle ({s_hi:.1f} m/s)"
    while hi > 0.5 + 1e-9:  # down in steps, warm: a level-flight limit above 0.5 must not lose the bracket
        lo = round(hi - 0.05, 4)
        s_lo = speed(lo)
        if s_lo is None:
            speed(hi)
            return hi, f"no level flight at throttle {lo:g}: throttle {hi:g} used ({s_hi:.1f} m/s)"
        if s_lo <= v_target:
            return brentq(lambda th: (speed(th) or 0.0) - v_target, lo, hi, xtol=1e-3), ""
        hi, s_hi = lo, s_lo
    return hi, f"{v_target:g} m/s is below the speed at the lowest throttle ({s_hi:.1f} m/s)"


def direct():
    """Mapping (a): fill the rest by fixed rules, then analyse in AirCrafter 2.0 (warm path, nominal)."""
    from openmdao.core.analysis_error import AnalysisError

    from aircrafter2.problems.multipoint import build_multipoint_problem, run_warm

    d = partial_design()
    mcfg = R.config(8, d["props"]["M2"], d["props"]["M3"])
    fixed = fixed_rules(d, mcfg)
    fixed.update(throttle_M2=1.0, throttle_M3=1.0)
    p = build_multipoint_problem(mcfg, driver=False, mda_retry=True)
    out = dict(tag="fr_ac1_direct", mapping="a", ac1=d, ok=False, its=0, time=None, cells=mcfg.base.propulsion.n_cells,
               props=d["props"], scoring=dict(mcfg.mp.scoring), rules=dict(V_h=V_H, V_v=V_V), table=MAPPING_TABLE,
               git=R.git_hash(), threads=os.environ.get("OMP_NUM_THREADS"), error=None)
    try:
        run_warm(p, mcfg, fixed)
        th2, note2 = speed_throttle(p, "M2", d["V2"])
        p.set_val("throttle_M2", th2)
        th3, note3 = speed_throttle(p, "M3", d["V3"])
        p.set_val("throttle_M3", th3)
        p.run_model()
        out.update(ok=True, throttle_notes=dict(M2=note2, M3=note3), mda_fallbacks=R.mda_fallbacks(p), **R.summarise(p, mcfg))
    except (AnalysisError, RuntimeError) as exc:
        out.update(error=f"MDA non-convergence: {str(exc)[:200]}", dv=fixed)
    R.dump(out["tag"], out)
    print("ac1_direct:", out["error"] or {k: round(x, 4) for k, x in out["score"].items()}, flush=True)


def completed():
    """Mapping (b): span, chord, payload counts and banner at AirCrafter 1.0's values; the optimiser chooses the rest."""
    d = partial_design()
    out = R.optimise("fr_ac1_completed", 8, d["props"]["M2"], d["props"]["M3"],
                     x0={"L_banner": float(d["banner_in"])},
                     fixed=dict(b=d["b"], c=d["c"], n_p=float(d["n_p"]), n_c=float(d["n_c"]), L_banner=float(d["banner_in"])))
    out.update(mapping="b", ac1=d, table=MAPPING_TABLE, scoring=out.get("scoring") or {})
    R.dump(out["tag"], out)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("part", choices=["direct", "completed"])
    globals()[ap.parse_args().part]()
