"""Take-off in the mission time: re-analysis of the published 2024-25 designs (docs/studies/takeoff_study.md,
docs/TAKEOFF_COURSE_PLAN.md). Analysis only: nothing is re-optimised, no published result is touched.

    .venv/bin/python studies/takeoff_study.py run        # every design, take-off off and on (+ CL_ground, throttle)
    .venv/bin/python studies/takeoff_study.py report     # prints the tables (the docs are written from the JSON)

Designs: the six of results_2025.md section 2 (woombah, ac1_direct, ac1_completed, multipoint, multipoint_2L, rdo =
the reduced-set RDO, rdor_ms0) and the bottle optima of section 1b (0.5 L, 1 L = multipoint, 2 L = multipoint_2L), at
their published design variables. Take-off on = configs/dbf2025_multipoint_takeoff.yaml's course block.

Per design and mission point: t_takeoff, s_ground_roll, V_LOF, E_takeoff, the take-off margins, the 3-lap time, laps,
and the scores, take-off off and on; the Eq. 3.3 ground roll implied by the design's own static thrust
(static_thrust_calculator.m solved for s_g: V_R = 1.2 V_stall, drag at C_L,to and T = T(0) through the roll); the
ground roll at CL_ground 0, 0.5 and 1 (sensitivity to the PLACEHOLDER); and, as an estimate for the take-off-throttle
question only, the ground roll if the throttle were 1.0 (the whole point re-analysed at full throttle: only its
roll is read).

Output: outputs/takeoff/reanalysis.json.
"""

import os

os.environ.setdefault("OMP_NUM_THREADS", "2")

import argparse  # noqa: E402
import json  # noqa: E402
import sys  # noqa: E402
import warnings  # noqa: E402
from multiprocessing import get_context  # noqa: E402
from pathlib import Path  # noqa: E402

import numpy as np  # noqa: E402

warnings.filterwarnings("ignore")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "studies"))
OUT = ROOT / "outputs" / "takeoff"
TAKEOFF_CFG = ROOT / "configs" / "dbf2025_multipoint_takeoff.yaml"
FT = 0.3048
PT_OUT = ["t_takeoff", "s_ground_roll", "V_lof", "E_takeoff", "takeoff_accel_margin", "takeoff_straight_margin",
          "Lap1time", "otherLapTime", "ThreeLapTime", "laps_frac", "energy_margin", "energy_3laps", "Vstall", "v",
          "m_total", "S", "T_static", "thrust_margin_static", "turn_speed_margin", "stall_margin"]
SCORES = ["score", "M1", "M2", "M3", "flight_score"]


def designs():
    """[(label, MultipointConfig, dv)]: the six of section 2, then the 0.5 L bottle optimum."""
    import results_2025 as R

    R.use_set("reduced")
    out = R.mc_designs()
    b = R.best_case()
    d = R.load(f"{b['tag']}_bottle0.5L")
    out.append(("bottle_0.5L", R._mcfg_for(d), d["dv"]))
    return out


def _takeoff_block():
    from aircrafter2.problems.multipoint import load_multipoint_config

    return load_multipoint_config(TAKEOFF_CFG).base.course


def _v(p, k):
    return float(np.ravel(p.get_val(k))[0])


def eq33_roll(p, pt, mcfg):
    """static_thrust_calculator.m solved for the ground roll with the design's own T(0):
    s_g = m V_R^2 / (2 (T0 - q(V_R) S C_D,to - mu W)), V_R = 1.2 sqrt(W / (q S C_L,to)) (Eq. 3.3)."""
    g = lambda k: _v(p, f"points.{pt}.lap.{k}")  # noqa: E731
    m, S, rho, grav, CLto = g("m"), g("S"), g("rho"), g("g"), g("CL_to")
    CD0, CD1, k, mu = g("CD0"), g("CD1"), g("k"), g("mu_roll")
    T0 = float(np.ravel(p.get_val(f"points.{pt}.lap.T_coeffs"))[0]) * g("thrust_factor")
    W = m * grav
    VR = 1.2 * np.sqrt(W / (0.5 * rho * S * CLto))
    D = 0.5 * rho * VR**2 * S * (CD0 + CD1 * CLto + k * CLto**2)
    F = T0 - D - mu * W
    return dict(s_eq33=m * VR**2 / (2.0 * F) if F > 0 else float("inf"), V_R=VR, T0=T0, F_net_eq33=F,
                t_eq33=m * VR / F if F > 0 else float("inf"))  # constant net force: t = m V_R / F


def evaluate(job):
    """One design in one setting: job = (label, mcfg, dv, setting, overrides)."""
    from openmdao.core.analysis_error import AnalysisError

    from aircrafter2.problems.multipoint import build_multipoint_problem, m1_check

    label, mcfg, dv, setting, over = job
    mcfg = mcfg.model_copy(deep=True)
    if setting != "off":
        mcfg.base.course = _takeoff_block()
        for k, x in over.get("takeoff", {}).items():
            setattr(mcfg.base.course.takeoff, k, x)
    p = build_multipoint_problem(mcfg, driver=False, mda_retry=True)
    for k, x in {**dv, **over.get("dv", {})}.items():
        p.set_val(k, x)
    try:
        p.run_model()
    except AnalysisError as exc:
        return label, setting, dict(ok=False, error=str(exc)[:300])
    res = dict(ok=True, score={k: _v(p, k) for k in SCORES})
    for pt in ("M2", "M3"):
        res[pt] = {k: _v(p, f"points.{pt}.{k}") for k in PT_OUT}
        res[pt].update(eq33_roll(p, pt, mcfg))
        res[pt]["throttle"] = _v(p, f"throttle_{pt}")
        res[pt]["legs"] = p.model._get_subsystem(f"points.{pt}.lap").leg_breakdown() if setting == "on" else None
    if setting in ("off", "on"):
        m1 = m1_check(mcfg, p)
        res["M1"] = {k: m1.get(k) for k in ("passed", "laps_frac", "ThreeLapTime", "error")}
    return label, setting, res


def run(workers=4):
    OUT.mkdir(parents=True, exist_ok=True)
    path = OUT / "reanalysis.json"
    done = json.loads(path.read_text()) if path.exists() else {}
    jobs = []
    for label, mcfg, dv in designs():
        settings = [("off", {}), ("on", {}), ("CLg0", {"takeoff": {"CL_ground": 0.0}}),
                    ("CLg0.5", {"takeoff": {"CL_ground": 0.5}}), ("CLg1", {"takeoff": {"CL_ground": 1.0}}),
                    ("full_throttle", {"dv": {"throttle_M2": 1.0, "throttle_M3": 1.0}})]
        jobs += [(label, mcfg, dv, s, o) for s, o in settings if s not in done.get(label, {})]
    print(f"{len(jobs)} jobs", flush=True)
    with get_context("forkserver").Pool(workers) as pool:
        for label, setting, res in pool.imap_unordered(evaluate, jobs):
            done.setdefault(label, {})[setting] = res
            path.write_text(json.dumps(done, indent=1, default=float))
            print(label, setting, res.get("score", {}).get("flight_score", res.get("error")), flush=True)


def report():
    d = json.loads((OUT / "reanalysis.json").read_text())
    print("| design | pt | throttle | t_TO (s) | s_roll (ft) | V_LOF | s_Eq3.3 (ft) | V_R | dt3 (s) | dlaps | "
          "s_roll CLg 0 / 0.5 / 1 (ft) | t_TO full thr. (s) | s_roll full thr. (ft) |")
    for label, r in d.items():
        if not (r.get("off", {}).get("ok") and r.get("on", {}).get("ok")):
            print(label, {s: x.get("error") for s, x in r.items() if not x.get("ok")})
            continue
        for pt in ("M2", "M3"):
            on, off = r["on"][pt], r["off"][pt]
            c0, c5, c1, ft = (r.get(s, {}).get(pt, {}) for s in ("CLg0", "CLg0.5", "CLg1", "full_throttle"))
            print(f"| {label} | {pt} | {on['throttle']:.3f} | {on['t_takeoff']:.2f} | {on['s_ground_roll'] / FT:.0f} | "
                  f"{on['V_lof']:.2f} | {on['s_eq33'] / FT:.0f} | {on['V_R']:.2f} | "
                  f"{on['ThreeLapTime'] - off['ThreeLapTime']:+.2f} | {on['laps_frac'] - off['laps_frac']:+.3f} | "
                  f"{c0.get("s_ground_roll", np.nan) / FT:.0f} / {c5.get("s_ground_roll", np.nan) / FT:.0f} / {c1.get('s_ground_roll', np.nan) / FT:.0f} | "
                  f"{ft.get('t_takeoff', np.nan):.2f} | {ft.get('s_ground_roll', np.nan) / FT:.0f} |")
        so, sn = r["off"]["score"], r["on"]["score"]
        print(f"|   {label} scores | F {so['flight_score']:.4f} -> {sn['flight_score']:.4f} "
              f"(M2 {sn['M2'] - so['M2']:+.4f}, M3 {sn['M3'] - so['M3']:+.4f}) |")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["run", "report"])
    ap.add_argument("--workers", type=int, default=4)
    a = ap.parse_args()
    run(a.workers) if a.cmd == "run" else report()
