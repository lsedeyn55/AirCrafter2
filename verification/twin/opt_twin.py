"""One full twin-motor optimisation (configs/full_mdo_twin.yaml), with motor inertia relief off and on.

Reports the optimum, the active constraints and DVs at bounds, the spar wall / wing mass for the
relief comparison, the OEI post-check and the wall-clock time.
Run: .venv/bin/python verification/twin/opt_twin.py [prop] [relief: off|on|both]
"""

import json
import sys
import time
import warnings
from pathlib import Path

import numpy as np

from aircrafter2.problems.full_mdo import build_full_problem, check_fixed_limits, load_full_config
from aircrafter2.stability import oei_check

warnings.filterwarnings("ignore")
ROOT = Path(__file__).resolve().parents[2]
KEYS = ["ThreeLapTime", "b", "c", "tc_max", "camber_scale", "capacity", "l_ht", "S_ht", "S_vt", "x_batt", "x_cb_frac",
        "v", "m_total", "m_empty", "m_wing", "m_tube", "t_tube", "r_tube", "wall_ratio", "Mx", "My", "Qz", "M_fl",
        "M_gd", "T_fl", "T_gd", "m_sys", "m_batt", "m_motor_group", "E_nom_total", "T_static", "T_static_unit",
        "I_static", "CD_extra", "CD_nac", "CD_fus", "CD_tail", "cruise.L_over_D", "SM", "V_h", "V_v",
        "fus_clear_margin", "tip_clear_margin", "prop_clear_margin", "y_m", "energy_margin", "thrust_margin_static",
        "stall_margin", "turn_speed_margin", "rpm_margin"]


def val(p, k):
    return float(np.ravel(p.get_val(k))[0])


def active(prob, tol=1e-4):
    """Constraints within tol of a bound and DVs at a bound, both in the driver's scaled units."""
    out = []
    cons = prob.driver.get_constraint_values(driver_scaling=True)
    for name, meta in prob.driver._cons.items():
        x = np.ravel(cons[name])
        for bound in ("lower", "upper"):
            b = meta[bound]
            if b is not None and np.all(np.abs(np.ravel(b)) < 1e20) and np.min(np.abs(x - np.ravel(b))) < tol:
                out.append(f"{meta['source']} ({bound})")
    dvs = prob.driver.get_design_var_values(driver_scaling=True)
    for name, meta in prob.driver._designvars.items():
        x = np.ravel(dvs[name])
        for bound in ("lower", "upper"):
            if np.min(np.abs(x - np.ravel(meta[bound]))) < 1e-6:
                out.append(f"DV {name} at {bound}")
    return out


def run(prop="10x7E", relief=False, cfg_name="full_mdo_twin_fixed_kv.yaml"):
    cfg = load_full_config(ROOT / "configs" / cfg_name)
    cfg.propulsion.prop = prop
    cfg.mass.options.motor_inertia_relief = relief
    prob = build_full_problem(cfg)
    prob.driver.options["disp"] = False
    t = time.perf_counter()
    try:
        res = prob.run_driver()
        ok, its = bool(res.success), int(res.iter_count)
    except Exception as exc:
        ok, its = False, f"error {exc.__class__.__name__}"
    dt = time.perf_counter() - t
    out = {k: val(prob, k) for k in KEYS}
    out.update(success=ok, iterations=its, time_s=dt, active=active(prob), limits=check_fixed_limits(prob),
               oei=oei_check(prob, cfg), relief=relief, prop=prop)
    return prob, out


if __name__ == "__main__":
    prop = sys.argv[1] if len(sys.argv) > 1 else "10x7E"
    mode = sys.argv[2] if len(sys.argv) > 2 else "both"
    results = {}
    for relief in {"off": [False], "on": [True], "both": [False, True]}[mode]:
        _, r = run(prop, relief)
        results["on" if relief else "off"] = r
        print(json.dumps(r, indent=1, default=str), flush=True)
    path = ROOT / "verification" / "twin" / f"opt_twin_{prop}.json"
    path.write_text(json.dumps(results, indent=1, default=str))
    print("wrote", path)
