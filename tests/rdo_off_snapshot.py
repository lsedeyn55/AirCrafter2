"""Snapshot of the deterministic full problem, for the RDO-off regression test (tests/test_rdo.py).

Uses only APIs that existed before RDO was added, so the reference (tests/data/rdo_off_reference.json) was made
by this same file on the pre-RDO commit. Regenerate (only if the deterministic model is meant to change):
    .venv/bin/python tests/rdo_off_snapshot.py tests/data/rdo_off_reference.json
The fixed-Kv configs (full_mdo_fixed_kv.yaml, full_mdo_twin_fixed_kv.yaml) are compared with this reference; the
Kv-as-design-variable defaults have their own (tests/data/kv_dv_reference.json, same script), regenerated
2026-10-22 after the placeholders migration (docs/PLACEHOLDERS_RULESETS_PLAN.md); the fixed-Kv configs still
reproduce rdo_off_reference.json bit for bit (since the USYD mass calibration, with the pre-calibration mass model,
tests/migration.py). kv_dv_reference.json was regenerated again on 2026-10-23 for the calibration.
"""

import json
import sys
import warnings
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
CONFIGS = ["full_mdo.yaml", "full_mdo_twin.yaml"]
OUTPUTS = ["ThreeLapTime", "Lap1time", "laps_frac", "energy_margin", "thrust_margin_static", "stall_margin",
           "turn_speed_margin", "rpm_margin", "SM", "V_h", "V_v", "wall_ratio", "r_tube", "m_total", "m_empty", "v",
           "alpha_cruise", "CD0", "k", "score"]


def _f(x):
    return [float(a) for a in np.ravel(x)]


def snapshot(cfg_name, iters=3, transform=None):
    from aircrafter2.problems.full_mdo import build_full_problem, load_full_config

    warnings.filterwarnings("ignore")
    cfg = load_full_config(ROOT / "configs" / cfg_name)
    if transform:  # e.g. tests/migration.py pre_calibration
        cfg = transform(cfg)
    prob = build_full_problem(cfg)
    prob.driver.options["disp"] = False
    prob.run_model()
    out = {"outputs": {k: _f(prob.get_val(k)) for k in OUTPUTS}}
    d = prob.driver
    out["dvs"] = {k: {f: _f(m[f]) if m[f] is not None else None for f in ("lower", "upper", "ref", "ref0")}
                  | {"units": m["units"]} for k, m in d._designvars.items()}
    out["cons"] = [[k, {f: _f(m[f]) if m[f] is not None else None for f in ("lower", "upper", "ref", "ref0")}
                    | {"units": m["units"]}] for k, m in d._cons.items()]
    out["objs"] = {k: {f: _f(m[f]) if m[f] is not None else None for f in ("ref", "ref0")} for k, m in d._objs.items()}
    out["con_values"] = {k: _f(v) for k, v in d.get_constraint_values().items()}
    J = prob.compute_totals(return_format="flat_dict")
    out["totals"] = {f"{of}|{wrt}": _f(v) for (of, wrt), v in J.items()}
    prob.driver.options["maxiter"] = iters
    prob.run_driver()
    out[f"after_{iters}_iterations"] = {k: _f(v) for k, v in d.get_design_var_values().items()}
    out[f"after_{iters}_iterations"]["score"] = _f(prob.get_val("score"))
    return out


if __name__ == "__main__":  # [output.json [config ...]]; the Kv-DV reference: tests/data/kv_dv_reference.json
    names = sys.argv[2:] or CONFIGS
    ref = {c: snapshot(c) for c in names}
    Path(sys.argv[1]).write_text(json.dumps(ref, indent=1))
