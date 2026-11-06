"""Snapshot of every pre-existing full-problem config at its start point (one coupled analysis each), for the
regression test that the 2024-25 results work leaves them unchanged (tests/test_results_2025.py).

The reference (tests/data/configs_reference.json) was made by this file on commit b2adbb8 (kv-dv, before the results
branch), in a git worktree:
    .venv/bin/python tests/configs_snapshot.py tests/data/configs_reference.json
Regenerated 2026-10-22 after the placeholders migration (docs/development/PLACEHOLDERS_RULESETS_PLAN.md): full_mdo,
full_mdo_twin, full_mdo_rdo and full_mdo_twin_rdo changed (newer motor, battery, x_motor, take-off distance); the
four *_fixed_kv entries reproduced bit for bit.
Regenerated 2026-10-23 after the USYD mass calibration (docs/development/MASS_CALIBRATION_PLAN.md §7): every entry changed.
Regenerated 2026-10-27 after the Q45 fix (lap 1's clustered grid): only lap-1-derived outputs moved (<= 5e-7).
"""

import json
import sys
import warnings
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
CONFIGS = ["full_mdo.yaml", "full_mdo_twin.yaml", "full_mdo_fixed_kv.yaml", "full_mdo_twin_fixed_kv.yaml",
           "full_mdo_rdo.yaml", "full_mdo_twin_rdo.yaml", "full_mdo_rdo_fixed_kv.yaml", "full_mdo_twin_rdo_fixed_kv.yaml"]
OUTPUTS = ["ThreeLapTime", "laps_frac", "energy_margin", "thrust_margin_static", "stall_margin", "rpm_margin", "SM",
           "wall_ratio", "r_tube", "m_total", "m_empty", "m_wing", "v", "alpha_cruise", "CD0", "k", "I_static",
           "T_static", "score"]


def snapshot(cfg_name, root=ROOT):
    from aircrafter2.problems.full_mdo import build_full_problem, load_full_config

    warnings.filterwarnings("ignore")
    cfg = load_full_config(root / "configs" / cfg_name)
    cfg.aero.num_y = 11
    prob = build_full_problem(cfg, driver=False)
    prob.run_model()
    return {k: float(np.ravel(prob.get_val(k))[0]) for k in OUTPUTS}


if __name__ == "__main__":
    Path(sys.argv[1]).write_text(json.dumps({c: snapshot(c) for c in CONFIGS}, indent=1))
