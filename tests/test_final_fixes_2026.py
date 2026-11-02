"""Final fixes before the final 2025-26 run (docs/RESULTS_2026_PLAN.md, "Final fixes"): the shared warm path, the
sampler and m1_check under mda_robust, kv_set_by; the dbf2026 score cap (D1), the rigid payload fuselage (D2) and the
undercarriage and leakage drag areas (D3)."""

import importlib.util
import json
import warnings

import numpy as np
import pytest

from .conftest import ROOT

DATA = ROOT / "tests" / "data"
CFG26 = ROOT / "configs" / "dbf2026_multipoint.yaml"


def _results_2026():
    spec = importlib.util.spec_from_file_location("results_2026_ff", ROOT / "studies" / "results_2026.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def optimum():
    return json.loads((DATA / "final_fixes_designs.json").read_text())["integer_optimum"]


def _val(p, name):
    return float(np.ravel(p.get_val(name))[0])


# --- D5 and the smaller items ---------------------------------------------------------------------------------------

def test_kv_set_by_ignores_fuselage_margin():
    R = _results_2026()
    assert R.kv_set_by(["rules.fuselage_margin (lower)"], ["b (upper)"]) == "interior"
    assert R.kv_set_by(["rules.fuse_margin (lower)", "rules.fuselage_margin (lower)"], []) == "rules.fuse_margin (lower)"
    assert R.kv_set_by(["points.M2.rpm_margin (lower)"], ["Kv (lower)"]) == "points.M2.rpm_margin (lower), Kv (lower)"


def test_old_constraint_band_in_the_model_frame():
    """The old set's SM_max (0.20 + 0.20 c, Q62) is above the 2025-26 SM_min, so the sensitivity can be feasible."""
    R = _results_2026()
    mcfg = R.config(old_constraints=True)
    assert mcfg.base.stability.SM_max == 0.40 and mcfg.base.stability.SM_min == 0.25


def test_initial_values_shared():
    """results_2026.initial_values is problems.multipoint.initial_values (moved; same values)."""
    from aircrafter2.problems.multipoint import initial_values, load_multipoint_config

    R = _results_2026()
    mcfg = load_multipoint_config(CFG26)
    assert R.initial_values(mcfg) == initial_values(mcfg)


def _warm_problem():
    from aircrafter2.problems.multipoint import build_multipoint_problem, load_multipoint_config, run_warm

    mcfg = load_multipoint_config(CFG26)
    p = build_multipoint_problem(mcfg, driver=False, mda_retry=True)
    run_warm(p, mcfg, optimum())
    return mcfg, p


def test_m1_check_robust_at_the_optimum():
    """Under mda_robust m1_check builds the M1 model with RetryNewton's robust mode: passed, finite values."""
    from aircrafter2.problems.multipoint import m1_check

    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        mcfg, p = _warm_problem()
        m1 = m1_check(mcfg, p)
    assert m1["passed"] and "error" not in m1
    assert np.isfinite(m1["laps_frac"]) and all(np.isfinite(v) for v in m1["constraints"].values())


def test_sampler_robust_at_the_optimum():
    """sampling.evaluate on the 2025-26 problem at the integer optimum (warm path, RetryNewton robust): 4 rows, 2
    workers, every row converges."""
    from aircrafter2.problems.multipoint import load_multipoint_config
    from aircrafter2.robustness.sampling import evaluate

    mcfg = load_multipoint_config(CFG26)
    rows = [{"lap.wind": w, "f_build": f} for w, f in ((0.0, 1.0), (2.0, 0.9), (5.0, 1.1), (8.0, 1.0))]
    ev = evaluate(mcfg, optimum(), rows, ["flight_score", "points.M2.SM", "points.M3.v"], workers=2)
    assert ev["failed"] == [] and np.isfinite(ev["values"]).all()
