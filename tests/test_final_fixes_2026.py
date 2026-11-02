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


# --- D1: the score cap -------------------------------------------------------------------------------------------

def _x(n_p, n_c, laps2, E, laps3, L, b_in):
    return dict(n_p=np.array([n_p]), n_c=np.array([n_c]), laps_M2=np.array([laps2]), E_M2=np.array([E]),
                laps_M3=np.array([laps3]), L_banner=np.array([L]), b=np.array([b_in * 0.0254]))


@pytest.mark.parametrize("x", [_x(42, 14, 8, 97.68, 4, 197.0, 59.0),    # [HB] Table 43 'actual'
                               _x(42, 14, 9, 100.0, 5, 197.0, 59.0)])   # [HB] Table 33 design
def test_cap_exact_below(x):
    """Below 1 - delta the capped score is the old formula bit for bit (==)."""
    from aircrafter2.scoring.dbf2026 import DBF2026, m3_term, net_income

    plug = DBF2026(NI_max=2800.0, B_max=1500.0)
    p = plug.parts(x)
    ni = net_income(x["n_p"], x["n_c"], x["laps_M2"], x["E_M2"])
    t3 = m3_term(x["laps_M3"], x["L_banner"], x["b"] / 0.3048)
    assert p["r_M2"] < 1 - 0.005 and p["r_M3"] < 1 - 0.005
    assert p["M2"] == 1.0 + ni / 2800.0 and p["M3"] == 2.0 + t3 / 1500.0


def test_cap_exact_above():
    """Above 1 + delta, M2 = 2 and M3 = 3 exactly (the mr_smoke payload: NI 3559 > 2800)."""
    from aircrafter2.scoring.dbf2026 import DBF2026

    p = DBF2026(NI_max=2800.0, B_max=1500.0).parts(_x(97, 32, 9, 100.0, 9, 249.0, 60.0))
    assert p["r_M2"] > 1.005 and p["r_M3"] > 1.005
    assert p["M2"] == 2.0 and p["M3"] == 3.0 and p["flight_score"] == 6.0


def test_cap_blend_c1_by_hand():
    """cap = r - (r - 1 + d)^2 / (4 d) on [1 - d, 1 + d]: value and slope continuous at both joins (by hand: slope
    1 - (r - 1 + d) / (2 d) = 1 at 1 - d and 0 at 1 + d; value 1 at 1 + d); bias d / 4 at r = 1."""
    from aircrafter2.scoring.dbf2026 import cap

    d, h = 0.005, 1e-30
    slope = lambda r: float(np.imag(cap(r + 1j * h, d)) / h)  # noqa: E731
    blend = lambda r: r - (r - 1 + d) ** 2 / (4 * d)  # noqa: E731
    assert blend(1 - d) == 1 - d and blend(1 + d) == pytest.approx(1.0, abs=1e-15)
    assert float(cap(1 - d, d)) == 1 - d and float(cap(1 + d, d)) == 1.0
    assert slope(1 - d - 1e-9) == 1.0 and slope(1 + d + 1e-9) == 0.0
    assert slope(1 - d + 1e-9) == pytest.approx(1.0, abs=1e-6) and slope(1 + d - 1e-9) == pytest.approx(0.0, abs=1e-6)
    assert float(cap(1.0, d)) == pytest.approx(1 - d / 4, rel=1e-14)


def test_floored_hard_cap():
    """floored() applies min(r, 1): inside the blend it keeps r (below 1) or gives 1 (above), not the blend."""
    from aircrafter2.scoring.dbf2026 import DBF2026, net_income

    x = _x(42, 14, 8, 100.0, 4, 197.0, 59.0)
    ni = float(net_income(42, 14, 8, 100.0))
    for r in (0.998, 1.002):
        plug = DBF2026(NI_max=ni / r, B_max=1500.0)
        f = plug.floored(x)
        assert float(np.ravel(f["M2"])[0]) == pytest.approx(1.0 + min(r, 1.0), rel=1e-12)
        assert float(np.ravel(plug.parts(x)["M2"])[0]) != pytest.approx(1.0 + min(r, 1.0), rel=1e-6)  # the smooth one differs


@pytest.mark.parametrize("r", [0.9, 1.0, 1.1])
def test_cap_check_partials(r):
    """DBF2026Score (cs partials) against central FD below, inside and above the blend."""
    from aircrafter2.scoring.dbf2026 import DBF2026, net_income

    from .conftest import check_partials_fd, run_component

    ni = float(net_income(42, 14, 8, 100.0))
    plug = DBF2026(NI_max=ni / r, B_max=4 * 197.0 / (0.05 * 59 / 12 + 0.75) / r)
    x = {k: float(v[0]) for k, v in _x(42, 14, 8, 100.0, 4, 197.0, 59.0).items()}
    p = run_component(plug.component(), **x)
    check_partials_fd(p)
