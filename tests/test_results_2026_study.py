"""The final-run study code (studies/results_2026.py, results_2026_campaign.py, aircrafter1_2026.py) and the planform
overlay: overrides, the resume rule, the design selection, the wind-limit bisection and the violation count, drawn or
computed from stored data, with no optimisation."""

import importlib.util
import json
import sys

import numpy as np
import pytest

from .conftest import ROOT

DATA = ROOT / "tests" / "data"


def _load(name):
    sys.path.insert(0, str(ROOT / "studies"))
    spec = importlib.util.spec_from_file_location(f"{name}_t", ROOT / "studies" / f"{name}.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture(scope="module")
def R():
    return _load("results_2026")


def test_config_overrides_and_bounds(R):
    mcfg = R.config(overrides={"stability.V_h_min": 0.66, "multipoint.stores2026.m_seat": 0.0, "mass.inputs.r_tb": 0.01},
                    bounds={"c": {"upper": 0.6}})
    assert mcfg.base.stability.V_h_min == 0.66
    assert mcfg.mp.stores2026.m_seat == 0.0
    assert mcfg.base.mass.inputs["r_tb"] == 0.01
    assert mcfg.base.design_variables["c"].upper == 0.6
    assert mcfg.base.design_variables["c"].lower == R.config().base.design_variables["c"].lower
    with pytest.raises(KeyError):
        R.set_path(mcfg, "stability.no_such_value", 1.0)
    # the configs on disk are untouched: a fresh load has the rule values
    assert R.config().base.stability.V_h_min == 0.35


def test_placeholder_paths_exist_and_ranges_bracket(R):
    mcfg = R.config()
    for name, path, lo, hi, why in R.PLACEHOLDER_RANGES:
        val = R._value_at(mcfg, path)
        assert lo < hi and why, name
        assert lo <= val <= hi, f"{name}: the nominal {val} is outside its range"


def test_any_violation_counts_converged_rows_only(R):
    cons = [("a", dict(lower=0.0)), ("b", dict(upper=1.0, ref=1.0))]
    V = np.array([[1.0, 0.5], [-1.0, 0.5], [1.0, 2.0], [np.nan, np.nan]])
    assert R.any_violation(V, ["a", "b"], cons) == pytest.approx(2 / 3)
    # on the bound within the tolerance is not a violation (an active constraint no parameter moves)
    assert R.any_violation(np.array([[-1e-9, 1.0]]), ["a", "b"], cons) == 0.0


def test_wind_limit_bisection(R):
    limit = 13.37
    out = R.wind_limit_at(lambda w, th: "warm" if w <= limit else "lap_invalid", 15.0)
    assert out["limit"] <= limit < out["limit"] + 0.05
    assert out["stopped_by"] == "lap_invalid" and out["theta"] == 15.0
    assert R.wind_limit_at(lambda w, th: "warm", 0.0)["limit"] is None


def test_rounding_candidates(R):
    assert R.rounded_candidates(72.0, 24.0) == [(72, 24)]
    assert R.rounded_candidates(71.6, 23.4) == [(72, 23), (71, 23), (72, 24)]  # as the final-fixes rule found at fx_smoke


def test_box_designs_stay_inside_the_fuselage_cap(R):
    mcfg = R.config()
    a, b = R.box_designs(mcfg, 10), R.box_designs(mcfg, 10)
    assert a == b  # seeded
    pitch = mcfg.mp.stores2026.pax_pitch
    for d in a:
        assert d["n_p"] * pitch <= 1.5 * 1.524 + 1e-9
        assert d["b"] <= 1.524
        assert all(mcfg.all_dvs()[k].lower <= x for k, x in d.items())


def test_extended_list_acts_on_the_2026_problem(R):
    """Every out-of-set parameter is an input of the 2025-26 problem (no silently ignored spread)."""
    from aircrafter2.problems.multipoint import build_multipoint_problem

    mcfg = R.extended_config(R.config(cfg=R.RDO_CFG))
    names = {q.name for q in mcfg.base.robust.parameters}
    assert {"f_tb", "lap.thrust_factor", "Kv_factor", "R_cell", "a_ht", "a_vt"} <= names
    assert not names & R.NOT_IN_2026
    p = build_multipoint_problem(mcfg, driver=False)
    for name in names:
        for pt in ("M2", "M3"):
            assert np.isfinite(R.v(p, f"points.{pt}.{name}"))


def test_campaign_resume_rule(tmp_path):
    C = _load("results_2026_campaign")
    f = tmp_path / "x.json"
    assert not C.is_done(f, "abc1234")
    f.write_text(json.dumps(dict(git="abc1234")))
    assert C.is_done(f, "abc1234")
    assert not C.is_done(f, "def5678")
    f.write_text(json.dumps(dict(git="abc1234-dirty")))
    assert not C.is_done(f, "abc1234")
    f.write_text("{not json")
    assert not C.is_done(f, "abc1234")


def test_ac1_fixed_rules_keep_its_choices(R):
    A = _load("aircrafter1_2026")
    d = dict(b=1.52, c=0.38, f=0.5, n_p=42, n_c=14, banner_in=225)
    mcfg = R.config()
    x = A.fixed_rules(d, mcfg)
    assert x["b"] == 1.52 and x["c"] == 0.38 and x["n_p"] == 42.0 and x["n_c"] == 14.0 and x["L_banner"] == 225.0
    assert x["capacity_M3"] * 8 * mcfg.base.propulsion.V_cell == pytest.approx(100.0)
    assert x["capacity_M2"] == pytest.approx(0.5 * x["capacity_M3"])
    S = x["b"] * x["c"]
    assert x["S_ht"] * x["l_ht"] / (S * x["c"]) == pytest.approx(A.V_H)
    assert x["S_vt"] * x["l_ht"] / (S * x["b"]) == pytest.approx(A.V_V)
    assert set(x) | {"throttle_M2", "throttle_M3"} == set(mcfg.all_dvs())


def test_planform_geometry_matches_the_single_view_formulas():
    from aircrafter2.plotting.figures import planform_geometry

    g = dict(b=1.5, c=0.4, l_ht=1.0, S_ht=0.16, AR_ht=4.0, S_vt=0.06, AR_vt=1.5, boom_offset=0.15, l_cb=1.0, w_cb=0.2,
             x_cb=0.1, x_motor=-0.5, D_prop=0.48)
    geo = planform_geometry(g)
    assert geo["bh"] == pytest.approx(0.8) and geo["ch"] == pytest.approx(0.2)
    assert geo["ht"][1] == pytest.approx(0.25 * 0.4 + 1.0 - 0.05)
    assert geo["boom"] == pytest.approx((0.25, 1.1))
    assert geo["cargo"][1] == pytest.approx(0.1 - 0.5) and geo["cargo"][3] == 1.0
    assert geo["prop"] == pytest.approx((-0.51, 0.24))
    cv = (0.06 / 1.5) ** 0.5
    assert geo["vt"] == pytest.approx((1.1 - 0.25 * cv, 1.1 + 0.75 * cv))


@pytest.mark.parametrize("datum", ["wing_le", "motor"])
def test_planform_overlay_draws_from_stored_run_json(tmp_path, datum):
    from aircrafter2.plotting.figures import planform_overlay

    designs = json.loads((DATA / "planform_overlay_designs.json").read_text())
    out = planform_overlay(designs, tmp_path / f"overlay_{datum}.pdf", datum=datum)
    assert (tmp_path / f"overlay_{datum}.pdf").stat().st_size > 1000
    assert (tmp_path / f"overlay_{datum}.png").stat().st_size > 1000
    assert out == tmp_path / f"overlay_{datum}.pdf"


def test_planform_overlay_rejects_an_unknown_datum(tmp_path):
    from aircrafter2.plotting.figures import planform_overlay

    with pytest.raises(ValueError):
        planform_overlay([], tmp_path / "x.pdf", datum="tail")
