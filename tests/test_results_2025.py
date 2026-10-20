"""2024-25 results work: the dbf2025 scoring plug-in, the store modules, the part-throttle powertrain fit, the
multipoint problem (shared airframe, totals), and the regression of every pre-existing config."""

import json

import numpy as np
import pytest
from openmdao.utils.assert_utils import assert_check_partials, assert_check_totals

from .conftest import CONFIGS, ROOT, run_component

LB = 0.45359237


# --- scoring ---------------------------------------------------------------------------------------------------
def test_dbf2025_matches_the_rules_away_from_the_kink():
    from aircrafter2.scoring.dbf2025 import DBF2025, m2_verbatim, m3_verbatim

    pl = DBF2025(R2=0.2, R3=27.0, bonus=2.5, x1_weight_lb=0.37)
    for fuel_lb, t3, laps in [(13.2, 112.2, 7.0), (13.2, 138.1, 7.0), (8.0, 90.0, 3.4), (20.0, 70.0, 11.0)]:
        p = pl.parts({"fuel_mass": np.array(fuel_lb * LB), "t3_M2": np.array(t3), "laps_M3": np.array(laps)})
        assert p["M2"] == pytest.approx(m2_verbatim(fuel_lb, t3, 0.2), rel=1e-12)
        assert p["M3"] == pytest.approx(m3_verbatim(laps - 1.0, 2.5, 0.37, 27.0), rel=1e-9)
        assert p["flight_score"] == pytest.approx(1.0 + p["M2"] + p["M3"], rel=1e-14)


def test_dbf2025_hand_worked_woombah_case():
    """Woombah's flown numbers (Design Report Table 39): 13.2 lb in M2; 7 laps in M3 = 6 before the launch; the
    0.37 lb X-1. Against the official USYD scores (2025 results: M2 1.58, M3 2.28), the best-team values that
    reproduce them are R2 = (13.2 / t) / 0.58 and R3 = (6 + B / 0.37) / 0.28."""
    from aircrafter2.scoring.dbf2025 import m2_verbatim, m3_verbatim

    for t3 in (112.2, 138.1):  # Table 39's two readings of the M2 time (3 x 37.4 s, or the 138.1 s flight time)
        r2 = 13.2 / t3 / 0.58
        assert m2_verbatim(13.2, t3, r2) == pytest.approx(1.58, abs=1e-12)
    assert 13.2 / 138.1 / 0.58 == pytest.approx(0.1648, abs=1e-4) and 13.2 / 112.2 / 0.58 == pytest.approx(0.2028,
                                                                                                             abs=1e-4)
    for bonus, r3 in [(0.0, 21.43), (2.5, 45.56)]:
        assert m3_verbatim(6.0, bonus, 0.37, (6.0 + bonus / 0.37) / 0.28) == pytest.approx(2.28, abs=1e-12)
        assert (6.0 + bonus / 0.37) / 0.28 == pytest.approx(r3, abs=0.01)


def test_dbf2025_component_partials():
    from aircrafter2.scoring.dbf2025 import DBF2025, DBF2025Score

    p = run_component(DBF2025Score(plugin=DBF2025(bonus=2.5)), fuel_mass=5.9, t3_M2=95.0, laps_M3=7.3)
    assert_check_partials(p.check_partials(method="cs", out_stream=None), atol=1e-10, rtol=1e-10)


# --- stores ----------------------------------------------------------------------------------------------------
@pytest.mark.parametrize("mission", ["M1", "M2", "M3"])
def test_stores_mass_station_drag_and_partials(mission):
    from aircrafter2.problems.multipoint import STORE_FLAGS
    from aircrafter2.stores import StoreConfig, Stores

    cfg = StoreConfig()
    flags = STORE_FLAGS[mission]
    inputs = dict(v=35.0, S=0.59, c=0.33) | (dict(fuel_mass=5.0) if flags["fuel"] else {})
    p = run_component(Stores(config=cfg, **flags), **inputs)
    m = p.get_val("c.m_pay")[0]
    if mission == "M1":
        assert m == 0.0 and p.get_val("c.CD_stores")[0] == 0.0
    else:
        expect = cfg.n_ext * cfg.bottle.empty_mass + cfg.m_pylons + cfg.m_x1 + cfg.m_x1_structure
        expect += 5.0 if mission == "M2" else 0.0
        assert m == pytest.approx(expect, rel=1e-14)
        cd_t = cfg.n_ext * cfg.cd_bottle * np.pi * cfg.bottle.diameter**2 / 4 / 0.59
        assert p.get_val("c.CD_tanks")[0] == pytest.approx(cd_t, rel=1e-14)
        assert p.get_val("c.CD_stores")[0] > cd_t
    assert cfg.fuel_capacity == pytest.approx(2 * 1.0 * 6.5 * LB, rel=1e-14)
    assert_check_partials(p.check_partials(method="cs", out_stream=None), atol=1e-10, rtol=1e-8)


# --- part-throttle powertrain fit -----------------------------------------------------------------------------------
def test_powertrain_part_throttle_fit_stays_physical():
    from aircrafter2.propulsion.powertrain import Powertrain

    ins = dict(n_cells=6.0, Kv=1050.0, Rm=0.03384, I0=1.79, capacity=2.25)
    fixed = run_component(Powertrain(prop="10x7E", v_top=80.0, n_motors=2), throttle=0.5, **ins)
    scaled = run_component(Powertrain(prop="10x7E", v_top=80.0, n_motors=2, fit_range="throttle"), throttle=0.5,
                           **ins)
    assert fixed.get_val("c.fit_residual_T")[0] > 10.0  # the [0, v_top] fit fails at part throttle
    assert scaled.get_val("c.fit_residual_T")[0] < 0.5
    tc = scaled.get_val("c.T_coeffs")
    T = [sum(tc[i] * v**i for i in range(4)) for v in np.linspace(0, scaled.get_val("c.v_fit_max")[0], 9)]
    assert np.all(np.diff(T) < 0)  # thrust falls with speed over the fit range
    data = scaled.check_partials(method="fd", form="central", step=1e-6, step_calc="rel_avg", out_stream=None,
                                 compact_print=True, includes=["c"])
    keep = {k: x for k, x in data["c"].items() if k[0] in ("T_coeffs", "P_coeffs", "v_fit_max", "T_static")}
    assert_check_partials({"c": keep}, atol=1e-4, rtol=1e-3)


# --- multipoint problem --------------------------------------------------------------------------------------------
@pytest.fixture(scope="module")
def multipoint():
    from aircrafter2.problems.multipoint import build_multipoint_problem, load_multipoint_config

    mcfg = load_multipoint_config(CONFIGS / "dbf2025_multipoint.yaml")
    mcfg.base.aero.num_y = 11
    prob = build_multipoint_problem(mcfg, driver=True)
    prob.set_val("c", 0.32)
    prob.set_val("Kv", 950.0)
    prob.run_model()
    return mcfg, prob


def test_multipoint_m3_flies_the_m2_airframe(multipoint):
    _, p = multipoint
    for k in ["m_wing", "m_ht", "m_vt", "m_fus", "r_tube", "wall_ratio"]:
        assert p.get_val(f"points.M3.{k}")[0] == p.get_val(f"points.M2.{k}")[0], k
    # gear = f_gear m: M2's closure m equals its m_total to the solver tolerance, M3 takes m_total itself
    assert p.get_val("points.M3.m_gear")[0] == pytest.approx(p.get_val("points.M2.m_gear")[0], rel=1e-9)
    assert p.get_val("points.M3.W")[0] == pytest.approx(p.get_val("points.M3.m_total")[0] * 9.80665, rel=1e-14)
    assert p.get_val("points.M2.m_total")[0] > p.get_val("points.M3.m_total")[0]
    assert p.get_val("points.M2.m_pay")[0] - p.get_val("points.M3.m_pay")[0] == pytest.approx(
        p.get_val("fuel_mass")[0], rel=1e-12)


def test_multipoint_totals(multipoint):
    _, p = multipoint
    data = p.check_totals(of=["score", "points.M2.SM", "points.M3.energy_margin"], method="fd", step=1e-6,
                          form="central", out_stream=None)
    assert_check_totals(data, atol=1e-4, rtol=5e-3)


# --- regression: every pre-existing config is unchanged ---------------------------------------------------------------
@pytest.mark.parametrize("cfg_name", json.loads((ROOT / "tests" / "data" / "configs_reference.json").read_text()))
def test_existing_configs_unchanged(cfg_name):
    from .configs_snapshot import snapshot

    ref = json.loads((ROOT / "tests" / "data" / "configs_reference.json").read_text())[cfg_name]
    now = snapshot(cfg_name)
    for k, x in ref.items():
        assert now[k] == pytest.approx(x, rel=1e-9, abs=1e-12), k


