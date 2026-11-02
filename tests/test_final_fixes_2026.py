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


# --- D2: the rigid payload fuselage --------------------------------------------------------------------------------

def _store_cfg(**update):
    from aircrafter2.problems.multipoint import load_multipoint_config

    sc = load_multipoint_config(CFG26).mp.stores2026
    return sc.model_copy(update=update) if update else sc


def _stores(x_cb_frac, c=0.37008, payload=True):
    from aircrafter2.stores2026 import Stores2026

    from .conftest import run_component

    return run_component(Stores2026(config=_store_cfg(), payload=payload), n_p=42.0, n_c=14.0, S=0.56, c=c,
                         x_cb_frac=x_cb_frac)


def _packaging(**inputs):
    from aircrafter2.stores2026 import Packaging2026

    from .conftest import run_component

    x = dict(c=0.37008, x_cb_frac=0.416, x_batt=-0.35, l_ht=1.1, S_ht=0.14839, AR_ht=4.0)
    sc = _store_cfg()
    from aircrafter2.stores2026 import fuselage_length

    x["l_cb"] = float(fuselage_length(sc, np.array([42.0]), np.array([14.0]))[2][0])
    x.update(inputs)
    return run_component(Packaging2026(config=sc, x_motor_ref=-0.5334), **x), x, sc


def test_rigid_fuselage_zero_shift_at_humpback():
    """At Humpback's reference values every shift is exactly 0.0: x_pay and x_motor equal the old stations (==)."""
    sc = _store_cfg()
    p = _stores(0.416)
    c = 0.37008
    masses = [42 * sc.m_seat, 14 * sc.m_slot, 42 * sc.m_duck, 14 * sc.m_puck]
    stations = [sc.x_pax_frac * c, sc.x_car_frac * c] * 2
    old = sum(m * x for m, x in zip(masses, stations)) / sum(masses)
    assert _val(p, "c.x_pay") == old
    q, _, _ = _packaging()
    assert _val(q, "c.x_motor") == -0.5334


def test_rigid_fuselage_moves_rigidly():
    """x_cb_frac + 0.1: the payload stations and the motor move by 0.1 c."""
    c = 0.37008
    assert _val(_stores(0.516), "c.x_pay") - _val(_stores(0.416), "c.x_pay") == pytest.approx(0.1 * c, rel=1e-12)
    a, _, _ = _packaging()
    b, _, _ = _packaging(x_cb_frac=0.516)
    assert _val(b, "c.x_motor") - _val(a, "c.x_motor") == pytest.approx(0.1 * c, rel=1e-12)


def test_motor_moves_forward_with_the_box_length():
    """Growing the box by dl (more passengers) moves the motor forward by dl / 2."""
    a, x, _ = _packaging()
    b, _, _ = _packaging(l_cb=x["l_cb"] + 0.2)
    assert _val(a, "c.x_motor") - _val(b, "c.x_motor") == pytest.approx(0.1, rel=1e-12)


def test_packaging_margins_by_hand():
    q, x, sc = _packaging(x_batt=-0.30, x_cb_frac=0.45, l_ht=1.0)
    x_cb = 0.45 * x["c"]
    x_motor = -0.5334 + (x_cb - 0.416 * 0.37008) - (x["l_cb"] - x["l_cb"]) / 2
    aft = x_cb + x["l_cb"] / 2
    c_ht = (x["S_ht"] / x["AR_ht"]) ** 0.5
    assert _val(q, "c.x_motor") == pytest.approx(x_motor, rel=1e-14)
    assert _val(q, "c.batt_fwd_margin") == pytest.approx(-0.30 - (x_motor + sc.d_batt), rel=1e-12)
    assert _val(q, "c.batt_aft_margin") == pytest.approx(aft + 0.30, rel=1e-12)
    assert _val(q, "c.tail_margin") == pytest.approx(0.25 * x["c"] + 1.0 - 0.25 * c_ht - aft - sc.tail_clear,
                                                     rel=1e-12)


@pytest.mark.parametrize("counts", [None, [42.0, 14.0]])
def test_fixed_count_fuselage(counts):
    """fuselage_counts on: the fuselage keeps its built size whatever the flown load; off: sized by n_p, n_c."""
    from aircrafter2.stores2026 import PayloadFuselage, fuselage_length

    from .conftest import run_component

    sc = _store_cfg(fuselage_counts=counts)
    p = run_component(PayloadFuselage(config=sc), n_p=36.0, n_c=12.0)
    n = counts or [36.0, 12.0]
    assert _val(p, "c.l_cb") == float(fuselage_length(sc, np.array([n[0]]), np.array([n[1]]))[2][0])


def test_rigid_fuselage_partials():
    from .conftest import check_partials_fd

    q, _, _ = _packaging(x_cb_frac=0.3, x_batt=-0.5)
    check_partials_fd(q)
    check_partials_fd(_stores(0.3))


def test_rigid_fuselage_cg_shift_by_hand():
    """Humpback with x_cb_frac + 0.1: the box structure, the payload and the motor group move by 0.1 c, the rest stays;
    x_cg moves by 0.1 c (m_cb + m_pay + m_motor_group) / m_total (masses unchanged)."""
    from aircrafter2.problems.multipoint import build_multipoint_problem, load_multipoint_config

    out = []
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        for f in (0.416, 0.516):
            mcfg = load_multipoint_config(ROOT / "configs" / "humpback_2026.yaml")
            mcfg.base.initial["x_cb_frac"] = f
            p = build_multipoint_problem(mcfg, driver=False, mda_retry=True)
            p.run_model()
            out.append({k: _val(p, f"points.M2.{k}") for k in ("x_cg", "m_cb", "m_pay", "m_motor_group", "m_total",
                                                                "x_motor", "x_pay")})
    a, b = out
    for k in ("m_cb", "m_pay", "m_motor_group", "m_total"):
        assert a[k] == pytest.approx(b[k], rel=1e-12)
    dc = 0.1 * 0.37008
    assert b["x_motor"] - a["x_motor"] == pytest.approx(dc, rel=1e-9)
    assert b["x_pay"] - a["x_pay"] == pytest.approx(dc, rel=1e-9)
    assert b["x_cg"] - a["x_cg"] == pytest.approx(dc * (a["m_cb"] + a["m_pay"] + a["m_motor_group"]) / a["m_total"],
                                                  rel=1e-9)


def test_2025_stations_unchanged():
    """The 2024-25 multipoint problem has no packaging component and the same stations and score as before D2 (to the
    last bits: == at the reference's thread count, a580942 vs 0cfb746)."""
    from aircrafter2.problems.multipoint import build_multipoint_problem, load_multipoint_config

    ref = json.loads((DATA / "dbf2025_stations_reference.json").read_text())
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        p = build_multipoint_problem(load_multipoint_config(ROOT / "configs" / "dbf2025_multipoint.yaml"),
                                     driver=False)
        p.run_model()
    assert not hasattr(p.model.points.M2, "packaging")
    for k, x in ref.items():  # bit for bit at the reference's thread count (checked when made, OMP_NUM_THREADS=3);
        if not k.startswith("_"):  # another BLAS thread count reorders the VLM's sums: the last bit can move
            assert _val(p, k) == pytest.approx(x, rel=1e-12, abs=1e-15), k


# --- D3: the undercarriage and leakage drag areas ----------------------------------------------------------------

def test_drag_areas_at_humpback_by_hand():
    """Humpback (the ruleset's areas): CD_areas = (A_gear + A_lp) / S at every point, added to CD_extra_total; with the
    areas zeroed the component is absent and CD_extra_total = CD_extra + CD_stores, as before."""
    from aircrafter2.problems.multipoint import build_multipoint_problem, load_multipoint_config

    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        mcfg = load_multipoint_config(ROOT / "configs" / "humpback_2026.yaml")
        a = mcfg.base.aero
        assert (a.cd_area_gear, a.cd_area_lp) == (0.0028, 0.00224)
        p = build_multipoint_problem(mcfg, driver=False, mda_retry=True)
        p.run_model()
        for pt in ("M2", "M3"):
            S = _val(p, f"points.{pt}.S")
            assert _val(p, f"points.{pt}.CD_areas") == pytest.approx((0.0028 + 0.00224) / S, rel=1e-14)
            assert _val(p, f"points.{pt}.CD_extra_total") == pytest.approx(
                _val(p, f"points.{pt}.CD_extra") + _val(p, f"points.{pt}.CD_stores") + _val(p, f"points.{pt}.CD_areas"),
                rel=1e-14)
        mcfg.base.aero.cd_area_gear = mcfg.base.aero.cd_area_lp = 0.0
        q = build_multipoint_problem(mcfg, driver=False)
    assert not hasattr(q.model.points.M2.mda, "cd_areas")


def test_drag_areas_zero_by_default():
    """The code default is 0: no 2024-25 or single-point config gets the component (their graphs are unchanged; the
    2024-25 values are compared == in test_2025_stations_unchanged and the configs snapshot)."""
    from aircrafter2.problems.full_mdo import load_full_config
    from aircrafter2.problems.multipoint import load_multipoint_config

    for name in ("full_mdo.yaml", "full_mdo_twin.yaml"):
        a = load_full_config(ROOT / "configs" / name).aero
        assert a.cd_area_gear == 0.0 and a.cd_area_lp == 0.0
    a = load_multipoint_config(ROOT / "configs" / "dbf2025_multipoint.yaml").base.aero
    assert a.cd_area_gear == 0.0 and a.cd_area_lp == 0.0
