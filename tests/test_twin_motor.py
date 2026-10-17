"""Twin-motor configurations: propulsion scaling, shared-pack topology, layout validation, plumbing."""

import numpy as np
import openmdao.api as om
import pytest
from openmdao.utils.assert_utils import assert_check_partials
from pydantic import ValidationError
from scipy.optimize import brentq

from aircrafter2.problems.full_mdo import (WOOMBAH_Y_M, WOOMBAH_Y_MOTOR, PropulsionInputs, build_full_problem,
                                           check_fixed_limits, load_full_config)
from aircrafter2.propulsion.powertrain import RPM2RADS, Powertrain, motor_stations

from .conftest import CONFIGS

INPUTS = dict(capacity=2.5, throttle=0.9, n_cells=4.0, m_nacelle=0.02)


def _pt(n_motors=1, topology="per_motor", **vals):
    p = om.Problem(reports=False)
    p.model.add_subsystem("c", Powertrain(prop="11x7E", n_motors=n_motors, battery_topology=topology))
    p.setup(force_alloc_complex=True)
    for k, v in {**INPUTS, **vals}.items():
        p.set_val(f"c.{k}", v)
    p.run_model()
    return p


def g(p, k):
    return p.get_val(f"c.{k}")


# --- layouts -----------------------------------------------------------------------------------------
def test_motor_stations_and_rejections():
    assert motor_stations(1) == [0.0]
    assert motor_stations(2, 0.3) == [-0.3, 0.3]
    for bad in (0, 3, 4):
        with pytest.raises(ValueError, match="not supported"):
            motor_stations(bad, 0.3)
    with pytest.raises(ValueError, match="y_motor"):
        motor_stations(2, None)
    with pytest.raises(ValidationError):
        PropulsionInputs(n_motors=3)
    with pytest.raises(ValidationError, match="y_motor"):
        PropulsionInputs(n_motors=2)
    with pytest.raises(ValidationError, match="exactly one"):
        PropulsionInputs(n_motors=2, y_motor=0.3, y_motor_m=0.25)
    assert PropulsionInputs(n_motors=2, y_motor_m=0.25).station_mode == "absolute"
    assert PropulsionInputs(n_motors=2, y_motor=0.3).station_mode == "fraction"
    with pytest.raises(ValidationError):
        PropulsionInputs(battery_topology="series")
    with pytest.raises(ValueError, match="not supported"):
        p = om.Problem(reports=False)
        p.model.add_subsystem("c", Powertrain(n_motors=3))
        p.setup()


def test_woombah_station():
    assert WOOMBAH_Y_MOTOR == pytest.approx(9.9 / 35.45)


# --- per_motor: exact n-fold scaling ------------------------------------------------------------------
def test_per_motor_twin_is_exactly_twice_single():
    one, two = _pt(1), _pt(2)
    for k in ["T_coeffs", "P_coeffs", "T_static", "T_nodes", "P_nodes", "E_nom_total", "E_usable", "m_batt",
              "m_motor_group"]:
        np.testing.assert_array_equal(g(two, k), 2.0 * g(one, k), err_msg=k)
    for k in ["RPM_coeffs", "rpm_max", "I_static", "I_pack_static", "T_static_unit", "eta_prop_nodes",
              "m_motor_unit", "D_prop"]:
        np.testing.assert_array_equal(g(two, k), g(one, k), err_msg=k)
    np.testing.assert_array_equal(g(one, "T_static_unit"), g(one, "T_static"))
    m_unit = 0.15 + 0.06 + 0.03 + 0.02
    assert g(two, "m_motor_unit")[0] == pytest.approx(m_unit, rel=1e-15)
    assert g(two, "m_sys")[0] == pytest.approx(2 * m_unit + g(two, "m_batt")[0] + 0.25, rel=1e-15)
    E_pack = 4.0 * 3.7 * 2.5  # Wh, one pack
    assert g(two, "E_nom_total")[0] == pytest.approx(2 * E_pack, rel=1e-15)


def test_single_motor_unchanged_by_new_defaults():
    """n = 1 with the new defaults (m_nacelle = 0) reproduces the original totals."""
    p = _pt(1, m_nacelle=0.0)
    assert g(p, "m_sys")[0] == pytest.approx(g(p, "m_batt")[0] + 0.15 + 0.06 + 0.03 + 0.25, rel=1e-15)
    np.testing.assert_array_equal(g(p, "I_pack_static"), g(p, "I_static"))


# --- shared pack ---------------------------------------------------------------------------------------
def test_shared_pack_lower_per_unit_thrust():
    per, sh = _pt(2, "per_motor"), _pt(2, "shared")
    assert np.all(g(sh, "T_nodes") < g(per, "T_nodes"))
    assert g(sh, "T_static_unit")[0] < g(per, "T_static_unit")[0]
    # one pack instead of two: half the energy and battery mass at the same per-pack capacity
    assert g(sh, "E_nom_total")[0] == pytest.approx(0.5 * g(per, "E_nom_total")[0], rel=1e-15)
    assert g(sh, "m_batt")[0] == pytest.approx(0.5 * g(per, "m_batt")[0], rel=1e-15)
    assert g(sh, "I_pack_static")[0] == pytest.approx(2.0 * g(sh, "I_static")[0], rel=1e-15)


def test_shared_pack_matches_hand_calculation_at_static_node():
    """Independent solve of the shared-pack static point (v = 0) from the derived equation
        I = (delta V_oc - omega/K_v) / (R_m + n delta^2 R_b),  (I - I_0)/K_v = C_P rho n_r^2 D^5 / (2 pi)."""
    n, dl, ncell, Vc, Rc, Kv, Rm, I0, rho = 2, 0.9, 4.0, 3.7, 0.005, 900.0, 0.05, 1.0, 1.18
    p = _pt(n, "shared", rho=rho)
    prop = p.model.c._prop
    Voc, Rb, kv = ncell * Vc, ncell * Rc, Kv * RPM2RADS

    def current(w):
        return (dl * Voc - w / kv) / (Rm + n * dl**2 * Rb)

    def torque_gap(w):
        nr = w / (2 * np.pi)
        return (current(w) - I0) / kv - prop.cp(0.0, w / RPM2RADS) * rho * nr**2 * prop.diameter**5 / (2 * np.pi)

    w = brentq(torque_gap, 100.0, kv * dl * Voc * 0.999, xtol=1e-12)
    I = current(w)
    nr = w / (2 * np.pi)
    T_unit = prop.ct(0.0, w / RPM2RADS) * rho * nr**2 * prop.diameter**4
    assert g(p, "T_static_unit")[0] == pytest.approx(T_unit, rel=1e-9)
    assert g(p, "T_static")[0] == pytest.approx(n * T_unit, rel=1e-9)
    assert g(p, "I_static")[0] == pytest.approx(dl * I, rel=1e-9)
    assert g(p, "I_pack_static")[0] == pytest.approx(n * dl * I, rel=1e-9)
    assert g(p, "P_nodes")[0] == pytest.approx(Voc * n * dl * I, rel=1e-9)  # total chemistry power
    # the pack terminal voltage is the same for both ESCs
    V_term = Voc - Rb * n * dl * I
    assert dl * V_term == pytest.approx(w / kv + I * Rm, rel=1e-9)


@pytest.mark.parametrize("n,topology", [(2, "per_motor"), (2, "shared")])
def test_twin_powertrain_partials(n, topology):
    p = _pt(n, topology)
    data = p.check_partials(method="fd", form="central", step=1e-6, step_calc="rel_avg", out_stream=None,
                            compact_print=True)
    for comp in data.values():
        for key in [k for k in comp if k[0] in ("rpm_limit", "fit_residual_T", "D_prop")]:
            del comp[key]
    assert_check_partials(data, atol=1e-4, rtol=1e-3)


# --- full-model plumbing ---------------------------------------------------------------------------------
def test_twin_config_loads_and_is_step7_plus_layout():
    one = load_full_config(CONFIGS / "full_mdo.yaml")
    two = load_full_config(CONFIGS / "full_mdo_twin.yaml")
    assert (one.propulsion.n_motors, two.propulsion.n_motors) == (1, 2)
    assert two.propulsion.y_motor_m == pytest.approx(WOOMBAH_Y_M, abs=1e-4)
    d1, d2 = one.model_dump(), two.model_dump()
    for d in (d1, d2):
        for k in ("n_motors", "y_motor", "y_motor_m", "m_nacelle", "battery_topology", "c_clear", "l_nac", "d_nac",
                  "Q_nac", "nacelle_ff"):
            d["propulsion"].pop(k)
        d["stability"].pop("x_motor")
        for k in ("motor_inertia_relief", "nacelle_torsion"):
            d["mass"]["options"].pop(k)
    assert d1 == d2  # identical apart from the layout fields


def test_full_twin_model_runs_with_two_units():
    cfg = load_full_config(CONFIGS / "full_mdo_twin.yaml")
    cfg.aero.num_y = 11
    prob = build_full_problem(cfg, driver=False)
    prob.run_model()
    v = lambda k: prob.get_val(k)[0]  # noqa: E731
    assert v("T_static") == pytest.approx(2 * v("T_static_unit"), rel=1e-14)
    assert v("m_motor_group") == pytest.approx(2 * v("m_motor_unit"), rel=1e-14)
    assert v("m_sys") == pytest.approx(v("m_motor_group") + v("m_batt") + v("m_avionics"), rel=1e-12)
    assert v("E_nom_total") == pytest.approx(2 * 4 * 3.7 * v("capacity"), rel=1e-12)
    assert v("cruise.L") == pytest.approx(v("W"), rel=1e-8)
    assert "I_pack_margin_A" not in check_fixed_limits(prob)


def test_cli_overrides_layout(monkeypatch):
    import aircrafter2.api as api
    from aircrafter2.cli import main

    seen = {}

    class Res:
        success, iterations, values = True, 0, {"score": 0.0}

    def fake_run(cfg, **kw):
        seen["cfg"] = cfg
        return Res()

    monkeypatch.setattr(api, "run_design", fake_run)
    main(["run", str(CONFIGS / "full_mdo.yaml"), "--analyse", "--n-motors", "2", "--battery-topology", "shared",
          "--out", "/dev/null/x"])
    pr = seen["cfg"].propulsion
    assert (pr.n_motors, pr.battery_topology) == (2, "shared")
    assert pr.y_motor_m == pytest.approx(WOOMBAH_Y_M) and pr.y_motor is None


# --- checkpoint 2: wing loads with wing-mounted units (G&G Eqs. 44-46, 48-50) ---------------------------
from aircrafter2.aero.parasite import ParasiteDrag, cf_turbulent  # noqa: E402
from aircrafter2.geometry import MotorPlacement  # noqa: E402
from aircrafter2.mass.garcia_gamboa.loads import WingLoads  # noqa: E402
from aircrafter2.stability import oei_check  # noqa: E402

LOADS = dict(m=3.0, g=9.80665, n=5.0, b=1.3, c_mean=0.2, rho=1.18, V_design=30.0, Cm=-0.08, x_gd=0.3,
             m_motor_unit=0.26, y_m=0.25, x_motor=-0.14)


def _loads(**opts):
    p = om.Problem(reports=False)
    p.model.add_subsystem("c", WingLoads(**opts))
    p.setup(force_alloc_complex=True)
    for k, val in LOADS.items():
        if k in p.model.c._in_names:
            p.set_val(f"c.{k}", val)
    p.run_model()
    return p


@pytest.mark.parametrize("relief", [False, True])
@pytest.mark.parametrize("torsion", [False, True])
def test_wing_loads_hand_values(relief, torsion):
    base = _loads()
    p = _loads(n_motor_pairs=1, motor_inertia_relief=relief, nacelle_torsion=torsion)
    L = LOADS
    W, gm = L["m"] * L["g"], L["g"] * L["m_motor_unit"]
    arm = 0.25 * L["c_mean"] - L["x_motor"]
    exp = {k: g(base, k)[0] for k in ["Q_fl", "M_fl", "T_fl", "Q_gd", "M_gd", "T_gd"]}
    if relief:
        exp["Q_fl"] -= L["n"] * gm
        exp["M_fl"] -= L["n"] * gm * L["y_m"]
        exp["Q_gd"] -= gm
        exp["M_gd"] -= gm * L["y_m"]
    if torsion:
        exp["T_fl"] += L["n"] * gm * arm
        exp["T_gd"] = abs(0.5 * W * L["c_mean"] * (0.25 - L["x_gd"]) - gm * arm)
    for k, val in exp.items():
        assert g(p, k)[0] == pytest.approx(val, rel=1e-9), k
    # published relations still hold without units (Eqs. 44, 45, 48, 49)
    assert g(base, "M_fl")[0] == pytest.approx(L["n"] * W * L["b"] / 8, rel=1e-14)
    assert g(base, "M_gd")[0] == pytest.approx(W * L["b"] / 4, rel=1e-14)


def test_wing_loads_ground_torsion_partly_cancels_with_forward_hold_point():
    """x_gd < 1/4: the tip reaction is nose-up and the nacelle nose-down, so they subtract."""
    p = _loads(n_motor_pairs=1)
    p.set_val("c.x_gd", 0.1)
    p.run_model()
    W, gm = LOADS["m"] * LOADS["g"], LOADS["g"] * LOADS["m_motor_unit"]
    arm = 0.25 * LOADS["c_mean"] - LOADS["x_motor"]
    assert g(p, "T_gd")[0] == pytest.approx(abs(0.5 * W * 0.2 * 0.15 - gm * arm), rel=1e-9)


@pytest.mark.parametrize("relief,torsion", [(False, True), (True, False), (True, True)])
def test_wing_loads_analytic_partials(relief, torsion):
    p = _loads(n_motor_pairs=1, motor_inertia_relief=relief, nacelle_torsion=torsion)
    assert_check_partials(p.check_partials(method="cs", out_stream=None), atol=1e-10, rtol=1e-10)


def test_wing_loads_default_has_no_new_inputs():
    names = _loads().model.c._in_names
    assert not {"m_motor_unit", "y_m", "x_motor"} & set(names)


# --- placement --------------------------------------------------------------------------------------------
@pytest.mark.parametrize("mode", ["absolute", "fraction"])
def test_motor_placement(mode):
    p = om.Problem(reports=False)
    p.model.add_subsystem("c", MotorPlacement(station_mode=mode))
    p.setup(force_alloc_complex=True)
    b, D, w, cc = 1.3, 0.254, 0.12, 0.025
    for k, val in dict(b=b, D_prop=D, w_cb=w, c_clear=cc).items():
        p.set_val(f"c.{k}", val)
    if mode == "fraction":
        p.set_val("c.y_motor", 0.4)
        ym = 0.4 * b / 2
    else:
        p.set_val("c.y_m", 0.2515)
        ym = 0.2515
    p.run_model()
    assert g(p, "fus_clear_margin")[0] == pytest.approx(ym - D / 2 - w / 2 - cc, rel=1e-14)
    assert g(p, "tip_clear_margin")[0] == pytest.approx(b / 2 - ym - D / 2, rel=1e-14)
    assert g(p, "prop_clear_margin")[0] == pytest.approx(2 * ym - D - cc, rel=1e-14)
    assert_check_partials(p.check_partials(method="cs", out_stream=None), atol=1e-12, rtol=1e-12)


# --- nacelle drag -----------------------------------------------------------------------------------------
@pytest.mark.parametrize("form", ["body", "nacelle"])
def test_nacelle_drag_hand_value_and_partials(form):
    p = om.Problem(reports=False)
    p.model.add_subsystem("c", ParasiteDrag(n_nacelles=2, nacelle_ff=form))
    p.setup(force_alloc_complex=True)
    p.set_val("c.v", 35.0)
    p.run_model()
    f = 0.15 / 0.05
    FF = 1 + 60 / f**3 + f / 400 if form == "body" else 1 + 0.35 / f
    cd = 2 * cf_turbulent(1.18 * 35.0 * 0.15 / 1.8e-5) * FF * 1.3 * np.pi * 0.05 * 0.15 / 0.375
    assert g(p, "CD_nac")[0] == pytest.approx(cd, rel=1e-12)
    assert g(p, "CD_extra")[0] == pytest.approx(g(p, "CD_tail")[0] + g(p, "CD_fus")[0] + cd, rel=1e-12)
    data = p.check_partials(method="fd", form="central", step=1e-6, step_calc="rel_avg", out_stream=None)
    assert_check_partials(data, atol=1e-6, rtol=1e-4)


def test_no_nacelle_drag_for_nose_layout():
    p = om.Problem(reports=False)
    p.model.add_subsystem("c", ParasiteDrag())
    p.setup()
    p.run_model()
    assert g(p, "CD_nac")[0] == 0.0


# --- full twin model ---------------------------------------------------------------------------------------
@pytest.fixture(scope="module")
def twin_coarse():
    cfg = load_full_config(CONFIGS / "full_mdo_twin.yaml")
    cfg.aero.num_y = 11
    cfg.mass.options.motor_inertia_relief = True  # exercise every new term
    cfg.propulsion.m_nacelle = 0.03
    prob = build_full_problem(cfg)
    prob.run_model()
    return cfg, prob


def test_twin_model_wires_loads_drag_and_cg(twin_coarse):
    cfg, p = twin_coarse
    v = lambda k: float(p.get_val(k)[0])  # noqa: E731
    assert v("y_m") == pytest.approx(cfg.propulsion.y_motor_m)
    assert v("CD_nac") > 0 and v("CD_extra") == pytest.approx(v("CD_tail") + v("CD_fus") + v("CD_nac"), rel=1e-12)
    W, gmu = v("W"), v("g") * v("m_motor_unit")
    assert v("M_fl") == pytest.approx(v("n") * W * v("b") / 8 - v("n") * gmu * v("y_m"), rel=1e-10)
    assert v("stability.m_motor_group") == pytest.approx(2 * v("m_motor_unit"), rel=1e-14)
    oei = oei_check(p, cfg)
    q = 0.5 * v("rho") * (1.2 * np.sqrt(2 * W / (v("rho") * v("S") * v("cruise.CL_max")))) ** 2
    a_v = 2 * np.pi * 2.0 / (2 + np.sqrt(8.0))
    d_req = v("T_static_unit") * v("y_m") / (q * v("S") * v("b")) / (0.9 * v("V_v") * a_v * 0.5)
    assert oei["rudder_req_deg"] == pytest.approx(np.degrees(d_req), rel=1e-12)
    assert oei_check(p, load_full_config(CONFIGS / "full_mdo.yaml")) is None


def test_twin_model_check_totals(twin_coarse):
    from openmdao.utils.assert_utils import assert_check_totals

    _, p = twin_coarse
    data = p.check_totals(method="fd", step=1e-6, form="central", out_stream=None)
    assert_check_totals(data, atol=1e-4, rtol=5e-3)
