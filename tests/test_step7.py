"""Step 7: propulsion (decision B), parasite drag, stability, lap propulsion mode, full model."""

import numpy as np
import openmdao.api as om
import pytest
from openmdao.utils.assert_utils import assert_check_partials, assert_check_totals

from aircrafter2.aero.parasite import ParasiteDrag, cf_turbulent
from aircrafter2.mission.smooth_lap import SmoothLapModel
from aircrafter2.problems.full_mdo import build_full_problem, load_full_config
from aircrafter2.propulsion.apc import DEFAULT_DIR, fit_prop, parse_apc_file
from aircrafter2.propulsion.powertrain import RPM2RADS, Powertrain
from aircrafter2.stability import StaticStability

from .conftest import CONFIGS


def _prob(comp, **inputs):
    p = om.Problem(reports=False)
    p.model.add_subsystem("c", comp)
    p.setup(force_alloc_complex=True)
    for k, v in inputs.items():
        p.set_val(f"c.{k}", v)
    p.run_model()
    return p


def _check_fd(p, atol=1e-5, rtol=1e-4, skip=()):
    data = p.check_partials(method="fd", form="central", step=1e-6, step_calc="rel_avg", out_stream=None,
                            compact_print=True)
    for comp in data.values():
        for key in [k for k in comp if k[0] in skip]:
            del comp[key]
    assert_check_partials(data, atol=atol, rtol=rtol)


# --- APC data ---------------------------------------------------------------------------------
@pytest.mark.parametrize("name,d_in,p_in", [("11x7E", 11.0, 7.0), ("105x45", 10.5, 4.5), ("9x45E", 9.0, 4.5)])
def test_apc_parse_and_fit(name, d_in, p_in):
    data = parse_apc_file(DEFAULT_DIR / f"PER3_{name}.dat")
    assert data.diameter == pytest.approx(d_in * 0.0254) and data.pitch == pytest.approx(p_in * 0.0254)
    s = fit_prop(data)
    k = data.Ct >= 0
    assert s.rms_ct < 0.002 and s.rms_cp < 0.003  # vs Ct ~ 0.1, Cp ~ 0.05
    i = np.argmin(np.abs(data.rpm - 8000) + np.abs(data.J - 0.4) * 1e4)
    assert s.ct(data.J[i], data.rpm[i]) == pytest.approx(data.Ct[i], abs=0.003)


# --- powertrain ---------------------------------------------------------------------------------
def test_powertrain_operating_point_balances_torque():
    pt = Powertrain(prop="11x7E")
    p = _prob(pt)
    x = {k: p.get_val(f"c.{k}") for k in ["throttle", "rho", "n_cells", "V_cell", "R_cell", "Kv", "Rm", "I0"]}
    r = pt.solve_nodes(x)
    kv = x["Kv"] * RPM2RADS
    Qm = (r["I_motor"] - x["I0"]) / kv
    n = r["omega"] / (2 * np.pi)
    Qp = pt._prop.cp(r["J"], r["omega"] / RPM2RADS) * x["rho"] * n**2 * pt._prop.diameter**5 / (2 * np.pi)
    assert np.max(np.abs(Qm - Qp)) < 1e-10
    assert p.get_val("c.fit_residual_T")[0] < 0.1  # N, cubic fit of T(v) through the nodes
    assert np.all(np.diff(p.get_val("c.T_nodes")) < 0)  # thrust falls with speed


def test_powertrain_partials():
    _check_fd(_prob(Powertrain(prop="11x7E"), capacity=2.5, throttle=0.9), atol=1e-4, rtol=1e-3,
              skip=("rpm_limit", "fit_residual_T"))


# --- parasite drag ------------------------------------------------------------------------------
def test_parasite_hand_values_and_partials():
    p = _prob(ParasiteDrag(), v=20.0, rho=1.2, mu=1.8e-5, S=0.4, S_ht=0.06, AR_ht=4.0, tc_tail=0.09, xt_tail=0.3)
    c = np.sqrt(0.06 / 4.0)
    cf = 0.455 / np.log10(1.2 * 20 * c / 1.8e-5) ** 2.58
    ff = 1 + 0.6 / 0.3 * 0.09 + 100 * 0.09**4
    assert p.get_val("c.CD_ht")[0] == pytest.approx(cf * ff * 0.06 * (1.977 + 0.52 * 0.09) / 0.4, rel=1e-12)
    assert cf_turbulent(1e6) == pytest.approx(0.455 / 6**2.58)
    _check_fd(p)
    q = _prob(ParasiteDrag(fuselage_drag=False))
    assert q.get_val("c.CD_fus")[0] == 0.0


# --- stability ----------------------------------------------------------------------------------
def test_stability_hand_values_and_partials():
    p = _prob(StaticStability(), c=0.25, S=0.375, AR=6.0, a_w=4.5, S_ht=0.06, AR_ht=4.0, l_ht=0.8, eta_h=0.9)
    V_h = 0.06 * 0.8 / (0.375 * 0.25)
    a_h = 2 * np.pi * 4 / (2 + np.sqrt(20))
    x_np = 0.25 * 0.25 + 0.9 * V_h * a_h / 4.5 * (1 - 2 * 4.5 / (np.pi * 6)) * 0.25
    assert p.get_val("c.x_np")[0] == pytest.approx(x_np, rel=1e-12)
    assert p.get_val("c.x_cg")[0] == pytest.approx(0.1, rel=1e-12)  # all stations default to 0.1 m
    _check_fd(p)


# --- lap model, propulsion mode ---------------------------------------------------------------------
def _lap(power_mode="propulsion", **vals):
    p = om.Problem(reports=False)
    p.model.add_subsystem("lap", SmoothLapModel(power_mode=power_mode), promotes=["*"])
    p.setup(force_alloc_complex=True)
    base = dict(m=3.0, S=0.3, V=25.0, rho=1.18, CD0=0.025, CD1=0.0, k=0.06, CL_max=1.3, CL_to=1.3,
                E_avail=1.2e5, g=9.81, T_coeffs=np.array([20.0, -0.3, -0.004, 0.0]),
                P_coeffs=np.array([400.0, 0.0, 0.0, 0.0]))
    base.update(vals)
    for k, v in base.items():
        if k in ("T_coeffs", "P_coeffs") and power_mode != "propulsion":
            continue
        p.set_val(k, v)
    p.run_model()
    return p


def test_lap_energy_with_constant_battery_power():
    p = _lap()
    t3 = p.get_val("ThreeLapTime")[0]
    assert p.get_val("energy_3laps")[0] == pytest.approx(400.0 * (t3 + 3 * 3.0), rel=1e-9)
    assert p.get_val("thrust_margin_static")[0] == pytest.approx(20.0 - p.get_val("Tstatic")[0], rel=1e-12)


def test_headwind_slows_the_lap():
    calm = _lap().get_val("ThreeLapTime")[0]
    windy = _lap(wind=4.0).get_val("ThreeLapTime")[0]
    assert windy > calm
    assert _lap(wind=0.0).get_val("ThreeLapTime")[0] == pytest.approx(calm, rel=1e-14)


def test_lap_propulsion_partials():
    p = _lap(wind=2.0)
    _check_fd(p, atol=20.0, rtol=5e-3, skip=("lapCount",))  # FD truncation on the v^3 coefficient


# --- full model -------------------------------------------------------------------------------------
@pytest.fixture(scope="module")
def full_coarse():
    cfg = load_full_config(CONFIGS / "full_mdo.yaml")
    cfg.aero.num_y = 11
    prob = build_full_problem(cfg)
    prob.run_model()
    return prob


def test_full_model_balances(full_coarse):
    p = full_coarse
    assert p.get_val("cruise.L")[0] == pytest.approx(p.get_val("W")[0], rel=1e-8)
    assert p.get_val("thrust_at_v.y")[0] == pytest.approx(p.get_val("cruise.D")[0], rel=1e-8)
    assert p.get_val("m_sys")[0] == pytest.approx(
        sum(p.get_val(k)[0] for k in ["m_batt", "m_motor", "m_esc", "m_prop", "m_avionics"]), rel=1e-12)


def test_full_model_check_totals(full_coarse):
    data = full_coarse.check_totals(method="fd", step=1e-6, form="central", out_stream=None)
    assert_check_totals(data, atol=1e-4, rtol=5e-3)
