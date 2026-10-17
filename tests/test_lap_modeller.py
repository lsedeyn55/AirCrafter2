"""Lap modeller: reference port, MATLAB check, smooth twin, coupled model, propulsion power mode."""

import csv
from pathlib import Path

import numpy as np
import openmdao.api as om
import pytest
from openmdao.utils.assert_utils import assert_check_partials, assert_check_totals

from aircrafter2.aero.polar import ThreePointPolar, TurnAlpha
from aircrafter2.mission.lap_modeller import DEFAULT_ASSUMPTIONS as A
from aircrafter2.mission.lap_modeller import isa_density, performance_modeller
from aircrafter2.mission.smooth_lap import SmoothLapModel
from aircrafter2.problems.lap_mdo import build_lap_mdo_problem, load_lap_mdo_config

from .conftest import CONFIGS, check_partials_fd

MATLAB_CSV = Path(__file__).parent / "data" / "matlab_reference.csv"
GRID = [(5, 1.2, 6, 20), (7, 1.5, 9, 25), (4, 1.5, 6, 18), (6, 1.0, 5, 28), (5, 1.5, 8, 15), (3, 1.0, 5, 22),
        (6, 1.4, 7, 24), (3.5, 1.2, 6, 16)]


def design(m, b, AR, V, bf=1.0):
    return dict(m=m, b=b, AR=AR, V=V, altitude=400.0, takeoff_distance=100.0, mission_time=300.0,
                battery_fraction=bf)


# --- code check against the original MATLAB ---------------------------------------------------
@pytest.mark.skipif(not MATLAB_CSV.exists(), reason="run verification/matlab/run_reference.m in MATLAB first")
def test_port_reproduces_matlab():
    """Port (matlab_compat=True) vs performance_modeller.m run in MATLAB R2026a (round-off level)."""
    rows = list(csv.DictReader(open(MATLAB_CSV)))
    assert rows
    for r in rows:
        d = design(float(r["m"]), float(r["b"]), float(r["AR"]), float(r["V"]), float(r["batteryFraction"]))
        py = performance_modeller(A, d, matlab_compat=True)
        if r["NaNflag"] == "1":
            # MATLAB screens these out; the port reports them through its margins instead
            assert py["stall_margin"] <= 0 or py["turn_speed_margin"] < 0, r
            continue
        assert py["Vstall"] == pytest.approx(float(r["Vstall"]), abs=1e-12)
        assert py["lapCount"] == int(float(r["lapCount"]))
        for key in ["Lap1time", "otherLapTime", "ThreeLapTime", "Preq", "Tstatic"]:
            ref = float(r[key])
            if np.isnan(ref):  # MATLAB's unflagged out-of-table NaN (PLAN M7), reproduced in compat mode
                assert np.isnan(py[key]), (key, r)
            else:
                assert py[key] == pytest.approx(ref, rel=1e-6), (key, r)


# --- smooth twin vs reference port ------------------------------------------------------------
@pytest.fixture(scope="module")
def twin():
    p = om.Problem(reports=False)
    p.model.add_subsystem("lap", SmoothLapModel(), promotes=["*"])
    p.setup(force_alloc_complex=True)
    return p


def _set_static(p, m, b, AR, V):
    vals = dict(m=m, S=b * b / AR, V=V, rho=isa_density(400.0), CD0=A["Cd0"], CD1=0.0,
                k=1 / (np.pi * AR * A["e"]), CL_max=A["Clmax"], CL_to=A["Clto"], eta=A["eta"],
                n_struct=A["n"], mu_roll=A["mu"], E_avail=A["E"], g=9.81, takeoff_distance=100.0,
                mission_time=300.0)
    for k, v in vals.items():
        p.set_val(k, v)


@pytest.mark.parametrize("pt", GRID)
def test_twin_matches_reference(twin, pt):
    ref = performance_modeller(A, design(*pt))
    _set_static(twin, *pt)
    twin.run_model()
    g = lambda k: twin.get_val(k)[0]  # noqa: E731
    assert g("ThreeLapTime") == pytest.approx(ref["ThreeLapTime"], rel=0.01)
    assert g("Lap1time") == pytest.approx(ref["Lap1time"], rel=0.015)
    assert g("otherLapTime") == pytest.approx(ref["otherLapTime"], rel=0.015)
    assert g("Preq") == pytest.approx(ref["Preq"], rel=1e-12)
    assert g("Tstatic") == pytest.approx(ref["Tstatic"], rel=1e-12)
    assert g("stall_margin") == pytest.approx(ref["stall_margin"], rel=1e-12)
    assert g("lapCount") == ref["lapCount"]


def test_twin_partials(twin):
    """The twin's cs partials against central FD."""
    _set_static(twin, 5.0, 1.3, 7.0, 21.0)
    twin.run_model()
    data = twin.check_partials(method="fd", form="central", step=1e-6, out_stream=None, compact_print=True)
    for comp in data.values():
        for key in [k for k in comp if k[0] == "lapCount"]:
            del comp[key]
    assert_check_partials(data, atol=1e-5, rtol=1e-4)


def test_twin_is_smooth_in_speed(twin):
    """No steps or kinks in ThreeLapTime(V), unlike the table-based reference."""
    _set_static(twin, 5.0, 1.2, 6.0, 20.0)
    Vs = np.linspace(19.0, 21.0, 21)
    t = []
    for V in Vs:
        twin.set_val("V", V)
        twin.run_model()
        t.append(twin.get_val("ThreeLapTime")[0])
    d2 = np.diff(t, 2)
    assert np.max(np.abs(d2)) < 0.05 * np.max(np.abs(np.diff(t)))


# --- polar ------------------------------------------------------------------------------------
def test_three_point_polar_is_exact_and_differentiable():
    p = om.Problem(reports=False)
    p.model.add_subsystem("c", ThreePointPolar())
    p.setup(force_alloc_complex=True)
    CL, CD = [0.3, 0.8, 1.2], [0.022, 0.035, 0.07]
    for i in range(3):
        p.set_val(f"c.CL_{i}", CL[i])
        p.set_val(f"c.CD_{i}", CD[i])
    p.run_model()
    c0, c1, k = (p.get_val(f"c.{n}")[0] for n in ("CD0", "CD1", "k"))
    for cl, cd in zip(CL, CD):
        assert c0 + c1 * cl + k * cl**2 == pytest.approx(cd, rel=1e-12)
    assert_check_partials(p.check_partials(method="fd", form="central", out_stream=None), atol=1e-6, rtol=1e-5)


def test_turn_alpha_partials():
    p = om.Problem(reports=False)
    p.model.add_subsystem("c", TurnAlpha(frac=0.95))
    p.setup()
    p.run_model()
    assert_check_partials(p.check_partials(method="fd", form="central", out_stream=None), atol=1e-6, rtol=1e-5)


# --- coupled model ------------------------------------------------------------------------------
def test_coupled_lap_check_totals():
    cfg = load_lap_mdo_config(CONFIGS / "lap_mdo.yaml")
    cfg.aero.num_y = 11
    prob = build_lap_mdo_problem(cfg)
    prob.run_model()
    assert prob.get_val("k")[0] > 0  # convex polar at the start point
    data = prob.check_totals(method="fd", step=1e-6, form="central", out_stream=None)
    assert_check_totals(data, atol=1e-4, rtol=5e-3)


def test_turn_below_decel_table_keeps_speed():
    """PLAN M7: entering a turn below the deceleration table's range, the speed is unchanged
    (MATLAB gives NaN there; the old port clamped and sped the aircraft up)."""
    from aircrafter2.mission.lap_modeller import leg_calc_turn

    v_dec = np.array([25.0, 23.0, 21.0, 20.32])
    t_dec = np.array([0.0, 1.0, 2.0, 103.0])
    v_om, om = np.array([18.0, 27.5]), np.array([0.6, 0.6])
    v, t = leg_calc_turn(v_dec, om, t_dec, 20.0, np.pi, v_om)
    assert v == 20.0 and t == pytest.approx(np.pi / 0.6)
    v, _ = leg_calc_turn(v_dec, om, t_dec, 20.0, np.pi, v_om, matlab_compat=True)
    assert np.isnan(v)


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
    check_partials_fd(p, atol=20.0, rtol=5e-3, skip=("lapCount",))  # FD truncation on the v^3 coefficient
