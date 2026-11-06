"""Take-off in the mission time and the editable course (docs/development/TAKEOFF_COURSE_PLAN.md).

Off = unchanged is also covered bit for bit by the regression tests against tests/data/configs_reference.json,
rdo_off_reference.json, kv_dv_reference.json and multipoint_nominal_reference.json (none regenerated for the take-off
work; configs_reference and kv_dv_reference were regenerated later for the placeholders migration).
"""

import warnings

import numpy as np
import openmdao.api as om
import pytest
from openmdao.core.analysis_error import AnalysisError
from scipy.integrate import solve_ivp

from aircrafter2.mission.course import (CourseConfig, Leg, check_course, lap_legs, load_preset, save_preset,
                                        SHIPPED_PRESETS)
from aircrafter2.mission.smooth_lap import FT2M, LAP1, TAKEOFF_OUTPUTS, SmoothLapModel

from .conftest import check_partials_fd

BASE = dict(m=3.0, S=0.3, V=25.0, rho=1.18, CD0=0.025, CD1=0.0, k=0.06, CL_max=1.3, CL_to=1.3, E_avail=1.2e5,
            g=9.81, T_coeffs=np.array([20.0, -0.3, -0.004, 0.0]), P_coeffs=np.array([400.0, 0.0, 0.0, 0.0]))
LEGACY = [("s", 500), ("t", 180), ("t", 360), ("s", 1000), ("t", 180), ("s", 500)]  # the old position-based rule


def _lap(opts=None, **vals):
    p = om.Problem(reports=False)
    p.model.add_subsystem("lap", SmoothLapModel(power_mode="propulsion", **(opts or {})), promotes=["*"])
    p.setup(force_alloc_complex=True)
    for k, v in {**BASE, **vals}.items():
        p.set_val(k, v)
    p.run_model()
    return p


def _v(p, k):
    return float(p.get_val(k)[0])


OUTS = ["Lap1time", "otherLapTime", "ThreeLapTime", "laps_frac", "energy_3laps", "turn_speed_margin",
        "thrust_margin_static"]


# --- off = the original model ---------------------------------------------------------------------------------------
@pytest.mark.parametrize("wind", [0.0, 3.0])
def test_explicit_wind_directions_equal_the_old_alternation_bit_for_bit(wind):
    old = _lap(dict(lap1=LEGACY, lap_other=LEGACY), wind=wind)
    new = _lap(wind=wind)  # default legs: Table 3.5 with explicit head / tail
    preset = CourseConfig().lap_options()
    cfg = _lap(dict(lap1=preset[0], lap_other=preset[1]), wind=wind)
    for k in OUTS:
        assert _v(new, k) == _v(old, k) == _v(cfg, k)
    for k, _ in TAKEOFF_OUTPUTS:
        assert _v(new, k) == 0.0


def test_default_preset_is_table_3_5():
    lap1, other = load_preset("dbf_2025")
    assert lap_legs(other) == [(k, float(s), w) for k, s, w in LAP1] and lap1 == other


# --- the ground roll against closed forms --------------------------------------------------------------------------
@pytest.mark.parametrize("wind", [0.0, 3.0])
def test_ground_roll_closed_form_constant_thrust_no_drag_no_lift(wind):
    """T constant, D = L = 0: a = (T - mu W) / m, so t = m (V_LOF - w) / (T - mu W) and s = m (V_LOF - w)^2 /
    (2 (T - mu W)) (ground distance, starting at airspeed w), E = P t."""
    T, P, m, mu, g = 20.0, 400.0, 3.0, 0.02, 9.81
    p = _lap(dict(takeoff=True), CD0=0.0, k=0.0, CL_ground=0.0, mu_roll=mu, T_coeffs=np.array([T, 0.0, 0.0, 0.0]),
             P_coeffs=np.array([P, 0.0, 0.0, 0.0]), wind=wind, m=m, g=g)
    vlof = _v(p, "V_lof")
    assert vlof == pytest.approx(1.1 * _v(p, "Vstall"), rel=1e-14)
    F = T - mu * m * g
    assert _v(p, "t_takeoff") == pytest.approx(m * (vlof - wind) / F, rel=1e-10)
    assert _v(p, "s_ground_roll") == pytest.approx(m * (vlof - wind) ** 2 / (2.0 * F), rel=1e-10)
    assert _v(p, "E_takeoff") == pytest.approx(P * m * (vlof - wind) / F, rel=1e-10)
    assert _v(p, "takeoff_accel_margin") == pytest.approx(F / m, rel=1e-12)
    assert _v(p, "takeoff_straight_margin") == pytest.approx(500 * FT2M - _v(p, "s_ground_roll"), rel=1e-12)


ROLL_CASES = [  # m, S, CL_max, CD0, k, CL_ground, mu, T_coeffs, P_coeffs, wind
    (3.0, 0.30, 1.3, 0.025, 0.06, 0.5, 0.02, [20.0, -0.3, -0.004, 0.0], [400.0, 2.0, 0.0, 0.0], 0.0),
    (6.0, 0.45, 1.4, 0.035, 0.05, 0.8, 0.04, [45.0, -0.9, 0.006, -1e-4], [1200.0, -5.0, 0.3, 0.0], 0.0),
    (10.0, 0.55, 1.5, 0.040, 0.07, 0.3, 0.02, [60.0, -1.1, -0.01, 0.0], [1500.0, 10.0, 0.0, 0.0], 0.0),
    (4.5, 0.35, 1.2, 0.030, 0.06, 0.0, 0.03, [25.0, -0.2, -0.02, 2e-4], [700.0, 0.0, 0.5, 0.0], 0.0),
    (6.0, 0.45, 1.4, 0.035, 0.05, 0.8, 0.04, [45.0, -0.9, 0.006, -1e-4], [1200.0, -5.0, 0.3, 0.0], 4.0),
    (3.0, 0.30, 1.3, 0.025, 0.06, 0.5, 0.02, [20.0, -0.3, -0.004, 0.0], [400.0, 2.0, 0.0, 0.0], 2.5),
]


@pytest.mark.parametrize("case", ROLL_CASES, ids=[f"case{i}" for i in range(len(ROLL_CASES))])
def test_ground_roll_against_solve_ivp_in_time(case):
    """The full T(v), D(v), mu (W - L(v)) ground roll integrated in time with an event at V_LOF (independent of the
    airspeed formulation): < 0.1 % on time, distance and energy."""
    m, S, CLmax, CD0, k, CLg, mu, tc, pc, w = case
    rho, g = 1.18, 9.81
    p = _lap(dict(takeoff=True), m=m, S=S, CL_max=CLmax, CL_to=CLmax, CD0=CD0, k=k, CL_ground=CLg, mu_roll=mu,
             T_coeffs=np.array(tc), P_coeffs=np.array(pc), wind=w, rho=rho, g=g)
    vlof = 1.1 * np.sqrt(m * g / (0.5 * rho * CLmax * S))
    q = lambda v: 0.5 * rho * v**2 * S  # noqa: E731

    def rhs(_, y):
        v = y[0]
        a = (np.polyval(tc[::-1], v) - q(v) * (CD0 + k * CLg**2) - mu * (m * g - q(v) * CLg)) / m
        return [a, v - w, np.polyval(pc[::-1], v)]

    event = lambda _, y: y[0] - vlof  # noqa: E731
    event.terminal = True
    sol = solve_ivp(rhs, (0.0, 200.0), [w, 0.0, 0.0], events=event, rtol=1e-10, atol=1e-12)
    t_ref, (_, s_ref, e_ref) = sol.t_events[0][0], sol.y_events[0][0]
    assert _v(p, "t_takeoff") == pytest.approx(t_ref, rel=1e-3)
    assert _v(p, "s_ground_roll") == pytest.approx(s_ref, rel=1e-3)
    assert _v(p, "E_takeoff") == pytest.approx(e_ref, rel=1e-3)


def test_takeoff_adds_the_roll_to_lap_1_and_the_window():
    off, on = _lap(), _lap(dict(takeoff=True), CL_ground=0.5)
    assert _v(on, "t_takeoff") > 0.0
    assert _v(on, "Lap1time") > _v(off, "Lap1time")
    assert _v(on, "otherLapTime") == pytest.approx(_v(off, "otherLapTime"), rel=1e-6)  # lap 2+ from a similar speed
    assert _v(on, "laps_frac") < _v(off, "laps_frac")
    assert _v(on, "energy_3laps") > _v(off, "energy_3laps")
    rows = on.model.lap.leg_breakdown()
    roll = [r for r in rows if r["kind"] == "ground roll"]
    assert len(roll) == 1 and roll[0]["lap"] == 1 and roll[0]["time"] == pytest.approx(_v(on, "t_takeoff"))
    assert sum(r["time"] for r in rows if r["lap"] == 1) == pytest.approx(_v(on, "Lap1time"), rel=1e-12)
    assert sum(r["energy"] for r in rows if r["lap"] == 2) == pytest.approx(
        (_v(on, "energy_3laps") - (sum(r["energy"] for r in rows if r["lap"] == 1)) - 3 * 400.0 * 3.0) / 2, rel=1e-9)
    # the start offset lengthens lap 1's first straight only
    off_set = _lap(dict(takeoff=True, start_offset_ft=50.0), CL_ground=0.5)
    assert _v(off_set, "takeoff_straight_margin") == pytest.approx(_v(on, "takeoff_straight_margin") + 50 * FT2M)
    assert _v(off_set, "Lap1time") > _v(on, "Lap1time")


def test_a_roll_longer_than_the_straight_and_a_weak_motor_stay_finite():
    p = _lap(dict(takeoff=True), mu_roll=0.75, CL_ground=0.5)  # friction > thrust at rest: the floor carries the roll
    assert _v(p, "takeoff_accel_margin") < 0.0
    assert _v(p, "takeoff_straight_margin") < 0.0
    for k in ["t_takeoff", "s_ground_roll", "Lap1time", "laps_frac", "energy_3laps"]:
        assert np.isfinite(_v(p, k))


def test_headwind_at_lift_off_speed_is_an_analysis_failure():
    p = _lap(dict(takeoff=True), CL_ground=0.5)
    vlof = _v(p, "V_lof")
    p.set_val("wind", 1.01 * vlof)
    with pytest.raises(AnalysisError, match="non-physical lap"):
        p.run_model()


def test_takeoff_needs_propulsion_mode():
    p = om.Problem(reports=False)
    p.model.add_subsystem("lap", SmoothLapModel(power_mode="aircrafter", takeoff=True))
    with pytest.raises(ValueError, match="propulsion"):
        p.setup()


# --- partials -------------------------------------------------------------------------------------------------------
CUSTOM = lap_legs([Leg(kind="straight", length_ft=600, wind="head"), Leg(kind="turn", angle_deg=180),
                   Leg(kind="straight", length_ft=400, wind="none"), Leg(kind="turn", angle_deg=90),
                   Leg(kind="straight", length_ft=600, wind="tail"), Leg(kind="turn", angle_deg=90)])


@pytest.mark.parametrize("opts,wind", [(dict(takeoff=True), 0.0), (dict(takeoff=True), 2.5),
                                       (dict(takeoff=True, lap1=CUSTOM, lap_other=CUSTOM, start_offset_ft=30.0), 2.0)],
                         ids=["takeoff", "takeoff_wind", "takeoff_custom_course"])
def test_lap_partials_with_takeoff(opts, wind):
    p = _lap(opts, wind=wind, CL_ground=0.5)
    check_partials_fd(p, atol=20.0, rtol=5e-3, skip=("lapCount",))


# --- courses --------------------------------------------------------------------------------------------------------
def test_custom_course_runs_and_oval_is_shorter():
    p = _lap(dict(lap1=CUSTOM, lap_other=CUSTOM), wind=2.0)
    assert np.isfinite(_v(p, "ThreeLapTime")) and _v(p, "ThreeLapTime") > 0.0
    oval = CourseConfig(preset="oval").lap_options()
    dbf = CourseConfig(preset="dbf_2025").lap_options()
    t_oval = _v(_lap(dict(lap1=oval[0], lap_other=oval[1])), "otherLapTime")
    t_dbf = _v(_lap(dict(lap1=dbf[0], lap_other=dbf[1])), "otherLapTime")
    assert t_oval < t_dbf
    mid = CourseConfig(preset="dbf_2025_360_mid").lap_options()
    assert np.isfinite(_v(_lap(dict(lap1=mid[0], lap_other=mid[1]), wind=2.0), "ThreeLapTime"))


def test_course_validation_errors():
    with pytest.raises(ValueError, match="empty"):
        CourseConfig(legs=[])
    with pytest.raises(ValueError, match="no straight"):
        CourseConfig(legs=[{"kind": "turn", "angle_deg": 360}])
    with pytest.raises(ValueError, match="must be > 0"):
        CourseConfig(legs=[{"kind": "straight", "length_ft": 0.0, "wind": "head"}, {"kind": "turn", "angle_deg": 360}])
    tail_first = [{"kind": "straight", "length_ft": 500, "wind": "tail"}, {"kind": "turn", "angle_deg": 180},
                  {"kind": "straight", "length_ft": 500, "wind": "head"}, {"kind": "turn", "angle_deg": 180}]
    CourseConfig(legs=tail_first)  # fine with take-off off
    with pytest.raises(ValueError, match="head-wind straight"):
        CourseConfig(legs=tail_first, takeoff={"in_mission_time": True})


def test_course_validation_warnings():
    legs = [Leg(kind="straight", length_ft=500, wind="head"), Leg(kind="turn", angle_deg=180),
            Leg(kind="straight", length_ft=800, wind="tail"), Leg(kind="turn", angle_deg=90)]
    errors, warns = check_course(legs, legs)
    assert errors == [] and len(warns) == 2
    assert any("360" in w for w in warns) and any("start line" in w for w in warns)
    with pytest.warns(UserWarning, match="course"):
        CourseConfig(legs=[leg.model_dump() for leg in legs])
    for name in SHIPPED_PRESETS:
        lap1, other = load_preset(name)
        assert check_course(lap1, other, takeoff_on=True) == ([], [])


def test_save_preset_refuses_shipped_names(tmp_path):
    legs = [leg.model_dump() for leg in load_preset("oval")[1]]
    with pytest.raises(ValueError, match="shipped"):
        save_preset("dbf_2025", legs, directory=tmp_path)
    path = save_preset("my_course", legs, directory=tmp_path)
    assert lap_legs(load_preset(str(path))[1]) == lap_legs(load_preset("oval")[1])


# --- smoke: the full problems with take-off on -----------------------------------------------------------------------
@pytest.fixture(scope="module")
def takeoff_mp():
    from pathlib import Path

    from aircrafter2.problems.multipoint import build_multipoint_problem, load_multipoint_config

    warnings.filterwarnings("ignore")
    mcfg = load_multipoint_config(Path(__file__).resolve().parents[1] / "configs" / "dbf2025_multipoint_takeoff.yaml")
    mcfg.base.aero.num_y = 11
    prob = build_multipoint_problem(mcfg, driver=False, mda_retry=True)
    prob.run_model()
    return mcfg, prob


def test_multipoint_and_m1_check_with_takeoff(takeoff_mp):
    from aircrafter2.problems.multipoint import m1_check

    mcfg, prob = takeoff_mp
    for pt in ("M2", "M3"):
        assert float(prob.get_val(f"points.{pt}.t_takeoff")[0]) > 0.0
        assert float(prob.get_val(f"points.{pt}.V_lof")[0]) > float(prob.get_val(f"points.{pt}.Vstall")[0])
    m1 = m1_check(mcfg, prob)
    assert "laps_frac" in m1 and np.isfinite(m1["laps_frac"])


def test_rdo_node_with_takeoff():
    from pathlib import Path

    from aircrafter2.problems.multipoint import load_multipoint_config
    from aircrafter2.robustness.rdo_multipoint import build_robust_multipoint

    warnings.filterwarnings("ignore")
    root = Path(__file__).resolve().parents[1] / "configs"
    mcfg = load_multipoint_config(root / "dbf2025_multipoint_rdo.yaml")
    mcfg.base.course = CourseConfig(takeoff={"in_mission_time": True})
    mcfg.base.aero.num_y = 11
    mcfg.base.robust.active = ["lap.wind"]  # d = 1: 3 nodes
    prob = build_robust_multipoint(mcfg, driver=False)
    prob.run_model()
    t = [float(prob.get_val(f"nodes.n{j}.points.M2.t_takeoff")[0]) for j in range(prob.rdo["N"])]
    assert all(x > 0.0 for x in t) and len(set(np.round(t, 9))) == 3  # the headwind shortens the roll


def test_leg_rows_roundtrip_order_and_errors():
    from aircrafter2.mission.course import legs_from_rows, load_preset, rows_from_legs

    _, other = load_preset("dbf_2025")
    legs, errors = legs_from_rows(rows_from_legs(other))
    assert legs == other and not errors
    rows = rows_from_legs(other)
    rows[0]["order"] = 99  # the first straight moved to the end
    legs, _ = legs_from_rows(rows)
    assert legs[-1] == other[0] and legs[0] == other[1]
    legs, errors = legs_from_rows([dict(order=1, kind="straight", length_ft=float("nan"), wind="head"),
                                   dict(order=2, kind=None)])
    assert not legs and errors == ["lap leg 1 (straight): give its length (ft)"]


def test_course_sketch_draws_presets():
    import matplotlib.pyplot as plt

    from aircrafter2.mission.course import Leg, load_preset
    from aircrafter2.plotting.course_sketch import course_sketch

    for name in ("dbf_2025", "dbf_2025_360_mid", "oval"):
        lap1, other = load_preset(name)
        fig = course_sketch(other, lap1, takeoff_on=True, start_offset_ft=30.0)
        assert "2000 ft of straights" in fig.axes[0].get_title()
        plt.close(fig)
    cross = [Leg(kind="straight", length_ft=300, wind="none"), Leg(kind="turn", angle_deg=90)]
    fig = course_sketch(cross, load_preset("oval")[1])  # a different lap 1: two panels
    assert len(fig.axes) == 2 and "90° of turning" in fig.axes[1].get_title()
    plt.close(fig)
