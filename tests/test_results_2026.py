"""2025-26 ruleset (docs/RESULTS_2026_PLAN.md): the dbf2026 scoring plug-in against hand calculations, the payload,
banner and payload-fuselage stores, the banner's deployment in the lap model, the wind-direction model (crosswind,
turn drift, symmetry, theta = 0 bit for bit) and the Q45 lap-1 grid."""

import json
import math
import warnings

import numpy as np
import openmdao.api as om
import pytest

from .conftest import ROOT

DATA = ROOT / "tests" / "data"
LB = 0.45359237
IN = 0.0254


# --- scoring: hand calculations with the rules' constants [2026 rules pp. 19-20] -------------------------------------

def test_net_income_humpback_table43_actual():
    """[HB] Table 43 'actual': 42 passengers, 14 cargo, 8 laps; 97.68 Wh pack (EF 0.9768)."""
    from aircrafter2.scoring.dbf2026 import net_income

    income = 42 * (6 + 2 * 8) + 14 * (10 + 8 * 8)          # 924 + 1036 = 1960
    cost = 8 * (10 + 42 * 0.5 + 14 * 2) * 0.9768            # 8 x 59 x 0.9768 = 461.0496
    assert income == 1960 and cost == pytest.approx(461.0496)
    assert net_income(42, 14, 8, 97.68) == pytest.approx(1960 - 461.0496, rel=1e-14)


def test_net_income_humpback_table33_design():
    """[HB] Table 33: 42 / 14, 9 laps, a 100 Wh pack (EF 1)."""
    from aircrafter2.scoring.dbf2026 import net_income

    assert net_income(42, 14, 9, 100.0) == pytest.approx(42 * 24 + 14 * 82 - 9 * 59, rel=1e-14)  # 2156 - 531 = 1625


def test_net_income_cost_above_income():
    """Three passengers and no cargo for 10 laps on 100 Wh: income 6 x 3 + 2 x 3 x 10 = 78, cost 10 x 11.5 = 115."""
    from aircrafter2.scoring.dbf2026 import net_income

    assert net_income(3, 0, 10, 100.0) == pytest.approx(78 - 115, rel=1e-14)


def test_m3_term_and_rac():
    """[HB] Table 43 'actual' M3: 4 laps with the 197 in banner, span 59 in (RAC 0.05 x 4.9167 + 0.75); the RAC floor
    0.90 is the 3 ft span."""
    from aircrafter2.scoring.dbf2026 import m3_term, rac

    assert rac(3.0) == pytest.approx(0.90)
    r = 0.05 * 59 / 12 + 0.75
    assert m3_term(4, 197.0, 59 / 12) == pytest.approx(4 * 197 / r, rel=1e-14)


def test_dbf2026_parts_and_floored():
    from aircrafter2.scoring.dbf2026 import DBF2026, net_income

    plug = DBF2026(NI_max=2800.0, B_max=1500.0)
    x = dict(n_p=np.array([42.0]), n_c=np.array([14.0]), laps_M2=np.array([8.6]), E_M2=np.array([97.68]),
             laps_M3=np.array([4.7]), L_banner=np.array([197.3]), b=np.array([59.2 * IN]))
    p = plug.parts(x)
    assert p["M2"][0] == pytest.approx(1 + net_income(42, 14, 8.6, 97.68) / 2800, rel=1e-14)
    rac = 0.05 * 59.2 / 12 + 0.75
    assert p["M3"][0] == pytest.approx(2 + 4.7 * 197.3 / rac / 1500, rel=1e-14)
    assert p["flight_score"][0] == pytest.approx(1 + p["M2"][0] + p["M3"][0])
    f = plug.floored(x)  # laps 8 and 4, banner 197.25 in, span 59 in (nearest inch)
    assert f["M2"] == pytest.approx(1 + net_income(42, 14, 8, 97.68) / 2800, rel=1e-12)
    assert f["M3"] == pytest.approx(2 + 4 * 197.25 / (0.05 * 59 / 12 + 0.75) / 1500, rel=1e-12)


def test_dbf2026_score_partials():
    from aircrafter2.scoring.dbf2026 import DBF2026

    p = om.Problem(reports=False)
    p.model.add_subsystem("s", DBF2026().component(), promotes=["*"])
    p.setup(force_alloc_complex=True)
    for k, x in dict(n_p=40.0, n_c=12.0, laps_M2=8.3, E_M2=95.0, laps_M3=4.4, L_banner=180.0, b=1.4).items():
        p.set_val(k, x)
    p.run_model()
    data = p.check_partials(method="fd", out_stream=None, compact_print=True)
    for comp in data.values():
        for key, d in comp.items():
            if key[0] == "M1":
                continue
            assert d["abs error"].forward < 1e-8 or d["rel error"].forward < 1e-5, key


# --- stores: payload, banner, payload fuselage --------------------------------------------------------------------

def _stores(payload, banner, **inp):
    from aircrafter2.problems.multipoint import load_multipoint_config
    from aircrafter2.stores2026 import PayloadFuselage, Stores2026

    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        cfg = load_multipoint_config(ROOT / "configs" / "dbf2026_multipoint.yaml").mp.stores2026
    p = om.Problem(reports=False)
    p.model.add_subsystem("st", Stores2026(config=cfg, payload=payload, banner=banner), promotes=["*"])
    p.model.add_subsystem("fus", PayloadFuselage(config=cfg), promotes=["*"])
    p.setup(force_alloc_complex=True)
    for k, x in inp.items():
        p.set_val(k, x)
    p.run_model()
    return p, cfg


def test_payload_masses_and_fuselage_humpback():
    """42 ducks and 14 pucks: [HB] Table 36's 1.84 + 5.25 lb plus the restraint PLACEHOLDERS; the passenger bay
    (39 in) sets the payload fuselage, at Humpback's cross-section."""
    p, c = _stores(True, False, n_p=42.0, n_c=14.0, S=0.5546, c=0.3701)
    assert 42 * c.m_duck / LB == pytest.approx(1.84, abs=0.005) and 14 * c.m_puck / LB == pytest.approx(5.25, abs=0.005)
    m = 42 * (c.m_duck + c.m_seat) + 14 * (c.m_puck + c.m_slot)
    assert p.get_val("m_pay")[0] == pytest.approx(m, rel=1e-12)
    assert p.get_val("l_cb")[0] == pytest.approx(39 * IN, rel=1e-4)  # KS over-estimate 5e-5 at 39 vs 23.5 in
    assert p.get_val("w_cb")[0] == pytest.approx(7.8 * IN) and p.get_val("h_cb")[0] == pytest.approx(5.5 * IN)
    assert p.get_val("CD_stores")[0] == 0.0


def test_banner_mass_and_drag_humpback():
    """The 197 in banner: [HB] Table 36's 'banner with deployment' 1.21 lb (fabric 0.22 oz/ft^2 + 0.3 lb ballast +
    the mechanism by difference), drag C_D,b L H / S, stowed minus deployed for the lap model."""
    S = 868 * IN**2
    p, c = _stores(False, True, n_p=42.0, n_c=14.0, S=S, c=0.3701, L_banner=197.0)
    assert p.get_val("m_banner")[0] / LB == pytest.approx(1.21, abs=0.002)
    area = (197 * IN) ** 2 / 5
    assert p.get_val("CD_banner")[0] == pytest.approx(c.cd_banner * area / S, rel=1e-12)
    assert p.get_val("dCD_stowed")[0] == pytest.approx(c.dcd_stowed - c.cd_banner * area / S, rel=1e-12)
    p.set_val("banner_drag_factor", 1.2)
    p.run_model()
    assert p.get_val("CD_banner")[0] == pytest.approx(1.2 * c.cd_banner * area / S, rel=1e-12)


@pytest.mark.parametrize("flags", [(True, False), (False, True), (False, False)])
def test_stores2026_partials(flags):
    inp = dict(n_p=30.0, n_c=11.0, S=0.5, c=0.33)
    if flags[1]:
        inp["L_banner"] = 150.0
    p, _ = _stores(*flags, **inp)
    data = p.check_partials(method="fd", out_stream=None, compact_print=True)
    for comp in data.values():
        for key, d in comp.items():
            if d["abs error"].forward is None:  # an undeclared (zero) pair
                continue
            assert d["abs error"].forward < 1e-8 or d["rel error"].forward < 1e-5, key


def test_fuselage_sizing_switches_to_the_cargo_bay():
    """Many pucks and few ducks: the cargo bays set the fuselage length."""
    p, c = _stores(True, False, n_p=3.0, n_c=20.0, S=0.5, c=0.33)
    assert p.get_val("l_cb")[0] == pytest.approx(20 * c.car_pitch, rel=1e-3)


# --- lap model: banner deployment, wind direction, Q45 ---------------------------------------------------------------

def _lap(wind=0.0, wind_dir=0.0, takeoff=False, legs="dbf_2025_360_mid", record=False, **opts):
    from aircrafter2.mission.course import CourseConfig
    from aircrafter2.mission.smooth_lap import SmoothLapModel

    ref = json.loads((DATA / "lap_wind_reference.json").read_text())
    if isinstance(legs, str):
        lap1, other = CourseConfig(preset=legs).lap_options()
    else:
        lap1 = other = legs
    p = om.Problem(reports=False)
    p.model.add_subsystem("lap", SmoothLapModel(power_mode="propulsion", lap1=lap1, lap_other=other, takeoff=takeoff,
                                                **opts), promotes=["*"])
    p.setup(force_alloc_complex=True)
    for k, x in ref["inputs"].items():
        p.set_val(k, x)
    p.set_val("wind", wind)
    p.set_val("wind_dir", wind_dir)
    p.run_model()
    return p


@pytest.mark.parametrize("takeoff", ["off", "on"])
@pytest.mark.parametrize("wind", [0.0, 3.0, 6.0, 9.0])
def test_wind_direction_zero_is_the_previous_model_bit_for_bit(takeoff, wind):
    """theta = 0, no turn drift, the uniform lap-1 grid: the lap model before the wind-direction change, bit for bit
    (tests/data/lap_wind_reference.json, recorded from the previous code)."""
    ref = json.loads((DATA / "lap_wind_reference.json").read_text())["values"][f"{takeoff}_{wind}"]
    p = _lap(wind, 0.0, takeoff == "on", lap1_cluster=1.0)
    assert {k: float(p.get_val(k)[0]) for k in ref} == ref


@pytest.mark.parametrize("drift", [False, True])
def test_wind_direction_symmetric(drift):
    a, b = _lap(6.0, 30.0, True, turn_drift=drift), _lap(6.0, -30.0, True, turn_drift=drift)
    for k in ("Lap1time", "otherLapTime", "laps_frac", "energy_3laps"):
        assert float(a.get_val(k)[0]) == pytest.approx(float(b.get_val(k)[0]), rel=1e-13), k


def test_pure_crosswind_straight():
    """A straight with the wind across it (theta = 90 deg): crabbed ground speed sqrt(V^2 - w^2). AirCrafter power
    mode holds the airspeed at V, so lap 2 of a one-straight course is flown at V: t = L / sqrt(V^2 - w^2)."""
    from aircrafter2.mission.smooth_lap import SmoothLapModel

    legs = [("s", 1000.0, 1.0)]
    p = om.Problem(reports=False)
    p.model.add_subsystem("lap", SmoothLapModel(power_mode="aircrafter", lap1=legs, lap_other=legs), promotes=["*"])
    p.setup()
    for k, x in dict(m=4.5, S=0.55, rho=1.18, CD0=0.035, k=0.075, CL_max=1.3, V=22.0, wind=5.0, wind_dir=90.0).items():
        p.set_val(k, x)
    p.run_model()
    rows = [r for r in p.model._get_subsystem("lap").leg_breakdown() if r["lap"] == 2]
    assert rows[0]["v_entry"] == pytest.approx(22.0, rel=1e-4) and rows[0]["v_exit"] == pytest.approx(22.0, rel=1e-4)
    assert rows[0]["time"] == pytest.approx(1000 * 0.3048 / math.sqrt(22.0**2 - 25.0), rel=1e-4)


def test_turn_drift_hand_calculation():
    """head 500 ft, 180 deg turn, tail 500 ft: the turn's drift (wind x t_turn, the air mass moving downwind) shortens
    the downwind straight by w cos(theta) t and offsets it by w sin(theta) t: flown length sqrt((L - a t)^2 + (c t)^2)."""
    legs = [("s", 500.0, 1.0), ("t", 180.0, None), ("s", 500.0, -1.0)]
    w, th = 6.0, 40.0
    p = _lap(w, th, legs=legs, turn_drift=True)
    rows = p.model._get_subsystem("lap").leg_breakdown()
    t_turn = rows[1]["time"]
    L = 500 * 0.3048
    a, c = w * math.cos(math.radians(th)), w * math.sin(math.radians(th))
    assert rows[2]["size"] * 0.3048 == pytest.approx(math.sqrt((L - a * t_turn) ** 2 + (c * t_turn) ** 2), rel=1e-10)
    assert rows[2]["drift_ft"] == pytest.approx(-a * t_turn / 0.3048, rel=1e-10)
    # without the switch, the turns ignore the wind (the previous model)
    q = _lap(w, th, legs=legs)
    assert q.model._get_subsystem("lap").leg_breakdown()[2]["size"] == pytest.approx(500.0)


@pytest.mark.parametrize("theta", [0.0, 30.0])
def test_lap_partials_wind_and_direction(theta):
    p = _lap(5.0, theta, True, turn_drift=True)
    data = p.check_partials(method="fd", out_stream=None, compact_print=True, includes=["lap"])
    for comp in data.values():
        for key, d in comp.items():
            if key[1] in ("wind", "wind_dir") and key[0] in ("Lap1time", "otherLapTime", "laps_frac"):
                assert d["abs error"].forward < 1e-6 or d["rel error"].forward < 2e-4, key


def test_banner_stowed_until_the_downwind_leg():
    """[2026 rules p. 20]: deployed on the downwind leg after the first upwind turn. On the 2025-26 course (the 360 deg
    turn mid back straight) lap 1's take-off straight and upwind turn fly stowed; every later leg is deployed."""
    from aircrafter2.mission.course import CourseConfig

    stowed = CourseConfig(preset="dbf_2025_360_mid").banner_stowed_legs()
    assert stowed == (0, 1)
    base = _lap(0.0, 0.0, True, banner_stowed_legs=stowed)
    rows = base.model._get_subsystem("lap").leg_breakdown()
    lap1 = [r for r in rows if r["lap"] == 1]
    assert [r["banner"] for r in lap1] == ["stowed", "stowed", "stowed"] + ["deployed"] * (len(lap1) - 3)
    assert all(r["banner"] == "deployed" for r in rows if r["lap"] == 2)
    # the stowed banner's drag (less than the towed one: dCD_stowed < 0) speeds up lap 1 only
    q = _lap(0.0, 0.0, True, banner_stowed_legs=stowed)
    q.set_val("dCD_stowed", -0.1)
    q.run_model()
    assert q.get_val("Lap1time")[0] < base.get_val("Lap1time")[0]
    # lap 2+ (deployed) only through lap 1's exit speed
    assert q.get_val("otherLapTime")[0] == pytest.approx(base.get_val("otherLapTime")[0], rel=1e-6)


@pytest.mark.parametrize("wind", [6.0, 9.0])
def test_q45_clustered_grid_against_640_steps(wind):
    """The clustered lap-1 grid (default) at 40 steps is within 0.05 % of a uniform 640-step integration near the
    wind limit; the old uniform 40-step grid is not (docs/RESULTS_2026_PLAN.md §4.6)."""
    ref = float(_lap(wind, 0.0, False, lap1_cluster=1.0, n_steps_straight=640).get_val("Lap1time")[0])
    new = float(_lap(wind, 0.0, False).get_val("Lap1time")[0])
    assert abs(new / ref - 1) < 5e-4


def test_multipoint_2026_totals():
    """check_totals of the 2025-26 multipoint problem at Humpback's start (the payload fuselage ahead of the planform:
    a feedback through the MDA would break the totals, as found at checkpoint 2)."""
    from aircrafter2.problems.multipoint import build_multipoint_problem, load_multipoint_config

    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        mcfg = load_multipoint_config(ROOT / "configs" / "dbf2026_multipoint.yaml")
        p = build_multipoint_problem(mcfg, driver=False, mda_retry=True)
        p.run_model()
        of = ["score", "points.M2.laps_frac", "points.M3.laps_frac", "points.M2.SM"]
        wrt = ["n_p", "n_c", "L_banner", "b", "throttle_M3"]
        J = p.compute_totals(of=of, wrt=wrt)
        x0 = {k: float(p.get_val(k)[0]) for k in wrt}
        for k in wrt:
            h = 1e-4 * max(abs(x0[k]), 1.0)
            vals = []
            for sgn in (1, -1):
                p.set_val(k, x0[k] + sgn * h)
                p.run_model()
                vals.append([float(p.get_val(o)[0]) for o in of])
            p.set_val(k, x0[k])
            for i, o in enumerate(of):
                fd = (vals[0][i] - vals[1][i]) / (2 * h)
                an = float(np.ravel(J[(o, k)])[0])
                assert an == pytest.approx(fd, rel=1e-3, abs=1e-6), (o, k)
    p.run_model()
