"""Streamlit UI, run headless."""

from pathlib import Path

APP = Path(__file__).resolve().parents[1] / "aircrafter2" / "ui" / "app.py"


def _button(at, label):
    return [b for b in at.button if b.label == label][0]


def test_streamlit_app_runs_headless():
    from streamlit.testing.v1 import AppTest

    at = AppTest.from_file(str(APP), default_timeout=900)
    at.run()
    assert not at.exception
    at.sidebar.select_slider[0].set_value(11)
    _button(at, "Run").click().run()
    assert not at.exception
    assert any("AirCrafter 2.0 result" in m.value for m in at.markdown)


def test_streamlit_robust_toggle_runs_headless():
    from streamlit.testing.v1 import AppTest

    at = AppTest.from_file(str(APP), default_timeout=900)
    at.run()
    at.sidebar.selectbox[0].set_value("full_mdo_rdo.yaml").run()
    robust = [c for c in at.sidebar.checkbox if c.label == "Robust design optimisation"][0]
    assert robust.value  # on for the RDO config
    at.sidebar.multiselect[0].set_value(["Kv_factor"])  # d = 1: 3 nodes (Kv is a design variable here)
    at.sidebar.select_slider[0].set_value(11)
    _button(at, "Run").click().run()
    assert not at.exception
    assert any("Robust statistics" in m.value for m in at.markdown)


def _ni(at, label):
    return [w for w in at.sidebar.number_input if w.label == label][0]


def _kv_caption(at):
    return [c.value for c in at.sidebar.caption if c.value.startswith("Kv design variable")]


def test_streamlit_kv_toggle_bounds_and_validation():
    from streamlit.testing.v1 import AppTest

    at = AppTest.from_file(str(APP), default_timeout=300)
    at.run()
    assert not at.exception
    toggle = [c for c in at.sidebar.checkbox if c.label == "Optimise motor Kv"][0]
    assert toggle.value  # on for the default config
    assert _kv_caption(at) == ["Kv design variable: 500 to 1500 rpm/V, start 900."]
    assert any(w.label == "Reference motor Kv (rpm/V) - PLACEHOLDER" for w in at.sidebar.number_input)
    # user-set bounds and start reach cfg.design_variables["Kv"] / cfg.initial["Kv"] (the caption reads them back)
    _ni(at, "Kv lower bound (rpm/V)").set_value(600.0)
    _ni(at, "Kv upper bound (rpm/V)").set_value(1200.0)
    _ni(at, "Kv start (rpm/V)").set_value(800.0).run()
    assert _kv_caption(at) == ["Kv design variable: 600 to 1200 rpm/V, start 800."]
    assert not _button(at, "Run").disabled
    # validation: reversed bounds, start outside, bounds outside 100-5000 -> errors and no run
    _ni(at, "Kv lower bound (rpm/V)").set_value(1300.0).run()
    errs = [e.value for e in at.sidebar.error]
    assert any("must be below the upper bound" in e for e in errs) and any("within the bounds" in e for e in errs)
    assert _button(at, "Run").disabled and not _kv_caption(at)
    _ni(at, "Kv lower bound (rpm/V)").set_value(600.0)
    _ni(at, "Kv upper bound (rpm/V)").set_value(6000.0).run()
    assert any("within 100-5000" in e.value for e in at.sidebar.error) and _button(at, "Run").disabled
    # off: fixed motor, no Kv design variable
    toggle = [c for c in at.sidebar.checkbox if c.label == "Optimise motor Kv"][0]
    toggle.uncheck().run()
    assert not at.exception and not _kv_caption(at) and not at.sidebar.error
    assert any(w.label == "Motor Kv (rpm/V) - PLACEHOLDER" for w in at.sidebar.number_input)
    assert not _button(at, "Run").disabled


def test_streamlit_takeoff_and_course_headless(tmp_path):
    """Take-off on, a leg added to the course, Analyse at num_y 11: the summary shows the take-off time."""
    from streamlit.testing.v1 import AppTest

    at = AppTest.from_file(str(APP), default_timeout=900)
    at.run()
    assert not at.exception
    k_lof = [w for w in at.number_input if w.label.startswith("Lift-off speed factor")][0]
    assert k_lof.disabled  # take-off off by default: its parameters are disabled
    assert not [w for w in at.number_input if w.label.startswith("Take-off distance")][0].disabled
    [c for c in at.checkbox if c.label == "Include take-off in mission time"][0].check().run()
    assert not [w for w in at.number_input if w.label.startswith("Lift-off speed factor")][0].disabled
    # add a leg: a 100 ft crosswind straight at the end of the lap (head and tail distances still match)
    _button(at, "Add leg (lap)").click().run()
    assert not at.exception and not at.error and not _button(at, "Run").disabled
    at.sidebar.select_slider[0].set_value(11)
    _button(at, "Run").click().run()
    assert not at.exception
    md = "\n".join(m.value for m in at.markdown)
    assert "AirCrafter 2.0 result" in md and "`t_takeoff`" in md and "ground roll" in md
    assert "Course: custom" in md and "Leg breakdown" in md and "| 1 | 1 | ground roll |" in md
    assert "| 1 | 8 | straight | 100 ft | none |" in md  # the added leg was flown (lap 1: roll + 7 legs)
    assert any(m.label.startswith("Take-off time") for m in at.metric)


def test_streamlit_course_validation_blocks_run():
    """A course error (a zero-length straight) shows and disables Run; a valid course re-enables it; save-as-preset
    refuses a shipped name."""
    import pandas as pd
    from streamlit.testing.v1 import AppTest

    at = AppTest.from_file(str(APP), default_timeout=300)
    key = "legs_full_mdo.yaml_dbf_2025"
    at.session_state[f"{key}_df"] = pd.DataFrame([
        dict(order=1, kind="straight", length_ft=0.0, angle_deg=None, wind="head"),
        dict(order=2, kind="turn", length_ft=None, angle_deg=360.0, wind=None)])
    at.session_state[f"{key}_v"] = 0
    at.run()
    assert not at.exception
    assert any("must be > 0" in e.value for e in at.error)
    assert _button(at, "Run").disabled and _button(at, "Save preset").disabled
    at.session_state[f"{key}_df"] = pd.DataFrame([
        dict(order=1, kind="straight", length_ft=500.0, angle_deg=None, wind="head"),
        dict(order=2, kind="turn", length_ft=None, angle_deg=360.0, wind=None)])
    at.session_state[f"{key}_v"] = 1
    at.run()
    assert not at.error and not _button(at, "Run").disabled
    assert any("differ" in w.value for w in at.warning)  # 500 ft head, no tail: a warning above the sketch
    at.text_input[0].set_value("dbf_2025").run()
    _button(at, "Save preset").click().run()
    assert any("shipped preset" in e.value for e in at.error)
