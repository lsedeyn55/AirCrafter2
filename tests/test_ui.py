"""Streamlit UI, run headless."""

from pathlib import Path

APP = Path(__file__).resolve().parents[1] / "aircrafter2" / "ui" / "app.py"


def test_streamlit_app_runs_headless():
    from streamlit.testing.v1 import AppTest

    at = AppTest.from_file(str(APP), default_timeout=900)
    at.run()
    assert not at.exception
    at.sidebar.select_slider[0].set_value(11)
    at.button[0].click().run()
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
    at.button[0].click().run()
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
    assert not at.button[0].disabled
    # validation: reversed bounds, start outside, bounds outside 100-5000 -> errors and no run
    _ni(at, "Kv lower bound (rpm/V)").set_value(1300.0).run()
    errs = [e.value for e in at.sidebar.error]
    assert any("must be below the upper bound" in e for e in errs) and any("within the bounds" in e for e in errs)
    assert at.button[0].disabled and not _kv_caption(at)
    _ni(at, "Kv lower bound (rpm/V)").set_value(600.0)
    _ni(at, "Kv upper bound (rpm/V)").set_value(6000.0).run()
    assert any("within 100-5000" in e.value for e in at.sidebar.error) and at.button[0].disabled
    # off: fixed motor, no Kv design variable
    toggle = [c for c in at.sidebar.checkbox if c.label == "Optimise motor Kv"][0]
    toggle.uncheck().run()
    assert not at.exception and not _kv_caption(at) and not at.sidebar.error
    assert any(w.label == "Motor Kv (rpm/V) - PLACEHOLDER" for w in at.sidebar.number_input)
    assert not at.button[0].disabled
