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
    at.sidebar.multiselect[0].set_value(["Kv"])  # d = 1: 3 nodes
    at.sidebar.select_slider[0].set_value(11)
    at.button[0].click().run()
    assert not at.exception
    assert any("Robust statistics" in m.value for m in at.markdown)
