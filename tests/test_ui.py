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
