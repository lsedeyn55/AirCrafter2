"""Step 8: scoring plug-ins, CLI, figures, Streamlit UI (headless)."""

from pathlib import Path

import numpy as np
import pytest

from aircrafter2.problems.full_mdo import build_full_problem, load_full_config
from aircrafter2.scoring import ScoringPlugin, available, get_plugin, register

from .conftest import CONFIGS

APP = Path(__file__).resolve().parents[1] / "aircrafter2" / "ui" / "app.py"


def test_placeholder_plugins_registered():
    assert {"three_lap_time", "laps_in_window"} <= set(available())
    assert get_plugin("three_lap_time").sense == "min"
    assert get_plugin("laps_in_window").sense == "max"
    with pytest.raises(KeyError):
        get_plugin("no_such_plugin")


@register("test_payload_per_time")
class _PayloadPerTime(ScoringPlugin):
    sense = "max"
    inputs = {"m_pay": "kg", "ThreeLapTime": "s"}

    def score(self, x):
        return x["m_pay"] / x["ThreeLapTime"]


def _coarse(plugin):
    cfg = load_full_config(CONFIGS / "full_mdo.yaml")
    cfg.aero.num_y = 11
    cfg.scoring.plugin = plugin
    return cfg


def test_custom_plugin_is_wired_as_objective():
    prob = build_full_problem(_coarse("test_payload_per_time"))
    prob.run_model()
    score = prob.get_val("score")[0]
    assert score == pytest.approx(prob.get_val("m_pay")[0] / prob.get_val("ThreeLapTime")[0], rel=1e-12)
    objs = prob.driver.get_objective_values()
    assert "score" in objs
    J = prob.compute_totals(of=["score"], wrt=["b"])
    assert np.isfinite(J["score", "b"]).all()


def test_analysis_only_problem_still_has_score():
    prob = build_full_problem(_coarse("laps_in_window"), driver=False)
    prob.run_model()
    assert prob.get_val("score")[0] == pytest.approx(prob.get_val("laps_frac")[0])


def test_cli_analyse_writes_outputs(tmp_path):
    from aircrafter2.cli import main

    cfg_file = tmp_path / "cfg.yaml"
    text = (CONFIGS / "full_mdo.yaml").read_text().replace("num_y: 21", "num_y: 11")
    text = text.replace("mass_config: mass_garcia_gamboa_2022.yaml", f"mass_config: {CONFIGS / 'mass_garcia_gamboa_2022.yaml'}")
    cfg_file.write_text(text)
    out = tmp_path / "out"
    assert main(["run", str(cfg_file), "--analyse", "--out", str(out)]) == 0
    for f in ["summary.md", "results.json", "n2.html", "xdsm.tex", "xdsm.tikz", "xdsm.pdf"]:
        assert (out / f).exists(), f
    for f in ["planform", "airfoil", "spanwise_lift", "drag_breakdown", "mass_breakdown"]:
        assert (out / "figures" / f"{f}.pdf").exists() and (out / "figures" / f"{f}.png").exists()


def test_streamlit_app_runs_headless():
    from streamlit.testing.v1 import AppTest

    at = AppTest.from_file(str(APP), default_timeout=900)
    at.run()
    assert not at.exception
    at.sidebar.select_slider[0].set_value(11)
    at.button[0].click().run()
    assert not at.exception
    assert any("AirCrafter 2.0 result" in m.value for m in at.markdown)
