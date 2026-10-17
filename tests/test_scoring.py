"""Scoring plug-ins: registry, custom plug-in wired as the objective, analysis-only problems."""

import numpy as np
import pytest

from aircrafter2.problems.full_mdo import build_full_problem, load_full_config
from aircrafter2.scoring import ScoringPlugin, available, get_plugin, register

from .conftest import CONFIGS


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
