"""Full MDF problem (full_mdo): force and mass balances, total derivatives."""

import pytest
from openmdao.utils.assert_utils import assert_check_totals

from aircrafter2.problems.full_mdo import build_full_problem, load_full_config

from .conftest import CONFIGS


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
