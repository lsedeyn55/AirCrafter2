"""Oval lap model (fixed cruise speed)."""

import numpy as np
import openmdao.api as om
import pytest
from openmdao.utils.assert_utils import assert_check_partials, assert_check_totals

from aircrafter2.mission import LoadFactor, OvalLapTime
from aircrafter2.problems.oval_lap import OvalLapProblemConfig, build_oval_lap_problem


def _cs(comp, inputs):
    p = om.Problem(reports=False)
    p.model.add_subsystem("c", comp)
    p.setup(force_alloc_complex=True)
    for k, v in inputs.items():
        p.set_val(f"c.{k}", v)
    p.run_model()
    assert_check_partials(p.check_partials(method="cs", out_stream=None), atol=1e-9, rtol=1e-8)
    return p


@pytest.mark.parametrize("mode,L", [("ks", 150.0), ("ks", 300.0), ("hard", 150.0)])
def test_load_factor_partials(mode, L):
    _cs(LoadFactor(max_mode=mode), {"L_turn": L, "W": 49.0, "n_struct": 5.0})


def test_lap_time_partials_and_hand_calc():
    V, n, Ls, g = 20.0, 3.0, 304.8, 9.80665
    p = _cs(OvalLapTime(), {"V": V, "n": n, "straight_length": Ls, "g": g})
    R = V**2 / (g * np.sqrt(n**2 - 1))
    assert p.get_val("c.turn_radius")[0] == pytest.approx(R, rel=1e-6)  # softplus floor is negligible at n = 3
    assert p.get_val("c.lap_time")[0] == pytest.approx((2 * Ls + 2 * np.pi * R) / V, rel=1e-6)
    assert p.get_val("c.turn_rate")[0] == pytest.approx(g * np.sqrt(n**2 - 1) / V, rel=1e-6)


def test_load_factor_is_capped():
    p = om.Problem(reports=False)
    p.model.add_subsystem("c", LoadFactor())
    p.setup()
    p.set_val("c.L_turn", 49.0 * 8)
    p.set_val("c.W", 49.0)
    p.set_val("c.n_struct", 5.0)
    p.run_model()
    assert 5.0 * 0.99 < p.get_val("c.n")[0] <= 5.0


def test_oval_problem_check_totals():
    cfg = OvalLapProblemConfig(aero={"num_y": 11}, power_available=400.0)
    prob = build_oval_lap_problem(cfg)
    prob.run_model()
    data = prob.check_totals(method="fd", step=1e-6, form="central", out_stream=None)
    assert_check_totals(data, atol=1e-5, rtol=2e-3)


def test_oval_optimum_sits_on_the_turn_margin():
    """Without a power limit the turn C_L margin must be the active constraint that sets R."""
    cfg = OvalLapProblemConfig(aero={"num_y": 11})
    prob = build_oval_lap_problem(cfg)
    prob.driver.options["disp"] = False
    res = prob.run_driver()
    assert res.success
    assert prob.get_val("turn.CL_max_over_CL")[0] == pytest.approx(cfg.turn_clmax_ratio_min, rel=1e-4)
    assert prob.get_val("lift_residual")[0] == pytest.approx(0.0, abs=1e-6)
    n = prob.get_val("n")[0]
    R = cfg.V**2 / (9.80665 * np.sqrt(n**2 - 1))
    assert prob.get_val("turn_radius")[0] == pytest.approx(R, rel=1e-5)
