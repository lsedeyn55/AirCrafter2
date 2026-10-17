"""Static longitudinal stability: neutral point and CG hand values, partials."""

import numpy as np
import pytest

from aircrafter2.stability import StaticStability

from .conftest import check_partials_fd, run_component


def test_stability_hand_values_and_partials():
    p = run_component(StaticStability(), c=0.25, S=0.375, AR=6.0, a_w=4.5, S_ht=0.06, AR_ht=4.0, l_ht=0.8, eta_h=0.9)
    V_h = 0.06 * 0.8 / (0.375 * 0.25)
    a_h = 2 * np.pi * 4 / (2 + np.sqrt(20))
    x_np = 0.25 * 0.25 + 0.9 * V_h * a_h / 4.5 * (1 - 2 * 4.5 / (np.pi * 6)) * 0.25
    assert p.get_val("c.x_np")[0] == pytest.approx(x_np, rel=1e-12)
    assert p.get_val("c.x_cg")[0] == pytest.approx(0.1, rel=1e-12)  # all stations default to 0.1 m
    check_partials_fd(p)
