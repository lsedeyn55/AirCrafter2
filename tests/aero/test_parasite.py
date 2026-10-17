"""Parasite drag of tails and fuselage: hand values and partials."""

import numpy as np
import pytest

from aircrafter2.aero.parasite import ParasiteDrag, cf_turbulent

from ..conftest import check_partials_fd, run_component


def test_parasite_hand_values_and_partials():
    p = run_component(ParasiteDrag(), v=20.0, rho=1.2, mu=1.8e-5, S=0.4, S_ht=0.06, AR_ht=4.0, tc_tail=0.09,
                      xt_tail=0.3)
    c = np.sqrt(0.06 / 4.0)
    cf = 0.455 / np.log10(1.2 * 20 * c / 1.8e-5) ** 2.58
    ff = 1 + 0.6 / 0.3 * 0.09 + 100 * 0.09**4
    assert p.get_val("c.CD_ht")[0] == pytest.approx(cf * ff * 0.06 * (1.977 + 0.52 * 0.09) / 0.4, rel=1e-12)
    assert cf_turbulent(1e6) == pytest.approx(0.455 / 6**2.58)
    check_partials_fd(p)
    q = run_component(ParasiteDrag(fuselage_drag=False))
    assert q.get_val("c.CD_fus")[0] == 0.0
