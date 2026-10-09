"""Calibration hook (Eqs. 127-128)."""

import pytest

from aircrafter2.mass.calibration import fit_correction_factors
from aircrafter2.mass.garcia_gamboa import build_mass_problem, mass_breakdown

from ..conftest import verification_case

# Table 15 real/reported masses and Table 14 individual factors
REAL = {
    "ubi15": {"m_wing": 0.680, "m_ht": 0.050, "m_vt": 0.030, "m_tb": 0.175, "m_cb": 0.155, "m_gear": 0.129},
    "gra13": {"m_wing": 1.400, "m_ht": 0.095, "m_vt": 0.070, "m_tb": 0.200, "m_cb": 0.060, "m_gear": 0.045},
}
INDIVIDUAL = {
    "ubi15": {"f_wing": 1.097, "f_tb": 11.61, "f_ht": 0.7840, "f_vt": 0.4091, "f_cb": 0.6739, "f_gear": 0.0109},
    "gra13": {"f_wing": 1.151, "f_tb": 19.49, "f_ht": 0.6390, "f_vt": 0.4252, "f_cb": 0.4615, "f_gear": 0.0033},
}


@pytest.mark.parametrize("name", ["ubi15", "gra13"])
def test_recovers_table14_individual_factors(name):
    """Eq. 127 fitted to the Table 15 real masses must give the Table 14 individual factors."""
    res = fit_correction_factors([(verification_case(name), REAL[name])], list(INDIVIDUAL[name]))
    assert res.success
    for f, expected in INDIVIDUAL[name].items():
        # Table 14 gives 3-4 significant figures and our Table 15 reproduction is within ~1 %
        assert res.factors[f] == pytest.approx(expected, rel=0.03), f


def test_recovers_synthetic_combined_factors():
    """Eq. 128 on synthetic data generated with known factors."""
    truth = {"f_wing": 1.2, "f_tb": 15.0}
    cases = []
    for name in ["ubi15", "gra13"]:
        cfg = verification_case(name)
        cfg.inputs.update(truth)
        prob = build_mass_problem(cfg)
        prob.run_model()
        got = mass_breakdown(prob)
        cases.append((verification_case(name), {k: got[k] for k in ["m_wing", "m_tb", "m_ht"]}))
    res = fit_correction_factors(cases, ["f_wing", "f_tb"])
    assert res.objective < 1e-16
    for f, v in truth.items():
        assert res.factors[f] == pytest.approx(v, rel=1e-6)
