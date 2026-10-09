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


def test_tail_boom_min_wall_and_refitted_factor():
    """Default basis: 1 mm minimum boom wall with f_tb refitted to Table 15 real boom masses."""
    import numpy as np

    from aircrafter2.config import MassConfig, load_yaml

    from ..conftest import CONFIGS

    cfg = MassConfig(**load_yaml(CONFIGS / "mass_garcia_gamboa_2022.yaml"))
    assert cfg.options.tb_min_gauge and cfg.inputs["t_tb_min"] == 1.0e-3
    cfg.inputs.update(b=1.5, c_mean=0.25, c_root=0.25, S_wing=0.375, t_at_spar=0.12, l_ht=0.8, l_tb=0.65,
                      S_ht=0.06, S_vt=0.03, S_cb=0.13, r_tb=0.008, V_design=20.0, Cm=-0.1, CL_alpha=4.5,
                      n=5.0, m_sys=1.2, m_pay=1.5)
    prob = build_mass_problem(cfg)
    prob.run_model()
    t_tb = prob.get_val("t_tb")[0]
    assert prob.get_val("t_tb_gauge_margin")[0] < 0  # the minimum wall governs at DBF scale
    assert t_tb == pytest.approx(1.0e-3, rel=0.01)  # KS floor over-estimate is < 1 %
    r, l = 0.008, 0.65
    tube = np.pi * (r**2 - (r - t_tb) ** 2) * l * 1600.0
    assert prob.get_val("m_tb")[0] == pytest.approx(cfg.inputs["f_tb"] * tube, rel=1e-9)


def test_refit_reproduces_default_f_tb():
    cases = []
    for n, m in {"ubi11": 0.126, "gra13": 0.200, "ubi15": 0.175}.items():
        c = verification_case(n)
        c.options.tb_min_gauge = True
        c.inputs["t_tb_min"] = 1.0e-3
        cases.append((c, {"m_tb": m}))
    res = fit_correction_factors(cases, ["f_tb"], x0=[1.0], bounds={"f_tb": (0.05, 100.0)})
    assert res.factors["f_tb"] == pytest.approx(0.847, abs=5e-4)
