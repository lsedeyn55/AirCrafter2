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


# --- USYD calibration hook: external sizing, lumped targets, leave-one-out ----------------------------------------

def _linear_case(name, m, S_ht, S_cb, l_tb, r_tb, targets):
    """A CalibrationCase whose fitted outputs are linear in the factors: areal tails, external sizing (gear =
    f_gear m), cargo bay f_cb S_cb, and a boom on its 1 mm minimum wall."""
    from aircrafter2.config import MassConfig, load_yaml
    from aircrafter2.mass.calibration import CalibrationCase

    from ..conftest import CONFIGS

    cfg = MassConfig(**load_yaml(CONFIGS / "mass_garcia_gamboa_2022.yaml"))
    cfg.options.tail_mass_model = "areal"
    cfg.inputs.update(b=1.6, c_mean=0.33, c_root=0.33, S_wing=0.53, t_at_spar=0.117, S_ht=S_ht, S_vt=0.06,
                      S_cb=S_cb, l_ht=l_tb + 0.15, l_tb=l_tb, r_tb=r_tb, n=5.0, V_design=30.0, Cm=-0.08,
                      CL_alpha=4.0, t_tb_min=1e-3, m_sys=0.0, m_pay=0.0)
    return CalibrationCase(name=name, config=cfg, targets=targets, sizing_mass=m)


def test_linear_fit_matches_hand_least_squares():
    """Eq. 128 with external sizing and a lumped target on a case whose masses are linear in the factors: the hook
    must return the hand-built linear least-squares solution of sum (m_calc / m_real - 1)^2."""
    import numpy as np

    from aircrafter2.mass.calibration import fit_correction_factors, leave_one_out, predict

    rho, t = 1600.0, 1e-3
    geo = {"A": dict(m=8.0, S_ht=0.19, S_cb=0.75, l_tb=0.85, r_tb=0.008),
           "B": dict(m=11.8, S_ht=0.15, S_cb=0.47, l_tb=0.95, r_tb=0.010)}
    real = {"A": {"m_ht": 0.20, "m_cb+m_tb": 1.45, "m_gear": 0.37},
            "B": {"m_ht": 0.19, "m_cb": 0.62, "m_tb": 0.30, "m_gear": 0.30}}
    cases = [_linear_case(n, targets=real[n], **g) for n, g in geo.items()]
    factors = ["a_ht", "f_cb", "f_tb", "f_gear"]

    def basis(g):  # d(output)/d(factor), by hand: areal tail, cargo bay, boom tube on the 1 mm wall, gear
        tube = np.pi * (g["r_tb"] ** 2 - (g["r_tb"] - t) ** 2) * g["l_tb"] * rho
        return {"m_ht": [g["S_ht"], 0, 0, 0], "m_cb": [0, g["S_cb"], 0, 0], "m_tb": [0, 0, tube, 0],
                "m_gear": [0, 0, 0, g["m"]]}

    rows = []
    for n, g in geo.items():
        B = basis(g)
        for key, y in real[n].items():
            rows.append(np.sum([B[k] for k in key.split("+")], axis=0) / y)
    A = np.array(rows)
    theta = np.linalg.lstsq(A, np.ones(len(rows)), rcond=None)[0]

    res = fit_correction_factors(cases, factors, x0=[1.0, 1.0, 3.0, 0.03])
    assert res.success
    for f, v in zip(factors, theta):
        assert res.factors[f] == pytest.approx(v, rel=1e-6), f
    assert res.objective == pytest.approx(float(np.sum((A @ theta - 1.0) ** 2)), rel=1e-6)
    _, pred = predict(cases[0], res.factors)  # external sizing: the gear sees the sizing mass, not a closure
    assert pred["m_gear"] == pytest.approx(res.factors["f_gear"] * 8.0, rel=1e-12)

    # leave-one-out on consistent synthetic data predicts the held-out case exactly
    truth = dict(zip(factors, theta))
    synth = []
    for n, g in geo.items():
        got, _ = predict(_linear_case(n, targets={"m_ht": 1.0}, **g), truth)
        synth.append(_linear_case(n, targets={k: sum(got[p] for p in k.split("+")) for k in real[n]}, **g))
    synth.append(_linear_case("C", targets={"m_ht": 0.2, "m_cb+m_tb": 1.0, "m_gear": 0.2},
                              **dict(m=6.0, S_ht=0.2, S_cb=0.6, l_tb=0.7, r_tb=0.009)))
    got_c, _ = predict(synth[2], truth)
    synth[2].targets = {k: sum(got_c[p] for p in k.split("+")) for k in synth[2].targets}
    loo = leave_one_out(synth, factors, x0=[1.0, 1.0, 3.0, 0.03])
    for name, r in loo.items():
        assert abs(r["structure_error"]) < 1e-6, name
        assert all(abs(e) < 1e-6 for e in r["errors"].values()), name
