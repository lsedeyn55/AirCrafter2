"""Code verification: reproduce Garcia & Gamboa (2022) Table 15 (calculated masses).

Inputs from Tables 3, 12, 13, 14 (configs/verification/*.yaml). This checks the
implementation against the paper's own numbers; it is not a design validation.
Interpretation choices needed to reproduce it are documented in
docs/verification/mass.md and docs/development/PLAN.md §7.
"""

import pytest

from aircrafter2.mass.garcia_gamboa import build_mass_problem, mass_breakdown

TOL = 0.03  # D5 working value: +-3 % per component


def test_table15_reproduction(tube_spar_case):
    prob = build_mass_problem(tube_spar_case)
    prob.run_model()
    got = mass_breakdown(prob)
    for name, expected in tube_spar_case.expected.items():
        assert got[name] == pytest.approx(expected, rel=TOL), (
            f"{tube_spar_case.name} {name}: {got[name]:.4f} vs Table 15 {expected:.4f}")


def test_closure_is_converged(tube_spar_case):
    prob = build_mass_problem(tube_spar_case)
    prob.run_model()
    m_total = prob.get_val("m_total")[0]
    assert prob.get_val("m")[0] == pytest.approx(m_total, rel=1e-10)
    parts = sum(prob.get_val(k)[0] for k in ["m_wing", "m_ht", "m_vt", "m_fus", "m_gear", "m_sys"])
    assert prob.get_val("m_empty")[0] == pytest.approx(parts, rel=1e-12)


def test_literal_reading_does_not_reproduce(tube_spar_case):
    """Documents PLAN G19/G23: with the self-consistent readings the paper's numbers are not reproduced."""
    case = tube_spar_case.model_copy(deep=True)
    case.options.tail_boom_load_mode = "eq54"
    case.options.sizing_radius_mode = "physical"
    prob = build_mass_problem(case)
    prob.run_model()
    got = mass_breakdown(prob)
    assert got["m_tb"] > 5 * case.expected["m_tb"]
    assert got["m_wing"] > 1.10 * case.expected["m_wing"]
