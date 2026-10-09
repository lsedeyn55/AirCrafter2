"""The OpenMDAO group must agree with an independent plain-Python implementation (hard max)."""

import itertools

import pytest

from aircrafter2.mass.garcia_gamboa import build_mass_problem, mass_breakdown
from .reference_gg import reference_mass

MODES = list(itertools.product(["paper", "exact"], ["paper", "bredt"], ["paper", "eq54"], ["paper", "physical"]))


@pytest.mark.parametrize("deflection,torsion,boom,radius", MODES)
def test_matches_reference(tube_spar_case, deflection, torsion, boom, radius):
    case = tube_spar_case.model_copy(deep=True)
    o = case.options
    o.deflection_mode, o.torsion_stiffness_mode = deflection, torsion
    o.tail_boom_load_mode, o.sizing_radius_mode = boom, radius
    prob = build_mass_problem(case)
    prob.run_model()
    got = mass_breakdown(prob)

    from aircrafter2.mass.garcia_gamboa.inputs import INPUTS
    p = {k: v[1] for k, v in INPUTS.items()}
    p.update(case.inputs)
    ref = reference_mass(p, deflection, torsion, boom, radius, n_panels=o.n_panels)
    for name in ["m_tube", "m_rib", "m_le", "m_te", "m_film", "m_wing", "m_ht", "m_vt", "m_tb", "m_cb",
                 "m_gear", "m_empty", "m_total"]:
        assert got[name] == pytest.approx(ref[name], rel=1e-8), name
    assert prob.get_val("t_tube")[0] == pytest.approx(ref["t_tube"], rel=1e-8)
    assert prob.get_val("t_tb")[0] == pytest.approx(ref["t_tb"], rel=1e-8)


def test_twist_term_matches_reference(tube_spar_case):
    case = tube_spar_case.model_copy(deep=True)
    case.options.include_twist = True
    case.inputs["theta_tip_max"] = 0.002  # small enough that Eq. 116 governs the torsion term
    prob = build_mass_problem(case)
    prob.run_model()
    from aircrafter2.mass.garcia_gamboa.inputs import INPUTS
    p = {k: v[1] for k, v in INPUTS.items()}
    p.update(case.inputs)
    ref = reference_mass(p, n_panels=case.options.n_panels, include_twist=True)
    assert prob.get_val("t_tube")[0] == pytest.approx(ref["t_tube"], rel=1e-8)
    assert prob.get_val("t_theta")[0] > prob.get_val("t_T")[0]
