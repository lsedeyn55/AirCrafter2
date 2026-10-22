"""check_partials (complex step) for every mass component, across option variants.

Inputs are perturbed away from the defaults so no term sits exactly on a max() kink.
"""

import numpy as np
import openmdao.api as om
import pytest
from openmdao.utils.assert_utils import assert_check_partials

from aircrafter2.mass.garcia_gamboa.loads import CriticalLoads, TailBoomLoads, WingLoads
from aircrafter2.mass.garcia_gamboa.masses import FuselageMass, GearMass, MassSum, TailMass, WingMass
from aircrafter2.mass.garcia_gamboa.materials import Allowables
from aircrafter2.mass.garcia_gamboa.section import SectionCoeffs, TubeFitMargin, TubeRadius
from aircrafter2.mass.garcia_gamboa.sizing import RibSizing, TailBoomSizing, TubeSparSizing
from aircrafter2.mass.statistical import StatisticalMass

COMPONENTS = [
    ("allowables", Allowables, {}),
    ("section", SectionCoeffs, {}),
    ("tube_radius", TubeRadius, {}),
    ("tube_fit", TubeFitMargin, {}),
    ("wing_loads", WingLoads, {}),
    ("critical_ks", CriticalLoads, {"max_mode": "ks"}),
    ("critical_hard", CriticalLoads, {"max_mode": "hard"}),
    ("boom_loads_paper", TailBoomLoads, {"load_mode": "paper"}),
    ("boom_loads_eq54", TailBoomLoads, {"load_mode": "eq54"}),
    ("spar_default", TubeSparSizing, {}),
    ("spar_hard", TubeSparSizing, {"max_mode": "hard", "use_min_gauge": False}),
    ("spar_exact_bredt_twist", TubeSparSizing,
     {"deflection_mode": "exact", "torsion_stiffness_mode": "bredt", "include_twist": True}),
    ("spar_physical", TubeSparSizing, {"sizing_radius_mode": "physical"}),
    ("ribs_ks", RibSizing, {}),
    ("ribs_hard", RibSizing, {"max_mode": "hard"}),
    ("boom_sizing", TailBoomSizing, {}),
    ("boom_sizing_hard", TailBoomSizing, {"max_mode": "hard", "use_min_gauge": False}),
    ("wing_mass_1", WingMass, {"n_panels": 1}),
    ("wing_mass_3", WingMass, {"n_panels": 3}),
    ("tail_mass", TailMass, {}),
    ("tail_mass_areal", TailMass, {"tail_mass_model": "areal"}),
    ("fuselage_mass", FuselageMass, {}),
    ("gear_mass", GearMass, {}),
    ("mass_sum", MassSum, {}),
    ("statistical", StatisticalMass, {"model": "all"}),
    ("statistical_balsa", StatisticalMass, {"model": "balsa"}),
]


@pytest.mark.parametrize("label,cls,opts", COMPONENTS, ids=[c[0] for c in COMPONENTS])
def test_check_partials(label, cls, opts):
    prob = om.Problem(reports=False)
    comp = prob.model.add_subsystem("comp", cls(**opts))
    prob.setup(force_alloc_complex=True)
    rng = np.random.default_rng(abs(hash(label)) % 2**32)
    for name, meta in comp.list_inputs(out_stream=None, return_format="dict").items():
        val = meta["val"]
        prob.set_val(f"comp.{name}", val * rng.uniform(0.8, 1.25, size=val.shape))
    prob.run_model()
    data = prob.check_partials(method="cs", out_stream=None, compact_print=True)
    assert_check_partials(data, atol=1e-8, rtol=1e-7)


@pytest.mark.parametrize("cls,opts,values", [
    (WingMass, {"n_panels": 2}, {"k_wing": 2.1, "f_build": 1.3}),
    (TailMass, {"tail_mass_model": "areal"}, {"a_ht": 1.2, "a_vt": 1.4, "f_build": 1.3}),
], ids=["wing_k_wing", "tail_areal_f_build"])
def test_check_partials_construction_factors(cls, opts, values):
    """k_wing != 1 and the areal tails with f_build != 1 (docs/studies/mass_calibration_usyd.md)."""
    prob = om.Problem(reports=False)
    prob.model.add_subsystem("comp", cls(**opts))
    prob.setup(force_alloc_complex=True)
    for name, val in values.items():
        prob.set_val(f"comp.{name}", val)
    prob.run_model()
    if cls is TailMass:
        assert prob.get_val("comp.m_ht")[0] == pytest.approx(1.2 * 0.1 * 1.3)
    else:
        assert prob.get_val("comp.m_wing")[0] == pytest.approx(2.1 * 1.12 * 1.3 * prob.get_val("comp.m_wing_bare")[0])
    data = prob.check_partials(method="cs", out_stream=None, compact_print=True)
    assert_check_partials(data, atol=1e-8, rtol=1e-7)
