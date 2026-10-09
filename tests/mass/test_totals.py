"""check_totals through the coupled mass closure (smooth mode, derived tube radius)."""

import pytest
from openmdao.utils.assert_utils import assert_check_totals

from aircrafter2.config import MassConfig, load_yaml
from aircrafter2.mass.garcia_gamboa import build_mass_problem

from ..conftest import CONFIGS

WRT = ["b", "c_mean", "c_root", "tc", "t_at_spar", "V_design", "m_pay", "m_sys", "f_wing", "f_tb", "n",
       "x_spar", "delta_tip_ratio"]
OF = ["m_total", "m_wing", "m_tb", "t_tube", "min_gauge_margin"]

GEOMETRY = dict(b=1.5, c_mean=0.25, c_root=0.28, S_wing=0.375, tc=0.12, t_at_spar=0.115, y1=0.25,
                l_ht=0.8, l_tb=0.7, S_ht=0.06, S_vt=0.03, S_cb=0.12, r_tb=0.008, V_design=22.0, Cm=-0.08,
                CL_alpha=4.8, x_spar=0.28, x_gd=0.28, n=5.0, A_c2=0.08, P_c=2.06, m_sys=1.2, m_pay=2.0)


@pytest.mark.parametrize("mode", ["paper", "physical"])
def test_check_totals(mode):
    data = load_yaml(CONFIGS / "mass_garcia_gamboa_2022.yaml")
    data["inputs"].update(GEOMETRY)
    data["options"]["sizing_radius_mode"] = mode
    data["options"]["n_panels"] = 2
    cfg = MassConfig(**data)
    prob = build_mass_problem(cfg)
    for name in WRT:
        prob.model.add_design_var(name)
    for name in OF:
        prob.model.add_constraint(name, lower=-1e9)
    prob.setup(force_alloc_complex=True)
    from aircrafter2.mass.garcia_gamboa import set_mass_inputs
    from aircrafter2.mass.garcia_gamboa.inputs import active_inputs
    active = set(active_inputs(cfg.options))
    set_mass_inputs(prob, {k: v for k, v in cfg.inputs.items() if k in active})
    prob.run_model()
    totals = prob.check_totals(method="cs", out_stream=None)
    assert_check_totals(totals, atol=1e-8, rtol=1e-6)
