"""Step 5: geometry, aero->mass coupling components, coupled MDA."""

import itertools

import numpy as np
import openmdao.api as om
import pytest
from openmdao.utils.assert_utils import assert_check_partials, assert_check_totals

from aircrafter2.aero.performance import LiftCurveSlope, WingSectionMoment
from aircrafter2.geometry import CargoBay, RectPlanform, TailGeometry
from aircrafter2.problems.aero_mass import build_aero_mass_problem, load_aero_mass_config

from .conftest import CONFIGS


def _check(comp, inputs, method="cs"):
    p = om.Problem(reports=False)
    p.model.add_subsystem("c", comp)
    p.setup(force_alloc_complex=True)
    for k, v in inputs.items():
        p.set_val(f"c.{k}", v)
    p.run_model()
    kw = {"form": "central", "step": 1e-6} if method == "fd" else {}
    assert_check_partials(p.check_partials(method=method, out_stream=None, **kw), atol=1e-7, rtol=1e-6)
    return p


def test_geometry_partials():
    _check(RectPlanform(), {"b": 1.4, "c": 0.22})
    _check(TailGeometry(), {"l_ht": 0.85, "S_ht": 0.07, "S_vt": 0.035, "S": 0.31, "b": 1.4, "c": 0.22,
                            "boom_offset": 0.12})
    _check(CargoBay(), {"l_cb": 0.4, "w_cb": 0.11, "h_cb": 0.13})


def test_coupling_component_partials():
    _check(LiftCurveSlope(), {"CL": 0.45, "alpha": 0.1})
    _check(WingSectionMoment(n_strips=4), {"Cm_sec": np.array([-0.1, -0.09, -0.11, -0.1]),
                                           "chords": np.full(5, 0.25), "widths": np.array([0.1, 0.2, 0.2, 0.25]),
                                           "S_ref": 0.375}, method="fd")


def test_wing_moment_is_mean_for_rectangular_wing():
    cm = np.array([-0.1, -0.08, -0.12])
    w = np.array([0.1, 0.3, 0.35])
    p = _check(WingSectionMoment(n_strips=3), {"Cm_sec": cm, "chords": np.full(4, 0.2), "widths": w,
                                               "S_ref": 2 * 0.2 * w.sum()}, method="fd")
    assert p.get_val("c.Cm_wing")[0] == pytest.approx(np.sum(cm * w) / w.sum(), rel=1e-12)


def _coarse_cfg(**options):
    cfg = load_aero_mass_config(CONFIGS / "aero_mass.yaml")
    cfg.aero.num_y = 11
    for k, v in options.items():
        setattr(cfg.mass.options, k, v)
    return cfg


def test_mda_consistency():
    """After the MDA: lift = weight, and the mass closure holds."""
    prob = build_aero_mass_problem(_coarse_cfg(), driver=False)
    prob.run_model()
    assert prob.get_val("cruise.L")[0] == pytest.approx(prob.get_val("W")[0], rel=1e-9)
    assert prob.get_val("m")[0] == pytest.approx(prob.get_val("m_total")[0], rel=1e-9)
    assert prob.get_val("Cm")[0] == pytest.approx(prob.get_val("cruise.Cm_wing")[0])


@pytest.mark.parametrize("corner", list(itertools.product([0, 1], repeat=4)))
def test_mda_converges_at_dv_corners(corner):
    cfg = _coarse_cfg()
    prob = build_aero_mass_problem(cfg, driver=False)
    for (name, bd), k in zip(cfg.design_variables.items(), corner):
        prob.set_val(name, bd.upper if k else bd.lower)
    prob.run_model()  # err_on_non_converge=True raises if the MDA fails
    assert prob.model.mda.nonlinear_solver._iter_count <= 10


@pytest.mark.parametrize("gauge", [True, False])
def test_coupled_check_totals(gauge):
    cfg = _coarse_cfg(use_min_gauge=gauge)
    prob = build_aero_mass_problem(cfg)
    prob.run_model()
    data = prob.check_totals(method="fd", step=1e-6, form="central", out_stream=None)
    assert_check_totals(data, atol=1e-5, rtol=3e-3)
