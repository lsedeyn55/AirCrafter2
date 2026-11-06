"""RDO with the reduced uncertainty set (mass and wind): the build-mass factor f_build and its RDO composition.

docs/development/RDO_REDUCED_PLAN.md. f_build = 1 must reproduce every existing result bit for bit: compared with == against the
regression references (tests/data/*_reference.json) and the published 2024-25 nominal evaluation (frozen from
results_2025.json at ec639dd into tests/data/multipoint_nominal_reference.json), none of them regenerated.
"""

import json
import warnings
from pathlib import Path

import numpy as np
import openmdao.api as om
import pytest
from openmdao.utils.assert_utils import assert_check_partials

from aircrafter2.mass.garcia_gamboa.masses import FuselageMass, GearMass, WingMass

from .migration import ref_params

ROOT = Path(__file__).resolve().parents[1]
CONFIGS = ROOT / "configs"
DATA = ROOT / "tests" / "data"
STRUCTURE = ["m_wing", "m_ht", "m_vt", "m_tb", "m_cb", "m_gear"]


# --- the component ----------------------------------------------------------------------------------------------------
@pytest.mark.parametrize("cls", [WingMass, FuselageMass, GearMass], ids=lambda c: c.__name__)
def test_f_build_partials(cls):
    prob = om.Problem(reports=False)
    prob.model.add_subsystem("comp", cls())
    prob.setup(force_alloc_complex=True)
    prob.set_val("comp.f_build", 1.3)
    prob.run_model()
    data = prob.check_partials(method="cs", out_stream=None, compact_print=True)
    assert_check_partials(data, atol=1e-8, rtol=1e-7)


def test_f_build_scales_every_structural_mass_and_closes_the_loop():
    """In the closed mass loop f_build scales the wing, tails, boom, cargo bay and gear at the converged total mass
    (the gear also sees the heavier total), and nothing else: m_sys and m_pay are untouched."""
    from aircrafter2.config.loader import load_yaml
    from aircrafter2.config.mass import MassConfig
    from aircrafter2.mass.garcia_gamboa.group import build_mass_problem, mass_breakdown

    cfg = MassConfig(**load_yaml(CONFIGS / "mass_garcia_gamboa_2022.yaml"))
    out = {}
    for fb in (1.0, 1.2):
        p = build_mass_problem(cfg)
        p.set_val("f_build", fb)
        p.run_model()
        out[fb] = mass_breakdown(p)
    a, b = out[1.0], out[1.2]
    assert b["m_total"] > a["m_total"]
    struct_a = sum(a[k] for k in STRUCTURE)
    struct_b = sum(b[k] for k in STRUCTURE)
    assert struct_b > 1.2 * struct_a * 0.999  # at least the factor (the heavier total also resizes the spar)
    assert b["m_total"] - b["m_empty"] == pytest.approx(a["m_total"] - a["m_empty"], rel=1e-12)  # payload


# --- f_build = 1 reproduces every existing result bit for bit ------------------------------------------------------------
@pytest.mark.parametrize("cfg_name", ref_params(json.loads((DATA / "configs_reference.json").read_text())))
def test_f_build_one_reproduces_the_config_references_bit_for_bit(cfg_name):
    from .configs_snapshot import snapshot

    ref = json.loads((DATA / "configs_reference.json").read_text())[cfg_name]
    now = snapshot(cfg_name)
    assert {k: now[k] for k in ref} == ref


def test_f_build_one_reproduces_the_published_multipoint_nominal_bit_for_bit():
    from aircrafter2.problems.multipoint import build_multipoint_problem, load_multipoint_config

    warnings.filterwarnings("ignore")
    from .migration import pre_calibration

    pub = json.loads((DATA / "multipoint_nominal_reference.json").read_text())  # frozen from the published commit
    mcfg = load_multipoint_config(CONFIGS / "dbf2025_multipoint.yaml")
    pre_calibration(mcfg.base)  # the published results' mass model (never regenerated)
    for pt in ("M2", "M3"):
        mcfg.mp.points[pt].prop = pub["props"][pt]
    mcfg.base.propulsion.n_cells = float(pub["cells"])
    p = build_multipoint_problem(mcfg, driver=False, mda_retry=True)
    for k, x in pub["dv"].items():
        p.set_val(k, x)
    p.run_model()
    assert float(p.get_val("flight_score")[0]) == pub["score"]["flight_score"]
    assert float(p.get_val("M2")[0]) == pub["score"]["M2"]
    assert float(p.get_val("M3")[0]) == pub["score"]["M3"]


# --- RDO composed with multipoint, f_build active -----------------------------------------------------------------------
def test_rdo_reduced_nodes_differ_in_f_build_and_designs_are_shared():
    from aircrafter2.problems.multipoint import load_multipoint_config
    from aircrafter2.robustness.rdo_multipoint import build_robust_multipoint, mp_params

    warnings.filterwarnings("ignore")
    mcfg = load_multipoint_config(CONFIGS / "dbf2025_multipoint_rdo.yaml")
    names = [p.name for p in mp_params(mcfg)]
    assert names == ["f_build", "f_tb", "m_avionics", "lap.wind"]
    mcfg.base.aero.num_y = 11
    mcfg.base.robust.active = ["f_build"]  # d = 1: N = 3 nodes
    prob = build_robust_multipoint(mcfg, driver=False)
    prob.run_model()
    N = prob.rdo["N"]
    assert N == 3
    fb = [float(prob.get_val(f"nodes.n{j}.points.{pt}.f_build")[0]) for j in range(N) for pt in ("M2", "M3")]
    assert fb[0] == fb[1] == 1.0  # reference node at the nominal, the same aircraft at both points
    assert len(set(np.round(fb, 12))) == 3
    m = [float(prob.get_val(f"nodes.n{j}.points.M2.m_total")[0]) for j in range(N)]
    assert len(set(np.round(m, 9))) == 3
    for j in range(1, N):  # design variables shared by every node
        assert prob.get_val(f"nodes.n{j}.b") == pytest.approx(prob.get_val("nodes.n0.b"))


# --- the wind limit ----------------------------------------------------------------------------------------------------
def test_headwind_beyond_the_wind_limit_is_an_analysis_failure_not_a_score():
    """Lap 1 starts at V_stall into the wind: a headwind at V_stall leaves no ground speed (the aircraft would not fly).
    The lap model raises AnalysisError (sampling: 'lap_invalid', reported as the wind limit), never returns a score."""
    from openmdao.core.analysis_error import AnalysisError

    from aircrafter2.mission.smooth_lap import SmoothLapModel

    prob = om.Problem(reports=False)
    prob.model.add_subsystem("lap", SmoothLapModel())
    prob.setup()
    prob.run_model()
    vstall = float(prob.get_val("lap.Vstall")[0])
    assert np.isfinite(prob.get_val("lap.laps_frac")[0])
    prob.set_val("lap.wind", 1.05 * vstall)
    with pytest.raises(AnalysisError, match="non-physical lap"):
        prob.run_model()
