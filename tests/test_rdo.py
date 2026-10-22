"""Robust design optimisation: quadrature, statistics component, robust totals, and RDO-off regression."""

import json
from math import comb
from pathlib import Path

import numpy as np
import openmdao.api as om
import pytest
from openmdao.utils.assert_utils import assert_check_partials, assert_check_totals

from aircrafter2.problems.full_mdo import load_full_config
from aircrafter2.robustness.quadrature import gauss_1d, smolyak, sobol, udr
from aircrafter2.robustness.stats import RobustStats
from aircrafter2.robustness.uncertainty import Dist, select_parameters

from .conftest import CONFIGS

REFERENCE = Path(__file__).parent / "data" / "rdo_off_reference.json"
DISTS = [Dist("a", "uniform", 2.0, 0.5), Dist("b", "normal", -1.0, 0.3), Dist("c", "triangular", 5.0, 1.0)]


# --- quadrature against analytic moments ------------------------------------------------------------
@pytest.mark.parametrize("kind,m2,m4", [("uniform", 1 / 3, 1 / 5), ("normal", 1.0, 3.0), ("triangular", 1 / 6, 1 / 15)])
def test_gauss_rules_reproduce_moments(kind, m2, m4):
    xi, w = gauss_1d(kind, 3)  # exact up to degree 5
    assert w.sum() == pytest.approx(1.0, rel=1e-14)
    assert w @ xi == pytest.approx(0.0, abs=1e-14)
    assert w @ xi**2 == pytest.approx(m2, rel=1e-12)
    assert w @ xi**4 == pytest.approx(m4, rel=1e-12)


def test_udr_is_exact_for_additive_functions():
    # f = 3 a^2 + b^2 + 0.5 c: additive, and f^2 has degree <= 4 in each variable, so UDR (3 points, exact to
    # degree 5) gives the exact mean and variance.
    a, b, c = DISTS
    ea2 = a.mean**2 + a.scale**2 / 3
    ea4 = a.mean**4 + 6 * a.mean**2 * a.scale**2 / 3 + a.scale**4 / 5
    eb2 = b.mean**2 + b.scale**2
    eb4 = sum(comb(4, k) * b.mean ** (4 - k) * b.scale**k * m for k, m in [(0, 1), (2, 1), (4, 3)])
    mean = 3 * ea2 + eb2 + 0.5 * c.mean
    var = 9 * (ea4 - ea2**2) + (eb4 - eb2**2) + 0.25 * c.scale**2 / 6
    r = udr(DISTS)
    P = r.nodes
    assert r.n == 2 * len(DISTS) + 1 and np.allclose(P[0], [d.mean for d in DISTS])
    mu, v = r.moments(3 * P[:, 0] ** 2 + P[:, 1] ** 2 + 0.5 * P[:, 2])
    assert mu == pytest.approx(mean, rel=1e-12)
    assert v == pytest.approx(var, rel=1e-10)


def test_smolyak_captures_a_cross_term_that_udr_misses():
    # f = a b: E = E[a] E[b]; Var = E[a^2] E[b^2] - (E[a] E[b])^2. UDR misses Var[a] Var[b].
    a, b, _ = DISTS
    ea2, eb2 = a.mean**2 + a.std**2, b.mean**2 + b.std**2
    var = ea2 * eb2 - (a.mean * b.mean) ** 2
    s = smolyak(DISTS, 2)
    mu, v = s.moments(s.nodes[:, 0] * s.nodes[:, 1])
    assert mu == pytest.approx(a.mean * b.mean, rel=1e-12) and v == pytest.approx(var, rel=1e-10)
    r = udr(DISTS)
    _, vu = r.moments(r.nodes[:, 0] * r.nodes[:, 1])
    assert vu == pytest.approx(var - a.std**2 * b.std**2, rel=1e-10)


def test_sobol_rule_has_the_reference_node_first_and_converges():
    s = sobol(DISTS, 2**12, seed=1)
    assert np.allclose(s.nodes[0], [d.mean for d in DISTS]) and s.a[0] == 0.0
    mu, v = s.moments(s.nodes[:, 0])
    assert mu == pytest.approx(DISTS[0].mean, abs=1e-4) and v == pytest.approx(DISTS[0].std**2, rel=1e-2)


def test_screening_selects_the_smallest_covering_set():
    contrib = {"f": {"p1": 6.0, "p2": 3.0, "p3": 1.0, "p4": 0.0}, "g": {"p1": 0.0, "p2": 0.0, "p3": 9.0, "p4": 1.0},
               "h": {"p1": 0.0, "p2": 0.0, "p3": 0.0, "p4": 0.0}}  # zero variance: skipped
    sel, cov, ok = select_parameters(contrib, threshold=0.85, max_dim=3)
    assert ok and sorted(sel) == ["p1", "p2", "p3"] and cov["f"] == pytest.approx(1.0)
    sel, cov, ok = select_parameters(contrib, threshold=0.99, max_dim=2)
    assert not ok and len(sel) == 2


# --- statistics component ---------------------------------------------------------------------------
@pytest.mark.parametrize("rule", [udr(DISTS), smolyak(DISTS, 2)], ids=["udr", "smolyak"])
def test_stats_partials(rule):
    p = om.Problem(reports=False)
    p.model.add_subsystem("s", RobustStats(rule=rule, quantities={"f": 1e-6, "g": 1e-3}, objective="f", k=1.5,
                                           sign=1.0, beta=2.0), promotes=["*"])
    p.setup(force_alloc_complex=True)
    rng = np.random.default_rng(3)
    for j in range(rule.n):
        p.set_val(f"f_n{j}", 10.0 + rng.normal())
        p.set_val(f"g_n{j}", 2.0)  # zero variance: sigma = eps, derivative finite
    p.run_model()
    f = np.array([p.get_val(f"f_n{j}")[0] for j in range(rule.n)])
    mu, var = rule.moments(f)
    assert p.get_val("f_mu")[0] == pytest.approx(mu, rel=1e-12)
    assert p.get_val("f_robust")[0] == pytest.approx(mu + 1.5 * np.sqrt(max(var, 0) + 1e-12), rel=1e-9)
    # zero variance: sigma = eps (PSD rule), or eps sqrt(1.5) with the smooth positive part var+ = eps^2 / 2
    assert p.get_val("g_sigma")[0] == pytest.approx(1e-3 * (1.0 if rule.psd else np.sqrt(1.5)), rel=1e-6)
    assert_check_partials(p.check_partials(method="cs", out_stream=None), atol=1e-8, rtol=1e-8)


# --- robust problem totals (small d, coarse mesh) ---------------------------------------------------
@pytest.fixture(scope="module")
def robust_small():
    from aircrafter2.robustness.rdo_problem import build_robust_problem

    cfg = load_full_config(CONFIGS / "full_mdo_rdo_fixed_kv.yaml")  # the fixed-motor RDO problem (Kv uncertain)
    cfg.aero.num_y = 11
    cfg.robust.active = ["Kv"]  # d = 1, N = 3
    prob = build_robust_problem(cfg)
    prob.run_model()
    return prob


def test_robust_nodes_share_the_design_and_carry_the_parameters(robust_small):
    p = robust_small
    assert p.rdo["N"] == 3
    kv = [p.get_val(f"nodes.n{j}.Kv")[0] for j in range(3)]
    assert kv[0] == pytest.approx(900.0) and kv[1] < 900.0 < kv[2]
    for j in range(3):
        assert p.get_val(f"nodes.n{j}.b")[0] == p.get_val("b")[0]
        assert p.get_val(f"nodes.n{j}.lap.wind")[0] == pytest.approx(1.5)  # inactive: held at its mean
        assert p.get_val(f"nodes.n{j}.m_pay")[0] == pytest.approx(1.5)


def test_robust_check_totals(robust_small):
    data = robust_small.check_totals(of=["stats.score_robust", "stats.energy_margin_lo"], method="fd", step=1e-6,
                                     form="central", out_stream=None)
    assert_check_totals(data, atol=1e-4, rtol=5e-3)


# --- regression: with RDO off, the deterministic problem is unchanged ---------------------------------
# The reference was made from full_mdo.yaml / full_mdo_twin.yaml before Kv became a design variable; their fixed-Kv
# versions must still reproduce it (the reference file is never regenerated for this). Since the USYD mass calibration
# they do so with the pre-calibration mass model (tests/migration.py pre_calibration).
@pytest.mark.parametrize("cfg_name,ref_name", [("full_mdo_fixed_kv.yaml", "full_mdo.yaml"),
                                               ("full_mdo_twin_fixed_kv.yaml", "full_mdo_twin.yaml")])
def test_rdo_off_matches_pre_rdo_reference(cfg_name, ref_name):
    from .migration import pre_calibration
    from .rdo_off_snapshot import snapshot

    assert not load_full_config(CONFIGS / cfg_name).robust.enabled
    ref = json.loads(REFERENCE.read_text())[ref_name]
    now = snapshot(cfg_name, transform=pre_calibration)
    assert list(now) == list(ref)
    assert [c[0] for c in now["cons"]] == [c[0] for c in ref["cons"]]  # same constraints, same order
    assert now["cons"] == ref["cons"] and now["dvs"] == ref["dvs"] and now["objs"] == ref["objs"]
    # The totals depend on the BLAS thread count at the 1e-9 level (summation order in the linear solves: e.g.
    # stall_margin|tc_scale differs by 2.5e-9 relative between OMP_NUM_THREADS=1 and 3 on the same code), and three
    # SLSQP iterations amplify that to ~1.3e-7 in the design; a real change to the model moves them by far more.
    for part, rtol in [("outputs", 1e-9), ("con_values", 1e-9), ("totals", 1e-7), ("after_3_iterations", 1e-6)]:
        assert now[part].keys() == ref[part].keys(), part
        for k in ref[part]:
            np.testing.assert_allclose(now[part][k], ref[part][k], rtol=rtol, atol=1e-12, err_msg=f"{part}: {k}")


# --- post-optimality Monte Carlo: relative spreads (old form) and distributions (shared spreads) ---------
def test_monte_carlo_accepts_spreads_and_distributions():
    from aircrafter2.robustness.suite import monte_carlo

    p = om.Problem(reports=False)
    p.model.add_subsystem("c", om.ExecComp("y = 2 * a + w"), promotes=["*"])
    p.setup()
    p.set_val("a", 10.0)
    p.set_val("w", 0.0)
    old = monte_carlo(p, {"a": 0.1}, ["y"], n=200)
    assert old["failed"] == 0 and np.all(np.abs(old["samples"][:, 0] - 20.0) <= 2.0 + 1e-12)
    new = monte_carlo(p, [Dist("a", "uniform", 10.0, 1.0), Dist("w", "uniform", 1.5, 1.5)], ["y"], n=200)
    y = new["samples"][:, 0]
    assert y.min() >= 18.0 - 1e-12 and y.max() <= 25.0 + 1e-12 and abs(y.mean() - 21.5) < 0.3
    assert p.get_val("a")[0] == 10.0 and p.get_val("w")[0] == 0.0  # inputs restored
