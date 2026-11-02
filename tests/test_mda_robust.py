"""MDA robustness (docs/MDA_ROBUSTNESS_PLAN.md): RetryNewton's continuation fallback (robust mode, F1) and the warm
path of studies/results_2026.py optimise (F2), both behind the problem-config switch mda_robust (default false).

The low-speed designs (tests/data/mda_low_speed_designs.json) are the cold-start smoke's failing SLSQP step (iter3)
and the stored low-speed optimum of the pre-fix smoke (pf_before): at both, M3's T = D balance has a second, sub-stall
root, and Newton from a far start diverges."""

import importlib.util
import json
import warnings

import numpy as np
import openmdao.api as om
import pytest
from openmdao.core.analysis_error import AnalysisError

from .conftest import ROOT

DATA = ROOT / "tests" / "data"
CFG26 = ROOT / "configs" / "dbf2026_multipoint.yaml"
# the configs with the switch on: the 2025-26 problem, its RDO and the as-built Humpback (both extend it)
ROBUST_ON = {"dbf2026_multipoint.yaml", "dbf2026_multipoint_rdo.yaml", "humpback_2026.yaml"}


def designs():
    return json.loads((DATA / "mda_low_speed_designs.json").read_text())


def _val(p, name):
    return float(np.ravel(p.get_val(name))[0])


# --- RetryNewton on a 1-D toy: R = atan(x - c), Newton diverges from |x - c| > 1.39 -----------------------------

class _Atan(om.ImplicitComponent):
    def setup(self):
        self.add_input("c", 0.0)
        self.add_output("x", 0.0)
        self.declare_partials("x", ["x", "c"])

    def apply_nonlinear(self, inputs, outputs, residuals):
        residuals["x"] = np.arctan(outputs["x"] - inputs["c"])

    def linearize(self, inputs, outputs, J):
        # floored so a diverged iterate stays non-singular: it fails as the MDA does (AnalysisError), not in the LU
        d = np.maximum(1.0 / (1.0 + (outputs["x"] - inputs["c"]) ** 2), 1e-8)
        J["x", "x"] = d
        J["x", "c"] = -d


def _toy(robust):
    from aircrafter2.robustness.rdo_problem import RetryNewton

    p = om.Problem(reports=False)
    p.model.add_subsystem("ivc", om.IndepVarComp("c", 0.0), promotes=["*"])
    g = p.model.add_subsystem("g", om.Group(), promotes=["*"])
    g.add_subsystem("atan", _Atan(), promotes=["*"])
    newton = RetryNewton(robust=robust, solve_subsystems=False, maxiter=30, atol=1e-10, rtol=1e-10, iprint=-1,
                         err_on_non_converge=True)
    g.nonlinear_solver = newton
    g.linear_solver = om.DirectSolver()
    p.setup()
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        p.run_model()  # converges at c = 0 (x = 0), cold
    return p, newton


@pytest.mark.parametrize("target, labels", [
    (4.0, [("current", False), ("continuation 4", True)]),                            # 4 steps of 1
    (8.0, [("current", False), ("continuation 4", False), ("continuation 8", True)]),  # 4 of 2 fail, 8 of 1 work
])
def test_retry_newton_continuation(target, labels):
    """Robust mode: the failed current start is followed by the continuation; "own last converged" equals the current
    state (the optimiser-step case) and is skipped; the log keeps the 3-tuples; the target input is held exactly."""
    p, newton = _toy(robust=True)
    p.set_val("c", target)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        p.run_model()
    assert [(e[1], e[2]) for e in newton.log if e[0] == 2] == labels
    assert all(len(e) == 3 for e in newton.log)
    assert _val(p, "x") == pytest.approx(target, abs=1e-9)
    assert p.model.g._inputs["atan.c"][0] == target  # exactly the target
    assert newton.good_in[0] == target


def test_retry_newton_continuation_fails_restores_target():
    """Every fallback fails: AnalysisError, the target input restored exactly, every attempt logged."""
    p, newton = _toy(robust=True)
    p.set_val("c", 100.0)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        with pytest.raises(AnalysisError):
            p.run_model()
    assert [(e[1], e[2]) for e in newton.log if e[0] == 2] == [
        ("current", False), ("continuation 4", False), ("continuation 8", False), ("cold", False)]
    assert p.model.g._inputs["atan.c"][0] == 100.0


def test_retry_newton_default_unchanged():
    """robust = False: today's order (current, own last converged, cold), no continuation; good_in stored only."""
    p, newton = _toy(robust=False)
    assert newton.good_in is not None
    p.set_val("c", 4.0)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        with pytest.raises(AnalysisError):
            p.run_model()
    assert [(e[1], e[2]) for e in newton.log if e[0] == 2] == [
        ("current", False), ("own last converged", False), ("cold", False)]


def test_retry_newton_off_reference_node_uses_reference_inputs():
    """A robust node that has never converged continues from the reference node's converged inputs and state."""
    p, newton = _toy(robust=True)  # this one plays the reference node
    q, other = _toy(robust=True)
    other.good = other.good_in = None
    other.reference = newton
    q.set_val("c", 4.0)
    q.model.g._outputs.set_val(np.array([4.0 + 3.0]))  # a far "current" (never used: reference first)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        q.run_model()
    assert [(e[1], e[2]) for e in other.log if e[0] == 2] == [("reference", False), ("continuation 4", True)]
    assert _val(q, "x") == pytest.approx(4.0, abs=1e-9)


# --- the 2025-26 multipoint problem ---------------------------------------------------------------------------------

def _problem(robust):
    from aircrafter2.problems.multipoint import build_multipoint_problem, load_multipoint_config

    mcfg = load_multipoint_config(CFG26)
    mcfg.base.mda_robust = robust
    return build_multipoint_problem(mcfg, driver=False, mda_retry=True)


def _logs(p):
    return {pt: p.model._get_subsystem(f"points.{pt}.mda").nonlinear_solver.log for pt in ("M2", "M3")}


def _set(p, dv):
    for k, x in dv.items():
        p.set_val(k, x)


def test_slsqp_step_converges_by_continuation():
    """The smoke's failing SLSQP step (Humpback's start -> iter3): converges with the switch on, M3 v 15.4786 m/s on the
    real root (not the sub-stall one), by a continuation.

    Not asserted: the switch-off failure. On the development machine (3 threads) M2's diverged attempt reaches a
    singular Jacobian there, and OpenMDAO's DirectSolver then spends ~10 min and > 3 GB on its SVD diagnosis before a
    RuntimeError (docs/MDA_ROBUSTNESS_PLAN.md, status); elsewhere it ends in AnalysisError."""
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        p = _problem(True)
        p.run_model()
        _set(p, designs()["iter3"])
        p.run_model()
    assert _val(p, "points.M3.v") == pytest.approx(15.4786, rel=1e-5)
    assert any(e[1].startswith("continuation") and e[2] for e in _logs(p)["M3"])


def test_fallback_only_at_humpback_start():
    """At Humpback's start every solve converges at its first attempt, and the switch changes nothing (bit for bit)."""
    keys = ["flight_score"] + [f"points.{pt}.{k}" for pt in ("M2", "M3")
                               for k in ("v", "alpha_cruise", "m_total", "SM")]
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        vals = {}
        for robust in (True, False):
            p = _problem(robust)
            p.run_model()
            vals[robust] = [_val(p, k) for k in keys]
            if robust:
                for log in _logs(p).values():
                    assert log and all(e[1] == "current" and e[2] for e in log)
    assert vals[True] == vals[False]


def test_warm_jump_to_low_speed_optimum():
    """Humpback's converged state -> the stored low-speed optimum (the --start / integer() case): converges at the first
    attempt."""
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        p = _problem(True)
        p.run_model()
        _set(p, designs()["pf_before"])
        p.run_model()
    for log in _logs(p).values():
        assert [(e[1], e[2]) for e in log] == [("current", True), ("current", True)]
    assert _val(p, "flight_score") == pytest.approx(5.981649, rel=1e-5)


def test_mda_robust_default_off():
    """Only the 2025-26 configs turn the switch on; every 2024-25 and single-point config keeps today's solver path."""
    from aircrafter2.problems.full_mdo import FullMDOConfig, load_full_config
    from aircrafter2.problems.multipoint import load_multipoint_config

    assert FullMDOConfig.model_fields["mda_robust"].default is False
    seen = {}
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        for path in sorted((ROOT / "configs").glob("*.yaml")):
            for loader in (load_multipoint_config, load_full_config):
                try:
                    cfg = loader(path)
                except Exception:  # noqa: BLE001 - not this kind of config
                    continue
                seen[path.name] = (cfg.base if hasattr(cfg, "base") else cfg).mda_robust
                break
    assert {"full_mdo.yaml", "dbf2025_multipoint.yaml", "dbf2026_multipoint.yaml"} <= set(seen)
    assert {n for n, on in seen.items() if on} == ROBUST_ON


def _results_2026():
    spec = importlib.util.spec_from_file_location("results_2026_study", ROOT / "studies" / "results_2026.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_optimise_warm_path_and_integer(tmp_path, monkeypatch):
    """optimise(start=the low-speed optimum, maxiter=1) and integer() on it: the warm path runs, no MDA error; integer()
    records its rounding check and runs the candidates that fit."""
    R = _results_2026()
    case = dict(dv=designs()["pf_before"], cells=8.0, props={"M2": "19x12E", "M3": "19x12E"},
                scoring={"NI_max": 2800.0, "B_max": 1500.0})
    monkeypatch.setattr(R, "RUNS", tmp_path)
    monkeypatch.setattr(R, "load", lambda tag: case)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        out = R.optimise("t_warm", start="case", maxiter=1)
        assert out["error"] is None and (out["ok"] or out["its"] >= 1)
        assert out["warm_path"] is True
        outs = R.integer("case", maxiter=1)
    check = json.loads((tmp_path / "case_int_check.json").read_text())
    assert "error" not in check["check"]
    assert {tuple(c) for c in check["candidates"]} == {(95, 31), (96, 32)}  # n_p 95.28, n_c 31.76
    assert all(np.isfinite(c["fuselage_margin"]) for c in check["check"].values())
    assert len(outs) == len(check["run"]) >= 1
    for o in outs:
        assert o["error"] is None, o["error"]
        assert o["warm_path"] is True


class _Flat(om.ImplicitComponent):
    """R = x**3 - c: dR/dx = 0 at x = 0, a singular Jacobian at the start."""

    def setup(self):
        self.add_input("c", 1.0)
        self.add_output("x", 0.0)
        self.declare_partials("x", ["x", "c"])

    def apply_nonlinear(self, inputs, outputs, residuals):
        residuals["x"] = outputs["x"] ** 3 - inputs["c"]

    def linearize(self, inputs, outputs, J):
        J["x", "x"] = 3 * outputs["x"] ** 2
        J["x", "c"] = -1.0


@pytest.mark.parametrize("quiet", [True, False])
def test_quiet_direct_solver_singular(quiet):
    """A singular Jacobian: QuietDirectSolver raises AnalysisError (a failed attempt for RetryNewton); OpenMDAO's own
    DirectSolver raises RuntimeError (unchanged with the switch off)."""
    from aircrafter2.robustness.rdo_problem import QuietDirectSolver

    p = om.Problem(reports=False)
    g = p.model.add_subsystem("g", om.Group(), promotes=["*"])
    g.add_subsystem("flat", _Flat(), promotes=["*"])
    g.nonlinear_solver = om.NewtonSolver(solve_subsystems=False, maxiter=5, iprint=-1, err_on_non_converge=True)
    g.linear_solver = QuietDirectSolver() if quiet else om.DirectSolver()
    p.setup()
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        with pytest.raises(AnalysisError if quiet else RuntimeError) as exc:
            p.run_model()
    if quiet:
        assert "singular Jacobian in 'g'" in str(exc.value)
    else:
        assert not isinstance(exc.value, AnalysisError)
