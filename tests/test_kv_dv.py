"""Motor Kv as a design variable: the rewind component, the default and fixed-Kv configs, totals w.r.t. Kv, the
current constraint, RDO with Kv_factor / Rm_ref, the API helpers and the Kv-DV regression reference."""

import json

import numpy as np
import pytest
from openmdao.utils.assert_utils import assert_check_partials, assert_check_totals

from aircrafter2.problems.full_mdo import (FullMDOConfig, build_full_problem, check_kv_robust, default_full_dvs,
                                           full_constraints, kv_is_dv, load_full_config)
from aircrafter2.propulsion.motor_scaling import MotorRewind

from .conftest import CONFIGS, ROOT, run_component
from .migration import ref_params

KV_REFERENCE = ROOT / "tests" / "data" / "kv_dv_reference.json"


def _coarse(name):
    cfg = load_full_config(CONFIGS / name)
    cfg.aero.num_y = 11
    return cfg


# --- the rewind component ---------------------------------------------------------------------------------
def test_motor_rewind_partials():
    p = run_component(MotorRewind(), Kv=1130.0, Kv_factor=1.03, Kv_ref=880.0, Rm_ref=0.047, I0_ref=1.2)
    assert_check_partials(p.check_partials(method="cs", out_stream=None), atol=1e-10, rtol=1e-10)


def test_motor_rewind_keeps_km_and_reproduces_the_reference_motor():
    p = run_component(MotorRewind(), Kv_ref=900.0, Rm_ref=0.05, I0_ref=1.0)
    km = []
    for kv in (600.0, 900.0, 1400.0):
        p.set_val("c.Kv", kv)
        p.run_model()
        km.append(1.0 / (p.get_val("c.Kv_eff")[0] * np.sqrt(p.get_val("c.Rm")[0])))
        assert p.get_val("c.I0")[0] / p.get_val("c.Kv_eff")[0] == pytest.approx(1.0 / 900.0, rel=1e-14)  # Q0 held
    assert np.ptp(km) < 1e-12 * km[0]
    p.set_val("c.Kv", 900.0)
    p.run_model()
    assert (p.get_val("c.Rm")[0], p.get_val("c.I0")[0], p.get_val("c.Kv_eff")[0]) == (0.05, 1.0, 900.0)  # exact


# --- configs ---------------------------------------------------------------------------------------------------
def test_default_configs_optimise_kv_and_fixed_kv_configs_do_not():
    assert "Kv" in default_full_dvs()
    for name in ["full_mdo.yaml", "full_mdo_twin.yaml", "full_mdo_rdo.yaml", "full_mdo_twin_rdo.yaml"]:
        assert kv_is_dv(load_full_config(CONFIGS / name)), name
    for name in ["full_mdo_fixed_kv.yaml", "full_mdo_twin_fixed_kv.yaml", "full_mdo_rdo_fixed_kv.yaml",
                 "full_mdo_twin_rdo_fixed_kv.yaml"]:
        assert not kv_is_dv(load_full_config(CONFIGS / name)), name
    default, fixed = load_full_config(CONFIGS / "full_mdo.yaml"), load_full_config(CONFIGS / "full_mdo_fixed_kv.yaml")
    d, f = default.model_dump(), fixed.model_dump()
    d["design_variables"].pop("Kv")
    # the fixed-Kv config is the default minus the Kv DV, plus the AIAC-era motor values it pins (the placeholders
    # migration moved the default to the newer T-Motor values; docs/development/PLACEHOLDERS_RULESETS_PLAN.md), nothing else
    pinned = dict(e_spec=150.0, Kv=900.0, Rm=0.05, I0=1.0, m_motor=0.15, m_esc=0.06, m_prop=0.03, m_avionics=0.25)
    assert {k: f["propulsion"][k] for k in pinned} == pinned
    assert f["takeoff_distance"] == 100.0 and f["stability"]["x_motor"] == -0.30
    for x in (d, f):
        for k in pinned:
            x["propulsion"].pop(k)
        x.pop("takeoff_distance")
        x["stability"].pop("x_motor")
    assert d == f
    names = lambda c: [n for n, _ in full_constraints(c)]  # noqa: E731
    assert "I_margin" in names(default) and "I_margin" not in names(fixed)
    assert names(fixed) == [n for n in names(default) if n != "I_margin"]  # the rest in the old order
    assert names(default).index("I_margin") == names(default).index("rpm_margin") + 1


def test_fixed_design_variables_must_name_design_variables(tmp_path):
    (tmp_path / "bad.yaml").write_text(f"extends: {CONFIGS / 'full_mdo.yaml'}\nfixed_design_variables: [Kvv]\n"
                                       f"mass_config: {CONFIGS / 'mass_garcia_gamboa_2022.yaml'}\n")
    with pytest.raises(ValueError, match="Kvv"):
        load_full_config(tmp_path / "bad.yaml")


def test_robust_parameters_must_match_the_motor_mode():
    rdo = load_full_config(CONFIGS / "full_mdo_rdo.yaml")  # Kv a DV
    for bad in (["Kv"], ["Rm", "m_pay"]):
        data = rdo.model_dump()
        data["robust"]["active"] = bad
        with pytest.raises(ValueError, match="Kv_factor"):
            FullMDOConfig(**data)
    fixed = load_full_config(CONFIGS / "full_mdo_rdo_fixed_kv.yaml")
    data = fixed.model_dump()
    data["robust"]["active"] = ["Kv_factor"]
    with pytest.raises(ValueError, match="Kv is fixed"):
        FullMDOConfig(**data)
    check_kv_robust(rdo)  # the shipped configs are consistent
    check_kv_robust(fixed)


# --- the full model -------------------------------------------------------------------------------------------
@pytest.fixture(scope="module")
def kv_models():
    out = {}
    for name in ["full_mdo.yaml", "full_mdo_fixed_kv.yaml", "same motor fixed"]:
        if name == "same motor fixed":  # full_mdo.yaml with its own reference motor fixed (Kv not a DV)
            cfg = _coarse("full_mdo.yaml")
            cfg.design_variables.pop("Kv")
            cfg.initial.pop("Kv", None)
        else:
            cfg = _coarse(name)
        prob = build_full_problem(cfg)
        prob.run_model()
        out[name] = prob
    return out


def test_kv_at_its_reference_reproduces_the_fixed_motor_exactly(kv_models):
    a, b = kv_models["full_mdo.yaml"], kv_models["same motor fixed"]
    kv_ref = load_full_config(CONFIGS / "full_mdo.yaml").propulsion.Kv  # configs/placeholders.yaml
    assert a.get_val("Kv")[0] == a.get_val("Kv_ref")[0] == kv_ref
    for k in ["ThreeLapTime", "m_total", "v", "I_static", "T_static", "rpm_margin", "SM", "energy_margin", "Rm", "I0",
              "score"]:
        assert a.get_val(k)[0] == b.get_val(k)[0], k


def test_full_model_totals_wrt_kv(kv_models):
    p = kv_models["full_mdo.yaml"]
    kv0 = float(p.get_val("Kv")[0])  # restored below: the fixture is shared
    p.set_val("Kv", 1000.0)
    p.run_model()
    data = p.check_totals(of=["score", "I_margin", "rpm_margin"], wrt=["Kv"], method="fd", step=1e-3, form="central",
                          out_stream=None)
    assert_check_totals(data, atol=1e-6, rtol=1e-4)
    p.set_val("Kv", kv0)
    p.run_model()


def test_suite_refuses_computed_motor_values(kv_models):
    from aircrafter2.robustness.suite import check_settable, sensitivities

    p = kv_models["full_mdo.yaml"]
    with pytest.raises(ValueError, match="Rm_ref"):
        check_settable(p, ["Rm"])
    check_settable(p, ["Rm_ref", "Kv_factor", "Kv", "m_pay"])
    with pytest.raises(ValueError, match="duplicate"):
        sensitivities(p, "score", ["Kv", "Kv"])
    check_settable(kv_models["full_mdo_fixed_kv.yaml"], ["Rm", "Kv"])  # inputs with a fixed motor


def test_api_motor_helpers(kv_models):
    from aircrafter2.api import motor_report, reanalyse_with_motor, sensitivity_assumptions, validate_kv_dv

    cfg = _coarse("full_mdo.yaml")
    p = kv_models["full_mdo.yaml"]
    mr = motor_report(p, cfg)
    pr = cfg.propulsion  # the reference motor (configs/placeholders.yaml)
    assert mr["kv_dv"] and mr["Kv"] == pr.Kv and mr["Rm"] == pr.Rm and mr["set_by"].startswith("interior")
    assert "Kv" not in sensitivity_assumptions(cfg) and "Rm_ref" in sensitivity_assumptions(cfg)
    assert "Kv" in sensitivity_assumptions(_coarse("full_mdo_fixed_kv.yaml"))
    assert validate_kv_dv(500, 1500, 900) == []
    assert len(validate_kv_dv(1500, 500, 900)) == 2  # bounds reversed, start outside
    assert "within 100-5000" in validate_kv_dv(50, 1500, 900)[0]
    # the real-motor re-analysis with the reference motor itself reproduces the design exactly
    dv = {k: float(p.get_val(k)[0]) for k in cfg.design_variables}
    ref = {"score": float(p.get_val("score")[0])}
    rr = reanalyse_with_motor(cfg, dv, pr.Kv, pr.Rm, pr.I0, pr.m_motor, reference=ref)
    assert rr["change"]["score"] == pytest.approx(0.0, abs=1e-10 * ref["score"])  # MDA tolerance (fresh start)


# --- RDO with Kv a design variable ------------------------------------------------------------------------------
def test_rdo_kv_factor_nodes_differ_while_the_design_kv_is_shared():
    from aircrafter2.robustness.rdo_problem import build_robust_problem

    cfg = _coarse("full_mdo_rdo.yaml")
    cfg.robust.active = ["Kv_factor"]  # d = 1, N = 3
    cfg.initial["Kv"] = 950.0
    p = build_robust_problem(cfg, driver=False)
    p.run_model()
    f = [p.get_val(f"nodes.n{j}.Kv_factor")[0] for j in range(3)]
    assert f[0] == pytest.approx(1.0) and f[1] < 1.0 < f[2]
    for j in range(3):
        assert p.get_val(f"nodes.n{j}.Kv")[0] == p.get_val("Kv")[0] == 950.0  # the DV, shared
        assert p.get_val(f"nodes.n{j}.Kv_eff")[0] == pytest.approx(950.0 * f[j], rel=1e-14)
        pr = cfg.propulsion  # Rm_ref (Kv_ref / Kv_eff)^2 with the reference motor of configs/placeholders.yaml
        assert p.get_val(f"nodes.n{j}.Rm")[0] == pytest.approx(pr.Rm * (pr.Kv / (950.0 * f[j])) ** 2, rel=1e-12)
    assert len({p.get_val(f"nodes.n{j}.I_static")[0] for j in range(3)}) == 3  # the nodes really differ


# --- regression: the Kv-DV defaults against their own reference ----------------------------------------------------
@pytest.mark.parametrize("cfg_name", ref_params(["full_mdo.yaml", "full_mdo_twin.yaml"]))
def test_kv_dv_defaults_match_reference(cfg_name):
    from .rdo_off_snapshot import snapshot

    ref = json.loads(KV_REFERENCE.read_text())[cfg_name]
    now = snapshot(cfg_name)
    assert list(now) == list(ref)
    assert now["cons"] == ref["cons"] and now["dvs"] == ref["dvs"] and now["objs"] == ref["objs"]
    for part, rtol in [("outputs", 1e-9), ("con_values", 1e-9), ("totals", 1e-7), ("after_3_iterations", 1e-6)]:
        assert now[part].keys() == ref[part].keys(), part
        for k in ref[part]:
            np.testing.assert_allclose(now[part][k], ref[part][k], rtol=rtol, atol=1e-12, err_msg=f"{part}: {k}")
