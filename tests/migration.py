"""Configs whose regression references are expected to differ after a deliberate change of their effective values.

The placeholders migration (docs/PLACEHOLDERS_RULESETS_PLAN.md) changed full_mdo, full_mdo_twin, full_mdo_rdo and
full_mdo_twin_rdo; their entries in configs_reference.json and kv_dv_reference.json were regenerated on 2026-10-22
with the user's go-ahead (the unchanged entries reproduced bit for bit), so the set is empty again. For a future
deliberate change, list the configs here: the reference tests then mark them strict xfail until the references are
regenerated, so a test fails loudly if a changed config still matches its old reference (or an unchanged one stops
matching)."""

import pytest

# USYD mass calibration (2026-10-23, docs/MASS_CALIBRATION_PLAN.md): every full config's mass model changed
CHANGED_BY_MIGRATION = {"full_mdo.yaml", "full_mdo_twin.yaml", "full_mdo_fixed_kv.yaml", "full_mdo_twin_fixed_kv.yaml",
                        "full_mdo_rdo.yaml", "full_mdo_twin_rdo.yaml", "full_mdo_rdo_fixed_kv.yaml",
                        "full_mdo_twin_rdo_fixed_kv.yaml"}
REASON = "effective values changed deliberately; reference not regenerated pending the user's go-ahead"


def ref_params(names):
    return [pytest.param(n, marks=pytest.mark.xfail(strict=True, reason=REASON)) if n in CHANGED_BY_MIGRATION else n
            for n in names]


# The mass model of every result published before the USYD mass calibration (docs/MASS_CALIBRATION_PLAN.md): the
# previous placeholders.yaml values (0.5 mm spar gauge, G&G Table 13 strength), G&G's correction factors with gg tails
# (mass_garcia_gamboa_2022.yaml), one wing panel and the placeholder cargo box. The frozen references that are never
# regenerated (rdo_off_reference.json, multipoint_nominal_reference.json) are reproduced with it.
PRE_CALIBRATION_MASS = dict(options=dict(tail_mass_model="gg", n_panels=1),
                            inputs=dict(t_tube_min=5.0e-4, sigma_spar=7.2e8, k_wing=1.0, f_wing=1.12, f_ht=0.6411,
                                        f_vt=0.3414, f_cb=0.7878, f_tb=0.847, f_gear=0.0089))
PRE_CALIBRATION_BOX = dict(l_cb=0.40, w_cb=0.12, h_cb=0.12)


def pre_calibration(cfg):
    """A FullMDOConfig (or a multipoint config's base) with the pre-calibration mass model, in place."""
    cfg.mass.options = cfg.mass.options.model_copy(update=PRE_CALIBRATION_MASS["options"])
    cfg.mass.inputs.update(PRE_CALIBRATION_MASS["inputs"])
    for k, v in PRE_CALIBRATION_BOX.items():
        setattr(cfg.geometry, k, v)
    return cfg
