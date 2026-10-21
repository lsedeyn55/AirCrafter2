"""The single-point configs whose effective values the placeholders migration changed (most recent line wins:
docs/PLACEHOLDERS_RULESETS_PLAN.md). Their regression references (configs_reference.json, kv_dv_reference.json) are
NOT regenerated until the user approves; the reference tests mark them strict xfail, so a test fails loudly if a
changed config ever matches its old reference again (or an unchanged one stops matching)."""

import pytest

CHANGED_BY_MIGRATION = {"full_mdo.yaml", "full_mdo_twin.yaml", "full_mdo_rdo.yaml", "full_mdo_twin_rdo.yaml"}
REASON = ("placeholders migration: newer T-Motor/Woombah motor, battery, x_motor and take-off distance values "
          "(docs/PLACEHOLDERS_RULESETS_PLAN.md); reference not regenerated pending the user's go-ahead")


def ref_params(names):
    return [pytest.param(n, marks=pytest.mark.xfail(strict=True, reason=REASON)) if n in CHANGED_BY_MIGRATION else n
            for n in names]
