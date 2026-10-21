"""Configs whose regression references are expected to differ after a deliberate change of their effective values.

The placeholders migration (docs/PLACEHOLDERS_RULESETS_PLAN.md) changed full_mdo, full_mdo_twin, full_mdo_rdo and
full_mdo_twin_rdo; their entries in configs_reference.json and kv_dv_reference.json were regenerated on 2026-10-22
with the user's go-ahead (the unchanged entries reproduced bit for bit), so the set is empty again. For a future
deliberate change, list the configs here: the reference tests then mark them strict xfail until the references are
regenerated, so a test fails loudly if a changed config still matches its old reference (or an unchanged one stops
matching)."""

import pytest

CHANGED_BY_MIGRATION = set()
REASON = "effective values changed deliberately; reference not regenerated pending the user's go-ahead"


def ref_params(names):
    return [pytest.param(n, marks=pytest.mark.xfail(strict=True, reason=REASON)) if n in CHANGED_BY_MIGRATION else n
            for n in names]
