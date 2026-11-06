"""configs/placeholders.yaml and configs/rulesets/: layer order, the migration, one place per value, save in place
(docs/development/PLACEHOLDERS_RULESETS_PLAN.md)."""

import json
import re
import shutil
import warnings
from pathlib import Path

import pytest
import yaml

from aircrafter2.config.layer_files import overridden_by, read_rows, save_rows
from aircrafter2.config.loader import expand_dotted, load_yaml
from aircrafter2.problems.full_mdo import load_full_config
from aircrafter2.problems.multipoint import load_multipoint_config

ROOT = Path(__file__).resolve().parents[1]
CONFIGS = ROOT / "configs"
LAYER_FILES = [CONFIGS / "placeholders.yaml", CONFIGS / "rulesets" / "dbf_2024_25.yaml",
               CONFIGS / "rulesets" / "generic.yaml", CONFIGS / "rulesets" / "dbf_2025_26.yaml"]


def _write(path, text):
    path.write_text(text)
    return path


# --- layer order --------------------------------------------------------------------------------------------------

def test_each_layer_overrides_the_one_below(tmp_path):
    """code default < placeholders < ruleset < config < study variant, one path decided at each layer."""
    _write(tmp_path / "ph.yaml",
           "propulsion.Kv: 111.0\npropulsion.I0: 2.0\npropulsion.Rm: 0.1\npropulsion.m_esc: 0.2\n")
    _write(tmp_path / "rs.yaml", "propulsion.I0: 2.5\npropulsion.Rm: 0.2\naltitude: 123.0\n")
    _write(tmp_path / "base.yaml", "placeholders: ph.yaml\nruleset: rs.yaml\npropulsion: {Rm: 0.3, m_esc: 0.5}\n")
    _write(tmp_path / "variant.yaml", "extends: base.yaml\npropulsion: {m_esc: 0.6}\n")
    p = load_full_config(tmp_path / "variant.yaml").propulsion
    assert p.v_top == 40.0      # code default (set nowhere)
    assert p.Kv == 111.0        # placeholders
    assert p.I0 == 2.5          # ruleset over placeholders
    assert p.Rm == 0.3          # config over ruleset
    assert p.m_esc == 0.6       # variant over config
    assert load_full_config(tmp_path / "variant.yaml").altitude == 123.0


def test_missing_placeholders_key_gets_the_default_file(tmp_path):
    _write(tmp_path / "c.yaml", "propulsion: {n_motors: 1}\n")
    ph = expand_dotted(load_yaml(CONFIGS / "placeholders.yaml"))
    cfg = load_full_config(tmp_path / "c.yaml")
    assert cfg.propulsion.Kv == ph["propulsion"]["Kv"]
    assert cfg.energy_reserve == ph["energy_reserve"]
    assert cfg.aero.n_crit == ph["aero"]["n_crit"]


def test_single_point_load_ignores_the_ruleset_multipoint_block(tmp_path):
    shutil.copytree(CONFIGS, tmp_path / "configs")
    c = tmp_path / "configs"
    _write(c / "sp.yaml", "extends: full_mdo.yaml\nruleset: rulesets/dbf_2024_25.yaml\n")
    cfg = load_full_config(c / "sp.yaml")  # FullMDOConfig forbids 'multipoint': the block must be dropped
    assert cfg.altitude == 800.0          # the ruleset's top-level keys still apply
    mp = load_multipoint_config(c / "dbf2025_multipoint.yaml")
    assert mp.mp.scoring["R2"] == 0.20 and mp.mp.rules.togw_max_lb == 55.0 and mp.mp.stores.n_ext == 2


def test_mass_layers_placeholders_between_mass_config_and_config(tmp_path):
    """mass_config < placeholders mass.inputs < the config's mass:"""
    _write(tmp_path / "m.yaml", "inputs: {r_tb: 0.001, x_gd: 0.002, n: 1.0}\n")
    _write(tmp_path / "ph.yaml", "mass.inputs.r_tb: 0.009\nmass.inputs.x_gd: 0.3\n")
    _write(tmp_path / "c.yaml", "placeholders: ph.yaml\nmass_config: m.yaml\nmass: {inputs: {x_gd: 0.4}}\n")
    inp = load_full_config(tmp_path / "c.yaml").mass.inputs
    assert inp["n"] == 1.0 and inp["r_tb"] == 0.009 and inp["x_gd"] == 0.4


# --- the migration ------------------------------------------------------------------------------------------------

# Paths whose effective value changed in the migration (most recent line wins; docs/development/PLACEHOLDERS_RULESETS_PLAN.md).
MOTOR = {"propulsion.e_spec": (150.0, 134.3), "propulsion.Kv": (900.0, 1050.0), "propulsion.Rm": (0.05, 0.03384),
         "propulsion.I0": (1.0, 1.79), "propulsion.m_motor": (0.15, 0.1127), "propulsion.m_esc": (0.06, 0.127),
         "propulsion.m_prop": (0.03, 0.018), "propulsion.m_avionics": (0.25, 0.222),
         "takeoff_distance": (100.0, 106.68)}
EXPECTED_CHANGES = {"full_mdo.yaml": {**MOTOR, "stability.x_motor": (-0.3, -0.1397)},
                    "full_mdo_rdo.yaml": {**MOTOR, "stability.x_motor": (-0.3, -0.1397)},
                    "full_mdo_twin.yaml": MOTOR, "full_mdo_twin_rdo.yaml": MOTOR,
                    "lap_mdo.yaml": {"takeoff_distance": (100.0, 106.68)}}


def _dump(name):
    from aircrafter2.problems.aero_mass import load_aero_mass_config
    from aircrafter2.problems.lap_mdo import load_lap_mdo_config

    f = CONFIGS / name
    if name.startswith("dbf2025") or name == "woombah_2025.yaml":
        return load_multipoint_config(f).model_dump(mode="json")
    if name == "aero_mass.yaml":
        return load_aero_mass_config(f).model_dump(mode="json")
    if name == "lap_mdo.yaml":
        return load_lap_mdo_config(f).model_dump(mode="json")
    return load_full_config(f).model_dump(mode="json")


def _diff(a, b, path="", out=None):
    out = {} if out is None else out
    if isinstance(a, dict) and isinstance(b, dict):
        for k in set(a) | set(b):
            _diff(a.get(k), b.get(k), f"{path}.{k}" if path else k, out)
    elif a != b:
        out[path] = (a, b)
    return out


# The USYD mass calibration (docs/development/MASS_CALIBRATION_PLAN.md) changes, in every config with a mass model, the Step 1
# inputs and the correction factors (to the values of placeholders.yaml), adds the tail model (areal; gg where a config
# pins it for the f_ht/f_vt spreads), and, in the 2024-25 configs, Woombah's n_panels and cargo box (the ruleset).
CAL_INPUTS = ["t_tube_min", "sigma_spar", "k_wing", "a_ht", "a_vt", "f_cb", "f_tb", "f_gear", "f_ht", "f_vt"]
PINNED_GG = {"full_mdo_rdo.yaml", "full_mdo_twin_rdo.yaml", "full_mdo_rdo_fixed_kv.yaml",
             "full_mdo_twin_rdo_fixed_kv.yaml", "dbf2025_multipoint_rdo_full.yaml"}


def _calibration_changes(name, before):
    pre = "base." if "base" in before else ""
    b = before.get("base", before)
    if "mass" not in b:
        return {}
    ph = expand_dotted(load_yaml(CONFIGS / "placeholders.yaml"))["mass"]
    out = {f"{pre}mass.options.tail_mass_model": (None, "gg" if name in PINNED_GG else "areal")}
    for k in CAL_INPUTS:
        if b["mass"]["inputs"].get(k) != ph["inputs"][k]:
            out[f"{pre}mass.inputs.{k}"] = (b["mass"]["inputs"].get(k), ph["inputs"][k])
    if pre:  # the 2024-25 multipoint configs: Woombah's airframe from the ruleset
        rs = expand_dotted(load_yaml(CONFIGS / "rulesets" / "dbf_2024_25.yaml"))
        out[f"{pre}mass.options.n_panels"] = (b["mass"]["options"]["n_panels"], rs["mass"]["options"]["n_panels"])
        for k in ("l_cb", "w_cb", "h_cb"):
            out[f"{pre}geometry.{k}"] = (b["geometry"][k], rs["geometry"][k])
    return out


def _without_f_build_source(params):
    """robust.parameters with the f_build/one-sided source tags dropped (the calibration cites them now)."""
    return [{k: v for k, v in p.items() if not (p["name"] == "f_build" and k == "source")} for p in params or []]


@pytest.mark.parametrize("name", sorted(json.loads((ROOT / "tests/data/configs_premigration.json").read_text())))
def test_configs_load_as_before_the_migration_except_the_listed_changes(name):
    """Every 2024-25 config and variant loads to the same model_dump as before (a snapshot taken before the
    migration); the single-point configs change exactly where the most-recent rule picked a newer value; and every
    config changes where the USYD mass calibration says (values from the layer files)."""
    before = json.loads((ROOT / "tests/data/configs_premigration.json").read_text())[name]
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        diff = _diff(before, _dump(name))
    rp = next((k for k in diff if k.endswith("robust.parameters")), None)
    if rp:  # only the f_build source tag may change in the shared spreads
        a, b = diff.pop(rp)
        assert _without_f_build_source(a) == _without_f_build_source(b)
    for k in [k for k in diff if _new_field(k, diff[k])]:
        del diff[k]
    assert diff == {**EXPECTED_CHANGES.get(name, {}), **_calibration_changes(name, before)}


# Schema fields added after the snapshot (docs/development/RESULTS_2026_PLAN.md, 2026-10-27): absent before, now at their defaults
# (or, for the 2025-26 store set, the universal values every multipoint config loads from placeholders.yaml and
# never uses: mp.kind is dbf2025)
NEW_FIELDS = {"course.turn_drift": False, "course.lap1_cluster": 2.0, "propulsion.f_fit": 0.8, "mp.kind": "dbf2025",
              "mp.payload": {}, "mp.payload_initial": {}, "mp.rules.pax_per_cargo_min": None,
              "disabled_constraints": [],  # 2026-10-27: the Constraints tab's on/off
              "mda_robust": False,  # 2026-11-02: the MDA continuation fallback (docs/development/MDA_ROBUSTNESS_PLAN.md)
              "aero.cd_area_gear": 0.0, "aero.cd_area_lp": 0.0}  # 2026-11-02: drag areas (Final fixes D3; 2025-26 only)


def _new_field(path, change):
    before, now = change
    key = path[5:] if path.startswith("base.") else path
    if before is not None:
        return False
    if key == "mp.stores2026" or key.startswith("mp.stores2026."):
        return True
    return key in NEW_FIELDS and NEW_FIELDS[key] == now


# --- one place per value ------------------------------------------------------------------------------------------

# Files allowed to set a layer value besides the layer files: the study variants (their pinned values), the
# aircraft-specific configs, and the mass verification cases (G&G's own inputs).
ALLOWED = {"full_mdo_fixed_kv.yaml", "full_mdo_twin_fixed_kv.yaml", "full_mdo_rdo_fixed_kv.yaml",
           "full_mdo_twin_rdo_fixed_kv.yaml", "dbf2025_multipoint_rdo_full.yaml",
           "dbf2025_multipoint_rdo_calibrated.yaml", "dbf2025_multipoint_rdo_onesided.yaml",
           "dbf2025_multipoint_takeoff.yaml", "dbf2025_multipoint_rdo_takeoff.yaml", "woombah_2025.yaml",
           "mass_usyd_woombah.yaml", "uncertainty.yaml", "uncertainty_2025_reduced_calibrated.yaml",
           "uncertainty_2025_reduced_onesided.yaml",
           # the USYD mass calibration: G&G's published factors (verification, uncalibrated comparisons), the
           # Woombah-out factors, and the full-set RDO configs that pin the gg tails for their f_ht/f_vt spreads
           "mass_garcia_gamboa_2022.yaml", "mass_usyd_loo_woombah.yaml", "full_mdo_rdo.yaml", "full_mdo_twin_rdo.yaml",
           # the 2025-26 study: the Humpback-out factors and Humpback as built (docs/development/RESULTS_2026_PLAN.md)
           "mass_usyd_loo_humpback.yaml", "humpback_2026.yaml"}


def _config_files():
    return [f for f in CONFIGS.rglob("*.yaml") if f not in LAYER_FILES and "verification" not in f.parts
            and "courses" not in f.parts]


def _sets(raw, path):
    parts = path.split(".")
    if parts[0] == "mass" and "inputs" in raw and "mass" not in raw:  # a mass-model file: inputs at the top level
        parts = parts[1:]
    node = raw
    for k in parts:
        if not isinstance(node, dict) or k not in node:
            return False
        node = node[k]
    return True


def test_every_layer_value_is_set_in_one_place():
    paths = sorted({r["parameter"] for f in LAYER_FILES for r in read_rows(f)[0]})
    spreads = sorted({s["name"] for f in LAYER_FILES for s in read_rows(f)[1]})
    offenders = []
    for f in _config_files():
        if f.name in ALLOWED:
            continue
        raw = yaml.safe_load(f.read_text()) or {}
        offenders += [f"{f.name}: {p}" for p in paths if _sets(raw, p)]
        params = (raw.get("parameters") or []) + ((raw.get("robust") or {}).get("parameters") or [])
        offenders += [f"{f.name}: spread {p['name']}" for p in params if p.get("name") in spreads]
    assert offenders == []


def test_layer_rows_are_tagged():
    for f in LAYER_FILES:
        rows, spreads = read_rows(f)
        assert all(r["status"] in ("PLACEHOLDER", "cited") and r["units"] and r["Q"] for r in rows + [
            dict(s, units=s["comment_units"]) for s in spreads]), f.name


# --- save in place ------------------------------------------------------------------------------------------------

def _edit_one_per_area(rows):
    seen, edited = set(), []
    for r in rows:
        if r["area"] in seen or not isinstance(r["value"], (int, float)) or isinstance(r["value"], bool):
            continue
        seen.add(r["area"])
        edited.append(dict(r, value=r["value"] * 1.5 + 0.25, status="cited", source=r["source"] + " (edited)"))
    return edited


@pytest.mark.parametrize("layer", LAYER_FILES, ids=lambda p: p.name)
def test_save_in_place_round_trip(tmp_path, layer):
    t = tmp_path / layer.name
    shutil.copy(layer, t)
    rows, spreads = read_rows(t)
    edited = _edit_one_per_area(rows)
    assert edited
    sp = [dict(spreads[0], spread=0.4, note="edited spread")]
    changed = save_rows(t, edited, sp)
    assert sorted(changed) == sorted([r["line"] for r in edited] + [spreads[0]["line"]])
    new_rows, new_spreads = read_rows(t)
    by_line = {r["line"]: r for r in new_rows}
    for r in edited:
        assert by_line[r["line"]]["value"] == r["value"] and by_line[r["line"]]["status"] == "cited"
        assert by_line[r["line"]]["source"].endswith("(edited)")
    assert new_spreads[0]["spread"] == 0.4 and new_spreads[0]["note"] == "edited spread"
    before, after = layer.read_text().splitlines(), t.read_text().splitlines()
    assert len(before) == len(after)
    assert [i + 1 for i, (a, b) in enumerate(zip(before, after)) if a != b] == sorted(changed)
    assert yaml.safe_load(t.read_text()) is not None


def test_placeholder_edit_reaches_single_point_and_multipoint_configs(tmp_path):
    shutil.copytree(CONFIGS, tmp_path / "configs")
    c = tmp_path / "configs"
    rows, _ = read_rows(c / "placeholders.yaml")
    edits = [dict(r, value={"energy_reserve": 0.3, "propulsion.Kv": 999.0}[r["parameter"]]) for r in rows
             if r["parameter"] in ("energy_reserve", "propulsion.Kv")]
    save_rows(c / "placeholders.yaml", edits)
    assert load_full_config(c / "full_mdo.yaml").energy_reserve == 0.3
    assert load_multipoint_config(c / "dbf2025_multipoint.yaml").base.energy_reserve == 0.3
    assert load_full_config(c / "full_mdo.yaml").propulsion.Kv == 999.0
    assert load_full_config(c / "full_mdo_fixed_kv.yaml").propulsion.Kv == 900.0  # the variant overrides it
    assert overridden_by(c / "full_mdo_fixed_kv.yaml", "propulsion.Kv").startswith("configs/full_mdo_fixed_kv.yaml:") \
        or "full_mdo_fixed_kv.yaml" in overridden_by(c / "full_mdo_fixed_kv.yaml", "propulsion.Kv")
    assert overridden_by(c / "full_mdo.yaml", "propulsion.Kv") == ""


def test_shared_spreads_make_the_reduced_rdo_set():
    """The reduced RDO set (d = 4) is the shared spreads in order; the calibrated variant overrides f_build in place;
    the published full set (standalone uncertainty.yaml) gets none of them."""
    red = load_multipoint_config(CONFIGS / "dbf2025_multipoint_rdo.yaml").base.robust.parameters
    assert [p.name for p in red] == ["f_build", "f_tb", "m_avionics", "lap.wind"]
    cal = load_multipoint_config(CONFIGS / "dbf2025_multipoint_rdo_calibrated.yaml").base.robust.parameters
    assert [p.name for p in cal] == ["f_build", "f_tb", "m_avionics", "lap.wind"] and cal[0].nominal == 2.73763
    full = load_multipoint_config(CONFIGS / "dbf2025_multipoint_rdo_full.yaml").base.robust.parameters
    assert "f_build" not in [p.name for p in full] and full[-1].name == "store_drag_factor"


def test_layer_files_have_area_headers_and_one_value_per_line():
    for f in LAYER_FILES:
        text = f.read_text()
        assert re.search(r"^# --- .+ ---$", text, re.M)
        for line in text.splitlines():
            if line and not line.startswith(("#", " ", "uncertainty:")):
                assert re.match(r"^[A-Za-z_][\w.]*: [^{\[]", line), line
