"""The Constraints tab's inventory (aircrafter2/constraints_view.py): the optimiser section is the set-up problem's
driver list, every constraint has a register entry, every AnalysisError and report-only check in the code is
accounted for, and every limit is traced to the file and line that sets its effective value."""

import json
import re
from pathlib import Path

import pytest
import yaml

from aircrafter2 import constraints_view as CV
from aircrafter2.config.layer_files import layer_files_of
from aircrafter2.config.loader import load_yaml
from aircrafter2.problems.full_mdo import full_constraints, load_full_config
from aircrafter2.problems.multipoint import load_multipoint_config, multipoint_constraints
from aircrafter2.scoring import get_plugin

ROOT = Path(__file__).resolve().parents[1]
CONFIGS = ROOT / "configs"
UI_CONFIGS = sorted(p.name for p in CONFIGS.glob("*.yaml") if "propulsion" in load_yaml(p))  # as the UI lists them
KEYS = ("lower", "upper", "equals", "ref", "units")


def load(name):
    p = CONFIGS / name
    return load_multipoint_config(p) if "multipoint" in load_yaml(p) else load_full_config(p)


def _robust_cfg(cfg):
    return (cfg.base if hasattr(cfg, "mp") else cfg).robust


def variants():
    """(id, config name, robust, edit): every UI config as is, the RDO ones also deterministic, and the
    mean_sigma_cap objective (single point, Kv on and off)."""
    out = [(n, n, None, None) for n in UI_CONFIGS]
    out += [(f"{n}-det", n, False, None) for n in UI_CONFIGS if (load_yaml(CONFIGS / n).get("robust") or {})
            .get("enabled")]
    for n in ("full_mdo_rdo.yaml", "full_mdo_twin_rdo_fixed_kv.yaml"):
        out.append((f"{n}-sigma_cap", n, None, dict(objective="mean_sigma_cap", sigma_cap=1.0)))
    return out


VARIANTS = variants()


def build(name, edit):
    cfg = load(name)
    if edit:
        for k, v in edit.items():
            setattr(_robust_cfg(cfg), k, v)
    return cfg


def opt_rows(rows):
    return [r for r in rows if r["section"] == 1]


def raw(r):
    """A row's driver kwargs as add_constraint got them (con_units: the raw units; 'units' is for display)."""
    return dict(lower=r["lower"], upper=r["upper"], equals=r["equals"], ref=r["ref"], units=r["con_units"])


def test_ui_configs_cover_the_kinds():
    kinds = set()
    for n in UI_CONFIGS:
        cfg = load(n)
        b = cfg.base if hasattr(cfg, "mp") else cfg
        kinds |= {("mp" if hasattr(cfg, "mp") else "sp"), ("rdo" if b.robust.enabled else "det"),
                  ("kv" if "Kv" in b.design_variables else "fixed"), ("twin" if b.propulsion.n_motors == 2 else "1")}
    assert kinds == {"mp", "sp", "rdo", "det", "kv", "fixed", "twin", "1"}


@pytest.mark.parametrize("vid,name,robust,edit", VARIANTS, ids=[v[0] for v in VARIANTS])
def test_optimiser_section_is_the_driver_list(vid, name, robust, edit):
    """Same names, bounds, refs, units and order as the driver of the problem the run builds (the full RDO rule,
    not the one-node rule constraint_table uses)."""
    cfg = build(name, edit)
    rows = opt_rows(CV.constraint_table(cfg, robust=robust, config_path=CONFIGS / name))
    prob = CV.setup_problem(cfg, robust=robust, reduced=False)
    want = []
    for n, meta in prob.driver._cons.items():
        kw = {k: meta.get(k) for k in KEYS}
        kw.update({k: None for k in ("lower", "upper") if kw[k] is not None and abs(kw[k]) >= CV.INF})
        want.append((n, kw))
    assert [(r["name"], raw(r)) for r in rows] == want
    assert all(CV.parse_driver_name(r["name"])[0] in CV.REGISTER for r in rows)


def _kw(kw):
    return {k: kw.get(k) for k in KEYS}


@pytest.mark.parametrize("name", [n for n in UI_CONFIGS])
def test_deterministic_rows_equal_the_problem_functions(name):
    cfg = load(name)
    rows = opt_rows(CV.constraint_table(cfg, robust=False))
    if hasattr(cfg, "mp"):
        want = multipoint_constraints(cfg)  # the multipoint builders add no plug-in constraints
        from aircrafter2.scoring.dbf2025 import DBF2025
        from aircrafter2.scoring.dbf2026 import DBF2026

        assert (DBF2026 if cfg.mp.kind == "dbf2026" else DBF2025)(**cfg.mp.scoring).constraints() == []
    else:  # add_full_driver: the plug-in's constraints (add_scoring), then full_constraints
        want = get_plugin(cfg.scoring.plugin, **cfg.scoring.options).constraints() + full_constraints(cfg)
    got = [(r["name"], raw(r)) for r in rows]
    assert [n for n, _ in got] == [n for n, _ in want]
    for (n, g), (_, w) in zip(got, want):
        assert g == _kw(w), n


def test_unregistered_driver_constraint_raises(monkeypatch):
    import aircrafter2.problems.full_mdo as F

    real = F.full_constraints
    monkeypatch.setattr(F, "full_constraints", lambda cfg: real(cfg) + [("x_cg", dict(lower=0.0))])
    monkeypatch.setattr(CV, "_CACHE", {})
    with pytest.raises(CV.UnregisteredConstraint, match="x_cg"):
        CV.constraint_table(load("full_mdo.yaml"))


def test_unregistered_margin_output_raises(monkeypatch):
    """A margin the model computes but nobody constrains or registers is an error too."""
    monkeypatch.setattr(CV, "_CACHE", {})
    monkeypatch.delitem(CV.REGISTER, "takeoff_accel_margin")
    monkeypatch.setattr(CV, "_ANALYSIS", [e for e in CV._ANALYSIS if e.name != "takeoff_accel_margin"])
    with pytest.raises(CV.UnregisteredConstraint, match="takeoff_accel_margin"):
        CV.constraint_table(load("full_mdo.yaml"))


def test_rdo_reduced_rule_is_one_node():
    prob = CV.setup_problem(load("full_mdo_rdo.yaml"), reduced=True)
    assert prob.rdo["N"] == 1


# --- sections 3 and 4: every AnalysisError and report-only check in the code -------------------------------------------

SCAN = [r"raise AnalysisError\(", r"(?i)report[- ]only|reported only|post-check|\(report\)", r"err_on_non_converge=True"]


def test_source_scan_every_site_is_in_the_register():
    claimed = {h for e in CV.REGISTER.values() for f, p in e.anchors for h in CV.code_lines(f, p)}
    waived = {CV.code_line(f, re.escape(p)) for f, p in CV.SCAN_WAIVED}
    missing = []
    for path in sorted((ROOT / "aircrafter2").rglob("*.py")):
        rel = str(path.relative_to(ROOT))
        if rel.startswith("aircrafter2/ui/") or rel == "aircrafter2/constraints_view.py":
            continue
        for i, line in enumerate(path.read_text().splitlines(), start=1):
            if any(re.search(p, line) for p in SCAN) and f"{rel}:{i}" not in claimed | waived:
                missing.append(f"{rel}:{i}: {line.strip()}")
    assert not missing, "not in constraints_view.REGISTER (anchors) or SCAN_WAIVED:\n" + "\n".join(missing)


def test_analysis_error_sites_are_section_3():
    for f, p in [(CV.SL, r"raise AnalysisError")]:
        for hit in CV.code_lines(f, p):
            owners = [e for e in CV.REGISTER.values() for af, ap in e.anchors if hit in CV.code_lines(af, ap)]
            assert owners and all(e.section == 3 for e in owners), hit


def test_report_only_outputs_are_rows():
    rows = CV.constraint_table(load("full_mdo_twin.yaml"))
    s3 = {r["constraint"] for r in rows if r["section"] == 3}
    assert {"nf_confidence_min", "n_stalled", "lapCount", "rpm_max", "fit_residual_T", "prop_clear_margin",
            "cl_margin_sec.mid_turn", "wind.lap", "sizing.spar"} <= s3
    s4 = {r["constraint"] for r in rows if r["section"] == 4}
    assert s4 == {"balance.alpha_cruise", "balance.v", "mass.closure"}


def test_every_hard_coded_location_resolves():
    for e in CV.REGISTER.values():
        for f, p, _ in e.hard_coded:
            CV.code_line(f, p)


# --- limit sources ------------------------------------------------------------------------------------------------------

def _file_value(src, path):
    """The value the traced file and line give for path."""
    from aircrafter2.config.layer_files import read_rows

    if src["kind"] in ("placeholders", "ruleset"):
        return next(r["value"] for r in read_rows(src["file"])[0] if r["line"] == src["line"])
    data = yaml.safe_load(src["file"].read_text())
    parts = path.split(".")[1:] if src["kind"] == "mass config" else path.split(".")
    for k in parts:
        data = data[k]
    return data


@pytest.mark.parametrize("name", UI_CONFIGS)
def test_every_limit_path_resolves_to_its_effective_value(name):
    cfg = load(name)
    rows = CV.constraint_table(cfg, config_path=CONFIGS / name)
    seen = 0
    for r in rows:
        for s in r["sources"]:
            val = CV.get_value(cfg, s["path"])
            assert s["kind"], (r["name"], s["path"])
            if s["kind"] == "code default":
                continue
            line = s["file"].read_text().splitlines()[s["line"] - 1]
            assert s["path"].split(".")[-1] in line, (s["path"], s["text"], line)
            fv = _file_value(s, s["path"])
            if s["path"] == "multipoint.rules.togw_max_lb" or not isinstance(val, (list, dict)):
                assert fv == val or (fv is None and val is None), (name, s["path"], s["text"], fv, val)
            seen += 1
    assert seen > 5


@pytest.mark.parametrize("name,path,where", [
    ("dbf2026_multipoint.yaml", "stability.SM_max", "rulesets/dbf_2025_26.yaml"),
    ("full_mdo.yaml", "stability.SM_max", "placeholders.yaml"),
    ("dbf2026_multipoint.yaml", "propulsion.rpm_fraction", "dbf2026_multipoint.yaml"),
    ("full_mdo.yaml", "propulsion.rpm_fraction", "full_mdo.yaml"),
    ("full_mdo_fixed_kv.yaml", "takeoff_distance", "full_mdo_fixed_kv.yaml"),
    ("full_mdo.yaml", "takeoff_distance", "placeholders.yaml"),
    ("dbf2026_multipoint.yaml", "propulsion.I_max", "rulesets/dbf_2025_26.yaml"),
    ("dbf2026_multipoint.yaml", "wall_ratio_max", "full_mdo.yaml"),
    ("full_mdo_rdo.yaml", "robust.beta", "full_mdo_rdo.yaml"),
])
def test_traced_sources(name, path, where):
    src = CV.limit_source(path, CONFIGS / name, *layer_files_of(CONFIGS / name))
    assert src["text"].split(":")[0] == where


def test_code_default_and_none():
    src = CV.limit_source("propulsion.pack_I_max", CONFIGS / "full_mdo.yaml")
    assert src["kind"] == "code default" and "pack_I_max = None" in src["text"]


def test_unused_fields_section():
    rows = CV.constraint_table(load("full_mdo.yaml"), config_path=CONFIGS / "full_mdo.yaml")
    s5 = {r["constraint"] for r in rows if r["section"] == 5}
    assert {"clmax_ratio_min", "propulsion.c_clear", "propulsion.pack_I_max", "robust.sigma_cap"} <= s5
    used = {p for r in rows if r["section"] < 5 for p in r["limit_paths"]}
    assert not s5 & used
    twin = CV.constraint_table(load("full_mdo_twin.yaml"))
    assert "propulsion.c_clear" not in {r["constraint"] for r in twin if r["section"] == 5}
    mp = CV.constraint_table(load("dbf2026_multipoint.yaml"))
    assert "multipoint.rules.laps_m2_min" in {r["constraint"] for r in mp if r["section"] == 5}


def test_effective_limits():
    rows = {r["name"]: r for r in CV.constraint_table(load("dbf2026_multipoint.yaml"))}
    assert "7,895 rpm" in rows["points.M2.rpm_margin"]["effective"]
    assert rows["rules.togw_margin"]["effective"].startswith("TOGW < 55 lb")
    rdo = {r["name"]: r for r in CV.constraint_table(load("full_mdo_rdo.yaml"))}
    assert rdo["stats.SM_lo"]["effective"].startswith("mu(SM) - 2 sigma(SM) >= 0.05")
    assert rdo["stats.SM_lo"]["robust"] and not rdo["nodes.n0.V_h"]["robust"]


def test_dv_bound_rules_caption():
    texts = [t for _, t in CV.dv_bound_rules(load("dbf2026_multipoint.yaml"))]
    assert any("span <= 5 ft" in t for t in texts) and any("RAC >= 0.90" in t for t in texts)
    assert CV.dv_bound_rules(load("full_mdo.yaml")) == []


# --- checkpoint 2: the tab's edits, on/off, saving -----------------------------------------------------------------------

def _driver(cfg):
    prob = CV.setup_problem(cfg)
    keys = ("lower", "upper", "equals", "ref", "units", "scaler", "adder")
    meta = lambda d: [(n, {k: m.get(k) for k in keys}) for n, m in d.items()]  # noqa: E731
    return meta(prob.driver._designvars), meta(prob.driver._cons), meta(prob.driver._objs)


@pytest.mark.parametrize("name", ["full_mdo.yaml", "full_mdo_rdo.yaml", "full_mdo_twin_fixed_kv.yaml",
                                  "dbf2026_multipoint.yaml"])
def test_no_edits_reproduce_the_config_and_its_driver(name):
    cfg = load(name)
    sp = not hasattr(cfg, "mp")
    run = CV.apply_limits(cfg, {}, off=CV.config_off(cfg) if sp else None)
    assert run.model_dump() == cfg.model_dump()
    assert _driver(run) == _driver(cfg)


def test_switched_off_constraint_leaves_the_run(tmp_path):
    from aircrafter2.api import run_design

    cfg = load("full_mdo_fixed_kv.yaml")
    cfg.aero.num_y = 11
    kw = dict(optimise=False, figures=False, sensitivities=False, n2=False)
    run_design(cfg, out_dir=tmp_path / "on", **kw)
    run_design(CV.apply_limits(cfg, off={"rpm_margin"}), out_dir=tmp_path / "off", **kw)
    on = json.loads((tmp_path / "on" / "constraints.json").read_text())
    off = json.loads((tmp_path / "off" / "constraints.json").read_text())
    assert [c["name"] for c in on["constraints"]] == [n for n, _ in full_constraints(cfg)]
    assert len(off["constraints"]) == len(on["constraints"]) - 1 and off["disabled"] == ["rpm_margin"]
    assert "rpm_margin" not in {c["name"] for c in off["constraints"]}
    summary = [line for line in (tmp_path / "off" / "summary.md").read_text().splitlines() if line]
    assert summary[1].startswith("**Constraints switched off for this run: `rpm_margin`")  # right below the title
    assert "switched off" not in (tmp_path / "on" / "summary.md").read_text()


def test_unknown_disabled_constraint_raises():
    cfg = load("full_mdo.yaml")
    cfg.disabled_constraints = ["nonexistent"]
    with pytest.raises(ValueError, match="nonexistent"):
        full_constraints(cfg)
    cfg.disabled_constraints = ["energy_margin"]
    with pytest.raises(ValueError, match="require_3lap_energy"):
        full_constraints(cfg)


def test_validation():
    cfg = load("full_mdo.yaml")
    assert CV.validate_limits(cfg) == []
    bad = CV.apply_limits(cfg, {"stability.SM_min": 0.3, "propulsion.rpm_fraction": 1.2, "propulsion.I_max": -1.0})
    errs = " ".join(CV.validate_limits(bad))
    assert "SM_min (0.3) must not exceed stability.SM_max" in errs and "(0, 1]" in errs and "I_max must be > 0" in errs
    assert CV.validate_limits(load("dbf2026_multipoint.yaml")) == []


def test_round_trip_one_value_per_source(tmp_path):
    """One edit per source kind (placeholders, ruleset, problem config) and a comment edit, in a copy of configs/:
    the values change on reload, every other byte stays, the comments are kept."""
    import shutil

    shutil.copytree(CONFIGS, tmp_path / "configs")
    c = tmp_path / "configs"
    files = {f: f.read_bytes() for f in c.rglob("*.yaml")}
    mp = c / "dbf2026_multipoint.yaml"
    res = CV.save_limits(mp, {"stability.SM_min": 0.06, "stability.SM_max": 0.45, "propulsion.rpm_fraction": 0.95},
                         {"stability.SM_min": dict(status="cited", Q="Q9", source="a test source")})
    assert not res["errors"]
    changed = {f: (a, f.read_bytes()) for f, a in files.items() if f.read_bytes() != a}
    assert set(changed) == {c / "placeholders.yaml", c / "rulesets/dbf_2025_26.yaml", mp}
    for f, (a, b) in changed.items():
        a, b = a.decode().splitlines(), b.decode().splitlines()
        diff = [(x, y) for x, y in zip(a, b) if x != y]
        assert len(a) == len(b) and len(diff) == 1, (f, diff)
        x, y = diff[0]
        assert x.split("#", 1)[0].split(":")[0] == y.split("#", 1)[0].split(":")[0]  # same key
        if f.name != "placeholders.yaml":  # the comment is kept (placeholders: edited on purpose)
            assert x.split("#", 1)[1] == y.split("#", 1)[1]
    assert "# -; cited; Q9; a test source" in (c / "placeholders.yaml").read_text()
    m = load_multipoint_config(mp)
    assert (m.base.stability.SM_min, m.base.stability.SM_max, m.base.propulsion.rpm_fraction) == (0.06, 0.45, 0.95)


def test_saving_a_code_default_is_an_error(tmp_path):
    import shutil

    shutil.copytree(CONFIGS, tmp_path / "configs")
    res = CV.save_limits(tmp_path / "configs" / "full_mdo.yaml", {"propulsion.pack_I_max": 80.0})
    assert res["errors"] and "code default" in res["errors"][0] and not res["written"]


def test_study_readers():
    r = CV.study_readers(CONFIGS)
    ph = r[(CONFIGS / "placeholders.yaml").resolve()]
    assert {"studies/results_2026.py", "studies/results_2025.py", "studies/validation_2026.py"} <= set(ph)
    mp = r[(CONFIGS / "dbf2026_multipoint.yaml").resolve()]
    assert {"studies/results_2026.py", "studies/validation_2026.py"} <= set(mp)  # validation: humpback extends it
    assert "studies/results_2025.py" not in mp
    assert "studies/aircrafter1_2025.py" in r[(CONFIGS / "dbf2025_multipoint.yaml").resolve()]  # via import


def test_run_states():
    rows = [dict(section=1, name="SM", constraint="SM"), dict(section=1, name="rpm_margin", constraint="rpm_margin"),
            dict(section=1, name="r_tube", constraint="r_tube"), dict(section=2, name="post.I_margin",
                                                                      constraint="post.I_margin")]
    cj = dict(tol=1e-8, constraints=[
        dict(name="SM", lower=0.05, upper=0.2, equals=None, ref=None, value=0.05),
        dict(name="rpm_margin", lower=0.0, upper=None, equals=None, ref=1000.0, value=-5.0),
        dict(name="r_tube", lower=0.005, upper=None, equals=None, ref=0.01, value=0.008)])
    CV.run_states(rows, cj, {"limits": {"I_margin_A": 3.0}})
    assert [r["state"] for r in rows] == ["active", "violated", "inactive", "ok"]
    assert rows[1]["margin"] == -5.0


def test_tab_paths_are_not_edited_elsewhere():
    paths = CV.tab_paths()
    assert not {p for p in paths if CV.edit_in(p)}
    assert "energy_reserve" not in paths and "mu_roll" not in paths  # model inputs: their own tabs
    assert {"stability.SM_max", "propulsion.rpm_fraction", "takeoff_distance", "require_3lap_energy"} <= paths


def test_editor_changes_map_cells_to_config_paths():
    """The table editor: an SM upper edit is stability.SM_max, a stall_margin lower edit is ignored (hard-coded), the
    rpm limit cell is rpm_fraction, unticking 'on' switches rpm_margin off, ticking 'robust' adds V_h."""
    import pandas as pd

    rows = [r for r in CV.constraint_table(load("full_mdo_rdo.yaml")) if r["section"] == 1]
    cols = ["on", "robust", "lower", "upper", "limit"]
    before = pd.DataFrame([dict(on=True, robust=r["robust"], lower=r["lower"], upper=r["upper"], limit=r["limit"])
                           for r in rows], columns=cols)
    after = before.copy()
    idx = {r["name"]: i for i, r in enumerate(rows)}
    after.at[idx["stats.SM_hi"], "upper"] = 0.3
    after.at[idx["stats.stall_margin_lo"], "lower"] = 1.0
    after.at[idx["nodes.n0.rpm_margin"], "limit"] = 0.9
    after.at[idx["nodes.n0.rpm_margin"], "on"] = False
    after.at[idx["nodes.n0.V_h"], "robust"] = True
    edits, off, robust, ignored = CV.editor_changes(rows, before, after, {}, set(), ["SM"])
    assert edits == {"stability.SM_max": 0.3, "propulsion.rpm_fraction": 0.9}
    assert off == {"rpm_margin"} and robust == ["SM", "V_h"]
    assert len(ignored) == 1 and ignored[0].startswith("stats.stall_margin_lo: its lower is not a config value")
