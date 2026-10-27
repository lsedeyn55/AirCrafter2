"""Streamlit UI, run headless."""

import json
from pathlib import Path

APP = Path(__file__).resolve().parents[1] / "aircrafter2" / "ui" / "app.py"


def _button(at, label):
    return [b for b in at.button if b.label == label][0]


def test_streamlit_app_runs_headless():
    from streamlit.testing.v1 import AppTest

    at = AppTest.from_file(str(APP), default_timeout=900)
    at.run()
    assert not at.exception
    at.sidebar.select_slider[0].set_value(11)
    _button(at, "Run").click().run()
    assert not at.exception
    assert any("AirCrafter 2.0 result" in m.value for m in at.markdown)


def test_streamlit_robust_toggle_runs_headless():
    from streamlit.testing.v1 import AppTest

    at = AppTest.from_file(str(APP), default_timeout=900)
    at.run()
    at.sidebar.selectbox[0].set_value("full_mdo_rdo.yaml").run()
    robust = [c for c in at.sidebar.checkbox if c.label == "Robust design optimisation"][0]
    assert robust.value  # on for the RDO config
    at.sidebar.multiselect[0].set_value(["Kv_factor"])  # d = 1: 3 nodes (Kv is a design variable here)
    at.sidebar.select_slider[0].set_value(11)
    _button(at, "Run").click().run()
    assert not at.exception
    assert any("Robust statistics" in m.value for m in at.markdown)


def _ni(at, label):
    return [w for w in at.sidebar.number_input if w.label == label][0]


def _kv_caption(at):
    return [c.value for c in at.sidebar.caption if c.value.startswith("Kv design variable")]


def test_streamlit_kv_toggle_bounds_and_validation():
    from streamlit.testing.v1 import AppTest

    at = AppTest.from_file(str(APP), default_timeout=300)
    at.run()
    assert not at.exception
    toggle = [c for c in at.sidebar.checkbox if c.label == "Optimise motor Kv"][0]
    assert toggle.value  # on for the default config
    # start = the reference motor's Kv, now the placeholders file's T-Motor V3115 1050 KV (was the 900 placeholder)
    assert _kv_caption(at) == ["Kv design variable: 500 to 1500 rpm/V, start 1050."]
    assert any(w.label == "Reference motor Kv (rpm/V) - PLACEHOLDER" for w in at.sidebar.number_input)
    # user-set bounds and start reach cfg.design_variables["Kv"] / cfg.initial["Kv"] (the caption reads them back)
    _ni(at, "Kv lower bound (rpm/V)").set_value(600.0)
    _ni(at, "Kv upper bound (rpm/V)").set_value(1200.0)
    _ni(at, "Kv start (rpm/V)").set_value(800.0).run()
    assert _kv_caption(at) == ["Kv design variable: 600 to 1200 rpm/V, start 800."]
    assert not _button(at, "Run").disabled
    # validation: reversed bounds, start outside, bounds outside 100-5000 -> errors and no run
    _ni(at, "Kv lower bound (rpm/V)").set_value(1300.0).run()
    errs = [e.value for e in at.sidebar.error]
    assert any("must be below the upper bound" in e for e in errs) and any("within the bounds" in e for e in errs)
    assert _button(at, "Run").disabled and not _kv_caption(at)
    _ni(at, "Kv lower bound (rpm/V)").set_value(600.0)
    _ni(at, "Kv upper bound (rpm/V)").set_value(6000.0).run()
    assert any("within 100-5000" in e.value for e in at.sidebar.error) and _button(at, "Run").disabled
    # off: fixed motor, no Kv design variable
    toggle = [c for c in at.sidebar.checkbox if c.label == "Optimise motor Kv"][0]
    toggle.uncheck().run()
    assert not at.exception and not _kv_caption(at) and not at.sidebar.error
    assert any(w.label == "Motor Kv (rpm/V) - PLACEHOLDER" for w in at.sidebar.number_input)
    assert not _button(at, "Run").disabled


def test_streamlit_takeoff_and_course_headless(tmp_path):
    """Take-off on, a leg added to the course, Analyse at num_y 11: the summary shows the take-off time."""
    from streamlit.testing.v1 import AppTest

    at = AppTest.from_file(str(APP), default_timeout=900)
    at.run()
    assert not at.exception
    # k_lof, CL_ground and the take-off distance moved to the Placeholders tab: the Mission column points there
    assert any(c.value.startswith("Take-off parameters: k_lof 1.1, CL_ground 0.15") for c in at.caption)
    [c for c in at.checkbox if c.label == "Include take-off in mission time"][0].check().run()
    # add a leg: a 100 ft crosswind straight at the end of the lap (head and tail distances still match)
    _button(at, "Add leg (lap)").click().run()
    assert not at.exception and not at.error and not _button(at, "Run").disabled
    at.sidebar.select_slider[0].set_value(11)
    _button(at, "Run").click().run()
    assert not at.exception
    md = "\n".join(m.value for m in at.markdown)
    assert "AirCrafter 2.0 result" in md and "`t_takeoff`" in md and "ground roll" in md
    assert "Course: custom" in md and "Leg breakdown" in md and "| 1 | 1 | ground roll |" in md
    assert "| 1 | 8 | straight | 100 ft | none |" in md  # the added leg was flown (lap 1: roll + 7 legs)
    assert any(m.label.startswith("Take-off time") for m in at.metric)


def test_streamlit_course_validation_blocks_run():
    """A course error (a zero-length straight) shows and disables Run; a valid course re-enables it; save-as-preset
    refuses a shipped name."""
    import pandas as pd
    from streamlit.testing.v1 import AppTest

    at = AppTest.from_file(str(APP), default_timeout=300)
    key = "legs_full_mdo.yaml_dbf_2025"
    at.session_state[f"{key}_df"] = pd.DataFrame([
        dict(order=1, kind="straight", length_ft=0.0, angle_deg=None, wind="head"),
        dict(order=2, kind="turn", length_ft=None, angle_deg=360.0, wind=None)])
    at.session_state[f"{key}_v"] = 0
    at.run()
    assert not at.exception
    assert any("must be > 0" in e.value for e in at.error)
    assert _button(at, "Run").disabled and _button(at, "Save preset").disabled
    at.session_state[f"{key}_df"] = pd.DataFrame([
        dict(order=1, kind="straight", length_ft=500.0, angle_deg=None, wind="head"),
        dict(order=2, kind="turn", length_ft=None, angle_deg=360.0, wind=None)])
    at.session_state[f"{key}_v"] = 1
    at.run()
    assert not at.error and not _button(at, "Run").disabled
    assert any("differ" in w.value for w in at.warning)  # 500 ft head, no tail: a warning above the sketch
    at.text_input[0].set_value("dbf_2025").run()
    _button(at, "Save preset").click().run()
    assert any("shipped preset" in e.value for e in at.error)


# --- placeholders and ruleset tabs (docs/PLACEHOLDERS_RULESETS_PLAN.md) ---------------------------------------------

CONFIGS = Path(__file__).resolve().parents[1] / "configs"
PH_KEY = "layer_full_mdo.yaml_placeholders.yaml"


def _ph_frame(configs, **values):
    """The Placeholders editor's frame for configs/placeholders.yaml with some values edited (as text)."""
    from aircrafter2.config.layer_files import read_rows, rows_to_frame

    df = rows_to_frame(read_rows(configs / "placeholders.yaml")[0])
    for name, val in values.items():
        df.loc[df["parameter"] == name, "value"] = val
    return df


def test_streamlit_three_tabs_and_multipoint_view():
    """full_mdo.yaml: the four tabs; dbf2025_multipoint.yaml: Constraints and the two layer tabs, an info box, no Run
    button."""
    from streamlit.testing.v1 import AppTest

    at = AppTest.from_file(str(APP), default_timeout=300)
    at.run()
    assert not at.exception
    assert [t.label for t in at.tabs] == ["Design and mission", "Constraints", "Placeholders", "Ruleset"]
    assert any(b.label == "Save (placeholders)" for b in at.button) and any(b.label == "Run" for b in at.button)
    at.sidebar.selectbox[0].set_value("dbf2025_multipoint.yaml").run()
    assert not at.exception
    assert [t.label for t in at.tabs] == ["Constraints", "Placeholders", "Ruleset"]
    assert not any(b.label == "Run" for b in at.button)
    assert any("multipoint" in i.value and "results_2025.py" in i.value for i in at.info)
    assert at.selectbox[0].value == "rulesets/dbf_2024_25.yaml"  # the config's ruleset by default


def test_streamlit_energy_reserve_edit_changes_the_run():
    """An unsaved energy-reserve edit in the Placeholders tab reaches an Analyse run (num_y 11)."""
    from streamlit.testing.v1 import AppTest

    at = AppTest.from_file(str(APP), default_timeout=900)
    at.run()
    at.sidebar.select_slider[0].set_value(11)
    _button(at, "Run").click().run()
    assert not at.exception
    base = json.loads((Path(at.session_state["last"]) / "results.json").read_text())["energy_margin"]
    at.session_state[f"{PH_KEY}_df"] = _ph_frame(CONFIGS, energy_reserve="0.4")
    at.run()
    assert any("Unsaved edits in configs/placeholders.yaml" in i.value for i in at.info)
    _button(at, "Run").click().run()
    assert not at.exception
    edited = json.loads((Path(at.session_state["last"]) / "results.json").read_text())["energy_margin"]
    assert edited < base - 1.0  # 40 % instead of 20 % of the usable energy held back
    assert (CONFIGS / "placeholders.yaml").read_text().count("energy_reserve: 0.2 ") == 1  # not saved


def test_streamlit_save_writes_the_file_and_discard_restores(tmp_path, monkeypatch):
    """Save writes the edit into a temporary copy of configs/ (AIRCRAFTER2_CONFIGS), changing only that line; Discard
    drops an unsaved edit and leaves the file alone."""
    import shutil

    from streamlit.testing.v1 import AppTest

    shutil.copytree(CONFIGS, tmp_path / "configs")
    c = tmp_path / "configs"
    monkeypatch.setenv("AIRCRAFTER2_CONFIGS", str(c))
    before = (c / "placeholders.yaml").read_text().splitlines()
    at = AppTest.from_file(str(APP), default_timeout=300)
    at.run()
    at.session_state[f"{PH_KEY}_df"] = _ph_frame(c, energy_reserve="0.25")
    at.run()
    assert any("Unsaved edits" in i.value for i in at.info)
    _button(at, "Save (placeholders)").click().run()
    assert not at.exception
    after = (c / "placeholders.yaml").read_text().splitlines()
    changed = [i for i, (a, b) in enumerate(zip(before, after)) if a != b]
    assert len(before) == len(after) and len(changed) == 1 and after[changed[0]].startswith("energy_reserve: 0.25 ")
    assert not any("Unsaved edits" in i.value for i in at.info)
    saved = (c / "placeholders.yaml").read_text()
    at.session_state[f"{PH_KEY}_df"] = _ph_frame(c, energy_reserve="0.35")
    at.run()
    assert any("Unsaved edits" in i.value for i in at.info)
    _button(at, "Discard changes (placeholders)").click().run()
    assert not at.exception and not any("Unsaved edits" in i.value for i in at.info)
    assert (c / "placeholders.yaml").read_text() == saved
    assert (CONFIGS / "placeholders.yaml").read_text().splitlines() == before  # the real file was never touched


# --- the Constraints tab (aircrafter2/constraints_view.py) ------------------------------------------------------------

def _cons_key(cfg, ruleset):
    return f"cons_{cfg}_{ruleset}"


def _app(cfg=None):
    from streamlit.testing.v1 import AppTest

    at = AppTest.from_file(str(APP), default_timeout=900)
    at.run()
    if cfg:
        at.sidebar.selectbox[0].set_value(cfg).run()
    return at


def test_constraints_tab_renders():
    for cfg, tabs in (("full_mdo.yaml", 4), ("full_mdo_twin.yaml", 4), ("dbf2026_multipoint.yaml", 3)):
        at = _app(None if cfg == "full_mdo.yaml" else cfg)
        assert not at.exception, cfg
        assert len(at.tabs) == tabs and at.tabs[1 if tabs == 4 else 0].label == "Constraints"
        assert any(c.value.startswith("Design-variable bounds: Design and mission tab (Kv: sidebar)") for c in at.caption)
        assert any(b.label == "Save (constraints)" for b in at.button)
        assert any(b.label == "Run" for b in at.button) == (tabs == 4)
    assert any("span <= 5 ft" in c.value for c in at.caption)  # dbf2026: the rules enforced as DV bounds


def test_constraint_rows_left_out_of_the_layer_editors():
    at = _app()
    ph = at.session_state[f"{PH_KEY}_df"]
    assert "stability.SM_max" not in set(ph["parameter"]) and "energy_reserve" in set(ph["parameter"])
    assert any(c.value.startswith("Constraint limits (SM, tail volumes") for c in at.caption)


def test_constraints_sm_max_edit_reaches_the_driver_bound(tmp_path, monkeypatch):
    """An unsaved SM_max edit in the Constraints tab sets the SM bound of an Analyse run (num_y 11)."""
    monkeypatch.setenv("AIRCRAFTER2_OUTPUTS", str(tmp_path))  # the run's folder: not outputs/ui_runs
    at = _app()
    at.session_state[f"{_cons_key('full_mdo.yaml', 'rulesets/generic.yaml')}_edits"] = {"stability.SM_max": 0.3}
    at.run()
    assert any("Unsaved edits in the Constraints tab" in i.value for i in at.info)
    at.sidebar.select_slider[0].set_value(11)
    _button(at, "Run").click().run()
    assert not at.exception
    cons = json.loads((Path(at.session_state["last"]) / "constraints.json").read_text())["constraints"]
    assert Path(at.session_state["last"]).is_relative_to(tmp_path)
    sm = next(c for c in cons if c["name"] == "SM")
    assert (sm["lower"], sm["upper"]) == (0.05, 0.3)
    assert (CONFIGS / "placeholders.yaml").read_text().count("stability.SM_max: 0.20 ") == 1  # not saved


def test_constraints_lower_above_upper_blocks_run():
    at = _app()
    at.session_state[f"{_cons_key('full_mdo.yaml', 'rulesets/generic.yaml')}_edits"] = {"stability.SM_min": 0.3}
    at.run()
    assert not at.exception
    assert any("must not exceed stability.SM_max" in e.value for e in at.error)
    assert _button(at, "Run").disabled and _button(at, "Save (constraints)").disabled


def test_constraints_switch_off_warns_for_safety():
    at = _app()
    at.session_state[f"{_cons_key('full_mdo.yaml', 'rulesets/generic.yaml')}_off"] = ["stall_margin"]
    at.run()
    assert not at.exception
    assert any("Safety constraints switched off for this run: stall_margin" in w.value for w in at.warning)
    assert not _button(at, "Run").disabled


def test_constraints_save_guard_on_a_study_config():
    """dbf2026_multipoint.yaml: an edit to a file the study scripts read shows them and needs the confirmation."""
    at = _app("dbf2026_multipoint.yaml")
    at.session_state[f"{_cons_key('dbf2026_multipoint.yaml', 'rulesets/dbf_2025_26.yaml')}_edits"] = {
        "propulsion.rpm_fraction": 0.95}
    at.run()
    assert not at.exception
    guard = [w.value for w in at.warning if "Saving changes the inputs of these studies' next runs" in w.value]
    assert guard and "results_2026.py" in guard[0] and "dbf2026_multipoint.yaml" in guard[0]
    assert _button(at, "Save (constraints)").disabled
    [c for c in at.checkbox if c.label == "I understand: save anyway"][0].check().run()
    assert not _button(at, "Save (constraints)").disabled  # not clicked: the real configs stay untouched


def test_constraints_save_writes_the_line_in_place(tmp_path, monkeypatch):
    """Save (constraints) in a temporary copy of configs/: the guard confirmed, the problem-config line is rewritten
    in place (comment kept), nothing else changes, and the tab reloads with no unsaved edits."""
    import shutil

    shutil.copytree(CONFIGS, tmp_path / "configs")
    c = tmp_path / "configs"
    monkeypatch.setenv("AIRCRAFTER2_CONFIGS", str(c))
    files = {f: f.read_bytes() for f in c.rglob("*.yaml")}
    at = _app("dbf2026_multipoint.yaml")
    at.session_state[f"{_cons_key('dbf2026_multipoint.yaml', 'rulesets/dbf_2025_26.yaml')}_edits"] = {
        "propulsion.rpm_fraction": 0.95}
    at.run()
    [w for w in at.checkbox if w.label == "I understand: save anyway"][0].check().run()
    _button(at, "Save (constraints)").click().run()
    assert not at.exception
    changed = [f for f, b in files.items() if f.read_bytes() != b]
    assert changed == [c / "dbf2026_multipoint.yaml"]
    before = files[changed[0]].decode().splitlines()
    after = changed[0].read_text().splitlines()
    diff = [(a, b) for a, b in zip(before, after) if a != b]
    assert len(before) == len(after) and len(diff) == 1
    assert diff[0][1].startswith("  rpm_fraction: 0.95 ") and diff[0][0].split("#")[1] == diff[0][1].split("#")[1]
    assert not any("Unsaved edits in the Constraints tab" in i.value for i in at.info)
    assert (CONFIGS / "dbf2026_multipoint.yaml").read_bytes() != changed[0].read_bytes()  # the real file untouched
