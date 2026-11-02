"""AirCrafter 2.0 Streamlit UI: a thin layer over api.run_design and constraints_view.py.

Start with:  aircrafter2 ui      (or: streamlit run aircrafter2/ui/app.py)
Read top to bottom on every interaction: the sidebar, the layer tabs (placeholders and ruleset), the Constraints tab,
the design inputs, then Run. The tabs are described in the README, "Placeholders and rulesets".
"""

import json
import os
import time
import warnings
from pathlib import Path

import pandas as pd
import streamlit as st

from aircrafter2 import constraints_view as CV
from aircrafter2.api import reanalyse_with_motor, run_design, validate_kv_dv
from aircrafter2.config.layer_files import (
    frame_to_rows,
    frame_to_spreads,
    layer_files_of,
    overridden_by,
    read_rows,
    rows_to_frame,
    rows_to_layer,
    save_rows,
    spread_overridden_by,
    spreads_to_frame,
)
from aircrafter2.config.loader import load_yaml
from aircrafter2.mission.course import (
    SHIPPED_PRESETS,
    CourseConfig,
    TakeoffConfig,
    check_course,
    legs_from_rows,
    list_presets,
    load_preset,
    load_preset_data,
    rows_from_legs,
    save_preset,
)
from aircrafter2.problems.full_mdo import WOOMBAH_Y_M, PropulsionInputs, check_kv_robust, load_full_config
from aircrafter2.problems.wing_aero import Bounds
from aircrafter2.propulsion.apc import DEFAULT_DIR
from aircrafter2.robustness.uncertainty import for_kv_mode
from aircrafter2.scoring import available

ROOT = Path(__file__).resolve().parents[2]
CONFIGS = Path(os.environ.get("AIRCRAFTER2_CONFIGS", ROOT / "configs"))  # configurable for tests (a temporary copy)
OUTPUTS = Path(os.environ.get("AIRCRAFTER2_OUTPUTS", ROOT / "outputs"))  # likewise (tests: a temporary folder)

st.set_page_config(page_title="AirCrafter 2.0", layout="wide")
st.title("AirCrafter 2.0")
st.caption(
    "DBF conceptual-design MDAO: OpenAeroStruct + NeuralFoil aero, Garcia & Gamboa mass, APC/motor/battery "
    "propulsion, AirCrafter lap model. Values marked PLACEHOLDER in the configs are not USYD data."
)

# --- sidebar: choose a config and the main switches -------------------------------------------------
full_configs = sorted(p.name for p in CONFIGS.glob("*.yaml") if "propulsion" in load_yaml(p))  # incl. extends:
with st.sidebar:
    st.header("Setup")
    cfg_name = st.selectbox(
        "Config", full_configs, index=full_configs.index("full_mdo.yaml") if "full_mdo.yaml" in full_configs else 0
    )
cfg_path = CONFIGS / cfg_name
is_multipoint = "multipoint" in load_yaml(cfg_path)
ph_path, cfg_ruleset = layer_files_of(cfg_path)


# --- the layer tabs: placeholders.yaml and the ruleset, edited (and saved in place) before the config is loaded ------
def _rel(path):
    """A path relative to configs/, or just its name."""
    path = Path(path).resolve()
    return str(path.relative_to(CONFIGS.resolve())) if path.is_relative_to(CONFIGS.resolve()) else path.name


def _same(a, b, keys):
    """True if two rows agree on these keys."""
    return all(str(a.get(k)) == str(b.get(k)) for k in keys)


CONSTRAINT_ROWS = CV.tab_paths()  # constraint limits: edited (and saved back here) in the Constraints tab only


def layer_editor(path, label, overridden, spread_over):
    """Editors for one layer file (values, then RDO spreads); Save / Discard. Returns the layer data with the
    current (possibly unsaved) edits, which the run uses. The constraint-limit rows are left to the Constraints tab:
    not shown, never rewritten here, but still in the returned layer data."""
    name = _rel(path)
    key = f"layer_{cfg_name}_{name}"
    all_rows, spreads = read_rows(path)
    hidden = [r for r in all_rows if r["parameter"] in CONSTRAINT_ROWS]
    rows = [r for r in all_rows if r["parameter"] not in CONSTRAINT_ROWS]
    if f"{key}_v" not in st.session_state:
        st.session_state[f"{key}_v"] = 0
    if f"{key}_df" not in st.session_state:
        st.session_state[f"{key}_df"] = rows_to_frame(rows, overridden)
    if f"{key}_sp" not in st.session_state:
        st.session_state[f"{key}_sp"] = spreads_to_frame(spreads, spread_over)
    v = st.session_state[f"{key}_v"]
    st.caption(
        f"`configs/{name}`: one row per value. Edit value, status and source; **overridden by** names the file "
        f"and line of `{cfg_name}` (or its extends: chain) that sets the value instead, so an edit there will "
        "not reach this config's runs."
    )
    if hidden:
        st.caption(
            "Constraint limits (SM, tail volumes, rpm fraction, current limit, take-off distance, rule "
            "limits, ...): **Constraints tab**, which saves them back into this file."
        )
    edited = st.data_editor(
        st.session_state[f"{key}_df"],
        key=f"{key}_ed{v}",
        hide_index=True,
        width="stretch",
        disabled=["area", "parameter", "units", "Q", "overridden by", "line"],
        column_config={
            "status": st.column_config.SelectboxColumn("status", options=["PLACEHOLDER", "cited"]),
            "line": None,
        },
    )
    new_rows = [r for r in frame_to_rows(edited) if r["parameter"] not in CONSTRAINT_ROWS]
    new_sp = spreads
    if spreads:
        st.markdown("**RDO spreads** (shared uncertain parameters; merged by name with a config's uncertainty file)")
        edited_sp = st.data_editor(
            st.session_state[f"{key}_sp"],
            key=f"{key}_sped{v}",
            hide_index=True,
            width="stretch",
            disabled=["name", "units", "kv_mode", "source", "Q", "overridden by", "line"],
            column_config={
                "dist": st.column_config.SelectboxColumn("dist", options=["uniform", "normal", "triangular"]),
                "status": st.column_config.SelectboxColumn("status", options=["PLACEHOLDER", "cited"]),
                "relative": st.column_config.CheckboxColumn("relative"),
                "line": None,
            },
        )
        new_sp = frame_to_spreads(edited_sp, spreads)
    unsaved = any(not _same(a, b, ("value", "status", "Q", "source")) for a, b in zip(rows, new_rows)) or any(
        not _same(a, b, ("dist", "nominal", "spread", "relative", "status", "note")) for a, b in zip(spreads, new_sp)
    )
    if unsaved:
        st.info(f"Unsaved edits in configs/{name}: they apply to the current run. Save writes them to the file.")
    b1, b2 = st.columns(2)
    if b1.button(f"Save ({label})", key=f"{key}_save", disabled=not unsaved):
        try:
            save_rows(path, new_rows, new_sp if spreads else None)
        except ValueError as exc:
            st.error(str(exc))
        else:
            for k in ("df", "sp"):
                st.session_state.pop(f"{key}_{k}", None)
            st.session_state[f"{key}_v"] += 1
            st.session_state["saved_msg"] = f"Saved configs/{name}."
            st.rerun()
    if b2.button(f"Discard changes ({label})", key=f"{key}_discard", disabled=not unsaved):
        for k in ("df", "sp"):
            st.session_state.pop(f"{key}_{k}", None)
        st.session_state[f"{key}_v"] += 1
        st.rerun()
    return rows_to_layer(new_rows + hidden, new_sp if spreads else None)


# --- the Constraints tab: every constraint, check and limit but the design-variable bounds (constraints_view.py) ------
GROUPS = ["performance", "propulsion", "stability", "structure", "aero", "twin", "rules", "scoring"]
CELL_HELP = {
    "lower": "the driver's lower bound: editable where a config value is the bound (SM_min, V_h_min, V_v_min, "
    "r_tube_min, laps_m2_min)",
    "upper": "the driver's upper bound: editable where a config value is the bound (SM_max, V_h_max, V_v_max, "
    "wall_ratio_max, sigma_cap)",
    "limit": "the physical limit behind a margin's 0 bound (column 'limit field'): rpm_fraction, I_max, pack_I_max, "
    "takeoff_distance, c_clear, the rule limits (togw_max_lb, energy_total_max_Wh, fuse_max_A, c_rating, "
    "pax_per_cargo_min, fuselage_span_ratio_max), rudder_max_deg, oei_speed_factor",
}


@st.cache_data(show_spinner=False)
def _study_readers(configs_dir):
    """constraints_view.study_readers with string keys, for the session cache."""
    return {str(k): v for k, v in CV.study_readers(configs_dir).items()}


def _frame(rows, cols):
    """Rows sorted by group and point, and their table."""
    order = {g: i for i, g in enumerate(GROUPS)}
    rows = sorted(rows, key=lambda r: (order.get(r["group"], len(GROUPS)), r["point"]))
    return rows, pd.DataFrame([{c: r.get(c) for c in cols} for r in rows], columns=cols)


def constraints_tab(cfg, key, runnable):
    """The Constraints tab for cfg (FullMDOConfig or MultipointConfig, the layer tabs' unsaved edits included).
    State per config and ruleset (session_state[key + _edits | _comments | _off | _robust | _v]). Returns
    (the config the run uses, error messages that block Run, safety constraints switched off)."""
    # --- state and captions ---
    ss = st.session_state
    mp = not runnable
    base = cfg.base if mp else cfg
    rdo = runnable and base.robust.enabled
    for suffix, default in (("edits", {}), ("comments", {}), ("robust", None), ("v", 0)):
        ss.setdefault(f"{key}_{suffix}", default.copy() if isinstance(default, dict) else default)
    if runnable:
        ss.setdefault(f"{key}_off", sorted(CV.config_off(cfg)))
    edits, comments = ss[f"{key}_edits"], ss[f"{key}_comments"]
    off = set(ss.get(f"{key}_off", []))
    st.caption("Design-variable bounds: Design and mission tab (Kv: sidebar).")
    rules = CV.dv_bound_rules(cfg)
    if rules:
        st.caption(
            "Rules enforced as design-variable bounds (not rows below): "
            + "; ".join(t for _, t in rules)
            + ". Span: Ruleset tab; the others: the config's design_variables and multipoint.payload."
        )
    st.caption(
        "Edit **lower** / **upper** where a config value is the bound, and **limit** where a margin's 0 "
        "bound stands for a physical limit (the **limit field** column names it). Unsaved edits apply to the "
        "current run; Save writes each value to the file and line in **set in**. "
        + (
            "On/off and robust apply to this run only and are not saved, except require_3lap_energy "
            "(energy_margin's on) and robust.robust_constraints, which are config fields."
            if runnable
            else "Multipoint configs are run by the study scripts, so there is no on/off here."
        )
    )

    # the table: every constraint present (switched-off ones too), with the edits and the robust choice
    # --- the table with the current edits ---
    tcfg = CV.apply_limits(cfg, edits, off=set() if runnable else None)
    robust = None
    if rdo:
        from aircrafter2.problems.full_mdo import full_constraints

        names = [n for n, _ in full_constraints(tcfg)]
        sel = ss[f"{key}_robust"] if ss[f"{key}_robust"] is not None else list(base.robust.robust_constraints)
        robust = [n for n in sel if n in names]
        tcfg.robust.robust_constraints = robust
    errors = CV.validate_limits(tcfg)
    try:
        with st.spinner("Setting up the problem (constraints) ..."):
            rows = CV.constraint_table(tcfg, config_path=cfg_path, placeholders_path=ph_path, ruleset_path=rs_path)
    except ValueError as exc:
        st.error(f"Constraints table: {exc}")
        return cfg, [str(exc)], []
    if runnable:
        names = {r["constraint"] for r in rows if r["section"] == 1}
        off &= names
    last = ss.get("last")
    if last and ss.get("last_cons_key") == key:
        out = Path(last)
        cj = json.loads((out / "constraints.json").read_text()) if (out / "constraints.json").exists() else {}
        rj = json.loads((out / "results.json").read_text()) if (out / "results.json").exists() else {}
        CV.run_states(rows, cj, rj)
        st.caption(
            f"value, margin and state: run {out.name} (active: |margin| / ref < {CV.ACTIVE_TOL:g}; "
            f"violated: beyond the bound by more than the driver's tolerance {cj.get('tol', '-')})."
        )
    for r in rows:
        r.update(
            on=r["constraint"] not in off,
            robust=r["constraint"] in (robust or []),
            display=r["name"] if r["section"] == 1 else r["constraint"],
        )

    # --- sections 1 and 2: the editors ---
    v = ss[f"{key}_v"]
    new_edits, new_off, new_robust, ignored = dict(edits), set(off), list(robust or []), []
    for section in (1, 2):
        sec = [r for r in rows if r["section"] == section]
        if not sec:
            continue
        st.markdown(f"**{section}. {CV.SECTIONS[section]}**")
        cols = (
            (["on"] if runnable and section == 1 else [])
            + (["robust"] if rdo and section == 1 else [])
            + [
                "group",
                "point",
                "display",
                "definition",
                "lower",
                "upper",
                "limit",
                "limit_field",
                "units",
                "effective",
                "set_in",
                "status",
                "Q",
                "kind",
                "value",
                "margin",
                "state",
            ]
        )
        sec, df = _frame(sec, cols)
        shape = hash(tuple(r["name"] for r in sec))
        edited = st.data_editor(
            df,
            key=f"{key}_ed{section}_{v}_{shape}",
            hide_index=True,
            width="stretch",
            disabled=[c for c in cols if c not in ("on", "robust", "lower", "upper", "limit")],
            column_config={
                "on": st.column_config.CheckboxColumn("on", help="off: dropped from this run"),
                "robust": st.column_config.CheckboxColumn("robust", help="mu -/+ beta sigma (RDO)"),
                "lower": st.column_config.NumberColumn("lower", help=CELL_HELP["lower"], format="%g"),
                "upper": st.column_config.NumberColumn("upper", help=CELL_HELP["upper"], format="%g"),
                "limit": st.column_config.NumberColumn("limit", help=CELL_HELP["limit"], format="%g"),
                "display": st.column_config.TextColumn("constraint"),
                "limit_field": st.column_config.TextColumn("limit field"),
                "effective": st.column_config.TextColumn("effective limit"),
                "set_in": st.column_config.TextColumn("set in"),
            },
        )
        new_edits, new_off, new_robust, more = CV.editor_changes(sec, df, edited, new_edits, new_off, new_robust)
        ignored += more
    for msg in ignored:
        st.warning(f"Edit ignored: {msg}.")
    if new_edits != edits or new_off != off or (rdo and new_robust != robust):
        ss[f"{key}_edits"] = new_edits
        if runnable:
            ss[f"{key}_off"] = sorted(new_off)
        if rdo and new_robust != robust:
            ss[f"{key}_robust"] = list(dict.fromkeys(new_robust))
        ss[f"{key}_v"] += 1
        st.rerun()

    # --- sections 3 to 5: read-only ---
    for section, note in (
        (3, "not enforced by the optimiser"),
        (4, "solved by the MDA's solvers"),
        (5, "not used by this problem"),
    ):
        sec = [r for r in rows if r["section"] == section]
        if not sec:
            continue
        st.markdown(f"**{section}. {CV.SECTIONS[section]}** ({note}; read-only)")
        sec, df = _frame(
            sec, ["group", "point", "constraint", "definition", "effective", "units", "set_in", "status", "kind"]
        )
        df.insert(
            3,
            "enforced",
            note
            if section != 3
            else ["by the G&G sizing" if r["constraint"].startswith("sizing.") else "not enforced" for r in sec],
        )
        st.dataframe(
            df.style.set_properties(color="gray") if section == 5 else df,
            hide_index=True,
            width="stretch",
            column_config={"effective": "effective limit / value", "set_in": "set in"},
        )

    # comments of the layer-file values (status, Q, source), as in the layer editors
    # --- the comments table ---
    paths = sorted({p for r in rows for p in r["cells"].values()})
    srcs = {
        p: next(s for r in rows for s in r["sources"] if s["path"] == p)
        for p in paths
        if any(s["path"] == p for r in rows for s in r["sources"])
    }
    layer = [p for p in paths if p in srcs and srcs[p]["kind"] in ("placeholders", "ruleset")]
    if layer:
        with st.expander("Comments of the limit values in the layer files (status, Q, source)"):
            cdf = pd.DataFrame(
                [
                    dict(
                        parameter=p,
                        value=CV.get_value(tcfg, p),
                        set_in=srcs[p]["text"],
                        **{k: comments.get(p, {}).get(k, srcs[p][k]) for k in ("status", "Q", "source")},
                    )
                    for p in layer
                ]
            )
            ced = st.data_editor(
                cdf,
                key=f"{key}_com_{v}",
                hide_index=True,
                width="stretch",
                disabled=["parameter", "value", "set_in"],
                column_config={
                    "status": st.column_config.SelectboxColumn("status", options=["PLACEHOLDER", "cited"]),
                    "set_in": "set in",
                },
            )
            new_c = {
                r["parameter"]: {k: r[k] or "" for k in ("status", "Q", "source")}
                for r in ced.to_dict("records")
                if any((r[k] or "") != srcs[r["parameter"]][k] for k in ("status", "Q", "source"))
            }
            if new_c != comments:
                ss[f"{key}_comments"] = new_c
                st.rerun()

    # the config the run uses
    # --- what the run uses ---
    run_cfg = CV.apply_limits(
        cfg, edits, off=off if runnable else None, robust=[n for n in robust if n not in off] if rdo else None
    )
    if rdo:
        with st.sidebar:
            st.caption(
                f"{len(run_cfg.robust.robust_constraints)} robust constraints (Constraints tab): "
                + ", ".join(run_cfg.robust.robust_constraints)
            )
    safety_off = sorted(n for n in off if CV.REGISTER[n].kind == "safety")

    # save / discard / reset
    # --- saving ---
    values = dict(edits)
    if runnable and ("energy_margin" in off) == cfg.require_3lap_energy:
        values["require_3lap_energy"] = "energy_margin" not in off
    if (
        rdo
        and ss[f"{key}_robust"] is not None
        and run_cfg.robust.robust_constraints != list(base.robust.robust_constraints)
    ):
        values["robust.robust_constraints"] = run_cfg.robust.robust_constraints
    unsaved = bool(values or comments)
    if unsaved:
        st.info(
            "Unsaved edits in the Constraints tab: they apply to the current run. Save writes them to the files "
            "that set them."
        )
    for msg in errors:
        st.error(msg)
    files = sorted(
        {str(CV.limit_source(p, cfg_path, ph_path, rs_path)["file"]) for p in set(values) | set(comments)} - {"None"}
    )
    readers = _study_readers(str(CONFIGS))
    hit = {f: readers.get(str(Path(f).resolve()), []) for f in files}
    hit = {f: s for f, s in hit.items() if s}
    ok = True
    if unsaved and hit:
        st.warning(
            "Saving changes the inputs of these studies' next runs:\n\n"
            + "\n".join(f"- `{_rel(f)}`: read by {', '.join(s)}" for f, s in hit.items())
        )
        ok = st.checkbox("I understand: save anyway", key=f"{key}_guard_{v}")
    b1, b2, b3 = st.columns(3)
    if b1.button("Save (constraints)", key=f"{key}_save", disabled=not unsaved or bool(errors) or not ok):
        res = CV.save_limits(cfg_path, values, comments, ph_path, rs_path)
        for msg in res["errors"]:
            st.error(msg)
        if not res["errors"]:
            for k in ("edits", "comments", "robust"):
                ss.pop(f"{key}_{k}", None)
            ss.pop(f"{key}_off", None)
            for k in [k for k in ss if str(k).startswith(f"layer_{cfg_name}_")]:
                if str(k).endswith(("_df", "_sp")):
                    ss.pop(k)
            ss["saved_msg"] = "Saved " + ", ".join(_rel(f) for f in res["written"]) + "."
            st.rerun()
    if b2.button("Discard changes (constraints)", key=f"{key}_discard", disabled=not (edits or comments)):
        for k in ("edits", "comments"):
            ss.pop(f"{key}_{k}", None)
        ss[f"{key}_v"] += 1
        st.rerun()
    if b3.button("Reset to config values", key=f"{key}_reset"):
        for k in ("edits", "comments", "robust", "off"):
            ss.pop(f"{key}_{k}", None)
        ss[f"{key}_v"] += 1
        st.rerun()
    return run_cfg, errors, safety_off


tab_names = (
    ["Constraints", "Placeholders", "Ruleset"]
    if is_multipoint
    else ["Design and mission", "Constraints", "Placeholders", "Ruleset"]
)
tabs = dict(zip(tab_names, st.tabs(tab_names)))
if st.session_state.get("saved_msg"):
    st.success(st.session_state.pop("saved_msg"))
rulesets = sorted(_rel(p) for p in (CONFIGS / "rulesets").glob("*.yaml"))
with tabs["Ruleset"]:  # the selector first: the placeholders' 'overridden by' column reads the chosen ruleset
    rs_name = st.selectbox(
        "Ruleset (configs/rulesets/)",
        rulesets,
        key=f"ruleset_{cfg_name}",
        index=rulesets.index(_rel(cfg_ruleset)) if cfg_ruleset and _rel(cfg_ruleset) in rulesets else 0,
        help="defaults to the ruleset the config names; the run uses the one chosen here",
    )
rs_path = CONFIGS / rs_name
with tabs["Placeholders"]:
    ph_rows, ph_spreads = read_rows(ph_path)
    ph_layer = layer_editor(
        ph_path,
        "placeholders",
        {r["parameter"]: overridden_by(cfg_path, r["parameter"], rs_path) for r in ph_rows},
        {s["name"]: spread_overridden_by(cfg_path, s["name"]) for s in ph_spreads},
    )
with tabs["Ruleset"]:
    rs_rows, rs_spreads = read_rows(rs_path)
    rs_layer = layer_editor(
        rs_path,
        "ruleset",
        {r["parameter"]: overridden_by(cfg_path, r["parameter"]) for r in rs_rows},
        {s["name"]: spread_overridden_by(cfg_path, s["name"]) for s in rs_spreads},
    )
cons_key = f"cons_{cfg_name}_{rs_name}"
if is_multipoint:
    from aircrafter2.problems.multipoint import load_multipoint_config

    with tabs["Constraints"]:
        constraints_tab(
            load_multipoint_config(cfg_path, placeholders=ph_layer, ruleset=rs_layer), cons_key, runnable=False
        )
    st.info(
        "This is a multipoint config: the UI runs single-point problems only. Edit and save its constraints, "
        "placeholders and ruleset here; the runs are `studies/results_2025.py` (and `studies/takeoff_study.py`) "
        "and `studies/results_2026.py`, which read the same files."
    )
    st.stop()
cfg = load_full_config(cfg_path, placeholders=ph_layer, ruleset=rs_layer)
with st.sidebar:
    from aircrafter2.scoring.base import _REGISTRY

    plugins = [n for n in available() if not getattr(_REGISTRY.get(n), "multipoint", False)]  # single-point only
    cfg.scoring.plugin = st.selectbox("Scoring plug-in", plugins, index=plugins.index(cfg.scoring.plugin))
    apc_dir = Path(cfg.propulsion.apc_dir).expanduser() if cfg.propulsion.apc_dir else DEFAULT_DIR
    props = sorted(p.stem.replace("PER3_", "") for p in apc_dir.glob("PER3_*E.dat"))
    cfg.propulsion.prop = st.selectbox(
        "APC propeller", props, index=props.index(cfg.propulsion.prop) if cfg.propulsion.prop in props else 0
    )
    st.subheader("Propulsion layout")
    pr = cfg.propulsion.model_dump()
    pr["n_motors"] = st.radio(
        "Layout",
        [1, 2],
        index=pr["n_motors"] - 1,
        horizontal=True,
        format_func=lambda n: "1: nose tractor" if n == 1 else "2: wing-mounted pair",
    )
    pr["battery_topology"] = st.radio(
        "Battery topology",
        ["per_motor", "shared"],
        index=["per_motor", "shared"].index(pr["battery_topology"]),
        horizontal=True,
        help="per_motor: one pack per motor; shared: one pack feeds every ESC",
    )
    st.caption("Battery capacity (design variable `capacity`) is the capacity **per pack**.")
    if pr["n_motors"] == 2:
        if pr["y_motor"] is not None:
            pr["y_motor"] = st.number_input(
                "Motor station y_motor (fraction of semi-span)", 0.05, 0.95, float(pr["y_motor"])
            )
        else:
            pr["y_motor_m"] = st.number_input(
                "Motor station y_m (m from centreline; Woombah 0.2515)",
                0.05,
                2.0,
                float(pr["y_motor_m"] or WOOMBAH_Y_M),
            )
        st.caption("Nacelle mass, length and diameter: Placeholders tab.")
        pr["Q_nac"] = st.number_input("Nacelle interference factor Q (Woombah 1.3)", 1.0, 2.0, float(pr["Q_nac"]))
        cfg.mass.options.motor_inertia_relief = st.checkbox(
            "Motor inertia relief on the spar", value=cfg.mass.options.motor_inertia_relief
        )
    try:
        cfg.propulsion = PropulsionInputs(**pr)
    except ValueError as exc:
        st.error(str(exc))
        st.stop()
    st.subheader("Motor")
    kv_errors = []
    kv_on = st.checkbox(
        "Optimise motor Kv",
        value="Kv" in cfg.design_variables,
        help="Kv as a design variable: the reference motor below is rewound to the design Kv (same "
        "frame; Rm and I0 scale with Kv) and the current limit becomes a constraint. Off: a fixed motor",
    )
    if kv_on:
        cfg.propulsion.Kv = st.number_input(
            "Reference motor Kv (rpm/V) - PLACEHOLDER", 100.0, 5000.0, float(cfg.propulsion.Kv)
        )
        st.caption(
            f"Reference motor: Rm = {cfg.propulsion.Rm:g} ohm, I0 = {cfg.propulsion.I0:g} A (PLACEHOLDERS, "
            "Q7). At Kv: Rm x (Kv_ref/Kv)^2, I0 x Kv/Kv_ref."
        )
        bd = cfg.design_variables.get("Kv", Bounds(lower=500.0, upper=1500.0))
        kv_lo = st.number_input("Kv lower bound (rpm/V)", value=float(bd.lower), step=50.0)
        kv_hi = st.number_input("Kv upper bound (rpm/V)", value=float(bd.upper), step=50.0)
        kv_x0 = st.number_input("Kv start (rpm/V)", value=float(cfg.initial.get("Kv", cfg.propulsion.Kv)), step=50.0)
        kv_errors = validate_kv_dv(kv_lo, kv_hi, kv_x0)
        for msg in kv_errors:
            st.error(msg)
        if not kv_errors:
            cfg.design_variables["Kv"] = Bounds(lower=kv_lo, upper=kv_hi)
            cfg.initial["Kv"] = kv_x0
            kb = cfg.design_variables["Kv"]  # read back: what the run will use
            st.caption(f"Kv design variable: {kb.lower:g} to {kb.upper:g} rpm/V, start {cfg.initial['Kv']:g}.")
    else:
        cfg.design_variables.pop("Kv", None)
        cfg.initial.pop("Kv", None)
        cfg.propulsion.Kv = st.number_input("Motor Kv (rpm/V) - PLACEHOLDER", 100.0, 5000.0, float(cfg.propulsion.Kv))
    cfg.aero.num_y = st.select_slider("Spanwise VLM nodes (half wing)", [11, 21, 41], value=cfg.aero.num_y)
    cfg.fuselage_drag = st.checkbox("Fuselage drag", value=cfg.fuselage_drag)
    st.subheader("Robust design (RDO)")
    robust_on = st.checkbox(
        "Robust design optimisation",
        value=cfg.robust.enabled,
        help="minimise mu + k sigma of the score with mu -/+ beta sigma constraints over fixed "
        "uncertainty nodes (docs/modules/robustness.md). Spreads: configs/uncertainty.yaml "
        "(PLACEHOLDERS)",
    )
    if robust_on:
        rdo_cfg = "full_mdo_rdo.yaml" if kv_on else "full_mdo_rdo_fixed_kv.yaml"
        names = [p.name for p in for_kv_mode(cfg.robust.parameters, kv_on)]
        if not cfg.robust.parameters or any(n not in names for n in (cfg.robust.active or [])):
            # no robust block (e.g. full_mdo.yaml), or one for the other motor mode: take the matching RDO config's
            cfg.robust = load_full_config(CONFIGS / rdo_cfg).robust
            st.caption(f"Robust settings from {rdo_cfg} ({'Kv a design variable' if kv_on else 'fixed motor'}).")
        rob = cfg.robust
        names = [p.name for p in for_kv_mode(rob.parameters, kv_on)]
        rob.active = st.multiselect(
            "Propagated parameters (others at their means)",
            names,
            default=rob.active if rob.active is not None else names,
        )
        rob.quadrature = st.radio(
            "Quadrature",
            ["udr", "smolyak"],
            index=0 if rob.quadrature != "smolyak" else 1,
            horizontal=True,
            help="udr: 2d + 1 nodes (no cross terms); smolyak level 2: cross terms, many more nodes",
        )
        rob.k = st.slider("k in mu + k sigma", 0.0, 3.0, float(rob.k), 0.25)
        rob.beta = st.slider("beta (constraint margin, ~97.7 % at 2 if normal)", 0.0, 3.0, float(rob.beta), 0.25)
        try:
            import cyipopt  # noqa: F401

            opts = ["slsqp", "ipopt"]
        except ImportError:
            opts = ["slsqp"]
        rob.optimizer = st.radio(
            "Optimiser", opts, index=opts.index(rob.optimizer) if rob.optimizer in opts else 0, horizontal=True
        )
        d = len(rob.active)
        n_nodes = 2 * d + 1 if rob.quadrature == "udr" else None
        st.caption(
            f"{d} parameters: "
            + (
                f"{n_nodes} analysis nodes, about {n_nodes} x the deterministic cost."
                if n_nodes
                else "Smolyak level 2: many nodes (127 for d = 7); slow."
            )
        )
    cfg.robust.enabled = robust_on
    mode = st.radio("Run", ["Analyse (no optimisation)", "Optimise"], index=0)
    figs = st.checkbox("Make figures", value=True)

# --- main: editable inputs (Design and mission tab) ---------------------------------------------------------------
with tabs["Design and mission"]:
    c1, c2, c3 = st.columns(3)
    with c1:
        st.subheader("Design-variable bounds")
        st.caption("Motor Kv, if optimised, is set in the sidebar (Motor).")
        dv = pd.DataFrame(
            [
                {"variable": k, "lower": b.lower, "upper": b.upper, "initial": cfg.initial.get(k, None)}
                for k, b in cfg.design_variables.items()
                if k != "Kv"
            ]
        )
        dv = st.data_editor(dv, hide_index=True, disabled=["variable"], key="dv")
        for _, row in dv.iterrows():
            cfg.design_variables[row["variable"]] = Bounds(lower=float(row["lower"]), upper=float(row["upper"]))
            if pd.notna(row["initial"]):
                cfg.initial[row["variable"]] = float(row["initial"])
    with c2:
        st.subheader("Key assumptions")
        cfg.mass.inputs["n"] = st.number_input("Load factor", 1.0, 12.0, float(cfg.mass.inputs.get("n", 5.0)))
        cfg.propulsion.n_cells = st.number_input(
            "Battery cells (series)", 1.0, 12.0, float(cfg.propulsion.n_cells), 1.0
        )
        st.caption(
            "Payload mass, energy reserve and the other placeholders: Placeholders tab; the current limit and "
            "the other constraint limits: Constraints tab."
        )
    with c3:
        st.subheader("Mission")
        to0 = cfg.course.takeoff
        to_on = st.checkbox(
            "Include take-off in mission time",
            value=to0.in_mission_time,
            help="add the ground roll (rest to lift-off, into the wind, at the throttle) to lap 1's "
            "time and energy: the rules time every mission from the first throttle advance. Off: the "
            "original model, lap 1 starts in the air at V_stall (every published result)",
        )
        k_lof, cl_ground, start_off = to0.k_lof, to0.CL_ground, to0.start_offset_ft
        st.caption(
            f"Take-off parameters: k_lof {k_lof:g}, CL_ground {cl_ground:g} are in the Placeholders tab, the "
            f"take-off distance {cfg.takeoff_distance:g} m (Eq. 3.3) in the Constraints tab; the start offset "
            f"({start_off:g} ft) and the mission time ({cfg.mission_time:g} s) in the Ruleset tab."
        )
        cfg.mu_roll = st.number_input(
            "Rolling friction mu_roll",
            0.0,
            0.5,
            float(cfg.mu_roll),
            0.01,
            help="used by the Eq. 3.3 constraint and, with take-off on, the ground roll",
        )
        reaction = st.number_input(
            "Pilot reaction time per lap (s)", 0.0, 30.0, float(cfg.course.pilot_reaction_time_s), 0.5
        )

    # --- mission: course layout ---------------------------------------------------------------------------
    LEG_COLS = ["order", "kind", "length_ft", "angle_deg", "wind"]
    LEG_COLUMN_CONFIG = {
        "order": st.column_config.NumberColumn("order", help="legs are flown in this order", step=1),
        "kind": st.column_config.SelectboxColumn("kind", options=["straight", "turn"], required=True),
        "length_ft": st.column_config.NumberColumn("length (ft)", help="straights", min_value=0.0),
        "angle_deg": st.column_config.NumberColumn("angle (deg)", help="turns", min_value=0.0),
        "wind": st.column_config.SelectboxColumn(
            "wind", options=["head", "tail", "none"], help="straights: along-track wind (none = crosswind)"
        ),
    }

    def leg_table(label, legs0, key):
        """Editable leg table (rows can be added and deleted); kept in session state so 'Add leg' can append a row."""
        dfk, vk = f"{key}_df", f"{key}_v"
        if dfk not in st.session_state:
            st.session_state[dfk] = pd.DataFrame(rows_from_legs(legs0), columns=LEG_COLS)
            st.session_state[vk] = 0
        edited = st.data_editor(
            st.session_state[dfk],
            key=f"{key}_ed{st.session_state[vk]}",
            num_rows="dynamic",
            hide_index=True,
            column_config=LEG_COLUMN_CONFIG,
            width="stretch",
        )
        b1, b2 = st.columns(2)
        if b1.button(f"Add leg ({label})", key=f"{key}_add"):
            last = pd.to_numeric(edited["order"], errors="coerce").max()
            new = dict(
                order=(0 if pd.isna(last) else int(last)) + 1,
                kind="straight",
                length_ft=100.0,
                angle_deg=None,
                wind="none",
            )
            st.session_state[dfk] = pd.concat([edited, pd.DataFrame([new], columns=LEG_COLS)], ignore_index=True)
            st.session_state[vk] += 1
            st.rerun()
        if b2.button(f"Sort by order ({label})", key=f"{key}_sort"):
            st.session_state[dfk] = edited.sort_values("order", kind="stable", na_position="last").reset_index(
                drop=True
            )
            st.session_state[vk] += 1
            st.rerun()
        return edited

    st.subheader("Mission: course")
    presets = list_presets()
    cfg_preset = cfg.course.preset if cfg.course.legs is None and cfg.course.preset in presets else None
    cc1, cc2 = st.columns([3, 2])
    with cc1:
        preset = st.selectbox(
            "Course preset (configs/courses/)",
            presets,
            index=presets.index(cfg_preset or ("dbf_2025" if "dbf_2025" in presets else presets[0])),
            key=f"preset_{cfg_name}",
        )
        st.caption(f"Source: {load_preset_data(preset).get('citation', '-')}")
        if preset == (cfg_preset or "dbf_2025") and cfg.course.legs is not None:
            src_lap1, src_other = cfg.course.resolve()  # the config's own inline legs
        else:
            src_lap1, src_other = load_preset(preset)
        tkey = f"legs_{cfg_name}_{preset}"
        other_df = leg_table("lap", src_other, tkey)
        other, course_errors = legs_from_rows(other_df.to_dict("records"), "lap")
        diff = st.checkbox("Different first lap", value=src_lap1 != src_other, key=f"{tkey}_diff")
        lap1 = other
        if diff:
            st.caption(
                "Lap 1 (with take-off on, it must start with a head-wind straight: the ground roll is into the wind)"
            )
            lap1_df = leg_table("lap 1", src_lap1, f"{tkey}_lap1")
            lap1, e1 = legs_from_rows(lap1_df.to_dict("records"), "lap 1")
            course_errors += e1
        course_warns = []
        if not course_errors:
            course_errors, course_warns = check_course(lap1, other, takeoff_on=to_on)
        if not course_errors:
            same = other == load_preset(preset)[1] and lap1 == load_preset(preset)[0]
            try:
                with warnings.catch_warnings():
                    warnings.simplefilter("ignore")
                    cfg.course = CourseConfig(
                        preset=preset if same else None,
                        legs=None if same else other,
                        lap1_legs=None if same or not diff else lap1,
                        pilot_reaction_time_s=reaction,
                        takeoff=TakeoffConfig(
                            in_mission_time=to_on, k_lof=k_lof, CL_ground=cl_ground, start_offset_ft=start_off
                        ),
                    )
            except ValueError as exc:
                course_errors.append(str(exc))
    with cc2:
        for msg in course_errors:
            st.error(msg)
        for msg in course_warns:
            st.warning(msg)
        if not course_errors:
            import matplotlib.pyplot as plt

            from aircrafter2.plotting.course_sketch import course_sketch

            fig = course_sketch(other, lap1 if diff else None, takeoff_on=to_on, start_offset_ft=start_off)
            st.pyplot(fig)
            plt.close(fig)
            st.caption(
                "Top view, straights to scale; turn radii are schematic (the lap model's turns follow the "
                "flight "
                "state). Wind arrow: the wind blows against the head-wind straights."
            )
        with st.expander("Save this course as a preset"):
            pname = st.text_input("Preset name (letters, digits, _ and -)", key="preset_name")
            if st.button("Save preset", disabled=bool(course_errors) or not pname):
                if pname in SHIPPED_PRESETS:
                    st.error(f"'{pname}' is a shipped preset and cannot be overwritten: choose another name.")
                else:
                    try:
                        path = save_preset(
                            pname, other, lap1 if diff else None, citation="user-defined course (AirCrafter 2.0 UI)"
                        )
                        st.success(f"Saved {path.relative_to(ROOT) if path.is_relative_to(ROOT) else path}.")
                    except ValueError as exc:
                        st.error(str(exc))


with tabs["Constraints"]:
    run_cfg, cons_errors, safety_off = constraints_tab(cfg, cons_key, runnable=True)

# --- run (below the tabs) -------------------------------------------------------------------------------------------
if run_cfg.disabled_constraints or not run_cfg.require_3lap_energy:
    off_now = list(run_cfg.disabled_constraints) + ([] if run_cfg.require_3lap_energy else ["energy_margin"])
    st.caption("Switched off for this run (Constraints tab): " + ", ".join(off_now))
if safety_off:
    st.warning(f"Safety constraints switched off for this run: {', '.join(safety_off)}.")
try:
    if cfg.robust.enabled:
        check_kv_robust(cfg)
except ValueError as exc:
    kv_errors.append(str(exc))
    st.error(str(exc))
if cons_errors:
    st.error("Run is blocked by an invalid limit in the Constraints tab.")
if st.button("Run", type="primary", disabled=bool(kv_errors or course_errors or cons_errors)):
    out = OUTPUTS / "ui_runs" / time.strftime("%Y%m%d_%H%M%S")  # named studies stay at the top of outputs/
    with st.status("Running AirCrafter 2.0 ...", expanded=True) as status:
        t0 = time.perf_counter()
        try:
            res = run_design(
                run_cfg,
                optimise=mode == "Optimise",
                out_dir=out,
                figures=figs,
                sensitivities=mode == "Optimise",
                progress=lambda s: st.write(f"{s} ..."),
            )
        except Exception as exc:  # show the failure to the user instead of a stack trace
            status.update(label=f"Failed: {exc}", state="error")
            st.stop()
        status.update(
            label=f"Done in {time.perf_counter() - t0:.0f} s (success = {res.success}, iterations = {res.iterations})",
            state="complete",
        )
    st.session_state["last"] = out
    st.session_state["last_cons_key"] = cons_key
    p = res.prob
    st.session_state["last_design"] = dict(
        cfg=run_cfg.model_copy(deep=True),
        dv={k: float(p.get_val(k)[0]) for k in cfg.design_variables},
        motor=res.values["motor"],
        m_motor=float(cfg.propulsion.m_motor),
        reference={
            k: float(p.get_val(k)[0])
            for k in [
                "score",
                "ThreeLapTime",
                "m_total",
                "I_margin",
                "rpm_margin",
                "energy_margin",
                "thrust_margin_static",
                "SM",
            ]
        },
    )

if "last" in st.session_state:
    out = Path(st.session_state["last"])
    st.subheader(f"Results: {out.name}")
    summary = (out / "summary.md").read_text() if (out / "summary.md").exists() else ""
    rj = json.loads((out / "results.json").read_text()) if (out / "results.json").exists() else {}
    if rj.get("course", {}).get("takeoff", {}).get("in_mission_time"):
        t1, t2, t3 = st.columns(3)
        t1.metric("Take-off time t_takeoff", f"{rj['t_takeoff']:.2f} s")
        t2.metric("Ground roll s_ground_roll", f"{rj['s_ground_roll']:.1f} m ({rj['s_ground_roll'] / 0.3048:.0f} ft)")
        t3.metric("Lift-off speed V_LOF", f"{rj['V_lof']:.2f} m/s")
    st.markdown(summary)
    figdir = out / "figures"
    if figdir.exists():
        st.subheader("Figures")
        pngs = sorted(figdir.glob("*.png"))
        cols = st.columns(2)
        for i, png in enumerate(pngs):
            with cols[i % 2]:
                st.image(str(png), caption=png.stem.replace("_", " "))
                pdf = png.with_suffix(".pdf")
                if pdf.exists():
                    st.download_button(
                        f"{pdf.name} (vector)", pdf.read_bytes(), file_name=pdf.name, key=f"dl_{pdf.name}"
                    )
    last = st.session_state.get("last_design")
    if last:
        with st.expander("Re-analyse this design with a real motor (no optimiser)"):
            mr = last["motor"]
            st.caption(
                f"Design Kv {mr['Kv']:.1f} rpm/V (Rm {mr['Rm']:.4g} ohm, I0 {mr['I0']:.3g} A), set by: "
                f"{mr['set_by']}. Enter a catalogue motor: the design is re-analysed with it unchanged."
            )
            r_kv = st.number_input("Real motor Kv (rpm/V)", 100.0, 5000.0, float(round(mr["Kv"])))
            r_rm = st.number_input("Real motor Rm (ohm)", 0.001, 1.0, float(mr["Rm"]), format="%.4f")
            r_i0 = st.number_input("Real motor I0 (A)", 0.0, 20.0, float(mr["I0"]))
            r_m = st.number_input("Real motor mass (kg)", 0.01, 2.0, float(last["m_motor"]))
            if st.button("Re-analyse with this motor"):
                rr = reanalyse_with_motor(last["cfg"], last["dv"], r_kv, r_rm, r_i0, r_m, reference=last["reference"])
                st.dataframe(
                    pd.DataFrame(
                        [
                            {"quantity": k, "real motor": rr["values"][k], "change": rr["change"].get(k)}
                            for k in last["reference"]
                        ]
                    ),
                    hide_index=True,
                )
    if (out / "results.json").exists():
        st.download_button("results.json", (out / "results.json").read_bytes(), file_name="results.json")
    st.caption(f"All outputs (incl. n2.html, xdsm.tex, cases.sql): {out}")

    # OpenVSP mock-up (optional dependency: hidden without OpenVSP)
    from aircrafter2.vsp import have_openvsp

    if have_openvsp() and (out / "geometry.json").exists():
        st.subheader("OpenVSP mock-up")
        st.caption(
            "Visual / consistency mock-up only. Placement, blades, spinner and nacelle shape are illustrative; "
            "everything else comes from this result (docs/modules/vsp.md)."
        )
        if st.button("Build OpenVSP model"):
            from aircrafter2.vsp.pipeline import build_run

            with st.spinner("Building the OpenVSP model ..."):
                st.session_state["vsp"] = {
                    k: str(v) for k, v in build_run(out)["files"].items() if k in ("vsp3", "step", "isometric", "check")
                }
        vf = st.session_state.get("vsp")
        if vf and Path(vf["vsp3"]).parent.parent == out:
            st.image(str(Path(vf["isometric"]).with_suffix(".png")), caption="isometric")
            for key, label in (("vsp3", ".vsp3 (OpenVSP)"), ("step", ".stp (CAD)")):
                pth = Path(vf[key])
                st.download_button(label, pth.read_bytes(), file_name=pth.name, key=f"dl_{key}")
            st.markdown(Path(vf["check"]).read_text())
