"""AirCrafter 2.0 Streamlit UI: a thin layer over aircrafter2.api.run_design.

Start with:  aircrafter2 ui      (or: streamlit run aircrafter2/ui/app.py)
"""

import json
import os
import time
import warnings
from pathlib import Path

import pandas as pd
import streamlit as st

from aircrafter2.api import reanalyse_with_motor, run_design, validate_kv_dv
from aircrafter2.config.layer_files import (frame_to_rows, frame_to_spreads, layer_files_of, overridden_by,
                                            read_rows, rows_to_frame, rows_to_layer, save_rows, spread_overridden_by,
                                            spreads_to_frame)
from aircrafter2.config.loader import load_yaml
from aircrafter2.mission.course import (SHIPPED_PRESETS, CourseConfig, TakeoffConfig, check_course, legs_from_rows,
                                        list_presets, load_preset, load_preset_data, rows_from_legs, save_preset)
from aircrafter2.problems.full_mdo import WOOMBAH_Y_M, PropulsionInputs, check_kv_robust, load_full_config
from aircrafter2.problems.wing_aero import Bounds
from aircrafter2.propulsion.apc import DEFAULT_DIR
from aircrafter2.robustness.uncertainty import for_kv_mode
from aircrafter2.scoring import available

ROOT = Path(__file__).resolve().parents[2]
CONFIGS = Path(os.environ.get("AIRCRAFTER2_CONFIGS", ROOT / "configs"))  # configurable for tests (a temporary copy)
OUTPUTS = ROOT / "outputs"

st.set_page_config(page_title="AirCrafter 2.0", layout="wide")
st.title("AirCrafter 2.0")
st.caption("DBF conceptual-design MDAO: OpenAeroStruct + NeuralFoil aero, Garcia & Gamboa mass, APC/motor/battery "
           "propulsion, AirCrafter lap model. Values marked PLACEHOLDER in the configs are not USYD data.")

# --- sidebar: choose a config and the main switches -------------------------------------------------
full_configs = sorted(p.name for p in CONFIGS.glob("*.yaml") if "propulsion" in load_yaml(p))  # incl. extends:
with st.sidebar:
    st.header("Setup")
    cfg_name = st.selectbox("Config", full_configs, index=full_configs.index("full_mdo.yaml")
                            if "full_mdo.yaml" in full_configs else 0)
cfg_path = CONFIGS / cfg_name
is_multipoint = "multipoint" in load_yaml(cfg_path)
ph_path, cfg_ruleset = layer_files_of(cfg_path)


# --- the layer tabs: placeholders.yaml and the ruleset, edited (and saved in place) before the config is loaded ------
def _rel(path):
    path = Path(path).resolve()
    return str(path.relative_to(CONFIGS.resolve())) if path.is_relative_to(CONFIGS.resolve()) else path.name


def _same(a, b, keys):
    return all(str(a.get(k)) == str(b.get(k)) for k in keys)


def layer_editor(path, label, overridden, spread_over):
    """Editors for one layer file (values, then RDO spreads); Save / Discard. Returns the layer data with the
    current (possibly unsaved) edits, which the run uses."""
    name = _rel(path)
    key = f"layer_{cfg_name}_{name}"
    rows, spreads = read_rows(path)
    if f"{key}_v" not in st.session_state:
        st.session_state[f"{key}_v"] = 0
    if f"{key}_df" not in st.session_state:
        st.session_state[f"{key}_df"] = rows_to_frame(rows, overridden)
    if f"{key}_sp" not in st.session_state:
        st.session_state[f"{key}_sp"] = spreads_to_frame(spreads, spread_over)
    v = st.session_state[f"{key}_v"]
    st.caption(f"`configs/{name}`: one row per value. Edit value, status and source; **overridden by** names the file "
               f"and line of `{cfg_name}` (or its extends: chain) that sets the value instead, so an edit there will "
               "not reach this config's runs.")
    edited = st.data_editor(
        st.session_state[f"{key}_df"], key=f"{key}_ed{v}", hide_index=True, width="stretch",
        disabled=["area", "parameter", "units", "Q", "overridden by", "line"],
        column_config={"status": st.column_config.SelectboxColumn("status", options=["PLACEHOLDER", "cited"]),
                       "line": None})
    new_rows = frame_to_rows(edited)
    new_sp = spreads
    if spreads:
        st.markdown("**RDO spreads** (shared uncertain parameters; merged by name with a config's uncertainty file)")
        edited_sp = st.data_editor(
            st.session_state[f"{key}_sp"], key=f"{key}_sped{v}", hide_index=True, width="stretch",
            disabled=["name", "units", "kv_mode", "source", "Q", "overridden by", "line"],
            column_config={"dist": st.column_config.SelectboxColumn("dist", options=["uniform", "normal",
                                                                                     "triangular"]),
                           "status": st.column_config.SelectboxColumn("status", options=["PLACEHOLDER", "cited"]),
                           "relative": st.column_config.CheckboxColumn("relative"), "line": None})
        new_sp = frame_to_spreads(edited_sp, spreads)
    unsaved = any(not _same(a, b, ("value", "status", "Q", "source")) for a, b in zip(rows, new_rows)) or any(
        not _same(a, b, ("dist", "nominal", "spread", "relative", "status", "note")) for a, b in zip(spreads, new_sp))
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
    return rows_to_layer(new_rows, new_sp if spreads else None)


tab_names = (["Placeholders", "Ruleset"] if is_multipoint else ["Design and mission", "Placeholders", "Ruleset"])
tabs = dict(zip(tab_names, st.tabs(tab_names)))
if st.session_state.get("saved_msg"):
    st.success(st.session_state.pop("saved_msg"))
rulesets = sorted(_rel(p) for p in (CONFIGS / "rulesets").glob("*.yaml"))
with tabs["Ruleset"]:  # the selector first: the placeholders' 'overridden by' column reads the chosen ruleset
    rs_name = st.selectbox("Ruleset (configs/rulesets/)", rulesets, key=f"ruleset_{cfg_name}",
                           index=rulesets.index(_rel(cfg_ruleset)) if cfg_ruleset and _rel(cfg_ruleset) in rulesets
                           else 0, help="defaults to the ruleset the config names; the run uses the one chosen here")
rs_path = CONFIGS / rs_name
with tabs["Placeholders"]:
    ph_rows, ph_spreads = read_rows(ph_path)
    ph_layer = layer_editor(ph_path, "placeholders",
                            {r["parameter"]: overridden_by(cfg_path, r["parameter"], rs_path) for r in ph_rows},
                            {s["name"]: spread_overridden_by(cfg_path, s["name"]) for s in ph_spreads})
with tabs["Ruleset"]:
    rs_rows, rs_spreads = read_rows(rs_path)
    rs_layer = layer_editor(rs_path, "ruleset", {r["parameter"]: overridden_by(cfg_path, r["parameter"])
                                                 for r in rs_rows},
                            {s["name"]: spread_overridden_by(cfg_path, s["name"]) for s in rs_spreads})
if is_multipoint:
    st.info("This is a multipoint (2024-25) config: the UI runs single-point problems only. Edit and save its "
            "placeholders and ruleset here; the 2024-25 runs are `studies/results_2025.py` (and "
            "`studies/takeoff_study.py`), which read the same files.")
    st.stop()
cfg = load_full_config(cfg_path, placeholders=ph_layer, ruleset=rs_layer)
with st.sidebar:
    from aircrafter2.scoring.base import _REGISTRY

    plugins = [n for n in available() if not getattr(_REGISTRY.get(n), "multipoint", False)]  # single-point only
    cfg.scoring.plugin = st.selectbox("Scoring plug-in", plugins, index=plugins.index(cfg.scoring.plugin))
    apc_dir = Path(cfg.propulsion.apc_dir).expanduser() if cfg.propulsion.apc_dir else DEFAULT_DIR
    props = sorted(p.stem.replace("PER3_", "") for p in apc_dir.glob("PER3_*E.dat"))
    cfg.propulsion.prop = st.selectbox("APC propeller", props, index=props.index(cfg.propulsion.prop)
                                       if cfg.propulsion.prop in props else 0)
    st.subheader("Propulsion layout")
    pr = cfg.propulsion.model_dump()
    pr["n_motors"] = st.radio("Layout", [1, 2], index=pr["n_motors"] - 1, horizontal=True,
                              format_func=lambda n: "1: nose tractor" if n == 1 else "2: wing-mounted pair")
    pr["battery_topology"] = st.radio("Battery topology", ["per_motor", "shared"],
                                      index=["per_motor", "shared"].index(pr["battery_topology"]), horizontal=True,
                                      help="per_motor: one pack per motor; shared: one pack feeds every ESC")
    st.caption("Battery capacity (design variable `capacity`) is the capacity **per pack**.")
    if pr["n_motors"] == 2:
        if pr["y_motor"] is not None:
            pr["y_motor"] = st.number_input("Motor station y_motor (fraction of semi-span)", 0.05, 0.95,
                                            float(pr["y_motor"]))
        else:
            pr["y_motor_m"] = st.number_input("Motor station y_m (m from centreline; Woombah 0.2515)", 0.05, 2.0,
                                              float(pr["y_motor_m"] or WOOMBAH_Y_M))
        st.caption("Nacelle mass, length and diameter: Placeholders tab.")
        pr["Q_nac"] = st.number_input("Nacelle interference factor Q (Woombah 1.3)", 1.0, 2.0, float(pr["Q_nac"]))
        cfg.mass.options.motor_inertia_relief = st.checkbox("Motor inertia relief on the spar",
                                                            value=cfg.mass.options.motor_inertia_relief)
    try:
        cfg.propulsion = PropulsionInputs(**pr)
    except ValueError as exc:
        st.error(str(exc))
        st.stop()
    st.subheader("Motor")
    kv_errors = []
    kv_on = st.checkbox("Optimise motor Kv", value="Kv" in cfg.design_variables,
                        help="Kv as a design variable: the reference motor below is rewound to the design Kv (same "
                        "frame; Rm and I0 scale with Kv) and the current limit becomes a constraint. Off: a fixed motor")
    if kv_on:
        cfg.propulsion.Kv = st.number_input("Reference motor Kv (rpm/V) - PLACEHOLDER", 100.0, 5000.0,
                                            float(cfg.propulsion.Kv))
        st.caption(f"Reference motor: Rm = {cfg.propulsion.Rm:g} ohm, I0 = {cfg.propulsion.I0:g} A (PLACEHOLDERS, "
                   "Q7). At Kv: Rm x (Kv_ref/Kv)^2, I0 x Kv/Kv_ref.")
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
    robust_on = st.checkbox("Robust design optimisation", value=cfg.robust.enabled,
                            help="minimise mu + k sigma of the score with mu -/+ beta sigma constraints over fixed "
                            "uncertainty nodes (docs/modules/robustness.md). Spreads: configs/uncertainty.yaml "
                            "(PLACEHOLDERS)")
    if robust_on:
        rdo_cfg = "full_mdo_rdo.yaml" if kv_on else "full_mdo_rdo_fixed_kv.yaml"
        names = [p.name for p in for_kv_mode(cfg.robust.parameters, kv_on)]
        if not cfg.robust.parameters or any(n not in names for n in (cfg.robust.active or [])):
            # no robust block (e.g. full_mdo.yaml), or one for the other motor mode: take the matching RDO config's
            cfg.robust = load_full_config(CONFIGS / rdo_cfg).robust
            st.caption(f"Robust settings from {rdo_cfg} ({'Kv a design variable' if kv_on else 'fixed motor'}).")
        rob = cfg.robust
        names = [p.name for p in for_kv_mode(rob.parameters, kv_on)]
        rob.active = st.multiselect("Propagated parameters (others at their means)", names,
                                    default=rob.active if rob.active is not None else names)
        rob.quadrature = st.radio("Quadrature", ["udr", "smolyak"], index=0 if rob.quadrature != "smolyak" else 1,
                                  horizontal=True, help="udr: 2d + 1 nodes (no cross terms); smolyak level 2: "
                                  "cross terms, many more nodes")
        rob.k = st.slider("k in mu + k sigma", 0.0, 3.0, float(rob.k), 0.25)
        rob.beta = st.slider("beta (constraint margin, ~97.7 % at 2 if normal)", 0.0, 3.0, float(rob.beta), 0.25)
        try:
            import cyipopt  # noqa: F401
            opts = ["slsqp", "ipopt"]
        except ImportError:
            opts = ["slsqp"]
        rob.optimizer = st.radio("Optimiser", opts, index=opts.index(rob.optimizer) if rob.optimizer in opts else 0,
                                 horizontal=True)
        d = len(rob.active)
        n_nodes = 2 * d + 1 if rob.quadrature == "udr" else None
        st.caption(f"{d} parameters: " + (f"{n_nodes} analysis nodes, about {n_nodes} x the deterministic cost."
                                          if n_nodes else "Smolyak level 2: many nodes (127 for d = 7); slow."))
    cfg.robust.enabled = robust_on
    mode = st.radio("Run", ["Analyse (no optimisation)", "Optimise"], index=0)
    figs = st.checkbox("Make figures", value=True)

# --- main: editable inputs (Design and mission tab) ---------------------------------------------------------------
with tabs["Design and mission"]:
    c1, c2, c3 = st.columns(3)
    with c1:
        st.subheader("Design-variable bounds")
        st.caption("Motor Kv, if optimised, is set in the sidebar (Motor).")
        dv = pd.DataFrame([{"variable": k, "lower": b.lower, "upper": b.upper, "initial": cfg.initial.get(k, None)}
                           for k, b in cfg.design_variables.items() if k != "Kv"])
        dv = st.data_editor(dv, hide_index=True, disabled=["variable"], key="dv")
        for _, row in dv.iterrows():
            cfg.design_variables[row["variable"]] = Bounds(lower=float(row["lower"]), upper=float(row["upper"]))
            if pd.notna(row["initial"]):
                cfg.initial[row["variable"]] = float(row["initial"])
    with c2:
        st.subheader("Key assumptions")
        cfg.mass.inputs["n"] = st.number_input("Load factor", 1.0, 12.0, float(cfg.mass.inputs.get("n", 5.0)))
        cfg.propulsion.n_cells = st.number_input("Battery cells (series)", 1.0, 12.0, float(cfg.propulsion.n_cells),
                                                 1.0)
        st.caption("Payload mass, current limit, energy reserve and the other placeholders: Placeholders tab.")
    with c3:
        st.subheader("Mission")
        to0 = cfg.course.takeoff
        to_on = st.checkbox("Include take-off in mission time", value=to0.in_mission_time,
                            help="add the ground roll (rest to lift-off, into the wind, at the throttle) to lap 1's "
                            "time and energy: the rules time every mission from the first throttle advance. Off: the "
                            "original model, lap 1 starts in the air at V_stall (every published result)")
        k_lof, cl_ground, start_off = to0.k_lof, to0.CL_ground, to0.start_offset_ft
        st.caption(f"Take-off parameters: k_lof {k_lof:g}, CL_ground {cl_ground:g} and the take-off distance "
                   f"{cfg.takeoff_distance:g} m (Eq. 3.3) are in the Placeholders tab; the start offset "
                   f"({start_off:g} ft) and the mission time ({cfg.mission_time:g} s) in the Ruleset tab.")
        cfg.mu_roll = st.number_input("Rolling friction mu_roll", 0.0, 0.5, float(cfg.mu_roll), 0.01,
                                      help="used by the Eq. 3.3 constraint and, with take-off on, the ground roll")
        reaction = st.number_input("Pilot reaction time per lap (s)", 0.0, 30.0,
                                   float(cfg.course.pilot_reaction_time_s),
                                   0.5)

    # --- mission: course layout ---------------------------------------------------------------------------
    LEG_COLS = ["order", "kind", "length_ft", "angle_deg", "wind"]
    LEG_COLUMN_CONFIG = {
        "order": st.column_config.NumberColumn("order", help="legs are flown in this order", step=1),
        "kind": st.column_config.SelectboxColumn("kind", options=["straight", "turn"], required=True),
        "length_ft": st.column_config.NumberColumn("length (ft)", help="straights", min_value=0.0),
        "angle_deg": st.column_config.NumberColumn("angle (deg)", help="turns", min_value=0.0),
        "wind": st.column_config.SelectboxColumn("wind", options=["head", "tail", "none"],
                                                 help="straights: along-track wind (none = crosswind)")}


    def leg_table(label, legs0, key):
        """Editable leg table (rows can be added and deleted); kept in session state so 'Add leg' can append a row."""
        dfk, vk = f"{key}_df", f"{key}_v"
        if dfk not in st.session_state:
            st.session_state[dfk] = pd.DataFrame(rows_from_legs(legs0), columns=LEG_COLS)
            st.session_state[vk] = 0
        edited = st.data_editor(st.session_state[dfk], key=f"{key}_ed{st.session_state[vk]}", num_rows="dynamic",
                                hide_index=True, column_config=LEG_COLUMN_CONFIG, width="stretch")
        b1, b2 = st.columns(2)
        if b1.button(f"Add leg ({label})", key=f"{key}_add"):
            last = pd.to_numeric(edited["order"], errors="coerce").max()
            new = dict(order=(0 if pd.isna(last) else int(last)) + 1, kind="straight", length_ft=100.0, angle_deg=None,
                       wind="none")
            st.session_state[dfk] = pd.concat([edited, pd.DataFrame([new], columns=LEG_COLS)], ignore_index=True)
            st.session_state[vk] += 1
            st.rerun()
        if b2.button(f"Sort by order ({label})", key=f"{key}_sort"):
            st.session_state[dfk] = edited.sort_values("order", kind="stable",
                                                       na_position="last").reset_index(drop=True)
            st.session_state[vk] += 1
            st.rerun()
        return edited


    st.subheader("Mission: course")
    presets = list_presets()
    cfg_preset = cfg.course.preset if cfg.course.legs is None and cfg.course.preset in presets else None
    cc1, cc2 = st.columns([3, 2])
    with cc1:
        preset = st.selectbox("Course preset (configs/courses/)", presets,
                              index=presets.index(cfg_preset or ("dbf_2025" if "dbf_2025" in presets else presets[0])),
                              key=f"preset_{cfg_name}")
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
            st.caption("Lap 1 (with take-off on, it must start with a head-wind straight: the ground roll is into "
                       "the wind)")
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
                        preset=preset if same else None, legs=None if same else other,
                        lap1_legs=None if same or not diff else lap1, pilot_reaction_time_s=reaction,
                        takeoff=TakeoffConfig(in_mission_time=to_on, k_lof=k_lof, CL_ground=cl_ground,
                                              start_offset_ft=start_off))
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
            st.caption("Top view, straights to scale; turn radii are schematic (the lap model's turns follow the "
                       "flight "
                       "state). Wind arrow: the wind blows against the head-wind straights.")
        with st.expander("Save this course as a preset"):
            pname = st.text_input("Preset name (letters, digits, _ and -)", key="preset_name")
            if st.button("Save preset", disabled=bool(course_errors) or not pname):
                if pname in SHIPPED_PRESETS:
                    st.error(f"'{pname}' is a shipped preset and cannot be overwritten: choose another name.")
                else:
                    try:
                        path = save_preset(pname, other, lap1 if diff else None,
                                           citation="user-defined course (AirCrafter 2.0 UI)")
                        st.success(f"Saved {path.relative_to(ROOT) if path.is_relative_to(ROOT) else path}.")
                    except ValueError as exc:
                        st.error(str(exc))


# --- run (below the tabs) -------------------------------------------------------------------------------------------
try:
    if cfg.robust.enabled:
        check_kv_robust(cfg)
except ValueError as exc:
    kv_errors.append(str(exc))
    st.error(str(exc))
if st.button("Run", type="primary", disabled=bool(kv_errors or course_errors)):
    out = OUTPUTS / "ui_runs" / time.strftime("%Y%m%d_%H%M%S")  # named studies stay at the top of outputs/
    with st.status("Running AirCrafter 2.0 ...", expanded=True) as status:
        t0 = time.perf_counter()
        try:
            res = run_design(cfg, optimise=mode == "Optimise", out_dir=out, figures=figs,
                             sensitivities=mode == "Optimise", progress=lambda s: st.write(f"{s} ..."))
        except Exception as exc:  # show the failure to the user instead of a stack trace
            status.update(label=f"Failed: {exc}", state="error")
            st.stop()
        status.update(label=f"Done in {time.perf_counter() - t0:.0f} s (success = {res.success}, "
                      f"iterations = {res.iterations})", state="complete")
    st.session_state["last"] = out
    p = res.prob
    st.session_state["last_design"] = dict(
        cfg=cfg.model_copy(deep=True), dv={k: float(p.get_val(k)[0]) for k in cfg.design_variables},
        motor=res.values["motor"], m_motor=float(cfg.propulsion.m_motor),
        reference={k: float(p.get_val(k)[0]) for k in ["score", "ThreeLapTime", "m_total", "I_margin", "rpm_margin",
                                                       "energy_margin", "thrust_margin_static", "SM"]})

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
                    st.download_button(f"{pdf.name} (vector)", pdf.read_bytes(), file_name=pdf.name,
                                       key=f"dl_{pdf.name}")
    last = st.session_state.get("last_design")
    if last:
        with st.expander("Re-analyse this design with a real motor (no optimiser)"):
            mr = last["motor"]
            st.caption(f"Design Kv {mr['Kv']:.1f} rpm/V (Rm {mr['Rm']:.4g} ohm, I0 {mr['I0']:.3g} A), set by: "
                       f"{mr['set_by']}. Enter a catalogue motor: the design is re-analysed with it unchanged.")
            r_kv = st.number_input("Real motor Kv (rpm/V)", 100.0, 5000.0, float(round(mr["Kv"])))
            r_rm = st.number_input("Real motor Rm (ohm)", 0.001, 1.0, float(mr["Rm"]), format="%.4f")
            r_i0 = st.number_input("Real motor I0 (A)", 0.0, 20.0, float(mr["I0"]))
            r_m = st.number_input("Real motor mass (kg)", 0.01, 2.0, float(last["m_motor"]))
            if st.button("Re-analyse with this motor"):
                rr = reanalyse_with_motor(last["cfg"], last["dv"], r_kv, r_rm, r_i0, r_m, reference=last["reference"])
                st.dataframe(pd.DataFrame([{"quantity": k, "real motor": rr["values"][k],
                                            "change": rr["change"].get(k)} for k in last["reference"]]),
                             hide_index=True)
    if (out / "results.json").exists():
        st.download_button("results.json", (out / "results.json").read_bytes(), file_name="results.json")
    st.caption(f"All outputs (incl. n2.html, xdsm.tex, cases.sql): {out}")

    # OpenVSP mock-up (optional dependency: hidden without OpenVSP)
    from aircrafter2.vsp import have_openvsp

    if have_openvsp() and (out / "geometry.json").exists():
        st.subheader("OpenVSP mock-up")
        st.caption("Visual / consistency mock-up only. Placement, blades, spinner and nacelle shape are illustrative; "
                   "everything else comes from this result (docs/modules/vsp.md).")
        if st.button("Build OpenVSP model"):
            from aircrafter2.vsp.pipeline import build_run

            with st.spinner("Building the OpenVSP model ..."):
                st.session_state["vsp"] = {k: str(v) for k, v in build_run(out)["files"].items()
                                           if k in ("vsp3", "step", "isometric", "check")}
        vf = st.session_state.get("vsp")
        if vf and Path(vf["vsp3"]).parent.parent == out:
            st.image(str(Path(vf["isometric"]).with_suffix(".png")), caption="isometric")
            for key, label in (("vsp3", ".vsp3 (OpenVSP)"), ("step", ".stp (CAD)")):
                pth = Path(vf[key])
                st.download_button(label, pth.read_bytes(), file_name=pth.name, key=f"dl_{key}")
            st.markdown(Path(vf["check"]).read_text())
