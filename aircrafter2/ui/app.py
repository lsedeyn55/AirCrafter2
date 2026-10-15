"""AirCrafter 2.0 Streamlit UI: a thin layer over aircrafter2.api.run_design.

Start with:  aircrafter2 ui      (or: streamlit run aircrafter2/ui/app.py)
"""

import time
from pathlib import Path

import pandas as pd
import streamlit as st

from aircrafter2.api import run_design
from aircrafter2.problems.full_mdo import WOOMBAH_Y_M, PropulsionInputs, load_full_config
from aircrafter2.problems.wing_aero import Bounds
from aircrafter2.propulsion.apc import DEFAULT_DIR
from aircrafter2.scoring import available

ROOT = Path(__file__).resolve().parents[2]
CONFIGS = ROOT / "configs"
OUTPUTS = ROOT / "outputs"

st.set_page_config(page_title="AirCrafter 2.0", layout="wide")
st.title("AirCrafter 2.0")
st.caption("DBF conceptual-design MDAO: OpenAeroStruct + NeuralFoil aero, Garcia & Gamboa mass, APC/motor/battery "
           "propulsion, AirCrafter lap model. Values marked PLACEHOLDER in the configs are not USYD data.")

# --- sidebar: choose a config and the main switches -------------------------------------------------
full_configs = sorted(p.name for p in CONFIGS.glob("*.yaml") if "propulsion:" in p.read_text())
with st.sidebar:
    st.header("Setup")
    cfg_name = st.selectbox("Config", full_configs, index=full_configs.index("step7_full.yaml")
                            if "step7_full.yaml" in full_configs else 0)
    cfg = load_full_config(CONFIGS / cfg_name)
    plugins = available()
    cfg.scoring.plugin = st.selectbox("Scoring plug-in", plugins, index=plugins.index(cfg.scoring.plugin))
    props = sorted(p.stem.replace("PER3_", "") for p in DEFAULT_DIR.glob("PER3_*E.dat"))
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
        pr["m_nacelle"] = st.number_input("Nacelle mass per unit (kg) - PLACEHOLDER", 0.0, 1.0, float(pr["m_nacelle"]))
        pr["l_nac"] = st.number_input("Nacelle length (m) - PLACEHOLDER", 0.02, 0.5, float(pr["l_nac"]))
        pr["d_nac"] = st.number_input("Nacelle diameter (m) - PLACEHOLDER", 0.01, 0.2, float(pr["d_nac"]))
        pr["Q_nac"] = st.number_input("Nacelle interference factor Q (Woombah 1.3)", 1.0, 2.0, float(pr["Q_nac"]))
        cfg.mass.options.motor_inertia_relief = st.checkbox("Motor inertia relief on the spar",
                                                            value=cfg.mass.options.motor_inertia_relief)
    try:
        cfg.propulsion = PropulsionInputs(**pr)
    except ValueError as exc:
        st.error(str(exc))
        st.stop()
    cfg.energy_reserve = st.slider("Energy reserve (fraction of usable)", 0.0, 0.5, float(cfg.energy_reserve), 0.05)
    cfg.aero.num_y = st.select_slider("Spanwise VLM nodes (half wing)", [11, 21, 41], value=cfg.aero.num_y)
    cfg.fuselage_drag = st.checkbox("Fuselage drag", value=cfg.fuselage_drag)
    mode = st.radio("Run", ["Analyse (no optimisation)", "Optimise"], index=0)
    figs = st.checkbox("Make figures", value=True)

# --- main: editable inputs --------------------------------------------------------------------------
c1, c2 = st.columns(2)
with c1:
    st.subheader("Design-variable bounds")
    dv = pd.DataFrame([{"variable": k, "lower": b.lower, "upper": b.upper, "initial": cfg.initial.get(k, None)}
                       for k, b in cfg.design_variables.items()])
    dv = st.data_editor(dv, hide_index=True, disabled=["variable"], key="dv")
    for _, row in dv.iterrows():
        cfg.design_variables[row["variable"]] = Bounds(lower=float(row["lower"]), upper=float(row["upper"]))
        if pd.notna(row["initial"]):
            cfg.initial[row["variable"]] = float(row["initial"])
with c2:
    st.subheader("Key assumptions")
    cfg.mass.inputs["m_pay"] = st.number_input("Payload mass (kg)", 0.0, 20.0, float(cfg.mass.inputs.get("m_pay", 1.5)))
    cfg.mass.inputs["n"] = st.number_input("Load factor", 1.0, 12.0, float(cfg.mass.inputs.get("n", 5.0)))
    cfg.propulsion.n_cells = st.number_input("Battery cells (series)", 1.0, 12.0, float(cfg.propulsion.n_cells), 1.0)
    cfg.propulsion.Kv = st.number_input("Motor Kv (rpm/V) - PLACEHOLDER", 100.0, 5000.0, float(cfg.propulsion.Kv))
    cfg.propulsion.I_max = st.number_input("Current limit (A) - PLACEHOLDER", 5.0, 200.0, float(cfg.propulsion.I_max))
    cfg.mission_time = st.number_input("Mission time (s)", 30.0, 1200.0, float(cfg.mission_time))

# --- run ----------------------------------------------------------------------------------------------
if st.button("Run", type="primary"):
    out = OUTPUTS / f"ui_{time.strftime('%Y%m%d_%H%M%S')}"
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

if "last" in st.session_state:
    out = Path(st.session_state["last"])
    st.subheader(f"Results: {out.name}")
    summary = (out / "summary.md").read_text() if (out / "summary.md").exists() else ""
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
    if (out / "results.json").exists():
        st.download_button("results.json", (out / "results.json").read_bytes(), file_name="results.json")
    st.caption(f"All outputs (incl. n2.html, xdsm.tex, cases.sql): {out}")
