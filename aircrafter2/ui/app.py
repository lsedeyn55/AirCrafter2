"""AirCrafter 2.0 Streamlit UI: a thin layer over aircrafter2.api.run_design.

Start with:  aircrafter2 ui      (or: streamlit run aircrafter2/ui/app.py)
"""

import time
from pathlib import Path

import pandas as pd
import streamlit as st

from aircrafter2.api import reanalyse_with_motor, run_design, validate_kv_dv
from aircrafter2.config.loader import load_yaml
from aircrafter2.problems.full_mdo import WOOMBAH_Y_M, PropulsionInputs, check_kv_robust, load_full_config
from aircrafter2.problems.wing_aero import Bounds
from aircrafter2.propulsion.apc import DEFAULT_DIR
from aircrafter2.robustness.uncertainty import for_kv_mode
from aircrafter2.scoring import available

ROOT = Path(__file__).resolve().parents[2]
CONFIGS = ROOT / "configs"
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
    cfg = load_full_config(CONFIGS / cfg_name)
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
    cfg.energy_reserve = st.slider("Energy reserve (fraction of usable)", 0.0, 0.5, float(cfg.energy_reserve), 0.05)
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

# --- main: editable inputs --------------------------------------------------------------------------
c1, c2 = st.columns(2)
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
    cfg.mass.inputs["m_pay"] = st.number_input("Payload mass (kg)", 0.0, 20.0, float(cfg.mass.inputs.get("m_pay", 1.5)))
    cfg.mass.inputs["n"] = st.number_input("Load factor", 1.0, 12.0, float(cfg.mass.inputs.get("n", 5.0)))
    cfg.propulsion.n_cells = st.number_input("Battery cells (series)", 1.0, 12.0, float(cfg.propulsion.n_cells), 1.0)
    cfg.propulsion.I_max = st.number_input("Current limit (A) - PLACEHOLDER", 5.0, 200.0, float(cfg.propulsion.I_max))
    cfg.mission_time = st.number_input("Mission time (s)", 30.0, 1200.0, float(cfg.mission_time))

# --- run ----------------------------------------------------------------------------------------------
try:
    if cfg.robust.enabled:
        check_kv_robust(cfg)
except ValueError as exc:
    kv_errors.append(str(exc))
    st.error(str(exc))
if st.button("Run", type="primary", disabled=bool(kv_errors)):
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
