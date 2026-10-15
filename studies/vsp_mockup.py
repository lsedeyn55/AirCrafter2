"""OpenVSP mock-ups of the single-vs-twin study designs (no re-optimisation) and the AIAC paper figure.

For each case in docs/studies/twin_motor_study_results.json:
  1. set the stored optimum design variables (tc_scale is not stored: it is recovered from the stored tc_max with
     airfoil.shape.recover_tc_scale, an exact 1-D inversion of the airfoil shape family) and run the coupled
     analysis with the case's config, prop, cells and options;
  2. check that ThreeLapTime and m_total match the stored values to 1e-4 relative (else stop and say so);
  3. write outputs/vsp/<case>/geometry.json, build the OpenVSP model, render the three-view, the isometric and
     the planform-overlay check (the 3D model's top view under plotting.figures.planform).
Also re-exports and builds outputs/twin_analyse/ (the plain twin-config analysis), and writes
outputs/vsp/paper_isometrics.pdf (three isometrics, same scale and camera).

Run: .venv/bin/python studies/vsp_mockup.py
"""

import json
import sys
import time
import warnings
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
STUDY = ROOT / "docs" / "studies" / "twin_motor_study_results.json"
OUT = ROOT / "outputs" / "vsp"
CFG = {1: "configs/step7_full.yaml", 2: "configs/step7_full_twin.yaml"}
CASES = ["n1_10x7E_4S", "n2_10x7E_4S", "n2_8x6E_4S_matched"]
DVS = ["b", "c", "camber_scale", "capacity", "l_ht", "S_ht", "S_vt", "x_batt", "x_cb_frac"]
TOL = 1e-4


def rebuild_case(r):
    from aircrafter2.airfoil import load_seed
    from aircrafter2.airfoil.shape import recover_tc_scale
    from aircrafter2.problems.full_mdo import build_full_problem, load_full_config

    cfg = load_full_config(ROOT / CFG[r["n"]])
    cfg.propulsion.prop = r["prop"]
    cfg.propulsion.n_cells = float(r["cells"])
    cfg.mass.options.motor_inertia_relief = bool(r.get("relief", False))
    if r.get("nacelle_ff"):
        cfg.propulsion.nacelle_ff = r["nacelle_ff"]
    prob = build_full_problem(cfg, driver=False)
    for k in DVS:
        prob.set_val(k, r[k])
    prob.set_val("tc_scale", recover_tc_scale(load_seed(cfg.airfoil.seed), r["tc_max"], r["camber_scale"]))
    prob.run_model()
    check = {}
    for k in ("ThreeLapTime", "m_total"):
        new = float(prob.get_val(k)[0])
        check[k] = (r[k], new, abs(new - r[k]) / abs(r[k]))
    return cfg, prob, check


def make_case(case_id, cfg, prob, out, config_path, optimised, success, vsp_out=None):
    """geometry.json, OpenVSP build + vsp_check.md + three-view + isometric, and the planform overlay check."""
    from aircrafter2.geometry.export import export_geometry, write_geometry
    from aircrafter2.plotting import figures as F
    from aircrafter2.vsp import render as Rd
    from aircrafter2.vsp.pipeline import build_run

    geom = export_geometry(prob, cfg, config_path=config_path, optimised=optimised, success=success,
                           case_id=case_id)
    write_geometry(geom, out / "geometry.json")
    vsp_out = vsp_out or out
    res = build_run(out, out_dir=vsp_out, name=case_id)
    meshes = res["meshes"]
    F.planform(prob, vsp_out / f"{case_id}_planform_overlay.pdf",
               underlay=lambda ax: Rd.draw_projection(ax, meshes, "top", alpha=0.55, flat="#b0b0b0"))
    return geom, meshes, res


def main():
    warnings.filterwarnings("ignore")
    from aircrafter2.vsp import INSTALL_HINT, have_openvsp

    if not have_openvsp():
        print(INSTALL_HINT)
        return 2
    from aircrafter2.vsp import render as Rd

    R = json.loads(STUDY.read_text())
    done, log = [], []
    for cid in CASES:
        r = R[cid]
        cfg, prob, check = rebuild_case(r)
        line = ", ".join(f"{k} stored {a:.6f} rebuilt {b:.6f} (rel {e:.1e})" for k, (a, b, e) in check.items())
        print(cid, line, flush=True)
        log.append((cid, check))
        if any(e > TOL for _, _, e in check.values()):
            print(f"{cid}: does not reproduce the stored optimum to {TOL:g}; re-run its optimisation "
                  "(studies/twin_motor_study.py) before building", file=sys.stderr)
            return 1
        t = time.perf_counter()
        geom, meshes, res = make_case(cid, cfg, prob, OUT / cid, CFG[r["n"]], True, r["success"])
        print(f"  built in {res['build_s']:.2f} s (OpenVSP), {time.perf_counter() - t:.1f} s with figures; "
              f"readback span {res['readback']['wing_span_m']:.4f} m, props {res['readback']['n_props']}; "
              f"vsp_check: {len(res['check']['flagged'])} item(s) over 5 %", flush=True)
        done.append((cid, geom, meshes, res))

    # the plain twin-config analysis
    from aircrafter2.problems.full_mdo import build_full_problem, load_full_config

    cfg = load_full_config(ROOT / CFG[2])
    prob = build_full_problem(cfg, driver=False)
    prob.run_model()
    run = ROOT / "outputs" / "twin_analyse"
    make_case("twin_analyse", cfg, prob, run, CFG[2], False, True, vsp_out=run / "vsp")

    labels = []
    for cid, geom, _, _ in done:
        P, pv = geom["propulsion"], geom["provenance"]
        lay = "1 motor (nose)" if P["n_motors"] == 1 else ("2 motors, matched power" if "matched" in cid
                                                           else "2 motors (wing)")
        labels.append(f"{lay}: {P['prop']} {pv['n_cells']:.0f}S\n3-lap {pv['ThreeLapTime_s']:.2f} s, "
                      f"m = {geom['stations']['masses_kg']['m_total']:.2f} kg")
    Rd.side_by_side([(g, m) for _, g, m, _ in done], OUT / "paper_isometrics.pdf", labels)
    print("wrote", OUT / "paper_isometrics.pdf")
    return 0


if __name__ == "__main__":
    sys.exit(main())
