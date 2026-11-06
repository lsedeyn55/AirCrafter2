"""vsp_check.md: read the saved .vsp3 back and compare it with geometry.json.

Reloads the written file, so the check covers the file round trip, reads the key parameters and CompGeom wetted areas,
and compares them with the framework values. The framework is never changed to match: differences are explained in the
report, and anything over 5 % is listed for docs/PLACEHOLDERS.md. See docs/modules/vsp.md.
"""

from pathlib import Path

import numpy as np

from . import require_openvsp

FLAG = 0.05


def _geoms(vsp):
    """{geometry name: [ids]} of the loaded model."""
    out = {}
    for g in vsp.FindGeoms():
        out.setdefault(vsp.GetGeomName(g), []).append(g)
    return out


def _airfoil_tc(vsp, wing_id):
    """The wing section's t/c as OpenVSP has it."""
    xs = vsp.GetXSec(vsp.GetXSecSurf(wing_id, 0), 0)
    up = np.array([(p.x(), p.y()) for p in vsp.GetAirfoilUpperPnts(xs)])
    lo = np.array([(p.x(), p.y()) for p in vsp.GetAirfoilLowerPnts(xs)])
    x = 0.5 * (1 - np.cos(np.linspace(0, np.pi, 2001)))
    t = np.interp(x, up[:, 0], up[:, 1]) - np.interp(x, lo[:, 0], lo[:, 1])
    i = int(np.argmax(t))
    return float(t[i]), float(x[i]), float(vsp.GetParmVal(vsp.GetXSecParm(xs, "ThickChord")))


def _cst_shape_difference(vsp, geom):
    """Max |dy/c| between the file airfoil (AeroSandbox Kulfan incl. LE term) and OpenVSP's own CST type
    built from the same upper/lower weights (OpenVSP's CST has no Kulfan LE-modification term)."""
    k = geom["wing"]["airfoil"]["kulfan"]
    w = vsp.AddGeom("WING")
    surf = vsp.GetXSecSurf(w, 0)
    vsp.ChangeXSecShape(surf, 0, vsp.XS_CST_AIRFOIL)
    xs = vsp.GetXSec(surf, 0)
    vsp.SetUpperCST(xs, len(k["upper_weights"]) - 1, list(k["upper_weights"]))
    vsp.SetLowerCST(xs, len(k["lower_weights"]) - 1, list(k["lower_weights"]))
    vsp.Update()
    pts = np.array([(p.x(), p.y()) for p in vsp.GetAirfoilCoordinates(w, 0.0)])
    vsp.DeleteGeom(w)
    i = int(np.argmin(pts[:, 0]))  # split at the leading edge; order the halves by increasing x
    a, b = pts[: i + 1], pts[i:]
    a, b = a[np.argsort(a[:, 0])], b[np.argsort(b[:, 0])]
    up, lo = (a, b) if a[:, 1].mean() > b[:, 1].mean() else (b, a)
    xy = np.array(geom["wing"]["airfoil"]["coordinates_selig"])
    n = len(xy) // 2
    xu, yu = xy[: n + 1, 0][::-1], xy[: n + 1, 1][::-1]
    xl, yl = xy[n:, 0], xy[n:, 1]
    x = np.linspace(0.0, 1.0, 1001)
    du = np.interp(x, up[:, 0], up[:, 1]) - np.interp(x, xu, yu)
    dl = np.interp(x, lo[:, 0], lo[:, 1]) - np.interp(x, xl, yl)
    d = np.maximum(np.abs(du), np.abs(dl))
    return float(d.max()), float(x[np.argmax(d)])


def vsp_check(geom, vsp3_path, md_path=None):
    """Compare the saved model with the framework values.

    Parameters
    ----------
    geom : dict
    vsp3_path : path
    md_path : path, optional
        Write the report here.

    Returns
    -------
    dict
        rows (quantity, OpenVSP, framework, % difference, note), wetted, flagged (the list over 5 %), cst_max_dy. Raises
        RuntimeError without OpenVSP.
    """
    require_openvsp()
    import openvsp as vsp

    vsp.VSPRenew()
    vsp.ReadVSPFile(str(vsp3_path))
    G = _geoms(vsp)
    W, HT, VT, TB, CB, P = (
        geom[k] for k in ("wing", "horizontal_tail", "vertical_tail", "tail_boom", "cargo_bay", "propulsion")
    )

    def gp(gid, n, grp):
        """One OpenVSP parameter."""
        return vsp.GetParmVal(gid, n, grp)

    def bb(gid):
        """A geometry's bounding box."""
        return {k: gp(gid, k, "BBox") for k in ("X_Min", "Y_Min", "Z_Min", "X_Len", "Y_Len", "Z_Len")}

    wing, ht, vt, cb, tb = (G[n][0] for n in ("Wing", "HorizontalTail", "VerticalTail", "CargoBay", "TailBoom"))
    props = [g for n, gs in G.items() if n.startswith("Prop") for g in gs]
    nacs = [g for n, gs in G.items() if n.startswith("Nacelle") for g in gs]
    rows = []

    def row(q, a, e, note=""):
        """Append one comparison row."""
        rows.append((q, a, e, (a - e) / e * 100.0 if e else float("nan"), note))

    # --- parameters: wing, tails, cargo bay, boom, props ---
    chord_w = vsp.GetParmVal(vsp.GetXSecParm(vsp.GetXSec(vsp.GetXSecSurf(wing, 0), 1), "Root_Chord"))
    row("wing span (m)", gp(wing, "TotalSpan", "WingGeom"), W["span_m"])
    row("wing area (m²)", gp(wing, "TotalArea", "WingGeom"), W["area_m2"])
    row("wing chord (m)", chord_w, W["chord_m"])
    tc_pts, xtc, tc_parm = _airfoil_tc(vsp, wing)
    row("airfoil t/c (OpenVSP ThickChord)", tc_parm, W["airfoil"]["tc_max"])
    row("airfoil t/c (from read-back points)", tc_pts, W["airfoil"]["tc_max"], "2001-point max on the file airfoil")
    row("airfoil x(t/c max) / c", xtc, W["airfoil"]["x_tc_max"], "framework: parabolic peak on 401 points")
    row("HT area (m²)", gp(ht, "TotalArea", "WingGeom"), HT["area_m2"])
    row("VT area (m²)", gp(vt, "TotalArea", "WingGeom"), VT["area_m2"])
    ht_c = vsp.GetParmVal(vsp.GetXSecParm(vsp.GetXSec(vsp.GetXSecSurf(ht, 0), 1), "Root_Chord"))
    row(
        "tail arm l_ht (m)",
        gp(ht, "X_Rel_Location", "XForm") + 0.25 * ht_c - 0.25 * chord_w,
        HT["tail_arm_m"],
        "HT c/4 minus wing c/4, from read-back positions",
    )
    b = bb(cb)
    row("cargo bay length (m)", b["X_Len"], CB["length_m"])
    row("cargo bay width (m)", b["Y_Len"], CB["width_m"])
    row("cargo bay height (m)", b["Z_Len"], CB["height_m"])
    row("cargo bay centre x (m)", b["X_Min"] + b["X_Len"] / 2, CB["x_centre_m"])
    row("tail boom start x (m)", bb(tb)["X_Min"], TB["x_start_m"])
    row("tail boom length (m)", bb(tb)["X_Len"], TB["length_m"])
    for p in props:
        row(f"{vsp.GetGeomName(p)} diameter (m)", gp(p, "Diameter", "Design"), P["D_prop_m"])
        row(f"{vsp.GetGeomName(p)} x (m)", gp(p, "X_Rel_Location", "XForm"), P["x_motor_m"])
    if P["n_motors"] == 2:
        for p in props:
            row(f"{vsp.GetGeomName(p)} |y| (m)", abs(gp(p, "Y_Rel_Location", "XForm")), P["y_m_m"])
        # clearances measured on the model: disc edges against the bay side and the wing tip
        D = gp(props[0], "Diameter", "Design")
        y = min(abs(gp(p, "Y_Rel_Location", "XForm")) for p in props)
        half_w = b["Y_Len"] / 2.0
        row(
            "fuselage clearance margin (m)",
            y - D / 2 - half_w - P["c_clear_m"],
            P["fus_clear_margin_m"],
            "y_m − D/2 − (bay half-width from its bounding box) − c_clear",
        )
        row(
            "tip clearance margin (m)",
            gp(wing, "TotalSpan", "WingGeom") / 2 - (y + D / 2),
            P["tip_clear_margin_m"],
            "b/2 − (y_m + D/2)",
        )
        for n in nacs:
            row(f"{vsp.GetGeomName(n)} diameter (m)", bb(n)["Y_Len"], P["d_nac_m"])
            row(f"{vsp.GetGeomName(n)} length (m)", bb(n)["X_Len"], P["l_nac_m"])

    # CompGeom wetted areas of the export set (no markers) vs the framework's own formulas (aero/parasite.py)
    # --- wetted areas (CompGeom) ---
    mesh = vsp.ComputeCompGeom(vsp.SET_FIRST_USER, False, 0)
    rid = vsp.FindLatestResultsID("Comp_Geom")
    wet, theo = {}, {}
    for n, w_, t_ in zip(
        vsp.GetStringResults(rid, "Comp_Name"),
        vsp.GetDoubleResults(rid, "Wet_Area"),
        vsp.GetDoubleResults(rid, "Theo_Area"),
    ):
        wet[n] = wet.get(n, 0.0) + w_
        theo[n] = theo.get(n, 0.0) + t_
    if mesh:
        vsp.DeleteGeom(mesh)
    fw = {
        "Wing": W["area_m2"] * (1.977 + 0.52 * W["airfoil"]["tc_max"]),
        "HorizontalTail": HT["area_m2"] * (1.977 + 0.52 * HT["tc"]),
        "VerticalTail": VT["area_m2"] * (1.977 + 0.52 * VT["tc"]),
        "CargoBay": 2
        * (CB["length_m"] * CB["width_m"] + CB["length_m"] * CB["height_m"] + CB["width_m"] * CB["height_m"]),
        "TailBoom": 2 * np.pi * TB["radius_m"] * TB["length_m"],
    }
    for n in nacs:
        fw[vsp.GetGeomName(n)] = np.pi * P["d_nac_m"] * P["l_nac_m"]
    wet_rows = []
    for n, f in fw.items():
        wet_rows.append(
            (
                n,
                theo.get(n, float("nan")),
                wet.get(n, float("nan")),
                f,
                (theo.get(n, np.nan) - f) / f * 100,
                (wet.get(n, np.nan) - f) / f * 100,
            )
        )
    cst_d, cst_x = _cst_shape_difference(vsp, geom)

    # --- what is over the limit ---
    flagged = [r for r in rows if abs(r[3]) > 100 * FLAG] + [
        (f"{r[0]} area (CompGeom theoretical vs framework)", r[1], r[3], r[4], "")
        for r in wet_rows
        if abs(r[4]) > 100 * FLAG
    ]
    if md_path:
        _write_md(md_path, geom, rows, wet_rows, flagged, (cst_d, cst_x), vsp3_path)
    return {"rows": rows, "wetted": wet_rows, "flagged": flagged, "cst_max_dy": cst_d}


def _write_md(path, geom, rows, wet_rows, flagged, cst, vsp3_path):
    """Write the report."""
    pv = geom["provenance"]
    L = [
        f"# OpenVSP consistency check: {pv.get('case_id') or Path(vsp3_path).stem}\n",
        f"Model `{Path(vsp3_path).name}`, reloaded from disk with OpenVSP and compared with `geometry.json`",
        f"(git {str(pv.get('git_hash'))[:10]}{' dirty' if pv.get('git_dirty') else ''}). Generated by "
        "`aircrafter2.vsp.check`. The framework is not changed to match.\n",
        "## Parameters\n",
        "| Quantity | OpenVSP | framework | difference | note |",
        "|---|---:|---:|---:|---|",
    ]
    L += [f"| {q} | {a:.6g} | {e:.6g} | {d:+.3f} % | {n} |" for q, a, e, d, n in rows]
    L += [
        "",
        "## Wetted areas (m²)\n",
        "CompGeom on the export set (the markers are excluded). *Theoretical* is each component's own closed "
        "surface; *wetted* trims the parts inside other components. The framework values are its own "
        "approximations (`aero/parasite.py`):",
        "- wing and tails: Raymer's S (1.977 + 0.52 t/c);",
        "- cargo bay: 2(lw + lh + wh);",
        "- boom: 2π r l, the side only;",
        "- nacelles: π d l.\n",
        "| Component | CompGeom theoretical | CompGeom wetted | framework | theo. vs fw | wetted vs fw |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    L += [f"| {n} | {t:.5f} | {w:.5f} | {f:.5f} | {dt:+.1f} % | {dw:+.1f} % |" for n, t, w, f, dt, dw in wet_rows]
    L += [
        "",
        "Why these differ:",
        "- **Wing and tails:** Raymer's 1.977 + 0.52 t/c approximates a thin airfoil's perimeter. The theoretical "
        "area also includes the tip caps. The wing's *wetted* area loses the strip inside the cargo bay and under "
        "the nacelles.",
        "- **Cargo bay:** the mock-up's box has illustrative rounded corners (radius 12% of min(w, h)), which take "
        "area off the sharp-cornered 2(lw + lh + wh). Its wetted area also loses the wing and boom footprints.",
        "- **Boom:** the theoretical area adds the two flat end caps (2πr²). The wetted area loses the length "
        "inside the cargo bay.",
        "- **Nacelles:** the illustrative streamlined pod (OpenVSP POD) has less area than the cylinder π d l "
        "that `parasite.py` uses.",
        "",
        f"**Airfoil types:** OpenVSP's CST airfoil type, built from the same upper and lower weights, differs from "
        f"the exported Kulfan coordinates by up to {cst[0]:.4f} c (at x/c = {cst[1]:.3f}). OpenVSP's CST has no "
        "Kulfan leading-edge term and no TE thickness, so the model uses a file airfoil instead.",
        "",
    ]
    L += ["## Over 5 %\n"]
    L += [f"- {q}: OpenVSP {a:.5g} vs framework {e:.5g} ({d:+.1f} %)" for q, a, e, d, _ in flagged] or ["- none"]
    Path(path).write_text("\n".join(L) + "\n")
    return path
