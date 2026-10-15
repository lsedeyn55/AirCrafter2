"""Build an OpenVSP 3.55.0 model from geometry.json.

Frame: the framework's own (origin at the wing root LE, x aft, y to starboard, z up), in metres. OpenVSP models
carry no length unit, so the model is built in metres and the STEP/IGES export unit is set to metres
(STEPSettings:LenUnit = LEN_M, saved in the .vsp3).

From the result: wing span / chord / airfoil, tail areas / aspect ratios / c/4 stations / t/c, boom start / length /
radius, cargo-bay box and station, prop diameter and stations, nacelle length and diameter, spar station and radius,
CG and NP. Illustrative (mockup_assumptions): vertical placement, blade shape, spinner, nacelle shape, the nose motor
mount, fairings.

Every key parameter is read back after it is set (`readback` in the result).
"""

import time
from pathlib import Path

import numpy as np

from . import require_openvsp

EXPORT_SET = None  # set index holding everything except the markers (assigned at build time)


def _vsp():
    require_openvsp()
    import openvsp as vsp

    return vsp


def write_dat(coords, path, name):
    """Selig-format airfoil file (name line, then x y)."""
    path = Path(path)
    path.write_text(name + "\n" + "\n".join(f"{x:.8f} {y:.8f}" for x, y in coords) + "\n")
    return path


class _Builder:
    def __init__(self, vsp):
        self.vsp = vsp
        self.readback = {}
        self.components = {}  # name -> list of geom ids
        self.markers = []

    # --- helpers -------------------------------------------------------------------------------------------------
    def setp(self, gid, name, group, value):
        vsp = self.vsp
        pid = vsp.FindParm(gid, name, group)
        if not pid:
            raise KeyError(f"OpenVSP parm {group}:{name} not found on {vsp.GetGeomName(gid)}")
        vsp.SetParmVal(pid, float(value))
        return pid

    def setx(self, xsec, name, value):
        pid = self.vsp.GetXSecParm(xsec, name)
        if not pid:
            raise KeyError(f"OpenVSP xsec parm {name} not found")
        self.vsp.SetParmVal(pid, float(value))
        return pid

    def place(self, gid, x=0.0, y=0.0, z=0.0, rx=0.0, ry=0.0, rz=0.0):
        for n, val in [("X_Rel_Location", x), ("Y_Rel_Location", y), ("Z_Rel_Location", z),
                       ("X_Rel_Rotation", rx), ("Y_Rel_Rotation", ry), ("Z_Rel_Rotation", rz)]:
            self.setp(gid, n, "XForm", val)

    def add(self, typ, name, component, marker=False):
        gid = self.vsp.AddGeom(typ)
        self.vsp.SetGeomName(gid, name)
        (self.markers if marker else self.components.setdefault(component, [])).append(gid)
        return gid

    # --- components ----------------------------------------------------------------------------------------------
    def wing(self, name, component, span_total, chord, mirrored=True, airfoil_dat=None, tc=None):
        """Rectangular one-section wing. airfoil_dat: file airfoil; else NACA 4-series symmetric at tc."""
        vsp = self.vsp
        gid = self.add("WING", name, component)
        if not mirrored:
            self.setp(gid, "Sym_Planar_Flag", "Sym", 0)
        surf = vsp.GetXSecSurf(gid, 0)
        x1 = vsp.GetXSec(surf, 1)
        half = span_total / 2.0 if mirrored else span_total
        vsp.SetDriverGroup(gid, 1, vsp.SPAN_WSECT_DRIVER, vsp.ROOTC_WSECT_DRIVER, vsp.TIPC_WSECT_DRIVER)
        for n, val in [("Span", half), ("Root_Chord", chord), ("Tip_Chord", chord), ("Sweep", 0.0),
                       ("Dihedral", 0.0), ("Twist", 0.0)]:
            self.setx(x1, n, val)
        for i in (0, 1):
            if airfoil_dat is not None:
                vsp.ChangeXSecShape(surf, i, vsp.XS_FILE_AIRFOIL)
                vsp.ReadFileAirfoil(vsp.GetXSec(surf, i), str(airfoil_dat))
            else:
                vsp.ChangeXSecShape(surf, i, vsp.XS_FOUR_SERIES)
                xs = vsp.GetXSec(surf, i)
                self.setx(xs, "ThickChord", tc)
                self.setx(xs, "Camber", 0.0)
        return gid

    def body(self, name, component, length, shape, w, h=None, caps=True, marker=False):
        """Constant-section FUSELAGE along +x from its origin: rounded rectangle (w x h) or circle (diameter w)."""
        vsp = self.vsp
        gid = self.add("FUSELAGE", name, component, marker=marker)
        self.setp(gid, "Length", "Design", length)
        surf = vsp.GetXSecSurf(gid, 0)
        # keep only the two end sections: a straight prism needs no intermediate sections
        while vsp.GetNumXSec(surf) > 2:
            vsp.CutXSec(gid, 1)
        for i in range(vsp.GetNumXSec(surf)):
            if shape == "circle":
                vsp.ChangeXSecShape(surf, i, vsp.XS_CIRCLE)
                self.setx(vsp.GetXSec(surf, i), "Circle_Diameter", w)
            else:
                vsp.ChangeXSecShape(surf, i, vsp.XS_ROUNDED_RECTANGLE)
                xs = vsp.GetXSec(surf, i)
                vsp.SetXSecWidthHeight(xs, w, h)
                self.setx(xs, "RoundRectXSec_Radius", 0.12 * min(w, h))  # illustrative corner radius
            # the default end sections carry +/-45 deg skinning tangents (a nose / tail taper), which bulge a
            # constant-section body by up to 18x its diameter; zero them for a straight prism
            xs = vsp.GetXSec(surf, i)
            for pid in vsp.GetXSecParmIDs(xs):
                if vsp.GetParmName(pid).endswith(("LAngle", "RAngle")):
                    vsp.SetParmVal(pid, 0.0)
        if caps:
            self.setp(gid, "CapUMinOption", "EndCap", vsp.FLAT_END_CAP)
            self.setp(gid, "CapUMaxOption", "EndCap", vsp.FLAT_END_CAP)
        return gid

    def fairing(self, name, length, w, h, point_first):
        """Illustrative straight taper between the cargo-bay section and a point (cargo bay component)."""
        vsp = self.vsp
        gid = self.body(name, "cargo bay", length, "rect", w, h, caps=False)
        surf = vsp.GetXSecSurf(gid, 0)
        vsp.ChangeXSecShape(surf, 0 if point_first else 1, vsp.XS_POINT)
        self.setp(gid, "CapUMinOption" if not point_first else "CapUMaxOption", "EndCap", vsp.FLAT_END_CAP)
        return gid

    def pod(self, name, component, length, diameter, marker=False):
        gid = self.add("POD", name, component, marker=marker)
        self.setp(gid, "Length", "Design", length)
        # OpenVSP 3.55 POD: max diameter = 2 Length / FineRatio (measured: FineRatio 6 on a 0.15 m pod gives a
        # 0.05 m bounding box), so FineRatio is length / radius here
        self.setp(gid, "FineRatio", "Design", 2.0 * length / diameter)
        return gid

    def prop(self, name, component, diameter, n_blades):
        gid = self.add("PROP", name, component)
        self.setp(gid, "Diameter", "Design", diameter)
        self.setp(gid, "NumBlade", "Design", n_blades)
        return gid

    def sphere(self, name, r):
        gid = self.add("ELLIPSOID", name, None, marker=True)
        for n in ("A_Radius", "B_Radius", "C_Radius"):
            self.setp(gid, n, "Design", r)
        return gid


def build(geom, out_dir, name=None, incidence=None):
    """Build, export and read back. Returns a dict with the file paths, the read-back values and the build time."""
    vsp = _vsp()
    t0 = time.perf_counter()
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    name = name or geom["provenance"].get("case_id") or "aircrafter2"
    W, HT, VT, TB, CB, P = (geom[k] for k in ("wing", "horizontal_tail", "vertical_tail", "tail_boom", "cargo_bay",
                                              "propulsion"))
    opts = geom.get("mockup_options", {})
    vsp.VSPRenew()
    vsp.ClearVSPModel()
    B = _Builder(vsp)

    # vertical placement (mockup_assumptions): wing chord line z = 0 on top of the cargo bay
    z_cb = -CB["height_m"] / 2.0
    c, b = W["chord_m"], W["span_m"]

    # wing: exported Kulfan coordinates as a file airfoil
    dat = write_dat(W["airfoil"]["coordinates_selig"], out / f"{name}_wing.dat", f"{name} wing (AirCrafter 2.0 CST)")
    wing = B.wing("Wing", "wing", b, c, mirrored=True, airfoil_dat=dat)

    # tails: rectangular, NACA 4-series symmetric at tc_tail, LE at x_c/4 - c/4
    z_ht = z_cb + TB["radius_m"]
    ht = B.wing("HorizontalTail", "tails", HT["span_m"], HT["chord_m"], mirrored=True, tc=HT["tc"])
    B.place(ht, x=HT["x_quarter_chord_m"] - 0.25 * HT["chord_m"], z=z_ht)
    vt = B.wing("VerticalTail", "tails", VT["height_m"], VT["chord_m"], mirrored=False, tc=VT["tc"])
    B.place(vt, x=VT["x_quarter_chord_m"] - 0.25 * VT["chord_m"], z=z_ht, rx=90.0)

    # cargo bay box and tail boom
    cb = B.body("CargoBay", "cargo bay", CB["length_m"], "rect", CB["width_m"], CB["height_m"])
    B.place(cb, x=CB["x_centre_m"] - CB["length_m"] / 2.0, z=z_cb)
    tb = B.body("TailBoom", "tail boom", TB["length_m"], "circle", 2.0 * TB["radius_m"])
    B.place(tb, x=TB["x_start_m"], z=z_cb)
    if opts.get("fairings", False):  # illustrative nose and tail fairings on the cargo bay (default off)
        x_front, x_rear = CB["x_centre_m"] - CB["length_m"] / 2.0, CB["x_centre_m"] + CB["length_m"] / 2.0
        ln, lt = 0.6 * CB["height_m"], 0.9 * CB["height_m"]
        nose = B.fairing("NoseFairing", ln, CB["width_m"], CB["height_m"], point_first=True)
        B.place(nose, x=x_front - ln, z=z_cb)
        tail = B.fairing("TailFairing", lt, CB["width_m"], CB["height_m"], point_first=False)
        B.place(tail, x=x_rear, z=z_cb)

    # propulsion: prop discs at x_motor; blade shape illustrative
    D, xm = P["D_prop_m"], P["x_motor_m"]
    nb = int(opts.get("n_blades", 2))
    d_sp = float(opts.get("spinner_diameter_frac", 0.18)) * D
    props = []
    if P["n_motors"] == 1:
        pr = B.prop("Prop", "propulsion", D, nb)
        B.place(pr, x=xm, z=z_cb)
        props.append(pr)
        sp = B.pod("Spinner", "propulsion", 1.6 * d_sp, d_sp)
        B.place(sp, x=xm - 0.8 * d_sp, z=z_cb)
        x_front = CB["x_centre_m"] - CB["length_m"] / 2.0
        if x_front - xm > d_sp:  # illustrative motor mount from the prop to the cargo bay front
            mount = B.body("MotorMount", "propulsion", x_front - xm, "circle", d_sp)
            B.place(mount, x=xm, z=z_cb)
    else:
        ym, ln, dn = P["y_m_m"], P["l_nac_m"], P["d_nac_m"]
        z_n = -float(opts.get("nacelle_drop_m", 0.8 * 0.0254))
        for side, y in (("R", ym), ("L", -ym)):
            nac = B.pod(f"Nacelle_{side}", "nacelles", ln, dn)
            B.place(nac, x=xm, y=y, z=z_n)
            pr = B.prop(f"Prop_{side}", "propulsion", D, nb)
            B.place(pr, x=xm, y=y, z=z_n)
            props.append(pr)

    # markers (never exported): CG / NP spheres on the cargo-bay axis, spar tube at x_spar c
    if opts.get("markers", True):
        r_m = 0.012
        cg = B.sphere("CG_marker", r_m)
        B.place(cg, x=geom["stations"]["x_cg_m"], z=z_cb)
        npm = B.sphere("NP_marker", r_m)
        B.place(npm, x=geom["stations"]["x_np_m"], z=z_cb)
        xy = np.array(W["airfoil"]["coordinates_selig"])
        xs = W["x_spar_frac"]
        n = len(xy) // 2
        yu = np.interp(xs, xy[:n + 1, 0][::-1], xy[:n + 1, 1][::-1])
        yl = np.interp(xs, xy[n:, 0], xy[n:, 1])
        spar = B.body("SparTube_marker", None, b, "circle", 2.0 * W["r_tube_m"], marker=True)
        B.place(spar, x=W["x_spar_m"], y=-b / 2.0, z=0.5 * (yu + yl) * c, rz=90.0)

    if incidence == "cruise" or (incidence is None and opts.get("wing_incidence") == "cruise"):
        for gids in B.components.values():
            for g in gids:
                B.setp(g, "Y_Rel_Rotation", "XForm", -W["cruise_aoa_deg"])  # nose up about the origin

    vsp.Update()

    # read back the key parameters
    rb = B.readback
    rb["wing_span_m"] = vsp.GetParmVal(wing, "TotalSpan", "WingGeom")
    rb["wing_area_m2"] = vsp.GetParmVal(wing, "TotalArea", "WingGeom")
    rb["wing_chord_m"] = vsp.GetParmVal(vsp.GetXSecParm(vsp.GetXSec(vsp.GetXSecSurf(wing, 0), 1), "Root_Chord"))
    rb["ht_area_m2"] = vsp.GetParmVal(ht, "TotalArea", "WingGeom")
    rb["ht_span_m"] = vsp.GetParmVal(ht, "TotalSpan", "WingGeom")
    rb["vt_area_m2"] = vsp.GetParmVal(vt, "TotalArea", "WingGeom")
    rb["ht_le_x_m"] = vsp.GetParmVal(ht, "X_Rel_Location", "XForm")
    rb["cargo_bay_length_m"] = vsp.GetParmVal(cb, "Length", "Design")
    rb["boom_length_m"] = vsp.GetParmVal(tb, "Length", "Design")
    rb["prop_diameters_m"] = [vsp.GetParmVal(p, "Diameter", "Design") for p in props]
    rb["prop_x_m"] = [vsp.GetParmVal(p, "X_Rel_Location", "XForm") for p in props]
    rb["prop_y_m"] = [vsp.GetParmVal(p, "Y_Rel_Location", "XForm") for p in props]
    rb["n_props"] = len(props)

    def bbox(g):
        return {k: vsp.GetParmVal(g, k, "BBox") for k in ("X_Min", "X_Len", "Y_Len", "Z_Len")}

    checks = {"cargo bay width": (bbox(cb)["Y_Len"], CB["width_m"]),
              "cargo bay height": (bbox(cb)["Z_Len"], CB["height_m"]),
              "cargo bay front x": (bbox(cb)["X_Min"], CB["x_centre_m"] - CB["length_m"] / 2.0),
              "boom diameter": (bbox(tb)["Y_Len"], 2.0 * TB["radius_m"]),
              "boom length": (bbox(tb)["X_Len"], TB["length_m"])}
    for gid in B.components.get("nacelles", []):
        checks[f"{vsp.GetGeomName(gid)} diameter"] = (bbox(gid)["Y_Len"], P["d_nac_m"])
        checks[f"{vsp.GetGeomName(gid)} length"] = (bbox(gid)["X_Len"], P["l_nac_m"])
    rb["bbox_checks"] = {k: {"vsp": a, "framework": e, "rel_diff": (a - e) / e if e else a - e}
                         for k, (a, e) in checks.items()}
    bad = {k: c for k, c in rb["bbox_checks"].items() if abs(c["rel_diff"]) > 0.01}
    if bad:
        raise RuntimeError(f"OpenVSP read-back differs from geometry.json by > 1 %: {bad}")
    rb["n_nacelles"] = len(B.components.get("nacelles", []))

    # sets: export set = everything but the markers; one temporary set per component for the render STLs
    export_set, comp_set = vsp.SET_FIRST_USER, vsp.SET_FIRST_USER + 1
    vsp.SetSetName(export_set, "export")
    vsp.SetSetName(comp_set, "component")
    for gids in B.components.values():
        for g in gids:
            vsp.SetSetFlag(g, export_set, True)
    veh = vsp.GetVehicleID()
    for grp in ("STEPSettings", "IGESSettings"):
        pid = vsp.FindParm(veh, "LenUnit", grp)
        if pid:
            vsp.SetParmVal(pid, vsp.LEN_M)
    vsp.Update()

    files = {"vsp3": out / f"{name}.vsp3", "step": out / f"{name}.stp", "stl": out / f"{name}.stl",
             "airfoil_dat": dat}
    vsp.SetVSP3FileName(str(files["vsp3"]))
    vsp.WriteVSPFile(str(files["vsp3"]), vsp.SET_ALL)
    vsp.ExportFile(str(files["step"]), export_set, vsp.EXPORT_STEP)
    _export_stl(vsp, files["stl"], export_set)
    comp_dir = out / "components"
    comp_dir.mkdir(exist_ok=True)
    files["component_stls"] = {}
    for comp, gids in list(B.components.items()) + [("markers", B.markers)]:
        if not gids:
            continue
        for g in vsp.FindGeoms():
            vsp.SetSetFlag(g, comp_set, g in gids)
        path = comp_dir / f"{comp.replace(' ', '_')}.stl"
        _export_stl(vsp, path, comp_set)
        files["component_stls"][comp] = path
    return {"files": files, "readback": rb, "build_s": time.perf_counter() - t0, "name": name}


def _export_stl(vsp, path, set_index):
    mesh = vsp.ExportFile(str(path), set_index, vsp.EXPORT_STL)
    if mesh:
        vsp.DeleteGeom(mesh)
    return path
