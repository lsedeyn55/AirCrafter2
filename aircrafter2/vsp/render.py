"""Thesis figures of the OpenVSP mock-up, rendered from its exported STL meshes (no OpenVSP needed).

OpenVSP 3.55.0 can only screenshot through its FLTK GUI (ScreenGrab needs VSP_USE_FLTK and a running GUI), so it
cannot render headlessly. The builder exports one STL per component; this module draws them with matplotlib:
orthographic three-views as depth-sorted 2D projections (the top view uses the same axes as
plotting.figures.planform: y across, x down) and an orthographic isometric. Meshes are rasterised inside the PDF;
text and axes stay vector.
"""

from pathlib import Path

import numpy as np
from matplotlib.collections import PolyCollection
from matplotlib.colors import to_rgb
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from ..plotting.style import PALETTE, TEXT_WIDTH_IN, figure, save

COLOURS = {"wing": PALETTE[0], "tails": PALETTE[2], "cargo bay": PALETTE[3], "tail boom": "#4d4d4d",
           "nacelles": PALETTE[4], "propulsion": PALETTE[5]}
ORDER = ["cargo bay", "tail boom", "tails", "nacelles", "wing", "propulsion"]
LIGHT = np.array([-0.4, -0.5, 0.75])
LIGHT = LIGHT / np.linalg.norm(LIGHT)
DPI = 300


def read_stl(path):
    """(n, 3, 3) triangle array from an ASCII or binary STL."""
    data = Path(path).read_bytes()
    if data[:5] == b"solid" and b"facet" in data[:400]:
        v = [ln.split()[1:4] for ln in data.decode().splitlines() if ln.strip().startswith("vertex")]
        return np.asarray(v, float).reshape(-1, 3, 3)
    n = int(np.frombuffer(data[80:84], "<u4")[0])
    rec = np.frombuffer(data[84:84 + 50 * n], dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))
    return rec["v"].astype(float)


def load_meshes(component_stls):
    return {k: read_stl(p) for k, p in component_stls.items() if k != "markers" and Path(p).exists()}


def _shade(tris, colour, light=LIGHT):
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    nn = np.linalg.norm(n, axis=1, keepdims=True)
    n = np.divide(n, nn, out=np.zeros_like(n), where=nn > 0)
    k = 0.45 + 0.55 * np.abs(n @ light)  # two-sided Lambert
    return np.clip(np.asarray(to_rgb(colour))[None, :] * k[:, None], 0, 1)


def _view_axes(view):
    """(horizontal axis index, vertical axis index, depth axis index, depth sign) for 2D projections."""
    return {"top": (1, 0, 2, 1.0),     # y across, x (down, inverted by the caller), seen from +z
            "side": (0, 2, 1, -1.0),   # x across, z up, seen from -y (port side)
            "front": (1, 2, 0, -1.0)}[view]  # y across, z up, seen from -x (ahead)


def draw_projection(ax, meshes, view, alpha=1.0, flat=None):
    """Draw every component's triangles projected onto the view plane, painter-sorted by depth."""
    h, v, d, s = _view_axes(view)
    polys, cols, depth = [], [], []
    for comp in ORDER:
        if comp not in meshes:
            continue
        t = meshes[comp]
        polys.append(t[:, :, [h, v]])
        cols.append(_shade(t, flat or COLOURS[comp]))
        depth.append(s * t[:, :, d].mean(axis=1))
    P, C, Dp = np.concatenate(polys), np.concatenate(cols), np.concatenate(depth)
    idx = np.argsort(Dp)
    pc = PolyCollection(P[idx], facecolors=C[idx], edgecolors=C[idx], linewidths=0.05, alpha=alpha)
    pc.set_rasterized(True)
    ax.add_collection(pc)
    lo, hi = P.reshape(-1, 2).min(axis=0), P.reshape(-1, 2).max(axis=0)
    return lo, hi


def _markers_2d(ax, geom, view):
    st, z = geom["stations"], -geom["cargo_bay"]["height_m"] / 2.0
    pts = {"top": lambda x: (0.0, x), "side": lambda x: (x, z), "front": lambda x: (0.0, z)}[view]
    if view == "front":
        return
    ax.plot(*pts(st["x_cg_m"]), "o", ms=5, color=PALETTE[1], mec="white", mew=0.6,
            label=f"CG (SM = {st['static_margin']:.2f})", zorder=5)
    ax.plot(*pts(st["x_np_m"]), "^", ms=5, color="k", mec="white", mew=0.6, label="neutral point", zorder=5)


def three_view(geom, meshes, path, title=None):
    fig, axs = figure(1.0, 0.95, nrows=2, ncols=2,
                      gridspec_kw=dict(height_ratios=[1.6, 1.0], hspace=0.35, wspace=0.25))
    for ax, view, lab in [(axs[0, 0], "top", ("y (m)", "x (m)")), (axs[0, 1], "front", ("y (m)", "z (m)")),
                          (axs[1, 0], "side", ("x (m)", "z (m)"))]:
        lo, hi = draw_projection(ax, meshes, view)
        _markers_2d(ax, geom, view)
        pad = 0.04 * (hi - lo).max()
        ax.set_xlim(lo[0] - pad, hi[0] + pad)
        ax.set_ylim(lo[1] - pad, hi[1] + pad)
        ax.set_aspect("equal")
        ax.set_xlabel(lab[0])
        ax.set_ylabel(lab[1])
        ax.set_title({"top": "top", "front": "front (from ahead)", "side": "side (from port)"}[view])
        if view == "top":
            ax.invert_yaxis()
            handles, labels = ax.get_legend_handles_labels()
    axs[1, 1].axis("off")
    axs[1, 1].legend(handles, labels, loc="lower left", fontsize=7, frameon=False)
    axs[1, 1].text(0.0, 0.95, title or _caption(geom), va="top", fontsize=7.5, transform=axs[1, 1].transAxes)
    fig.dpi = DPI
    return save(fig, path, dpi=DPI)


def _caption(geom):
    P, W, pv = geom["propulsion"], geom["wing"], geom["provenance"]
    lay = "1 nose tractor" if P["n_motors"] == 1 else f"2 wing tractors at y = ±{P['y_m_m']:.3f} m"
    return (f"{pv.get('case_id') or ''}\n{lay}, APC {P['prop']} ({P['D_prop_m'] / 0.0254:.0f} in), "
            f"{pv['n_cells']:.0f}S\n3-lap time {pv['ThreeLapTime_s']:.2f} s, "
            f"m = {geom['stations']['masses_kg']['m_total']:.2f} kg\n"
            f"b = {W['span_m']:.3f} m, c = {W['chord_m']:.3f} m, t/c = {W['airfoil']['tc_max']:.3f}\n\n"
            "Illustrative (not from the result): vertical\nplacement, blade and nacelle shape, spinner,\nmotor mount"
            + (", fairings." if geom.get("mockup_options", {}).get("fairings") else "."))


def draw_iso(ax, meshes, geom=None, elev=22.0, azim=-128.0, limits=None, zoom=1.35):
    ax.set_proj_type("ortho")
    for comp in ORDER:
        if comp not in meshes:
            continue
        t = meshes[comp]
        pc = Poly3DCollection(t, facecolors=_shade(t, COLOURS[comp]), edgecolors="none", linewidths=0.0)
        pc.set_rasterized(True)
        ax.add_collection3d(pc)
    if limits is None:
        allp = np.concatenate([m.reshape(-1, 3) for m in meshes.values()])
        limits = iso_limits([allp])
    (x0, x1), (y0, y1), (z0, z1) = limits
    ax.set_xlim(x0, x1)
    ax.set_ylim(y0, y1)
    ax.set_zlim(z0, z1)
    ax.set_box_aspect((x1 - x0, y1 - y0, z1 - z0), zoom=zoom)
    ax.view_init(elev=elev, azim=azim)
    if geom is not None:  # CG / NP sit inside the cargo bay: draw them as pins above the model, on top of the mesh
        ax.computed_zorder = False
        st, z = geom["stations"], -geom["cargo_bay"]["height_m"] / 2.0
        top = z1 - 0.15 * (z1 - z0)
        for x, mk, col, lab in ((st["x_cg_m"], "o", PALETTE[1], "CG"), (st["x_np_m"], "^", "k", "NP")):
            ax.plot([x, x], [0, 0], [z, top], color=col, lw=0.6, ls=":", zorder=10)
            ax.scatter([x], [0], [top], marker=mk, s=16, color=col, edgecolors="white", linewidths=0.5,
                       depthshade=False, zorder=11, label=f"{lab} (pin above its station)")
    ax.set_axis_off()
    return limits


def iso_limits(point_sets, pad=0.03):
    p = np.concatenate([np.asarray(s).reshape(-1, 3) for s in point_sets])
    lo, hi = p.min(axis=0), p.max(axis=0)
    span = (hi - lo).max()
    return [(lo[i] - pad * span, hi[i] + pad * span) for i in range(3)]


def isometric(geom, meshes, path, limits=None, title=True):
    fig = figure(0.8, 0.75)[0]
    fig.clf()
    ax = fig.add_subplot(projection="3d")
    draw_iso(ax, meshes, geom, limits=limits)
    ax.legend(loc="lower right", fontsize=7, frameon=False)
    if title:
        ax.set_title(_caption(geom).split("\n\n")[0].replace("\n", ", ", 1), fontsize=7.5)
    fig.dpi = DPI
    return save(fig, path, dpi=DPI)


def side_by_side(items, path, labels):
    """Isometrics of several designs at the same scale and camera. items: list of (geom, meshes)."""
    limits = iso_limits([m.reshape(-1, 3) for _, ms in items for m in ms.values()])
    w = TEXT_WIDTH_IN
    fig = figure(1.0, 0.30)[0]
    fig.clf()
    for i, ((geom, meshes), lab) in enumerate(zip(items, labels)):
        ax = fig.add_subplot(1, len(items), i + 1, projection="3d")
        draw_iso(ax, meshes, geom, limits=limits, zoom=1.5)
        ax.set_title(lab, fontsize=7.5, y=0.93)
        if i == 0:
            ax.legend(loc="lower left", fontsize=6.5, frameon=False, bbox_to_anchor=(0.02, 0.06))
    fig.subplots_adjust(left=0.0, right=1.0, top=0.95, bottom=0.0, wspace=0.0)
    fig.set_size_inches(w, w * 0.30)
    fig.dpi = DPI
    return save(fig, path, dpi=DPI)
