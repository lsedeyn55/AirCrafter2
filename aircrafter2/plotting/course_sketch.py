"""Top-view schematic of a course, drawn from the legs, for the UI (docs/modules/mission.md, "Course").

Straights are drawn to scale (ft). Turns are arcs of a fixed schematic radius: to the left, except whole 360 deg loops,
drawn to the right so they stay outside the course (the lap model ignores the turn direction, and its turn radius comes
from the flight state, not from the course). The first straight starts at the start line heading +x; the wind arrow
points along the wind (it blows against the head-wind straights).
"""

import numpy as np

from ..mission.course import lap_length_ft, total_turning_deg

R_SCHEMATIC_FT = 80.0  # schematic turn radius (ft)


def _path(legs, r=R_SCHEMATIC_FT, n_arc=60):
    """(segments, heading of each head-wind straight): segments = [(kind, xs, ys, leg number)]."""
    x, y, hdg = 0.0, 0.0, 0.0
    segs, head_hdgs = [], []
    for i, leg in enumerate(legs, start=1):
        if leg.kind == "straight":
            x1, y1 = x + leg.length_ft * np.cos(hdg), y + leg.length_ft * np.sin(hdg)
            segs.append(("straight", np.array([x, x1]), np.array([y, y1]), i, leg.wind))
            if leg.wind == "head":
                head_hdgs.append(hdg)
            elif leg.wind == "tail":
                head_hdgs.append(hdg + np.pi)
            x, y = x1, y1
        else:
            ang = np.radians(leg.angle_deg)
            side = -1.0 if abs(leg.angle_deg % 360.0) < 1e-9 else 1.0  # +1 left, -1 right (whole loops)
            cx, cy = x - side * r * np.sin(hdg), y + side * r * np.cos(hdg)
            th = hdg - side * np.pi / 2 + side * np.linspace(0.0, ang, n_arc)
            xs, ys = cx + r * np.cos(th), cy + r * np.sin(th)
            segs.append(("turn", xs, ys, i, None))
            x, y, hdg = xs[-1], ys[-1], hdg + side * ang
    return segs, head_hdgs


def _draw(ax, legs, title, takeoff_on=False, start_offset_ft=0.0):
    """Draw one lap on an axis."""
    from matplotlib.patches import Rectangle

    segs, head_hdgs = _path(legs)
    colour = {"head": "#c0392b", "tail": "#2471a3", "none": "#7f8c8d", None: "#555555"}
    if takeoff_on and legs and legs[0].kind == "straight":  # runway along the first straight, from brake release
        L = min(legs[0].length_ft + start_offset_ft, 300.0)
        ax.add_patch(Rectangle((-start_offset_ft, -12.0), L, 24.0, color="#d5d8dc", zorder=0))
        ax.annotate("runway", (-start_offset_ft + 5.0, 18.0), fontsize=7, color="0.4")
    for kind, xs, ys, i, wind in segs:
        ax.plot(
            xs, ys, color=colour[wind], lw=2.0 if kind == "straight" else 1.4, ls="-" if kind == "straight" else "--"
        )
        k = len(xs) // 2
        xm, ym = (xs.mean(), ys.mean()) if kind == "straight" else (xs[k], ys[k])
        ax.annotate(
            str(i),
            (xm, ym),
            fontsize=8,
            ha="center",
            va="center",
            bbox=dict(boxstyle="circle,pad=0.2", fc="white", ec="0.5", lw=0.6),
        )
    ax.plot([0.0, 0.0], [-40.0, 40.0], color="k", lw=2.5)
    ax.annotate("start", (0.0, -58.0), fontsize=8, ha="center")
    xs = np.concatenate([s[1] for s in segs])
    ys = np.concatenate([s[2] for s in segs])
    span = max(xs.max() - xs.min(), ys.max() - ys.min(), 200.0)
    if head_hdgs:  # wind blows opposite to the head-wind heading
        h = head_hdgs[0] + np.pi
        x0, y0 = xs.min() + 0.08 * span, ys.max() + 0.12 * span
        ax.annotate(
            "",
            (x0 + 0.15 * span * np.cos(h) + 0.15 * span, y0 + 0.15 * span * np.sin(h)),
            (x0 + 0.15 * span, y0),
            arrowprops=dict(arrowstyle="->", lw=1.5),
        )
        ax.annotate("wind", (x0 + 0.15 * span, y0 + 0.04 * span), fontsize=8, ha="center")
    else:
        ax.annotate("no along-track wind", (xs.min(), ys.max() + 0.12 * span), fontsize=8)
    ax.set_title(
        f"{title}: {lap_length_ft(legs):.0f} ft of straights, {total_turning_deg(legs):g}° of turning", fontsize=9
    )
    ax.set_aspect("equal")
    ax.set_xlim(xs.min() - 0.1 * span, xs.max() + 0.1 * span)
    ax.set_ylim(ys.min() - 0.1 * span, ys.max() + 0.25 * span)
    ax.set_xlabel("ft")
    ax.tick_params(labelsize=7)


def course_sketch(other, lap1=None, takeoff_on=False, start_offset_ft=0.0):
    """Matplotlib figure of a lap, and of a different lap 1 beside it, if given.

    Parameters
    ----------
    other : list of course.Leg
    lap1 : list of course.Leg, optional
    takeoff_on : bool
        Draw the runway.
    start_offset_ft : float

    Returns
    -------
    Figure
    """
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D

    laps = [("lap 1", lap1), ("lap 2+", other)] if lap1 is not None and lap1 != other else [("every lap", other)]
    fig, axes = plt.subplots(1, len(laps), figsize=(5.0 * len(laps), 4.0), squeeze=False)
    for ax, (title, legs) in zip(axes[0], laps):
        first = title in ("lap 1", "every lap")
        _draw(ax, legs, title, takeoff_on and first, start_offset_ft if first else 0.0)
    handles = [
        Line2D([], [], color="#c0392b", lw=2, label="head-wind straight"),
        Line2D([], [], color="#2471a3", lw=2, label="tail-wind straight"),
        Line2D([], [], color="#7f8c8d", lw=2, label="crosswind straight"),
        Line2D([], [], color="#555555", lw=1.4, ls="--", label=f"turn (schematic radius {R_SCHEMATIC_FT:.0f} ft)"),
    ]
    fig.legend(handles=handles, loc="lower center", ncol=2, fontsize=7, frameon=False)
    fig.tight_layout(rect=(0, 0.12, 1, 1))
    return fig
