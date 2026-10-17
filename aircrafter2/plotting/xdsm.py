"""XDSM of the AirCrafter 2.0 MDF architecture (docs/PLAN.md §4, as built in full_mdo).

write_tex   pyXDSM TikZ source (compile with LaTeX, e.g. Overleaf; needs the pyxdsm .sty files).
draw_pdf    a matplotlib quick-look version (no LaTeX needed).
"""

from pathlib import Path

from matplotlib.patches import FancyBboxPatch

from .style import PALETTE, figure, save

BLOCKS = [  # (id, label, kind)
    ("opt", "Optimiser\n(SLSQP)", "optimizer"),
    ("mda", "MDA\n(Newton)", "mda"),
    ("geo", "Geometry", "function"),
    ("af", "Airfoil\n(CST)", "function"),
    ("prop", "Propulsion", "function"),
    ("aero", "Aero\n(VLM +\nNeuralFoil)", "function"),
    ("mass", "Mass\n(G&G 2022)", "function"),
    ("stab", "Stability", "function"),
    ("lap", "Mission\n(lap model)", "function"),
    ("score", "Scoring\nplug-in", "function"),
]
DATA = [  # (from, to, label)  upper triangle: feed-forward; lower: feedback
    ("opt", "geo", "b, c, l_ht, S_ht, S_vt, stations"), ("opt", "af", "t/c, camber"), ("opt", "prop", "capacity"),
    ("mda", "aero", "α, V"), ("mda", "mass", "m"),
    ("geo", "aero", "mesh, S, tails"), ("geo", "mass", "b, c, l_tb, S_cb, y_m"), ("geo", "stab", "V_h, stations"),
    ("af", "aero", "Kulfan"), ("af", "mass", "t/c, A/c², P/c"),
    ("prop", "aero", "T(V)"), ("prop", "geo", "D_prop (twin)"), ("prop", "mass", "m_sys, m_unit (twin)"),
    ("prop", "lap", "T(v), P(v), E"),
    ("aero", "mass", "C_m, C_Lα"), ("aero", "stab", "a_w"), ("aero", "lap", "polar, C_L,max"),
    ("mass", "aero", "W"), ("mass", "stab", "masses"), ("mass", "lap", "m"),
    ("lap", "score", "lap times, laps"),
    ("score", "opt", "f"), ("stab", "opt", "SM"), ("lap", "opt", "stall, turn, thrust, energy margins"),
    ("mass", "opt", "tube t/r, r_tube"), ("geo", "opt", "V_h, V_v, prop clearances (twin)"),
    ("aero", "opt", "section c_l margin"), ("prop", "opt", "rpm, v-fit margins"),
]


def write_tex(path):
    from pyxdsm.XDSM import FUNC, OPT, SOLVER, XDSM

    def tex(label):
        for a, b in [("_", r"\_"), ("α", r"$\alpha$"), ("²", r"$^2$"), ("&", r"\&")]:
            label = label.replace(a, b)
        return label.replace("\n", r"\\ ")

    kinds = {"optimizer": OPT, "mda": SOLVER, "function": FUNC}
    x = XDSM()
    for bid, label, kind in BLOCKS:
        x.add_system(bid, kinds[kind], tex(label))
    for a, b, label in DATA:
        x.connect(a, b, tex(label))
    path = Path(path)
    x.write(path.stem, build=False, outdir=str(path.parent))  # \input{} path relative to the .tex
    return path.with_suffix(".tex")


def draw_pdf(path):
    n = len(BLOCKS)
    idx = {bid: i for i, (bid, _, _) in enumerate(BLOCKS)}
    import textwrap

    fig, ax = figure(1.0, 1.0)
    colours = {"optimizer": PALETTE[2], "mda": PALETTE[3], "function": PALETTE[0]}
    for i, (_, label, kind) in enumerate(BLOCKS):
        ax.add_patch(FancyBboxPatch((i - 0.45, -i - 0.35), 0.9, 0.7, boxstyle="round,pad=0.02",
                                    fc=colours[kind], alpha=0.25, ec=colours[kind]))
        ax.text(i, -i, label, ha="center", va="center", fontsize=5)
    for a, b, label in DATA:
        i, j = idx[a], idx[b]
        ax.text(j, -i, textwrap.fill(label, 12), ha="center", va="center", fontsize=4.2,
                bbox=dict(boxstyle="square,pad=0.15", fc="#f2f2f2", ec="#999999", lw=0.4))
    for i in range(n):
        ax.plot([i, i], [0.5, -n + 0.5], color="#dddddd", lw=0.4, zorder=0)
        ax.plot([-0.5, n - 0.5], [-i, -i], color="#dddddd", lw=0.4, zorder=0)
    ax.set_xlim(-0.6, n - 0.4)
    ax.set_ylim(-n + 0.4, 0.6)
    ax.axis("off")
    ax.set_title("AirCrafter 2.0 MDF (upper: feed-forward, lower: feedback)")
    return save(fig, path)
