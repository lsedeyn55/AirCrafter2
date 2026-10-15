"""One consistent, thesis-quality figure style (vector PDF, embedded TrueType fonts)."""

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

TEXT_WIDTH_IN = 6.3  # A4 with ~25 mm margins
PALETTE = ["#1b4f72", "#c0392b", "#117a65", "#b9770e", "#6c3483", "#5d6d7e", "#2e86c1", "#a04000"]


def apply():
    plt.rcParams.update({
        "font.family": "serif",
        "font.serif": ["CMU Serif", "Computer Modern Roman", "DejaVu Serif"],
        "mathtext.fontset": "cm",
        "font.size": 9,
        "axes.labelsize": 9,
        "axes.titlesize": 9,
        "legend.fontsize": 8,
        "xtick.labelsize": 8,
        "ytick.labelsize": 8,
        "axes.grid": True,
        "grid.alpha": 0.3,
        "grid.linewidth": 0.5,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "axes.prop_cycle": matplotlib.cycler(color=PALETTE),
        "lines.linewidth": 1.4,
        "figure.dpi": 150,
        "savefig.bbox": "tight",
        "savefig.pad_inches": 0.02,
        "pdf.fonttype": 42,
        "ps.fonttype": 42,
    })


def figure(width=1.0, aspect=0.62, **kw):
    """New figure `width` x text width wide, height = aspect x width."""
    apply()
    w = TEXT_WIDTH_IN * width
    return plt.subplots(figsize=(w, w * aspect), **kw)


ALSO_PNG = True  # PNG copies for the UI and quick looks; the PDFs are the thesis versions


def save(fig, path, dpi=None):
    """Save `path` (PDF: vector, rasterised artists at `dpi`) plus a PNG copy (`dpi`, default 200)."""
    from pathlib import Path

    fig.savefig(path, dpi=dpi or "figure")
    if ALSO_PNG and Path(path).suffix == ".pdf":
        fig.savefig(Path(path).with_suffix(".png"), dpi=dpi or 200)
    plt.close(fig)
    return path
