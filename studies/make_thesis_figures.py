"""Generate the thesis figure set (vector PDFs) from the full model.

Optimises `configs/step7_full.yaml` (with the prop given, default 10x7E), then writes into
outputs/thesis_figures/:
- the standard figures: planform, airfoil, spanwise lift, drag and mass breakdowns, convergence, sensitivities;
- a 7x7 design-space surface of the objective over (b, c);
- the XDSM (.tex for LaTeX, .pdf quick look);
- the N2 diagram.

Run: .venv/bin/python studies/make_thesis_figures.py [prop]
"""

import sys
from pathlib import Path

from aircrafter2.api import run_design
from aircrafter2.problems.full_mdo import load_full_config

ROOT = Path(__file__).resolve().parents[1]


def main():
    cfg = load_full_config(ROOT / "configs" / "step7_full.yaml")
    cfg.propulsion.prop = sys.argv[1] if len(sys.argv) > 1 else "10x7E"
    out = ROOT / "outputs" / "thesis_figures"
    res = run_design(cfg, optimise=True, out_dir=out, figures=True, sensitivities=True, design_space=True,
                     progress=lambda s: print(f"[figures] {s} ...", flush=True))
    print(f"success = {res.success}; {len(res.files)} files in {out}")
    for f in res.files:
        print("  ", f)


if __name__ == "__main__":
    main()
