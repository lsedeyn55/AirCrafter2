"""One call from a saved run to the OpenVSP mock-up: build, consistency check, figures.

Used by `aircrafter2 vsp <run_dir>`, `aircrafter2 run <cfg> --vsp`, the Streamlit UI and examples/vsp_mockup.py.
"""

from pathlib import Path

from ..geometry.export import load_geometry


def build_run(run_dir, out_dir=None, name=None, figures=True, incidence=None):
    """run_dir holds geometry.json (or is the file itself). Writes into out_dir (default <run_dir>/vsp/)."""
    from . import render as R
    from .build import build
    from .check import vsp_check

    src = Path(run_dir)
    geom = load_geometry(src)
    run = src if src.is_dir() else src.parent
    out = Path(out_dir) if out_dir else run / "vsp"
    name = name or geom["provenance"].get("case_id") or run.name
    res = build(geom, out, name, incidence=incidence)
    chk = vsp_check(geom, res["files"]["vsp3"], out / "vsp_check.md")
    res["files"]["check"] = out / "vsp_check.md"
    res["check"] = chk
    res["geom"] = geom
    if figures:
        meshes = R.load_meshes(res["files"]["component_stls"])
        res["meshes"] = meshes
        res["files"]["three_view"] = R.three_view(geom, meshes, out / f"{name}_three_view.pdf")
        res["files"]["isometric"] = R.isometric(geom, meshes, out / f"{name}_isometric.pdf")
    return res
