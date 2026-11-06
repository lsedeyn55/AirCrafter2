"""Q45: step resolution of lap 1's first straight near the wind limit (docs/development/RESULTS_2026_PLAN.md §4.6).

Lap-1 time (and the lap-2+ time, a control) at the default 40 steps against a uniform 640-step reference, for the
uniform grid (the old one, lap1_cluster = 1) and the clustered grids (s_i = L (i/N)^p, p = 2, 3), at winds 0-12 m/s (failures past the validity edge are NaN),
take-off off and on, for a light design (the calibrated 2024-25 multipoint optimum, its M3 point), a heavy one (the
same case with 2 L bottles, its M2 and M3 points) and AirCrafter 1.0 completed's M3 point (Q45's worst published case).
Analysis only, at each design's calibrated 2024-25 variables.

    .venv/bin/python studies/q45_resolution.py            # writes outputs/results_2026/q45_resolution.json
"""

import json
import warnings
from multiprocessing import get_context
from pathlib import Path

import numpy as np

warnings.filterwarnings("ignore")
ROOT = Path(__file__).resolve().parents[1]
RUNS = ROOT / "outputs" / "results" / "runs"
OUT = ROOT / "outputs" / "results_2026"
WINDS = tuple(np.round(np.arange(0.0, 12.01, 0.5), 2))  # up to and past the light designs' V_stall
GRIDS = (1.0, 2.0, 3.0)
DESIGNS = {"light (2024-25 multipoint, M3)": ("mp_6S_11x8E_10x7E", "M3"),
           "heavy (2024-25 multipoint 2 L, M2)": ("mp_6S_11x8E_10x7E_bottle2L", "M2"),
           "heavy (2024-25 multipoint 2 L, M3)": ("mp_6S_11x8E_10x7E_bottle2L", "M3"),
           "ac1_completed (M3; Q45's +19 % case)": ("ac1_completed", "M3")}


def _job(job):
    import sys

    sys.path.insert(0, str(ROOT / "studies"))
    from openmdao.core.analysis_error import AnalysisError  # noqa: F401
    from results_2025 import config
    from takeoff_study import _takeoff_block

    from aircrafter2.problems.multipoint import build_multipoint_problem

    label, tag, pt, setting = job
    d = json.loads((RUNS / f"{tag}.json").read_text())
    mcfg = config(d["cells"], d["props"]["M2"], d["props"]["M3"], bottle=d.get("bottle"))
    if setting == "on":
        mcfg.base.course = _takeoff_block()
    p = build_multipoint_problem(mcfg, driver=False, mda_retry=True)
    for k, x in d["dv"].items():
        p.set_val(k, x)
    p.run_model()
    comp = p.model._get_subsystem(f"points.{pt}.lap")
    base = {k: np.array(comp._inputs[k], dtype=float) for k in comp._inputs}
    vstall = float(np.real(comp._model(base)["Vstall"][0]))
    rows = []
    for w in WINDS:
        row = dict(wind=float(w))

        def lap(n, c):
            comp.options["n_steps_straight"] = n
            comp.options["lap1_cluster"] = c
            with np.errstate(all="ignore"):
                r = comp._model(dict(base, wind=np.array([w])))
            return float(np.real(np.ravel(r["Lap1time"])[0])), float(np.real(np.ravel(r["otherLapTime"])[0]))

        ref, ref_other = lap(640, 1.0)
        row.update(lap1_640=ref, other_640=ref_other)
        for c in GRIDS:
            t1, t2 = lap(40, c)
            row[f"lap1_p{c:g}"] = t1
            row[f"err_p{c:g}"] = t1 / ref - 1.0 if np.isfinite(ref) and ref > 0 else float("nan")
            row[f"err_other_p{c:g}"] = t2 / ref_other - 1.0 if np.isfinite(ref_other) else float("nan")
        rows.append(row)
    comp.options["n_steps_straight"] = 40
    return label, setting, dict(tag=tag, point=pt, V_stall=vstall, rows=rows)


def main(workers=4):
    jobs = [(lab, tag, pt, s) for lab, (tag, pt) in DESIGNS.items() for s in ("off", "on")]
    out = {}
    with get_context("forkserver").Pool(workers) as pool:
        for label, setting, res in pool.imap_unordered(_job, jobs):
            out.setdefault(label, {})[setting] = res
            print(label, setting, f"V_stall {res['V_stall']:.2f}",
                  [(r["wind"], *(round(100 * r[f"err_p{c:g}"], 3) for c in GRIDS)) for r in res["rows"]], flush=True)
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "q45_resolution.json").write_text(json.dumps(out, indent=1, default=float))


if __name__ == "__main__":
    main()
