"""Twin config: does the coupled MDA converge across the design-variable bounds?

Samples random corners of the DV box, random interior points and the centre; runs the analysis
(no optimiser) from the config's initial guesses. Run: .venv/bin/python verification/twin/mda_sweep.py
"""

import time
import warnings
from pathlib import Path

import numpy as np

from aircrafter2.problems.full_mdo import build_full_problem, load_full_config

warnings.filterwarnings("ignore")
ROOT = Path(__file__).resolve().parents[2]


def _get(prob, k):
    try:
        return float(prob.get_val(k)[0])
    except KeyError:  # e.g. clearance margins on the single-motor model
        return float("nan")


def main(n_corners=16, n_random=8, seed=0, relief=False, cfg_name="full_mdo_twin_fixed_kv.yaml"):
    cfg = load_full_config(ROOT / "configs" / cfg_name)
    cfg.mass.options.motor_inertia_relief = relief
    dvs = cfg.design_variables
    lo = np.array([b.lower for b in dvs.values()])
    hi = np.array([b.upper for b in dvs.values()])
    rng = np.random.default_rng(seed)
    pts = [("centre", 0.5 * (lo + hi))]
    pts += [(f"corner {i}", np.where(rng.integers(0, 2, len(lo)) == 1, hi, lo)) for i in range(n_corners)]
    pts += [(f"random {i}", lo + rng.random(len(lo)) * (hi - lo)) for i in range(n_random)]
    rows = []
    for label, x in pts:
        prob = build_full_problem(cfg, driver=False)
        for name, val in zip(dvs, x):
            prob.set_val(name, val)
        t = time.perf_counter()
        try:
            prob.run_model()
            ok, err = True, ""
        except Exception as exc:  # non-convergence raises (err_on_non_converge)
            ok, err = False, f"{exc.__class__.__name__}: {str(exc)[:80]}"
        dt = time.perf_counter() - t
        its = prob.model.mda.nonlinear_solver._iter_count
        v = {k: _get(prob, k) for k in ("v", "m_total", "fus_clear_margin", "tip_clear_margin")}
        rows.append((label, ok, its, dt, v, err, dict(zip(dvs, x))))
        print(f"{label:10s} ok={ok} newton_its={its} {dt:5.1f}s v={v['v']:.2f} m={v['m_total']:.3f} "
              f"fus_clear={v['fus_clear_margin']:+.3f} {err}", flush=True)
        if not ok:
            print("    DVs:", {k: round(float(val), 4) for k, val in zip(dvs, x)}, flush=True)
    n_ok = sum(r[1] for r in rows)
    print(f"converged {n_ok}/{len(rows)}; max Newton iterations {max(r[2] for r in rows)}")
    return rows


if __name__ == "__main__":
    import sys

    main(cfg_name=sys.argv[1] if len(sys.argv) > 1 else "full_mdo_twin_fixed_kv.yaml")
