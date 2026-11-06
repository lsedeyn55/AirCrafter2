"""RDO plan checks: do the MDAs of all quadrature nodes converge?

Points: the config start point, the deterministic optimum, and the 25-point design-box sweep of
verification/twin/mda_sweep.py (centre, 16 random corners, 8 random interior points; seed 0), single 10x7E 4S.
At every point and every node of the UDR rule over the screened parameters (configs/full_mdo_rdo.yaml):
  cold   from the config's initial guesses (as a fresh analysis)
  warm   from the nominal node's converged state (how the robust model initialises the off-nominal nodes)
MDA non-convergence is reported separately from constraint values.

Writes outputs/rdo/node_check.json. Run: .venv/bin/python studies/rdo_node_check.py
"""

import os

os.environ.setdefault("OMP_NUM_THREADS", "1")  # one thread per worker process
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")

import json  # noqa: E402
import time  # noqa: E402
import warnings  # noqa: E402
from multiprocessing import Pool  # noqa: E402
from pathlib import Path  # noqa: E402

import numpy as np  # noqa: E402

warnings.filterwarnings("ignore")
ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "outputs" / "rdo"
CFG = ROOT / "configs" / "full_mdo_rdo_fixed_kv.yaml"  # pinned: docs/development/RDO_PLAN.md (fixed motor)


def v(p, k):
    return float(np.ravel(p.get_val(k))[0])


def _setup():
    from aircrafter2.problems.full_mdo import build_full_problem, load_full_config
    from aircrafter2.robustness.rdo_problem import node_values

    cfg = load_full_config(CFG)
    cfg.propulsion.prop = os.environ.get("RDO_PROP", cfg.propulsion.prop)  # validation runs
    prob = build_full_problem(cfg, driver=False)
    prob.final_setup()  # creates the vectors; keeps the config values
    return cfg, prob, node_values(cfg, prob)


def check_point(args):
    label, x = args
    warnings.filterwarnings("ignore")
    cfg, prob, (rule, values) = _setup()
    cold0 = prob.model._outputs.asarray(copy=True)
    rows = []
    nominal_state = None
    for mode in ("cold", "warm"):
        for j, vals in enumerate(values):
            if mode == "warm" and nominal_state is None:
                rows.append(dict(node=j, mode=mode, ok=None, err="nominal node failed"))
                continue
            # the output vector includes the auto-IVC outputs (the inputs), so set the DVs after restoring it
            prob.model._outputs.set_val(cold0 if mode == "cold" else nominal_state)
            for name, val in zip(cfg.design_variables, x):
                prob.set_val(name, val)
            for name, val in vals.items():
                prob.set_val(name, val)
            t = time.perf_counter()
            try:
                prob.run_model()
                ok, err = True, ""
            except Exception as exc:  # non-convergence raises (err_on_non_converge)
                ok, err = False, f"{exc.__class__.__name__}: {str(exc)[:100]}"
            its = prob.model.mda.nonlinear_solver._iter_count
            rows.append(dict(node=j, mode=mode, ok=ok, its=its, time=time.perf_counter() - t, err=err,
                             **({k: v(prob, k) for k in ("ThreeLapTime", "energy_margin", "SM", "v", "m_total")}
                                if ok else {})))
            if mode == "cold" and j == 0 and ok:
                nominal_state = prob.model._outputs.asarray(copy=True)
    return dict(label=label, x=[float(a) for a in x], rows=rows)


def main():
    from aircrafter2.problems.full_mdo import load_full_config

    OUT.mkdir(parents=True, exist_ok=True)
    cfg = load_full_config(CFG)
    dvs = cfg.design_variables
    lo = np.array([b.lower for b in dvs.values()])
    hi = np.array([b.upper for b in dvs.values()])
    rng = np.random.default_rng(0)  # same points as verification/twin/mda_sweep.py
    pts = [("start", np.array([cfg.initial.get(k, 0.5 * (b.lower + b.upper)) for k, b in dvs.items()]))]
    det = json.loads((OUT / "screening.json").read_text())["dv"]
    pts.append(("det. optimum", np.array([det[k] for k in dvs])))
    pts.append(("centre", 0.5 * (lo + hi)))
    pts += [(f"corner {i}", np.where(rng.integers(0, 2, len(lo)) == 1, hi, lo)) for i in range(16)]
    pts += [(f"random {i}", lo + rng.random(len(lo)) * (hi - lo)) for i in range(8)]
    t = time.perf_counter()
    workers = int(os.environ.get("RDO_WORKERS", "5"))  # ~1 GB per worker; 14 GB machine
    with Pool(min(workers, len(pts))) as pool:
        res = pool.map(check_point, pts)
    wall = time.perf_counter() - t
    (OUT / "node_check.json").write_text(json.dumps(dict(points=res, wall=wall, dv_names=list(dvs)), indent=1))
    for r in res:
        cold = [x for x in r["rows"] if x["mode"] == "cold"]
        warm = [x for x in r["rows"] if x["mode"] == "warm"]
        fc = [x["node"] for x in cold if not x["ok"]]
        fw = [x["node"] for x in warm if x["ok"] is False]
        print(f"{r['label']:13s} cold ok {len(cold) - len(fc)}/{len(cold)} {fc or ''}  "
              f"warm ok {sum(1 for x in warm if x['ok'])}/{len(warm)} {fw or ''}"
              f"{'  (nominal failed: no warm start)' if warm and warm[0]['ok'] is None else ''}")
    print(f"wall {wall:.0f} s")


if __name__ == "__main__":
    main()
