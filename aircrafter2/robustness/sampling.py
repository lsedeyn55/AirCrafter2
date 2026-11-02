"""Parallel evaluation of the deterministic model at many parameter points (Monte Carlo, quadrature checks).

Each worker process builds the deterministic problem once, sets the design, converges the nominal point, and runs its
share of the samples, warm-starting each from the previous converged state. Called by the study scripts. With mda_robust
it goes through problems.multipoint.run_warm (docs/DECISIONS.md#ff-d5). Details: docs/modules/robustness.md.

Memory: one problem is ~0.4-2 GB, so keep `workers` small (5 on a 14 GB machine).
"""

import os
import time
from multiprocessing import get_context

import numpy as np

_W = {}


def _value(prob, name):
    """One scalar output of the problem."""
    return float(np.ravel(prob.get_val(name))[0])


def _init(cfg, dv, outputs):
    """Worker start-up: build the problem and converge the nominal point; prob = None if that fails."""
    import warnings

    warnings.filterwarnings("ignore")
    from ..problems.full_mdo import FullMDOConfig, build_full_problem

    if isinstance(cfg, FullMDOConfig):
        prob = build_full_problem(cfg, driver=False)
        names = lambda k: [k]  # noqa: E731
        robust = False
    else:  # a MultipointConfig: each parameter at both mission points (the same aircraft)
        from ..problems.multipoint import build_multipoint_problem
        from .rdo_multipoint import param_inputs

        robust = cfg.base.mda_robust  # the 2025-26 problem: RetryNewton robust mode, warm path to the design
        prob = build_multipoint_problem(cfg, driver=False, mda_retry=robust)
        names = param_inputs
    try:
        if robust:
            from ..problems.multipoint import run_warm

            run_warm(prob, cfg, dv)
        else:
            for k, v in dv.items():
                prob.set_val(k, v)
            prob.run_model()
    except Exception:  # the nominal point itself does not converge: every sample is MDA non-convergence (an
        _W.update(prob=None, outputs=outputs)  # initializer that raises makes the pool respawn workers forever)
        return
    _W.update(
        prob=prob,
        outputs=outputs,
        nominal=prob.model._outputs.asarray(copy=True),
        inputs={k: _value(prob, k) for k in dv},
        names=names,
    )


def _restore_nominal():
    """Put the worker's problem back at the nominal converged state."""
    prob = _W["prob"]
    prob.model._outputs.set_val(_W["nominal"])  # includes the auto-IVC outputs: reset the design too
    for k, v in _W["inputs"].items():
        prob.set_val(k, v)


def _run(task):
    """Run one sample: (index, output values, how).

    how is "warm", "nominal" (retried once from the nominal state), or an analysis failure: "lap_invalid" (the
    aircraft's wind limit, a headwind near V_stall: it can't make progress into the wind and wouldn't fly,
    mission/smooth_lap.py) or "failed" (MDA non-convergence). A failure's row is NaN, never a constraint violation.
    """
    i, vals = task
    prob = _W["prob"]
    if prob is None:
        return i, [np.nan] * len(_W["outputs"]), "failed"
    msg = ""
    for attempt in ("warm", "nominal"):
        if attempt == "nominal":
            _restore_nominal()
        for k, v in vals.items():
            for name in _W["names"](k):
                prob.set_val(name, v)
        try:
            prob.run_model()
            return i, [_value(prob, o) for o in _W["outputs"]], attempt
        except Exception as exc:  # MDA non-convergence (AnalysisError), NaN residuals, or a non-physical lap
            msg = str(exc)
            continue
    _restore_nominal()
    return i, [np.nan] * len(_W["outputs"]), "lap_invalid" if "non-physical lap" in msg else "failed"


def evaluate(cfg, dv, rows, outputs, workers=5, chunksize=None):
    """Run the deterministic model at a design for each parameter point, in parallel.

    Parameters
    ----------
    cfg : FullMDOConfig or MultipointConfig
        With a MultipointConfig, each parameter is set at both mission points.
    dv : dict
        {design variable: value}.
    rows : list of dict
        {param: value} per sample.
    outputs : list of str
    workers : int
    chunksize : int, optional

    Returns
    -------
    dict
        values = (n, len(outputs)) array with NaN rows for analysis failures, failed = indices of every analysis
        failure, lap_invalid = the subset outside the lap model's validity (the rest are MDA non-convergence), retried =
        indices recovered from the nominal state, wall [s], workers.
    """
    os.environ.setdefault("OMP_NUM_THREADS", "1")
    n = len(rows)
    chunksize = chunksize or max(1, n // (4 * workers))
    t = time.perf_counter()
    with get_context("forkserver").Pool(workers, initializer=_init, initargs=(cfg, dv, list(outputs))) as pool:
        res = list(pool.imap_unordered(_run, list(enumerate(rows)), chunksize=chunksize))
    wall = time.perf_counter() - t
    values = np.full((n, len(outputs)), np.nan)
    failed, retried, invalid = [], [], []
    for i, row, how in res:
        values[i] = row
        if how in ("failed", "lap_invalid"):
            failed.append(i)
            if how == "lap_invalid":
                invalid.append(i)
        elif how == "nominal":
            retried.append(i)
    return dict(
        values=values,
        failed=sorted(failed),
        lap_invalid=sorted(invalid),
        retried=sorted(retried),
        wall=wall,
        workers=workers,
    )


def violation_probabilities(values, outputs, constraints, rtol=1e-6):
    """P(violation) per constraint over the converged rows.

    A sample violates a bound by more than rtol * max(|bound|, ref, 1): an active constraint that no uncertain parameter
    moves (e.g. V_h at its lower bound) sits on the bound to the optimiser's tolerance and isn't counted.

    Parameters
    ----------
    values : array (n, len(outputs))
    outputs : list of str
    constraints : list of (str, dict)
        full_constraints.
    rtol : float

    Returns
    -------
    dict
        {name: probability}.
    """
    idx = {o: j for j, o in enumerate(outputs)}
    ok = ~np.isnan(values).any(axis=1)
    out = {}
    for name, kw in constraints:
        if name not in idx:
            continue
        col = values[ok, idx[name]]
        bad = np.zeros(len(col), bool)
        for side, sgn in (("lower", -1.0), ("upper", 1.0)):
            bound = kw.get(side)
            if bound is not None:
                tol = rtol * max(abs(bound), abs(kw.get("ref") or 0.0), 1.0)
                bad |= sgn * (col - bound) > tol
        out[name] = float(bad.mean()) if len(col) else float("nan")
    return out
