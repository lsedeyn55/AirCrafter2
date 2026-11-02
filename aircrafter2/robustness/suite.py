"""Post-optimality checks, run on a converged design (pragmatic, not full RDO, robust design optimisation).

sensitivities uses compute_totals; worst_cases perturbs one input at a time; monte_carlo samples the uncertain inputs.
They take a run problem from api.py or a study, and return plain dicts. Robustness inside the optimiser is in
rdo_problem.py. Details: docs/modules/robustness.md, "Post-optimality checks".
"""

import numpy as np


def _val(prob, name):
    """One scalar output of the problem."""
    return float(np.ravel(prob.get_val(name))[0])


def sensitivities(prob, of, wrt):
    """Normalised total derivatives (x / f) df/dx, from compute_totals (the same analytic and cs chain as the
    optimiser).

    Parameters
    ----------
    prob : om.Problem
    of : str
        The quantity f.
    wrt : list of str
        Promoted input or design variable names, each once.

    Returns
    -------
    dict
        {name: (x / f) df/dx} at the current point. Raises ValueError for duplicates or computed names.
    """
    if len(set(wrt)) != len(list(wrt)):
        raise ValueError(f"duplicate names in wrt: {sorted(n for n in set(wrt) if list(wrt).count(n) > 1)}")
    check_settable(prob, wrt)
    J = prob.compute_totals(of=[of], wrt=list(wrt), return_format="dict")[of]
    f = _val(prob, of)
    return {w: float(np.ravel(J[w])[0]) * _val(prob, w) / f for w in wrt}


def check_settable(prob, names):
    """Raise if any name is computed by the model: setting it would be silently overwritten by the next run.

    For example Rm and I0 when Kv is a design variable: perturb Rm_ref, Kv_factor instead.

    Parameters
    ----------
    prob : om.Problem
    names : iterable of str

    Raises ValueError.
    """
    res = prob.model._resolver
    computed = [n for n in names if res.is_prom(n, "output")]
    if computed:
        hint = " (Kv is a design variable: use Rm_ref / I0_ref / Kv_factor)" if set(computed) & {"Rm", "I0"} else ""
        raise ValueError(f"{computed} are computed by the model, not inputs; setting them has no effect{hint}")


def _snapshot(prob, outputs):
    """{output: value} at the current point."""
    return {k: _val(prob, k) for k in outputs}


def worst_cases(prob, outputs, mass_pct=10.0, thrust_pct=10.0, capacity_pct=10.0, headwind=3.0):
    """One-at-a-time perturbations, re-analysed with the full MDA. Restores the inputs afterwards.

    Empty mass + X % (ballast), thrust - Y % (lap model), battery capacity - Z %, headwind. Thrust is scaled in the lap
    model only: the MDA's level-speed balance keeps the nominal thrust, since that speed sets the stall margin and the
    structural design speed.

    Parameters
    ----------
    prob : om.Problem
    outputs : list of str
    mass_pct, thrust_pct, capacity_pct : float
        Percent.
    headwind : float
        m/s.

    Returns
    -------
    dict
        {case: {output: value}}, including 'nominal'.
    """
    prob.run_model()
    results = {"nominal": _snapshot(prob, outputs)}
    m_empty = _val(prob, "m_empty")
    cap = _val(prob, "capacity")
    cases = {
        f"empty mass +{mass_pct:g} %": [("m_ballast", 0.01 * mass_pct * m_empty, 0.0)],
        f"thrust -{thrust_pct:g} %": [("lap.thrust_factor", 1 - 0.01 * thrust_pct, 1.0)],
        f"capacity -{capacity_pct:g} %": [("capacity", cap * (1 - 0.01 * capacity_pct), cap)],
        f"headwind {headwind:g} m/s": [("lap.wind", headwind, 0.0)],
    }
    check_settable(prob, [name for changes in cases.values() for name, _, _ in changes])
    for label, changes in cases.items():
        for name, new, _ in changes:
            prob.set_val(name, new)
        prob.run_model()
        results[label] = _snapshot(prob, outputs)
        for name, _, old in changes:
            prob.set_val(name, old)
    prob.run_model()
    return results


def monte_carlo(prob, spreads, outputs, n=40, seed=0):
    """Random sampling of uncertain inputs, with a full MDA per sample. Run it on final candidates only.

    Parameters
    ----------
    prob : om.Problem
    spreads : dict or list of Dist
        {input: relative half-width}, each input ~ U(x (1 - s), x (1 + s)); or a list of uncertainty.Dist (e.g.
        resolve(load_uncertainty("configs/uncertainty.yaml"), ...)), sampled by inverse CDF.
    outputs : list of str
    n, seed : int
        Sample count and random seed.

    Returns
    -------
    dict
        samples = array (n, len(outputs)), names = outputs, failed = count.
    """
    rng = np.random.default_rng(seed)
    if isinstance(spreads, dict):
        names = list(spreads)
        draw = {k: (lambda u, k=k, s=s: nominal[k] * (1 + s * (2 * u - 1))) for k, s in spreads.items()}
    else:
        names = [d.name for d in spreads]
        draw = {d.name: d.ppf for d in spreads}
    check_settable(prob, names)
    nominal = {k: _val(prob, k) for k in names}
    rows, failed = [], 0
    for _ in range(n):
        for k in names:
            prob.set_val(k, draw[k](rng.uniform()))
        try:
            prob.run_model()
            rows.append([_val(prob, o) for o in outputs])
        except Exception:  # MDA failure counts as a failed sample
            failed += 1
    for k, v in nominal.items():
        prob.set_val(k, v)
    prob.run_model()
    return dict(samples=np.array(rows), names=list(outputs), failed=failed)
