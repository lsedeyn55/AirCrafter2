"""Robustness suite (pragmatic, not full RDO), run on a converged design.

sensitivities   normalised total derivatives (x / f) df/dx of the objective w.r.t. design variables and
                fixed assumptions, from OpenMDAO compute_totals (same analytic/cs chain as the optimiser).
worst_cases     one-at-a-time perturbations, re-analysed with the full MDA:
                  empty mass + X % (ballast), thrust - Y % (lap model), battery capacity - Z %, headwind.
                Thrust is scaled in the lap model only; the MDA's level-speed balance keeps the nominal thrust
                (that speed sets the stall margin and the structural design speed).
monte_carlo     random sampling of uncertain inputs (independent; relative uniform spreads, or the
                distributions of configs/uncertainty.yaml), full MDA per sample; returns objective statistics
                and constraint-violation probabilities. Run it on final candidates only (each sample is a
                full coupled analysis).

Robust design optimisation (robustness inside the optimiser) is in rdo_problem.py.
"""

import numpy as np


def _val(prob, name):
    return float(np.ravel(prob.get_val(name))[0])


def sensitivities(prob, of, wrt):
    """{name: (x / f) df/dx} at the current point; wrt are promoted input or DV names (each once)."""
    if len(set(wrt)) != len(list(wrt)):
        raise ValueError(f"duplicate names in wrt: {sorted(n for n in set(wrt) if list(wrt).count(n) > 1)}")
    check_settable(prob, wrt)
    J = prob.compute_totals(of=[of], wrt=list(wrt), return_format="dict")[of]
    f = _val(prob, of)
    return {w: float(np.ravel(J[w])[0]) * _val(prob, w) / f for w in wrt}


def check_settable(prob, names):
    """Raise if any name is computed by the model: setting it would be silently overwritten by the next run (e.g. Rm
    and I0 when Kv is a design variable: perturb Rm_ref, Kv_factor instead)."""
    res = prob.model._resolver
    computed = [n for n in names if res.is_prom(n, "output")]
    if computed:
        hint = " (Kv is a design variable: use Rm_ref / I0_ref / Kv_factor)" if set(computed) & {"Rm", "I0"} else ""
        raise ValueError(f"{computed} are computed by the model, not inputs; setting them has no effect{hint}")


def _snapshot(prob, outputs):
    return {k: _val(prob, k) for k in outputs}


def worst_cases(prob, outputs, mass_pct=10.0, thrust_pct=10.0, capacity_pct=10.0, headwind=3.0):
    """Returns {case: {output: value}} including 'nominal'. Restores the inputs afterwards."""
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
    """spreads: {input: relative half-width}, each input ~ U(x (1 - s), x (1 + s)); or a list of
    uncertainty.Dist (e.g. resolve(load_uncertainty("configs/uncertainty.yaml"), ...)), sampled by inverse CDF.

    Returns dict(samples=array (n, len(outputs)), names=outputs, failed=count).
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
