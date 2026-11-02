"""Robust design optimisation of the multipoint problem: N uncertainty nodes x 2 mission points.

Each node is problems/multipoint.py's model, with the uncertain parameters at the rule's values, the same at both
mission points (the same aircraft). As rdo_problem.py: fixed nodes, the UDR (univariate dimension reduction) rule by
default, RetryNewton at every point's MDA. Inputs the model computes (m_pay: the stores) are excluded: the fuel is
weighed at the field (rules p. 10), and the store masses are fixed. Details: docs/modules/robustness.md.
"""

import numpy as np
import openmdao.api as om

from ..problems.multipoint import (
    POINTS,
    add_dvs,
    apply_multipoint_values,
    build_multipoint_model,
    multipoint_constraints,
    set_dv_defaults,
)
from .quadrature import make_rule
from .stats import RobustStats, key_of
from .uncertainty import Dist, for_kv_mode, resolve

COMPUTED = {"m_pay"}  # model inputs in the single-point problem that the multipoint stores compute


def _value(prob, name):
    """One scalar output of the problem."""
    return float(np.ravel(prob.get_val(name))[0])


def mp_params(mcfg):
    """The uncertain parameters of the multipoint problem: the motor mode's, without the inputs the model computes.

    Parameters
    ----------
    mcfg : MultipointConfig

    Returns
    -------
    list of UncertainParam
    """
    from ..problems.full_mdo import kv_is_dv

    return [p for p in for_kv_mode(mcfg.base.robust.parameters, kv_is_dv(mcfg.base)) if p.name not in COMPUTED]


def resolved_dists_mp(mcfg, current_value):
    """The distributions of the multipoint problem.

    Parameters
    ----------
    mcfg : MultipointConfig
    current_value : callable
        current_value(name) supplies config values for nominal = None.

    Returns
    -------
    (list of Dist, list of Dist)
        All, and the active ones. Raises ValueError if robust.active names something unknown.
    """
    rob = mcfg.base.robust
    dists = resolve(mp_params(mcfg), current_value)
    names = [d.name for d in dists]
    active = names if rob.active is None else list(rob.active)
    unknown = sorted(set(active) - set(names))
    if unknown:
        raise ValueError(f"robust.active names {unknown} are not uncertain parameters of the multipoint problem")
    return dists, [d for d in dists if d.name in active]


def param_inputs(name, prefix=""):
    """The model inputs of one uncertain parameter: the same value at both mission points.

    Parameters
    ----------
    name : str
    prefix : str
        Path to the multipoint group.

    Returns
    -------
    list of str
    """
    return [f"{prefix}points.{p}.{name}" for p in POINTS]


def first_order_contributions_mp(prob, dists, of, prefix=""):
    """First-order variance contributions, with dq/dp summed over both points' inputs of the parameter.

    Parameters
    ----------
    prob : om.Problem
    dists : list of Dist
    of : list of str
    prefix : str

    Returns
    -------
    dict
        {quantity: {param: (dq/dp sigma)^2}}.
    """
    wrt = [w for d in dists for w in param_inputs(d.name, prefix)]
    J = prob.compute_totals(of=list(of), wrt=wrt, return_format="dict")
    return {
        q: {
            d.name: (sum(float(np.ravel(J[q][w])[0]) for w in param_inputs(d.name, prefix)) * d.std) ** 2 for d in dists
        }
        for q in of
    }


def robust_names(mcfg, extra=()):
    """The robust constraints: the config's robust list at both points, plus extra.

    Parameters
    ----------
    mcfg : MultipointConfig
    extra : iterable of str
        Absolute names.

    Returns
    -------
    list of str
    """
    names = []
    for n, _ in multipoint_constraints(mcfg):
        tail = n.split(".", 2)[-1] if n.startswith("points.") else None
        if tail in mcfg.base.robust.robust_constraints or n in extra:
            names.append(n)
    return names


def _template_dists(mcfg):
    """Distributions with the right kinds but placeholder numbers: the rule's size before setup."""
    act = mcfg.base.robust.active
    return [Dist(p.name, p.dist, 1.0, 0.1) for p in mp_params(mcfg) if act is None or p.name in act]


def build_robust_multipoint(mcfg, driver=True, extra_robust=()):
    """The robust multipoint problem.

        maximise   mu_F - k sigma_F                    (F = M1 + M2 + M3)
        subject to mu_g - beta sigma_g >= lower, mu_g + beta sigma_g <= upper   (robust constraints, both points)
                   g(x, p_bar) within its bounds at the reference node      (the others)

    Parameters
    ----------
    mcfg : MultipointConfig
    driver : bool
        Add the optimiser.
    extra_robust : iterable of str
        More robust constraints, by absolute name.

    Returns
    -------
    om.Problem
        Set up; prob.rdo = dict(rule, dists, active, values, solvers, quantities, N).
    """
    from ..robustness.rdo_problem import RetryNewton  # noqa: F401  (the MDAs are RetryNewton via mda_retry)

    rob = mcfg.base.robust
    template = make_rule(rob, _template_dists(mcfg))
    N = template.n
    cons = multipoint_constraints(mcfg)
    rnames = robust_names(mcfg, extra_robust)
    dvs = list(mcfg.all_dvs())

    prob = om.Problem(reports=False)
    m = prob.model
    nodes = m.add_subsystem("nodes", om.ParallelGroup(), promotes_inputs=dvs)
    for j in range(N):
        g = om.Group()
        build_multipoint_model(g, mcfg, mda_retry=True, defaults=False)
        nodes.add_subsystem(f"n{j}", g, promotes_inputs=dvs)
    set_dv_defaults(m, mcfg)

    ref_of = {n: kw.get("ref") or 1.0 for n, kw in cons}
    quantities = {"flight_score": rob.sigma_eps * 5.0}
    quantities.update({key_of(n): rob.sigma_eps * ref_of[n] for n in rnames})
    m.add_subsystem(
        "stats",
        RobustStats(rule=template, quantities=quantities, objective="flight_score", k=rob.k, sign=-1.0, beta=rob.beta),
    )
    for j in range(N):
        m.connect(f"nodes.n{j}.flight_score", f"stats.flight_score_n{j}")
        for n in rnames:
            m.connect(f"nodes.n{j}.{n}", f"stats.{key_of(n)}_n{j}")

    if driver:
        b = mcfg.base
        prob.driver = om.ScipyOptimizeDriver(optimizer="SLSQP", tol=b.tol, maxiter=b.maxiter, disp=True)
        add_dvs(m, mcfg)
        m.add_objective("stats.flight_score_robust", ref=-5.0)  # maximise mu - k sigma
        for name, kw in cons:
            if name in rnames:
                k = key_of(name)
                if kw.get("lower") is not None:
                    m.add_constraint(f"stats.{k}_lo", lower=kw["lower"], ref=kw.get("ref"))
                if kw.get("upper") is not None:
                    m.add_constraint(f"stats.{k}_hi", upper=kw["upper"], ref=kw.get("ref"))
            else:
                m.add_constraint(f"nodes.n0.{name}", **kw)
    prob.setup()

    for j in range(N):
        apply_multipoint_values(prob, mcfg, prefix=f"nodes.n{j}.")
    dists, act = resolved_dists_mp(mcfg, lambda n: _value(prob, f"nodes.n0.points.M2.{n}"))
    rule = make_rule(rob, act)
    m.stats.options["rule"] = rule
    base = {d.name: d.mean for d in dists}
    values = []
    for row in rule.nodes:
        vals = dict(base)
        vals.update({d.name: float(x) for d, x in zip(act, row)})
        values.append(vals)
    for j, vals in enumerate(values):
        for name, val in vals.items():
            for inp in param_inputs(name, f"nodes.n{j}."):
                prob.set_val(inp, val)
    solvers = {p: [getattr(getattr(nodes, f"n{j}").points, p).mda.nonlinear_solver for j in range(N)] for p in POINTS}
    for p in POINTS:
        for s in solvers[p][1:]:
            s.reference = solvers[p][0]
    prob.rdo = dict(
        rule=rule, dists=dists, active=act, values=values, solvers=solvers, quantities=["flight_score"] + rnames, N=N
    )
    return prob


def node_report_mp(prob):
    """Per point and node: solves, attempts and failed attempts of its MDA.

    Parameters
    ----------
    prob : om.Problem

    Returns
    -------
    list of dict
    """
    rows = []
    for p, ss in prob.rdo["solvers"].items():
        for j, s in enumerate(ss):
            fails = [e for e in s.log if not e[2]]
            rows.append(dict(point=p, node=j, solves=s.n_solve, attempts=len(s.log), failed_attempts=len(fails)))
    return rows


def robust_summary_mp(prob):
    """The statistics at the current point.

    Parameters
    ----------
    prob : om.Problem

    Returns
    -------
    dict
        {quantity: dict(mu, sigma, lo, hi[, robust])}.
    """
    out = {}
    for q in prob.rdo["quantities"]:
        k = key_of(q)
        d = {o: _value(prob, f"stats.{k}_{o}") for o in ("mu", "sigma", "lo", "hi")}
        if q == "flight_score":
            d["robust"] = _value(prob, "stats.flight_score_robust")
        out[q] = d
    return out
