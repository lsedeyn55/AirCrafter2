"""Robust design optimisation (RDO) of the full MDF problem: a multi-point model at fixed uncertainty nodes.

    model
      nodes  (om.ParallelGroup; serial without MPI)
        n0 .. n{N-1}   build_full_model(...) each, with the design variables promoted and shared;
                       node j has the uncertain parameters at the rule's node j (n0 = the reference point,
                       every parameter at its mean); its own score component
      stats  (RobustStats): node outputs -> mu, sigma, mu + k sigma, mu -/+ beta sigma

    minimise   mu_f + k sigma_f                       (f: the scoring plug-in output; mu - k sigma if maximised)
               or mu_f subject to sigma_f <= sigma_cap (epsilon-constraint)
    w.r.t.     the design variables of the deterministic problem
    subject to mu_g - beta sigma_g >= lower, mu_g + beta sigma_g <= upper   (robust constraints)
               g(x, p_bar) within its bounds at the reference node           (the other constraints)

Fixed nodes (common random numbers) make the objective a deterministic, smooth function of the design, and
the totals come through each node's coupled derivatives (DirectSolver on every MDA), as in the deterministic
problem. Each node's Newton solver is a RetryNewton: it warm-starts from the node's last converged state,
starts an off-reference node from the reference node's state, and falls back to those states (then to the cold
initial state) before declaring MDA non-convergence.
"""

import numpy as np
import openmdao.api as om
from openmdao.core.analysis_error import AnalysisError

from .quadrature import make_rule
from .stats import RobustStats, key_of
from .uncertainty import Dist, for_kv_mode, resolve


def _value(prob, name):
    return float(np.ravel(prob.get_val(name))[0])


def resolved_dists(cfg, current_value):
    """(all Dists, active Dists) of cfg.robust; current_value(name) supplies config values for nominal = None."""
    from ..problems.full_mdo import kv_is_dv

    rob = cfg.robust
    dists = resolve(for_kv_mode(rob.parameters, kv_is_dv(cfg)), current_value)
    names = [d.name for d in dists]
    active = names if rob.active is None else list(rob.active)
    unknown = sorted(set(active) - set(names))
    if unknown:
        raise ValueError(f"robust.active names {unknown} are not in robust.parameters")
    return dists, [d for d in dists if d.name in active]


def _node_table(rule, dists, active):
    base = {d.name: d.mean for d in dists}
    values = []
    for row in rule.nodes:
        vals = dict(base)
        vals.update({d.name: float(x) for d, x in zip(active, row)})
        values.append(vals)
    return values


def node_values(cfg, prob):
    """(rule, [{param: value} per node]) for a deterministic problem `prob` holding the config values.

    Active parameters take the rule's node values; the inactive ones are held at their means.
    """
    dists, act = resolved_dists(cfg, lambda n: _value(prob, n))
    rule = make_rule(cfg.robust, act)
    return rule, _node_table(rule, dists, act)


class RetryNewton(om.NewtonSolver):
    """Newton for a robust node, with warm starts and fallbacks before MDA non-convergence is declared.

    Start: the node's current state (its last converged one), or, for a node that has never converged, the
    reference node's converged state. Every attempt is logged in ``self.log`` as (solve number, start, converged).

    robust=False (the default; every config with ``mda_robust: false``): on failure, retry from the node's last
    converged state, the reference node's last converged state, then the cold initial state (as after setup).

    robust=True (``mda_robust: true``, docs/MDA_ROBUSTNESS_PLAN.md): on failure, retry from the node's last converged
    state (skipped when it equals the state the solve started from), then by continuation, then from the reference
    node's state, then cold. The continuation steps the MDA's input vector linearly from the last converged inputs
    (``good_in``; the reference node's for a node that has never converged) to the target in 4 steps (t = 1/4 .. 1, the
    last step the target exactly), each solve warm-started from the previous one; if a step fails, once more in 8
    steps (log labels "continuation 4", "continuation 8"). The interpolated inputs include upstream outputs such as
    the T(v) fit coefficients, so the intermediate states are smooth but not physical: only t = 1 is the answer. It
    takes Newton across a design step that leaves the previous state outside the new root's basin (the low-speed M3
    banner designs, whose T = D balance has a second, sub-stall root). Under ``mda_robust`` the MDA's linear solver is a
    QuietDirectSolver, so a diverged attempt that reaches a singular Jacobian fails as an AnalysisError too.
    """

    CONTINUATION_STEPS = (4, 8)

    def __init__(self, robust=False, **kwargs):
        super().__init__(**kwargs)
        self.robust = robust
        self.reference = None  # the reference node's RetryNewton (set by build_robust_problem)
        self.good = None  # last converged output vector of this MDA
        self.good_in = None  # its input vector (stored in both modes; read by the continuation only)
        self.cold = None
        self.log = []
        self.n_solve = 0

    def _converged(self, system):
        self.good = system._outputs.asarray(copy=True)
        self.good_in = system._inputs.asarray(copy=True)

    def solve(self):
        if self.robust:
            return self._solve_robust()
        system = self._system()
        out = system._outputs
        if self.cold is None:
            self.cold = out.asarray(copy=True)
        self.n_solve += 1
        ref_good = self.reference.good if self.reference is not None else None
        starts = [("current", None)]
        if self.good is None and ref_good is not None:  # first solve of an off-reference node
            starts = [("reference", ref_good)]
        starts += [("own last converged", self.good), ("reference", ref_good), ("cold", self.cold)]
        tried, err = set(), None
        for label, vec in starts:
            if label in tried or (label != "current" and vec is None):
                continue
            tried.add(label)
            if vec is not None:
                out.set_val(vec)
            try:
                super().solve()
            except AnalysisError as exc:
                self.log.append((self.n_solve, label, False))
                err = exc
                continue
            self.log.append((self.n_solve, label, True))
            self._converged(system)
            return
        raise err

    def _attempt(self, system, label, vec):
        """One Newton solve from vec (None: the current state); True if it converged."""
        if vec is not None:
            system._outputs.set_val(vec)
        try:
            super().solve()
        except AnalysisError as exc:
            self.log.append((self.n_solve, label, False))
            self._err = exc
            return False
        self.log.append((self.n_solve, label, True))
        self._converged(system)
        return True

    def _continuation(self, system, n_steps, start_in, start_out):
        """Step the input vector from start_in to the target in n_steps, each solve warm-started from the last."""
        inp, out = system._inputs, system._outputs
        target = inp.asarray(copy=True)
        out.set_val(start_out)
        ok = False
        try:
            for i in range(1, n_steps + 1):
                inp.set_val(target if i == n_steps else start_in + (i / n_steps) * (target - start_in))
                super().solve()
            ok = True
        except AnalysisError as exc:
            self._err = exc
        finally:
            if not ok:  # on success the last step set the target exactly; the solve moves only internal inputs
                inp.set_val(target)
        self.log.append((self.n_solve, f"continuation {n_steps}", ok))
        if ok:
            self._converged(system)
        return ok

    def _solve_robust(self):
        system = self._system()
        out = system._outputs
        if self.cold is None:
            self.cold = out.asarray(copy=True)
        self.n_solve += 1
        self._err = None
        ref = self.reference
        ref_good = ref.good if ref is not None else None
        current = out.asarray(copy=True)
        if self.good is None and ref_good is not None:  # first solve of an off-reference node
            first = ("reference", ref_good)
        else:
            first = ("current", None)
        if self._attempt(system, *first):
            return
        tried = {first[0]}
        if self.good is not None and not np.array_equal(self.good, current):
            if self._attempt(system, "own last converged", self.good):
                return
        if self.good is not None:
            cont = (self.good_in, self.good)
        elif ref is not None and ref.good is not None:
            cont = (ref.good_in, ref.good)
        else:
            cont = None
        if cont is not None and cont[0] is not None:
            for n in self.CONTINUATION_STEPS:
                if self._continuation(system, n, cont[0].copy(), cont[1].copy()):
                    return
        for label, vec in (("reference", ref_good), ("cold", self.cold)):
            if label in tried or vec is None:
                continue
            tried.add(label)
            if self._attempt(system, label, vec):
                return
        raise self._err


class QuietDirectSolver(om.DirectSolver):
    """DirectSolver for an MDA under ``mda_robust``: a singular or NaN Jacobian raises AnalysisError at once.

    OpenMDAO's DirectSolver raises RuntimeError and first builds a diagnosis (a dense SVD of the Jacobian), which on a
    diverged Newton iterate of the 2025-26 MDA takes about 10 minutes and several GB. Here the diagnosis is a short
    message, and the error is an AnalysisError, so RetryNewton counts it as a failed attempt and falls back
    (docs/MDA_ROBUSTNESS_PLAN.md, status). A converged state is never singular, so results are unchanged.
    """

    def _linearize(self):
        from openmdao.solvers.linear import direct

        fmt = direct.format_singular_error
        direct.format_singular_error = lambda system, matrix: f"singular Jacobian in '{system.pathname}'"
        try:
            super()._linearize()
        except RuntimeError as exc:
            raise AnalysisError(str(exc)) from exc
        finally:
            direct.format_singular_error = fmt


class _NodeSetter:
    """prob.set_val / get_val with node-local names prefixed; design variables are shared (top level)."""

    def __init__(self, prob, prefix, shared):
        self.prob, self.prefix, self.shared = prob, prefix, set(shared)

    def _name(self, name):
        return name if name in self.shared else self.prefix + name

    def set_val(self, name, val, units=None, indices=None):
        self.prob.set_val(self._name(name), val, units=units, indices=indices)

    def get_val(self, name, units=None, indices=None):
        return self.prob.get_val(self._name(name), units=units, indices=indices)


def _template_dists(cfg):
    """Distributions with the right kinds but placeholder numbers: the rule's size before setup."""
    from ..problems.full_mdo import kv_is_dv

    act = cfg.robust.active
    return [
        Dist(p.name, p.dist, 1.0, 0.1)
        for p in for_kv_mode(cfg.robust.parameters, kv_is_dv(cfg))
        if act is None or p.name in act
    ]


def build_robust_problem(cfg, driver=True):
    """The multi-point robust problem of a FullMDOConfig with an enabled ``robust:`` block.

    Returns the set-up problem; ``prob.rdo`` holds dict(rule, dists, active, values, solvers, quantities).
    """
    from ..problems.full_mdo import (
        DV_REF,
        DV_UNITS,
        add_full_driver,
        apply_full_values,
        build_full_model,
        check_kv_robust,
        full_constraints,
    )
    from ..scoring import get_plugin

    rob = cfg.robust
    check_kv_robust(cfg)  # again: the UI edits a validated config
    if rob.dv_tolerances:
        raise NotImplementedError(
            "robust.dv_tolerances (manufacturing tolerances on design variables) is a hook for later; leave it empty"
        )
    if driver and rob.objective == "mean_sigma_cap" and rob.sigma_cap is None:
        raise ValueError("robust.objective = mean_sigma_cap needs robust.sigma_cap")
    del add_full_driver  # the robust driver is set up below; the deterministic one is untouched
    plugin = get_plugin(cfg.scoring.plugin, **cfg.scoring.options)
    N = make_rule(rob, _template_dists(cfg)).n
    dvs = list(cfg.design_variables)
    cons = full_constraints(cfg)
    robust_names = [n for n, _ in cons if n in rob.robust_constraints]
    missing = sorted(set(rob.robust_constraints) - {n for n, _ in cons})
    if missing:
        raise ValueError(f"robust.robust_constraints {missing} are not constraints of this problem")

    prob = om.Problem(reports=False)
    m = prob.model
    nodes = m.add_subsystem("nodes", om.ParallelGroup(), promotes_inputs=dvs)
    for j in range(N):
        g = om.Group()
        build_full_model(g, cfg, mda_retry=True)
        g.add_subsystem("scoring", plugin.component(), promotes_inputs=list(plugin.inputs), promotes_outputs=["score"])
        nodes.add_subsystem(f"n{j}", g, promotes_inputs=dvs)
    for name in dvs:  # start: cfg.initial, else the reference motor's Kv, else the middle of the bounds
        bd = cfg.design_variables[name]
        start = cfg.propulsion.Kv if name == "Kv" else 0.5 * (bd.lower + bd.upper)
        m.set_input_defaults(name, cfg.initial.get(name, start), units=DV_UNITS.get(name))

    ref_of = {n: kw.get("ref") or 1.0 for n, kw in cons}
    quantities = {"score": rob.sigma_eps * plugin.ref}
    quantities.update({key_of(n): rob.sigma_eps * ref_of[n] for n in robust_names})
    sign = 1.0 if plugin.sense == "min" else -1.0
    m.add_subsystem(
        "stats",
        RobustStats(
            rule=make_rule(rob, _template_dists(cfg)),
            quantities=quantities,
            objective="score",
            k=rob.k,
            sign=sign,
            beta=rob.beta,
        ),
    )
    for j in range(N):
        m.connect(f"nodes.n{j}.score", f"stats.score_n{j}")
        for n in robust_names:
            m.connect(f"nodes.n{j}.{n}", f"stats.{key_of(n)}_n{j}")

    if driver:
        if rob.optimizer == "ipopt":
            prob.driver = om.pyOptSparseDriver(optimizer="IPOPT", print_results=False)
            prob.driver.opt_settings.update(
                {
                    "tol": cfg.tol,
                    "max_iter": cfg.maxiter,
                    "print_level": 3,
                    "hessian_approximation": "limited-memory",
                    "nlp_scaling_method": "gradient-based",
                }
            )
        else:
            prob.driver = om.ScipyOptimizeDriver(optimizer="SLSQP", tol=cfg.tol, maxiter=cfg.maxiter, disp=True)
        for name, bd in cfg.design_variables.items():
            m.add_design_var(name, lower=bd.lower, upper=bd.upper, units=DV_UNITS.get(name), ref=DV_REF.get(name))
        oref = plugin.ref if plugin.sense == "min" else -plugin.ref
        m.add_objective("stats.score_robust" if rob.objective == "mean_plus_k_sigma" else "stats.score_mu", ref=oref)
        for name, kw in robust_constraints(cfg, cons, robust_names, plugin):
            m.add_constraint(name, **kw)
    prob.setup()

    for j in range(N):
        apply_full_values(_NodeSetter(prob, f"nodes.n{j}.", dvs), cfg)
    dists, act = resolved_dists(cfg, lambda n: _value(prob, f"nodes.n0.{n}"))
    rule = make_rule(rob, act)
    m.stats.options["rule"] = rule  # same size and weights as the template rule (weights do not depend on values)
    values = _node_table(rule, dists, act)
    for j, vals in enumerate(values):
        for name, val in vals.items():
            prob.set_val(f"nodes.n{j}.{name}", val)
    solvers = [getattr(nodes, f"n{j}").mda.nonlinear_solver for j in range(N)]
    for s in solvers[1:]:
        s.reference = solvers[0]
    prob.rdo = dict(
        rule=rule, dists=dists, active=act, values=values, solvers=solvers, quantities=["score"] + robust_names, N=N
    )
    return prob


def robust_constraints(cfg, cons, robust_names, plugin):
    """[(name, add_constraint kwargs)] of the robust driver, in order: the sigma cap (mean_sigma_cap), then each
    constraint as mu -/+ beta sigma (robust_names) or at the reference node n0, then the plug-in's at n0."""
    rob = cfg.robust
    out = []
    if rob.objective != "mean_plus_k_sigma":
        out.append(("stats.score_sigma", dict(upper=rob.sigma_cap, ref=plugin.ref)))
    for name, kw in cons:
        if name in robust_names:
            k = key_of(name)
            ref = kw.get("ref")
            if kw.get("lower") is not None:
                out.append((f"stats.{k}_lo", dict(lower=kw["lower"], ref=ref)))
            if kw.get("upper") is not None:
                out.append((f"stats.{k}_hi", dict(upper=kw["upper"], ref=ref)))
        else:  # deterministic, at the reference node
            out.append((f"nodes.n0.{name}", kw))
    out += [(f"nodes.n0.{name}", kw) for name, kw in plugin.constraints()]
    return out


def node_report(prob):
    """Per node: solves, fallback attempts and failures of its MDA (for the reports)."""
    rows = []
    for j, s in enumerate(prob.rdo["solvers"]):
        fails = [e for e in s.log if not e[2]]
        rows.append(
            dict(
                node=j,
                solves=s.n_solve,
                attempts=len(s.log),
                failed_attempts=len(fails),
                recovered=len({e[0] for e in fails} & {e[0] for e in s.log if e[2]}),
            )
        )
    return rows


def robust_summary(prob):
    """{quantity: dict(mu, sigma, lo, hi[, robust], nodes=[...])} at the current point."""
    out = {}
    for q in prob.rdo["quantities"]:
        k = key_of(q)
        d = {o: _value(prob, f"stats.{k}_{o}") for o in ("mu", "sigma", "lo", "hi")}
        if q == "score":
            d["robust"] = _value(prob, "stats.score_robust")
        d["nodes"] = [_value(prob, f"nodes.n{j}.{q}") for j in range(prob.rdo["N"])]
        out[q] = d
    return out
