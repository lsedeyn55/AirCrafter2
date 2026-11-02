"""Correction-factor fitting, Garcia & Gamboa (2022) Section 4.2.1.

Eq. 127 (one aircraft):     F = sum_i ( m_calc,i / m_real,i - 1 )^2
Eq. 128 (several aircraft): F = sum_j sum_i ( m_calc,i / m_real,i - 1 )_j^2

The paper minimised these with Excel's GRG solver. Here they are minimised with
scipy's bounded least squares, re-running the mass group for every evaluation.
Relative residuals with equal weights, as G&G. This is an offline tool for
calibrating the mass model against USYD aircraft, not part of the optimisation loop.

Usage:
    cases = [(config_a, {"m_wing": 0.68, "m_tb": 0.175, ...}), (config_b, {...})]
    result = fit_correction_factors(cases, ["f_wing", "f_tb"])
    result.factors  -> {"f_wing": ..., "f_tb": ...}

A case is either a (MassConfig, targets) pair, which runs the mass closure as G&G do, or a CalibrationCase:
- ``sizing_mass`` (kg) fits the component equations at that mass with no closure (external sizing: the loads see the
  aircraft's design mass, so the factors are isolated from the closure error and m_sys is not needed);
- a target key is a mass output (``m_fus``) or a sum of outputs (``"m_cb+m_tb"``): a lumped real mass;
- ``values`` are set on the problem after the config (e.g. the wing-unit mass and stations of a twin).

``leave_one_out`` refits without each case in turn and predicts the case left out (docs/studies/mass_calibration_usyd.md).
"""

from dataclasses import dataclass, field

import numpy as np
from scipy.optimize import least_squares

from .garcia_gamboa import build_mass_problem, mass_breakdown, set_mass_inputs

FACTOR_BOUNDS = {
    "f_wing": (0.5, 3.0),
    "k_wing": (0.1, 10.0),
    "f_tb": (0.1, 100.0),
    "f_ht": (0.01, 3.0),
    "f_vt": (0.01, 3.0),
    "a_ht": (0.01, 10.0),  # kg/m^2
    "a_vt": (0.01, 10.0),  # kg/m^2
    "f_cb": (0.01, 5.0),
    "f_gear": (0.0, 0.1),
}

STRUCTURE = ["m_wing", "m_ht", "m_vt", "m_cb", "m_tb", "m_fus", "m_gear"]  # Eq. 40 without m_sys (wing to gear)


@dataclass
class CalibrationCase:
    name: str
    config: object  # MassConfig
    targets: dict  # {output or "a+b": real mass in kg}
    sizing_mass: float = None  # kg; None = the mass closure
    values: dict = field(default_factory=dict)  # {promoted input: (value, units)} set after the config


@dataclass
class CalibrationResult:
    factors: dict
    objective: float  # Eq. 127 / 128 value at the optimum
    relative_errors: list  # per case: {component: m_calc / m_real - 1}
    success: bool
    message: str


def _as_case(c, i):
    if isinstance(c, CalibrationCase):
        return c
    cfg, real = c
    return CalibrationCase(name=f"case{i}", config=cfg, targets=real)


def _with_inputs(case, inputs):
    """A copy of a case with config inputs overridden (held factors)."""
    from dataclasses import replace

    cfg = case.config.model_copy(deep=True)
    cfg.inputs.update(inputs)
    return replace(case, config=cfg)


def predicted(masses, key):
    """A target's model value: one output, or the sum of '+'-joined outputs."""
    return sum(masses[k] for k in key.split("+"))


class _CaseProblem:
    """The mass group of one case, set up once, re-run for each set of factors."""

    def __init__(self, case):
        import openmdao.api as om

        from .garcia_gamboa import GarciaGamboaMass
        from .garcia_gamboa.inputs import active_inputs

        self.case = case
        cfg = case.config.model_copy(deep=True)
        self.active = set(active_inputs(cfg.options))
        cfg.options.max_mode = "hard"  # the paper's fit uses the exact max
        if case.sizing_mass is None:
            self.prob = build_mass_problem(cfg)
        else:
            prob = om.Problem(reports=False)
            prob.model.add_subsystem("mass", GarciaGamboaMass(config=cfg.options, external_sizing=True), promotes=["*"])
            prob.setup()
            active = set(active_inputs(cfg.options))
            set_mass_inputs(prob, {k: v for k, v in cfg.inputs.items() if k in active})
            prob.set_val("m", case.sizing_mass, units="kg")
            self.prob = prob
        for name, (val, units) in case.values.items():
            self.prob.set_val(name, val, units=units)

    def run(self, factors):
        """Masses with these factors; factors the case's options do not use (f_ht with areal tails) are skipped."""
        set_mass_inputs(self.prob, {k: v for k, v in factors.items() if k in self.active})
        self.prob.run_model()
        return mass_breakdown(self.prob)


def _bounds(factors, bounds):
    bounds = bounds or {}
    return (
        [bounds.get(f, FACTOR_BOUNDS[f])[0] for f in factors],
        [bounds.get(f, FACTOR_BOUNDS[f])[1] for f in factors],
    )


def fit_correction_factors(cases, factors, x0=None, bounds=None):
    """Fit ``factors`` (names from FACTOR_BOUNDS) to real component masses.

    ``cases``: (MassConfig, {mass output: real mass in kg}) pairs or CalibrationCase objects.
    One case gives Eq. 127; several give the combined fit of Eq. 128.
    """
    cases = [_as_case(c, i) for i, c in enumerate(cases)]
    probs = [_CaseProblem(c) for c in cases]
    for cp in probs:
        unused = [f for f in factors if f not in cp.active]
        if unused:
            raise ValueError(f"{cp.case.name}: factors {unused} are not inputs with its options (tail_mass_model?)")
    if x0 is None:
        first_inputs = cases[0].config.inputs
        x0 = [first_inputs.get(f, np.mean(FACTOR_BOUNDS[f])) for f in factors]
    lo, hi = _bounds(factors, bounds)
    x0 = np.clip(np.asarray(x0, float), lo, hi)

    def errors(x):
        out = []
        for cp in probs:
            got = cp.run(dict(zip(factors, x)))
            out.append({k: predicted(got, k) / v - 1.0 for k, v in cp.case.targets.items()})
        return out

    def residuals(x):
        return np.concatenate([list(e.values()) for e in errors(x)])

    sol = least_squares(residuals, x0, bounds=(lo, hi), xtol=1e-12, ftol=1e-12, gtol=1e-12)
    return CalibrationResult(
        factors=dict(zip(factors, sol.x.tolist())),
        objective=float(np.sum(sol.fun**2)),
        relative_errors=errors(sol.x),
        success=bool(sol.success),
        message=sol.message,
    )


def predict(case, factors):
    """Masses of one case with the given factors: (all outputs, {target: model value})."""
    cp = _CaseProblem(_as_case(case, 0))
    got = cp.run(factors)
    return got, {k: predicted(got, k) for k in cp.case.targets}


def structure_total(targets):
    """Sum of a case's structural targets (wing to gear) that have data; lumped keys count once."""
    return sum(v for k, v in targets.items() if all(p in STRUCTURE for p in k.split("+")))


def leave_one_out(cases, factors, x0=None, bounds=None, fixed=None):
    """Refit without each case and predict it. Returns {name: dict(factors, errors, structure_error, ...)}.

    errors: per target, m_pred / m_real - 1 on the case left out; structure_error: the same on the sum of its
    structural targets. ``factors``: a list, or a callable (remaining cases) -> (factor list, {factor: value}) for
    when the remaining cases do not identify every factor (those are held at the given values). ``fixed``:
    {factor: value} applied to every fold's prediction.
    """
    cases = [_as_case(c, i) for i, c in enumerate(cases)]
    out = {}
    for i, held in enumerate(cases):
        rest = cases[:i] + cases[i + 1 :]
        names, held_at = factors(rest) if callable(factors) else (factors, {})
        if held_at:  # the held factors apply to the fold's fit too, not only to the prediction (else the fit
            # would use whatever the case configs carry, e.g. factors calibrated with the left-out case in the data)
            rest = [_with_inputs(c, held_at) for c in rest]
        start = None if x0 is None else [x0[f] for f in names] if isinstance(x0, dict) else x0
        fit = fit_correction_factors(rest, names, x0=start, bounds=bounds)
        use = {**(fixed or {}), **held_at, **fit.factors}
        _, pred = predict(held, use)
        real_tot = structure_total(held.targets)
        pred_tot = sum(v for k, v in pred.items() if all(p in STRUCTURE for p in k.split("+")))
        out[held.name] = dict(
            factors=fit.factors,
            held_at=held_at,
            objective=fit.objective,
            success=fit.success,
            errors={k: pred[k] / v - 1.0 for k, v in held.targets.items()},
            structure_real=real_tot,
            structure_pred=pred_tot,
            structure_error=pred_tot / real_tot - 1.0,
        )
    return out
