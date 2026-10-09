"""Correction-factor fitting, Garcia & Gamboa (2022) Section 4.2.1.

Eq. 127 (one aircraft):     F = sum_i ( m_calc,i / m_real,i - 1 )^2
Eq. 128 (several aircraft): F = sum_j sum_i ( m_calc,i / m_real,i - 1 )_j^2

The paper minimised these with Excel's GRG solver. Here they are minimised with
scipy's bounded least squares, re-running the mass closure for every evaluation.
This is an offline tool for calibrating the mass model against USYD aircraft,
not part of the optimisation loop.

Usage:
    cases = [(config_a, {"m_wing": 0.68, "m_tb": 0.175, ...}), (config_b, {...})]
    result = fit_correction_factors(cases, ["f_wing", "f_tb"])
    result.factors  -> {"f_wing": ..., "f_tb": ...}
"""

from dataclasses import dataclass

import numpy as np
from scipy.optimize import least_squares

from .garcia_gamboa import build_mass_problem, mass_breakdown, set_mass_inputs

FACTOR_BOUNDS = {
    "f_wing": (0.5, 3.0),
    "f_tb": (1.0, 100.0),
    "f_ht": (0.01, 3.0),
    "f_vt": (0.01, 3.0),
    "f_cb": (0.01, 5.0),
    "f_gear": (0.0, 0.1),
}


@dataclass
class CalibrationResult:
    factors: dict
    objective: float  # Eq. 127 / 128 value at the optimum
    relative_errors: list  # per case: {component: m_calc / m_real - 1}
    success: bool
    message: str


def fit_correction_factors(cases, factors, x0=None, bounds=None):
    """Fit ``factors`` (names from FACTOR_BOUNDS) to real component masses.

    ``cases`` is a list of (MassConfig, {mass output name: real mass in kg}).
    One case gives Eq. 127; several give the combined fit of Eq. 128.
    """
    probs = []
    for cfg, _ in cases:
        cfg = cfg.model_copy(deep=True)
        cfg.options.max_mode = "hard"  # the paper's fit uses the exact max
        probs.append(build_mass_problem(cfg))

    first_inputs = cases[0][0].inputs
    if x0 is None:
        x0 = [first_inputs.get(f, np.mean(FACTOR_BOUNDS[f])) for f in factors]
    bounds = bounds or {}
    lo = [bounds.get(f, FACTOR_BOUNDS[f])[0] for f in factors]
    hi = [bounds.get(f, FACTOR_BOUNDS[f])[1] for f in factors]

    def errors(x):
        out = []
        for prob, (_, real) in zip(probs, cases):
            set_mass_inputs(prob, dict(zip(factors, x)))
            prob.run_model()
            got = mass_breakdown(prob)
            out.append({k: got[k] / v - 1.0 for k, v in real.items()})
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
