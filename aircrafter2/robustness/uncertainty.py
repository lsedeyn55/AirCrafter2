"""Uncertain model parameters: specs, distributions and first-order screening.

An uncertain parameter is a model input (promoted name, or lap.<name> for lap-model inputs) with an independent
distribution. Design variables are never uncertain parameters (manufacturing tolerances on them are a hook,
RobustConfig.dv_tolerances, not implemented). Read from the robust: block and configs/uncertainty.yaml; feeds
quadrature.py and the robust problems. Details: docs/modules/robustness.md, "Uncertain parameters".
"""

from dataclasses import dataclass
from itertools import combinations
from pathlib import Path
from typing import Literal, Optional

import numpy as np
from pydantic import BaseModel, ConfigDict, Field
from scipy.stats import norm

DIST_STD = {"uniform": 1.0 / np.sqrt(3.0), "normal": 1.0, "triangular": 1.0 / np.sqrt(6.0)}

# Constraints made robust by default: energy, take-off thrust, stall margin, static margin and the spar wall.
DEFAULT_ROBUST = ["energy_margin", "thrust_margin_static", "stall_margin", "SM", "wall_ratio", "r_tube"]


class UncertainParam(BaseModel):
    """One uncertain parameter as a config writes it (field descriptions below)."""

    model_config = ConfigDict(extra="forbid")
    name: str = Field(description="promoted model input; lap-model inputs as lap.<name>")
    dist: Literal["uniform", "normal", "triangular"] = "uniform"
    nominal: Optional[float] = Field(None, description="centre / mean / mode; None = the config's own value")
    spread: float = Field(gt=0, description="half-width (uniform, triangular) or standard deviation (normal)")
    relative: bool = Field(False, description="spread as a fraction of the nominal value")
    units: Optional[str] = None
    source: str = "PLACEHOLDER"
    kv_mode: Literal["both", "fixed", "dv"] = Field(
        "both",
        description="motor mode the parameter applies to: 'fixed' (Kv fixed: Kv, Rm), 'dv' (Kv a design "
        "variable: Kv_factor, Rm_ref), or both",
    )


def for_kv_mode(params, kv_dv):
    """The parameters that apply to the motor mode, in list order.

    Parameters
    ----------
    params : list of UncertainParam
    kv_dv : bool
        Kv is a design variable.

    Returns
    -------
    list of UncertainParam
    """
    return [p for p in params if p.kv_mode == "both" or p.kv_mode == ("dv" if kv_dv else "fixed")]


class RobustConfig(BaseModel):
    """The optional ``robust:`` block of FullMDOConfig. Off by default: the deterministic problem is unchanged."""

    model_config = ConfigDict(extra="forbid")
    enabled: bool = False
    uncertainty_config: Optional[str] = Field(
        None,
        description="YAML with a 'parameters' list, relative to the "
        "config file (merged into 'parameters' by load_full_config)",
    )
    parameters: list[UncertainParam] = []
    active: Optional[list[str]] = Field(
        None,
        description="parameters propagated by the quadrature (from the "
        "screening); the others are held at their means. None = all",
    )
    quadrature: Literal["udr", "smolyak", "sobol"] = Field(
        "udr",
        description="udr: univariate dimension reduction (Rahman & Xu 2004), Gauss rules per dimension; "
        "smolyak: sparse grid (cross terms); sobol: fixed scrambled Sobol' sample (verification only)",
    )
    points: int = Field(3, ge=1, description="Gauss points per dimension (udr)")
    smolyak_level: int = Field(2, ge=1)
    sobol_n: int = Field(1024, ge=2)
    sobol_seed: int = 0
    objective: Literal["mean_plus_k_sigma", "mean_sigma_cap"] = Field(
        "mean_plus_k_sigma",
        description="minimise mu + k sigma (maximise mu - k sigma for 'max' plug-ins), or "
        "optimise mu subject to sigma <= sigma_cap (epsilon-constraint, for the Pareto study)",
    )
    k: float = Field(1.0, ge=0)
    sigma_cap: Optional[float] = Field(None, gt=0, description="score sigma cap for objective = mean_sigma_cap")
    beta: float = Field(
        2.0,
        ge=0,
        description="mu - beta sigma >= lower, mu + beta sigma <= upper; 2 ~ 97.7 % "
        "one-sided if the constraint is normally distributed",
    )
    robust_constraints: list[str] = Field(
        DEFAULT_ROBUST,
        description="constraints by moment matching; the others stay deterministic at the reference node",
    )
    optimizer: Literal["slsqp", "ipopt"] = Field(
        "slsqp",
        description="slsqp: ScipyOptimizeDriver (as the deterministic problem); ipopt: pyOptSparse IPOPT, "
        "which backs off from an MDA failure (AnalysisError -> fail flag) instead of stopping",
    )
    sigma_eps: float = Field(1e-6, gt=0, description="sigma = sqrt(var + eps^2), eps = sigma_eps * constraint ref")
    screening_threshold: float = Field(0.9, gt=0, le=1)
    max_dim: int = Field(5, ge=1)
    dv_tolerances: dict = Field(
        {},
        description="HOOK, not implemented: manufacturing tolerances on design "
        "variables (e.g. b, c, tc_scale perturbed as x + delta at the nodes)",
    )


def load_uncertainty(path, kv_dv=None):
    """The parameter list of an uncertainty YAML (configs/uncertainty.yaml).

    Parameters
    ----------
    path : str or Path
    kv_dv : bool, optional
        True or False keeps only the parameters of that motor mode (for_kv_mode); None keeps all.

    Returns
    -------
    list of UncertainParam
    """
    from ..config.loader import load_yaml

    params = [UncertainParam(**p) for p in load_yaml(Path(path))["parameters"]]
    return params if kv_dv is None else for_kv_mode(params, kv_dv)


@dataclass
class Dist:
    """An uncertain parameter resolved to absolute numbers: x = mean + scale * xi.

    uniform      xi ~ U(-1, 1),                  scale = half-width,  std = scale / sqrt(3)
    normal       xi ~ N(0, 1),                   scale = std
    triangular   xi on [-1, 1], density 1 - |xi| (symmetric, mode = mean), scale = half-width, std = scale / sqrt(6)
    """

    name: str
    kind: str
    mean: float
    scale: float
    units: Optional[str] = None

    @property
    def std(self):
        """Standard deviation, in the parameter's units."""
        return self.scale * DIST_STD[self.kind]

    @property
    def bounds(self):
        """(lower, upper) of the parameter; infinite for a normal."""
        return (-np.inf, np.inf) if self.kind == "normal" else (self.mean - self.scale, self.mean + self.scale)

    def ppf(self, u):
        """Inverse CDF of the standardised variable, mapped to x.

        Parameters
        ----------
        u : float or array
            Probability, in [0, 1].

        Returns
        -------
        float or array
        """
        u = np.asarray(u, dtype=float)
        if self.kind == "uniform":
            xi = 2.0 * u - 1.0
        elif self.kind == "normal":
            xi = norm.ppf(u)
        else:
            xi = np.where(u < 0.5, -1.0 + np.sqrt(2.0 * u), 1.0 - np.sqrt(2.0 * (1.0 - u)))
        return self.mean + self.scale * xi


def resolve(params, current_value):
    """UncertainParams to Dists.

    Parameters
    ----------
    params : list of UncertainParam
    current_value : callable
        current_value(name) gives the config value where nominal is None.

    Returns
    -------
    list of Dist
    """
    out = []
    for p in params:
        nom = current_value(p.name) if p.nominal is None else p.nominal
        scale = p.spread * abs(nom) if p.relative else p.spread
        out.append(Dist(p.name, p.dist, float(nom), float(scale), p.units))
    return out


def _value(prob, name):
    """One scalar output of the problem."""
    return float(np.ravel(prob.get_val(name))[0])


def first_order_contributions(prob, dists, of):
    """First-order variance contributions at the current point (set the parameters to their means first).

    Parameters
    ----------
    prob : om.Problem
    dists : list of Dist
    of : list of str

    Returns
    -------
    dict
        {quantity: {param: (df/dp sigma)^2}}.
    """
    J = prob.compute_totals(of=list(of), wrt=[d.name for d in dists], return_format="dict")
    return {q: {d.name: (float(np.ravel(J[q][d.name])[0]) * d.std) ** 2 for d in dists} for q in of}


def select_parameters(contrib, threshold=0.9, max_dim=5):
    """Smallest set (<= max_dim) covering threshold of every quantity's first-order variance.

    Quantities with zero first-order variance are skipped (nothing to cover). Ties: the set with the largest minimum
    coverage.

    Parameters
    ----------
    contrib : dict
        {quantity: {param: variance contribution}}.
    threshold : float
    max_dim : int

    Returns
    -------
    (list of str, dict, bool)
        selected, coverage {quantity: share}, reached.
    """
    qs = {q: c for q, c in contrib.items() if sum(c.values()) > 0}
    names = list(next(iter(contrib.values())))
    share = {q: {n: c[n] / sum(c.values()) for n in names} for q, c in qs.items()}

    def cover(sel):
        """Share of each quantity's variance that the set sel covers."""
        return {q: sum(s[n] for n in sel) for q, s in share.items()}

    best = None
    for size in range(1, min(max_dim, len(names)) + 1):
        for sel in combinations(names, size):
            cv = cover(sel)
            score = min(cv.values()) if cv else 1.0
            if best is None or score > best[0] + 1e-12:
                best = (score, sel, cv)
        if best[0] >= threshold:
            break
    score, sel, cv = best
    return list(sel), cv, score >= threshold


def screen(prob, dists, of, threshold=0.9, max_dim=5):
    """Screen the parameters: first-order variance contributions (df/dp_i sigma_i)^2 at the reference point.

    Sets every parameter to its mean, runs the model, and screens the objective and the robust constraints. Restores
    nothing: the caller owns prob.

    Parameters
    ----------
    prob : om.Problem
    dists : list of Dist
    of : list of str
    threshold : float
    max_dim : int

    Returns
    -------
    dict
        contrib, selected, coverage, reached, values.
    """
    for d in dists:
        prob.set_val(d.name, d.mean)
    prob.run_model()
    contrib = first_order_contributions(prob, dists, of)
    selected, coverage, reached = select_parameters(contrib, threshold, max_dim)
    return dict(
        contrib=contrib, selected=selected, coverage=coverage, reached=reached, values={q: _value(prob, q) for q in of}
    )
