"""Propagation rules: nodes and weights for the statistics of a model output f(p).

Every rule is a ``Rule``: N nodes (physical parameter values, shape (N, d)), a mean-weight vector ``a`` and a
symmetric matrix Q such that, for node outputs f (shape (N,)),

    mu = a . f,        var = f^T Q f,        Q = diag(D) - U diag(c) U^T  (stored factored, U is N x r).

That one form covers every rule below, so the statistics component (stats.py) has exact, analytic partials
(d mu/df = a, d var/df = 2 Q f) whatever the rule.

udr       univariate dimension reduction (Rahman & Xu 2004):
              f(p) ~ f_hat(p) = sum_i f(p_bar_1 .. p_i .. p_bar_d) - (d - 1) f(p_bar),
          with an n-point Gauss rule in each dimension (Gauss-Legendre for uniform inputs, Gauss-Hermite for
          normal ones, Gauss rule of the triangular density for triangular ones). Nodes: the reference point
          p_bar (the means) plus the off-mean Gauss points of each dimension; n = 3 gives N = 2d + 1.
          mu = E[f_hat]. var = Var[f_hat] = sum_i Var_i[f_i(p_i)], the exact variance of the additive
          approximation (never negative; cross terms are not represented).
smolyak   Smolyak sparse grid, combination technique on the same Gauss rules with 1, 3, 5, ... points;
          level L is exact for polynomials of total degree 2L + 1 and represents cross terms. Weights can be
          negative, so var can be negative: stats.py takes a smooth positive part.
sobol     a fixed (scrambled, seeded) Sobol' sample with equal weights and the unbiased sample variance.
          For verification (Monte Carlo reference), not for the optimiser.
"""

from dataclasses import dataclass
from itertools import product
from math import comb

import numpy as np
from numpy.polynomial.hermite_e import hermegauss
from numpy.polynomial.legendre import leggauss
from scipy.stats import qmc


@dataclass
class Rule:
    name: str
    nodes: np.ndarray  # (N, d), physical values
    a: np.ndarray  # (N,)
    D: np.ndarray  # (N,)    Q = diag(D) - U diag(c) U^T
    U: np.ndarray  # (N, r)
    c: np.ndarray  # (r,)
    psd: bool  # Q positive semi-definite (var >= 0 for any f)

    @property
    def n(self):
        return len(self.a)

    @property
    def Q(self):
        return np.diag(self.D) - (self.U * self.c) @ self.U.T

    def var(self, f):  # complex-step safe (no float cast)
        return self.D @ (f * f) - self.c @ (self.U.T @ f) ** 2

    def var_grad(self, f):
        return 2.0 * (self.D * f - self.U @ (self.c * (self.U.T @ f)))

    def moments(self, f):
        f = np.asarray(f, dtype=float)
        return float(self.a @ f), float(self.var(f))


def _gauss_from_discrete(x, w, n):
    """n-point Gauss rule of the discrete measure (x, w) by the Stieltjes procedure and Golub-Welsch."""
    alpha, beta = np.zeros(n), np.zeros(n)
    p_prev, p = np.zeros_like(x), np.ones_like(x)
    norm_prev = None
    for k in range(n):
        nk = np.sum(w * p * p)
        alpha[k] = np.sum(w * x * p * p) / nk
        beta[k] = nk / norm_prev if norm_prev is not None else np.sum(w)
        p_next = (x - alpha[k]) * p - (beta[k] if k > 0 else 0.0) * p_prev
        p_prev, p, norm_prev = p, p_next, nk
    J = np.diag(alpha) + np.diag(np.sqrt(beta[1:]), 1) + np.diag(np.sqrt(beta[1:]), -1)
    xi, V = np.linalg.eigh(J)
    return xi, beta[0] * V[0] ** 2


def gauss_1d(kind, n):
    """n-point Gauss rule (xi, w) of the standardised variable, weights summing to 1."""
    if kind == "uniform":
        xi, w = leggauss(n)
        return xi, w / 2.0
    if kind == "normal":
        xi, w = hermegauss(n)
        return xi, w / np.sqrt(2.0 * np.pi)
    if kind == "triangular":  # density 1 - |xi| on [-1, 1]; piecewise linear, so 40 Legendre points are exact
        t, wt = leggauss(40)
        x = np.concatenate([0.5 * (t - 1.0), 0.5 * (t + 1.0)])
        w = np.concatenate([0.5 * wt, 0.5 * wt]) * (1.0 - np.abs(x))
        xi, ww = _gauss_from_discrete(x, w, n)
        xi[np.abs(xi) < 1e-13] = 0.0
        return xi, ww
    raise ValueError(kind)


def _means(dists):
    return np.array([d.mean for d in dists])


def udr(dists, n=3):
    """Univariate dimension reduction with n Gauss points per dimension."""
    d = len(dists)
    pbar = _means(dists)
    nodes = [pbar.copy()]
    per_dim = []  # [(node indices, weights)] per dimension
    for i, ds in enumerate(dists):
        xi, w = gauss_1d(ds.kind, n)
        idx = []
        for x in xi:
            if abs(x) < 1e-12:  # the reference point itself
                idx.append(0)
            else:
                p = pbar.copy()
                p[i] = ds.mean + ds.scale * x
                nodes.append(p)
                idx.append(len(nodes) - 1)
        per_dim.append((idx, w))
    N = len(nodes)
    a = np.zeros(N)
    a[0] = -(d - 1.0)
    D, U = np.zeros(N), np.zeros((N, d))
    for i, (idx, w) in enumerate(per_dim):
        np.add.at(U[:, i], idx, w)
        a += U[:, i]
        D += U[:, i]
    if d == 0:
        a[0] = 1.0
    return Rule(f"udr{n}", np.array(nodes).reshape(N, d), a, D, U, np.ones(d), True)


def smolyak(dists, level=2):
    """Smolyak sparse grid (combination technique) on Gauss rules with 2i - 1 points, i = 1 .. level + 1."""
    d = len(dists)
    if d == 0:
        return udr(dists)
    q = d + level
    rules = {i: [gauss_1d(ds.kind, 2 * i - 1) for ds in dists] for i in range(1, level + 2)}
    acc = {}
    for idx in product(range(1, level + 2), repeat=d):
        s = sum(idx)
        if not (max(d, q - d + 1) <= s <= q):
            continue
        coef = (-1) ** (q - s) * comb(d - 1, q - s)
        grids = [rules[i][j] for j, i in enumerate(idx)]
        for pts in product(*[range(len(g[0])) for g in grids]):
            xi = tuple(round(float(grids[j][0][k]), 12) for j, k in enumerate(pts))
            w = coef * np.prod([grids[j][1][k] for j, k in enumerate(pts)])
            acc[xi] = acc.get(xi, 0.0) + w
    keys = [k for k, w in acc.items() if abs(w) > 1e-14]
    keys.sort(key=lambda k: (sum(abs(x) > 0 for x in k), k))  # the reference point first
    xi = np.array(keys)
    w = np.array([acc[k] for k in keys])
    nodes = _means(dists) + xi * np.array([ds.scale for ds in dists])
    return Rule(f"smolyak{level}", nodes, w, w.copy(), w[:, None], np.ones(1), bool(np.all(w >= 0)))


def sobol(dists, n=1024, seed=0):
    """Fixed scrambled Sobol' sample (n should be a power of 2), equal weights, unbiased variance.

    Node 0 is the reference point (the means) with zero weight, so that every rule has the reference point
    first (non-robust constraints are evaluated there); the n samples follow.
    """
    d = len(dists)
    u = qmc.Sobol(d=max(d, 1), scramble=True, seed=seed).random(n)[:, :d]
    nodes = np.column_stack([ds.ppf(u[:, j]) for j, ds in enumerate(dists)]) if d else np.zeros((n, 0))
    nodes = np.vstack([_means(dists)[None, :], nodes])
    a = np.concatenate([[0.0], np.full(n, 1.0 / n)])
    s = n / (n - 1.0)  # unbiased
    return Rule(f"sobol{n}", nodes, a, s * a, a[:, None], np.array([s]), True)


def make_rule(rcfg, dists):
    """The rule a RobustConfig asks for, over the given (active) distributions."""
    if rcfg.quadrature == "udr":
        return udr(dists, rcfg.points)
    if rcfg.quadrature == "smolyak":
        return smolyak(dists, rcfg.smolyak_level)
    return sobol(dists, rcfg.sobol_n, rcfg.sobol_seed)
