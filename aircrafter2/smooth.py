"""Smooth replacements for max, min and clip in the gradient path, used throughout the physics components.

All functions are complex-step safe: branching only looks at real parts, and the arithmetic is done with numpy ufuncs
that accept complex input. The KS function is the Kreisselmeier-Steinhauser aggregation.
"""

import numpy as np

TINY = 1e-30


def _floor_positive(x):
    """(x floored at TINY, the mask of values above it)."""
    x = np.asarray(x)
    return np.where(np.real(x) > TINY, x, TINY), np.real(x) > TINY


def ks_max(xs, rho=100.0, mode="ks"):
    """Smooth (or hard) element-wise maximum of positive arrays.

    Kreisselmeier-Steinhauser aggregation in log space, for strictly positive quantities (thicknesses, loads, masses):

        f = exp( KS_rho( ln x_1, ..., ln x_n ) )

    This makes rho dimensionless and the result scale-invariant. The over-estimate relative to the true max is bounded
    by a factor n**(1/rho); for rho = 100 and n = 2 that's 0.7 %. Values <= TINY are floored to TINY and get zero
    derivative.

    Parameters
    ----------
    xs : list of array
        The values.
    rho : float
        Sharpness.
    mode : str
        "ks", or "hard" for the exact max (reporting and verification), with the derivative of the selected argument.

    Returns
    -------
    (array, list of array)
        f, and df/dx_i for each x_i. Raises ValueError for an unknown mode.
    """
    if len(xs) == 1:
        x = np.asarray(xs[0])
        return x, [np.ones_like(x)]

    floored = [_floor_positive(x) for x in xs]
    vals = [v for v, _ in floored]
    active = [a for _, a in floored]

    if mode == "hard":
        stack = np.stack([np.real(v) for v in vals])
        idx = np.argmax(stack, axis=0)
        f = np.choose(idx, vals)
        grads = [np.where(idx == i, 1.0, 0.0) * active[i] for i in range(len(vals))]
        return f, grads
    if mode != "ks":
        raise ValueError(f"unknown max mode {mode!r} (use 'ks' or 'hard')")

    logs = [np.log(v) for v in vals]
    m = np.max(np.stack([np.real(lv) for lv in logs]), axis=0)
    exps = [np.exp(rho * (lv - m)) for lv in logs]
    s = sum(exps)
    f = np.exp(m + np.log(s) / rho)
    grads = [f * (e / s) / v * a for e, v, a in zip(exps, vals, active)]
    return f, grads


def ks_min(xs, rho=100.0, mode="ks"):
    """Smooth minimum of positive arrays: 1 / ks_max(1/x).

    Parameters
    ----------
    xs : list of array
    rho : float
    mode : str
        As in ks_max.

    Returns
    -------
    (array, list of array)
        f, and df/dx_i for each x_i.
    """
    inv = [1.0 / np.asarray(x) for x in xs]
    g, dg = ks_max(inv, rho, mode)
    f = 1.0 / g
    grads = [f**2 * d / np.asarray(x) ** 2 for d, x in zip(dg, xs)]
    return f, grads


def softplus(x, scale, mode="ks"):
    """Smooth max(x, 0): s * ln(1 + exp(x / s)), with a dimensional transition width s.

    Parameters
    ----------
    x : array
    scale : float
        Transition width s, in the units of x.
    mode : str
        "hard" gives max(x, 0).

    Returns
    -------
    (array, array)
        f and df/dx.
    """
    x = np.asarray(x)
    if mode == "hard":
        pos = np.real(x) > 0.0
        return np.where(pos, x, 0.0 * x), np.where(pos, 1.0, 0.0)
    z = x / scale
    zr = np.real(z)
    big = zr > 0.0
    # stable: z + log1p(exp(-z)) for z > 0, log1p(exp(z)) otherwise
    f = scale * np.where(big, z + np.log1p(np.exp(-np.where(big, z, 0.0))), np.log1p(np.exp(np.where(big, 0.0, z))))
    sig = np.where(
        big,
        1.0 / (1.0 + np.exp(-np.where(big, z, 0.0))),
        np.exp(np.where(big, 0.0, z)) / (1.0 + np.exp(np.where(big, 0.0, z))),
    )
    return f, sig


def smooth_abs(x, eps):
    """Smooth absolute value, sqrt(x^2 + eps^2).

    Parameters
    ----------
    x : array
    eps : float
        Smoothing width, in the units of x.

    Returns
    -------
    (array, array)
        f and df/dx.
    """
    x = np.asarray(x)
    f = np.sqrt(x * x + eps * eps)
    return f, x / f
