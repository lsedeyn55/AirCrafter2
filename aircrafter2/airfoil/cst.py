"""Kulfan (CST with leading-edge modification) airfoil geometry.

Same definition as AeroSandbox's ``get_kulfan_coordinates`` (which NeuralFoil was trained on):
class function C(x) = x^0.5 (1 - x), Bernstein shape functions of order 7 (8 weights per side),
TE thickness added as +/- x te/2, and the LEM term w_LE x (1 - x)^(8.5) added to both surfaces.
References: Kulfan (2008) AIAA J. Aircraft 45(1); Kulfan (2020) LEM note (see AeroSandbox docs).

Everything here is complex-step safe, so components can use exact cs partials.
Parameter vector convention (length 18): [upper(8), lower(8), le_weight, te_thickness].
"""

from math import comb

import numpy as np

N_W = 8
N1, N2 = 0.5, 1.0
_K = np.array([comb(N_W - 1, i) for i in range(N_W)], dtype=float)


def cosspace(n):
    return 0.5 * (1.0 - np.cos(np.linspace(0.0, np.pi, n)))


def bernstein(x):
    """(len(x), 8) Bernstein basis for order 7."""
    x = np.asarray(x)
    i = np.arange(N_W)
    return _K * x[..., None] ** i * (1.0 - x[..., None]) ** (N_W - 1 - i)


def surfaces(x, upper, lower, le, te):
    """Upper and lower y/c at chord stations x."""
    x = np.asarray(x)
    C = np.where(np.real(x) > 0, x, 0.0 * x) ** N1 * (1.0 - x) ** N2
    B = bernstein(x)
    lem = le * x * (1.0 - x) ** (N_W + 0.5)
    yu = C * (B @ upper) + x * te / 2.0 + lem
    yl = C * (B @ lower) - x * te / 2.0 + lem
    return yu, yl


def coordinates(upper, lower, le, te, n_per_side=201):
    """Selig-ordered (TE -> upper -> LE -> lower -> TE) coordinates, shape (2n-1, 2)."""
    x = cosspace(n_per_side)
    yu, yl = surfaces(x, upper, lower, le, te)
    xs = np.concatenate((x[::-1], x[1:]))
    ys = np.concatenate((yu[::-1], yl[1:]))
    return np.stack((xs, ys), axis=1)


def _parabolic_peak(x, y):
    """Peak of samples y(x) refined by the parabola through the max and its neighbours."""
    i = int(np.argmax(np.real(y)))
    i = min(max(i, 1), len(y) - 2)
    x0, x1, x2 = x[i - 1], x[i], x[i + 1]
    y0, y1, y2 = y[i - 1], y[i], y[i + 1]
    # Lagrange parabola; vertex of a x^2 + b x + c
    d = (x0 - x1) * (x0 - x2) * (x1 - x2)
    a = (x2 * (y1 - y0) + x1 * (y0 - y2) + x0 * (y2 - y1)) / d
    b = (x2**2 * (y0 - y1) + x1**2 * (y2 - y0) + x0**2 * (y1 - y2)) / d
    c = (x1 * x2 * (x1 - x2) * y0 + x2 * x0 * (x2 - x0) * y1 + x0 * x1 * (x0 - x1) * y2) / d
    xv = -b / (2.0 * a)
    return a * xv**2 + b * xv + c, xv


def section_geometry(upper, lower, le, te, x_spar, n=401):
    """Non-dimensional section properties (unit chord).

    Returns dict:
        tc_max    maximum thickness / chord (parabolic peak refinement)
        x_tc_max  its chordwise position
        t_at_spar thickness / chord at x = x_spar (exact CST evaluation)
        A_c2      section area / c^2   (trapezoidal integral of the thickness)
        P_c       perimeter / c        (polyline length of both surfaces)
    """
    x = cosspace(n)
    yu, yl = surfaces(x, upper, lower, le, te)
    t = yu - yl
    tc_max, x_tc = _parabolic_peak(x, t)
    yus, yls = surfaces(np.atleast_1d(x_spar), upper, lower, le, te)
    area = np.sum(0.5 * (t[1:] + t[:-1]) * np.diff(x))

    def length(y):
        return np.sum(np.sqrt(np.diff(x) ** 2 + np.diff(y) ** 2))

    perim = length(yu) + length(yl) + te  # closing the blunt TE
    return dict(tc_max=tc_max, x_tc_max=x_tc, t_at_spar=(yus - yls)[0], A_c2=area, P_c=perim)
