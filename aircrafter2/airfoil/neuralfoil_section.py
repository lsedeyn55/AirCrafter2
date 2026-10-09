"""NeuralFoil section aerodynamics at a prescribed section lift coefficient.

The VLM gives each spanwise strip a section C_l and a Reynolds number. NeuralFoil takes
alpha, not C_l, so for each strip we solve  C_l,NF(alpha; k, Re) = C_l  on the pre-stall
branch, then read C_d and C_m there. C_l,max(Re) is the peak of C_l(alpha).

Derivatives use the implicit-function theorem, so nothing is differentiated through the
root-finder. Only the network's own partials are taken, by central finite differences:

    d alpha / d C_l = 1 / C_l,alpha
    d X / d C_l     = X_alpha / C_l,alpha                         (X = C_d, C_m)
    d X / d p       = X_p - X_alpha C_l,p / C_l,alpha             (p = ln Re, Kulfan params)
    d C_l,max / d p = C_l,p at alpha_peak                         (envelope theorem)

If C_l exceeds C_l,max the strip is "clamped" at alpha_peak: the outputs are then flat in C_l,
and the optimiser sees the stall through the cl_margin constraint instead.

Reference: NeuralFoil v0.3.3 (P. Sharpe, MIT), https://github.com/peterdsharpe/NeuralFoil; trained on
XFoil analyses of Kulfan-parameterised airfoils. Cite the version actually used in the thesis.
"""

from dataclasses import dataclass

import numpy as np

N_PARAMS = 18  # [upper(8), lower(8), le_weight, te_thickness]


@dataclass
class NFSettings:
    model_size: str = "xlarge"
    n_crit: float = 9.0
    alpha_min: float = -8.0  # deg
    alpha_max: float = 25.0  # deg
    alpha_step: float = 0.5  # deg
    h_alpha: float = 1e-4  # deg, central-difference step in alpha (docs/neuralfoil_fd_steps.md)
    h_logre: float = 1e-5  # central-difference step in ln(Re)
    h_k: float = 1e-6  # on each Kulfan parameter; the TE-thickness derivative needs <= 1e-6
    newton_iters: int = 8


def pack(upper, lower, le, te):
    return np.concatenate([np.ravel(upper), np.ravel(lower), np.ravel(le), np.ravel(te)])


def nf_eval(P, alpha, Re, s: NFSettings):
    """P: (18,) or (18, N); alpha (deg), Re: (N,). Returns dict of (N,) arrays."""
    import neuralfoil as nf

    alpha = np.atleast_1d(np.asarray(alpha, float))
    N = alpha.size
    P = np.asarray(P, float)
    if P.ndim == 1:
        P = np.tile(P[:, None], (1, N))
    kp = dict(upper_weights=P[0:8], lower_weights=P[8:16], leading_edge_weight=P[16], TE_thickness=P[17])
    r = nf.get_aero_from_kulfan_parameters(kp, alpha, np.broadcast_to(Re, (N,)), n_crit=s.n_crit,
                                           model_size=s.model_size)
    return {k: np.asarray(r[k], float) for k in ("CL", "CD", "CM", "analysis_confidence")}


def _cl(P, alpha, Re, s):
    return nf_eval(P, alpha, Re, s)["CL"]


def solve_sections(P, Cl_target, Re, s: NFSettings):
    """Solve each strip. Returns dict of (n,) arrays."""
    Cl_target = np.asarray(Cl_target, float)
    Re = np.asarray(Re, float)
    n = Cl_target.size
    grid = np.arange(s.alpha_min, s.alpha_max + 0.5 * s.alpha_step, s.alpha_step)
    na = grid.size

    # 1. polar on the alpha grid for every strip
    g = nf_eval(P, np.tile(grid, n), np.repeat(Re, na), s)
    Clg = g["CL"].reshape(n, na)

    # 2. C_l,max: grid argmax, then Newton on dC_l/dalpha = 0
    idx = np.argmax(Clg, axis=1)
    interior = (idx > 0) & (idx < na - 1)
    i0 = np.clip(idx, 1, na - 2)
    y0, y1, y2 = (Clg[np.arange(n), i0 + k] for k in (-1, 0, 1))
    curv = y0 - 2 * y1 + y2
    a_pk = grid[i0] + np.where(curv < 0, s.alpha_step * (y0 - y2) / (2 * np.where(curv < 0, curv, -1.0)), 0.0)
    a_pk = np.where(interior, a_pk, grid[idx])
    h = 0.05
    for _ in range(4):
        c = nf_eval(P, np.concatenate([a_pk - h, a_pk, a_pk + h]), np.tile(Re, 3), s)["CL"].reshape(3, n)
        d1 = (c[2] - c[0]) / (2 * h)
        d2 = (c[2] - 2 * c[1] + c[0]) / h**2
        step = np.where(interior & (d2 < 0), -d1 / np.where(d2 < 0, d2, -1.0), 0.0)
        a_pk = np.clip(a_pk + np.clip(step, -s.alpha_step, s.alpha_step), grid[0], grid[-1])
    Clmax = _cl(P, a_pk, Re, s)

    # 3. alpha at the target C_l on the pre-stall branch
    clamped = Cl_target >= Clmax
    alpha = np.empty(n)
    for i in range(n):
        br_a = grid[: idx[i] + 1]
        br_c = np.maximum.accumulate(Clg[i, : idx[i] + 1])
        if clamped[i]:
            alpha[i] = a_pk[i]
        elif br_c.size < 2 or Cl_target[i] < br_c[0]:
            slope = (Clg[i, 1] - Clg[i, 0]) / s.alpha_step
            alpha[i] = grid[0] + (Cl_target[i] - Clg[i, 0]) / max(slope, 1e-3)
        else:
            alpha[i] = np.interp(Cl_target[i], br_c, br_a)
    ha = s.h_alpha
    for _ in range(s.newton_iters):
        free = ~clamped
        if not free.any():
            break
        c = nf_eval(P, np.concatenate([alpha - ha, alpha, alpha + ha]), np.tile(Re, 3), s)["CL"].reshape(3, n)
        res = c[1] - Cl_target
        cla = (c[2] - c[0]) / (2 * ha)
        upd = np.where(free & (cla > 1e-4), -res / np.where(cla > 1e-4, cla, 1.0), 0.0)
        alpha = np.where(free, np.minimum(alpha + upd, a_pk), alpha)
        if np.max(np.abs(res[free])) < 1e-12:
            break

    f = nf_eval(P, alpha, Re, s)
    return dict(alpha=alpha, Cl=f["CL"], Cd=f["CD"], Cm=f["CM"], conf=f["analysis_confidence"],
                Clmax=Clmax, alpha_peak=a_pk, clamped=clamped, peak_in_grid=interior)


def _central(P, alpha, Re, s, directions):
    """Central differences of (CL, CD, CM) at (alpha, Re) along the given directions.

    directions: list of ("alpha", h) | ("logre", h) | ("k", j, h). Returns dict name -> (3, n, ndir).
    """
    n = alpha.size
    Ps, As, Rs = [], [], []
    for d in directions:
        for sign in (1.0, -1.0):
            Pd = np.tile(np.asarray(P, float)[:, None], (1, n))
            a, r = alpha.copy(), Re.copy()
            if d[0] == "alpha":
                a = a + sign * d[1]
            elif d[0] == "logre":
                r = r * np.exp(sign * d[1])
            else:
                Pd[d[1]] += sign * d[2]
            Ps.append(Pd)
            As.append(a)
            Rs.append(r)
    out = nf_eval(np.concatenate(Ps, axis=1), np.concatenate(As), np.concatenate(Rs), s)
    stack = np.stack([out["CL"], out["CD"], out["CM"]]).reshape(3, len(directions), 2, n)
    steps = np.array([d[-1] for d in directions])
    deriv = (stack[:, :, 0, :] - stack[:, :, 1, :]) / (2 * steps[None, :, None])
    return np.transpose(deriv, (0, 2, 1))  # (3, n, ndir)


def section_sensitivities(P, sol, Re, s: NFSettings):
    """Total derivatives of the solved strip quantities.

    Returns dict with arrays (n,) for d/dCl and d/dlnRe, and (n, 18) for d/dk, for
    alpha (deg), Cd, Cm; and dClmax_dlnRe (n,), dClmax_dk (n, 18).
    """
    dirs = [("alpha", s.h_alpha), ("logre", s.h_logre)] + [("k", j, s.h_k) for j in range(N_PARAMS)]
    D = _central(P, sol["alpha"], Re, s, dirs)  # (3, n, 20)
    Cl_a, Cl_r, Cl_k = D[0, :, 0], D[0, :, 1], D[0, :, 2:]
    free = ~sol["clamped"]
    inv = np.where(free, 1.0 / np.where(np.abs(Cl_a) > 1e-8, Cl_a, 1.0), 0.0)

    out = {"alpha_dCl": inv, "alpha_dlnRe": -Cl_r * inv, "alpha_dk": -Cl_k * inv[:, None]}
    for q, name in ((1, "Cd"), (2, "Cm")):
        X_a, X_r, X_k = D[q, :, 0], D[q, :, 1], D[q, :, 2:]
        out[f"{name}_dCl"] = X_a * inv
        out[f"{name}_dlnRe"] = X_r - X_a * Cl_r * inv
        out[f"{name}_dk"] = X_k - (X_a * inv)[:, None] * Cl_k

    Dp = _central(P, sol["alpha_peak"], Re, s, dirs[1:])  # (3, n, 19)
    out["Clmax_dlnRe"] = Dp[0, :, 0]
    out["Clmax_dk"] = Dp[0, :, 1:]
    return out
