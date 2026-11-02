"""Independent hand calculations used to sanity-check the aero model (the wing_aero problem).

Lifting-line theory, a Trefftz-plane induced drag and a textbook wing drag estimate. Called by verification/ and tests/;
results in docs/verification/aero.md.
"""

import numpy as np

from ..airfoil.neuralfoil_section import NFSettings, pack, solve_sections


def lifting_line_rect(AR, a0=2.0 * np.pi, n_terms=60):
    """Prandtl lifting-line theory, solved with Glauert's Fourier series, for an untwisted rectangular wing.

    Source: Anderson, Fundamentals of Aerodynamics, Ch. 5; Glauert, The Elements of Aerofoil and Airscrew Theory.

    Parameters
    ----------
    AR : float
    a0 : float
        Section lift-curve slope [1/rad].
    n_terms : int

    Returns
    -------
    dict
        e = 1 / (1 + delta), the span efficiency, and CL_alpha [1/rad].
    """
    th = np.linspace(0.0, np.pi / 2.0, n_terms + 1)[1:]
    n = 2 * np.arange(n_terms) + 1  # symmetric loading: odd terms only
    mu = a0 / (4.0 * AR)  # a0 c / (4 b) with b / c = AR
    M = np.sin(np.outer(th, n)) * (mu * n[None, :] + np.sin(th)[:, None])
    An = np.linalg.solve(M, mu * np.sin(th))  # per radian of alpha
    delta = np.sum(n[1:] * (An[1:] / An[0]) ** 2)
    return {"e": 1.0 / (1.0 + delta), "CL_alpha": np.pi * AR * An[0]}


def trefftz_induced_drag(gamma_half, y_nodes_half, V, rho, S):
    """Induced drag from a VLM's spanwise circulation, evaluated in the Trefftz plane (flat wake).

    D_i = -rho/2 sum_j G_j w_j dy_j, with w the downwash of the trailing vortices. An independent check of
    OpenAeroStruct's near-field (Kutta-Joukowski) force integration.

    Parameters
    ----------
    gamma_half : array
        Strip circulations on the half wing (tip -> root) [m**2/s].
    y_nodes_half : array
        Node y (tip -> 0) [m].
    V, rho, S : float
        m/s, kg/m**3, m**2.

    Returns
    -------
    dict
        CL, CDi.
    """
    y = np.concatenate([y_nodes_half, -y_nodes_half[-2::-1]])
    g = np.concatenate([gamma_half, gamma_half[::-1]])
    trail = np.concatenate([[0.0], g, [0.0]])
    trail = trail[:-1] - trail[1:]  # trailing vortex at each node
    ym, dy = 0.5 * (y[1:] + y[:-1]), np.diff(y)
    w = np.array([np.sum(trail / (2.0 * np.pi * (yj - y))) for yj in ym])
    q = 0.5 * rho * V**2
    L = rho * V * np.sum(g * dy)
    Di = 0.5 * rho * np.sum(g * w * dy)
    return {"CL": abs(L) / (q * S), "CDi": abs(Di) / (q * S)}


def hand_wing_drag(seed, CL, Re, AR, e, settings=None):
    """Textbook wing drag estimate: CD = c_d,2D(c_l = CL, Re) + CL^2 / (pi e AR), with the 2D profile drag from
    NeuralFoil at the mean-chord Reynolds number.

    Parameters
    ----------
    seed : dict
        Kulfan dict.
    CL, Re, AR, e : float
    settings : NFSettings, optional

    Returns
    -------
    dict
        cd_2d, CDi, CD.
    """
    s = settings or NFSettings()
    P = pack(seed["upper_weights"], seed["lower_weights"], seed["leading_edge_weight"], seed["TE_thickness"])
    sol = solve_sections(P, np.array([CL]), np.array([Re]), s)
    cd = float(sol["Cd"][0])
    cdi = CL**2 / (np.pi * e * AR)
    return {"cd_2d": cd, "CDi": cdi, "CD": cd + cdi}
