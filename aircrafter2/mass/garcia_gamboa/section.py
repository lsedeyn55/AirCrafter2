"""Wing cross-section geometric coefficients and the spar-tube radius (Garcia & Gamboa 2022, Section 2.2.2).

Input convention: A_c2 is the aerofoil area over chord squared (A/c^2) and P_c the perimeter over chord (P/c). These are
what Tables 4, 8 and 12 actually list, and what the airfoil module computes from coordinates; K_A follows Eq. 9,
A / ((t/c) c^2) (docs/DECISIONS.md#plan-g2). Feeds the sizing and the wing mass. See docs/modules/mass.md.
"""

import numpy as np

from ...analytic import AnalyticComponent, D


class SectionCoeffs(AnalyticComponent):
    """K_A (Eq. 9), K_P (Eq. 10), K_A,le and K_A,te (Eq. 11), K_A,rib (Eqs. 11, 16), K_tube (Eq. 15). Partials are
    analytic.

    Inputs
    ------
    A_c2, P_c, tc, le_ratio, te_ratio, r_tube [m], c_mean [m], c_root [m]

    Outputs
    -------
    K_A, K_P, K_A_le, K_A_te, rib_ratio, K_A_rib, K_tube

    Symbols
    -------
    A, P : A/c^2 and P/c
    tc : t/c
    le, te : A_le / A and A_te / A
    r : spar tube outer radius [m]
    c, cr : mean and root chord [m]
    hole : tube area / aerofoil area at the mean chord
    """

    def io(self):
        ins = {
            "A_c2": (0.08, None),
            "P_c": (2.07, None),
            "tc": (0.12, None),
            "le_ratio": (0.021, None),
            "te_ratio": (0.021, None),
            "r_tube": (0.01, "m"),
            "c_mean": (0.3, "m"),
            "c_root": (0.35, "m"),
        }
        outs = {
            "K_A": (0.6, None),
            "K_P": (2.0, None),
            "K_A_le": (0.01, None),
            "K_A_te": (0.01, None),
            "rib_ratio": (0.9, None),
            "K_A_rib": (0.5, None),
            "K_tube": (0.03, None),
        }
        return ins, outs

    def evaluate(self, x):
        A, P, tc = x["A_c2"], x["P_c"], x["tc"]
        le, te, r, c, cr = x["le_ratio"], x["te_ratio"], x["r_tube"], x["c_mean"], x["c_root"]

        K_A = A / tc  # Eq. 9
        gK_A = D({"A_c2": 1.0 / tc, "tc": -A / tc**2})

        # Eq. 16 as printed has a typo: its second A_rib should be A_le; corrected here (docs/DECISIONS.md#plan-g3). The
        # tube hole is taken relative to the aerofoil area at the mean chord, A = (A/c^2) c^2.
        hole = np.pi * r**2 / (A * c**2)
        rib = 1.0 - le - te - hole
        g_rib = D(
            {
                "le_ratio": -1.0,
                "te_ratio": -1.0,
                "r_tube": -2.0 * np.pi * r / (A * c**2),
                "A_c2": hole / A,
                "c_mean": 2.0 * hole / c,
            }
        )

        values = {
            "K_A": K_A,
            "K_P": P,  # Eq. 10
            "K_A_le": le * K_A,  # Eq. 11
            "K_A_te": te * K_A,  # Eq. 11
            "rib_ratio": rib,
            "K_A_rib": rib * K_A,  # Eq. 11 with Eq. 16
            "K_tube": r / cr,  # Eq. 15 (sizing uses the root chord, Sec. 3.3)
        }
        grads = {
            "K_A": gK_A,
            "K_P": D({"P_c": 1.0}),
            "K_A_le": D({"le_ratio": K_A}) + le * gK_A,
            "K_A_te": D({"te_ratio": K_A}) + te * gK_A,
            "rib_ratio": g_rib,
            "K_A_rib": rib * gK_A + K_A * g_rib,
            "K_tube": D({"r_tube": 1.0 / cr, "c_root": -r / cr**2}),
        }
        return values, grads


class TubeRadius(AnalyticComponent):
    """Derived spar-tube radius (docs/DECISIONS.md#plan-d1): r_tube = k_fill * (t/c at the spar) * c_root / 2 -
    clearance.

    Tube mass falls monotonically with r until minimum gauge binds (t_sigma ~ 1/r^2, t_delta ~ 1/r^3, m_tube ~ r t), so
    a free r_tube design variable would sit on this geometric limit at almost every optimum. Partials are analytic.

    Inputs
    ------
    t_at_spar, c_root [m], k_fill, clearance [m]

    Outputs
    -------
    r_tube [m]
    """

    def io(self):
        ins = {
            "t_at_spar": (0.12, None),
            "c_root": (0.35, "m"),
            "k_fill": (0.85, None),
            "clearance": (0.001, "m"),
        }
        return ins, {"r_tube": (0.015, "m")}

    def evaluate(self, x):
        t, c, k, clr = x["t_at_spar"], x["c_root"], x["k_fill"], x["clearance"]
        r = 0.5 * k * t * c - clr
        g = D({"t_at_spar": 0.5 * k * c, "c_root": 0.5 * k * t, "k_fill": 0.5 * t * c, "clearance": -1.0})
        return {"r_tube": r}, {"r_tube": g}


class TubeFitMargin(AnalyticComponent):
    """Constraint for r_tube_mode = design_variable: margin >= 0 means the tube fits. Partials are analytic.

    Inputs
    ------
    r_tube [m], t_at_spar, c_root [m], k_fill, clearance [m]

    Outputs
    -------
    tube_fit_margin [m]
    """

    def io(self):
        ins = {
            "r_tube": (0.01, "m"),
            "t_at_spar": (0.12, None),
            "c_root": (0.35, "m"),
            "k_fill": (0.85, None),
            "clearance": (0.001, "m"),
        }
        return ins, {"tube_fit_margin": (0.0, "m")}

    def evaluate(self, x):
        t, c, k, clr, r = x["t_at_spar"], x["c_root"], x["k_fill"], x["clearance"], x["r_tube"]
        m = 0.5 * k * t * c - clr - r
        g = D(
            {"t_at_spar": 0.5 * k * c, "c_root": 0.5 * k * t, "k_fill": 0.5 * t * c, "clearance": -1.0, "r_tube": -1.0}
        )
        return {"tube_fit_margin": m}, {"tube_fit_margin": g}
