"""Independent plain-Python implementation of the Garcia & Gamboa (2022) tube-spar mass model.

Written separately from the OpenMDAO components, directly from the paper's
equations, with hard max() and a fixed-point loop for the mass closure. Used
only to cross-check the OpenMDAO group.

It supports the same interpretation switches as the package (see docs/PLAN.md §7)
but none of the smoothing (hard max only; no minimum gauge).
"""

import math


def reference_mass(p, deflection="paper", torsion="paper", boom="paper", radius="paper",
                   n_panels=1, include_twist=False, iters=200):
    sf = p["SF"]
    sig_s, tau_s = p["sigma_spar"] / sf, p["tau_spar"] / sf
    sig_b, tau_b = p["sigma_balsa"] / sf, p["tau_balsa"] / sf
    sig_tb, tau_tb = p["sigma_tb"] / sf, p["tau_tb"] / sf
    b, c, cr, tc = p["b"], p["c_mean"], p["c_root"], p["tc"]
    r = p["r_tube"]
    q = 0.5 * p["rho"] * p["V_design"] ** 2
    q_div = 0.5 * p["rho"] * (p["div_speed_factor"] * p["V_design"]) ** 2
    Cm = abs(p["Cm"])
    n = p["n"]

    # section (Eqs. 9-11, 15, 16)
    K_A = p["A_c2"] / tc
    A = p["A_c2"] * c * c
    K_A_le, K_A_te = p["le_ratio"] * K_A, p["te_ratio"] * K_A
    K_A_rib = (1 - p["le_ratio"] - p["te_ratio"] - math.pi * r * r / A) * K_A
    r_s = r * cr / c if radius == "paper" else r

    m = 10.0
    for _ in range(iters):
        W = m * p["g"]
        # loads at the root (Eqs. 44-53)
        Qz = max(n * W / 2, W / 2)
        Mx = max(n * W * b / 8, W * b / 4)
        T_fl = q * Cm * c * c * b / 2
        My = max(T_fl, W / 2 * abs(0.25 - p["x_gd"]) * c)
        # tube spar (Eqs. 107-118)
        t_sig = Mx / (sig_s * math.pi * r_s**2)
        y1 = p["y1"]
        if deflection == "paper":
            poly = 3 * b**4 + 24 * b**3 * y1 - 120 * b**2 * y1**2 + 224 * b * y1**3 - 112 * y1**4
        else:
            poly = 3 * (b**4 + 4 * b**3 * y1 - 20 * b**2 * y1**2 + 32 * b * y1**3 - 16 * y1**4)
        t_del = n * W * poly / (384 * b * p["E_spar"] * p["delta_tip_ratio"] * b * math.pi * r_s**3)
        t_tau = 3 * Qz / (4 * tau_s * math.pi * r_s)
        t_T = My / (2 * math.pi * r_s**2 * tau_s)
        denom = 16 if torsion == "paper" else 8
        t_div = max(0.0, b * b * c * c * q_div * (p["x_spar"] - 0.25) * p["CL_alpha"]
                    / (denom * math.pi * r_s**3 * p["G_spar"]))
        tors = [t_T, t_div]
        if include_twist:
            tors.append(b * T_fl / (denom * math.pi * r_s**3 * p["G_spar"] * p["theta_tip_max"]))
        t = max(t_sig, t_del) + t_tau + max(tors)
        # ribs (Eqs. 100, 103, 104), root chord
        f = p["c_rib_frac"]
        nt = max(600 * n * W * f**2 / (391 * tau_b * tc * cr),
                 2400 * n * W * cr * f**3 / (1173 * sig_b * tc**2 * cr**2))
        # wing mass (Eqs. 26-32)
        m_tube = math.pi * (r**2 - (r - t) ** 2) * b * p["rho_spar"]
        m_rib = nt * K_A_rib * tc * c * c * p["rho_balsa"]
        m_le = K_A_le * tc * c * c * b * p["rho_balsa"]
        m_te = K_A_te * tc * c * c * b * p["rho_balsa"]
        m_film = p["gamma_film"] * p["P_c"] * c * b
        m_wing = p["f_wing"] ** (n_panels - 1) * (m_tube + m_rib + m_le + m_te + m_film) * (1 + p["winglet_ratio"])
        # tails (Eqs. 34, 35)
        m_ht = p["f_ht"] * p["S_ht"] / p["S_wing"] * m_wing
        m_vt = p["f_vt"] * p["S_vt"] / p["S_wing"] * m_wing
        # tail boom (Eqs. 54-56, 119-123)
        if boom == "paper":
            K_ht = p["l_ht"] / c * (b / c)  # Eq. 5 as printed
            K_tb = p["l_tb"] / c * (b / c)  # Eq. 6 as printed
            Q_tb = q * Cm * b * c / K_ht  # Eq. 55
            M_tb = q * Cm * K_tb * b * c * c / K_ht  # Eq. 56
        else:
            Q_tb = q * Cm * b * c * c / p["l_ht"]  # Eq. 54
            M_tb = Q_tb * p["l_tb"]
        rtb, ltb = p["r_tb"], p["l_tb"]
        t_tb = max(M_tb / (sig_tb * math.pi * rtb**2),
                   Q_tb * ltb**2 / (2 * p["E_tb"] * math.pi * rtb**3 * p["theta_tb_max"])) \
            + 3 * Q_tb / (4 * tau_tb * math.pi * rtb)
        m_tb = p["f_tb"] * math.pi * (rtb**2 - (rtb - t_tb) ** 2) * ltb * p["rho_tb"]
        m_cb = p["f_cb"] * p["S_cb"]
        m_gear = p["f_gear"] * m
        m_empty = m_wing + m_ht + m_vt + m_tb + m_cb + m_gear + p["m_sys"]
        m_new = m_empty + p["m_pay"]
        if abs(m_new - m) < 1e-13:
            m = m_new
            break
        m = m_new
    return dict(m_tube=m_tube, m_rib=m_rib, m_le=m_le, m_te=m_te, m_film=m_film, m_wing=m_wing, m_ht=m_ht,
                m_vt=m_vt, m_tb=m_tb, m_cb=m_cb, m_gear=m_gear, m_empty=m_empty, m_total=m, t_tube=t, t_tb=t_tb)
