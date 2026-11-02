"""Low-fidelity longitudinal static stability, used by full_mdo: neutral point, CG and static margin.

Takes the masses, stations, tail geometry and the wing's lift slope a_w after the MDA, and feeds the SM constraint.
Stick-fixed, wing + tail, fuselage contribution neglected, e.g. Nelson, Flight Stability and Automatic Control, Ch. 2;
Raymer, Ch. 16. An AVL replacement later only has to provide x_np (and could add lateral-directional derivatives).
Details: docs/modules/stability.md.
Abbreviations: MDA = multidisciplinary analysis, CG = centre of gravity.
"""

import numpy as np
import openmdao.api as om

MASS_STATIONS = [  # (mass input, station input)
    ("m_wing", "x_wing"),
    ("m_tail", "x_tail"),
    ("m_tb", "x_tb"),
    ("m_cb", "x_cb"),
    ("m_pay", "x_pay"),
    ("m_motor_group", "x_motor"),
    ("m_batt", "x_batt"),
    ("m_avionics", "x_avionics"),
    ("m_gear", "x_gear"),
]


class StaticStability(om.ExplicitComponent):
    """Neutral point, CG and static margin. Partials are complex step: short expressions.

        x_np = x_ac,w + eta_h V_h (a_h / a_w) (1 - d eps / d alpha) c
        V_h  = S_ht l_ht / (S c)
        a_h  = 2 pi A_h / (2 + sqrt(A_h^2 + 4))        Helmbold (Raymer Ch. 12, no sweep, Mach ~ 0)
        d eps / d alpha = 2 a_w / (pi A)               elliptic-loading downwash estimate

    The CG is the mass-weighted mean of the component stations (origin: wing root leading edge, x aft). SM = (x_np -
    x_cg) / c.

    Inputs
    ------
    c [m], S [m**2], AR, a_w [1/rad], S_ht [m**2], AR_ht, l_ht [m], eta_h
    Each mass and station in MASS_STATIONS [kg, m]

    Outputs
    -------
    x_np, x_cg [m], SM, V_h, a_h [1/rad], downwash_grad

    Symbols
    -------
    a_w, a_h : wing and tail lift-curve slopes [1/rad]; a_w is the coupled C_L,alpha from the VLM (vortex-lattice
    method)
    eta_h : tail dynamic-pressure ratio
    deps : downwash gradient, d eps / d alpha
    """

    def setup(self):
        for name, val, units in [
            ("c", 0.25, "m"),
            ("S", 0.375, "m**2"),
            ("AR", 6.0, None),
            ("a_w", 4.5, "1/rad"),
            ("S_ht", 0.06, "m**2"),
            ("AR_ht", 4.0, None),
            ("l_ht", 0.8, "m"),
            ("eta_h", 0.9, None),
        ]:
            self.add_input(name, val, units=units)
        for m_name, x_name in MASS_STATIONS:
            self.add_input(m_name, 0.1, units="kg")
            self.add_input(x_name, 0.1, units="m")
        for name, units in [
            ("x_np", "m"),
            ("x_cg", "m"),
            ("SM", None),
            ("V_h", None),
            ("a_h", "1/rad"),
            ("downwash_grad", None),
        ]:
            self.add_output(name, 0.1, units=units)
        self.declare_partials("*", "*", method="cs")

    def compute(self, inputs, outputs):
        vals = {k: inputs[k] for k in inputs}
        c = vals["c"]
        Ah = vals["AR_ht"]
        a_h = 2 * np.pi * Ah / (2 + np.sqrt(Ah**2 + 4))
        deps = 2 * vals["a_w"] / (np.pi * vals["AR"])
        V_h = vals["S_ht"] * vals["l_ht"] / (vals["S"] * c)
        x_np = 0.25 * c + vals["eta_h"] * V_h * a_h / vals["a_w"] * (1 - deps) * c
        m_tot = sum(vals[m] for m, _ in MASS_STATIONS)
        x_cg = sum(vals[m] * vals[s] for m, s in MASS_STATIONS) / m_tot
        outputs["x_np"] = x_np
        outputs["x_cg"] = x_cg
        outputs["SM"] = (x_np - x_cg) / c
        outputs["V_h"] = V_h
        outputs["a_h"] = a_h
        outputs["downwash_grad"] = deps


def oei_check(prob, cfg):
    """One-engine-inoperative yaw, twin layout: report-only post-check (not a constraint; docs/DECISIONS.md#plan-d8).

    The live unit gives a yawing moment N = T_unit y_m, using the static thrust per unit (the worst case).
    Windmilling drag of the dead prop is neglected. Required rudder at the check speed
    V = k V_stall (k = oei_speed_factor), from a tail-volume rudder model:

        C_n,req   = N / (q S b)
        C_n,dr    = eta_v V_v a_v tau_r            (per rad of rudder; a_v Helmbold on AR_vt)
        delta_req = C_n,req / C_n,dr

    eta_v, tau_r, delta_max and k are all placeholders (StabilityInputs).

    Parameters
    ----------
    prob : om.Problem
        A run problem.
    cfg : FullMDOConfig

    Returns
    -------
    dict or None
        N_yaw_Nm, V_check_mps, Cn_req, Cn_per_rad_rudder, rudder_req_deg, rudder_max_deg, rudder_margin_deg, ok.
        None for n_motors = 1.
    """
    if cfg.propulsion.n_motors != 2:
        return None
    st = cfg.stability

    def val(k):
        """One scalar output of the problem."""
        return float(np.ravel(prob.get_val(k))[0])

    T, ym, S, b = val("T_static_unit"), val("y_m"), val("S"), val("b")
    rho, W, CLmax, V_v = val("rho"), val("W"), val("cruise.CL_max"), val("V_v")
    V_stall = np.sqrt(2 * W / (rho * S * CLmax))
    V = st.oei_speed_factor * V_stall
    q = 0.5 * rho * V**2
    a_v = 2 * np.pi * st.AR_vt / (2 + np.sqrt(st.AR_vt**2 + 4))
    N = T * ym
    Cn_req = N / (q * S * b)
    Cn_dr = st.eta_v * V_v * a_v * st.rudder_tau
    d_req = np.degrees(Cn_req / Cn_dr)
    return {
        "N_yaw_Nm": N,
        "V_check_mps": V,
        "Cn_req": Cn_req,
        "Cn_per_rad_rudder": Cn_dr,
        "rudder_req_deg": d_req,
        "rudder_max_deg": st.rudder_max_deg,
        "rudder_margin_deg": st.rudder_max_deg - d_req,
        "ok": bool(d_req <= st.rudder_max_deg),
    }
