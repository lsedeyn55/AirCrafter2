"""Low-fidelity longitudinal static stability (step 7).

Neutral point (stick-fixed, wing + tail; fuselage contribution neglected), e.g. Nelson,
Flight Stability and Automatic Control, Ch. 2; Raymer, Ch. 16:

    x_np = x_ac,w + eta_h V_h (a_h / a_w) (1 - d eps / d alpha) c
    V_h  = S_ht l_ht / (S c)
    a_h  = 2 pi A_h / (2 + sqrt(A_h^2 + 4))        Helmbold (Raymer Ch. 12, no sweep, Mach ~ 0)
    d eps / d alpha = 2 a_w / (pi A)               elliptic-loading downwash estimate

a_w is the coupled wing C_L,alpha from the VLM. The CG is the mass-weighted mean of the component
stations (origin: wing root leading edge, x aft). SM = (x_np - x_cg) / c.

Interface for an AVL replacement later: inputs (geometry, masses, stations, a_w), outputs
(x_np, x_cg, SM). A future AVLStability component only has to provide x_np (and could add
lateral-directional derivatives).
"""

import numpy as np
import openmdao.api as om

MASS_STATIONS = [  # (mass input, station input)
    ("m_wing", "x_wing"), ("m_tail", "x_tail"), ("m_tb", "x_tb"), ("m_cb", "x_cb"), ("m_pay", "x_pay"),
    ("m_motor_group", "x_motor"), ("m_batt", "x_batt"), ("m_avionics", "x_avionics"), ("m_gear", "x_gear"),
]


class StaticStability(om.ExplicitComponent):
    def setup(self):
        for name, val, units in [
            ("c", 0.25, "m"), ("S", 0.375, "m**2"), ("AR", 6.0, None), ("a_w", 4.5, "1/rad"),
            ("S_ht", 0.06, "m**2"), ("AR_ht", 4.0, None), ("l_ht", 0.8, "m"), ("eta_h", 0.9, None),
        ]:
            self.add_input(name, val, units=units)
        for m_name, x_name in MASS_STATIONS:
            self.add_input(m_name, 0.1, units="kg")
            self.add_input(x_name, 0.1, units="m")
        for name, units in [("x_np", "m"), ("x_cg", "m"), ("SM", None), ("V_h", None), ("a_h", "1/rad"),
                            ("downwash_grad", None)]:
            self.add_output(name, 0.1, units=units)
        self.declare_partials("*", "*", method="cs")

    def compute(self, inputs, outputs):
        x = {k: inputs[k] for k in inputs}
        c = x["c"]
        Ah = x["AR_ht"]
        a_h = 2 * np.pi * Ah / (2 + np.sqrt(Ah**2 + 4))
        deps = 2 * x["a_w"] / (np.pi * x["AR"])
        V_h = x["S_ht"] * x["l_ht"] / (x["S"] * c)
        x_np = 0.25 * c + x["eta_h"] * V_h * a_h / x["a_w"] * (1 - deps) * c
        m_tot = sum(x[m] for m, _ in MASS_STATIONS)
        x_cg = sum(x[m] * x[s] for m, s in MASS_STATIONS) / m_tot
        outputs["x_np"] = x_np
        outputs["x_cg"] = x_cg
        outputs["SM"] = (x_np - x_cg) / c
        outputs["V_h"] = V_h
        outputs["a_h"] = a_h
        outputs["downwash_grad"] = deps
