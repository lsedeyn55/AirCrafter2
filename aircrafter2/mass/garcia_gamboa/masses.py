"""Component masses and mass summation, Garcia & Gamboa (2022) Section 2.2.

The mass equations use the wing mean geometric chord ``c_mean`` (Sec. 2.2.2,
last paragraph).
"""

import numpy as np

from ...analytic import AnalyticComponent, D


class WingMass(AnalyticComponent):
    """Circular tube spar wing mass (Sec. 2.2.5).

    Eq. 30  m_tube = pi (r^2 - (r - t)^2) b rho_tube
    Eq. 26  m_rib  = (n_rib t_rib) K_A,rib (t/c) c^2 rho_rib    (Eq. 32 text says "Eq. 23": typo, PLAN G4)
    Eq. 27  m_te   = K_A,te (t/c) c^2 b rho_te
    Eq. 31  m_le   = K_A,le (t/c) c^2 b rho_le
    Eq. 28  m_film = gamma_film K_P c b                          (gamma = t_film rho_film)
    Eq. 32  m_wing = f_wing^(n_panels - 1) (sum) (1 + S_winglet/S_wing)

    The exponent reading of Eq. 32 is confirmed by Tables 14-15 (PLAN G5).
    Ribs and LE/TE stringers are balsa (Table 13).
    """

    def initialize(self):
        self.options.declare("n_panels", default=1, types=int)

    def io(self):
        ins = {
            "r_tube": (0.012, "m"),
            "t_tube": (5e-4, "m"),
            "b": (3.0, "m"),
            "c_mean": (0.3, "m"),
            "tc": (0.12, None),
            "K_A_rib": (0.55, None),
            "K_A_le": (0.013, None),
            "K_A_te": (0.013, None),
            "K_P": (2.07, None),
            "nt_rib": (5e-3, "m"),
            "rho_spar": (1600.0, "kg/m**3"),
            "rho_balsa": (200.0, "kg/m**3"),
            "gamma_film": (0.036, "kg/m**2"),
            "f_wing": (1.12, None),
            "winglet_ratio": (0.0, None),
        }
        outs = {name: (0.1, "kg") for name in ["m_tube", "m_rib", "m_le", "m_te", "m_film", "m_wing_bare", "m_wing"]}
        return ins, outs

    def evaluate(self, x):
        r, t, b, c, tc = x["r_tube"], x["t_tube"], x["b"], x["c_mean"], x["tc"]
        rs, rb, gam = x["rho_spar"], x["rho_balsa"], x["gamma_film"]
        pi = np.pi

        ring = pi * (2.0 * r * t - t**2)
        m_tube = ring * b * rs  # Eq. 30
        g_tube = D({"r_tube": 2.0 * pi * t * b * rs, "t_tube": 2.0 * pi * (r - t) * b * rs,
                    "b": ring * rs, "rho_spar": ring * b})

        nt, Kr = x["nt_rib"], x["K_A_rib"]
        m_rib = nt * Kr * tc * c**2 * rb  # Eq. 26
        g_rib = D({"nt_rib": Kr * tc * c**2 * rb, "K_A_rib": nt * tc * c**2 * rb, "tc": nt * Kr * c**2 * rb,
                   "c_mean": 2.0 * nt * Kr * tc * c * rb, "rho_balsa": nt * Kr * tc * c**2})

        def stringer(kname):  # Eqs. 27, 31
            K = x[kname]
            m = K * tc * c**2 * b * rb
            g = D({kname: tc * c**2 * b * rb, "tc": K * c**2 * b * rb, "c_mean": 2.0 * K * tc * c * b * rb,
                   "b": K * tc * c**2 * rb, "rho_balsa": K * tc * c**2 * b})
            return m, g

        m_le, g_le = stringer("K_A_le")
        m_te, g_te = stringer("K_A_te")

        Kp = x["K_P"]
        m_film = gam * Kp * c * b  # Eq. 28
        g_film = D({"gamma_film": Kp * c * b, "K_P": gam * c * b, "c_mean": gam * Kp * b, "b": gam * Kp * c})

        bare = m_tube + m_rib + m_le + m_te + m_film
        g_bare = g_tube + g_rib + g_le + g_te + g_film

        npan = self.options["n_panels"]
        f, wr = x["f_wing"], x["winglet_ratio"]
        F = f ** (npan - 1)
        dF = (npan - 1) * f ** (npan - 2) if npan > 1 else 0.0 * f
        m_wing = F * bare * (1.0 + wr)  # Eq. 32
        g_wing = (F * (1.0 + wr)) * g_bare + D({"f_wing": dF * bare * (1.0 + wr), "winglet_ratio": F * bare})

        return ({"m_tube": m_tube, "m_rib": m_rib, "m_le": m_le, "m_te": m_te, "m_film": m_film,
                 "m_wing_bare": bare, "m_wing": m_wing},
                {"m_tube": g_tube, "m_rib": g_rib, "m_le": g_le, "m_te": g_te, "m_film": g_film,
                 "m_wing_bare": g_bare, "m_wing": g_wing})


class TailMass(AnalyticComponent):
    """Eq. 34  m_ht = f_ht S_ht/S_wing m_wing;  Eq. 35  m_vt = f_vt S_vt/S_wing m_wing."""

    def io(self):
        ins = {
            "m_wing": (0.7, "kg"),
            "S_wing": (0.9, "m**2"),
            "S_ht": (0.1, "m**2"),
            "S_vt": (0.08, "m**2"),
            "f_ht": (0.6411, None),
            "f_vt": (0.3414, None),
        }
        return ins, {"m_ht": (0.05, "kg"), "m_vt": (0.03, "kg")}

    def evaluate(self, x):
        mw, S = x["m_wing"], x["S_wing"]
        values, grads = {}, {}
        for surf in ("ht", "vt"):
            f, Ss = x[f"f_{surf}"], x[f"S_{surf}"]
            m = f * Ss / S * mw
            values[f"m_{surf}"] = m
            grads[f"m_{surf}"] = D({f"f_{surf}": Ss / S * mw, f"S_{surf}": f / S * mw,
                                    "S_wing": -m / S, "m_wing": f * Ss / S})
        return values, grads


class FuselageMass(AnalyticComponent):
    """Eq. 36  m_tb  = f_tb pi (r_tb^2 - (r_tb - t_tb)^2) l_tb rho_tb
    Eq. 37  m_cb  = f_cb S_cb
    Eq. 38  m_fus = m_tb + m_cb
    """

    def io(self):
        ins = {
            "r_tb": (0.015, "m"),
            "t_tb": (2e-4, "m"),
            "l_tb": (1.5, "m"),
            "rho_tb": (1600.0, "kg/m**3"),
            "f_tb": (12.81, None),
            "f_cb": (0.7878, "kg/m**2"),
            "S_cb": (0.2, "m**2"),
        }
        return ins, {"m_tb": (0.1, "kg"), "m_cb": (0.1, "kg"), "m_fus": (0.2, "kg")}

    def evaluate(self, x):
        r, t, l, rho, f = x["r_tb"], x["t_tb"], x["l_tb"], x["rho_tb"], x["f_tb"]
        ring = np.pi * (2.0 * r * t - t**2)
        m_tb = f * ring * l * rho
        g_tb = D({"r_tb": f * 2.0 * np.pi * t * l * rho, "t_tb": f * 2.0 * np.pi * (r - t) * l * rho,
                  "l_tb": f * ring * rho, "rho_tb": f * ring * l, "f_tb": ring * l * rho})
        m_cb = x["f_cb"] * x["S_cb"]
        g_cb = D({"f_cb": x["S_cb"], "S_cb": x["f_cb"]})
        return ({"m_tb": m_tb, "m_cb": m_cb, "m_fus": m_tb + m_cb},
                {"m_tb": g_tb, "m_cb": g_cb, "m_fus": g_tb + g_cb})


class GearMass(AnalyticComponent):
    """Eq. 39  m_gear = f_gear m   (m: total mass, the coupling variable of the closure)."""

    def io(self):
        return {"m": (12.0, "kg"), "f_gear": (0.0089, None)}, {"m_gear": (0.1, "kg")}

    def evaluate(self, x):
        return {"m_gear": x["f_gear"] * x["m"]}, {"m_gear": D({"f_gear": x["m"], "m": x["f_gear"]})}


class MassSum(AnalyticComponent):
    """Eq. 40  m_empty = m_wing + m_ht + m_vt + m_fus + m_gear + m_sys
    Eq. 41  m_total = m_empty + m_pay

    The closure m = m_total is enforced by the enclosing solver, not here.
    """

    PARTS = ["m_wing", "m_ht", "m_vt", "m_fus", "m_gear", "m_sys"]

    def io(self):
        ins = {name: (0.1, "kg") for name in self.PARTS + ["m_pay"]}
        return ins, {"m_empty": (2.0, "kg"), "m_total": (5.0, "kg")}

    def evaluate(self, x):
        m_empty = sum(x[p] for p in self.PARTS)
        g_empty = D({p: 1.0 for p in self.PARTS})
        return ({"m_empty": m_empty, "m_total": m_empty + x["m_pay"]},
                {"m_empty": g_empty, "m_total": g_empty + D({"m_pay": 1.0})})
