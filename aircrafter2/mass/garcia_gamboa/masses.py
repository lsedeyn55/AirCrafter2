"""Component masses and the mass summation, Garcia & Gamboa (G&G 2022) Section 2.2.

The last step of the mass group: takes wall thicknesses from the sizing and section coefficients, and gives m_total to
the closure. The mass equations use the wing mean geometric chord c_mean (Sec. 2.2.2, last paragraph).
BUILD_NOTE, below, explains f_build (docs/DECISIONS.md#rr-f-build). See docs/modules/mass.md.
"""

import numpy as np

from ...analytic import AnalyticComponent, D


BUILD_NOTE = """f_build (default 1, not G&G) is one multiplier on the whole structural mass, i.e. how heavy the airframe
comes out when built (docs/development/RDO_REDUCED_PLAN.md). It scales m_wing, m_tb, m_cb and m_gear, and the tails: with
tail_mass_model gg they follow m_wing (m_ht, m_vt = f S/S_wing m_wing), with areal (m = a S) they are scaled
directly. It acts inside the mass closure: the loads, the sizing, the total mass and the CG all see it. The breakdown
of the bare wing (m_tube ... m_film, m_wing_bare) is not scaled. f_tb also scales the boom (overlap accepted: f_tb
carries the CG effect at the tail, f_build the overall build). Since the USYD calibration
(docs/studies/mass_calibration_usyd.md) the correction factors carry the construction bias, and f_build stays at 1
with its spread from the leave-one-out errors (PLACEHOLDERS Q39)."""


class WingMass(AnalyticComponent):
    """Circular tube spar wing mass (Sec. 2.2.5).

    Eq. 30  m_tube = pi (r^2 - (r - t)^2) b rho_tube
    Eq. 26  m_rib  = (n_rib t_rib) K_A,rib (t/c) c^2 rho_rib    (Eq. 32 text says "Eq. 23", a typo: plan-g4)
    Eq. 27  m_te   = K_A,te (t/c) c^2 b rho_te
    Eq. 31  m_le   = K_A,le (t/c) c^2 b rho_le
    Eq. 28  m_film = gamma_film K_P c b                          (gamma = t_film rho_film)
    Eq. 32  m_wing = f_wing^(n_panels - 1) (sum) (1 + S_winglet/S_wing)
            (times k_wing and f_build, not G&G: k_wing is the wing construction factor, fitted on the USYD
            aircraft whatever their n_panels (docs/studies/mass_calibration_usyd.md; default 1); f_build is the
            build-mass factor, see BUILD_NOTE)

    The exponent reading of Eq. 32 is confirmed by Tables 14-15 (docs/DECISIONS.md#plan-g5).
    Ribs and LE/TE stringers are balsa (Table 13).

    Partials are analytic.

    Options
    -------
    n_panels : int

    Inputs
    ------
    r_tube, t_tube, b, c_mean [m], tc, K_A_rib, K_A_le, K_A_te, K_P, nt_rib, densities. Full list in `io()`.

    Outputs
    -------
    m_tube, m_rib, m_le, m_te, m_film, m_wing_bare, m_wing [kg]. Full list in `io()`.

    Symbols
    -------
    r, t : tube radius and wall [m]
    K_A_*, K_P : section coefficients (section.py)
    f_wing, n_panels : joint factor per panel joint and panels
    k_wing, f_build : construction and build factors (not G&G)
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
            "k_wing": (1.0, None),
            "f_build": (1.0, None),
        }
        outs = {name: (0.1, "kg") for name in ["m_tube", "m_rib", "m_le", "m_te", "m_film", "m_wing_bare", "m_wing"]}
        return ins, outs

    def evaluate(self, x):
        r, t, b, c, tc = x["r_tube"], x["t_tube"], x["b"], x["c_mean"], x["tc"]
        rs, rb, gam = x["rho_spar"], x["rho_balsa"], x["gamma_film"]
        pi = np.pi

        ring = pi * (2.0 * r * t - t**2)
        m_tube = ring * b * rs  # Eq. 30
        g_tube = D(
            {
                "r_tube": 2.0 * pi * t * b * rs,
                "t_tube": 2.0 * pi * (r - t) * b * rs,
                "b": ring * rs,
                "rho_spar": ring * b,
            }
        )

        nt, Kr = x["nt_rib"], x["K_A_rib"]
        m_rib = nt * Kr * tc * c**2 * rb  # Eq. 26
        g_rib = D(
            {
                "nt_rib": Kr * tc * c**2 * rb,
                "K_A_rib": nt * tc * c**2 * rb,
                "tc": nt * Kr * c**2 * rb,
                "c_mean": 2.0 * nt * Kr * tc * c * rb,
                "rho_balsa": nt * Kr * tc * c**2,
            }
        )

        def stringer(kname):
            """Leading- or trailing-edge stringer mass (Eqs. 27, 31) and its gradient."""
            K = x[kname]
            m = K * tc * c**2 * b * rb
            g = D(
                {
                    kname: tc * c**2 * b * rb,
                    "tc": K * c**2 * b * rb,
                    "c_mean": 2.0 * K * tc * c * b * rb,
                    "b": K * tc * c**2 * rb,
                    "rho_balsa": K * tc * c**2 * b,
                }
            )
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
        fb, kw = x["f_build"], x["k_wing"]
        m_wing = kw * F * bare * (1.0 + wr) * fb  # Eq. 32 (x k_wing x f_build)
        g_wing = (kw * F * (1.0 + wr) * fb) * g_bare + D(
            {
                "f_wing": kw * dF * bare * (1.0 + wr) * fb,
                "winglet_ratio": kw * F * bare * fb,
                "k_wing": F * bare * (1.0 + wr) * fb,
                "f_build": kw * F * bare * (1.0 + wr),
            }
        )

        return (
            {
                "m_tube": m_tube,
                "m_rib": m_rib,
                "m_le": m_le,
                "m_te": m_te,
                "m_film": m_film,
                "m_wing_bare": bare,
                "m_wing": m_wing,
            },
            {
                "m_tube": g_tube,
                "m_rib": g_rib,
                "m_le": g_le,
                "m_te": g_te,
                "m_film": g_film,
                "m_wing_bare": g_bare,
                "m_wing": g_wing,
            },
        )


class TailMass(AnalyticComponent):
    """Tail masses.

    tail_mass_model gg (default, G&G):
        Eq. 34  m_ht = f_ht S_ht/S_wing m_wing;  Eq. 35  m_vt = f_vt S_vt/S_wing m_wing
    tail_mass_model areal (not G&G): m_ht = a_ht S_ht f_build;  m_vt = a_vt S_vt f_build  (a in kg/m^2)

    The USYD tails don't scale with their wings, but their areal masses are consistent
    (docs/studies/mass_calibration_usyd.md). In gg mode f_build reaches the tails through m_wing; in areal mode it
    multiplies them directly (BUILD_NOTE). Partials are analytic.

    Options
    -------
    tail_mass_model : "gg" or "areal"

    Outputs
    -------
    m_ht, m_vt [kg]
    """

    def initialize(self):
        self.options.declare("tail_mass_model", default="gg", values=["gg", "areal"])

    def io(self):
        if self.options["tail_mass_model"] == "areal":
            # S_wing is kept (unused) so the group's interface is the same in both modes (it is promoted by name)
            ins = {
                "S_wing": (0.9, "m**2"),
                "S_ht": (0.1, "m**2"),
                "S_vt": (0.08, "m**2"),
                "a_ht": (1.0, "kg/m**2"),
                "a_vt": (1.0, "kg/m**2"),
                "f_build": (1.0, None),
            }
        else:
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
        values, grads = {}, {}
        if self.options["tail_mass_model"] == "areal":
            fb = x["f_build"]
            for surf in ("ht", "vt"):
                a, Ss = x[f"a_{surf}"], x[f"S_{surf}"]
                values[f"m_{surf}"] = a * Ss * fb
                grads[f"m_{surf}"] = D({f"a_{surf}": Ss * fb, f"S_{surf}": a * fb, "f_build": a * Ss})
            return values, grads
        mw, S = x["m_wing"], x["S_wing"]
        for surf in ("ht", "vt"):
            f, Ss = x[f"f_{surf}"], x[f"S_{surf}"]
            m = f * Ss / S * mw
            values[f"m_{surf}"] = m
            grads[f"m_{surf}"] = D(
                {f"f_{surf}": Ss / S * mw, f"S_{surf}": f / S * mw, "S_wing": -m / S, "m_wing": f * Ss / S}
            )
        return values, grads


class FuselageMass(AnalyticComponent):
    """Tail boom, cargo bay and fuselage mass. Partials are analytic.

    Eq. 36  m_tb  = f_tb pi (r_tb^2 - (r_tb - t_tb)^2) l_tb rho_tb
    Eq. 37  m_cb  = f_cb S_cb  (the equation as printed: its text also mentions t_cb and rho_cb,
    docs/DECISIONS.md#plan-g13) Eq. 38  m_fus = m_tb + m_cb
    (m_tb and m_cb times f_build, not G&G: see BUILD_NOTE)

    Outputs
    -------
    m_tb, m_cb, m_fus [kg]
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
            "f_build": (1.0, None),
        }
        return ins, {"m_tb": (0.1, "kg"), "m_cb": (0.1, "kg"), "m_fus": (0.2, "kg")}

    def evaluate(self, x):
        r, t, l, rho, f, fb = x["r_tb"], x["t_tb"], x["l_tb"], x["rho_tb"], x["f_tb"], x["f_build"]
        ring = np.pi * (2.0 * r * t - t**2)
        m_tb = f * ring * l * rho * fb
        g_tb = D(
            {
                "r_tb": f * 2.0 * np.pi * t * l * rho * fb,
                "t_tb": f * 2.0 * np.pi * (r - t) * l * rho * fb,
                "l_tb": f * ring * rho * fb,
                "rho_tb": f * ring * l * fb,
                "f_tb": ring * l * rho * fb,
                "f_build": f * ring * l * rho,
            }
        )
        m_cb = x["f_cb"] * x["S_cb"] * fb
        g_cb = D({"f_cb": x["S_cb"] * fb, "S_cb": x["f_cb"] * fb, "f_build": x["f_cb"] * x["S_cb"]})
        return ({"m_tb": m_tb, "m_cb": m_cb, "m_fus": m_tb + m_cb}, {"m_tb": g_tb, "m_cb": g_cb, "m_fus": g_tb + g_cb})


class GearMass(AnalyticComponent):
    """Eq. 39  m_gear = f_gear m   (m: total mass, the coupling variable of the closure; times f_build, not G&G).

    Partials are analytic.
    """

    def io(self):
        return {"m": (12.0, "kg"), "f_gear": (0.0089, None), "f_build": (1.0, None)}, {"m_gear": (0.1, "kg")}

    def evaluate(self, x):
        fb = x["f_build"]
        return (
            {"m_gear": x["f_gear"] * x["m"] * fb},
            {"m_gear": D({"f_gear": x["m"] * fb, "m": x["f_gear"] * fb, "f_build": x["f_gear"] * x["m"]})},
        )


class MassSum(AnalyticComponent):
    """Empty and total mass. Partials are analytic.

    Eq. 40  m_empty = m_wing + m_ht + m_vt + m_fus + m_gear + m_sys (+ m_ballast)
    Eq. 41  m_total = m_empty + m_pay

    m_ballast (default 0) isn't in G&G: it's the robustness hook for "mass + X %" worst cases. The closure m = m_total
    is enforced by the enclosing solver, not here.

    Outputs
    -------
    m_empty, m_total [kg]
    """

    PARTS = ["m_wing", "m_ht", "m_vt", "m_fus", "m_gear", "m_sys", "m_ballast"]

    def io(self):
        ins = {name: (0.1, "kg") for name in self.PARTS + ["m_pay"]}
        return ins, {"m_empty": (2.0, "kg"), "m_total": (5.0, "kg")}

    def evaluate(self, x):
        m_empty = sum(x[p] for p in self.PARTS)
        g_empty = D({p: 1.0 for p in self.PARTS})
        return (
            {"m_empty": m_empty, "m_total": m_empty + x["m_pay"]},
            {"m_empty": g_empty, "m_total": g_empty + D({"m_pay": 1.0})},
        )
