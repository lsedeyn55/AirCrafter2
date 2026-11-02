"""Wing and tail-boom loads, Garcia & Gamboa (2022) Section 3.2.

All wing loads are evaluated at the wing root (y = 0), where they peak and
where the paper sizes the structure. ``c_mean`` is the mean geometric chord
(Sec. 3.2: "the parameter c refers to the wing mean geometric chord").
"""

from ...analytic import AnalyticComponent, D, combine
from ...smooth import ks_max, smooth_abs

_EPS_CM = 1e-6  # smoothing width for |Cm|
_EPS_X = 1e-6  # smoothing width for |x_c/4 - x_gd| (chord fraction)
_EPS_T = 1e-6  # N m, smoothing width for the signed ground torque with a nacelle term


class WingLoads(AnalyticComponent):
    """Flight (Eqs. 43-46) and ground-test (Eqs. 48-50) root loads, plus dynamic pressures.

    Eq. 43  w_z  = n W / b                     (uniform spanwise lift)
    Eq. 44  Q_fl = n W / b (b/2 - y)          -> n W / 2 at y = 0
    Eq. 45  M_fl = n W / (2b) (b/2 - y)^2     -> n W b / 8
    Eq. 46  T_fl = q |Cm| c^2 (b/2 - y)       -> q |Cm| c^2 b / 2
    Eq. 48  Q_gd = W / 2
    Eq. 49  M_gd = W / 2 (b/2 - y)            -> W b / 4
    Eq. 50  T_gd = W / 2 (x_c/4 - x_gd)       (magnitude; x_gd given as a chord fraction)

    q is the dynamic pressure at the design speed (Sec. 3.2.1). q_div uses the
    design speed times ``div_speed_factor`` (the paper suggests 1.2, after Eq. 47).

    Wing-mounted propulsion units (twin layout; not in G&G, derivation in docs/modules/mass.md and
    PLAN 6h). One unit of mass m_u per semi-span at (y_m, x_m), x from the LE (aft +), torsion about
    the quarter chord (as G&G, Eqs. 46 and 50), with ``n_motor_pairs = 1``:
      motor_inertia_relief (default off: the published model)
        Eq. 44  Q_fl = n W / 2 - n g m_u          Eq. 48  Q_gd = W / 2 - g m_u
        Eq. 45  M_fl = n W b / 8 - n g m_u y_m    Eq. 49  M_gd = W b / 4 - g m_u y_m
      nacelle_torsion (default on; a no-op without wing units)
        Eq. 46  T_fl = q |Cm| c^2 b / 2 + n g m_u (c/4 - x_m)      (both nose-down for Cm < 0)
        Eq. 50  T_gd = | W/2 c (1/4 - x_gd) - g m_u (c/4 - x_m) |  (signed sum, then magnitude)
    """

    def initialize(self):
        self.options.declare(
            "n_motor_pairs",
            default=0,
            values=[0, 1],
            desc="symmetric pairs of wing-mounted units (one unit per semi-span per pair)",
        )
        self.options.declare("motor_inertia_relief", default=False, types=bool)
        self.options.declare("nacelle_torsion", default=True, types=bool)

    def io(self):
        ins = {
            "m": (12.0, "kg"),
            "g": (9.80665, "m/s**2"),
            "n": (2.0, None),
            "b": (3.0, "m"),
            "c_mean": (0.3, "m"),
            "rho": (1.225, "kg/m**3"),
            "V_design": (15.0, "m/s"),
            "Cm": (-0.1, None),
            "x_gd": (0.3, None),
            "div_speed_factor": (1.2, None),
        }
        if self.options["n_motor_pairs"]:
            ins.update({"m_motor_unit": (0.25, "kg"), "y_m": (0.25, "m"), "x_motor": (-0.14, "m")})
        outs = {
            name: (1.0, u)
            for name, u in [
                ("W", "N"),
                ("q", "Pa"),
                ("q_div", "Pa"),
                ("Q_fl", "N"),
                ("M_fl", "N*m"),
                ("T_fl", "N*m"),
                ("Q_gd", "N"),
                ("M_gd", "N*m"),
                ("T_gd", "N*m"),
            ]
        }
        return ins, outs

    def evaluate(self, x):
        m, g, n, b, c = x["m"], x["g"], x["n"], x["b"], x["c_mean"]
        rho, V, k = x["rho"], x["V_design"], x["div_speed_factor"]

        W = m * g
        gW = D({"m": g, "g": m})

        q = 0.5 * rho * V**2
        gq = D({"rho": 0.5 * V**2, "V_design": rho * V})
        q_div = 0.5 * rho * (k * V) ** 2
        gq_div = D({"rho": 0.5 * (k * V) ** 2, "V_design": rho * k**2 * V, "div_speed_factor": rho * k * V**2})

        aCm, daCm = smooth_abs(x["Cm"], _EPS_CM)
        T_fl = 0.5 * q * aCm * c**2 * b  # Eq. 46, y = 0
        gT_fl = (0.5 * aCm * c**2 * b) * gq + D(
            {
                "Cm": 0.5 * q * c**2 * b * daCm,
                "c_mean": q * aCm * c * b,
                "b": 0.5 * q * aCm * c**2,
            }
        )

        arm, darm = smooth_abs(0.25 - x["x_gd"], _EPS_X)
        T_gd = 0.5 * W * arm * c  # Eq. 50

        values = {
            "W": W,
            "q": q,
            "q_div": q_div,
            "Q_fl": 0.5 * n * W,  # Eq. 44
            "M_fl": n * W * b / 8.0,  # Eq. 45
            "T_fl": T_fl,
            "Q_gd": 0.5 * W,  # Eq. 48
            "M_gd": 0.25 * W * b,  # Eq. 49
            "T_gd": T_gd,
        }
        grads = {
            "W": gW,
            "q": gq,
            "q_div": gq_div,
            "Q_fl": D({"n": 0.5 * W}) + (0.5 * n) * gW,
            "M_fl": D({"n": W * b / 8.0, "b": n * W / 8.0}) + (n * b / 8.0) * gW,
            "T_fl": gT_fl,
            "Q_gd": 0.5 * gW,
            "M_gd": D({"b": 0.25 * W}) + (0.25 * b) * gW,
            "T_gd": (0.5 * arm * c) * gW + D({"x_gd": -0.5 * W * c * darm, "c_mean": 0.5 * W * arm}),
        }
        if self.options["n_motor_pairs"]:
            self._wing_units(x, values, grads, W, gW, T_fl, gT_fl)
        return values, grads

    def _wing_units(self, x, values, grads, W, gW, T_fl, gT_fl):
        """Add the wing-mounted unit terms (one unit per semi-span) to the root loads in place."""
        g, n, c, mu, ym, xm = x["g"], x["n"], x["c_mean"], x["m_motor_unit"], x["y_m"], x["x_motor"]
        if self.options["motor_inertia_relief"]:
            # flight: inertial load n g m_u at y_m opposes the lift (Eqs. 44, 45)
            values["Q_fl"] = values["Q_fl"] - n * g * mu
            grads["Q_fl"] = grads["Q_fl"] + D({"n": -g * mu, "g": -n * mu, "m_motor_unit": -n * g})
            values["M_fl"] = values["M_fl"] - n * g * mu * ym
            grads["M_fl"] = grads["M_fl"] + D(
                {"n": -g * mu * ym, "g": -n * mu * ym, "m_motor_unit": -n * g * ym, "y_m": -n * g * mu}
            )
            # ground test: weight g m_u at y_m, tip reaction W/2 at b/2 (Eqs. 48, 49)
            values["Q_gd"] = values["Q_gd"] - g * mu
            grads["Q_gd"] = grads["Q_gd"] + D({"g": -mu, "m_motor_unit": -g})
            values["M_gd"] = values["M_gd"] - g * mu * ym
            grads["M_gd"] = grads["M_gd"] + D({"g": -mu * ym, "m_motor_unit": -g * ym, "y_m": -g * mu})
        if self.options["nacelle_torsion"]:
            arm = 0.25 * c - xm  # unit ahead of the quarter-chord axis: > 0
            garm = D({"c_mean": 0.25, "x_motor": -1.0})
            values["T_fl"] = T_fl + n * g * mu * arm  # Eq. 46 + nacelle (both nose-down)
            grads["T_fl"] = (
                gT_fl + D({"n": g * mu * arm, "g": n * mu * arm, "m_motor_unit": n * g * arm}) + (n * g * mu) * garm
            )
            # Eq. 50: signed (nose-up +) tip-reaction torque plus the unit's nose-down torque, then |.|
            sgd = 0.5 * W * c * (0.25 - x["x_gd"]) - g * mu * arm
            gsgd = (
                (0.5 * c * (0.25 - x["x_gd"])) * gW
                + D(
                    {
                        "c_mean": 0.5 * W * (0.25 - x["x_gd"]),
                        "x_gd": -0.5 * W * c,
                        "g": -mu * arm,
                        "m_motor_unit": -g * arm,
                    }
                )
                - (g * mu) * garm
            )
            T_gd, dT = smooth_abs(sgd, _EPS_T)
            values["T_gd"] = T_gd
            grads["T_gd"] = dT * gsgd


class CriticalLoads(AnalyticComponent):
    """Critical strength loads: max of flight and ground cases (Eqs. 51-53).

    Uses ks_max (log-space KS) or the hard max, per the ``max_mode`` option.
    """

    def initialize(self):
        self.options.declare("max_mode", default="ks", values=["ks", "hard"])
        self.options.declare("rho_ks", default=100.0)

    def io(self):
        ins = {
            name: (1.0, u)
            for name, u in [
                ("Q_fl", "N"),
                ("Q_gd", "N"),
                ("M_fl", "N*m"),
                ("M_gd", "N*m"),
                ("T_fl", "N*m"),
                ("T_gd", "N*m"),
            ]
        }
        outs = {"Qz": (1.0, "N"), "Mx": (1.0, "N*m"), "My": (1.0, "N*m")}
        return ins, outs

    def evaluate(self, x):
        mode, rho = self.options["max_mode"], self.options["rho_ks"]
        values, grads = {}, {}
        for out, (a, b) in {"Qz": ("Q_fl", "Q_gd"), "Mx": ("M_fl", "M_gd"), "My": ("T_fl", "T_gd")}.items():
            f, (wa, wb) = ks_max([x[a], x[b]], rho, mode)  # Eqs. 51, 52, 53
            values[out] = f
            grads[out] = combine([wa, wb], [D({a: 1.0}), D({b: 1.0})])
        return values, grads


class TailBoomLoads(AnalyticComponent):
    """Tail-boom shear and root bending moment (Eqs. 54-56).

    load_mode="paper" (default) reproduces the paper's Table 15 numbers. It uses Eq. 55,
    Q_tb = q |Cm| b c / K_ht, with K_ht = (l_ht / c) A taken literally from Eq. 5
    (A = b / c), which gives

        Q_tb = q |Cm| c^3 / l_ht                 (= Eq. 54 divided by A)
        M_tb = q |Cm| b c^2 l_tb / l_ht          (Eq. 56; A cancels)

    So the shear is A times smaller than the moment balance of Eq. 54 implies, and
    M_tb != Q_tb l_tb. This reading reproduces the Table 15 tail-boom masses of all three
    tube-spar aircraft to ~1 % (docs/verification/mass.md). f_tb in Table 14 was
    calibrated with it, so it stays the default while those factors are used.

    load_mode="eq54" is the self-consistent version:
        Q_tb = q |Cm| b c^2 / l_ht  (Eq. 54),   M_tb = Q_tb l_tb
    It needs a recalibrated f_tb.
    """

    def initialize(self):
        self.options.declare("load_mode", default="paper", values=["paper", "eq54"])

    def io(self):
        ins = {
            "q": (150.0, "Pa"),
            "Cm": (-0.1, None),
            "b": (3.0, "m"),
            "c_mean": (0.3, "m"),
            "l_ht": (1.0, "m"),
            "l_tb": (1.0, "m"),
        }
        return ins, {"Q_tb": (1.0, "N"), "M_tb": (1.0, "N*m")}

    def evaluate(self, x):
        q, b, c, lht, ltb = x["q"], x["b"], x["c_mean"], x["l_ht"], x["l_tb"]
        aCm, daCm = smooth_abs(x["Cm"], _EPS_CM)

        M = q * aCm * b * c**2 * ltb / lht  # Eq. 56 (both modes)
        gM = D(
            {
                "q": aCm * b * c**2 * ltb / lht,
                "Cm": q * daCm * b * c**2 * ltb / lht,
                "b": q * aCm * c**2 * ltb / lht,
                "c_mean": 2.0 * q * aCm * b * c * ltb / lht,
                "l_tb": q * aCm * b * c**2 / lht,
                "l_ht": -M / lht,
            }
        )
        if self.options["load_mode"] == "paper":
            Q = q * aCm * c**3 / lht  # Eq. 55 with Eq. 5 as printed
            gQ = D(
                {
                    "q": aCm * c**3 / lht,
                    "Cm": q * daCm * c**3 / lht,
                    "c_mean": 3.0 * q * aCm * c**2 / lht,
                    "l_ht": -Q / lht,
                }
            )
            return {"Q_tb": Q, "M_tb": M}, {"Q_tb": gQ, "M_tb": gM}

        Q = q * aCm * b * c**2 / lht  # Eq. 54
        gQ = D(
            {
                "q": aCm * b * c**2 / lht,
                "Cm": q * daCm * b * c**2 / lht,
                "b": q * aCm * c**2 / lht,
                "c_mean": 2.0 * q * aCm * b * c / lht,
                "l_ht": -Q / lht,
            }
        )
        return {"Q_tb": Q, "M_tb": Q * ltb}, {"Q_tb": gQ, "M_tb": ltb * gQ + D({"l_tb": Q})}
