"""Structural sizing: tube spar, ribs and tail boom.

Garcia & Gamboa (2022), Sections 3.3.2 (ribs), 3.3.3 (tube spar) and 3.4
(tail boom). Every max() on the gradient path uses ``ks_max``, with a "hard"
mode for reporting and verification.

Formulas are written in r_tube directly: pi K_tube^2 c^2 = pi r^2 (Eq. 15),
so the paper's chord dependence cancels.
"""

import numpy as np

from ...analytic import AnalyticComponent, D, combine
from ...smooth import ks_max, softplus


def deflection_poly(b, y1, mode):
    """Bracketed polynomial in Eq. 110 and its derivatives (dP/db, dP/dy1).

    mode="paper": as printed in Eqs. 70, 109, 110.
    mode="exact": exact Euler-Bernoulli double integration for I_2 = (1 - 2 y1/b)^2 I_1
                  outboard of y1 (Eq. 108), uniform load. Same 384 b E I prefactor.
    Both reduce to 3 b^4 at y1 = 0 (PLAN G10).
    """
    if mode == "paper":
        P = 3 * b**4 + 24 * b**3 * y1 - 120 * b**2 * y1**2 + 224 * b * y1**3 - 112 * y1**4
        dPdb = 12 * b**3 + 72 * b**2 * y1 - 240 * b * y1**2 + 224 * y1**3
        dPdy = 24 * b**3 - 240 * b**2 * y1 + 672 * b * y1**2 - 448 * y1**3
    elif mode == "exact":
        P = 3 * (b**4 + 4 * b**3 * y1 - 20 * b**2 * y1**2 + 32 * b * y1**3 - 16 * y1**4)
        dPdb = 3 * (4 * b**3 + 12 * b**2 * y1 - 40 * b * y1**2 + 32 * y1**3)
        dPdy = 3 * (4 * b**3 - 40 * b**2 * y1 + 96 * b * y1**2 - 64 * y1**3)
    else:
        raise ValueError(f"unknown deflection_mode {mode!r}")
    return P, dPdb, dPdy


class TubeSparSizing(AnalyticComponent):
    """Tube spar wall thickness, Eqs. 105-118.

    Eq. 107  t_sigma = M_x / (sigma pi r^2)
    Eq. 110  t_delta = n W P(b, y1) / (384 b E delta_max pi r^3),  delta_max = ratio * b
    Eq. 112  t_tau   = 3 Q_z / (4 tau pi r)
    Eq. 114  t_T     = M_y / (2 pi r^2 tau)
    Eq. 116  t_theta = b M_y^fl / (16 pi r^3 G theta_max)            [optional, include_twist]
    Eq. 117  t_div   = b^2 c^2 q_div (x_spar - 1/4) dCL/da / (16 pi r^3 G), floored at 0 (PLAN G9)
    Eq. 118  t_tube  = max(t_sigma, t_delta) + t_tau + max(t_T, t_theta, t_div)

    sizing_radius_mode="paper" sizes with K_tube = r / c_mean (Eq. 15, with c the mean chord as
    defined in Sec. 2.2.2) times the sizing chord c_root (Sec. 3.3), i.e. an effective radius
    r_s = r c_root / c_mean. That literal reading reproduces Table 15 (docs/mass_verification.md).
    It is unconservative for tapered wings and identical for rectangular ones.
    "physical" uses r_s = r.

    torsion_stiffness_mode="bredt" uses GJ = 2 pi r^3 G t (Bredt-Batho, single wall) instead of
    the paper's Eq. 115 GJ = 4 pi r^3 G t; that halves the 16 in Eqs. 116 and 117 to 8 (PLAN G12).
    With use_min_gauge, t_tube = max(t_tube,req, t_tube_min).
    """

    def initialize(self):
        self.options.declare("max_mode", default="ks", values=["ks", "hard"])
        self.options.declare("rho_ks", default=100.0)
        self.options.declare("softplus_scale", default=1e-6, desc="m, width of the smooth floor on t_div")
        self.options.declare("deflection_mode", default="paper", values=["paper", "exact"])
        self.options.declare("torsion_stiffness_mode", default="paper", values=["paper", "bredt"])
        self.options.declare("include_twist", default=False)
        self.options.declare("use_min_gauge", default=True)
        self.options.declare("sizing_radius_mode", default="paper", values=["paper", "physical"])

    def io(self):
        ins = {
            "Mx": (100.0, "N*m"),
            "Qz": (60.0, "N"),
            "My": (5.0, "N*m"),
            "T_fl": (3.0, "N*m"),
            "W": (120.0, "N"),
            "n": (2.0, None),
            "b": (3.0, "m"),
            "c_mean": (0.3, "m"),
            "r_tube": (0.012, "m"),
            "c_root": (0.35, "m"),
            "y1": (0.4, "m"),
            "delta_tip_ratio": (0.1, None),
            "sigma_spar_allow": (480e6, "Pa"),
            "tau_spar_allow": (36e6, "Pa"),
            "E_spar": (84e9, "Pa"),
            "G_spar": (5e9, "Pa"),
            "q_div": (200.0, "Pa"),
            "CL_alpha": (5.0, "1/rad"),
            "x_spar": (0.3, None),
        }
        if self.options["include_twist"]:
            ins["theta_tip_max"] = (0.035, "rad")
        if self.options["use_min_gauge"]:
            ins["t_tube_min"] = (5e-4, "m")
        outs = {name: (1e-4, "m") for name in
                ["t_sigma", "t_delta", "t_tau", "t_T", "t_div", "t_tube_req", "t_tube"]}
        if self.options["include_twist"]:
            outs["t_theta"] = (1e-4, "m")
        if self.options["use_min_gauge"]:
            outs["min_gauge_margin"] = (0.0, "m")
        outs["wall_ratio"] = (0.05, None)
        return ins, outs

    def evaluate(self, x):
        opt = self.options
        mode, rho = opt["max_mode"], opt["rho_ks"]
        b, c = x["b"], x["c_mean"]
        if opt["sizing_radius_mode"] == "paper":
            r = x["r_tube"] * x["c_root"] / c
            g_rs = D({"r_tube": x["c_root"] / c, "c_root": x["r_tube"] / c, "c_mean": -r / c})
        else:
            r = x["r_tube"]
            g_rs = D({"r_tube": 1.0})
        sig, tau, E, G = x["sigma_spar_allow"], x["tau_spar_allow"], x["E_spar"], x["G_spar"]
        pi = np.pi
        values, grads = {}, {}

        # Eq. 107: bending strength
        k = 1.0 / (sig * pi * r**2)
        t_sig = x["Mx"] * k
        g_sig = D({"Mx": k, "sigma_spar_allow": -t_sig / sig, "_rs": -2.0 * t_sig / r})

        # Eq. 110: tip deflection, delta_max = ratio * b
        P, dPdb, dPdy = deflection_poly(b, x["y1"], opt["deflection_mode"])
        n, W, ratio = x["n"], x["W"], x["delta_tip_ratio"]
        t_del = n * W * P / (384.0 * E * ratio * b**2 * pi * r**3)
        g_del = D({
            "n": t_del / n, "W": t_del / W, "E_spar": -t_del / E, "delta_tip_ratio": -t_del / ratio,
            "_rs": -3.0 * t_del / r, "b": t_del * (dPdb / P - 2.0 / b), "y1": t_del * dPdy / P,
        })

        # Eq. 112: shear (3/2 peak factor as printed; PLAN G11)
        k = 3.0 / (4.0 * tau * pi * r)
        t_tau = x["Qz"] * k
        g_tau = D({"Qz": k, "tau_spar_allow": -t_tau / tau, "_rs": -t_tau / r})

        # Eq. 114: torsion strength
        k = 1.0 / (2.0 * pi * r**2 * tau)
        t_T = x["My"] * k
        g_T = D({"My": k, "tau_spar_allow": -t_T / tau, "_rs": -2.0 * t_T / r})

        # GJ = gfac * pi r^3 G t: paper Eq. 115 uses gfac = 4, Bredt-Batho gives 2
        gfac = 4.0 if opt["torsion_stiffness_mode"] == "paper" else 2.0

        # Eq. 117: divergence (raw value may be negative if the spar is ahead of c/4)
        A0 = b**2 * c**2 * x["CL_alpha"] / (4.0 * gfac * pi * r**3 * G)
        arm = x["x_spar"] - 0.25
        raw = A0 * x["q_div"] * arm
        g_raw = D({
            "b": 2.0 * raw / b, "c_mean": 2.0 * raw / c, "CL_alpha": A0 * x["q_div"] * arm / x["CL_alpha"],
            "_rs": -3.0 * raw / r, "G_spar": -raw / G, "q_div": A0 * arm, "x_spar": A0 * x["q_div"],
        })
        t_div, dsp = softplus(raw, opt["softplus_scale"], mode)
        g_div = dsp * g_raw

        tors, g_tors_list = [t_T], [g_T]
        if opt["include_twist"]:
            # Eq. 116 (generalised for gfac)
            th = x["theta_tip_max"]
            k = b / (4.0 * gfac * pi * r**3 * G * th)
            t_th = x["T_fl"] * k
            g_th = D({"T_fl": k, "b": t_th / b, "_rs": -3.0 * t_th / r, "G_spar": -t_th / G,
                      "theta_tip_max": -t_th / th})
            tors.append(t_th)
            g_tors_list.append(g_th)
            values["t_theta"], grads["t_theta"] = t_th, g_th
        tors.append(t_div)
        g_tors_list.append(g_div)

        # Eq. 118
        t_bend, w_bend = ks_max([t_sig, t_del], rho, mode)
        t_tors, w_tors = ks_max(tors, rho, mode)
        t_req = t_bend + t_tau + t_tors
        g_req = combine(w_bend, [g_sig, g_del]) + g_tau + combine(w_tors, g_tors_list)

        if opt["use_min_gauge"]:
            t_tube, (wa, wb) = ks_max([t_req, x["t_tube_min"]], rho, mode)
            g_tube = wa * g_req + D({"t_tube_min": wb})
            values["min_gauge_margin"] = t_req - x["t_tube_min"]
            grads["min_gauge_margin"] = g_req + D({"t_tube_min": -1.0})
        else:
            t_tube, g_tube = t_req, g_req

        values.update(t_sigma=t_sig, t_delta=t_del, t_tau=t_tau, t_T=t_T, t_div=t_div,
                      t_tube_req=t_req, t_tube=t_tube)
        grads.update(t_sigma=g_sig, t_delta=g_del, t_tau=g_tau, t_T=g_T, t_div=g_div,
                     t_tube_req=g_req, t_tube=g_tube)

        # chain rule: "_rs" holds d/d r_s; map it onto (r_tube, c_root, c_mean)
        for name, g in grads.items():
            if "_rs" in g:
                g = D(g)
                grads[name] = g + g.pop("_rs") * g_rs

        # wall thickness / actual tube radius (thin-wall validity check)
        r_act = x["r_tube"]
        values["wall_ratio"] = t_tube / r_act
        grads["wall_ratio"] = (1.0 / r_act) * grads["t_tube"] + D({"r_tube": -t_tube / r_act**2})
        return values, grads


class RibSizing(AnalyticComponent):
    """Total rib thickness n_rib * t_rib, Eqs. 97-104.

    Derived in the paper for the D-box aft ribs and applied to the tube concept with
    c_rib = aft part of the rib (~70 % of chord, Sec. 3.3.3). Sizing chord is the
    root chord (Sec. 3.3). No integer rib count enters the gradient path.

    Eq. 100  (n t)_tau   =  600 n W (c_rib/c)^2 / (391 tau (t/c) c)
    Eq. 103  (n t)_sigma = 2400 n W (c_rib/c)^3 / (1173 sigma (t/c)^2 c)
    Eq. 104  n t = max of the two
    """

    def initialize(self):
        self.options.declare("max_mode", default="ks", values=["ks", "hard"])
        self.options.declare("rho_ks", default=100.0)

    def io(self):
        ins = {
            "n": (2.0, None),
            "W": (120.0, "N"),
            "c_root": (0.35, "m"),
            "tc": (0.12, None),
            "c_rib_frac": (0.7, None),
            "sigma_balsa_allow": (6.67e6, "Pa"),
            "tau_balsa_allow": (0.933e6, "Pa"),
        }
        outs = {"nt_rib_tau": (1e-3, "m"), "nt_rib_sigma": (1e-3, "m"), "nt_rib": (1e-3, "m")}
        return ins, outs

    def evaluate(self, x):
        n, W, c, tc, f = x["n"], x["W"], x["c_root"], x["tc"], x["c_rib_frac"]
        sig, tau = x["sigma_balsa_allow"], x["tau_balsa_allow"]

        a = 600.0 * n * W * f**2 / (391.0 * tau * tc * c)  # Eq. 100
        ga = D({"n": a / n, "W": a / W, "c_rib_frac": 2.0 * a / f, "tau_balsa_allow": -a / tau,
                "tc": -a / tc, "c_root": -a / c})
        s = 2400.0 * n * W * f**3 / (1173.0 * sig * tc**2 * c)  # Eq. 103
        gs = D({"n": s / n, "W": s / W, "c_rib_frac": 3.0 * s / f, "sigma_balsa_allow": -s / sig,
                "tc": -2.0 * s / tc, "c_root": -s / c})
        nt, w = ks_max([a, s], self.options["rho_ks"], self.options["max_mode"])  # Eq. 104
        return ({"nt_rib_tau": a, "nt_rib_sigma": s, "nt_rib": nt},
                {"nt_rib_tau": ga, "nt_rib_sigma": gs, "nt_rib": combine(w, [ga, gs])})


class TailBoomSizing(AnalyticComponent):
    """Tail-boom wall thickness, Eqs. 119-123.

    Eq. 119  t_sigma = M_tb / (sigma pi r^2)
    Eq. 121  t_theta = Q_tb l_tb^2 / (2 E pi r^3 theta_max)
    Eq. 122  t_tau   = 3 Q_tb / (4 tau pi r)
    Eq. 123  t_tb    = max(t_sigma, t_theta) + t_tau

    Minimum wall (use_min_gauge): t_tb = max(t_tb,req, t_tb_min). At DBF scale the load-based
    wall is micrometres, so in practice the commercial tube wall governs. The output
    t_tb_gauge_margin = t_tb,req - t_tb_min is < 0 when the minimum wall governs. G&G's f_tb was
    fitted to load-based walls, so with the minimum wall active f_tb must be recalibrated on the
    same basis (docs/tail_boom_calibration.md).
    """

    def initialize(self):
        self.options.declare("max_mode", default="ks", values=["ks", "hard"])
        self.options.declare("rho_ks", default=100.0)
        self.options.declare("use_min_gauge", default=True)

    def io(self):
        ins = {
            "Q_tb": (10.0, "N"),
            "M_tb": (15.0, "N*m"),
            "l_tb": (1.5, "m"),
            "r_tb": (0.015, "m"),
            "sigma_tb_allow": (480e6, "Pa"),
            "tau_tb_allow": (36e6, "Pa"),
            "E_tb": (84e9, "Pa"),
            "theta_tb_max": (0.01745, "rad"),
        }
        if self.options["use_min_gauge"]:
            ins["t_tb_min"] = (5e-4, "m")
        outs = {name: (1e-4, "m") for name in ["t_tb_sigma", "t_tb_theta", "t_tb_tau", "t_tb_req", "t_tb"]}
        if self.options["use_min_gauge"]:
            outs["t_tb_gauge_margin"] = (0.0, "m")
        return ins, outs

    def evaluate(self, x):
        mode, rho = self.options["max_mode"], self.options["rho_ks"]
        Q, M, l, r = x["Q_tb"], x["M_tb"], x["l_tb"], x["r_tb"]
        sig, tau, E, th = x["sigma_tb_allow"], x["tau_tb_allow"], x["E_tb"], x["theta_tb_max"]
        pi = np.pi

        k = 1.0 / (sig * pi * r**2)
        t_s = M * k  # Eq. 119
        g_s = D({"M_tb": k, "sigma_tb_allow": -t_s / sig, "r_tb": -2.0 * t_s / r})

        k = l**2 / (2.0 * E * pi * r**3 * th)
        t_th = Q * k  # Eq. 121
        g_th = D({"Q_tb": k, "l_tb": 2.0 * t_th / l, "E_tb": -t_th / E, "r_tb": -3.0 * t_th / r,
                  "theta_tb_max": -t_th / th})

        k = 3.0 / (4.0 * tau * pi * r)
        t_t = Q * k  # Eq. 122
        g_t = D({"Q_tb": k, "tau_tb_allow": -t_t / tau, "r_tb": -t_t / r})

        mx, w = ks_max([t_s, t_th], rho, mode)
        t_req = mx + t_t  # Eq. 123
        g_req = combine(w, [g_s, g_th]) + g_t
        if self.options["use_min_gauge"]:
            t_tb, (wa, wb) = ks_max([t_req, x["t_tb_min"]], rho, mode)
            g_tb = wa * g_req + D({"t_tb_min": wb})
        else:
            t_tb, g_tb = t_req, g_req
        values = {"t_tb_sigma": t_s, "t_tb_theta": t_th, "t_tb_tau": t_t, "t_tb_req": t_req, "t_tb": t_tb}
        grads = {"t_tb_sigma": g_s, "t_tb_theta": g_th, "t_tb_tau": g_t, "t_tb_req": g_req, "t_tb": g_tb}
        if self.options["use_min_gauge"]:
            values["t_tb_gauge_margin"] = t_req - x["t_tb_min"]
            grads["t_tb_gauge_margin"] = g_req + D({"t_tb_min": -1.0})
        return values, grads
