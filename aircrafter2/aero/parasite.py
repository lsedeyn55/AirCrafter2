"""Parasite drag of the tails and (optionally) the fuselage: component build-up.

Raymer, Aircraft Design: A Conceptual Approach, Ch. 12 (component build-up method):

    CD_0,i = C_f,i FF_i Q_i S_wet,i / S_ref
    C_f    = 0.455 / (log10 Re)^2.58            turbulent flat plate (Mach term dropped, M < 0.1)
    FF     = 1 + (0.6 / x_t) (t/c) + 100 (t/c)^4    thin lifting surfaces (Mach/sweep term dropped)
    FF     = 1 + 60 / f^3 + f / 400              bodies, fineness f = l / d
    S_wet  ~ S (1.977 + 0.52 t/c)               thin surfaces (Raymer Ch. 7 approximation)

Fully turbulent C_f is conservative for small RC surfaces. The wing's own profile drag comes from
NeuralFoil strips, not from here. Tails: rectangular, chord sqrt(S / AR). Fuselage: the cargo-bay
box (equivalent diameter from its cross-section area) plus the tail-boom tube; included only with
fuselage_drag=True (brief: "switchable").

Nacelles (twin wing-mounted layout, n_nacelles = 2): the same body build-up per nacelle, a cylinder
of length l_nac and diameter d_nac, S_wet ~ pi d l, C_f at Re_l, times the interference factor Q_nac.
nacelle_ff="body" (default, as briefed) uses the body form factor above; "nacelle" uses Raymer's
nacelle / smooth external store form FF = 1 + 0.35 / f (Ch. 12). For the nose layout the nose is in
the fuselage term and CD_nac = 0.
"""

import numpy as np
import openmdao.api as om


def cf_turbulent(Re):
    return 0.455 / np.log10(Re) ** 2.58


class ParasiteDrag(om.ExplicitComponent):
    def initialize(self):
        self.options.declare("fuselage_drag", default=True)
        self.options.declare("n_nacelles", default=0, values=[0, 2])
        self.options.declare("nacelle_ff", default="body", values=["body", "nacelle"])

    def setup(self):
        for name, val, units in [
            ("v", 20.0, "m/s"), ("rho", 1.18, "kg/m**3"), ("mu", 1.8e-5, "Pa*s"), ("S", 0.375, "m**2"),
            ("S_ht", 0.06, "m**2"), ("S_vt", 0.03, "m**2"), ("AR_ht", 4.0, None), ("AR_vt", 2.0, None),
            ("tc_tail", 0.09, None), ("xt_tail", 0.3, None), ("Q_tail", 1.0, None),
            ("l_cb", 0.4, "m"), ("w_cb", 0.12, "m"), ("h_cb", 0.12, "m"), ("l_tb", 0.65, "m"),
            ("r_tb", 0.008, "m"), ("Q_fus", 1.0, None),
        ]:
            self.add_input(name, val, units=units)
        if self.options["n_nacelles"]:
            for name, val, units in [("l_nac", 0.15, "m"), ("d_nac", 0.05, "m"), ("Q_nac", 1.3, None)]:
                self.add_input(name, val, units=units)
        for name in ["CD_ht", "CD_vt", "CD_cb", "CD_boom", "CD_tail", "CD_fus", "CD_nac", "CD_extra"]:
            self.add_output(name, 0.001)
        self.declare_partials("*", "*", method="cs")

    def compute(self, inputs, outputs):
        x = {k: inputs[k] for k in inputs}
        q_over_mu = x["rho"] * x["v"] / x["mu"]
        tc, xt = x["tc_tail"], x["xt_tail"]
        FF_tail = 1.0 + 0.6 / xt * tc + 100.0 * tc**4

        def surface(Sname, ARname):
            S = x[Sname]
            c = np.sqrt(S / x[ARname])
            Swet = S * (1.977 + 0.52 * tc)
            return cf_turbulent(q_over_mu * c) * FF_tail * x["Q_tail"] * Swet / x["S"]

        outputs["CD_ht"] = surface("S_ht", "AR_ht")
        outputs["CD_vt"] = surface("S_vt", "AR_vt")
        outputs["CD_tail"] = outputs["CD_ht"] + outputs["CD_vt"]

        l, w, h = x["l_cb"], x["w_cb"], x["h_cb"]
        d_eq = np.sqrt(4.0 * w * h / np.pi)
        f = l / d_eq
        S_cb = 2.0 * (l * w + l * h + w * h)
        cd_cb = cf_turbulent(q_over_mu * l) * (1 + 60 / f**3 + f / 400) * x["Q_fus"] * S_cb / x["S"]
        lb, rb = x["l_tb"], x["r_tb"]
        fb = lb / (2 * rb)
        cd_boom = cf_turbulent(q_over_mu * lb) * (1 + 60 / fb**3 + fb / 400) * x["Q_fus"] * 2 * np.pi * rb * lb / x["S"]
        on = 1.0 if self.options["fuselage_drag"] else 0.0
        outputs["CD_cb"] = on * cd_cb
        outputs["CD_boom"] = on * cd_boom
        outputs["CD_fus"] = on * (cd_cb + cd_boom)
        if self.options["n_nacelles"]:
            ln, dn = x["l_nac"], x["d_nac"]
            fn = ln / dn
            FF = (1 + 60 / fn**3 + fn / 400) if self.options["nacelle_ff"] == "body" else (1 + 0.35 / fn)
            outputs["CD_nac"] = self.options["n_nacelles"] * cf_turbulent(q_over_mu * ln) * FF * x["Q_nac"] \
                * np.pi * dn * ln / x["S"]
        else:
            outputs["CD_nac"] = 0.0
        outputs["CD_extra"] = outputs["CD_tail"] + outputs["CD_fus"] + outputs["CD_nac"]
