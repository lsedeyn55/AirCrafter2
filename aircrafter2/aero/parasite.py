"""Parasite drag of the tails, the fuselage and the nacelles: a component build-up.

Uses Raymer, Aircraft Design: A Conceptual Approach, Ch. 12. Takes the geometry and flight condition inside the MDA, and
gives CD_extra, which the aero points add to the wing's own drag. The wing's profile drag comes from the NeuralFoil
strips, not from here. Details and formulas: docs/modules/aero.md, "Parasite drag".
"""

import numpy as np
import openmdao.api as om


def cf_turbulent(Re):
    """Turbulent flat-plate skin friction, C_f = 0.455 / (log10 Re)^2.58 (Raymer Ch. 12, Mach term dropped, M < 0.1).

    Parameters
    ----------
    Re : float or array
        Reynolds number.

    Returns
    -------
    float or array
        C_f.
    """
    return 0.455 / np.log10(Re) ** 2.58


class ParasiteDrag(om.ExplicitComponent):
    """CD_0,i = C_f,i FF_i Q_i S_wet,i / S_ref, summed over the components.

    Partials are complex step: short expressions, and the fineness terms are awkward by hand.

    Options
    -------
    fuselage_drag : bool
        Include the cargo box and tail boom.
    n_nacelles : 0 or 2
        Twin wing-mounted layout. The nose layout's nose is in the fuselage term.
    nacelle_ff : "body" or "nacelle"
        Which Raymer form factor the nacelles use (docs/DECISIONS.md#plan-t6).

    Inputs
    ------
    v [m/s], rho [kg/m**3], mu [Pa*s], S, S_ht, S_vt [m**2], AR_ht, AR_vt, tc_tail, xt_tail, Q_tail
    l_cb, w_cb, h_cb, l_tb, r_tb [m], Q_fus
    l_nac, d_nac [m], Q_nac : only with nacelles

    Outputs
    -------
    CD_ht, CD_vt, CD_cb, CD_boom, CD_tail, CD_fus, CD_nac, CD_extra

    Symbols
    -------
    C_f : skin friction (cf_turbulent)
    FF : form factor
    Q : interference factor
    S_wet, Swet : wetted area [m**2]
    f, fb, fn : fineness, length / diameter, of the cargo box, boom and nacelle
    d_eq : equivalent diameter of the box [m]
    q_over_mu : rho v / mu, Reynolds number per metre [1/m]
    xt : position of maximum thickness, chord fraction
    """

    def initialize(self):
        self.options.declare("fuselage_drag", default=True)
        self.options.declare("n_nacelles", default=0, values=[0, 2])
        self.options.declare("nacelle_ff", default="body", values=["body", "nacelle"])

    def setup(self):
        for name, val, units in [
            ("v", 20.0, "m/s"),
            ("rho", 1.18, "kg/m**3"),
            ("mu", 1.8e-5, "Pa*s"),
            ("S", 0.375, "m**2"),
            ("S_ht", 0.06, "m**2"),
            ("S_vt", 0.03, "m**2"),
            ("AR_ht", 4.0, None),
            ("AR_vt", 2.0, None),
            ("tc_tail", 0.09, None),
            ("xt_tail", 0.3, None),
            ("Q_tail", 1.0, None),
            ("l_cb", 0.4, "m"),
            ("w_cb", 0.12, "m"),
            ("h_cb", 0.12, "m"),
            ("l_tb", 0.65, "m"),
            ("r_tb", 0.008, "m"),
            ("Q_fus", 1.0, None),
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
            """CD_0 of one rectangular tail surface, chord sqrt(S / AR)."""
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
            outputs["CD_nac"] = (
                self.options["n_nacelles"] * cf_turbulent(q_over_mu * ln) * FF * x["Q_nac"] * np.pi * dn * ln / x["S"]
            )
        else:
            outputs["CD_nac"] = 0.0
        outputs["CD_extra"] = outputs["CD_tail"] + outputs["CD_fus"] + outputs["CD_nac"]
