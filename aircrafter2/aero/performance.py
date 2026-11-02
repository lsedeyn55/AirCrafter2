"""Wing totals from the strips: C_L,max, section moment, lift-curve slope, drag, lift and L/D.

The last step of an aero point (aero/wing.py), after the vortex-lattice method (VLM) and the NeuralFoil strips. Its
outputs feed the MDA balances, the mass model (C_m, C_L,alpha) and the polar. Critical-section C_L,max is a classical
method; see docs/modules/aero.md, "Wing".
Abbreviations: MDA = multidisciplinary analysis.
"""

import numpy as np
import openmdao.api as om

from ..smooth import ks_min


class CriticalSectionCLmax(om.ExplicitComponent):
    """Wing C_L,max by the critical-section method.

    The VLM is linear in alpha, so the spanwise section-lift shape at a reference C_L scales with C_L. The wing reaches
    C_L,max when the first strip reaches its section C_l,max:

        C_L,max = C_L * min_i ( C_l,max,i / C_l,i )

    This is the classical critical-section method used with lifting-line or VLM span loadings (exact textbook reference
    to be added in docs/modules/aero.md). The min is a KS-min (or hard min). C_l,max,i is evaluated at the strip
    Reynolds number of the analysis point, an approximation when the point isn't near stall. Strips with C_l,i <= 0 are
    excluded (floored), so the reference point must have C_L > 0. Partials are analytic.

    Options
    -------
    n_strips : int
        Spanwise strips on the half wing.
    max_mode : "ks" or "hard"
        Smooth or hard minimum.
    rho_ks : float
        Sharpness of the KS (Kreisselmeier-Steinhauser) minimum.

    Inputs
    ------
    CL, Cl : float, array (n_strips,)
        Wing lift coefficient and section lift coefficients at the reference point.
    Clmax_sec : array (n_strips,)
        Section maximum lift coefficients.

    Outputs
    -------
    CL_max, CL_max_over_CL : float
    """

    def initialize(self):
        self.options.declare("n_strips", types=int)
        self.options.declare("max_mode", default="ks", values=["ks", "hard"])
        self.options.declare("rho_ks", default=100.0)

    def setup(self):
        n = self.options["n_strips"]
        self.add_input("CL", 0.5)
        self.add_input("Cl", 0.5 * np.ones(n))
        self.add_input("Clmax_sec", 1.3 * np.ones(n))
        self.add_output("CL_max", 1.2)
        self.add_output("CL_max_over_CL", 2.0)
        self.declare_partials("CL_max", ["CL", "Cl", "Clmax_sec"])
        self.declare_partials("CL_max_over_CL", ["Cl", "Clmax_sec"])

    def _ratio(self, inputs):
        """KS minimum of C_l,max,i / C_l,i: (floored Cl, active strips, ratios, minimum, its weights)."""
        Cl = inputs["Cl"]
        floor = 1e-6
        Clf = np.where(np.real(Cl) > floor, Cl, floor)
        active = np.real(Cl) > floor
        r = inputs["Clmax_sec"] / Clf
        f, grads = ks_min([r[i : i + 1] for i in range(r.size)], self.options["rho_ks"], self.options["max_mode"])
        return Clf, active, r, f[0], np.array([g[0] for g in grads])

    def compute(self, inputs, outputs):
        _, _, _, m, _ = self._ratio(inputs)
        outputs["CL_max_over_CL"] = m
        outputs["CL_max"] = inputs["CL"] * m

    def compute_partials(self, inputs, J):
        Clf, active, r, m, w = self._ratio(inputs)
        dm_dCl = -w * r / Clf * active
        dm_dClmax = w / Clf
        J["CL_max_over_CL", "Cl"] = dm_dCl[None, :]
        J["CL_max_over_CL", "Clmax_sec"] = dm_dClmax[None, :]
        J["CL_max", "CL"] = m
        J["CL_max", "Cl"] = inputs["CL"] * dm_dCl[None, :]
        J["CL_max", "Clmax_sec"] = inputs["CL"] * dm_dClmax[None, :]


class WingSectionMoment(om.ExplicitComponent):
    """Wing pitching-moment coefficient from the strip section moments (about each c/4).

        Cm_wing = f sum_i Cm_i c_i^2 w_i / (S c_ref),   f = 2 for a symmetric half-wing model

    With c_ref = S/b this is the area- and chord-weighted mean section C_m (for a rectangular wing, the width-weighted
    mean). Used as G&G's (Garcia & Gamboa 2022) wing C_m in Eqs. 46 and 54. Partials are complex step: a short
    expression.

    Options
    -------
    n_strips : int

    Inputs
    ------
    Cm_sec : array (n_strips,)
    chords [m] : array (n_strips + 1,)
    widths [m] : array (n_strips,)
    S_ref [m**2]

    Outputs
    -------
    Cm_wing
    """

    def initialize(self):
        self.options.declare("n_strips", types=int)

    def setup(self):
        n = self.options["n_strips"]
        self.add_input("Cm_sec", np.zeros(n))
        self.add_input("chords", 0.25 * np.ones(n + 1), units="m")
        self.add_input("widths", 0.1 * np.ones(n), units="m")
        self.add_input("S_ref", 0.375, units="m**2")
        self.add_output("Cm_wing", -0.1)
        self.declare_partials("Cm_wing", "*", method="cs")

    def compute(self, inputs, outputs):
        ch, w = inputs["chords"], inputs["widths"]
        c = 0.5 * (ch[1:] + ch[:-1])
        b_half = w.sum()
        c_ref = inputs["S_ref"] / (2.0 * b_half)
        outputs["Cm_wing"] = 2.0 * (inputs["Cm_sec"] * c**2 * w).sum() / (inputs["S_ref"] * c_ref)


class LiftCurveSlope(om.ExplicitComponent):
    """Wing lift-curve slope, C_L,alpha = C_L / alpha, per rad.

    Exact for the flat-plate VLM, whose alpha is measured from zero lift (docs/DECISIONS.md#plan-a2), so C_L is linear
    in alpha through the origin. Needs alpha != 0. Partials are analytic.

    Inputs
    ------
    CL, alpha [rad]

    Outputs
    -------
    CL_alpha [1/rad]
    """

    def setup(self):
        self.add_input("CL", 0.5)
        self.add_input("alpha", 5.0, units="rad")
        self.add_output("CL_alpha", 5.0, units="1/rad")
        self.declare_partials("CL_alpha", ["CL", "alpha"])

    def compute(self, inputs, outputs):
        outputs["CL_alpha"] = inputs["CL"] / inputs["alpha"]

    def compute_partials(self, inputs, J):
        J["CL_alpha", "CL"] = 1.0 / inputs["alpha"]
        J["CL_alpha", "alpha"] = -inputs["CL"] / inputs["alpha"] ** 2


class WingPerformance(om.ExplicitComponent):
    """CD = CDi (VLM) + CDp (NeuralFoil strips) + CD_extra (parasite; set by full_mdo); L, D, L/D.

    Partials are complex step: a short expression.

    Inputs
    ------
    CL, CDi, CDp, CD_extra, v [m/s], rho [kg/m**3], S_ref [m**2]

    Outputs
    -------
    CD, q [Pa], L [N], D [N], L_over_D
    """

    def setup(self):
        for name, val in [("CL", 0.5), ("CDi", 0.01), ("CDp", 0.01), ("CD_extra", 0.0)]:
            self.add_input(name, val)
        self.add_input("v", 15.0, units="m/s")
        self.add_input("rho", 1.225, units="kg/m**3")
        self.add_input("S_ref", 0.375, units="m**2")
        self.add_output("CD", 0.02)
        self.add_output("q", 100.0, units="Pa")
        self.add_output("L", 20.0, units="N")
        self.add_output("D", 1.0, units="N")
        self.add_output("L_over_D", 20.0)
        self.declare_partials("*", "*", method="cs")

    def compute(self, inputs, outputs):
        CD = inputs["CDi"] + inputs["CDp"] + inputs["CD_extra"]
        q = 0.5 * inputs["rho"] * inputs["v"] ** 2
        outputs["CD"] = CD
        outputs["q"] = q
        outputs["L"] = q * inputs["S_ref"] * inputs["CL"]
        outputs["D"] = q * inputs["S_ref"] * CD
        outputs["L_over_D"] = inputs["CL"] / CD
