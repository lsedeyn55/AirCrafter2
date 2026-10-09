"""Wing C_L,max (critical-section method) and wing totals."""

import numpy as np
import openmdao.api as om

from ..smooth import ks_min


class CriticalSectionCLmax(om.ExplicitComponent):
    """Wing C_L,max by the critical-section method.

    The VLM is linear in alpha, so the spanwise section-lift shape at a reference C_L scales
    with C_L. The wing reaches C_L,max when the first strip reaches its section C_l,max:

        C_L,max = C_L * min_i ( C_l,max,i / C_l,i )

    This is the classical critical-section method used with lifting-line or VLM span loadings
    (exact textbook reference to be added in docs/aero.md). The min is a KS-min (or hard min). C_l,max,i is evaluated at the strip
    Reynolds number of the analysis point, an approximation when the point is not near stall.
    Strips with C_l,i <= 0 are excluded (floored), so the reference point must have C_L > 0.
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
        Cl = inputs["Cl"]
        floor = 1e-6
        Clf = np.where(np.real(Cl) > floor, Cl, floor)
        active = np.real(Cl) > floor
        r = inputs["Clmax_sec"] / Clf
        f, grads = ks_min([r[i:i + 1] for i in range(r.size)], self.options["rho_ks"], self.options["max_mode"])
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


class WingPerformance(om.ExplicitComponent):
    """CD = CDi (VLM) + CDp (NeuralFoil strips) + CD_extra (parasite, later steps); L, D, L/D."""

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
