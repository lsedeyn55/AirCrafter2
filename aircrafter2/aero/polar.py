"""Drag polars for the mission model, from three coupled aero points.

ThreePointPolar fits CD(CL) through the cruise, mid and near-C_L,max points of the full model, and feeds the lap model.
With CD1 = 0 it reduces to the AirCrafter form CD = CD0 + k CL^2 (Kallarackel 2025, Eqs. 3.5-3.6).
Why three points: docs/modules/aero.md, "Polar".
Abbreviations: VLM = vortex-lattice method.
"""

import numpy as np
import openmdao.api as om


class ThreePointPolar(om.ExplicitComponent):
    """CD(CL) = CD0 + CD1 CL + k CL^2, exactly through three analysed points.

    Partials are complex step, through a 3 x 3 solve.

    Inputs
    ------
    CL_0, CD_0, CL_1, CD_1, CL_2, CD_2 : the three points

    Outputs
    -------
    CD0, CD1, k : the polar's coefficients
    """

    def setup(self):
        for i, (cl, cd) in enumerate([(0.3, 0.02), (0.7, 0.03), (1.1, 0.06)]):
            self.add_input(f"CL_{i}", cl)
            self.add_input(f"CD_{i}", cd)
        self.add_output("CD0", 0.015)
        self.add_output("CD1", 0.0)
        self.add_output("k", 0.05)
        self.declare_partials("*", "*", method="cs")

    def compute(self, inputs, outputs):
        CL = np.array([inputs[f"CL_{i}"][0] for i in range(3)])
        CD = np.array([inputs[f"CD_{i}"][0] for i in range(3)])
        A = np.stack([np.ones(3, dtype=CL.dtype), CL, CL**2], axis=1)
        c = np.linalg.solve(A, CD)
        outputs["CD0"], outputs["CD1"], outputs["k"] = c[0], c[1], c[2]


class TurnAlpha(om.ExplicitComponent):
    """Angles of attack of the mid and near-C_L,max points of the polar.

    alpha (from zero lift) for C_L = frac C_L,max, and the midpoint, using C_L ~ alpha (flat-plate VLM, plan-a2 in
    docs/DECISIONS.md). Partials are complex step: a short expression.

    Options
    -------
    frac : float
        Fraction of C_L,max at the turn point.

    Inputs
    ------
    alpha_c [deg], CL_c, CL_max

    Outputs
    -------
    alpha_t, alpha_m [deg]
    """

    def initialize(self):
        self.options.declare("frac", default=0.95)

    def setup(self):
        self.add_input("alpha_c", 6.0, units="deg")
        self.add_input("CL_c", 0.4)
        self.add_input("CL_max", 1.2)
        self.add_output("alpha_t", 15.0, units="deg")
        self.add_output("alpha_m", 10.0, units="deg")
        self.declare_partials("*", "*", method="cs")

    def compute(self, inputs, outputs):
        at = inputs["alpha_c"] * self.options["frac"] * inputs["CL_max"] / inputs["CL_c"]
        outputs["alpha_t"] = at
        outputs["alpha_m"] = 0.5 * (at + inputs["alpha_c"])
