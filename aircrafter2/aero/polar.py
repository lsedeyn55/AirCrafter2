"""Drag polars for the mission model.

ThreePointPolar fits CD(CL) = CD0 + CD1 CL + k CL^2 exactly through three analysed points
(cruise, mid, near-C_L,max), so the lap model *interpolates* the coupled VLM + NeuralFoil drag
over the whole C_L range it uses (cruise, the straights down to stall, the C_L,max turns), with
no extrapolation. With CD1 = 0 it reduces to the AirCrafter form CD = CD0 + k CL^2
(Kallarackel 2025, Eqs. 3.5-3.6).

A two-point CD0 + k CL^2 fit was tried first and rejected: off-design (e.g. a highly cambered
section at high speed and low C_L) its k went negative, an unphysical polar that was then
extrapolated to C_L,max (docs/step6_lap_mdo.md).
"""

import numpy as np
import openmdao.api as om


class ThreePointPolar(om.ExplicitComponent):
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
    """alpha (from zero lift) for C_L = frac C_L,max, and the midpoint, using C_L ~ alpha (flat-plate VLM)."""

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
