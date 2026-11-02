"""Wing profile drag from NeuralFoil strip theory, plus section C_m and C_l,max.

For every spanwise strip, it finds the section angle that gives the vortex-lattice section lift, and reads drag and
moment from NeuralFoil there. It sits inside each aero point (aero/wing.py), and replaces OpenAeroStruct's built-in
viscous drag (with_viscous=False), so profile drag isn't counted twice. Partials come from the implicit-function theorem
(airfoil/neuralfoil_section.py); only NeuralFoil itself is finite-differenced. Details: docs/modules/aero.md, "Wing".
"""

import numpy as np
import openmdao.api as om

from ..airfoil.neuralfoil_section import NFSettings, pack, section_sensitivities, solve_sections

KULFAN_INPUTS = [("upper_weights", 8), ("lower_weights", 8), ("le_weight", 1), ("te_thickness", 1)]


class NeuralFoilStripDrag(om.ExplicitComponent):
    """Strip drag, moment and C_l,max over the half wing.

        Re_i  = rho V c_i / mu
        alpha_i solves C_l,NF(alpha_i; k, Re_i) = C_l,i   (the VLM section lift)
        C_d,i, C_m,i = NeuralFoil at (alpha_i, Re_i)
        CD_p  = 2 sum_i C_d,i c_i w_i / S_ref             (factor 2: both halves)

    Options
    -------
    n_strips : int
    symmetric : bool
        Half-wing model, so CD_p counts both halves.
    settings : NFSettings or None

    Inputs
    ------
    upper_weights, lower_weights (8,), le_weight, te_thickness : Kulfan parameters
    Cl : array (n_strips,), section lift coefficients from the VLM
    chords [m] (n_strips + 1,), widths [m] (n_strips,), S_ref [m**2], v [m/s], rho [kg/m**3], mu [Pa*s]

    Outputs
    -------
    Re_sec, alpha_sec [deg], Cd_sec, Cm_sec, Clmax_sec, cl_margin_sec : arrays (n_strips,)
    CDp, nf_confidence_min, n_stalled : floats; the last two have no constraint

    Symbols
    -------
    P : the 18 Kulfan parameters as one vector
    c_mid : strip mid chords [m]
    w : strip widths [m]
    f : 2 for a half-wing model
    """

    def initialize(self):
        self.options.declare("n_strips", types=int)
        self.options.declare("symmetric", default=True)
        self.options.declare("settings", default=None, types=(NFSettings, type(None)))

    def setup(self):
        n = self.options["n_strips"]
        self._s = self.options["settings"] or NFSettings()
        self.add_input("upper_weights", 0.2 * np.ones(8))
        self.add_input("lower_weights", -0.2 * np.ones(8))
        self.add_input("le_weight", 0.0)
        self.add_input("te_thickness", 0.0)
        self.add_input("Cl", 0.5 * np.ones(n), desc="section lift coefficients from the VLM")
        self.add_input("chords", 0.25 * np.ones(n + 1), units="m")
        self.add_input("widths", 0.1 * np.ones(n), units="m")
        self.add_input("S_ref", 0.375, units="m**2")
        self.add_input("v", 15.0, units="m/s")
        self.add_input("rho", 1.225, units="kg/m**3")
        self.add_input("mu", 1.789e-5, units="Pa*s")

        self.add_output("Re_sec", 2e5 * np.ones(n))
        self.add_output("alpha_sec", np.zeros(n), units="deg")
        self.add_output("Cd_sec", 0.01 * np.ones(n))
        self.add_output("Cm_sec", np.zeros(n))
        self.add_output("Clmax_sec", 1.3 * np.ones(n))
        self.add_output("cl_margin_sec", np.ones(n), desc="Clmax_sec - Cl (>= 0: no stalled strip)")
        self.add_output("CDp", 0.01)
        self.add_output("nf_confidence_min", 1.0, desc="report only")
        self.add_output("n_stalled", 0.0, desc="report only")

        kul = [k for k, _ in KULFAN_INPUTS]
        flow = ["chords", "v", "rho", "mu"]
        for out in ["alpha_sec", "Cd_sec", "Cm_sec"]:
            self.declare_partials(out, ["Cl"] + kul + flow)
        self.declare_partials("Clmax_sec", kul + flow)
        self.declare_partials("cl_margin_sec", ["Cl"] + kul + flow)
        self.declare_partials("Re_sec", flow)
        self.declare_partials("CDp", ["Cl", "widths", "S_ref"] + kul + flow)

    def _solve(self, inputs):
        """Solve every strip: (Kulfan vector, mid chords, Reynolds numbers, solution dict)."""
        P = pack(inputs["upper_weights"], inputs["lower_weights"], inputs["le_weight"], inputs["te_thickness"])
        ch = inputs["chords"]
        c_mid = 0.5 * (ch[1:] + ch[:-1])
        Re = inputs["rho"] * inputs["v"] * c_mid / inputs["mu"]
        return P, c_mid, Re, solve_sections(P, inputs["Cl"], Re, self._s)

    def compute(self, inputs, outputs):
        _, c_mid, Re, sol = self._solve(inputs)
        f = 2.0 if self.options["symmetric"] else 1.0
        outputs["Re_sec"] = Re
        outputs["alpha_sec"] = sol["alpha"]
        outputs["Cd_sec"] = sol["Cd"]
        outputs["Cm_sec"] = sol["Cm"]
        outputs["Clmax_sec"] = sol["Clmax"]
        outputs["cl_margin_sec"] = sol["Clmax"] - inputs["Cl"]
        outputs["CDp"] = f * np.sum(sol["Cd"] * c_mid * inputs["widths"]) / inputs["S_ref"]
        outputs["nf_confidence_min"] = np.min(sol["conf"])
        outputs["n_stalled"] = np.sum(sol["clamped"])

    def compute_partials(self, inputs, J):
        P, c_mid, Re, sol = self._solve(inputs)
        sens = section_sensitivities(P, sol, Re, self._s)
        n = self.options["n_strips"]
        f = 2.0 if self.options["symmetric"] else 1.0
        S, w = inputs["S_ref"][0], inputs["widths"]

        # d ln Re_i / d (chords, v, rho, mu)
        dlnRe = {
            "chords": np.zeros((n, n + 1)),
            "v": np.full((n, 1), 1.0 / inputs["v"][0]),
            "rho": np.full((n, 1), 1.0 / inputs["rho"][0]),
            "mu": np.full((n, 1), -1.0 / inputs["mu"][0]),
        }
        i = np.arange(n)
        dlnRe["chords"][i, i] = 0.5 / c_mid
        dlnRe["chords"][i, i + 1] = 0.5 / c_mid

        def kulfan_blocks(dk):
            """Split an (n, 18) Jacobian into one block per Kulfan input."""
            out, j = {}, 0
            for name, size in KULFAN_INPUTS:
                out[name] = dk[:, j : j + size]
                j += size
            return out

        per_strip = {"alpha_sec": "alpha", "Cd_sec": "Cd", "Cm_sec": "Cm"}
        for out, q in per_strip.items():
            J[out, "Cl"] = np.diag(sens[f"{q}_dCl"])
            for name, blk in kulfan_blocks(sens[f"{q}_dk"]).items():
                J[out, name] = blk
            for flow, d in dlnRe.items():
                J[out, flow] = sens[f"{q}_dlnRe"][:, None] * d

        for name, blk in kulfan_blocks(sens["Clmax_dk"]).items():
            J["Clmax_sec", name] = blk
            J["cl_margin_sec", name] = blk
        J["cl_margin_sec", "Cl"] = -np.eye(n)
        for flow, d in dlnRe.items():
            J["Clmax_sec", flow] = sens["Clmax_dlnRe"][:, None] * d
            J["cl_margin_sec", flow] = sens["Clmax_dlnRe"][:, None] * d
            J["Re_sec", flow] = Re[:, None] * d

        # CDp = f/S sum Cd_i c_i w_i
        cw = f * c_mid * w / S
        J["CDp", "Cl"] = (cw * sens["Cd_dCl"])[None, :]
        for name, blk in kulfan_blocks(sens["Cd_dk"]).items():
            J["CDp", name] = (cw @ blk)[None, :]
        dch = np.zeros(n + 1)  # direct dependence through c_i (Re dependence added below)
        np.add.at(dch, i, 0.5 * f * sol["Cd"] * w / S)
        np.add.at(dch, i + 1, 0.5 * f * sol["Cd"] * w / S)
        for flow, d in dlnRe.items():
            val = (cw * sens["Cd_dlnRe"]) @ d
            J["CDp", flow] = val + dch if flow == "chords" else val
        J["CDp", "widths"] = (f * sol["Cd"] * c_mid / S)[None, :]
        J["CDp", "S_ref"] = -f * np.sum(sol["Cd"] * c_mid * w) / S**2
