"""Aero-only wing analysis point: OpenAeroStruct VLM + NeuralFoil profile drag.

    b, c --RectWingMesh--> mesh --OAS AeroPoint (with_viscous=False)--> CL, CDi, section Cl
    Kulfan params, section Cl, chords, widths --NeuralFoilStripDrag--> CDp, Cm, Clmax per strip
    --CriticalSectionCLmax--> CL_max ;  --WingPerformance--> CD, L, D, L/D

Promoted inputs: b, c, alpha (deg), v, rho, mu, upper_weights, lower_weights, le_weight,
te_thickness. Promoted outputs: CL, CDi, CDp, CD, L, D, L_over_D, CL_max, S_ref, plus the
per-strip arrays from the strip-drag component.
"""

import numpy as np
import openmdao.api as om
from openaerostruct.aerodynamics.aero_groups import AeroPoint

from ..airfoil.neuralfoil_section import NFSettings
from .mesh import RectWingMesh, template_mesh
from .performance import CriticalSectionCLmax, WingPerformance
from .profile_drag import NeuralFoilStripDrag


def oas_surface(num_x, num_y, spacing="cosine"):
    """Minimal OAS surface dict for an aero-only, inviscid VLM surface.

    OAS reads the template mesh *values* at setup (e.g. eval_mtx.py checks which half-wing it
    is), so it must have the right orientation; its size does not matter (tested).
    """
    return {
        "name": "wing",
        "symmetry": True,
        "S_ref_type": "projected",
        "mesh": template_mesh(num_x, num_y, spacing),  # orientation template; real mesh is connected
        "CL0": 0.0,
        "CD0": 0.0,
        "k_lam": 0.05,
        "t_over_c_cp": np.array([0.12]),
        "c_max_t": 0.3,
        "with_viscous": False,  # profile drag comes from NeuralFoil strips instead
        "with_wave": False,
    }


class WingAero(om.Group):
    def initialize(self):
        self.options.declare("num_x", default=3, types=int)
        self.options.declare("num_y", default=11, types=int)
        self.options.declare("spacing", default="cosine")
        self.options.declare("nf", default=None, types=(NFSettings, type(None)))
        self.options.declare("max_mode", default="ks")
        self.options.declare("rho_ks", default=100.0)

    def setup(self):
        nx, ny = self.options["num_x"], self.options["num_y"]
        n = ny - 1
        surf = oas_surface(nx, ny, self.options["spacing"])

        self.add_subsystem("mesh", RectWingMesh(num_x=nx, num_y=ny, spacing=self.options["spacing"]),
                           promotes_inputs=["b", "c"])
        self.add_subsystem("vlm", AeroPoint(surfaces=[surf]),
                           promotes_inputs=["v", "alpha", "beta", "Mach_number", "re", "rho", "cg"])
        self.connect("mesh.mesh", ["vlm.wing.def_mesh", "vlm.aero_states.wing_def_mesh"])

        self.add_subsystem("strips", NeuralFoilStripDrag(n_strips=n, settings=self.options["nf"]),
                           promotes_inputs=["upper_weights", "lower_weights", "le_weight", "te_thickness",
                                            "v", "rho", "mu"],
                           promotes_outputs=["CDp", "Re_sec", "alpha_sec", "Cd_sec", "Cm_sec", "Clmax_sec",
                                             "cl_margin_sec", "nf_confidence_min", "n_stalled"])
        self.connect("vlm.wing_perf.Cl", "strips.Cl")
        self.connect("vlm.wing.chords", "strips.chords")
        self.connect("vlm.wing.widths", "strips.widths")
        self.connect("vlm.wing.S_ref", "strips.S_ref")

        self.add_subsystem("clmax", CriticalSectionCLmax(n_strips=n, max_mode=self.options["max_mode"],
                                                         rho_ks=self.options["rho_ks"]),
                           promotes_outputs=["CL_max", "CL_max_over_CL"])
        self.connect("vlm.wing_perf.CL", "clmax.CL")
        self.connect("vlm.wing_perf.Cl", "clmax.Cl")
        self.connect("Clmax_sec", "clmax.Clmax_sec")

        self.add_subsystem("perf", WingPerformance(), promotes_inputs=["v", "rho"],
                           promotes_outputs=["CD", "q", "L", "D", "L_over_D"])
        self.connect("vlm.wing_perf.CL", "perf.CL")
        self.connect("vlm.wing_perf.CDi", "perf.CDi")
        self.connect("CDp", "perf.CDp")
        self.connect("vlm.wing.S_ref", "perf.S_ref")

        self.set_input_defaults("alpha", 3.0, units="deg")
        self.set_input_defaults("v", 15.0, units="m/s")
        self.set_input_defaults("rho", 1.225, units="kg/m**3")
        self.set_input_defaults("Mach_number", 0.0)
        self.set_input_defaults("re", 1e6, units="1/m")  # unused: OAS viscous drag is off
        self.set_input_defaults("cg", np.zeros(3), units="m")
        self.set_input_defaults("beta", 0.0, units="deg")
