"""Airfoil shape parameterisation and section geometry components.

Reduced shape modes on a seed Kulfan airfoil, to keep the design variables few (docs/DECISIONS.md#plan-d3). Turns
tc_scale, camber_scale and optional bump modes into Kulfan parameters, which feed the NeuralFoil strips, and section
geometry, which feeds the mass model. Equations: docs/modules/aero.md, "Airfoil".
"""

import numpy as np
import openmdao.api as om

from . import cst


def mode_basis(n_modes):
    """The matrix B (8 x n_modes) of the bump modes.

    B linearly interpolates n_modes control values spread over the 8 Bernstein weights (leading edge to trailing edge),
    so each mode is a smooth local bump.

    Parameters
    ----------
    n_modes : int
        Number of bump modes.

    Returns
    -------
    array, shape (8, n_modes)
    """
    if n_modes == 0:
        return np.zeros((cst.N_W, 0))
    if n_modes == 1:
        return np.ones((cst.N_W, 1))
    pos = np.linspace(0.0, cst.N_W - 1, n_modes)
    B = np.zeros((cst.N_W, n_modes))
    for i in range(cst.N_W):
        for k in range(n_modes):
            B[i, k] = max(0.0, 1.0 - abs(i - pos[k]) / (pos[1] - pos[0]))
    return B


class AirfoilShape(om.ExplicitComponent):
    """Reduced shape design variables -> Kulfan parameters. Linear, so partials are constant.

        thickness weights  w_t = (u0 - l0) / 2        camber weights  w_c = (u0 + l0) / 2
        upper = camber_scale * w_c + tc_scale * w_t + B @ d_upper
        lower = camber_scale * w_c - tc_scale * w_t + B @ d_lower
        le_weight = camber_scale * le0      (the LEM term is added to both surfaces: a camber mode)

    n_modes = 0 gives the 2-variable (tc_scale, camber_scale) family.

    Options
    -------
    seed : dict
        Kulfan dict: upper_weights, lower_weights, leading_edge_weight, TE_thickness.
    n_modes : int

    Inputs
    ------
    tc_scale, camber_scale
    d_upper, d_lower : arrays (n_modes,), only with modes
    te_thickness_in : trailing-edge thickness, a separate input (a buildability choice)

    Outputs
    -------
    upper_weights, lower_weights (8,), le_weight, te_thickness
    """

    def initialize(self):
        self.options.declare(
            "seed", types=dict, desc="Kulfan dict: upper_weights, lower_weights, leading_edge_weight, TE_thickness"
        )
        self.options.declare("n_modes", default=0, types=int)

    def setup(self):
        s = self.options["seed"]
        u0, l0 = np.asarray(s["upper_weights"], float), np.asarray(s["lower_weights"], float)
        self._wt, self._wc = 0.5 * (u0 - l0), 0.5 * (u0 + l0)
        self._le0 = float(s["leading_edge_weight"])
        self._B = mode_basis(self.options["n_modes"])
        nm = self.options["n_modes"]

        self.add_input("tc_scale", 1.0)
        self.add_input("camber_scale", 1.0)
        if nm:
            self.add_input("d_upper", np.zeros(nm))
            self.add_input("d_lower", np.zeros(nm))
        self.add_input("te_thickness_in", float(s["TE_thickness"]))

        self.add_output("upper_weights", u0)
        self.add_output("lower_weights", l0)
        self.add_output("le_weight", self._le0)
        self.add_output("te_thickness", float(s["TE_thickness"]))

        self.declare_partials("upper_weights", "tc_scale", val=self._wt)
        self.declare_partials("upper_weights", "camber_scale", val=self._wc)
        self.declare_partials("lower_weights", "tc_scale", val=-self._wt)
        self.declare_partials("lower_weights", "camber_scale", val=self._wc)
        self.declare_partials("le_weight", "camber_scale", val=self._le0)
        self.declare_partials("te_thickness", "te_thickness_in", val=1.0)
        if nm:
            self.declare_partials("upper_weights", "d_upper", val=self._B)
            self.declare_partials("lower_weights", "d_lower", val=self._B)

    def compute(self, inputs, outputs):
        ts, cs = inputs["tc_scale"], inputs["camber_scale"]
        up = cs * self._wc + ts * self._wt
        lo = cs * self._wc - ts * self._wt
        if self.options["n_modes"]:
            up = up + self._B @ inputs["d_upper"]
            lo = lo + self._B @ inputs["d_lower"]
        outputs["upper_weights"] = up
        outputs["lower_weights"] = lo
        outputs["le_weight"] = cs * self._le0
        outputs["te_thickness"] = inputs["te_thickness_in"]


class AirfoilGeometry(om.ExplicitComponent):
    """Kulfan parameters -> t/c, thickness at the spar, A/c^2, P/c (feeds the mass model).

    Exact complex-step partials (cst.py is complex-safe).

    Inputs
    ------
    upper_weights, lower_weights (8,), le_weight, te_thickness, x_spar

    Outputs
    -------
    tc_max, x_tc_max, t_at_spar, A_c2, P_c : see cst.section_geometry
    """

    def setup(self):
        self.add_input("upper_weights", 0.2 * np.ones(cst.N_W))
        self.add_input("lower_weights", -0.2 * np.ones(cst.N_W))
        self.add_input("le_weight", 0.0)
        self.add_input("te_thickness", 0.0)
        self.add_input("x_spar", 0.25)
        for name in ["tc_max", "x_tc_max", "t_at_spar", "A_c2", "P_c"]:
            self.add_output(name, 0.1)
        self.declare_partials("*", "*", method="cs")

    def compute(self, inputs, outputs):
        g = cst.section_geometry(
            inputs["upper_weights"],
            inputs["lower_weights"],
            inputs["le_weight"][0],
            inputs["te_thickness"][0],
            inputs["x_spar"][0],
        )
        for k, v in g.items():
            outputs[k] = v


class AirfoilGroup(om.Group):
    """Shape design variables -> Kulfan parameters -> section geometry. Promotes everything.

    Options
    -------
    seed : dict
        Kulfan dict (seeds.load_seed).
    n_modes : int
    """

    def initialize(self):
        self.options.declare("seed", types=dict)
        self.options.declare("n_modes", default=0, types=int)

    def setup(self):
        self.add_subsystem(
            "shape", AirfoilShape(seed=self.options["seed"], n_modes=self.options["n_modes"]), promotes=["*"]
        )
        self.add_subsystem("geometry", AirfoilGeometry(), promotes=["*"])


def recover_tc_scale(seed, tc_max, camber_scale=1.0, x_spar=0.25):
    """The tc_scale that gives section t/c = tc_max (n_modes = 0 family): the inverse of AirfoilShape +
    section_geometry.

    Thickness y_u - y_l = C(x) B(x) (2 tc_scale w_t) + x te: camber_scale and the leading-edge term cancel, and t/c
    rises monotonically with tc_scale, so a 1-D root solve recovers it to solver precision. Used to rebuild stored
    results that kept tc_max but not tc_scale.

    Parameters
    ----------
    seed : dict
        Kulfan dict.
    tc_max, camber_scale, x_spar : float
        The target t/c, the camber scale, and the spar station (chord fraction).

    Returns
    -------
    float
        tc_scale.
    """
    from scipy.optimize import brentq

    u0, l0 = np.asarray(seed["upper_weights"], float), np.asarray(seed["lower_weights"], float)
    wt, wc = 0.5 * (u0 - l0), 0.5 * (u0 + l0)
    le, te = camber_scale * float(seed["leading_edge_weight"]), float(seed["TE_thickness"])

    def gap(ts):
        """t/c at tc_scale ts, minus the target."""
        g = cst.section_geometry(camber_scale * wc + ts * wt, camber_scale * wc - ts * wt, le, te, x_spar)
        return g["tc_max"] - tc_max

    return brentq(gap, 0.05, 5.0, xtol=1e-14, rtol=1e-14)
