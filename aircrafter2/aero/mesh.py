"""Rectangular half-wing vortex-lattice mesh for OpenAeroStruct (OAS), the first step of an aero point.

OAS convention for symmetric surfaces: y runs from -b/2 (tip) to 0 (root), with the symmetry plane at y = 0; x is
chordwise from the leading edge. Fed by the span and chord; feeds aero/wing.py. See docs/modules/aero.md, "Wing".
"""

import numpy as np
import openmdao.api as om


class RectWingMesh(om.ExplicitComponent):
    """Mesh of the half wing from span and chord. It's linear in b and c, so partials are constant.

    Options
    -------
    num_x, num_y : int
        Chordwise nodes, and spanwise nodes on the half wing.
    spacing : "cosine" or "uniform"
        Spanwise spacing.

    Inputs
    ------
    b [m], c [m]

    Outputs
    -------
    mesh [m] : array (num_x, num_y, 3)
    """

    def initialize(self):
        self.options.declare("num_x", default=3, types=int, desc="chordwise nodes")
        self.options.declare("num_y", default=11, types=int, desc="spanwise nodes on the half wing")
        self.options.declare(
            "spacing",
            default="cosine",
            values=["cosine", "uniform"],
            desc="spanwise spacing; cosine clusters panels at the tip",
        )

    def setup(self):
        nx, ny = self.options["num_x"], self.options["num_y"]
        j = np.arange(ny) / (ny - 1)
        # unit-span pattern from -1/2 (tip) to 0 (root)
        if self.options["spacing"] == "cosine":
            self._eta = -0.5 * np.cos(0.5 * np.pi * j)
        else:
            self._eta = -0.5 * (1.0 - j)
        self._eta[-1] = 0.0  # root exactly on the symmetry plane
        self._xi = np.linspace(0.0, 1.0, nx)

        self.add_input("b", 1.5, units="m")
        self.add_input("c", 0.25, units="m")
        self.add_output("mesh", np.zeros((nx, ny, 3)), units="m")

        size = nx * ny * 3
        idx = np.arange(size).reshape(nx, ny, 3)
        self.declare_partials(
            "mesh", "b", rows=idx[:, :, 1].ravel(), cols=np.zeros(nx * ny, int), val=np.tile(self._eta, nx)
        )
        self.declare_partials(
            "mesh", "c", rows=idx[:, :, 0].ravel(), cols=np.zeros(nx * ny, int), val=np.repeat(self._xi, ny)
        )

    def compute(self, inputs, outputs):
        nx, ny = self.options["num_x"], self.options["num_y"]
        mesh = np.zeros((nx, ny, 3), dtype=inputs["b"].dtype)
        mesh[:, :, 0] = self._xi[:, None] * inputs["c"]
        mesh[:, :, 1] = self._eta[None, :] * inputs["b"]
        outputs["mesh"] = mesh


def template_mesh(num_x, num_y, spacing="cosine", b=1.0, c=1.0):
    """Mesh with the same node pattern as RectWingMesh (OAS reads its orientation at setup).

    Parameters
    ----------
    num_x, num_y : int
        Mesh nodes.
    spacing : str
        "cosine" or "uniform".
    b, c : float
        Span and chord [m].

    Returns
    -------
    array, shape (num_x, num_y, 3)
        The mesh [m].
    """
    comp = RectWingMesh(num_x=num_x, num_y=num_y, spacing=spacing)
    prob = om.Problem(reports=False)
    prob.model.add_subsystem("m", comp)
    prob.setup()
    prob.set_val("m.b", b)
    prob.set_val("m.c", c)
    prob.run_model()
    return prob.get_val("m.mesh").copy()
