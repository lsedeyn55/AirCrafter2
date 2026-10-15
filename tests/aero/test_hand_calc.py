"""Step 3 sanity check: NeuralFoil + VLM drag against hand calculations for one airfoil.

Wing: rectangular, AR 6 (b = 1.5 m, c = 0.25 m), NACA 4412, 15 m/s, sea level.
"""

import numpy as np
import openmdao.api as om
import pytest

from aircrafter2.aero import WingAero
from aircrafter2.airfoil import AirfoilGroup, load_seed
from aircrafter2.atmosphere import isa
from aircrafter2.verification.hand_calcs import hand_wing_drag, lifting_line_rect, trefftz_induced_drag

B, C, V = 1.5, 0.25, 15.0
AR = B / C


@pytest.fixture(scope="module")
def wing():
    seed = load_seed("naca4412")
    p = om.Problem(reports=False)
    p.model.add_subsystem("airfoil", AirfoilGroup(seed=seed), promotes=["*"])
    p.model.add_subsystem("wing", WingAero(num_x=2, num_y=41), promotes=["*"])
    p.setup()
    a = isa(0.0)
    p.set_val("b", B)
    p.set_val("c", C)
    p.set_val("v", V)
    p.set_val("rho", a["rho"])
    p.set_val("mu", a["mu"])
    p.set_val("alpha", 6.5)  # deg from zero lift -> CL ~ 0.5
    p.run_model()
    return p, seed, a


def test_trefftz_matches_near_field(wing):
    p, _, a = wing
    nx, ny = 2, 41
    G = p.get_val("vlm.circulations").reshape(nx - 1, ny - 1)[-1]
    y = p.get_val("mesh.mesh")[0, :, 1]
    t = trefftz_induced_drag(G, y, V, a["rho"], p.get_val("vlm.wing.S_ref")[0])
    assert t["CL"] == pytest.approx(p.get_val("vlm.wing_perf.CL")[0], rel=2e-3)
    assert t["CDi"] == pytest.approx(p.get_val("vlm.wing_perf.CDi")[0], rel=5e-3)


def test_induced_drag_vs_lifting_line(wing):
    """VLM and lifting-line span efficiency agree within 5 % (they differ by ~4 % at AR 6, docs)."""
    p, _, _ = wing
    CL, CDi = p.get_val("vlm.wing_perf.CL")[0], p.get_val("vlm.wing_perf.CDi")[0]
    e_vlm = CL**2 / (np.pi * AR * CDi)
    e_llt = lifting_line_rect(AR)["e"]
    assert e_llt == pytest.approx(0.954, abs=0.002)  # classic Glauert value for AR 6
    assert e_vlm == pytest.approx(e_llt, rel=0.05)


def test_profile_drag_vs_2d(wing):
    """Strip-integrated CDp vs the 2D section drag at c_l = CL and the chord Reynolds number.

    Not expected to match exactly: the rectangular wing's tip strips run at lower c_l, where
    NACA 4412's drag is above its minimum, so the strip integral is ~5 % higher than the
    uniform-loading estimate (docs/verification/aero.md).
    """
    p, seed, a = wing
    CL = p.get_val("vlm.wing_perf.CL")[0]
    Re = a["rho"] * V * C / a["mu"]
    hand = hand_wing_drag(seed, CL, Re, AR, e=1.0)
    assert p.get_val("CDp")[0] == pytest.approx(hand["cd_2d"], rel=0.06)
    assert p.get_val("CDp")[0] > hand["cd_2d"]


def test_total_drag_vs_hand_calc(wing):
    p, seed, a = wing
    CL = p.get_val("vlm.wing_perf.CL")[0]
    Re = a["rho"] * V * C / a["mu"]
    hand = hand_wing_drag(seed, CL, Re, AR, e=lifting_line_rect(AR)["e"])
    assert p.get_val("CD")[0] == pytest.approx(hand["CD"], rel=0.05)
