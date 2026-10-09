"""Airfoil and aero component tests: CST vs AeroSandbox, partials, totals."""

import numpy as np
import openmdao.api as om
import pytest
from openmdao.utils.assert_utils import assert_check_partials, assert_check_totals

from aircrafter2.aero.mesh import RectWingMesh
from aircrafter2.aero.performance import CriticalSectionCLmax
from aircrafter2.aero.profile_drag import NeuralFoilStripDrag
from aircrafter2.airfoil import AirfoilGeometry, AirfoilShape, load_seed
from aircrafter2.airfoil import cst
from aircrafter2.problems.wing_aero import WingAeroProblemConfig, build_wing_aero_problem


@pytest.mark.parametrize("name", ["naca4412", "e423", "s1223", "clarky"])
def test_cst_matches_aerosandbox(name):
    import aerosandbox as asb
    from aerosandbox.geometry.airfoil.airfoil_families import get_kulfan_coordinates

    ka = asb.KulfanAirfoil(name)
    args = (ka.upper_weights, ka.lower_weights, ka.leading_edge_weight, ka.TE_thickness)
    ref = get_kulfan_coordinates(lower_weights=args[1], upper_weights=args[0], leading_edge_weight=args[2],
                                 TE_thickness=args[3], n_points_per_side=201)
    assert np.abs(cst.coordinates(*args, 201) - ref).max() < 1e-12
    g = cst.section_geometry(*args, 0.25)
    af = ka.to_airfoil()
    assert g["tc_max"] == pytest.approx(af.max_thickness(), rel=2e-3)
    assert g["A_c2"] == pytest.approx(af.area(), rel=1e-3)


def test_s1223_matches_garcia_gamboa_table12():
    """G&G Table 12 lists A/c^2 = 0.0649 and P/c = 2.088 for GRA13's S1223 (PLAN G2)."""
    s = load_seed("s1223")
    g = cst.section_geometry(s["upper_weights"], s["lower_weights"], s["leading_edge_weight"], s["TE_thickness"], 0.25)
    assert g["A_c2"] == pytest.approx(0.0649, rel=0.01)
    assert g["P_c"] == pytest.approx(2.088, rel=0.01)


def _check(comp, inputs=None, method="cs", atol=1e-6, rtol=1e-6, step=None, skip_outputs=()):
    prob = om.Problem(reports=False)
    prob.model.add_subsystem("comp", comp)
    prob.setup(force_alloc_complex=(method == "cs"))
    for k, v in (inputs or {}).items():
        prob.set_val(f"comp.{k}", v)
    prob.run_model()
    kw = {"step": step, "form": "central", "step_calc": "rel_avg"} if method == "fd" else {}
    data = prob.check_partials(method=method, out_stream=None, compact_print=True, **kw)
    for comp_data in data.values():  # report-only outputs have no declared partials by design
        for key in [k for k in comp_data if k[0] in skip_outputs]:
            del comp_data[key]
    assert_check_partials(data, atol=atol, rtol=rtol)
    return prob


def test_mesh_partials():
    _check(RectWingMesh(num_x=3, num_y=7), {"b": 1.3, "c": 0.27})


@pytest.mark.parametrize("n_modes", [0, 3])
def test_shape_partials(n_modes):
    seed = load_seed("naca4412")
    inp = {"tc_scale": 0.9, "camber_scale": 1.1}
    if n_modes:
        inp.update(d_upper=np.array([0.01, -0.02, 0.005]), d_lower=np.array([0.0, 0.01, -0.01]))
    _check(AirfoilShape(seed=seed, n_modes=n_modes), inp)


def test_geometry_partials():
    s = load_seed("e423")
    # the component uses cs partials, so check against central FD
    _check(AirfoilGeometry(), {"upper_weights": s["upper_weights"], "lower_weights": s["lower_weights"],
                               "le_weight": s["leading_edge_weight"], "te_thickness": 0.002, "x_spar": 0.27},
           method="fd", step=1e-6, atol=1e-6, rtol=1e-4)


@pytest.mark.parametrize("mode", ["ks", "hard"])
def test_clmax_partials(mode):
    _check(CriticalSectionCLmax(n_strips=5, max_mode=mode),
           {"CL": 0.6, "Cl": np.array([0.3, 0.5, 0.62, 0.7, 0.72]), "Clmax_sec": np.array([1.3, 1.32, 1.35, 1.37, 1.4])})


def test_strip_drag_partials_against_fd():
    """Implicit-function partials vs finite differences through the full section solve."""
    s = load_seed("naca4412")
    n = 6
    inputs = {
        "upper_weights": s["upper_weights"], "lower_weights": s["lower_weights"],
        "le_weight": s["leading_edge_weight"], "te_thickness": s["TE_thickness"],
        "Cl": np.linspace(0.3, 0.8, n), "chords": np.linspace(0.30, 0.24, n + 1),
        "widths": np.full(n, 0.12), "S_ref": 2 * 0.27 * 0.72, "v": 18.0, "rho": 1.18, "mu": 1.8e-5,
    }
    _check(NeuralFoilStripDrag(n_strips=n), inputs, method="fd", step=1e-5, atol=1e-4, rtol=2e-3,
           skip_outputs=("nf_confidence_min", "n_stalled"))


def test_wing_aero_check_totals():
    cfg = WingAeroProblemConfig(aero={"num_y": 11})
    prob = build_wing_aero_problem(cfg)
    prob.run_model()
    data = prob.check_totals(method="fd", step=1e-6, form="central", out_stream=None)
    assert_check_totals(data, atol=1e-5, rtol=2e-3)
