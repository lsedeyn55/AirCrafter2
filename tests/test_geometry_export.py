"""geometry.json export (no OpenVSP needed): values equal prob.get_val, run_design round-trip, tc_scale recovery,
STL reader."""

import json

import numpy as np
import pytest

from aircrafter2.airfoil import cst, load_seed
from aircrafter2.airfoil.shape import recover_tc_scale
from aircrafter2.geometry.export import export_geometry, load_geometry
from aircrafter2.problems.full_mdo import build_full_problem, load_full_config

from .conftest import CONFIGS


def _solved(name):
    cfg = load_full_config(CONFIGS / name)
    cfg.aero.num_y = 11
    prob = build_full_problem(cfg, driver=False)
    prob.run_model()
    return cfg, prob


@pytest.fixture(scope="module")
def single():
    return _solved("full_mdo.yaml")


@pytest.fixture(scope="module")
def twin():
    return _solved("full_mdo_twin.yaml")


def g1(prob, k, units=None):
    return float(np.ravel(prob.get_val(k, units=units) if units else prob.get_val(k))[0])


SCALARS = [  # (path in geometry.json, problem variable, units)
    (("wing", "span_m"), "b", "m"), (("wing", "chord_m"), "c", "m"), (("wing", "area_m2"), "S", "m**2"),
    (("wing", "aspect_ratio"), "AR", None), (("wing", "x_spar_frac"), "x_spar", None),
    (("wing", "r_tube_m"), "r_tube", "m"), (("wing", "t_tube_m"), "t_tube", "m"),
    (("wing", "cruise_aoa_deg"), "alpha_cruise", "deg"), (("wing", "airfoil", "tc_max"), "tc_max", None),
    (("wing", "airfoil", "x_tc_max"), "x_tc_max", None), (("wing", "airfoil", "tc_scale"), "tc_scale", None),
    (("horizontal_tail", "area_m2"), "S_ht", "m**2"), (("horizontal_tail", "aspect_ratio"), "AR_ht", None),
    (("horizontal_tail", "tail_arm_m"), "l_ht", "m"), (("horizontal_tail", "tc"), "tc_tail", None),
    (("vertical_tail", "area_m2"), "S_vt", "m**2"), (("vertical_tail", "aspect_ratio"), "AR_vt", None),
    (("tail_boom", "length_m"), "l_tb", "m"), (("tail_boom", "radius_m"), "r_tb", "m"),
    (("cargo_bay", "length_m"), "l_cb", "m"), (("cargo_bay", "width_m"), "w_cb", "m"),
    (("cargo_bay", "height_m"), "h_cb", "m"), (("cargo_bay", "x_centre_m"), "stations.x_cb", "m"),
    (("propulsion", "D_prop_m"), "D_prop", "m"), (("propulsion", "x_motor_m"), "x_motor", "m"),
    (("stations", "x_cg_m"), "x_cg", "m"), (("stations", "x_np_m"), "x_np", "m"),
    (("stations", "static_margin"), "SM", None), (("stations", "x_batt_m"), "x_batt", "m"),
]
TWIN = [(("propulsion", "y_m_m"), "y_m", "m"), (("propulsion", "l_nac_m"), "l_nac", "m"),
        (("propulsion", "d_nac_m"), "d_nac", "m"), (("propulsion", "Q_nac"), "Q_nac", None),
        (("propulsion", "c_clear_m"), "c_clear", "m"),
        (("propulsion", "fus_clear_margin_m"), "fus_clear_margin", "m"),
        (("propulsion", "tip_clear_margin_m"), "tip_clear_margin", "m")]


def _get(d, path):
    for k in path:
        d = d[k]
    return d


@pytest.mark.parametrize("which", ["single", "twin"])
def test_export_values_equal_problem(which, request):
    cfg, prob = request.getfixturevalue(which)
    g = export_geometry(prob, cfg)
    json.dumps(g)  # plain JSON
    for path, name, units in SCALARS + (TWIN if which == "twin" else []):
        assert _get(g, path) == g1(prob, name, units), path
    a = g["wing"]["airfoil"]
    assert a["kulfan"]["upper_weights"] == [float(x) for x in prob.get_val("upper_weights")]
    assert a["kulfan"]["lower_weights"] == [float(x) for x in prob.get_val("lower_weights")]
    xy = cst.coordinates(prob.get_val("upper_weights"), prob.get_val("lower_weights"), g1(prob, "le_weight"),
                         g1(prob, "te_thickness"))
    np.testing.assert_array_equal(np.array(a["coordinates_selig"]), xy)
    # tails: exactly the planform() / parasite formulas; c/4 stations as stations.x_tail
    ht, vt = g["horizontal_tail"], g["vertical_tail"]
    assert ht["span_m"] == pytest.approx(np.sqrt(g1(prob, "S_ht") * g1(prob, "AR_ht")), rel=1e-15)
    assert ht["chord_m"] == pytest.approx(np.sqrt(g1(prob, "S_ht") / g1(prob, "AR_ht")), rel=1e-15)
    assert vt["chord_m"] * vt["height_m"] == pytest.approx(g1(prob, "S_vt"), rel=1e-14)
    assert ht["x_quarter_chord_m"] == pytest.approx(g1(prob, "stations.x_tail"), rel=1e-14)
    assert g["tail_boom"]["x_start_m"] + g["tail_boom"]["length_m"] / 2 == pytest.approx(
        g1(prob, "stations.x_tb"), rel=1e-14)
    assert g["stations"]["masses_kg"]["m_total"] == g1(prob, "m_total", "kg")
    assert ("y_m_m" in g["propulsion"]) == (which == "twin")
    for entry in g["mockup_assumptions"].values():
        assert {"value", "source", "note"} <= set(entry)


def test_run_design_writes_geometry_json(tmp_path):
    from aircrafter2.api import run_design

    cfg = load_full_config(CONFIGS / "full_mdo.yaml")
    cfg.aero.num_y = 11
    res = run_design(cfg, optimise=False, out_dir=tmp_path, figures=False, sensitivities=False, n2=False)
    g = load_geometry(tmp_path)
    assert g == json.loads(json.dumps(export_geometry(res.prob, cfg, optimised=False, success=True,
                                                      case_id=tmp_path.name)))
    assert g["provenance"]["optimised"] is False and g["provenance"]["n_motors"] == 1


def test_recover_tc_scale_inverts_the_shape():
    seed = load_seed("naca4412")
    for ts, cs in [(0.73, 0.32), (1.0, 1.0), (1.4, 1.8)]:
        u0, l0 = np.asarray(seed["upper_weights"]), np.asarray(seed["lower_weights"])
        wt, wc = 0.5 * (u0 - l0), 0.5 * (u0 + l0)
        tc = cst.section_geometry(cs * wc + ts * wt, cs * wc - ts * wt, cs * seed["leading_edge_weight"],
                                  seed["TE_thickness"], 0.25)["tc_max"]
        assert recover_tc_scale(seed, tc, cs) == pytest.approx(ts, rel=1e-10)


def test_read_stl_ascii_and_binary(tmp_path):
    from aircrafter2.vsp.render import read_stl

    tri = np.array([[[0.0, 0, 0], [1, 0, 0], [0, 1, 0]], [[0, 0, 1], [1, 0, 1], [0, 1, 1]]])
    (tmp_path / "a.stl").write_text("solid t\n" + "".join(
        " facet normal 0 0 1\n  outer loop\n" + "".join(f"   vertex {x} {y} {z}\n" for x, y, z in t)
        + "  endloop\n endfacet\n" for t in tri) + "endsolid t\n")
    rec = np.zeros(2, dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))
    rec["v"] = tri
    (tmp_path / "b.stl").write_bytes(b"\0" * 80 + np.uint32(2).tobytes() + rec.tobytes())
    np.testing.assert_allclose(read_stl(tmp_path / "a.stl"), tri)
    np.testing.assert_allclose(read_stl(tmp_path / "b.stl"), tri)
