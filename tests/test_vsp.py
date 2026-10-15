"""OpenVSP mock-up: builds, read-back, props / nacelles per layout, CLI. OpenVSP tests skip without OpenVSP; the
'no OpenVSP' behaviour is tested everywhere by simulating a missing install."""

import pytest

from aircrafter2.cli import main
from aircrafter2.geometry.export import export_geometry, write_geometry
from aircrafter2.problems.full_mdo import build_full_problem, load_full_config

from .conftest import CONFIGS


def _run_dir(tmp_path_factory, cfg_name, label):
    cfg = load_full_config(CONFIGS / cfg_name)
    cfg.aero.num_y = 11
    prob = build_full_problem(cfg, driver=False)
    prob.run_model()
    d = tmp_path_factory.mktemp(label)
    write_geometry(export_geometry(prob, cfg, case_id=label), d / "geometry.json")
    return d


@pytest.fixture(scope="module")
def runs(tmp_path_factory):
    return {1: _run_dir(tmp_path_factory, "step7_full.yaml", "single"),
            2: _run_dir(tmp_path_factory, "step7_full_twin.yaml", "twin")}


# --- without OpenVSP (always runs) ------------------------------------------------------------------------------
def test_cli_without_openvsp_points_to_readme(monkeypatch, capsys, runs):
    import aircrafter2.vsp as V

    monkeypatch.setattr(V, "have_openvsp", lambda: False)
    assert main(["vsp", str(runs[1])]) == 2
    assert main(["run", str(CONFIGS / "step7_full.yaml"), "--analyse", "--vsp", "--out", "/dev/null/x"]) == 2
    err = capsys.readouterr().err
    assert "README" in err and "OpenVSP (optional, installed locally)" in err


def test_require_openvsp_message(monkeypatch):
    import builtins

    import aircrafter2.vsp as V

    real = builtins.__import__

    def fake(name, *a, **k):
        if name == "openvsp":
            raise ImportError("simulated")
        return real(name, *a, **k)

    monkeypatch.setattr(builtins, "__import__", fake)
    assert V.have_openvsp() is False
    with pytest.raises(RuntimeError, match="README"):
        V.require_openvsp()


# --- with OpenVSP -----------------------------------------------------------------------------------------------
@pytest.fixture(scope="module")
def built(runs):
    pytest.importorskip("openvsp")
    from aircrafter2.geometry.export import load_geometry
    from aircrafter2.vsp.pipeline import build_run

    return {n: (load_geometry(d), build_run(d, figures=(n == 2))) for n, d in runs.items()}


@pytest.mark.parametrize("n", [1, 2])
def test_build_readback_matches_geometry(built, n):
    g, res = built[n]
    rb = res["readback"]
    for key, ref in [("wing_span_m", g["wing"]["span_m"]), ("wing_area_m2", g["wing"]["area_m2"]),
                     ("ht_area_m2", g["horizontal_tail"]["area_m2"]), ("vt_area_m2", g["vertical_tail"]["area_m2"])]:
        assert rb[key] == pytest.approx(ref, rel=5e-3), key
    for path in ("vsp3", "step", "stl", "airfoil_dat", "check"):
        assert res["files"][path].exists() and res["files"][path].stat().st_size > 0, path


@pytest.mark.parametrize("n", [1, 2])
def test_props_and_nacelles_per_layout(built, n):
    g, res = built[n]
    rb = res["readback"]
    assert rb["n_props"] == g["propulsion"]["n_motors"] == n
    assert rb["n_nacelles"] == (2 if n == 2 else 0)
    assert all(d == pytest.approx(g["propulsion"]["D_prop_m"], rel=1e-9) for d in rb["prop_diameters_m"])
    if n == 2:
        assert sorted(rb["prop_y_m"]) == pytest.approx([-g["propulsion"]["y_m_m"], g["propulsion"]["y_m_m"]])


def test_check_report_and_figures(built):
    _, res = built[2]
    rows = {q: d for q, a, e, d, _ in res["check"]["rows"]}
    assert abs(rows["wing span (m)"]) < 1e-6 and abs(rows["tail arm l_ht (m)"]) < 1e-6
    assert abs(rows["fuselage clearance margin (m)"]) < 1e-6
    assert res["files"]["three_view"].exists() and res["files"]["isometric"].with_suffix(".png").exists()


def test_cli_vsp_builds(runs, tmp_path):
    pytest.importorskip("openvsp")
    assert main(["vsp", str(runs[1]), "--out", str(tmp_path), "--no-figures"]) == 0
    assert (tmp_path / "single.vsp3").exists() and (tmp_path / "vsp_check.md").exists()
