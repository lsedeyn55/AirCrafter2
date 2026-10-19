"""Command-line interface: `aircrafter2 run --analyse` writes the full set of outputs."""

from .conftest import CONFIGS


def test_cli_analyse_writes_outputs(tmp_path):
    from aircrafter2.cli import main

    cfg_file = tmp_path / "cfg.yaml"
    text = (CONFIGS / "full_mdo.yaml").read_text().replace("num_y: 21", "num_y: 11")
    text = text.replace("mass_config: mass_garcia_gamboa_2022.yaml", f"mass_config: {CONFIGS / 'mass_garcia_gamboa_2022.yaml'}")
    cfg_file.write_text(text)
    out = tmp_path / "out"
    assert main(["run", str(cfg_file), "--analyse", "--out", str(out)]) == 0
    for f in ["summary.md", "results.json", "n2.html", "xdsm.tex", "xdsm.tikz", "xdsm.pdf"]:
        assert (out / f).exists(), f
    for f in ["planform", "airfoil", "spanwise_lift", "drag_breakdown", "mass_breakdown"]:
        assert (out / "figures" / f"{f}.pdf").exists() and (out / "figures" / f"{f}.png").exists()


def test_cli_robust_flag_needs_uncertain_parameters(capsys):
    import pytest

    from aircrafter2.cli import main

    assert main(["run", str(CONFIGS / "full_mdo.yaml"), "--robust", "--analyse", "--no-figures"]) == 2
    assert "robust: block" in capsys.readouterr().err
    with pytest.raises(SystemExit):  # mutually exclusive
        main(["run", str(CONFIGS / "full_mdo_rdo.yaml"), "--robust", "--no-robust"])
