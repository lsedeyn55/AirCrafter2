import pytest
from pydantic import ValidationError

from aircrafter2.config import MassConfig, load_yaml

from .conftest import CONFIGS


def test_default_mass_config_loads():
    cfg = MassConfig(**load_yaml(CONFIGS / "mass_garcia_gamboa_2022.yaml"))
    assert cfg.inputs["f_wing"] == 1.12
    assert cfg.options.r_tube_mode == "derived"


def test_extends_merges_over_parent():
    data = load_yaml(CONFIGS / "verification" / "ubi15.yaml")
    assert data["options"]["max_mode"] == "hard"  # from _tube_spar_common.yaml
    assert data["options"]["rho_ks"] == 100.0  # from the base file
    assert data["inputs"]["b"] == 3.324  # from ubi15.yaml
    assert data["inputs"]["f_wing"] == 1.12  # from the base file


def test_misspelt_input_is_an_error():
    with pytest.raises(ValidationError, match="unknown mass input"):
        MassConfig(inputs={"wingspan": 1.5})


def test_unknown_option_is_an_error():
    with pytest.raises(ValidationError):
        MassConfig(options={"max_mod": "hard"})
