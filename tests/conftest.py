from pathlib import Path

import pytest

from aircrafter2.config import MassVerificationCase, load_yaml

ROOT = Path(__file__).resolve().parents[1]
CONFIGS = ROOT / "configs"
TUBE_SPAR_CASES = ["ubi11", "gra13", "ubi15"]


def verification_case(name):
    return MassVerificationCase(**load_yaml(CONFIGS / "verification" / f"{name}.yaml"))


@pytest.fixture(params=TUBE_SPAR_CASES)
def tube_spar_case(request):
    return verification_case(request.param)
