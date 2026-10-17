from pathlib import Path

import openmdao.api as om
import pytest
from openmdao.utils.assert_utils import assert_check_partials

from aircrafter2.config import MassVerificationCase, load_yaml

ROOT = Path(__file__).resolve().parents[1]
CONFIGS = ROOT / "configs"
TUBE_SPAR_CASES = ["ubi11", "gra13", "ubi15"]


def verification_case(name):
    return MassVerificationCase(**load_yaml(CONFIGS / "verification" / f"{name}.yaml"))


@pytest.fixture(params=TUBE_SPAR_CASES)
def tube_spar_case(request):
    return verification_case(request.param)


def run_component(comp, **inputs):
    """One-component problem (complex-step allocated), inputs set, model run once."""
    p = om.Problem(reports=False)
    p.model.add_subsystem("c", comp)
    p.setup(force_alloc_complex=True)
    for k, v in inputs.items():
        p.set_val(f"c.{k}", v)
    p.run_model()
    return p


def check_partials_fd(p, atol=1e-5, rtol=1e-4, skip=()):
    """Central-FD partials check; outputs named in `skip` are left out."""
    data = p.check_partials(method="fd", form="central", step=1e-6, step_calc="rel_avg", out_stream=None,
                            compact_print=True)
    for comp in data.values():
        for key in [k for k in comp if k[0] in skip]:
            del comp[key]
    assert_check_partials(data, atol=atol, rtol=rtol)
