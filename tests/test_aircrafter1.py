"""AirCrafter 1.0 reference copy (reference/aircrafter1): mode 'original' reproduces the shipped code exactly.

Runs check_original.m in MATLAB (about 2 min) and reads its JSON; skipped when MATLAB is not installed.
"""

import json
import shutil
import subprocess

import pytest

from .conftest import ROOT

REF = ROOT / "reference" / "aircrafter1"


def test_shipped_copy_is_verbatim():
    import hashlib

    for line in (REF / "shipped_copy" / "SHA256SUMS").read_text().splitlines():
        digest, name = line.split()
        assert hashlib.sha256((REF / "shipped_copy" / name).read_bytes()).hexdigest() == digest, name


@pytest.mark.skipif(shutil.which("matlab") is None, reason="MATLAB not installed")
def test_original_mode_reproduces_the_shipped_code(tmp_path):
    if not (REF / "apc_txt_tested").exists() or not (REF / "baseline" / "shipped_tested.mat").exists():
        pytest.skip("APC .txt links or the shipped baseline are missing (reference/aircrafter1/README.md)")
    # output to a file, not a pipe: MATLAB's detached MathWorksServiceHost inherits the pipe and keeps it open after
    # MATLAB exits, so capture_output would block until the timeout
    with open(tmp_path / "matlab.log", "w") as log:
        subprocess.run(["matlab", "-batch", "run('reference/aircrafter1/check_original.m')"], cwd=ROOT, check=True,
                       stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT, timeout=1800)
    res = json.loads((REF / "baseline" / "check_original.json").read_text())
    failed = [k for k, v in res.items() if v is False]
    assert res["all_equal"], failed
