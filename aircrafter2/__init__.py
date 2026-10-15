"""AirCrafter 2.0: a DBF-specific MDAO framework built on OpenMDAO.

See docs/PLAN.md for the architecture and module interfaces.
"""

import os
import tempfile
from pathlib import Path

__version__ = "0.1.0"


def _default_openmdao_workdir():
    """Scratch folder for OpenMDAO's per-problem ``<problem name>_out/`` directories (coloring files etc.).

    OpenMDAO puts them in ``OPENMDAO_WORKDIR``, else the current directory, which littered the repo root. In a
    source checkout they go to the git-ignored ``outputs/_openmdao/``, otherwise to the system temp folder.
    """
    root = Path(__file__).resolve().parents[1]
    base = root / "outputs" if (root / "pyproject.toml").exists() else Path(tempfile.gettempdir()) / "aircrafter2"
    return base / "_openmdao"


if not os.environ.get("OPENMDAO_WORKDIR"):  # a user setting wins
    _workdir = _default_openmdao_workdir()
    _workdir.mkdir(parents=True, exist_ok=True)  # OpenMDAO creates only the leaf <name>_out/ itself
    os.environ["OPENMDAO_WORKDIR"] = str(_workdir)
