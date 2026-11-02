"""Configuration: YAML loading (with ``extends``) and pydantic schemas.

Why pydantic: docs/DECISIONS.md#plan-d7. The layers and the load order: docs/modules/config.md.
"""

from .loader import load_yaml
from .mass import MassConfig, MassOptions, MassVerificationCase

__all__ = ["load_yaml", "MassConfig", "MassOptions", "MassVerificationCase"]
