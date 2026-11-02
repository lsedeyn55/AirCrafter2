"""Stability: low-fidelity static longitudinal stability (interface ready for AVL).

See docs/modules/stability.md.
"""

from .static import StaticStability, oei_check

__all__ = ["StaticStability", "oei_check"]
