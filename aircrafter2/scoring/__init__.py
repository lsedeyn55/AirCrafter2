"""Scoring plug-ins: the objective lives here, not in the disciplines."""

from . import dbf2025, placeholder  # noqa: F401  (registers the plug-ins)
from .base import ScoringPlugin, add_scoring, available, get_plugin, register

__all__ = ["ScoringPlugin", "add_scoring", "available", "get_plugin", "register"]
