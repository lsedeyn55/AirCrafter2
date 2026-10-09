"""Scoring plug-ins: the objective lives here, not in the disciplines."""

from . import placeholder  # noqa: F401  (registers the placeholder plug-ins)
from .base import ScoringPlugin, add_scoring, available, get_plugin, register

__all__ = ["ScoringPlugin", "add_scoring", "available", "get_plugin", "register"]
