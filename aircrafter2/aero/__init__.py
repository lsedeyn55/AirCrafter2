"""Aerodynamics: OpenAeroStruct VLM (aero-only) with NeuralFoil strip profile drag.
Abbreviations: VLM = vortex-lattice method.
"""

from .wing import WingAero

__all__ = ["WingAero"]
