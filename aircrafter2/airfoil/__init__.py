"""Airfoil: Kulfan/CST parameterisation, section geometry and NeuralFoil section aerodynamics."""

from .seeds import load_seed
from .shape import AirfoilGeometry, AirfoilGroup, AirfoilShape

__all__ = ["load_seed", "AirfoilGeometry", "AirfoilGroup", "AirfoilShape"]
