"""Garcia, G.L. & Gamboa, P.V. (2022), "Mass prediction models for air cargo challenge aircraft", The Aeronautical
Journal 126, 249-296, doi:10.1017/aer.2021.68.

Structure-based (class II 1/2) mass model for the circular tube spar wing concept (Sections 2.2.5, 3.3.3), with the tail
(2.2.7), fuselage (2.2.8, 3.4) and landing gear (2.2.9) models and the mass closure (2.2.10).
See docs/modules/mass.md.
"""

from .group import GarciaGamboaMass, build_mass_problem, mass_breakdown, set_mass_inputs
from .inputs import INPUTS

__all__ = ["GarciaGamboaMass", "build_mass_problem", "mass_breakdown", "set_mass_inputs", "INPUTS"]
