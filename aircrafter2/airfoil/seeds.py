"""Seed airfoils: a name from the AeroSandbox database (UIUC/NACA) or a coordinate file."""

from pathlib import Path

import numpy as np


def load_seed(name_or_path):
    """Return the 8+8+LE+TE Kulfan parameters of a named airfoil or a Selig/Lednicer .dat file."""
    import aerosandbox as asb

    p = Path(str(name_or_path))
    if p.suffix.lower() in {".dat", ".txt"} or p.exists():
        af = asb.Airfoil(name=p.stem, coordinates=str(p)).to_kulfan_airfoil(n_weights_per_side=8)
    else:
        af = asb.KulfanAirfoil(str(name_or_path))
    return dict(
        upper_weights=np.asarray(af.upper_weights, float),
        lower_weights=np.asarray(af.lower_weights, float),
        leading_edge_weight=float(af.leading_edge_weight),
        TE_thickness=float(af.TE_thickness),
    )
