"""geometry.json: a complete, plain-JSON geometry export of a solved full-model problem.

Read by the OpenVSP mock-up builder (aircrafter2.vsp), which never touches OpenMDAO, so any saved run can be rebuilt in
3D. Coordinates are the framework's own: origin at the wing root leading edge, x aft, y to starboard, z up, metres.
Values come from prob.get_val or the config; the only derived quantities are the tail span and chord, computed exactly
as in plotting.figures.planform (HT) and aero.parasite (both tails): chord = sqrt(S / AR), span = sqrt(S AR). Anything
the framework doesn't define is in `mockup_assumptions`, with its source. See docs/modules/vsp.md.
"""

import json
import subprocess
from pathlib import Path

import numpy as np

from ..airfoil import cst
from .mockup import mockup_assumptions

SCHEMA_VERSION = 1
ROOT = Path(__file__).resolve().parents[2]
MASSES = [
    "m_wing",
    "m_ht",
    "m_vt",
    "m_tb",
    "m_cb",
    "m_gear",
    "m_motor_group",
    "m_batt",
    "m_avionics",
    "m_pay",
    "m_empty",
    "m_total",
]


def _v(prob, name, units=None):
    """One scalar output, in the given units."""
    return float(np.ravel(prob.get_val(name, units=units) if units else prob.get_val(name))[0])


def _arr(prob, name):
    """One array output as a list of floats."""
    return [float(x) for x in np.ravel(prob.get_val(name))]


def git_state(root=ROOT):
    """The commit of a checkout.

    Parameters
    ----------
    root : path

    Returns
    -------
    dict
        git_hash and git_dirty, both None if git isn't available.
    """
    try:
        h = subprocess.run(["git", "rev-parse", "HEAD"], cwd=root, capture_output=True, text=True, check=True)
        d = subprocess.run(
            ["git", "status", "--porcelain", "--untracked-files=no"],
            cwd=root,
            capture_output=True,
            text=True,
            check=True,
        )
        return {"git_hash": h.stdout.strip(), "git_dirty": bool(d.stdout.strip())}
    except OSError, subprocess.CalledProcessError:
        return {"git_hash": None, "git_dirty": None}


def export_geometry(prob, cfg, config_path=None, optimised=None, success=None, case_id=None):
    """Everything needed to rebuild the solved design in 3D.

    Parameters
    ----------
    prob : om.Problem
        A run problem.
    cfg : FullMDOConfig
    config_path, optimised, success, case_id
        Provenance fields.

    Returns
    -------
    dict
        The geometry.json content, in metres, kilograms and degrees.
    """
    from ..stability import oei_check

    pr, st = cfg.propulsion, cfg.stability  # noqa: F841
    v = lambda k, u=None: _v(prob, k, u)  # noqa: E731
    up, lo = _arr(prob, "upper_weights"), _arr(prob, "lower_weights")
    le, te = v("le_weight"), v("te_thickness")
    xy = cst.coordinates(np.array(up), np.array(lo), le, te)
    c = v("c", "m")
    S_ht, AR_ht, S_vt, AR_vt = v("S_ht", "m**2"), v("AR_ht"), v("S_vt", "m**2"), v("AR_vt")

    g = {
        "schema_version": SCHEMA_VERSION,
        "units": {"length": "m", "area": "m^2", "mass": "kg", "angle": "deg", "time": "s"},
        "frame": "origin at the wing root leading edge; x aft, y to starboard, z up (framework coordinates)",
        "provenance": {
            "case_id": case_id,
            "config_path": str(config_path) if config_path else None,
            **git_state(),
            "n_motors": pr.n_motors,
            "battery_topology": pr.battery_topology,
            "prop": pr.prop,
            "n_cells": pr.n_cells,
            "scoring_plugin": cfg.scoring.plugin,
            "optimised": optimised,
            "success": success,
            "score": v("score"),
            "ThreeLapTime_s": v("ThreeLapTime"),
        },
        "wing": {
            "span_m": v("b", "m"),
            "chord_m": c,
            "area_m2": v("S", "m**2"),
            "aspect_ratio": v("AR"),
            "planform": "rectangular",
            "x_spar_frac": v("x_spar"),
            "x_spar_m": v("x_spar") * c,
            "r_tube_m": v("r_tube", "m"),
            "t_tube_m": v("t_tube", "m"),
            "cruise_aoa_deg": v("alpha_cruise", "deg"),
            "cruise_aoa_note": "cruise AoA (no incidence is modelled)",
            "airfoil": {
                "seed": cfg.airfoil.seed,
                "tc_scale": v("tc_scale"),
                "camber_scale": v("camber_scale"),
                "kulfan": {"upper_weights": up, "lower_weights": lo, "le_weight": le, "te_thickness": te},
                "tc_max": v("tc_max"),
                "x_tc_max": v("x_tc_max"),
                "coordinates_selig": [[float(a), float(b)] for a, b in xy],
                "coordinates_note": "airfoil.cst.coordinates(): TE -> upper -> LE -> lower -> TE, x/c, y/c",
            },
        },
        "horizontal_tail": {
            "area_m2": S_ht,
            "aspect_ratio": AR_ht,
            "span_m": float(np.sqrt(S_ht * AR_ht)),
            "chord_m": float(np.sqrt(S_ht / AR_ht)),
            "x_quarter_chord_m": 0.25 * c + v("l_ht", "m"),
            "tail_arm_m": v("l_ht", "m"),
            "tc": v("tc_tail"),
            "section": "symmetric (NACA 00xx at tc)",
        },
        "vertical_tail": {
            "area_m2": S_vt,
            "aspect_ratio": AR_vt,
            "height_m": float(np.sqrt(S_vt * AR_vt)),
            "chord_m": float(np.sqrt(S_vt / AR_vt)),
            "x_quarter_chord_m": 0.25 * c + v("l_ht", "m"),
            "tc": v("tc_tail"),
            "section": "symmetric (NACA 00xx at tc)",
            "note": "single centreline surface; l_vt = l_ht (geometry.TailGeometry)",
        },
        "tail_boom": {
            "x_start_m": 0.25 * c + v("boom_offset", "m"),
            "length_m": v("l_tb", "m"),
            "radius_m": v("r_tb", "m"),
        },
        "cargo_bay": {
            "length_m": v("l_cb", "m"),
            "width_m": v("w_cb", "m"),
            "height_m": v("h_cb", "m"),
            "x_centre_m": v("stations.x_cb", "m"),
            "x_cb_frac": v("x_cb_frac"),
        },
        "propulsion": {
            "n_motors": pr.n_motors,
            "prop": pr.prop,
            "D_prop_m": v("D_prop", "m"),
            "x_motor_m": v("x_motor", "m"),
            "x_motor_note": "prop disc / motor station from the wing LE (stability.x_motor)",
        },
        "stations": {
            "x_cg_m": v("x_cg", "m"),
            "x_np_m": v("x_np", "m"),
            "static_margin": v("SM"),
            "x_batt_m": v("x_batt", "m"),
            "x_payload_m": v("stations.x_cb", "m"),
            "masses_kg": {k: v(k, "kg") for k in MASSES},
            "note": "for annotation only",
        },
        "mockup_assumptions": mockup_assumptions(cfg.mockup, pr.n_motors),
        "mockup_options": cfg.mockup.model_dump(),
    }
    if pr.n_motors == 2:
        g["propulsion"].update(
            {
                "y_m_m": v("y_m", "m"),
                "station_mode": pr.station_mode,
                "l_nac_m": v("l_nac", "m"),
                "d_nac_m": v("d_nac", "m"),
                "Q_nac": v("Q_nac"),
                "c_clear_m": v("c_clear", "m"),
                "fus_clear_margin_m": v("fus_clear_margin", "m"),
                "tip_clear_margin_m": v("tip_clear_margin", "m"),
                "prop_clear_margin_m": v("prop_clear_margin", "m"),
            }
        )
        oei = oei_check(prob, cfg)
        g["stations"]["oei"] = {
            k: (float(x) if isinstance(x, (int, float, np.floating)) and not isinstance(x, bool) else x)
            for k, x in oei.items()
        }
    return g


def write_geometry(g, path):
    """Write geometry.json.

    Parameters
    ----------
    g : dict
        From export_geometry.
    path : path

    Returns
    -------
    Path
        The file written.
    """
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(g, indent=1))
    return path


def load_geometry(path):
    """Read geometry.json.

    Parameters
    ----------
    path : path
        The file, or a run folder holding it.

    Returns
    -------
    dict
    """
    path = Path(path)
    if path.is_dir():
        path = path / "geometry.json"
    return json.loads(path.read_text())
