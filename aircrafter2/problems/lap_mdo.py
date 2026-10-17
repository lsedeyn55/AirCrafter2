"""The AirCrafter lap modeller coupled to aero + mass (MDF).

    [aero_mass model: airfoil, geometry, MDA{cruise aero, L = W balance, G&G mass}]
    mid and near-C_L,max aero points -> three-point drag polar CD = CD0 + CD1 CL + k CL^2
    SmoothLapModel(m = m_total, S, V, polar, CL_max)  -> Lap1time, ThreeLapTime, laps_frac, margins

    minimise   ThreeLapTime  (placeholder objective; year-specific scoring comes later)
    w.r.t.     b, c, tc_scale, camber_scale, V
    subject to stall_margin >= 0 (V >= 1.2 V_stall), turn_speed_margin >= 0,
               Preq <= power_available (fixed PLACEHOLDER: no propulsion model here),
               energy for 3 laps <= usable energy, spar wall ratio and minimum tube radius,
               no stalled strip at cruise.

The lap model replaces the static Cd0, e, Clmax and fixed mass of AirCrafter 1.0 with the
coupled outputs. The propulsion efficiency eta stays the 1.0 assumption (0.7); full_mdo replaces both eta and the
power limit with the powertrain model.
"""

import openmdao.api as om
from pydantic import Field

from ..aero import WingAero
from ..aero.polar import ThreePointPolar, TurnAlpha
from ..mission.smooth_lap import SmoothLapModel
from .aero_mass import (SHARED_AERO, AeroMassProblemConfig, add_aero_mass_model, add_structural_constraints,
                         apply_aero_mass_values)
from .wing_aero import Bounds


class LapMDOConfig(AeroMassProblemConfig):
    polar_turn_frac: float = Field(0.95, description="polar's high point at C_L = frac * C_L,max")
    eta: float = Field(0.7, description="propulsive efficiency, assumptions_file.m (no propulsion model here)")
    mu_roll: float = Field(0.02, description="rolling friction, assumptions_file.m")
    takeoff_distance: float = Field(100.0, description="m, inputs_file.m")
    mission_time: float = Field(300.0, description="s, inputs_file.m")
    E_avail: float = Field(0.8 * 100 * 3600.0, description="J, 80 % of 100 Wh (assumptions_file.m)")
    n_struct: float = Field(5.0, description="assumptions_file.m")
    power_available: float = Field(300.0, description="W electrical, fixed PLACEHOLDER: this problem has no propulsion "
                                   "model; full_mdo computes the available power from the powertrain")
    lap_objective: str = Field("ThreeLapTime", description="placeholder objective (full_mdo uses a scoring plug-in)")


def default_lap_dvs():
    return {
        "b": Bounds(lower=0.9, upper=1.52),
        "c": Bounds(lower=0.15, upper=0.45),
        "tc_scale": Bounds(lower=0.5, upper=1.6),
        "camber_scale": Bounds(lower=0.0, upper=2.0),
        "v": Bounds(lower=10.0, upper=30.0),  # m/s, inputs_file.m V range
    }


def add_lap_model(model, cfg: LapMDOConfig):
    a = cfg.aero
    model.add_subsystem("turn_alpha", TurnAlpha(frac=cfg.polar_turn_frac))
    model.connect("alpha_cruise", "turn_alpha.alpha_c")
    model.connect("cruise.vlm.wing_perf.CL", "turn_alpha.CL_c")
    model.connect("cruise.CL_max", "turn_alpha.CL_max")
    points = [("mid", "turn_alpha.alpha_m"), ("turn", "turn_alpha.alpha_t")]
    for name, src in points:  # analysis points must run before the polar fit
        model.add_subsystem(name, WingAero(num_x=a.num_x, num_y=a.num_y, spacing=a.spacing, nf=a.nf_settings(),
                                           max_mode=a.max_mode, rho_ks=a.rho_ks), promotes_inputs=SHARED_AERO)
        model.connect(src, f"{name}.alpha")
    model.add_subsystem("polar", ThreePointPolar(), promotes_outputs=["CD0", "CD1", "k"])
    model.connect("cruise.vlm.wing_perf.CL", "polar.CL_0")
    model.connect("cruise.CD", "polar.CD_0")
    for i, (name, _) in enumerate(points, start=1):
        model.connect(f"{name}.vlm.wing_perf.CL", f"polar.CL_{i}")
        model.connect(f"{name}.CD", f"polar.CD_{i}")

    model.add_subsystem("lap", SmoothLapModel(rho_ks=a.rho_ks),
                        promotes_inputs=["CD0", "CD1", "k", "rho", "g", ("V", "v"), ("S", "S"), ("m", "m_total")],
                        promotes_outputs=["*"])
    model.connect("cruise.CL_max", ["lap.CL_max", "lap.CL_to"])  # no flaps modelled: take-off at C_L,max


LAP_INPUTS = ["eta", "mu_roll", "takeoff_distance", "mission_time", "E_avail", "n_struct"]


def build_lap_mdo_problem(cfg: LapMDOConfig, driver=True):
    prob = om.Problem(reports=False)
    m = prob.model
    add_aero_mass_model(m, cfg)
    add_lap_model(m, cfg)
    if driver:
        prob.driver = om.ScipyOptimizeDriver(optimizer="SLSQP", tol=cfg.tol, maxiter=cfg.maxiter, disp=True)
        units = {"b": "m", "c": "m", "v": "m/s"}
        for name, bd in cfg.design_variables.items():
            m.add_design_var(name, lower=bd.lower, upper=bd.upper, units=units.get(name))
        m.add_objective(cfg.lap_objective, ref=100.0)
        m.add_constraint("stall_margin", lower=0.0, ref=1.0)
        m.add_constraint("turn_speed_margin", lower=0.0, ref=1.0)
        m.add_constraint("Preq", upper=cfg.power_available, ref=100.0)
        m.add_constraint("energy_3laps", upper=cfg.E_avail, ref=1e4)
        add_structural_constraints(m, cfg)
    prob.setup()
    apply_aero_mass_values(prob, cfg)
    for name in LAP_INPUTS:
        prob.set_val(f"lap.{name}", getattr(cfg, name))
    return prob


def load_lap_mdo_config(path):
    """Like load_aero_mass_config, for lap_mdo YAMLs."""
    from pathlib import Path

    from ..config.loader import _merge, load_yaml

    path = Path(path)
    data = load_yaml(path)
    base = load_yaml(path.parent / data.pop("mass_config")) if "mass_config" in data else {}
    data["mass"] = _merge(base, data.get("mass", {}))
    if "design_variables" not in data:
        data["design_variables"] = default_lap_dvs()
    return LapMDOConfig(**data)
