"""Wing-aero problem: aero-only rectangular wing at fixed lift.

minimise   CD                       (= drag, since q and S are fixed)
w.r.t.     b, c, alpha, tc_scale, camber_scale (and bump modes, if any)
subject to L = W                    (fixed lift)
           b c = S                  (fixed wing area, i.e. fixed wing loading)
           tc_max >= tc_min         (room for the spar; PLACEHOLDER, aero_mass couples mass)
           CL_max / CL >= 1.2^2     (cruise at >= 1.2 V_stall, as in the lap modeller)
           cl_margin_sec >= 0       (no stalled strip)
"""

from typing import Literal

import openmdao.api as om
from pydantic import BaseModel, ConfigDict, Field

from ..aero import WingAero
from ..airfoil import AirfoilGroup, load_seed
from ..atmosphere import isa
from ..config.aero import AeroConfig, AirfoilConfig


class Bounds(BaseModel):
    model_config = ConfigDict(extra="forbid")
    lower: float
    upper: float


class WingAeroProblemConfig(BaseModel):
    model_config = ConfigDict(extra="forbid")

    airfoil: AirfoilConfig = AirfoilConfig()
    aero: AeroConfig = AeroConfig()
    altitude: float = Field(400.0, description="m; inputs_file.m: Wichita ~400 m")
    V: float = Field(20.0, gt=0, description="m/s")
    weight: float = Field(49.0, gt=0, description="N, the fixed lift")
    S: float = Field(0.375, gt=0, description="m^2, fixed wing area")
    tc_min: float = Field(0.10, description="PLACEHOLDER minimum t/c for the spar (OPEN_QUESTIONS Q5)")
    clmax_ratio_min: float = Field(1.44, description="(1.2)^2: V >= 1.2 V_stall")
    design_variables: dict[str, Bounds] = {
        "b": Bounds(lower=0.9, upper=1.52),  # m; 3-5 ft from inputs_file.m
        "c": Bounds(lower=0.15, upper=0.45),
        "alpha": Bounds(lower=-5.0, upper=15.0),  # deg, from zero lift (flat-plate VLM)
        "tc_scale": Bounds(lower=0.6, upper=1.5),
        "camber_scale": Bounds(lower=0.0, upper=2.0),
    }
    optimizer: Literal["SLSQP"] = "SLSQP"
    tol: float = 1e-8
    maxiter: int = 100
    initial: dict[str, float] = {"b": 1.2, "c": 0.3125, "alpha": 3.0, "tc_scale": 1.0, "camber_scale": 1.0}


def build_wing_aero_problem(cfg: WingAeroProblemConfig, driver=True):
    prob = om.Problem(reports=False)
    m = prob.model
    seed = load_seed(cfg.airfoil.seed)
    m.add_subsystem("airfoil", AirfoilGroup(seed=seed, n_modes=cfg.airfoil.n_modes), promotes=["*"])
    a = cfg.aero
    m.add_subsystem(
        "wing",
        WingAero(
            num_x=a.num_x, num_y=a.num_y, spacing=a.spacing, nf=a.nf_settings(), max_mode=a.max_mode, rho_ks=a.rho_ks
        ),
        promotes=["*"],
    )
    m.add_subsystem(
        "lift_con", om.ExecComp("lift_residual = L / W - 1.0", L={"units": "N"}, W={"units": "N"}), promotes=["*"]
    )
    m.add_subsystem(
        "area_con",
        om.ExecComp("area = b * c", area={"units": "m**2"}, b={"units": "m"}, c={"units": "m"}),
        promotes=["*"],
    )
    m.set_input_defaults("b", cfg.initial.get("b", 1.2), units="m")
    m.set_input_defaults("c", cfg.initial.get("c", 0.3), units="m")

    if driver:
        prob.driver = om.ScipyOptimizeDriver(optimizer=cfg.optimizer, tol=cfg.tol, maxiter=cfg.maxiter, disp=True)
        units = {"b": "m", "c": "m", "alpha": "deg"}
        for name, bd in cfg.design_variables.items():
            m.add_design_var(name, lower=bd.lower, upper=bd.upper, units=units.get(name))
        m.add_objective("CD", scaler=100.0)
        m.add_constraint("lift_residual", equals=0.0)
        m.add_constraint("area", equals=cfg.S, units="m**2")
        m.add_constraint("tc_max", lower=cfg.tc_min)
        m.add_constraint("CL_max_over_CL", lower=cfg.clmax_ratio_min)
        m.add_constraint("cl_margin_sec", lower=0.0)

    prob.setup()
    atm = isa(cfg.altitude)
    prob.set_val("v", cfg.V, units="m/s")
    prob.set_val("rho", atm["rho"], units="kg/m**3")
    prob.set_val("mu", atm["mu"], units="Pa*s")
    prob.set_val("lift_con.W", cfg.weight, units="N")
    if cfg.airfoil.te_thickness is not None:
        prob.set_val("te_thickness_in", cfg.airfoil.te_thickness)
    for name, val in cfg.initial.items():
        prob.set_val(name, val)
    return prob


def load_wing_aero_config(path):
    """A wing_aero YAML over its layers (placeholders.yaml, the ruleset): configs/wing_aero.yaml."""
    from ..config.loader import load_layered

    return WingAeroProblemConfig(**load_layered(path, WingAeroProblemConfig.model_fields))
