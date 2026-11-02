"""Oval-lap problem: minimise the lap time of a simple oval at fixed cruise speed.

One wing and airfoil, two aero points at the same speed:

    cruise: alpha_cruise such that L = W                (equality constraint)
    turn:   alpha_turn; load factor n = L_turn / W (KS-capped at n_struct)

    minimise   lap_time = (2 L_s + 2 pi R) / V,   R = V^2 / (g sqrt(n^2 - 1))
    w.r.t.     b, c, alpha_cruise, alpha_turn, tc_scale, camber_scale
    subject to L_cruise = W
               CL_max / CL_turn   >= turn_clmax_ratio_min   (the C_L margin that sets the turn radius)
               CL_max / CL_cruise >= 1.44                    (cruise at >= 1.2 V_stall)
               tc_max >= tc_min                              (spar placeholder)
               cl_margin_sec >= 0 at both points
               P_turn = D_turn V / eta <= P_available        (optional)

Mass is fixed (W) here; aero_mass couples the G&G mass model. Without a power limit the problem
favours a large wing, because at fixed speed only the turn radius matters, and a larger wing has
more lift to spare. The bounds on b and c, plus the optional power constraint, keep it finite.
"""

from typing import Literal

import openmdao.api as om
from pydantic import BaseModel, ConfigDict, Field

from ..aero import WingAero
from ..airfoil import AirfoilGroup, load_seed
from ..atmosphere import isa
from ..config.aero import AeroConfig, AirfoilConfig
from ..mission import LoadFactor, OvalLapTime
from .wing_aero import Bounds

FT = 0.3048


class OvalLapProblemConfig(BaseModel):
    model_config = ConfigDict(extra="forbid")

    airfoil: AirfoilConfig = AirfoilConfig()
    aero: AeroConfig = AeroConfig()
    altitude: float = 400.0
    V: float = Field(20.0, gt=0, description="m/s, PLACEHOLDER fixed cruise and turn speed")
    weight: float = Field(49.0, gt=0, description="N, PLACEHOLDER fixed weight (aero_mass couples the G&G mass model)")
    straight_length: float = Field(1000 * FT, description="m; 1000 ft, the long leg in performance_modeller.m")
    n_struct: float = Field(5.0, description="structural load factor limit, assumptions_file.m")
    turn_clmax_ratio_min: float = Field(
        1.21, description="PLACEHOLDER: turn at <= C_L,max / 1.21 (1.1 speed margin to stall in the turn)"
    )
    clmax_ratio_min: float = Field(1.44, description="(1.2)^2 cruise margin, as in the lap modeller")
    tc_min: float = 0.10
    power_available: float | None = Field(None, description="W; optional turn power limit (PLACEHOLDER)")
    eta: float = Field(0.7, description="propulsive efficiency, assumptions_file.m")
    design_variables: dict[str, Bounds] = {
        "b": Bounds(lower=0.9, upper=1.52),
        "c": Bounds(lower=0.15, upper=0.45),
        "alpha_cruise": Bounds(lower=-5.0, upper=15.0),
        "alpha_turn": Bounds(lower=-5.0, upper=30.0),  # deg from zero lift; the C_L margin limits it
        "tc_scale": Bounds(lower=0.6, upper=1.5),
        "camber_scale": Bounds(lower=0.0, upper=2.0),
    }
    optimizer: Literal["SLSQP"] = "SLSQP"
    tol: float = 1e-8
    maxiter: int = 100
    initial: dict[str, float] = {
        "b": 1.2,
        "c": 0.3,
        "alpha_cruise": 6.0,
        "alpha_turn": 10.0,
        "tc_scale": 1.0,
        "camber_scale": 1.0,
    }


SHARED = ["b", "c", "v", "rho", "mu", "upper_weights", "lower_weights", "le_weight", "te_thickness"]


def build_oval_lap_problem(cfg: OvalLapProblemConfig, driver=True):
    prob = om.Problem(reports=False)
    m = prob.model
    a = cfg.aero
    m.add_subsystem(
        "airfoil", AirfoilGroup(seed=load_seed(cfg.airfoil.seed), n_modes=cfg.airfoil.n_modes), promotes=["*"]
    )
    for point in ("cruise", "turn"):
        m.add_subsystem(
            point,
            WingAero(
                num_x=a.num_x,
                num_y=a.num_y,
                spacing=a.spacing,
                nf=a.nf_settings(),
                max_mode=a.max_mode,
                rho_ks=a.rho_ks,
            ),
            promotes_inputs=SHARED + [("alpha", f"alpha_{point}")],
        )

    m.add_subsystem(
        "lift_con",
        om.ExecComp("lift_residual = L / W - 1.0", L={"units": "N"}, W={"units": "N"}),
        promotes_inputs=["W"],
        promotes_outputs=["lift_residual"],
    )
    m.connect("cruise.L", "lift_con.L")
    m.add_subsystem(
        "load",
        LoadFactor(max_mode=a.max_mode, rho_ks=a.rho_ks),
        promotes_inputs=["W", "n_struct"],
        promotes_outputs=["n", "n_aero"],
    )
    m.connect("turn.L", "load.L_turn")
    m.add_subsystem(
        "lap",
        OvalLapTime(),
        promotes_inputs=["n", "straight_length", "g"],
        promotes_outputs=["lap_time", "turn_radius", "turn_rate"],
    )
    m.connect("v", "lap.V")
    m.add_subsystem(
        "power",
        om.ExecComp("P_turn = D * V / eta", P_turn={"units": "W"}, D={"units": "N"}, V={"units": "m/s"}),
        promotes_inputs=["eta"],
        promotes_outputs=["P_turn"],
    )
    m.connect("turn.D", "power.D")
    m.connect("v", "power.V")
    m.add_subsystem(
        "area", om.ExecComp("S = b * c", S={"units": "m**2"}, b={"units": "m"}, c={"units": "m"}), promotes=["*"]
    )

    m.set_input_defaults("b", cfg.initial.get("b", 1.2), units="m")
    m.set_input_defaults("c", cfg.initial.get("c", 0.3), units="m")
    m.set_input_defaults("v", cfg.V, units="m/s")
    m.set_input_defaults("W", cfg.weight, units="N")

    if driver:
        prob.driver = om.ScipyOptimizeDriver(optimizer=cfg.optimizer, tol=cfg.tol, maxiter=cfg.maxiter, disp=True)
        units = {"b": "m", "c": "m", "alpha_cruise": "deg", "alpha_turn": "deg"}
        for name, bd in cfg.design_variables.items():
            m.add_design_var(name, lower=bd.lower, upper=bd.upper, units=units.get(name))
        m.add_objective("lap_time", ref=10.0)
        m.add_constraint("lift_residual", equals=0.0)
        m.add_constraint("turn.CL_max_over_CL", lower=cfg.turn_clmax_ratio_min)
        m.add_constraint("cruise.CL_max_over_CL", lower=cfg.clmax_ratio_min)
        m.add_constraint("tc_max", lower=cfg.tc_min)
        m.add_constraint("cruise.cl_margin_sec", lower=0.0)
        m.add_constraint("turn.cl_margin_sec", lower=0.0)
        if cfg.power_available is not None:
            m.add_constraint("P_turn", upper=cfg.power_available, ref=100.0)

    prob.setup()
    atm = isa(cfg.altitude)
    prob.set_val("v", cfg.V, units="m/s")
    prob.set_val("rho", atm["rho"], units="kg/m**3")
    prob.set_val("mu", atm["mu"], units="Pa*s")
    prob.set_val("W", cfg.weight, units="N")
    prob.set_val("n_struct", cfg.n_struct)
    prob.set_val("straight_length", cfg.straight_length, units="m")
    prob.set_val("g", 9.80665, units="m/s**2")
    prob.set_val("eta", cfg.eta)
    if cfg.airfoil.te_thickness is not None:
        prob.set_val("te_thickness_in", cfg.airfoil.te_thickness)
    for name, val in cfg.initial.items():
        prob.set_val(name, val)
    return prob
