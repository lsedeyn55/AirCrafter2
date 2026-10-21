"""Coupled aero + mass (MDF).

    airfoil (shape DVs -> Kulfan, t/c, t at spar, A/c^2, P/c)
    geometry (b, c, l_ht, tails, cargo bay -> S, chords, l_tb, S_cb, V_h, V_v)
    MDA  [Newton, solve_subsystems, DirectSolver]
        cruise aero point  (alpha_cruise)  -> L, D, C_m,wing, C_L,alpha, C_L,max
        balance            alpha_cruise such that L = W
        G&G mass group     (its own NLBGS closure on m)  -> W, masses, tube sizing

Coupling: W (mass -> aero via the lift balance); C_m and C_L,alpha (aero -> mass, Eqs. 46, 47,
54); t/c, t at spar, A/c^2, P/c (airfoil -> mass); the tube radius follows the airfoil (D1).

The optimisation minimises cruise drag at fixed speed. With mass in the loop, a thicker section
costs profile drag but buys a lighter spar (larger tube radius), which lowers W and with it the
induced drag. That is the trade this problem exists to show.
"""

from typing import Literal

import openmdao.api as om
from pydantic import BaseModel, ConfigDict, Field

from ..aero import WingAero
from ..airfoil import AirfoilGroup, load_seed
from ..atmosphere import isa
from ..config.aero import AeroConfig, AirfoilConfig
from ..config.mass import MassConfig
from ..geometry import CargoBay, RectPlanform, TailGeometry
from ..mass.garcia_gamboa import GarciaGamboaMass, set_mass_inputs
from ..mass.garcia_gamboa.inputs import active_inputs
from .wing_aero import Bounds

SHARED_AERO = ["b", "c", "v", "rho", "mu", "upper_weights", "lower_weights", "le_weight", "te_thickness"]
# mass inputs that other disciplines provide (connected), so the config must not set them
MASS_FROM_OTHERS = {"b", "c_mean", "c_root", "S_wing", "tc", "t_at_spar", "A_c2", "P_c", "x_spar", "l_ht",
                    "l_tb", "S_ht", "S_vt", "S_cb", "Cm", "CL_alpha", "V_design", "rho", "g"}


class GeometryInputs(BaseModel):
    model_config = ConfigDict(extra="forbid")
    l_ht: float = Field(0.8, description="m, tail arm (PLACEHOLDER)")
    S_ht: float = Field(0.06, description="m^2 (PLACEHOLDER)")
    S_vt: float = Field(0.03, description="m^2 (PLACEHOLDER)")
    boom_offset: float = Field(0.15, description="m, wing c/4 to boom start (PLACEHOLDER)")
    l_cb: float = Field(0.40, description="m, cargo bay box (PLACEHOLDER)")
    w_cb: float = 0.12
    h_cb: float = 0.12
    x_spar: float = Field(0.25, description="spar position / chord (brief default)")


class AeroMassProblemConfig(BaseModel):
    model_config = ConfigDict(extra="forbid")

    airfoil: AirfoilConfig = AirfoilConfig()
    aero: AeroConfig = AeroConfig()
    mass: MassConfig = MassConfig()
    geometry: GeometryInputs = GeometryInputs()
    altitude: float = 400.0
    V: float = Field(20.0, gt=0, description="m/s, cruise = design speed for the loads (PLACEHOLDER)")
    clmax_ratio_min: float = 1.44
    wall_ratio_max: float = Field(0.2, description="t_tube / r_tube: thin-wall validity of Eqs. 105-117")
    r_tube_min: float = Field(0.005, description="m, smallest spar tube (PLACEHOLDER: 10 mm OD, OPEN_QUESTIONS Q3)")
    design_variables: dict[str, Bounds] = {
        "b": Bounds(lower=0.9, upper=1.52),
        "c": Bounds(lower=0.15, upper=0.45),
        "tc_scale": Bounds(lower=0.5, upper=1.6),
        "camber_scale": Bounds(lower=0.0, upper=2.0),
    }
    objective: Literal["D", "P_cruise"] = "D"
    tol: float = 1e-8
    maxiter: int = 100
    initial: dict[str, float] = {"b": 1.2, "c": 0.3, "tc_scale": 1.0, "camber_scale": 1.0, "alpha_cruise": 6.0}
    mda_maxiter: int = 30


def load_aero_mass_config(path):
    """Load an aero_mass YAML: ``mass_config`` names the base mass YAML; ``mass`` overrides it."""
    from pathlib import Path

    from ..config.loader import _merge, load_layered, load_yaml

    path = Path(path)
    data = load_layered(path, AeroMassProblemConfig.model_fields)  # placeholders.yaml and the ruleset underneath
    base = load_yaml(path.parent / data.pop("mass_config")) if "mass_config" in data else {}
    data["mass"] = _merge(base, data.get("mass", {}))
    return AeroMassProblemConfig(**data)


class AeroMassMDA(om.Group):
    def initialize(self):
        self.options.declare("cfg", types=AeroMassProblemConfig)

    def setup(self):
        cfg = self.options["cfg"]
        a = cfg.aero
        self.add_subsystem("cruise", WingAero(num_x=a.num_x, num_y=a.num_y, spacing=a.spacing, nf=a.nf_settings(),
                                              max_mode=a.max_mode, rho_ks=a.rho_ks),
                           promotes_inputs=SHARED_AERO + [("alpha", "alpha_cruise")])
        bal = om.BalanceComp()
        bal.add_balance("alpha_cruise", units="deg", eq_units="N", lhs_name="L_cruise", rhs_name="W",
                        val=cfg.initial.get("alpha_cruise", 6.0), normalize=True)
        self.add_subsystem("lift_balance", bal, promotes_inputs=["W"], promotes_outputs=["alpha_cruise"])
        self.connect("cruise.L", "lift_balance.L_cruise")

        mass_opts = cfg.mass.options.model_copy(update={"standalone_solver": True})
        self.add_subsystem("mass", GarciaGamboaMass(config=mass_opts),
                           promotes_inputs=["*", ("tc", "tc_max"), ("S_wing", "S"), ("V_design", "v")],
                           promotes_outputs=["*"])
        self.connect("cruise.Cm_wing", "Cm")
        self.connect("cruise.CL_alpha", "CL_alpha")

        newton = om.NewtonSolver(solve_subsystems=True, maxiter=cfg.mda_maxiter, atol=1e-10, rtol=1e-10,
                                 iprint=-1, err_on_non_converge=True)
        newton.linesearch = om.BoundsEnforceLS()
        self.nonlinear_solver = newton
        self.linear_solver = om.DirectSolver()


def add_aero_mass_model(m, cfg: AeroMassProblemConfig):
    """Add airfoil, geometry and the aero + mass MDA to a model (all promoted)."""
    m.add_subsystem("airfoil", AirfoilGroup(seed=load_seed(cfg.airfoil.seed), n_modes=cfg.airfoil.n_modes),
                    promotes=["*"])
    m.add_subsystem("planform", RectPlanform(), promotes=["*"])
    m.add_subsystem("tail", TailGeometry(), promotes=["*"])
    m.add_subsystem("cargo", CargoBay(), promotes=["*"])
    m.add_subsystem("mda", AeroMassMDA(cfg=cfg), promotes=["*"])
    m.add_subsystem("power", om.ExecComp("P_cruise = D * V", P_cruise={"units": "W"}, D={"units": "N"},
                                         V={"units": "m/s"}), promotes_outputs=["P_cruise"])
    m.connect("cruise.D", "power.D")
    m.connect("v", "power.V")

    geo = cfg.geometry
    for name, val, units in [("b", cfg.initial.get("b", 1.2), "m"), ("c", cfg.initial.get("c", 0.3), "m"),
                             ("v", cfg.V, "m/s"), ("rho", 1.2, "kg/m**3"), ("x_spar", geo.x_spar, None),
                             ("l_ht", geo.l_ht, "m"), ("S_ht", geo.S_ht, "m**2"), ("S_vt", geo.S_vt, "m**2"),
                             ("g", 9.80665, "m/s**2")]:
        m.set_input_defaults(name, val, units=units)


def add_structural_constraints(m, cfg: AeroMassProblemConfig):
    m.add_constraint("cruise.cl_margin_sec", lower=0.0)
    m.add_constraint("wall_ratio", upper=cfg.wall_ratio_max)
    m.add_constraint("r_tube", lower=cfg.r_tube_min, units="m", ref=0.01)


def apply_aero_mass_values(prob, cfg: AeroMassProblemConfig, exclude=()):
    """Set the config values on a set-up problem (flight condition, geometry, mass inputs, initial point)."""
    atm = isa(cfg.altitude)
    prob.set_val("v", cfg.V, units="m/s")
    prob.set_val("rho", atm["rho"], units="kg/m**3")
    prob.set_val("mu", atm["mu"], units="Pa*s")
    geo = cfg.geometry
    for name in ["l_ht", "S_ht", "S_vt", "boom_offset", "l_cb", "w_cb", "h_cb", "x_spar"]:
        prob.set_val(name, getattr(geo, name))
    active = set(active_inputs(cfg.mass.options)) - MASS_FROM_OTHERS - set(exclude)
    set_mass_inputs(prob, {k: v for k, v in cfg.mass.inputs.items() if k in active})
    if cfg.airfoil.te_thickness is not None:
        prob.set_val("te_thickness_in", cfg.airfoil.te_thickness)
    for name, val in cfg.initial.items():
        prob.set_val(name, val)


def build_aero_mass_problem(cfg: AeroMassProblemConfig, driver=True):
    prob = om.Problem(reports=False)
    m = prob.model
    add_aero_mass_model(m, cfg)
    if driver:
        prob.driver = om.ScipyOptimizeDriver(optimizer="SLSQP", tol=cfg.tol, maxiter=cfg.maxiter, disp=True)
        units = {"b": "m", "c": "m"}
        for name, bd in cfg.design_variables.items():
            m.add_design_var(name, lower=bd.lower, upper=bd.upper, units=units.get(name))
        if cfg.objective == "D":
            m.add_objective("cruise.D", ref=1.0)
        else:
            m.add_objective("P_cruise", ref=10.0)
        m.add_constraint("cruise.CL_max_over_CL", lower=cfg.clmax_ratio_min)
        add_structural_constraints(m, cfg)
    prob.setup()
    apply_aero_mass_values(prob, cfg)
    return prob
