"""Step 7: the full AirCrafter 2.0 MDF problem (decision B: propulsion sets the power).

    airfoil, geometry (planform, tails, cargo bay)
    powertrain (APC prop + motor + battery at fixed throttle) -> T(v), P(v), T_static, masses
    MDA  [Newton, solve_subsystems]:
        parasite drag (tails, optional fuselage)  at v
        cruise aero point                         at (alpha_cruise, v)
        balance alpha_cruise:  L = W
        balance v:             T(v) = D(v)        (v: the full-throttle level speed, also the G&G design speed)
        G&G mass (NLBGS closure), m_sys from the powertrain
    mid / near-C_L,max aero points -> three-point polar
    lap model (propulsion mode): T(v) and P(v) on every leg, battery energy, laps
    static stability: neutral point, CG, static margin

DVs: b, c, tc_scale, camber_scale, battery capacity, tail arm, S_ht, S_vt, battery station.
Objective: placeholder ThreeLapTime (scoring plug-ins, step 8).
"""

import numpy as np
import openmdao.api as om
from pydantic import BaseModel, ConfigDict, Field

from ..aero import WingAero
from ..aero.parasite import ParasiteDrag
from ..aero.polar import ThreePointPolar, TurnAlpha
from ..mass.garcia_gamboa import GarciaGamboaMass
from ..mission.smooth_lap import SmoothLapModel
from ..propulsion.powertrain import Powertrain
from ..stability import StaticStability
from .aero_mass import (SHARED_AERO, AeroMassProblemConfig, apply_aero_mass_values)
from .wing_aero import Bounds
from ..airfoil import AirfoilGroup, load_seed
from ..geometry import CargoBay, RectPlanform, TailGeometry


class PropulsionInputs(BaseModel):
    model_config = ConfigDict(extra="forbid")
    prop: str = Field("11x7E", description="APC PER3 name (discrete, D4)")
    throttle: float = 1.0
    n_cells: float = Field(4.0, description="series cells (discrete)")
    V_cell: float = Field(3.7, description="V nominal, PLACEHOLDER LiPo")
    R_cell: float = Field(0.005, description="ohm per cell, PLACEHOLDER")
    usable_fraction: float = Field(0.8, description="assumptions_file.m: use 80 % of the pack")
    e_spec: float = Field(150.0, description="Wh/kg pack-level, PLACEHOLDER")
    Kv: float = Field(900.0, description="rpm/V, PLACEHOLDER motor")
    Rm: float = Field(0.05, description="ohm, PLACEHOLDER motor")
    I0: float = Field(1.0, description="A, PLACEHOLDER motor")
    m_motor: float = Field(0.15, description="kg, PLACEHOLDER")
    m_esc: float = Field(0.06, description="kg, PLACEHOLDER")
    m_prop: float = Field(0.03, description="kg, PLACEHOLDER")
    m_avionics: float = Field(0.25, description="kg, PLACEHOLDER receiver, servos, wiring")
    I_max: float = Field(60.0, description="A, PLACEHOLDER motor/ESC current limit")
    rpm_fraction: float = Field(0.85, description="of APC's 150000/D limit (propSelector.m)")
    v_top: float = Field(40.0, description="m/s, top of the T(v) fit range")


class StabilityInputs(BaseModel):
    model_config = ConfigDict(extra="forbid")
    AR_ht: float = Field(4.0, description="PLACEHOLDER")
    AR_vt: float = Field(2.0, description="PLACEHOLDER")
    tc_tail: float = Field(0.09, description="PLACEHOLDER tail section t/c")
    eta_h: float = Field(0.9, description="tail dynamic-pressure ratio, PLACEHOLDER")
    wing_cg_frac: float = Field(0.40, description="wing CG / chord, PLACEHOLDER")
    x_motor: float = Field(-0.30, description="m from wing LE, PLACEHOLDER (tractor nose)")
    x_avionics: float = Field(-0.10, description="m, PLACEHOLDER")
    x_gear: float = Field(0.05, description="m, PLACEHOLDER")
    x_cb_frac: float = Field(0.25, description="cargo bay and payload centred at this chord fraction, PLACEHOLDER")
    SM_min: float = Field(0.05, description="PLACEHOLDER (OPEN_QUESTIONS Q9)")
    SM_max: float = Field(0.20, description="PLACEHOLDER")
    V_h_min: float = Field(0.35, description="PLACEHOLDER, around Raymer Table 6.4 values")
    V_h_max: float = Field(0.65, description="PLACEHOLDER")
    V_v_min: float = Field(0.02, description="PLACEHOLDER")
    V_v_max: float = Field(0.06, description="PLACEHOLDER")


class FullMDOConfig(AeroMassProblemConfig):
    propulsion: PropulsionInputs = PropulsionInputs()
    stability: StabilityInputs = StabilityInputs()
    fuselage_drag: bool = True
    polar_turn_frac: float = 0.95
    mu_roll: float = 0.02
    takeoff_distance: float = 100.0
    mission_time: float = 300.0
    n_struct: float = 5.0
    lap_objective: str = "ThreeLapTime"


def default_full_dvs():
    return {
        "b": Bounds(lower=0.9, upper=1.52),
        "c": Bounds(lower=0.15, upper=0.45),
        "tc_scale": Bounds(lower=0.5, upper=1.6),
        "camber_scale": Bounds(lower=0.0, upper=2.0),
        "capacity": Bounds(lower=0.5, upper=6.0),
        "l_ht": Bounds(lower=0.4, upper=1.2),
        "S_ht": Bounds(lower=0.02, upper=0.15),
        "S_vt": Bounds(lower=0.01, upper=0.10),
        "x_batt": Bounds(lower=-0.35, upper=0.2),
    }


class PolyAt(om.ExplicitComponent):
    """y = sum c_i v^i."""

    def initialize(self):
        self.options.declare("n", default=4, types=int)

    def setup(self):
        self.add_input("coeffs", np.zeros(self.options["n"]))
        self.add_input("v", 20.0, units="m/s")
        self.add_output("y", 1.0)
        self.declare_partials("*", "*", method="cs")

    def compute(self, inputs, outputs):
        c, v = inputs["coeffs"], inputs["v"]
        outputs["y"] = sum(c[i] * v**i for i in range(len(c)))


class FullMDA(om.Group):
    def initialize(self):
        self.options.declare("cfg")

    def setup(self):
        cfg = self.options["cfg"]
        a = cfg.aero
        self.add_subsystem("parasite", ParasiteDrag(fuselage_drag=cfg.fuselage_drag),
                           promotes_inputs=["v", "rho", "mu", "S", "S_ht", "S_vt", "AR_ht", "AR_vt", "tc_tail",
                                            "l_cb", "w_cb", "h_cb", "l_tb", "r_tb"],
                           promotes_outputs=["CD_extra", "CD_tail", "CD_fus"])
        self.add_subsystem("cruise", WingAero(num_x=a.num_x, num_y=a.num_y, spacing=a.spacing, nf=a.nf_settings(),
                                              max_mode=a.max_mode, rho_ks=a.rho_ks),
                           promotes_inputs=SHARED_AERO + [("alpha", "alpha_cruise")])
        self.connect("CD_extra", "cruise.perf.CD_extra")
        self.add_subsystem("thrust_at_v", PolyAt(), promotes_inputs=[("coeffs", "T_coeffs"), "v"])

        bal = om.BalanceComp()
        bal.add_balance("alpha_cruise", units="deg", eq_units="N", lhs_name="L_cruise", rhs_name="W",
                        val=cfg.initial.get("alpha_cruise", 6.0), normalize=True)
        bal.add_balance("v", units="m/s", eq_units="N", lhs_name="T_cruise", rhs_name="D_cruise",
                        val=cfg.initial.get("v", 22.0), normalize=True, lower=5.0, upper=cfg.propulsion.v_top)
        self.add_subsystem("balance", bal, promotes_inputs=["W"], promotes_outputs=["alpha_cruise", "v"])
        self.connect("cruise.L", "balance.L_cruise")
        self.connect("cruise.D", "balance.D_cruise")
        self.connect("thrust_at_v.y", "balance.T_cruise")

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


def build_full_model(m, cfg: FullMDOConfig):
    a, st = cfg.aero, cfg.stability
    m.add_subsystem("airfoil", AirfoilGroup(seed=load_seed(cfg.airfoil.seed), n_modes=cfg.airfoil.n_modes),
                    promotes=["*"])
    m.add_subsystem("planform", RectPlanform(), promotes=["*"])
    m.add_subsystem("tail", TailGeometry(), promotes=["*"])
    m.add_subsystem("cargo", CargoBay(), promotes=["*"])
    m.add_subsystem("propulsion", Powertrain(prop=cfg.propulsion.prop, v_top=cfg.propulsion.v_top),
                    promotes_inputs=["rho", "capacity", "throttle", "n_cells", "V_cell", "R_cell", "usable_fraction",
                                     "e_spec", "Kv", "Rm", "I0", "m_motor", "m_esc", "m_prop", "m_avionics"],
                    promotes_outputs=["T_coeffs", "P_coeffs", "T_static", "I_static", "rpm_max", "rpm_limit",
                                      "E_usable", "m_batt", "m_sys"])
    m.add_subsystem("mda", FullMDA(cfg=cfg), promotes=["*"])

    # polar from three coupled aero points (step 6)
    m.add_subsystem("turn_alpha", TurnAlpha(frac=cfg.polar_turn_frac))
    m.connect("alpha_cruise", "turn_alpha.alpha_c")
    m.connect("cruise.vlm.wing_perf.CL", "turn_alpha.CL_c")
    m.connect("cruise.CL_max", "turn_alpha.CL_max")
    points = [("mid", "turn_alpha.alpha_m"), ("turn", "turn_alpha.alpha_t")]
    for name, src in points:
        m.add_subsystem(name, WingAero(num_x=a.num_x, num_y=a.num_y, spacing=a.spacing, nf=a.nf_settings(),
                                       max_mode=a.max_mode, rho_ks=a.rho_ks), promotes_inputs=SHARED_AERO)
        m.connect(src, f"{name}.alpha")
        m.connect("CD_extra", f"{name}.perf.CD_extra")
    m.add_subsystem("polar", ThreePointPolar(), promotes_outputs=["CD0", "CD1", "k"])
    m.connect("cruise.vlm.wing_perf.CL", "polar.CL_0")
    m.connect("cruise.CD", "polar.CD_0")
    for i, (name, _) in enumerate(points, start=1):
        m.connect(f"{name}.vlm.wing_perf.CL", f"polar.CL_{i}")
        m.connect(f"{name}.CD", f"polar.CD_{i}")

    m.add_subsystem("lap", SmoothLapModel(rho_ks=a.rho_ks, power_mode="propulsion"),
                    promotes_inputs=["CD0", "CD1", "k", "rho", "g", "T_coeffs", "P_coeffs", ("V", "v"), ("S", "S"),
                                     ("m", "m_total"), ("E_avail", "E_usable")],
                    promotes_outputs=["*"])
    m.connect("cruise.CL_max", ["lap.CL_max", "lap.CL_to"])

    # stability: stations and grouped masses
    m.add_subsystem("stations", om.ExecComp(
        ["x_wing = fw * c", "x_tail = 0.25 * c + l_ht", "x_tb = 0.25 * c + boom_offset + 0.5 * l_tb",
         "x_cb = fcb * c", "m_tail = m_ht + m_vt", "m_motor_group = m_motor + m_esc + m_prop"],
        x_wing={"units": "m"}, x_tail={"units": "m"}, x_tb={"units": "m"}, x_cb={"units": "m"},
        c={"units": "m"}, l_ht={"units": "m"}, boom_offset={"units": "m"}, l_tb={"units": "m"},
        fw={"val": st.wing_cg_frac}, fcb={"val": st.x_cb_frac},
        m_tail={"units": "kg"}, m_ht={"units": "kg"}, m_vt={"units": "kg"}, m_motor_group={"units": "kg"},
        m_motor={"units": "kg"}, m_esc={"units": "kg"}, m_prop={"units": "kg"}),
        promotes_inputs=["c", "l_ht", "boom_offset", "l_tb", "m_ht", "m_vt", "m_motor", "m_esc", "m_prop"])
    m.add_subsystem("stability", StaticStability(),
                    promotes_inputs=["c", "S", "AR", "S_ht", "AR_ht", "l_ht", "eta_h", "m_wing", "m_tb", "m_cb",
                                     "m_pay", "m_batt", "m_avionics", "m_gear", "x_batt", "x_motor", "x_avionics",
                                     "x_gear"],
                    promotes_outputs=["x_np", "x_cg", "SM"])
    m.connect("cruise.CL_alpha", "stability.a_w")
    for s in ["x_wing", "x_tail", "x_tb", "m_tail", "m_motor_group"]:
        m.connect(f"stations.{s}", f"stability.{s}")
    m.connect("stations.x_cb", ["stability.x_cb", "stability.x_pay"])

    m.add_subsystem("limits", om.ExecComp(
        ["I_margin = I_max - I_static", "rpm_margin = frac * rpm_limit - rpm_max", "v_fit_margin = v_top - v"],
        I_margin={"units": "A"}, I_max={"units": "A"}, I_static={"units": "A"},
        rpm_margin={"units": "rpm"}, rpm_limit={"units": "rpm"}, rpm_max={"units": "rpm"},
        frac={"val": cfg.propulsion.rpm_fraction}, v_fit_margin={"units": "m/s"}, v_top={"units": "m/s"},
        v={"units": "m/s"}),
        promotes=["*"])
    m.add_subsystem("energy_con", om.ExecComp("energy_margin = E_usable - energy_3laps", energy_margin={"units": "J"},
                                              E_usable={"units": "J"}, energy_3laps={"units": "J"}), promotes=["*"])

    geo = cfg.geometry
    for name, val, units in [("b", cfg.initial.get("b", 1.2), "m"), ("c", cfg.initial.get("c", 0.3), "m"),
                             ("rho", 1.2, "kg/m**3"), ("x_spar", geo.x_spar, None), ("l_ht", geo.l_ht, "m"),
                             ("S_ht", geo.S_ht, "m**2"), ("S_vt", geo.S_vt, "m**2"), ("g", 9.80665, "m/s**2"),
                             ("AR_ht", st.AR_ht, None), ("r_tb", 0.008, "m"), ("l_cb", geo.l_cb, "m"),
                             ("w_cb", geo.w_cb, "m"), ("h_cb", geo.h_cb, "m"), ("boom_offset", geo.boom_offset, "m"),
                             ("m_motor", cfg.propulsion.m_motor, "kg"), ("m_esc", cfg.propulsion.m_esc, "kg"),
                             ("m_prop", cfg.propulsion.m_prop, "kg"), ("m_avionics", cfg.propulsion.m_avionics, "kg"),
                             ("m_pay", cfg.mass.inputs.get("m_pay", 1.0), "kg"), ("mu", 1.8e-5, "Pa*s")]:
        m.set_input_defaults(name, val, units=units)


def add_full_driver(prob, cfg: FullMDOConfig):
    m, st = prob.model, cfg.stability
    prob.driver = om.ScipyOptimizeDriver(optimizer="SLSQP", tol=cfg.tol, maxiter=cfg.maxiter, disp=True)
    units = {"b": "m", "c": "m", "l_ht": "m", "S_ht": "m**2", "S_vt": "m**2", "x_batt": "m", "capacity": "A*h"}
    for name, bd in cfg.design_variables.items():
        m.add_design_var(name, lower=bd.lower, upper=bd.upper, units=units.get(name))
    m.add_objective(cfg.lap_objective, ref=100.0)
    m.add_constraint("stall_margin", lower=0.0, ref=1.0)
    m.add_constraint("turn_speed_margin", lower=0.0, ref=1.0)
    m.add_constraint("thrust_margin_static", lower=0.0, ref=10.0)
    m.add_constraint("energy_margin", lower=0.0, ref=1e4)
    m.add_constraint("I_margin", lower=0.0, ref=10.0)
    m.add_constraint("rpm_margin", lower=0.0, ref=1000.0)
    m.add_constraint("v_fit_margin", lower=0.0, ref=10.0)
    m.add_constraint("SM", lower=st.SM_min, upper=st.SM_max)
    m.add_constraint("V_h", lower=st.V_h_min, upper=st.V_h_max)
    m.add_constraint("V_v", lower=st.V_v_min, upper=st.V_v_max)
    m.add_constraint("wall_ratio", upper=cfg.wall_ratio_max)
    m.add_constraint("r_tube", lower=cfg.r_tube_min, units="m", ref=0.01)
    m.add_constraint("cruise.cl_margin_sec", lower=0.0)


def apply_full_values(prob, cfg: FullMDOConfig):
    apply_aero_mass_values(prob, cfg, exclude={"m_sys"})  # m_sys comes from the powertrain
    pr, st = cfg.propulsion, cfg.stability
    for name in ["throttle", "n_cells", "V_cell", "R_cell", "usable_fraction", "e_spec", "Kv", "Rm", "I0",
                 "m_motor", "m_esc", "m_prop", "m_avionics"]:
        prob.set_val(name, getattr(pr, name))
    prob.set_val("I_max", pr.I_max, units="A")
    prob.set_val("v_top", pr.v_top, units="m/s")
    for name in ["AR_ht", "AR_vt", "tc_tail", "eta_h", "x_motor", "x_avionics", "x_gear"]:
        prob.set_val(name, getattr(st, name))
    for name in ["mu_roll", "takeoff_distance", "mission_time", "n_struct"]:
        prob.set_val(f"lap.{name}", getattr(cfg, name))
    for name, val in cfg.initial.items():
        prob.set_val(name, val)


def build_full_problem(cfg: FullMDOConfig, driver=True):
    prob = om.Problem(reports=False)
    build_full_model(prob.model, cfg)
    if driver:
        add_full_driver(prob, cfg)
    prob.setup()
    apply_full_values(prob, cfg)
    return prob


def load_full_config(path):
    from pathlib import Path

    from ..config.loader import _merge, load_yaml

    path = Path(path)
    data = load_yaml(path)
    base = load_yaml(path.parent / data.pop("mass_config")) if "mass_config" in data else {}
    data["mass"] = _merge(base, data.get("mass", {}))
    if "design_variables" not in data:
        data["design_variables"] = default_full_dvs()
    return FullMDOConfig(**data)
