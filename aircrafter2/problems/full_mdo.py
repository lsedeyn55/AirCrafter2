"""The full AirCrafter 2.0 MDF problem (decision B: propulsion sets the power).

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

DVs: b, c, tc_scale, camber_scale, battery capacity, tail arm, S_ht, S_vt, battery and cargo-bay stations.
Objective: a scoring plug-in (aircrafter2.scoring; placeholder default three_lap_time).
"""

from typing import Literal, Optional

import numpy as np
import openmdao.api as om
from pydantic import BaseModel, ConfigDict, Field, model_validator

from ..aero import WingAero
from ..aero.parasite import ParasiteDrag
from ..aero.polar import ThreePointPolar, TurnAlpha
from ..mass.garcia_gamboa import GarciaGamboaMass
from ..mission.smooth_lap import SmoothLapModel
from ..propulsion.powertrain import Powertrain, motor_stations
from ..scoring import add_scoring, get_plugin
from ..stability import StaticStability
from .aero_mass import (SHARED_AERO, AeroMassProblemConfig, apply_aero_mass_values)
from .wing_aero import Bounds
from ..airfoil import AirfoilGroup, load_seed
from ..geometry import CargoBay, MotorPlacement, RectPlanform, TailGeometry
from ..geometry.mockup import MockupInputs


# Woombah (USYD 2025 DBF design report, "Project Woombah"): nacelles at 9.9 in along the semi-span
# (Sec. 5.3.2, "Nacelles"); span 70.9 in (Table 30), so y_motor = 9.9 / 35.45.
WOOMBAH_Y_M = 9.9 * 0.0254
WOOMBAH_Y_MOTOR = 9.9 / (70.9 / 2.0)


class PropulsionInputs(BaseModel):
    model_config = ConfigDict(extra="forbid")
    n_motors: Literal[1, 2] = Field(1, description="identical propulsion units: 1 = centreline nose tractor, "
                                    "2 = symmetric wing-mounted tractor pair (discrete, D4; not a DV)")
    battery_topology: Literal["per_motor", "shared"] = Field(
        "per_motor", description="per_motor: one pack per unit (2025 rule C-F2, Woombah); shared: one pack feeds "
        "every ESC. 'capacity' is the capacity of ONE pack in both")
    y_motor_m: Optional[float] = Field(None, description="m, wing-motor station from the centreline (n_motors = 2); "
                                       f"Woombah 9.9 in = {WOOMBAH_Y_M:.4f} m. Give this OR y_motor")
    y_motor: Optional[float] = Field(None, description="wing-motor station as a fraction of the semi-span "
                                     f"(n_motors = 2; Woombah {WOOMBAH_Y_MOTOR:.4f}). Give this OR y_motor_m")
    c_clear: float = Field(0.025, description="m, prop-tip clearance to the fuselage and between discs, PLACEHOLDER "
                           "(Woombah as built: ~2.0 in with 11 in props)")
    l_nac: float = Field(0.15, description="m, nacelle length, PLACEHOLDER (not in the Woombah report)")
    d_nac: float = Field(0.05, description="m, nacelle diameter, PLACEHOLDER (not in the Woombah report)")
    Q_nac: float = Field(1.3, description="nacelle interference factor; Woombah Sec. 4.3.2: Q ~ 1.3 with motors "
                         "5.5 in ahead of the LE (Raymer)")
    nacelle_ff: Literal["body", "nacelle"] = Field("body", description="form factor: Raymer body (as briefed) or "
                                                   "Raymer nacelle/store 1 + 0.35/f (PLAN 6h T6)")
    m_nacelle: float = Field(0.0, description="kg per unit; 0 for the nose layout (in G&G's fuselage). "
                             "PLACEHOLDER for twin (OPEN_QUESTIONS Q25)")
    pack_I_max: Optional[float] = Field(None, description="A, pack current limit (shared topology), PLACEHOLDER; "
                                        "None = off")
    prop: str = Field("11x7E", description="APC PER3 name (discrete, D4)")
    apc_dir: Optional[str] = Field(None, description="folder with the APC PER3_*.dat files; None = the "
                                   "AIRCRAFTER2_APC_DIR environment variable, else THESIS/PERFILES2")
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

    @model_validator(mode="after")
    def _check_layout(self):
        if self.n_motors == 2:
            if (self.y_motor is None) == (self.y_motor_m is None):
                raise ValueError("n_motors = 2 needs exactly one of y_motor_m (m, absolute station) or "
                                 "y_motor (fraction of the semi-span)")
            if self.y_motor is not None:
                motor_stations(2, self.y_motor)  # checks 0 < y_motor < 1
            elif self.y_motor_m <= 0:
                raise ValueError(f"y_motor_m must be > 0, got {self.y_motor_m}")
        return self

    @property
    def station_mode(self):
        return "fraction" if self.y_motor is not None else "absolute"


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
    eta_v: float = Field(0.9, description="fin dynamic-pressure ratio, PLACEHOLDER (OEI post-check only)")
    rudder_tau: float = Field(0.5, description="rudder effectiveness d(alpha_v)/d(delta_r), PLACEHOLDER (OEI only)")
    rudder_max_deg: float = Field(25.0, description="deg, maximum rudder deflection, PLACEHOLDER (OEI only)")
    oei_speed_factor: float = Field(1.2, description="OEI check speed / V_stall, PLACEHOLDER (OEI only)")
    SM_min: float = Field(0.05, description="PLACEHOLDER (OPEN_QUESTIONS Q9)")
    SM_max: float = Field(0.20, description="PLACEHOLDER")
    V_h_min: float = Field(0.35, description="PLACEHOLDER, around Raymer Table 6.4 values")
    V_h_max: float = Field(0.65, description="PLACEHOLDER")
    V_v_min: float = Field(0.02, description="PLACEHOLDER")
    V_v_max: float = Field(0.06, description="PLACEHOLDER")


class ScoringConfig(BaseModel):
    model_config = ConfigDict(extra="forbid")
    plugin: str = Field("three_lap_time", description="registered name or 'module:Class' (aircrafter2.scoring)")
    options: dict = {}


class FullMDOConfig(AeroMassProblemConfig):
    propulsion: PropulsionInputs = PropulsionInputs()
    stability: StabilityInputs = StabilityInputs()
    fuselage_drag: bool = True
    polar_turn_frac: float = 0.95
    mu_roll: float = 0.02
    takeoff_distance: float = 100.0
    mission_time: float = 300.0
    n_struct: float = 5.0
    scoring: ScoringConfig = ScoringConfig()
    energy_reserve: float = Field(0.2, description="fraction of usable battery energy kept in reserve "
                                  "(PLACEHOLDER, OPEN_QUESTIONS Q24)")
    require_3lap_energy: bool = Field(True, description="constrain the reserve-adjusted energy to cover 3 laps")
    mockup: MockupInputs = Field(MockupInputs(), description="3D mock-up placement only (aircrafter2.vsp); never "
                                 "enters the model")


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
        "x_cb_frac": Bounds(lower=0.0, upper=0.6),
    }


def check_fixed_limits(prob):
    """Static current per motor/ESC (fixed by the powertrain choice), the optional pack current, and the RPM
    constraint value, after optimising."""
    out = {"I_margin_A": float(prob.get_val("I_margin")[0]), "rpm_margin": float(prob.get_val("rpm_margin")[0])}
    ok = out["I_margin_A"] >= 0 and out["rpm_margin"] >= -1.0
    try:
        out["I_pack_margin_A"] = float(prob.get_val("I_pack_margin")[0])
        ok = ok and out["I_pack_margin_A"] >= 0
    except KeyError:  # no pack limit configured
        pass
    out["ok"] = bool(ok)
    return out


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
        twin = cfg.propulsion.n_motors == 2
        self.add_subsystem("parasite", ParasiteDrag(fuselage_drag=cfg.fuselage_drag, n_nacelles=2 if twin else 0,
                                                    nacelle_ff=cfg.propulsion.nacelle_ff),
                           promotes_inputs=["v", "rho", "mu", "S", "S_ht", "S_vt", "AR_ht", "AR_vt", "tc_tail",
                                            "l_cb", "w_cb", "h_cb", "l_tb", "r_tb"]
                           + (["l_nac", "d_nac", "Q_nac"] if twin else []),
                           promotes_outputs=["CD_extra", "CD_tail", "CD_fus", "CD_nac"])
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

        mass_opts = cfg.mass.options.model_copy(update={"standalone_solver": True,
                                                        "n_motor_pairs": 1 if twin else 0})
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
    pr = cfg.propulsion
    m.add_subsystem("propulsion", Powertrain(prop=pr.prop, v_top=pr.v_top, n_motors=pr.n_motors,
                                             battery_topology=pr.battery_topology, apc_dir=pr.apc_dir),
                    promotes_inputs=["rho", "capacity", "throttle", "n_cells", "V_cell", "R_cell", "usable_fraction",
                                     "e_spec", "Kv", "Rm", "I0", "m_motor", "m_esc", "m_prop", "m_nacelle",
                                     "m_avionics"],
                    promotes_outputs=["T_coeffs", "P_coeffs", "RPM_coeffs", "T_static", "T_static_unit", "I_static",
                                      "I_pack_static", "rpm_max", "rpm_limit", "D_prop", "E_nom_total", "E_usable",
                                      "m_batt", "m_motor_unit", "m_motor_group", "m_sys"])
    if pr.n_motors == 2:  # wing station and prop clearances (D8)
        m.add_subsystem("motor_placement", MotorPlacement(station_mode=pr.station_mode), promotes=["*"])
    m.add_subsystem("mda", FullMDA(cfg=cfg), promotes=["*"])

    # polar from three coupled aero points (as in lap_mdo)
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

    m.add_subsystem("mission_energy", om.ExecComp("E_mission = (1.0 - reserve) * E_usable",
                                                  E_mission={"units": "J"}, E_usable={"units": "J"},
                                                  reserve={"val": cfg.energy_reserve}),
                    promotes=["E_mission", "E_usable"])
    m.add_subsystem("lap", SmoothLapModel(rho_ks=a.rho_ks, power_mode="propulsion"),
                    promotes_inputs=["CD0", "CD1", "k", "rho", "g", "T_coeffs", "P_coeffs", ("V", "v"), ("S", "S"),
                                     ("m", "m_total"), ("E_avail", "E_mission")],
                    promotes_outputs=["*"])
    m.connect("cruise.CL_max", ["lap.CL_max", "lap.CL_to"])

    # stability: stations and grouped masses
    m.add_subsystem("stations", om.ExecComp(
        ["x_wing = fw * c", "x_tail = 0.25 * c + l_ht", "x_tb = 0.25 * c + boom_offset + 0.5 * l_tb",
         "x_cb = fcb * c", "m_tail = m_ht + m_vt"],
        x_wing={"units": "m"}, x_tail={"units": "m"}, x_tb={"units": "m"}, x_cb={"units": "m"},
        c={"units": "m"}, l_ht={"units": "m"}, boom_offset={"units": "m"}, l_tb={"units": "m"},
        fw={"val": st.wing_cg_frac}, fcb={"val": st.x_cb_frac},
        m_tail={"units": "kg"}, m_ht={"units": "kg"}, m_vt={"units": "kg"}),
        promotes_inputs=["c", "l_ht", "boom_offset", "l_tb", "m_ht", "m_vt", ("fcb", "x_cb_frac")])
    m.add_subsystem("stability", StaticStability(),
                    promotes_inputs=["c", "S", "AR", "S_ht", "AR_ht", "l_ht", "eta_h", "m_wing", "m_tb", "m_cb",
                                     "m_pay", "m_batt", "m_avionics", "m_gear", "x_batt", "x_motor", "x_avionics",
                                     "x_gear", "m_motor_group"],
                    promotes_outputs=["x_np", "x_cg", "SM"])
    m.connect("cruise.CL_alpha", "stability.a_w")
    for s in ["x_wing", "x_tail", "x_tb", "m_tail"]:
        m.connect(f"stations.{s}", f"stability.{s}")
    m.connect("stations.x_cb", ["stability.x_cb", "stability.x_pay"])

    # rotor speed rises with flight speed, so the RPM limit is checked at the top (full-throttle level) speed
    m.add_subsystem("rpm_at_v", PolyAt(), promotes_inputs=[("coeffs", "RPM_coeffs"), "v"])
    m.add_subsystem("limits", om.ExecComp(
        ["I_margin = I_max - I_static", "rpm_margin = frac * rpm_limit - rpm_v", "v_fit_margin = v_top - v"],
        I_margin={"units": "A"}, I_max={"units": "A"}, I_static={"units": "A"},
        rpm_margin={"units": "rpm"}, rpm_limit={"units": "rpm"}, rpm_v={"units": "rpm"},
        frac={"val": cfg.propulsion.rpm_fraction}, v_fit_margin={"units": "m/s"}, v_top={"units": "m/s"},
        v={"units": "m/s"}),
        promotes=["*"])
    m.connect("rpm_at_v.y", "rpm_v")
    if pr.pack_I_max is not None:  # shared pack: the pack current can bind before the per-ESC limit
        m.add_subsystem("pack_limit", om.ExecComp("I_pack_margin = pack_I_max - I_pack_static",
                                                  I_pack_margin={"units": "A"}, pack_I_max={"units": "A"},
                                                  I_pack_static={"units": "A"}), promotes=["*"])
    m.add_subsystem("energy_con", om.ExecComp("energy_margin = E_mission - energy_3laps", energy_margin={"units": "J"},
                                              E_mission={"units": "J"}, energy_3laps={"units": "J"}), promotes=["*"])

    geo = cfg.geometry
    for name, val, units in [("b", cfg.initial.get("b", 1.2), "m"), ("c", cfg.initial.get("c", 0.3), "m"),
                             ("rho", 1.2, "kg/m**3"), ("x_spar", geo.x_spar, None), ("l_ht", geo.l_ht, "m"),
                             ("S_ht", geo.S_ht, "m**2"), ("S_vt", geo.S_vt, "m**2"), ("g", 9.80665, "m/s**2"),
                             ("AR_ht", st.AR_ht, None), ("r_tb", 0.008, "m"), ("l_cb", geo.l_cb, "m"),
                             ("w_cb", geo.w_cb, "m"), ("h_cb", geo.h_cb, "m"), ("boom_offset", geo.boom_offset, "m"),
                             ("m_motor", cfg.propulsion.m_motor, "kg"), ("m_esc", cfg.propulsion.m_esc, "kg"),
                             ("m_prop", cfg.propulsion.m_prop, "kg"), ("m_avionics", cfg.propulsion.m_avionics, "kg"),
                             ("m_nacelle", cfg.propulsion.m_nacelle, "kg"), ("x_motor", st.x_motor, "m"),
                             ("m_pay", cfg.mass.inputs.get("m_pay", 1.0), "kg"), ("mu", 1.8e-5, "Pa*s")]:
        m.set_input_defaults(name, val, units=units)
    if pr.n_motors == 2:
        if pr.station_mode == "absolute":
            m.set_input_defaults("y_m", pr.y_motor_m, units="m")
        else:
            m.set_input_defaults("y_motor", pr.y_motor)
        for name, val, units in [("c_clear", pr.c_clear, "m"), ("l_nac", pr.l_nac, "m"), ("d_nac", pr.d_nac, "m"),
                                 ("Q_nac", pr.Q_nac, None)]:
            m.set_input_defaults(name, val, units=units)


def add_full_driver(prob, cfg: FullMDOConfig):
    m, st = prob.model, cfg.stability
    prob.driver = om.ScipyOptimizeDriver(optimizer="SLSQP", tol=cfg.tol, maxiter=cfg.maxiter, disp=True)
    units = {"b": "m", "c": "m", "l_ht": "m", "S_ht": "m**2", "S_vt": "m**2", "x_batt": "m", "capacity": "A*h",
             "y_m": "m"}
    for name, bd in cfg.design_variables.items():
        m.add_design_var(name, lower=bd.lower, upper=bd.upper, units=units.get(name))
    add_scoring(m, get_plugin(cfg.scoring.plugin, **cfg.scoring.options))
    m.add_constraint("stall_margin", lower=0.0, ref=1.0)
    m.add_constraint("turn_speed_margin", lower=0.0, ref=1.0)
    m.add_constraint("thrust_margin_static", lower=0.0, ref=10.0)
    if cfg.require_3lap_energy:
        m.add_constraint("energy_margin", lower=0.0, ref=1e4)
    # rpm at the top speed depends on the design (through v): a real constraint.
    m.add_constraint("rpm_margin", lower=0.0, ref=1000.0)
    # The static current depends only on the powertrain (fixed prop, cells, throttle), not on the DVs,
    # so it is checked after the optimisation (check_fixed_limits), not passed to SLSQP.
    m.add_constraint("v_fit_margin", lower=0.0, ref=10.0)
    m.add_constraint("SM", lower=st.SM_min, upper=st.SM_max)
    m.add_constraint("V_h", lower=st.V_h_min, upper=st.V_h_max)
    m.add_constraint("V_v", lower=st.V_v_min, upper=st.V_v_max)
    m.add_constraint("wall_ratio", upper=cfg.wall_ratio_max)
    m.add_constraint("r_tube", lower=cfg.r_tube_min, units="m", ref=0.01)
    m.add_constraint("cruise.cl_margin_sec", lower=0.0)
    if cfg.propulsion.n_motors == 2:  # prop-to-prop clearance is implied by the fuselage one: reported only
        m.add_constraint("fus_clear_margin", lower=0.0, ref=0.01)
        m.add_constraint("tip_clear_margin", lower=0.0, ref=0.01)


def apply_full_values(prob, cfg: FullMDOConfig):
    apply_aero_mass_values(prob, cfg, exclude={"m_sys"})  # m_sys comes from the powertrain
    pr, st = cfg.propulsion, cfg.stability
    for name in ["throttle", "n_cells", "V_cell", "R_cell", "usable_fraction", "e_spec", "Kv", "Rm", "I0",
                 "m_motor", "m_esc", "m_prop", "m_nacelle", "m_avionics"]:
        prob.set_val(name, getattr(pr, name))
    prob.set_val("I_max", pr.I_max, units="A")
    if pr.pack_I_max is not None:
        prob.set_val("pack_I_max", pr.pack_I_max, units="A")
    prob.set_val("v_top", pr.v_top, units="m/s")
    for name in ["AR_ht", "AR_vt", "tc_tail", "eta_h", "x_motor", "x_avionics", "x_gear"]:
        prob.set_val(name, getattr(st, name))
    for name in ["mu_roll", "takeoff_distance", "mission_time", "n_struct"]:
        prob.set_val(f"lap.{name}", getattr(cfg, name))
    prob.set_val("x_cb_frac", st.x_cb_frac)
    for name, val in cfg.initial.items():
        prob.set_val(name, val)


def build_full_problem(cfg: FullMDOConfig, driver=True):
    prob = om.Problem(reports=False)
    build_full_model(prob.model, cfg)
    if driver:
        add_full_driver(prob, cfg)
    else:  # the score is still computed (reports, robustness), just not optimised
        add_scoring(prob.model, get_plugin(cfg.scoring.plugin, **cfg.scoring.options), objective=False)
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
