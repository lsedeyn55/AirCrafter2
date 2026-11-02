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

DVs: b, c, tc_scale, camber_scale, battery capacity, tail arm, S_ht, S_vt, battery and cargo-bay stations, and the
motor Kv (default; leave "Kv" out of design_variables, e.g. configs/full_mdo_fixed_kv.yaml, for a fixed motor).
With Kv a DV the config's motor is a reference motor and a MotorRewind component scales Rm and I0 with Kv (the same
frame rewound; propulsion/motor_scaling.py), and the static current limit becomes a constraint.
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
from ..mission.course import CourseConfig
from ..mission.smooth_lap import SmoothLapModel
from ..propulsion.motor_scaling import MotorRewind
from ..propulsion.powertrain import Powertrain, motor_stations
from ..robustness.uncertainty import RobustConfig
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
    Kv: float = Field(900.0, description="rpm/V, PLACEHOLDER motor; the reference motor's Kv when Kv is a DV")
    Rm: float = Field(0.05, description="ohm, PLACEHOLDER motor; the reference motor's Rm when Kv is a DV")
    I0: float = Field(1.0, description="A, PLACEHOLDER motor; the reference motor's I0 when Kv is a DV")
    m_motor: float = Field(0.15, description="kg, PLACEHOLDER")
    m_esc: float = Field(0.06, description="kg, PLACEHOLDER")
    m_prop: float = Field(0.03, description="kg, PLACEHOLDER")
    m_avionics: float = Field(0.25, description="kg, PLACEHOLDER receiver, servos, wiring")
    I_max: float = Field(60.0, description="A, PLACEHOLDER motor/ESC current limit")
    rpm_fraction: float = Field(0.85, description="of APC's 150000/D limit (propSelector.m)")
    v_top: float = Field(40.0, description="m/s, top of the T(v) fit range")
    fit_range: Literal["fixed", "throttle"] = Field("fixed", description="T(v) fit nodes in [0, v_top], or scaled "
                                                    "below the windmill speed at this throttle (part throttle)")
    f_fit: float = Field(0.8, gt=0.0, le=1.0, description="fit_range = throttle: top of the T(v) fit, v_fit_max = f_fit "
                         "J_max n0 D (n0 the no-load rotor speed); keep the top node inside the APC data (C_T >= 0)")

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
    disabled_constraints: list[str] = Field([], description="full_constraints names left out of this run (the UI's "
                                            "Constraints tab); energy_margin is require_3lap_energy instead")
    mockup: MockupInputs = Field(MockupInputs(), description="3D mock-up placement only (aircrafter2.vsp); never "
                                 "enters the model")
    robust: RobustConfig = Field(RobustConfig(), description="robust design optimisation (off by default; "
                                 "aircrafter2.robustness.rdo_problem)")
    course: CourseConfig = Field(CourseConfig(), description="course layout, pilot reaction time and the take-off "
                                 "switch (mission/course.py; default: Table 3.5, take-off off = the original model)")
    mda_robust: bool = Field(False, description="the MDA continuation fallback (robustness.rdo_problem.RetryNewton "
                             "robust mode) and the warm path for --start (studies/results_2026.py), "
                             "docs/MDA_ROBUSTNESS_PLAN.md; off by default, so the 2024-25 and single-point results "
                             "are unchanged")

    @model_validator(mode="after")
    def _check_kv_robust(self):
        check_kv_robust(self)
        return self


KV_DV_ONLY = ("Kv_factor", "Rm_ref")  # uncertain parameters of the rewound motor (Kv a DV)
KV_FIXED_ONLY = ("Kv", "Rm")          # uncertain parameters of the fixed motor


def kv_is_dv(cfg):
    """True when the motor Kv is a design variable (then MotorRewind scales Rm and I0)."""
    return "Kv" in cfg.design_variables


def check_kv_robust(cfg):
    """Robust parameters must match the motor mode; raises ValueError (never remaps names)."""
    act = set(cfg.robust.active or [])
    if kv_is_dv(cfg) and act & set(KV_FIXED_ONLY):
        raise ValueError(f"Kv is a design variable, so robust.active cannot contain {sorted(act & set(KV_FIXED_ONLY))}: "
                         "use Kv_factor (winding tolerance, multiplies the design Kv) and Rm_ref (reference-motor "
                         "resistance) instead (docs/modules/robustness.md)")
    if not kv_is_dv(cfg) and act & set(KV_DV_ONLY):
        raise ValueError(f"Kv is fixed (not in design_variables), so robust.active cannot contain "
                         f"{sorted(act & set(KV_DV_ONLY))}: those act on the rewound motor; use Kv and Rm instead")


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
        "Kv": Bounds(lower=500.0, upper=1500.0),  # rpm/V, PLACEHOLDER bounds (OPEN_QUESTIONS Q34)
    }


def check_fixed_limits(prob):
    """Static current per motor/ESC, the optional pack current, and the RPM constraint value, after a run.

    With a fixed motor the static current is set by the powertrain choice alone, so this post-check is the only
    current check; with Kv a design variable I_margin (and I_pack_margin) are optimiser constraints as well."""
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
        self.options.declare("retry", default=False, types=bool,
                             desc="Newton with warm-start fallbacks (robust nodes, robustness.rdo_problem.RetryNewton)")
        self.options.declare("sizing", default="own", values=["own", "external"],
                             desc="own: the structure is sized at this point's mass and speed (closure m = m_total); "
                             "external: sized at m_sizing, v_sizing from another point (multipoint M3)")
        self.options.declare("stores", default=None, desc="(StoreConfig, dict of Stores flags) or None")

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
        stores = self.options["stores"]
        if stores is not None and is_2026(stores):  # 2025-26 payload, banner and payload fuselage (stores2026.py)
            from ..stores2026 import Stores2026

            scfg, flags = stores
            self.add_subsystem("stores", Stores2026(config=scfg, x_cb_frac0=float(cfg.stability.x_cb_frac), **flags),
                               promotes_inputs=["S", "c", "n_p", "n_c", "banner_drag_factor", "x_cb_frac"]
                               + (["L_banner"] if flags.get("banner") else []),
                               promotes_outputs=["m_pay", "x_pay", "CD_stores", "CD_banner", "dCD_stowed",
                                                 "m_banner"])
        elif stores is not None:  # external stores: mass, CG and drag (stores.py)
            from ..stores import Stores

            scfg, flags = stores
            self.add_subsystem("stores", Stores(config=scfg, **flags),
                               promotes_inputs=["v", "rho", "mu", "S", "c", "store_drag_factor"]
                               + (["fuel_mass"] if flags.get("fuel") else []),
                               promotes_outputs=["m_pay", "x_pay", "CD_stores", "CD_tanks", "CD_pylons", "CD_x1",
                                                 "fuel_capacity"])
        area = cfg.aero.cd_area_gear + cfg.aero.cd_area_lp
        if area:  # undercarriage + leakage drag areas (2025-26, [HB] Table 29): only when set, so 0 = the old graph
            self.add_subsystem("cd_areas", om.ExecComp("CD_areas = A / S", CD_areas={"val": 0.0},
                                                       A={"val": area, "units": "m**2"}, S={"units": "m**2"}),
                               promotes_inputs=["S"], promotes_outputs=["CD_areas"])
        terms = ["CD_extra"] + (["CD_stores"] if stores is not None else []) + (["CD_areas"] if area else [])
        if len(terms) > 1:
            self.add_subsystem("cd_extra_total", om.ExecComp("CD_extra_total = " + " + ".join(terms)),
                               promotes=["*"])
        cd_extra = cd_extra_name(cfg, stores)
        self.add_subsystem("cruise", WingAero(num_x=a.num_x, num_y=a.num_y, spacing=a.spacing, nf=a.nf_settings(),
                                              max_mode=a.max_mode, rho_ks=a.rho_ks),
                           promotes_inputs=SHARED_AERO + [("alpha", "alpha_cruise")])
        self.connect(cd_extra, "cruise.perf.CD_extra")
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
        if self.options["sizing"] == "own":
            self.add_subsystem("mass", GarciaGamboaMass(config=mass_opts),
                               promotes_inputs=["*", ("tc", "tc_max"), ("S_wing", "S"), ("V_design", "v")],
                               promotes_outputs=["*"])
        else:  # the airframe of another point: loads at m_sizing, v_sizing; this point's own weight for the balance
            self.add_subsystem("mass", GarciaGamboaMass(config=mass_opts, external_sizing=True),
                               promotes_inputs=["*", ("tc", "tc_max"), ("S_wing", "S"), ("V_design", "v_sizing"),
                                                ("m", "m_sizing"), ("W", "W_sizing"), ("Cm", "Cm_sizing"),
                                                ("CL_alpha", "CL_alpha_sizing")],
                               promotes_outputs=["*", ("W", "W_sizing")])
            self.add_subsystem("weight", om.ExecComp("W = m_total * g", W={"units": "N"}, m_total={"units": "kg"},
                                                     g={"units": "m/s**2", "val": 9.80665}),
                               promotes=["*"])
        if self.options["sizing"] == "own":  # external: the sizing point's Cm and CL_alpha (Cm_sizing, CL_alpha_sizing)
            self.connect("cruise.Cm_wing", "Cm")
            self.connect("cruise.CL_alpha", "CL_alpha")

        if self.options["retry"]:
            from ..robustness.rdo_problem import RetryNewton
            newton = RetryNewton(solve_subsystems=True, maxiter=cfg.mda_maxiter, atol=1e-10, rtol=1e-10,
                                 iprint=-1, err_on_non_converge=True, robust=cfg.mda_robust)
        else:
            newton = om.NewtonSolver(solve_subsystems=True, maxiter=cfg.mda_maxiter, atol=1e-10, rtol=1e-10,
                                     iprint=-1, err_on_non_converge=True)
        newton.linesearch = om.BoundsEnforceLS()
        self.nonlinear_solver = newton
        if self.options["retry"] and cfg.mda_robust:  # a singular Jacobian fails the attempt at once
            from ..robustness.rdo_problem import QuietDirectSolver
            self.linear_solver = QuietDirectSolver()
        else:
            self.linear_solver = om.DirectSolver()


def cd_extra_name(cfg, stores):
    """The drag increment the wing aero points add: CD_extra alone, or CD_extra_total when stores or the constant drag
    areas (aero.cd_area_gear, aero.cd_area_lp) add to it (FullMDA)."""
    area = cfg.aero.cd_area_gear + cfg.aero.cd_area_lp
    return "CD_extra_total" if stores is not None or area else "CD_extra"


def is_2026(stores):
    """True for the 2025-26 store set (stores2026.Store2026Config): payload, banner and the payload fuselage."""
    from ..stores2026 import Store2026Config

    return stores is not None and isinstance(stores[0], Store2026Config)


def build_full_model(m, cfg: FullMDOConfig, mda_retry=False, sizing="own", stores=None):
    """The coupled analysis of one mission point.

    sizing  "own" (default): the structure is sized at this point; "external": at the promoted inputs m_sizing and
            v_sizing (another point's m_total and v: problems/multipoint.py)
    stores  None (default), or (StoreConfig, Stores flags): external stores replace the payload input m_pay and
            its station (stores.py), and add their drag to CD_extra
    """
    a, st = cfg.aero, cfg.stability
    m.add_subsystem("airfoil", AirfoilGroup(seed=load_seed(cfg.airfoil.seed), n_modes=cfg.airfoil.n_modes),
                    promotes=["*"])
    if is_2026(stores):  # the payload fuselage, from the payload counts (design variables): before the planform
        from ..stores2026 import PayloadFuselage

        m.add_subsystem("payload_fuselage", PayloadFuselage(config=stores[0]), promotes=["*"])
    m.add_subsystem("planform", RectPlanform(), promotes=["*"])
    m.add_subsystem("tail", TailGeometry(), promotes=["*"])
    m.add_subsystem("cargo", CargoBay(), promotes=["*"])
    pr = cfg.propulsion
    if kv_is_dv(cfg):  # the reference motor rewound to the design Kv: Rm, I0 follow Kv
        m.add_subsystem("motor", MotorRewind(), promotes_inputs=["Kv", "Kv_factor", "Kv_ref", "Rm_ref", "I0_ref"],
                        promotes_outputs=["Kv_eff", "Rm", "I0"])
    m.add_subsystem("propulsion", Powertrain(prop=pr.prop, v_top=pr.v_top, n_motors=pr.n_motors,
                                             battery_topology=pr.battery_topology, apc_dir=pr.apc_dir,
                                             fit_range=pr.fit_range, f_fit=pr.f_fit),
                    promotes_inputs=["rho", "capacity", "throttle", "n_cells", "V_cell", "R_cell", "usable_fraction",
                                     "e_spec", ("Kv", "Kv_eff") if kv_is_dv(cfg) else "Kv", "Rm", "I0", "m_motor",
                                     "m_esc", "m_prop", "m_nacelle", "m_avionics"],
                    promotes_outputs=["T_coeffs", "P_coeffs", "RPM_coeffs", "T_static", "T_static_unit", "I_static",
                                      "I_pack_static", "rpm_max", "rpm_limit", "D_prop", "E_nom_total", "E_usable",
                                      "m_batt", "m_motor_unit", "m_motor_group", "m_sys", "v_fit_max"])
    if pr.n_motors == 2:  # wing station and prop clearances (D8)
        m.add_subsystem("motor_placement", MotorPlacement(station_mode=pr.station_mode), promotes=["*"])
    m.add_subsystem("mda", FullMDA(cfg=cfg, retry=mda_retry, sizing=sizing, stores=stores), promotes=["*"])
    cd_extra = cd_extra_name(cfg, stores)

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
        m.connect(cd_extra, f"{name}.perf.CD_extra")
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
    co = cfg.course
    lap1, lap_other = co.lap_options()
    banner = stores is not None and stores[1].get("banner", False)
    m.add_subsystem("lap", SmoothLapModel(rho_ks=a.rho_ks, power_mode="propulsion", lap1=lap1, lap_other=lap_other,
                                          pilot_reaction_time=co.pilot_reaction_time_s,
                                          takeoff=co.takeoff.in_mission_time,
                                          start_offset_ft=co.takeoff.start_offset_ft,
                                          lap1_cluster=co.lap1_cluster, turn_drift=co.turn_drift,
                                          banner_stowed_legs=co.banner_stowed_legs() if banner else ()),
                    promotes_inputs=["CD0", "CD1", "k", "rho", "g", "T_coeffs", "P_coeffs", ("V", "v"), ("S", "S"),
                                     ("m", "m_total"), ("E_avail", "E_mission")],
                    promotes_outputs=["*"])
    m.connect("cruise.CL_max", ["lap.CL_max", "lap.CL_to"])
    if banner:  # lap 1's legs before deployment fly the stowed banner instead of the towed one
        m.connect("dCD_stowed", "lap.dCD_stowed")

    # stability: stations and grouped masses
    m.add_subsystem("stations", om.ExecComp(
        ["x_wing = fw * c", "x_tail = 0.25 * c + l_ht", "x_tb = 0.25 * c + boom_offset + 0.5 * l_tb",
         "x_cb = fcb * c", "m_tail = m_ht + m_vt"],
        x_wing={"units": "m"}, x_tail={"units": "m"}, x_tb={"units": "m"}, x_cb={"units": "m"},
        c={"units": "m"}, l_ht={"units": "m"}, boom_offset={"units": "m"}, l_tb={"units": "m"},
        fw={"val": st.wing_cg_frac}, fcb={"val": st.x_cb_frac},
        m_tail={"units": "kg"}, m_ht={"units": "kg"}, m_vt={"units": "kg"}),
        promotes_inputs=["c", "l_ht", "boom_offset", "l_tb", "m_ht", "m_vt", ("fcb", "x_cb_frac")])
    if is_2026(stores):  # the rigid payload fuselage: motor station and packaging margins (stores2026.py, D2)
        from ..stores2026 import Packaging2026

        m.add_subsystem("packaging", Packaging2026(config=stores[0], x_motor_ref=float(st.x_motor),
                                                   x_cb_frac0=float(st.x_cb_frac)),
                        promotes_inputs=["c", "x_cb_frac", "l_cb", "x_batt", "l_ht", "S_ht", "AR_ht"],
                        promotes_outputs=["x_motor", "batt_fwd_margin", "batt_aft_margin", "tail_margin", "x_cb_aft"])
    m.add_subsystem("stability", StaticStability(),
                    promotes_inputs=["c", "S", "AR", "S_ht", "AR_ht", "l_ht", "eta_h", "m_wing", "m_tb", "m_cb",
                                     "m_pay", "m_batt", "m_avionics", "m_gear", "x_batt", "x_motor", "x_avionics",
                                     "x_gear", "m_motor_group"],
                    promotes_outputs=["x_np", "x_cg", "SM"])
    m.connect("cruise.CL_alpha", "stability.a_w")
    for s in ["x_wing", "x_tail", "x_tb", "m_tail"]:
        m.connect(f"stations.{s}", f"stability.{s}")
    if stores is not None:  # payload = the stores, at their own CG station
        m.connect("stations.x_cb", "stability.x_cb")
        m.connect("x_pay", "stability.x_pay")
    else:
        m.connect("stations.x_cb", ["stability.x_cb", "stability.x_pay"])

    # rotor speed rises with flight speed, so the RPM limit is checked at the top (full-throttle level) speed
    m.add_subsystem("rpm_at_v", PolyAt(), promotes_inputs=[("coeffs", "RPM_coeffs"), "v"])
    top = "v_fit_max" if pr.fit_range == "throttle" else "v_top"  # part throttle: the fit range moves with throttle
    m.add_subsystem("limits", om.ExecComp(
        ["I_margin = I_max - I_static", "rpm_margin = frac * rpm_limit - rpm_v", f"v_fit_margin = {top} - v"],
        I_margin={"units": "A"}, I_max={"units": "A"}, I_static={"units": "A"},
        rpm_margin={"units": "rpm"}, rpm_limit={"units": "rpm"}, rpm_v={"units": "rpm"},
        frac={"val": cfg.propulsion.rpm_fraction}, v_fit_margin={"units": "m/s"}, **{top: {"units": "m/s"}},
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
        if name == "m_pay" and stores is not None:
            continue  # computed by the stores
        if name in ("l_cb", "w_cb", "h_cb") and is_2026(stores):
            continue  # the payload fuselage, sized by the payload counts (stores2026.py)
        if name == "x_motor" and is_2026(stores):
            continue  # computed: the motor group moves with the payload fuselage (Packaging2026)
        m.set_input_defaults(name, val, units=units)
    if pr.n_motors == 2:
        if pr.station_mode == "absolute":
            m.set_input_defaults("y_m", pr.y_motor_m, units="m")
        else:
            m.set_input_defaults("y_motor", pr.y_motor)
        for name, val, units in [("c_clear", pr.c_clear, "m"), ("l_nac", pr.l_nac, "m"), ("d_nac", pr.d_nac, "m"),
                                 ("Q_nac", pr.Q_nac, None)]:
            m.set_input_defaults(name, val, units=units)


DV_UNITS = {"b": "m", "c": "m", "l_ht": "m", "S_ht": "m**2", "S_vt": "m**2", "x_batt": "m", "capacity": "A*h",
            "y_m": "m", "Kv": "rpm/V"}
DV_REF = {"Kv": 1000.0}  # optimiser scaling (O(1) for SLSQP); DVs not listed keep ref = None


def full_constraints(cfg: FullMDOConfig):
    """The full problem's constraints, in order: [(promoted output, add_constraint kwargs)]."""
    st = cfg.stability
    cons = [("stall_margin", dict(lower=0.0, ref=1.0)),
            ("turn_speed_margin", dict(lower=0.0, ref=1.0)),
            ("thrust_margin_static", dict(lower=0.0, ref=10.0))]
    if cfg.require_3lap_energy:
        cons.append(("energy_margin", dict(lower=0.0, ref=1e4)))
    # rpm at the top speed depends on the design (through v): a real constraint.
    cons.append(("rpm_margin", dict(lower=0.0, ref=1000.0)))
    # The static current depends only on the powertrain and the motor. With a fixed motor (prop, cells, throttle
    # and Kv fixed) no DV moves it, so it is only checked after the optimisation (check_fixed_limits). With Kv a
    # DV it moves with Kv, so it is a constraint (and the pack current, when a pack limit is set).
    if kv_is_dv(cfg):
        cons.append(("I_margin", dict(lower=0.0, ref=10.0)))
        if cfg.propulsion.pack_I_max is not None:
            cons.append(("I_pack_margin", dict(lower=0.0, ref=10.0)))
    cons += [("v_fit_margin", dict(lower=0.0, ref=10.0)),
             ("SM", dict(lower=st.SM_min, upper=st.SM_max)),
             ("V_h", dict(lower=st.V_h_min, upper=st.V_h_max)),
             ("V_v", dict(lower=st.V_v_min, upper=st.V_v_max)),
             ("wall_ratio", dict(upper=cfg.wall_ratio_max)),
             ("r_tube", dict(lower=cfg.r_tube_min, units="m", ref=0.01)),
             ("cruise.cl_margin_sec", dict(lower=0.0))]
    if cfg.propulsion.n_motors == 2:  # prop-to-prop clearance is implied by the fuselage one: reported only
        cons += [("fus_clear_margin", dict(lower=0.0, ref=0.01)),
                 ("tip_clear_margin", dict(lower=0.0, ref=0.01))]
    return drop_disabled(cfg, cons)


def drop_disabled(cfg, cons):
    """cons without cfg.disabled_constraints; an unknown name (or energy_margin: use require_3lap_energy) raises."""
    off = list(cfg.disabled_constraints)
    if not off:
        return cons
    if "energy_margin" in off:
        raise ValueError("disabled_constraints cannot contain energy_margin: set require_3lap_energy: false instead")
    names = [n for n, _ in cons]
    unknown = sorted(set(off) - set(names))
    if unknown:
        raise ValueError(f"disabled_constraints {unknown} are not constraints of this problem ({names})")
    return [(n, kw) for n, kw in cons if n not in off]


def add_full_driver(prob, cfg: FullMDOConfig):
    m = prob.model
    prob.driver = om.ScipyOptimizeDriver(optimizer="SLSQP", tol=cfg.tol, maxiter=cfg.maxiter, disp=True)
    for name, bd in cfg.design_variables.items():
        m.add_design_var(name, lower=bd.lower, upper=bd.upper, units=DV_UNITS.get(name), ref=DV_REF.get(name))
    add_scoring(m, get_plugin(cfg.scoring.plugin, **cfg.scoring.options))
    for name, kw in full_constraints(cfg):
        m.add_constraint(name, **kw)


def apply_full_values(prob, cfg: FullMDOConfig, stores=False, skip=()):
    """Config values onto a set-up full model. stores: m_pay is computed (stores.py); skip: names not to set
    (e.g. inputs connected from another mission point)."""
    apply_aero_mass_values(prob, cfg, exclude={"m_sys"} | ({"m_pay"} if stores else set()) | set(skip))
    pr, st = cfg.propulsion, cfg.stability
    motor = ["Kv", "Rm", "I0"]
    if kv_is_dv(cfg):  # reference motor; the design Kv starts at Kv_ref (cfg.initial["Kv"] overrides below)
        motor = []
        for name in ["Kv", "Rm", "I0"]:
            prob.set_val(f"{name}_ref", getattr(pr, name))
        prob.set_val("Kv", pr.Kv)
    for name in ["throttle", "n_cells", "V_cell", "R_cell", "usable_fraction", "e_spec", *motor,
                 "m_motor", "m_esc", "m_prop", "m_nacelle", "m_avionics"]:
        prob.set_val(name, getattr(pr, name))
    prob.set_val("I_max", pr.I_max, units="A")
    if pr.pack_I_max is not None:
        prob.set_val("pack_I_max", pr.pack_I_max, units="A")
    if pr.fit_range == "fixed":
        prob.set_val("v_top", pr.v_top, units="m/s")
    for name in ["AR_ht", "AR_vt", "tc_tail", "eta_h", "x_motor", "x_avionics", "x_gear"]:
        prob.set_val(name, getattr(st, name))
    for name in ["mu_roll", "takeoff_distance", "mission_time", "n_struct"]:
        prob.set_val(f"lap.{name}", getattr(cfg, name))
    if cfg.course.takeoff.in_mission_time:
        for name in ["k_lof", "CL_ground"]:
            prob.set_val(f"lap.{name}", getattr(cfg.course.takeoff, name))
    prob.set_val("x_cb_frac", st.x_cb_frac)
    for name, val in cfg.initial.items():
        if name not in skip:
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


def load_full_config(path, placeholders=None, ruleset=None):
    """A FullMDOConfig YAML over its layers (placeholders.yaml, the ruleset; config/loader.py load_layered).
    placeholders/ruleset: layer data to use instead of the files (the UI's unsaved edits)."""
    from pathlib import Path

    from ..config.loader import load_layered

    path = Path(path)
    return full_config_from_dict(load_layered(path, FullMDOConfig.model_fields, placeholders=placeholders,
                                              ruleset=ruleset), path)


def full_config_from_dict(data, path):
    """FullMDOConfig from loaded YAML data (extends: resolved); relative file references resolve against path."""
    from pathlib import Path

    from ..config.loader import _merge, load_yaml

    path = Path(path)
    data = dict(data)
    pool = {p["name"]: p for p in data.pop("uncertainty_pool", None) or []}  # placeholders.yaml + ruleset spreads
    base = load_yaml(path.parent / data.pop("mass_config")) if "mass_config" in data else {}
    data["mass"] = _merge(base, data.get("mass", {}))
    if "design_variables" not in data:
        data["design_variables"] = default_full_dvs()
    fixed = data.pop("fixed_design_variables", None) or []  # e.g. [Kv]: drop DVs inherited through extends:
    dvs = dict(data["design_variables"])
    unknown = [n for n in fixed if n not in dvs]
    if unknown:
        raise ValueError(f"fixed_design_variables {unknown} are not design variables of {path.name}")
    data["design_variables"] = {k: b for k, b in dvs.items() if k not in fixed}
    for n in fixed:
        (data.get("initial") or {}).pop(n, None)
    rob = data.get("robust") or {}
    if rob.get("uncertainty_config") or (rob.get("enabled") and pool):
        # the pool: the layers' shared spreads, then the uncertainty YAML's entries, then 'parameters', merged by name
        # (an entry replaces the one of the same name in place, or is appended). A 'standalone: true' YAML is the
        # whole pool by itself (configs/uncertainty.yaml: the published 15-parameter set, without the shared spreads).
        shared = dict(pool)
        if rob.get("uncertainty_config"):
            unc = load_yaml(path.parent / rob["uncertainty_config"])
            if unc.get("standalone"):
                shared = {}
            shared.update({p["name"]: p for p in unc.get("parameters") or []})
        shared.update({p["name"]: p for p in rob.get("parameters", [])})
        rob["parameters"] = list(shared.values())
    return FullMDOConfig(**data)
