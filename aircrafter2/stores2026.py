"""Payload, banner and payload-sized fuselage of the 2025-26 ruleset (Banner Towing Bush Plane).

Gives the mass, CG station, drag and cargo box of one mission point. PayloadFuselage runs ahead of the planform,
Stores2026 inside the MDA, and Packaging2026 after it (problems/full_mdo.py). Their outputs feed the mass, stability and
lap models. Sources: [R26] (2026-DBF-Rules.pdf), [HB] (Project Humpback report). Why: docs/DECISIONS.md#r26-payload,
docs/DECISIONS.md#r26-banner, docs/DECISIONS.md#ff-d2. Details: docs/modules/stores.md.
"""

import numpy as np
import openmdao.api as om
from typing import Optional

from pydantic import BaseModel, ConfigDict, Field

from .smooth import ks_max

LB = 0.45359237
INCH = 0.0254
OZ_FT2 = 0.028349523125 / 0.09290304  # kg/m^2 per oz/ft^2


class Store2026Config(BaseModel):
    """Payload, banner and fuselage-sizing data. Every value is in configs/placeholders.yaml (universal) or the ruleset
    (configs/rulesets/dbf_2025_26.yaml), with its citation; there are no code defaults."""

    model_config = ConfigDict(extra="forbid")
    m_duck: float = Field(description="kg per passenger")
    m_puck: float = Field(description="kg per cargo")
    m_seat: float = Field(description="kg of restraint and foam per passenger seat")
    m_slot: float = Field(description="kg of restraint per cargo slot")
    pax_pitch: float = Field(description="m of passenger bay per passenger (rows of 3 abreast)")
    car_pitch: float = Field(description="m of cargo bay per cargo (two bays side by side)")
    w_cb: float = Field(description="m, payload fuselage width")
    h_cb: float = Field(description="m, payload fuselage height")
    rho_ks_bay: float = Field(
        50.0,
        description="KS sharpness of the bay-length max (log space, dimensionless: at most 1.4 % over the larger bay at a tie)",  # noqa: E501
    )
    x_pax_frac: float = Field(description="passenger CG station / chord from the wing LE")
    x_car_frac: float = Field(description="cargo CG station / chord")
    banner_ar: float = Field(description="banner length / height (rule limit 5)")
    banner_rho_A: float = Field(description="kg/m^2, banner fabric")
    m_banner_ballast: float = Field(description="kg")
    m_banner_mech: float = Field(description="kg, deployment and release mechanism, bridle and towline")
    cd_banner: float = Field(description="banner C_D on its own area L x H")
    dcd_stowed: float = Field(description="stowed banner Delta C_D0 on the wing area")
    x_banner_frac: float = Field(description="banner (stowed and mechanism) station / chord")
    # the rigid payload fuselage (docs/DECISIONS.md#ff-d2): shifts are differences from Humpback's layout, so each is
    # exactly 0.0 there
    x_cb_frac_ref: float = Field(
        description="Humpback's box centre / chord: the reference of the payload and motor stations"
    )
    c_ref: float = Field(description="m, Humpback's chord (the reference of the box station x_cb_ref)")
    n_p_ref: float = Field(description="passengers of Humpback's fuselage (its length l_cb_ref)")
    n_c_ref: float = Field(description="cargo of Humpback's fuselage")
    d_batt: float = Field(
        description="m, motor-to-pack-centre clearance: half the motor length + half the pack length (batt_fwd_margin)"
    )
    tail_clear: float = Field(description="m, clearance from the box's aft end to the HT leading edge (tail_margin)")
    fuselage_counts: Optional[list[float]] = Field(
        None,
        description="(n_p, n_c) the fuselage is sized for, if not the "
        "flown counts: the as-built fuselage of a validation flight with a "
        "smaller load ([HB] Table 42 flew 36/12 in the 42/14 fuselage)",
    )


class Stores2026(om.ExplicitComponent):
    """Payload and banner mass, CG station and drag at one mission point. All arithmetic is complex-step safe, so
    partials are complex step.

        mass   restraints n_p m_seat + n_c m_slot at every point; M2 adds n_p m_duck + n_c m_puck; M3 adds the banner,
               m_b = rho_A L H + m_ballast + m_mech, H = L / AR_banner
        CG     x_pay = sum m_i x_i / m_pay, stations as chord fractions from the wing leading edge ([HB] Table 36 /
        chord)
        drag   deployed banner on its own area, CD_banner = f_b C_D,b L H / S; stowed banner Delta C_D0

    Options
    -------
    config : Store2026Config
    payload : bool
        Ducks and pucks on board (M2).
    banner : bool
        Banner on board (M3), with the input L_banner.
    x_cb_frac0 : float
        Default of the input x_cb_frac (the config's, so it matches the stations' in a stand-alone model).

    Inputs
    ------
    n_p, n_c, S [m**2], c [m], banner_drag_factor, x_cb_frac, L_banner [inch] (banner on)

    Outputs
    -------
    m_pay [kg], x_pay [m], CD_stores, CD_banner, dCD_stowed, m_banner [kg]

    Symbols
    -------
    n_p, n_c : passengers and cargo, the design's capacity (shared by every point)
    L, H : banner length and height [m]
    m_b, cd_b : banner mass [kg] and drag
    dcd : dCD_stowed = CD_stowed - CD_banner, added on lap 1's legs before deployment (mission/smooth_lap.py)
    dx : shift of the box from Humpback's layout, chord fraction; 0.0 at Humpback
    """

    def initialize(self):
        self.options.declare("config", types=Store2026Config)
        self.options.declare("payload", default=False, desc="ducks and pucks on board (M2)")
        self.options.declare("banner", default=False, desc="banner on board (M3): input L_banner")
        self.options.declare(
            "x_cb_frac0",
            default=0.416,
            desc="default of the input x_cb_frac (the config's, so it matches the stations' in a stand-alone model)",
        )

    def setup(self):
        for name, val, units in [
            ("n_p", 42.0, None),
            ("n_c", 14.0, None),
            ("S", 0.5, "m**2"),
            ("c", 0.3, "m"),
            ("banner_drag_factor", 1.0, None),
            ("x_cb_frac", self.options["x_cb_frac0"], None),
        ]:
            self.add_input(name, val, units=units)
        if self.options["banner"]:
            self.add_input("L_banner", 197.0, units="inch")
        for name, units in [
            ("m_pay", "kg"),
            ("x_pay", "m"),
            ("CD_stores", None),
            ("CD_banner", None),
            ("dCD_stowed", None),
            ("m_banner", "kg"),
        ]:
            self.add_output(name, 0.1, units=units)
        self.declare_partials("*", "*", method="cs")

    def compute(self, inputs, outputs):
        cfg, opts = self.options["config"], self.options
        vals = {k: inputs[k] for k in inputs}
        n_p, n_c, chord, S = vals["n_p"], vals["n_c"], vals["c"], vals["S"]
        dx = vals["x_cb_frac"] - cfg.x_cb_frac_ref  # the payload moves with the box (rigid fuselage); 0.0 at Humpback
        x_pax, x_car = (cfg.x_pax_frac + dx) * chord, (cfg.x_car_frac + dx) * chord
        masses = [n_p * cfg.m_seat, n_c * cfg.m_slot]
        stations = [x_pax, x_car]
        if opts["payload"]:
            masses += [n_p * cfg.m_duck, n_c * cfg.m_puck]
            stations += [x_pax, x_car]
        m_b = cd_b = dcd = 0.0 * S
        if opts["banner"]:
            L = vals["L_banner"] * INCH
            H = L / cfg.banner_ar
            m_b = cfg.banner_rho_A * L * H + cfg.m_banner_ballast + cfg.m_banner_mech
            masses.append(m_b)
            stations.append(cfg.x_banner_frac * chord)
            cd_b = vals["banner_drag_factor"] * cfg.cd_banner * L * H / S
            dcd = cfg.dcd_stowed - cd_b
        m = sum(masses)
        outputs["m_pay"] = m
        outputs["x_pay"] = sum(mi * xi for mi, xi in zip(masses, stations)) / m
        outputs["CD_stores"] = cd_b
        outputs["CD_banner"] = cd_b
        outputs["dCD_stowed"] = dcd
        outputs["m_banner"] = m_b


def fuselage_length(c, n_p, n_c):
    """Lengths of the payload fuselage for the given counts: l_cb = KSmax(l_pax, l_car).

    Parameters
    ----------
    c : Store2026Config
    n_p, n_c : array
        Passenger and cargo counts.

    Returns
    -------
    (array, array, array)
        l_pax, l_car, l_cb [m].
    """
    l_pax, l_car = c.pax_pitch * n_p, c.car_pitch * n_c
    return l_pax, l_car, ks_max([l_pax, l_car], c.rho_ks_bay)[0]


class PayloadFuselage(om.ExplicitComponent):
    """The payload fuselage (cargo box), sized by the payload counts: l_cb = KSmax(pax_pitch n_p, car_pitch n_c), w_cb
    and h_cb held.

    It runs ahead of the planform and the MDA, since it depends on the design variables n_p, n_c only. Partials are
    complex step, through the KS (Kreisselmeier-Steinhauser) maximum.

    Options
    -------
    config : Store2026Config

    Inputs
    ------
    n_p, n_c

    Outputs
    -------
    l_cb, w_cb, h_cb, l_pax, l_car [m]
    """

    def initialize(self):
        self.options.declare("config", types=Store2026Config)

    def setup(self):
        self.add_input("n_p", 42.0)
        self.add_input("n_c", 14.0)
        for name in ("l_cb", "w_cb", "h_cb", "l_pax", "l_car"):
            self.add_output(name, 0.5, units="m")
        self.declare_partials("l_cb", ["n_p", "n_c"], method="cs")
        self.declare_partials("l_pax", "n_p", method="cs")
        self.declare_partials("l_car", "n_c", method="cs")

    def compute(self, inputs, outputs):
        cfg = self.options["config"]
        if cfg.fuselage_counts is not None:  # sized for fixed counts (an as-built fuselage), not the flown ones
            n_p, n_c = (np.array([float(n)]) + 0.0 * inputs["n_p"] for n in cfg.fuselage_counts)
        else:
            n_p, n_c = inputs["n_p"], inputs["n_c"]
        l_pax, l_car, outputs["l_cb"] = fuselage_length(cfg, n_p, n_c)
        outputs["w_cb"] = cfg.w_cb
        outputs["h_cb"] = cfg.h_cb
        outputs["l_pax"] = l_pax
        outputs["l_car"] = l_car


class Packaging2026(om.ExplicitComponent):
    """The rigid payload fuselage's packaging, after the MDA (docs/DECISIONS.md#ff-d2).

    The motor group sits at the box front at Humpback's nose length, and the margins keep the battery inside the
    fuselage and the tail behind it. Every shift is a difference from Humpback's layout (x_cb_frac_ref, c_ref, l_cb_ref
    at n_p_ref, n_c_ref), so it's exactly 0.0 there. Partials are complex step: short expressions.

        x_motor         = x_motor_ref + (x_cb - x_cb_ref) - (l_cb - l_cb_ref) / 2,  x_cb = x_cb_frac c
        batt_fwd_margin = x_batt - (x_motor + d_batt)                    >= 0
        batt_aft_margin = (x_cb + l_cb / 2) - x_batt                     >= 0
        tail_margin     = (x_tail - 0.25 c_ht) - (x_cb + l_cb / 2) - tail_clear >= 0,
                          x_tail = 0.25 c + l_ht (the HT quarter chord), c_ht = sqrt(S_ht / AR_ht)

    Options
    -------
    config : Store2026Config
    x_motor_ref : float
        m, Humpback's motor station (stability.x_motor).
    x_cb_frac0 : float
        Default of x_cb_frac (as Stores2026).

    Inputs
    ------
    c, l_cb, x_batt, l_ht [m], x_cb_frac, S_ht [m**2], AR_ht

    Outputs
    -------
    x_motor, batt_fwd_margin, batt_aft_margin, tail_margin, x_cb_aft [m]
    """

    def initialize(self):
        self.options.declare("config", types=Store2026Config)
        self.options.declare("x_motor_ref", types=float, desc="m, Humpback's motor station (stability.x_motor)")
        self.options.declare("x_cb_frac0", default=0.416, desc="default of x_cb_frac (as Stores2026)")

    def setup(self):
        # defaults as the other components of the point (stations, StaticStability): no ambiguity in a stand-alone model
        for name, val, units in [
            ("c", 0.25, "m"),
            ("x_cb_frac", self.options["x_cb_frac0"], None),
            ("l_cb", 1.0, "m"),
            ("x_batt", 0.1, "m"),
            ("l_ht", 0.8, "m"),
            ("S_ht", 0.06, "m**2"),
            ("AR_ht", 4.0, None),
        ]:
            self.add_input(name, val, units=units)
        for name in ("x_motor", "batt_fwd_margin", "batt_aft_margin", "tail_margin", "x_cb_aft"):
            self.add_output(name, 0.0, units="m")
        self.declare_partials("*", "*", method="cs")

    def compute(self, inputs, outputs):
        cfg = self.options["config"]
        x_cb = inputs["x_cb_frac"] * inputs["c"]
        x_cb_ref = cfg.x_cb_frac_ref * cfg.c_ref
        l_cb_ref = fuselage_length(cfg, np.array([cfg.n_p_ref]), np.array([cfg.n_c_ref]))[2]
        x_motor = self.options["x_motor_ref"] + (x_cb - x_cb_ref) - (inputs["l_cb"] - l_cb_ref) / 2.0
        aft = x_cb + inputs["l_cb"] / 2.0
        c_ht = np.sqrt(inputs["S_ht"] / inputs["AR_ht"])
        x_tail = 0.25 * inputs["c"] + inputs["l_ht"]
        outputs["x_motor"] = x_motor
        outputs["batt_fwd_margin"] = inputs["x_batt"] - (x_motor + cfg.d_batt)
        outputs["batt_aft_margin"] = aft - inputs["x_batt"]
        outputs["tail_margin"] = (x_tail - 0.25 * c_ht) - aft - cfg.tail_clear
        outputs["x_cb_aft"] = aft
