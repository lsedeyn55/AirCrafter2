"""Payload, banner and payload-sized fuselage of the 2025-26 ruleset (Banner Towing Bush Plane): mass, CG station,
drag and the cargo box of one mission point (problems/multipoint.py). docs/RESULTS_2026_PLAN.md §4.2-4.3.

Rules (2026-DBF-Rules.pdf, printed page "N of 34"): M2 carries passengers (2.3 in rubber ducks, 0.6-0.7 oz) and cargo
(hockey pucks, 6 oz) in a single passenger compartment and separate cargo bay(s), capable of >= 3 passengers per
declared cargo, >= 3 passengers and >= 1 cargo (p. 10-11); the restraints stay in the aircraft (p. 10-11). M3 tows a
banner (length >= 10 in, length / height <= 5), stowed externally for take-off and deployed on the downwind leg
after the first upwind turn (p. 12, p. 20). M1 flies with no payload (p. 19).

Model (per mission point; flags: `payload` = ducks and pucks on board (M2), `banner` = the banner on board (M3)):
    fuselage  (PayloadFuselage, ahead of the planform and the MDA) the cargo box is the constant-section payload
              fuselage (calibration convention), sized by the payload
              at Humpback's packing [HB] §4.2, §5.1: l_pax = pax_pitch n_p (rows of 3 abreast), l_car = car_pitch n_c
              (two bays of 7 side by side below the passengers), l_cb = KSmax(l_pax, l_car), w_cb, h_cb held at
              Humpback's maximum width and height. n_p and n_c are the design's capacity (shared by every point).
    mass      restraints n_p m_seat + n_c m_slot at every point; M2 adds n_p m_duck + n_c m_puck; M3 adds the banner
              m_b = rho_A L H + m_ballast + m_mech, H = L / AR_banner
    CG        x_pay = sum m_i x_i / m_pay, stations as chord fractions from the wing LE ([HB] Table 36 / chord)
    drag      deployed banner on its own area: CD_banner = f_b C_D,b L H / S (M3; f_b the banner-drag factor, an
              uncertain parameter); stowed banner Delta C_D0 (CD_stowed). The M3 point's steady (cruise) state is the
              tow, so CD_stores = CD_banner; the lap model flies lap 1's legs before deployment with
              dCD_stowed = CD_stowed - CD_banner added (mission/smooth_lap.py, banner_stowed_legs).
All arithmetic is complex-step safe (method="cs" partials).
"""

import numpy as np
import openmdao.api as om
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
    rho_ks_bay: float = Field(50.0, description="KS sharpness of the bay-length max (log space, dimensionless: at most 1.4 % over the larger bay at a tie)")
    x_pax_frac: float = Field(description="passenger CG station / chord from the wing LE")
    x_car_frac: float = Field(description="cargo CG station / chord")
    banner_ar: float = Field(description="banner length / height (rule limit 5)")
    banner_rho_A: float = Field(description="kg/m^2, banner fabric")
    m_banner_ballast: float = Field(description="kg")
    m_banner_mech: float = Field(description="kg, deployment and release mechanism, bridle and towline")
    cd_banner: float = Field(description="banner C_D on its own area L x H")
    dcd_stowed: float = Field(description="stowed banner Delta C_D0 on the wing area")
    x_banner_frac: float = Field(description="banner (stowed and mechanism) station / chord")


class Stores2026(om.ExplicitComponent):
    """Payload and banner mass, CG station and drag, and the payload fuselage, at one mission point."""

    def initialize(self):
        self.options.declare("config", types=Store2026Config)
        self.options.declare("payload", default=False, desc="ducks and pucks on board (M2)")
        self.options.declare("banner", default=False, desc="banner on board (M3): input L_banner")

    def setup(self):
        for name, val, units in [("n_p", 42.0, None), ("n_c", 14.0, None), ("S", 0.5, "m**2"), ("c", 0.3, "m"),
                                 ("banner_drag_factor", 1.0, None)]:
            self.add_input(name, val, units=units)
        if self.options["banner"]:
            self.add_input("L_banner", 197.0, units="inch")
        for name, units in [("m_pay", "kg"), ("x_pay", "m"), ("CD_stores", None), ("CD_banner", None),
                            ("dCD_stowed", None), ("m_banner", "kg")]:
            self.add_output(name, 0.1, units=units)
        self.declare_partials("*", "*", method="cs")

    def compute(self, inputs, outputs):
        c, o = self.options["config"], self.options
        x = {k: inputs[k] for k in inputs}
        n_p, n_c, chord, S = x["n_p"], x["n_c"], x["c"], x["S"]
        masses = [n_p * c.m_seat, n_c * c.m_slot]
        stations = [c.x_pax_frac * chord, c.x_car_frac * chord]
        if o["payload"]:
            masses += [n_p * c.m_duck, n_c * c.m_puck]
            stations += [c.x_pax_frac * chord, c.x_car_frac * chord]
        m_b = cd_b = dcd = 0.0 * S
        if o["banner"]:
            L = x["L_banner"] * INCH
            H = L / c.banner_ar
            m_b = c.banner_rho_A * L * H + c.m_banner_ballast + c.m_banner_mech
            masses.append(m_b)
            stations.append(c.x_banner_frac * chord)
            cd_b = x["banner_drag_factor"] * c.cd_banner * L * H / S
            dcd = c.dcd_stowed - cd_b
        m = sum(masses)
        outputs["m_pay"] = m
        outputs["x_pay"] = sum(mi * xi for mi, xi in zip(masses, stations)) / m
        outputs["CD_stores"] = cd_b
        outputs["CD_banner"] = cd_b
        outputs["dCD_stowed"] = dcd
        outputs["m_banner"] = m_b


class PayloadFuselage(om.ExplicitComponent):
    """The payload fuselage (cargo box) sized by the payload counts, ahead of the planform and the MDA (it depends on
    the design variables n_p, n_c only): l_cb = KSmax(pax_pitch n_p, car_pitch n_c), w_cb and h_cb held."""

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
        c = self.options["config"]
        l_pax, l_car = c.pax_pitch * inputs["n_p"], c.car_pitch * inputs["n_c"]
        outputs["l_cb"] = ks_max([l_pax, l_car], c.rho_ks_bay)[0]
        outputs["w_cb"] = c.w_cb
        outputs["h_cb"] = c.h_cb
        outputs["l_pax"] = l_pax
        outputs["l_car"] = l_car
