"""External stores of the 2024-25 ruleset (X-1 Supersonic Flight Program): fuel tanks on pylons, the X-1 test vehicle,
an optional internal tank. Each gives mass, station and drag at one mission point (problems/multipoint.py).

Rules (Rules_2025.pdf): tanks are commercial beverage bottles (>= 16 fl oz, all external tanks the same bottle for all
missions), at least two external tanks in M2 and M3 (empty allowed in M3), on removable pylons, the declared pylon set
fitted for every M2 and M3 attempt (p. 8-10); at most one internal tank, any bottle (p. 8); the X-1 (<= 0.55 lb) under
the fuselage between the external tanks in M2 and M3 (p. 8-9, p. 11). M1 flies with no payload and no pylons (p. 10-11).

Model (per mission point; flags say which stores are on board):
    mass   m_pay = n_ext m_bottle + m_internal_bottle + m_fuel + m_pylons + m_X1 + m_X1_structure
    CG     x_pay = sum m_i x_i / m_pay, stations as chord fractions from the wing LE (Woombah Table 34 / c)
    drag   CD_stores = f_drag [ n_ext Q_tank C_D,bottle A_bottle                   (pressure drag, frontal area)
                              + n_pyl Q_pyl C_f(Re_l) FF_wing S_wet,pyl            (Raymer component build-up)
                              + Q_X1 (C_f FF S_wet)_X1 wing + body ] / S
C_D,bottle = 0.523 per bottle, frontal area, bottom first: Woombah CFD, validated there against Hoerner's 0.82 for a
long cylinder (2025 USYD DBF Design Report, p. 43). Woombah's FlightStream build-up gives a far smaller bottle term
(Table 26, p. 28: CD0 0.00045 on the wing area for both bottles, about 1/20 of the CFD value); the CFD value is used.
Raymer, Aircraft Design: A Conceptual Approach, Ch. 12 (as aero/parasite.py) for the pylons and the X-1.
f_drag (store drag factor) is an uncertain parameter in RDO; interference factors are PLACEHOLDERS (OPEN_QUESTIONS).
All arithmetic is complex-step safe (method="cs" partials).
"""

from typing import Optional

import numpy as np
import openmdao.api as om
from pydantic import BaseModel, ConfigDict, Field

from .aero.parasite import cf_turbulent

LB = 0.45359237
INCH = 0.0254


class Bottle(BaseModel):
    model_config = ConfigDict(extra="forbid")
    volume_L: float = Field(1.0, description="L; Woombah: 1 L Pepsi bottle (Design Report p. 18)")
    diameter: float = Field(0.085, description="m, PLACEHOLDER (1 L PET bottle; OPEN_QUESTIONS Q36)")
    empty_mass: float = Field(0.040, description="kg, PLACEHOLDER (1 L PET bottle with cap; Q36)")


# Candidate external bottles (rules: >= 16 fl oz, any commercial non-glass bottle, p. 8). 1 L is Woombah's (p. 18);
# the dimensions and empty masses are PLACEHOLDERS (OPEN_QUESTIONS Q36).
BOTTLES = {"0.5L": dict(volume_L=0.5, diameter=0.065, empty_mass=0.025),   # 16.9 fl oz
           "1L": dict(volume_L=1.0, diameter=0.085, empty_mass=0.040),
           "2L": dict(volume_L=2.0, diameter=0.105, empty_mass=0.050)}


class StoreConfig(BaseModel):
    """The store set of a design (shared by its mission points) and the drag/mass data."""
    model_config = ConfigDict(extra="forbid")
    n_ext: int = Field(2, ge=2, description="external tanks = pylons (rules: >= 2 in M2 and M3, p. 9)")
    bottle: Bottle = Bottle()
    internal_bottle: Optional[Bottle] = Field(None, description="optional internal tank (rules p. 8); None = none")
    fill_density: float = Field(6.5 * LB, description="kg/L, steel powder packed density 6.5 lb/L (Woombah p. 18)")
    cd_bottle: float = Field(0.523, description="per bottle, frontal area, bottom first (Woombah CFD, p. 43)")
    Q_tank: float = Field(1.0, description="tank interference factor, PLACEHOLDER (Q37)")
    pylon_chord: float = Field(8.1 * INCH, description="m, Woombah Table 16 (p. 18)")
    pylon_thickness: float = Field(1.0 * INCH, description="m, Woombah Table 16")
    pylon_height: float = Field(1.82 * INCH, description="m, Woombah Table 16")
    pylon_xt: float = Field(0.3, description="pylon max-thickness location (x/c), PLACEHOLDER")
    Q_pylon: float = Field(1.0, description="pylon interference factor, PLACEHOLDER (Q37)")
    m_pylons: float = Field(0.44 * LB, description="kg, pylons + adapters for the set: Woombah 'payload structure' "
                                                   "0.44 lb (Table 34, p. 45)")
    m_x1: float = Field(0.37 * LB, description="kg, X-1 as flown: 0.37 lb (Woombah Table 39, p. 59); rules <= 0.55 lb")
    m_x1_structure: float = Field(0.22 * LB, description="kg, X-1 release structure: 0.22 lb (Woombah Table 34)")
    x1_wing_area: float = Field(82.0 * INCH**2, description="m^2, Woombah Table 25 (p. 24)")
    x1_wing_chord: float = Field(0.5 * (5.90 + 3.93) * INCH, description="m, mean of root/tip, Woombah Table 25")
    x1_wing_tc: float = Field(0.10, description="SD8020 (Woombah p. 24), t/c ~ 0.10, PLACEHOLDER")
    x1_body_length: float = Field(10.62 * INCH, description="m, Woombah Table 25")
    x1_body_diameter: float = Field(np.sqrt(2.36 * 2.75) * INCH, description="m, equivalent of 2.36 x 2.75 in")
    Q_x1: float = Field(1.0, description="X-1 interference factor, PLACEHOLDER (Q37)")
    x_tank_frac: float = Field(4.84 / 12.90, description="tank/pylon station / c from the wing LE: Woombah 4.84 in "
                                                         "(Table 34) / 12.90 in rectangular-equivalent chord")
    x_x1_frac: float = Field(-4.64 / 12.90, description="X-1 station / c: Woombah -4.64 in (Table 34) / 12.90 in")
    x_x1s_frac: float = Field(10.47 / 12.90, description="X-1 release structure / c: Woombah 10.47 in / 12.90 in")
    x_internal_frac: float = Field(0.25, description="internal tank station / c, PLACEHOLDER")

    @property
    def fuel_capacity(self):
        """kg of fill the declared tanks hold (external + internal)."""
        v = self.n_ext * self.bottle.volume_L + (self.internal_bottle.volume_L if self.internal_bottle else 0.0)
        return v * self.fill_density


class Stores(om.ExplicitComponent):
    """Store mass, CG station and drag at one mission point."""

    def initialize(self):
        self.options.declare("config", types=StoreConfig)
        self.options.declare("tanks", default=True, desc="external tanks (and the internal one, if any) on board")
        self.options.declare("fuel", default=False, desc="tanks filled (M2): input fuel_mass")
        self.options.declare("pylons", default=True)
        self.options.declare("x1", default=True)

    def setup(self):
        for name, val, units in [("v", 20.0, "m/s"), ("rho", 1.18, "kg/m**3"), ("mu", 1.8e-5, "Pa*s"),
                                 ("S", 0.5, "m**2"), ("c", 0.3, "m"), ("store_drag_factor", 1.0, None)]:
            self.add_input(name, val, units=units)
        if self.options["fuel"]:
            self.add_input("fuel_mass", 3.0, units="kg", desc="fill of all tanks (M2 design variable)")
        for name, units in [("m_pay", "kg"), ("x_pay", "m"), ("CD_stores", None), ("CD_tanks", None),
                            ("CD_pylons", None), ("CD_x1", None), ("fuel_capacity", "kg")]:
            self.add_output(name, 0.1, units=units)
        self.declare_partials("*", "*", method="cs")

    def compute(self, inputs, outputs):
        c = self.options["config"]
        o = self.options
        x = {k: inputs[k] for k in inputs}
        chord, S = x["c"], x["S"]
        masses, stations = [], []
        if o["tanks"]:
            masses.append(c.n_ext * c.bottle.empty_mass + 0.0 * chord)
            stations.append(c.x_tank_frac * chord)
            if c.internal_bottle is not None:
                masses.append(c.internal_bottle.empty_mass + 0.0 * chord)
                stations.append(c.x_internal_frac * chord)
        if o["fuel"]:  # fill split by volume between the external and internal tanks
            v_ext = c.n_ext * c.bottle.volume_L
            v_int = c.internal_bottle.volume_L if c.internal_bottle is not None else 0.0
            masses.append(x["fuel_mass"] * v_ext / (v_ext + v_int))
            stations.append(c.x_tank_frac * chord)
            if v_int:
                masses.append(x["fuel_mass"] * v_int / (v_ext + v_int))
                stations.append(c.x_internal_frac * chord)
        if o["pylons"]:
            masses.append(c.m_pylons + 0.0 * chord)
            stations.append(c.x_tank_frac * chord)
        if o["x1"]:
            masses += [c.m_x1 + 0.0 * chord, c.m_x1_structure + 0.0 * chord]
            stations += [c.x_x1_frac * chord, c.x_x1s_frac * chord]
        m = sum(masses) if masses else 0.0 * chord
        outputs["m_pay"] = m
        outputs["x_pay"] = (sum(mi * xi for mi, xi in zip(masses, stations)) / m) if masses else 0.0 * chord

        re_per_m = x["rho"] * x["v"] / x["mu"]
        cd_t = cd_p = cd_x = 0.0 * S
        if o["tanks"]:
            area = np.pi * c.bottle.diameter**2 / 4.0
            cd_t = c.n_ext * c.Q_tank * c.cd_bottle * area / S
        if o["pylons"]:
            tc = c.pylon_thickness / c.pylon_chord
            ff = 1.0 + 0.6 / c.pylon_xt * tc + 100.0 * tc**4
            swet = c.pylon_chord * c.pylon_height * (1.977 + 0.52 * tc)
            cd_p = c.n_ext * c.Q_pylon * cf_turbulent(re_per_m * c.pylon_chord) * ff * swet / S
        if o["x1"]:
            tc = c.x1_wing_tc
            ff_w = 1.0 + 0.6 / 0.3 * tc + 100.0 * tc**4
            swet_w = c.x1_wing_area * (1.977 + 0.52 * tc)
            f = c.x1_body_length / c.x1_body_diameter
            ff_b = 1.0 + 60.0 / f**3 + f / 400.0
            swet_b = np.pi * c.x1_body_diameter * c.x1_body_length
            cd_x = c.Q_x1 * (cf_turbulent(re_per_m * c.x1_wing_chord) * ff_w * swet_w
                             + cf_turbulent(re_per_m * c.x1_body_length) * ff_b * swet_b) / S
        k = x["store_drag_factor"]
        outputs["CD_tanks"] = k * cd_t
        outputs["CD_pylons"] = k * cd_p
        outputs["CD_x1"] = k * cd_x
        outputs["CD_stores"] = k * (cd_t + cd_p + cd_x)
        outputs["fuel_capacity"] = c.fuel_capacity
