"""External stores of the 2024-25 ruleset (X-1 Supersonic Flight Program): tanks on pylons, the X-1 test vehicle, an
optional internal tank.

Each gives mass, CG station and drag at one mission point. Part of the MDA in problems/full_mdo.py: it feeds m_pay and
x_pay to the mass and stability models, and CD_stores to the drag (docs/DECISIONS.md#r25-multipoint).
Sources: Rules_2025.pdf; Woombah (2025 USYD DBF Design Report); Raymer Ch. 12. Details: docs/modules/stores.md.
"""

from typing import Optional

import numpy as np
import openmdao.api as om
from pydantic import BaseModel, ConfigDict, Field

from .aero.parasite import cf_turbulent

LB = 0.45359237
INCH = 0.0254


class Bottle(BaseModel):
    """A commercial bottle used as a tank: volume [L], diameter [m] and empty mass [kg]."""

    model_config = ConfigDict(extra="forbid")
    volume_L: float = Field(description="L")
    diameter: float = Field(description="m")
    empty_mass: float = Field(description="kg")


def _ruleset_bottles(name="dbf_2024_25"):
    """The candidate bottles of a ruleset file, or {} if the file doesn't exist."""
    from .config.loader import CONFIGS_DIR, load_layer

    path = CONFIGS_DIR / "rulesets" / f"{name}.yaml"
    return dict(load_layer(path).get("bottles") or {}) if path.exists() else {}


# Candidate external bottles of the bottle study: the 'bottles:' block of configs/rulesets/dbf_2024_25.yaml (rules:
# >= 16 fl oz, any commercial non-glass bottle, p. 8; dimensions and empty masses PLACEHOLDERS, OPEN_QUESTIONS Q36).
BOTTLES = _ruleset_bottles()


class StoreConfig(BaseModel):
    """The store set of a design (shared by its mission points) and the drag/mass data. Every value is in the ruleset
    (configs/rulesets/dbf_2024_25.yaml, multipoint.stores), with its citation; there are no code defaults."""

    model_config = ConfigDict(extra="forbid")
    n_ext: int = Field(ge=2, description="external tanks = pylons")
    bottle: Bottle
    internal_bottle: Optional[Bottle] = Field(None, description="optional internal tank; None = none")
    fill_density: float = Field(description="kg/L")
    cd_bottle: float = Field(description="per bottle, frontal area")
    Q_tank: float = Field(description="tank interference factor")
    pylon_chord: float = Field(description="m")
    pylon_thickness: float = Field(description="m")
    pylon_height: float = Field(description="m")
    pylon_xt: float = Field(description="pylon max-thickness location (x/c)")
    Q_pylon: float = Field(description="pylon interference factor")
    m_pylons: float = Field(description="kg, pylons + adapters for the set")
    m_x1: float = Field(description="kg, X-1 as flown")
    m_x1_structure: float = Field(description="kg, X-1 release structure")
    x1_wing_area: float = Field(description="m^2")
    x1_wing_chord: float = Field(description="m")
    x1_wing_tc: float = Field(description="X-1 wing t/c")
    x1_body_length: float = Field(description="m")
    x1_body_diameter: float = Field(description="m")
    Q_x1: float = Field(description="X-1 interference factor")
    x_tank_frac: float = Field(description="tank/pylon station / c from the wing LE")
    x_x1_frac: float = Field(description="X-1 station / c")
    x_x1s_frac: float = Field(description="X-1 release structure station / c")
    x_internal_frac: float = Field(description="internal tank station / c")

    @classmethod
    def from_ruleset(cls, name="dbf_2024_25", **over):
        """The ruleset's store set (configs/rulesets/<name>.yaml), with field overrides.

        Parameters
        ----------
        name : str
            Ruleset file name.
        **over
            Fields to override.

        Returns
        -------
        StoreConfig
        """
        from .config.loader import CONFIGS_DIR, load_layer

        data = dict(load_layer(CONFIGS_DIR / "rulesets" / f"{name}.yaml")["multipoint"]["stores"])
        data.update(over)
        return cls(**data)

    @property
    def fuel_capacity(self):
        """kg of fill the declared tanks hold (external + internal)."""
        v = self.n_ext * self.bottle.volume_L + (self.internal_bottle.volume_L if self.internal_bottle else 0.0)
        return v * self.fill_density


class Stores(om.ExplicitComponent):
    """Store mass, CG station and drag at one mission point. All arithmetic is complex-step safe, so partials are
    complex step.

    Options
    -------
    config : StoreConfig
    tanks : bool
        External tanks (and the internal one, if any) on board.
    fuel : bool
        Tanks filled (M2), with the input fuel_mass.
    pylons, x1 : bool
        Pylons and the X-1 on board.

    Inputs
    ------
    v [m/s], rho [kg/m**3], mu [Pa*s], S [m**2], c [m], store_drag_factor, fuel_mass [kg] (fuel on)

    Outputs
    -------
    m_pay [kg], x_pay [m], CD_stores, CD_tanks, CD_pylons, CD_x1, fuel_capacity [kg]

    Symbols
    -------
    chord, S : wing chord [m] and area [m**2]
    re_per_m : rho v / mu, Reynolds number per metre [1/m]
    cd_t, cd_p, cd_x : drag of the tanks, pylons and X-1, before the factor k
    tc, ff, swet : t/c, form factor and wetted area of a lifting surface (as aero/parasite.py)
    f, ff_b, swet_b : fineness, form factor and wetted area of the X-1 body
    k : store drag factor f_drag, an uncertain parameter in the robust problem
    """

    def initialize(self):
        self.options.declare("config", types=StoreConfig)
        self.options.declare("tanks", default=True, desc="external tanks (and the internal one, if any) on board")
        self.options.declare("fuel", default=False, desc="tanks filled (M2): input fuel_mass")
        self.options.declare("pylons", default=True)
        self.options.declare("x1", default=True)

    def setup(self):
        for name, val, units in [
            ("v", 20.0, "m/s"),
            ("rho", 1.18, "kg/m**3"),
            ("mu", 1.8e-5, "Pa*s"),
            ("S", 0.5, "m**2"),
            ("c", 0.3, "m"),
            ("store_drag_factor", 1.0, None),
        ]:
            self.add_input(name, val, units=units)
        if self.options["fuel"]:
            self.add_input("fuel_mass", 3.0, units="kg", desc="fill of all tanks (M2 design variable)")
        for name, units in [
            ("m_pay", "kg"),
            ("x_pay", "m"),
            ("CD_stores", None),
            ("CD_tanks", None),
            ("CD_pylons", None),
            ("CD_x1", None),
            ("fuel_capacity", "kg"),
        ]:
            self.add_output(name, 0.1, units=units)
        self.declare_partials("*", "*", method="cs")

    def compute(self, inputs, outputs):
        cfg = self.options["config"]
        opts = self.options
        vals = {k: inputs[k] for k in inputs}
        chord, S = vals["c"], vals["S"]
        masses, stations = [], []
        if opts["tanks"]:
            masses.append(cfg.n_ext * cfg.bottle.empty_mass + 0.0 * chord)
            stations.append(cfg.x_tank_frac * chord)
            if cfg.internal_bottle is not None:
                masses.append(cfg.internal_bottle.empty_mass + 0.0 * chord)
                stations.append(cfg.x_internal_frac * chord)
        if opts["fuel"]:  # fill split by volume between the external and internal tanks
            v_ext = cfg.n_ext * cfg.bottle.volume_L
            v_int = cfg.internal_bottle.volume_L if cfg.internal_bottle is not None else 0.0
            masses.append(vals["fuel_mass"] * v_ext / (v_ext + v_int))
            stations.append(cfg.x_tank_frac * chord)
            if v_int:
                masses.append(vals["fuel_mass"] * v_int / (v_ext + v_int))
                stations.append(cfg.x_internal_frac * chord)
        if opts["pylons"]:
            masses.append(cfg.m_pylons + 0.0 * chord)
            stations.append(cfg.x_tank_frac * chord)
        if opts["x1"]:
            masses += [cfg.m_x1 + 0.0 * chord, cfg.m_x1_structure + 0.0 * chord]
            stations += [cfg.x_x1_frac * chord, cfg.x_x1s_frac * chord]
        m = sum(masses) if masses else 0.0 * chord
        outputs["m_pay"] = m
        outputs["x_pay"] = (sum(mi * xi for mi, xi in zip(masses, stations)) / m) if masses else 0.0 * chord

        re_per_m = vals["rho"] * vals["v"] / vals["mu"]
        cd_t = cd_p = cd_x = 0.0 * S
        if opts["tanks"]:
            area = np.pi * cfg.bottle.diameter**2 / 4.0
            cd_t = cfg.n_ext * cfg.Q_tank * cfg.cd_bottle * area / S
        if opts["pylons"]:
            tc = cfg.pylon_thickness / cfg.pylon_chord
            ff = 1.0 + 0.6 / cfg.pylon_xt * tc + 100.0 * tc**4
            swet = cfg.pylon_chord * cfg.pylon_height * (1.977 + 0.52 * tc)
            cd_p = cfg.n_ext * cfg.Q_pylon * cf_turbulent(re_per_m * cfg.pylon_chord) * ff * swet / S
        if opts["x1"]:
            tc = cfg.x1_wing_tc
            ff_w = 1.0 + 0.6 / 0.3 * tc + 100.0 * tc**4
            swet_w = cfg.x1_wing_area * (1.977 + 0.52 * tc)
            f = cfg.x1_body_length / cfg.x1_body_diameter
            ff_b = 1.0 + 60.0 / f**3 + f / 400.0
            swet_b = np.pi * cfg.x1_body_diameter * cfg.x1_body_length
            cd_x = (
                cfg.Q_x1
                * (
                    cf_turbulent(re_per_m * cfg.x1_wing_chord) * ff_w * swet_w
                    + cf_turbulent(re_per_m * cfg.x1_body_length) * ff_b * swet_b
                )
                / S
            )
        k = vals["store_drag_factor"]
        outputs["CD_tanks"] = k * cd_t
        outputs["CD_pylons"] = k * cd_p
        outputs["CD_x1"] = k * cd_x
        outputs["CD_stores"] = k * (cd_t + cd_p + cd_x)
        outputs["fuel_capacity"] = cfg.fuel_capacity
