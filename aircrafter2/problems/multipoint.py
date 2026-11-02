"""The multipoint problem: one airframe flying Mission 2 and Mission 3, and checked on Mission 1.

Two full mission points (problems/full_mdo.py) share the airframe and the motor; M3 flies the structure sized at M2. On
top sit the flight score (scoring/dbf2025.py or dbf2026.py) and the rule limits. `kind` selects the ruleset: 2024-25 or
2025-26. To evaluate a 2025-26 design, use build_multipoint_problem(mcfg, driver=False, mda_retry=True), then run_warm.
Why: docs/DECISIONS.md#r25-multipoint, docs/DECISIONS.md#ff-d5. Details: docs/modules/problems.md, "multipoint";
docs/modules/scoring.md.
"""

from typing import Literal, Optional

import numpy as np
import openmdao.api as om
from pydantic import BaseModel, ConfigDict, Field

from ..stores import LB, StoreConfig
from ..stores2026 import Store2026Config
from .full_mdo import (
    DV_REF,
    DV_UNITS,
    FullMDOConfig,
    apply_full_values,
    build_full_model,
    full_constraints,
    full_config_from_dict,
    kv_is_dv,
)
from .wing_aero import Bounds

POINTS = ("M2", "M3")
STORE_FLAGS = {
    "M2": dict(tanks=True, fuel=True, pylons=True, x1=True),  # Rules p. 11
    "M3": dict(tanks=True, fuel=False, pylons=True, x1=True),  # p. 11: tanks may be empty
    "M1": dict(tanks=False, fuel=False, pylons=False, x1=False),
}  # p. 10-11: no payload, no pylons
# 2025-26 (2026-DBF-Rules.pdf): M2 carries the passengers and cargo, M3 the banner, M1 nothing (p. 10, p. 19-20); the
# restraints and the payload fuselage are on board at every point (stores2026.py)
STORE_FLAGS_2026 = {
    "M2": dict(payload=True, banner=False),
    "M3": dict(payload=False, banner=True),
    "M1": dict(payload=False, banner=False),
}
EXTRA_UNITS = {"fuel_mass": "kg", "n_p": None, "n_c": None, "L_banner": "inch"}
# the 2025-26 extra design variables and the points that read them (the payload counts size the fuselage: all points)
EXTRA_POINTS_2026 = {"n_p": ("M1", "M2", "M3"), "n_c": ("M1", "M2", "M3"), "L_banner": ("M3",)}
PER_POINT_UNITS = {"capacity": "A*h", "x_batt": "m", "throttle": None}
EXTRA_REF = {"n_p": 10.0, "n_c": 10.0, "L_banner": 100.0}  # optimiser scaling of the 2025-26 extras (O(1))
# structural sizing inputs of an externally sized point, from the sizing point: mass, speed, wing Cm and CL_alpha
SIZING_LINKS = [
    ("m_total", "m_sizing"),
    ("v", "v_sizing"),
    ("cruise.Cm_wing", "Cm_sizing"),
    ("cruise.CL_alpha", "CL_alpha_sizing"),
]
# the flight-safety constraints that fail the M1 check (m1_check); the model-validity limits are only reported
M1_SAFETY = frozenset({"stall_margin", "SM", "thrust_margin_static", "wall_ratio", "r_tube", "turn_speed_margin"})
# 2025-26 (kind dbf2026; placeholders, docs/OPEN_QUESTIONS.md Q9, Q62; docs/DECISIONS.md#mr-m1-sm): SM fails the M1
# check only below SM_min; above SM_max it is listed (SM_high), since a too-large margin costs trim drag and elevator authority, not flight safety.
# The 2024-25 check (kind dbf2025) keeps both bounds.
M1_SM_ONE_SIDED = frozenset({"dbf2026"})


class PointConfig(BaseModel):
    """One mission point's settings: its APC prop (enumerated; rules p. 14)."""

    model_config = ConfigDict(extra="forbid")
    prop: str = Field(description="APC prop of this mission point (enumerated; rules p. 14)")


class RulesConfig(BaseModel):
    """Rule limits and the battery / ESC data that turn them into constraints. The values are in the ruleset
    (configs/rulesets/<ruleset>.yaml, multipoint.rules), with their citations; there are no code defaults."""

    model_config = ConfigDict(extra="forbid")
    togw_max_lb: float = Field(description="TOGW limit (lb)")
    energy_total_max_Wh: float = Field(
        description="propulsion total stored energy (Wh); also the per-pack limit for packs that share the total"
    )
    fuse_max_A: float = Field(description="arming fuse rating (A)")
    c_rating: float = Field(description="pack C rating: fuse <= capacity x C")
    laps_m2_min: Optional[float] = Field(description="M2 laps required in the window (None: no requirement)")
    span_max_ft: float = Field(description="maximum wingspan (ft): the upper bound of b")
    pax_per_cargo_min: Optional[float] = Field(
        None,
        description="2025-26: passengers per cargo the aircraft must "
        "carry (n_p >= this x n_c); None for other rulesets",
    )
    fuselage_span_ratio_max: Optional[float] = Field(
        None,
        description="2025-26: payload fuselage length / span "
        "upper limit (an engineering cap, PLACEHOLDER); None = no cap",
    )


class MultipointSettings(BaseModel):
    """The multipoint block of a config: points, per-point variables, ruleset kind, stores, payload, scoring and
    rules."""

    model_config = ConfigDict(extra="forbid")
    points: dict[Literal["M2", "M3"], PointConfig]
    per_point: list[str] = Field(
        ["capacity", "x_batt", "throttle"], description="design variables duplicated per point (suffix _M2 / _M3)"
    )
    per_point_bounds: dict[str, Bounds] = Field({}, description="bounds of per-point DVs not in design_variables")
    per_point_initial: dict[str, float] = Field({}, description="start values by suffixed name, e.g. throttle_M3")
    kind: Literal["dbf2025", "dbf2026"] = Field(
        "dbf2025", description="the ruleset's problem: its store set, extra design variables and scoring plug-in"
    )
    fuel: Bounds = Field(
        Bounds(lower=0.0, upper=1e9), description="kg; the upper bound is capped at the tanks' capacity"
    )
    fuel_initial: float = 3.0
    stores: Optional[StoreConfig] = Field(None, description="dbf2025: the ruleset's multipoint.stores")
    stores2026: Optional[Store2026Config] = Field(None, description="dbf2026: payload, banner and payload fuselage")
    payload: dict[str, Bounds] = Field({}, description="dbf2026: bounds of n_p, n_c and L_banner (inch)")
    payload_initial: dict[str, float] = Field({}, description="dbf2026: start values of n_p, n_c, L_banner")
    scoring: dict = Field(
        {},
        description="the plug-in's options (dbf2025: R2, R3, bonus, x1_weight_lb, w2, w3; "
        "dbf2026: NI_max, B_max, w2, w3)",
    )
    rules: RulesConfig = Field(description="the ruleset's multipoint.rules")


class MultipointConfig(BaseModel):
    """A base FullMDOConfig (airframe, powertrain, analysis settings) plus the multipoint block."""

    model_config = ConfigDict(extra="forbid")
    base: FullMDOConfig
    mp: MultipointSettings

    def store_set(self, name):
        """(store config, flags) of one point (M1, M2, M3)."""
        if self.mp.kind == "dbf2026":
            return self.mp.stores2026, STORE_FLAGS_2026[name]
        return self.mp.stores, STORE_FLAGS[name]

    def extra_dvs(self):
        """The ruleset's design variables outside the base airframe and the per-point ones: {name: Bounds}."""
        if self.mp.kind == "dbf2026":
            return {k: self.mp.payload[k] for k in ("n_p", "n_c", "L_banner")}
        return {"fuel_mass": self.fuel_bounds}

    def extra_initial(self):
        """Start values of the extra design variables: {name: value}."""
        if self.mp.kind == "dbf2026":
            return {k: self.mp.payload_initial.get(k, 0.5 * (b.lower + b.upper)) for k, b in self.extra_dvs().items()}
        return {"fuel_mass": self.mp.fuel_initial}

    def extra_points(self, name):
        """The extra design variables that point `name` reads."""
        if self.mp.kind == "dbf2026":
            return [k for k, pts in EXTRA_POINTS_2026.items() if name in pts]
        return ["fuel_mass"] if name == "M2" else []

    def point_config(self, name):
        """The base config with this point's prop. M1 flies M3's prop.

        Parameters
        ----------
        name : str
            "M1", "M2" or "M3".

        Returns
        -------
        FullMDOConfig
        """
        cfg = self.base.model_copy(deep=True)
        cfg.propulsion.prop = self.mp.points["M3" if name == "M1" else name].prop
        return cfg

    @property
    def shared_dvs(self):
        """The design variables shared by the points: {name: Bounds}."""
        return {k: b for k, b in self.base.design_variables.items() if k not in self.mp.per_point}

    def point_dvs(self, name):
        """One point's own design variables, suffixed: {capacity_M2: Bounds, ...}.

        Parameters
        ----------
        name : str
            "M2" or "M3".

        Returns
        -------
        dict
            Raises ValueError if a per-point variable has no bounds.
        """
        out = {}
        for k in self.mp.per_point:
            bd = self.base.design_variables.get(k) or self.mp.per_point_bounds.get(k)
            if bd is None:
                raise ValueError(f"per-point design variable {k!r} needs bounds (design_variables or per_point_bounds)")
            out[f"{k}_{name}"] = bd
        return out

    @property
    def fuel_bounds(self):
        """Bounds of fuel_mass [kg], capped at the tanks' capacity."""
        return Bounds(lower=self.mp.fuel.lower, upper=min(self.mp.fuel.upper, self.mp.stores.fuel_capacity))

    def all_dvs(self):
        """Every design variable of the problem: {name: Bounds}."""
        dvs = dict(self.shared_dvs)
        for p in POINTS:
            dvs.update(self.point_dvs(p))
        dvs.update(self.extra_dvs())
        return dvs


def load_multipoint_config(path, placeholders=None, ruleset=None):
    """A multipoint YAML over its layers: placeholders.yaml, then the ruleset (its multipoint: block included).

    Parameters
    ----------
    path : str or Path
    placeholders, ruleset : dict, optional
        Layer data to use instead of the files (the UI's unsaved edits).

    Returns
    -------
    MultipointConfig
    """
    from pathlib import Path

    from ..config.loader import load_layered

    path = Path(path)
    data = load_layered(path, FullMDOConfig.model_fields, multipoint=True, placeholders=placeholders, ruleset=ruleset)
    mp = data.pop("multipoint")
    return MultipointConfig(base=full_config_from_dict(data, path), mp=MultipointSettings(**mp))


class _PointSetter:
    """set_val and get_val for one point: shared names at the top, per-point names suffixed, the rest under
    <prefix>points.<point>.

    prefix: e.g. "nodes.n3." for a robust-design node.
    """

    def __init__(self, prob, point, shared, per_point, prefix="", extra=("fuel_mass",)):
        """Keep the problem, the point and the name sets."""
        self.prob, self.point, self.shared, self.per_point = prob, point, set(shared), set(per_point)
        self.prefix, self.extra = prefix, set(extra)

    def _name(self, name):
        """The model name of a config name, at this point."""
        if name in self.shared or name in self.extra:
            return name
        if name in self.per_point:
            return f"{name}_{self.point}"
        return f"{self.prefix}points.{self.point}.{name}"

    def set_val(self, name, val, units=None, indices=None):
        """prob.set_val with this point's name."""
        self.prob.set_val(self._name(name), val, units=units, indices=indices)

    def get_val(self, name, units=None, indices=None):
        """prob.get_val with this point's name."""
        return self.prob.get_val(self._name(name), units=units, indices=indices)


def multipoint_constraints(mcfg):
    """Each point's full-model constraints, then the rules.

    Parameters
    ----------
    mcfg : MultipointConfig

    Returns
    -------
    list of (str, dict)
        (absolute output name, add_constraint kwargs).
    """
    cons = []
    for p in POINTS:
        for name, kw in full_constraints(mcfg.point_config(p)):
            cons.append((f"points.{p}.{name}", kw))
        cons += [
            (f"rules_{p}.energy_margin_Wh", dict(lower=0.0, ref=10.0)),
            (f"rules_{p}.fuse_margin", dict(lower=0.0, ref=10.0)),
            (f"rules_{p}.fuse_margin_C", dict(lower=0.0, ref=10.0)),
        ]
    cons.append(("rules.togw_margin", dict(lower=0.0, ref=1.0)))
    if mcfg.mp.rules.laps_m2_min is not None:
        cons.append(("points.M2.laps_frac", dict(lower=mcfg.mp.rules.laps_m2_min)))
    if mcfg.mp.rules.pax_per_cargo_min is not None:
        cons.append(("rules.pax_margin", dict(lower=0.0, ref=10.0)))
    if mcfg.mp.rules.fuselage_span_ratio_max is not None:
        cons.append(("rules.fuselage_margin", dict(lower=0.0, ref=0.1)))
    if mcfg.mp.kind == "dbf2026":  # the rigid payload fuselage (stores2026.Packaging2026): battery inside, tail behind
        cons += [
            (f"points.{p}.{n}", dict(lower=0.0, ref=0.1))
            for p in POINTS
            for n in ("batt_fwd_margin", "batt_aft_margin")
        ]
        cons.append(("points.M2.tail_margin", dict(lower=0.0, ref=0.1)))  # shared geometry: the same at M3
    return cons


def build_multipoint_model(m, mcfg, mda_retry=False, defaults=True):
    """Add the two mission points, the score and the rule components to a group, with the design variables promoted at
    its level.

    Parameters
    ----------
    m : om.Group
    mcfg : MultipointConfig
    mda_retry : bool
        Use RetryNewton in the points' MDAs.
    defaults : bool
        Set the design variables' input defaults here (False when m is one of several robust nodes: set them at the
        top).

    Returns
    -------
    ScoringPlugin
        The score's plug-in.
    """
    from ..scoring.dbf2025 import DBF2025, DBF2025Score
    from ..scoring.dbf2026 import DBF2026, DBF2026Score

    shared = list(mcfg.shared_dvs)
    per = mcfg.mp.per_point
    extra = list(mcfg.extra_dvs())
    points = m.add_subsystem(
        "points", om.ParallelGroup(), promotes_inputs=shared + [f"{k}_{p}" for p in POINTS for k in per] + extra
    )
    for p in POINTS:
        g = om.Group()
        build_full_model(
            g,
            mcfg.point_config(p),
            mda_retry=mda_retry,
            sizing="own" if p == "M2" else "external",
            stores=mcfg.store_set(p),
        )
        promo = shared + [(k, f"{k}_{p}") for k in per] + mcfg.extra_points(p)
        points.add_subsystem(p, g, promotes_inputs=promo)
    for src, dst in SIZING_LINKS:  # M3 flies the airframe sized at M2 (same structural loads)
        m.connect(f"points.M2.{src}", f"points.M3.{dst}")

    if defaults:
        set_dv_defaults(m, mcfg)

    if mcfg.mp.kind == "dbf2026":
        plugin = DBF2026(**mcfg.mp.scoring)
        m.add_subsystem(
            "scoring",
            DBF2026Score(plugin=plugin),
            promotes_inputs=["n_p", "n_c", "L_banner", "b"],
            promotes_outputs=DBF2026Score.OUT,
        )
        m.connect("points.M2.laps_frac", "scoring.laps_M2")
        m.connect("points.M2.E_nom_total", "scoring.E_M2")
        m.connect("points.M3.laps_frac", "scoring.laps_M3")
    else:
        plugin = DBF2025(**mcfg.mp.scoring)
        m.add_subsystem(
            "scoring",
            DBF2025Score(plugin=plugin),
            promotes_inputs=["fuel_mass"],
            promotes_outputs=["score", "M1", "M2", "M3", "flight_score", "n_pre"],
        )
        m.connect("points.M2.ThreeLapTime", "scoring.t3_M2")
        m.connect("points.M3.laps_frac", "scoring.laps_M3")

    r = mcfg.mp.rules
    eqs = ["togw_margin = togw_max - m_total"]
    kw = dict(togw_margin={"units": "kg"}, togw_max={"units": "kg", "val": r.togw_max_lb * LB}, m_total={"units": "kg"})
    promo = []
    if r.pax_per_cargo_min is not None:  # 2025-26: >= 3 passengers per cargo [2026 rules p. 10]
        eqs.append("pax_margin = n_p - ratio * n_c")
        kw.update(ratio={"val": r.pax_per_cargo_min})
        promo += ["n_p", "n_c"]
    if r.fuselage_span_ratio_max is not None:  # 2025-26: payload fuselage no longer than a multiple of the span
        eqs.append("fuselage_margin = ratio_fus * b - l_cb")
        kw.update(
            ratio_fus={"val": r.fuselage_span_ratio_max},
            b={"units": "m"},
            l_cb={"units": "m"},
            fuselage_margin={"units": "m"},
        )
        promo += ["b"]
    m.add_subsystem("rules", om.ExecComp(eqs, **kw), promotes_inputs=promo)
    if r.fuselage_span_ratio_max is not None:
        m.connect("points.M2.l_cb", "rules.l_cb")
    m.connect("points.M2.m_total", "rules.m_total")
    for p in POINTS:
        m.add_subsystem(
            f"rules_{p}",
            om.ExecComp(
                [
                    "energy_margin_Wh = e_max - E_nom_total",
                    "fuse_margin = fuse_max - I_pack_static",
                    "fuse_margin_C = c_rating * capacity - I_pack_static",
                ],
                energy_margin_Wh={"units": "W*h"},
                e_max={"units": "W*h", "val": r.energy_total_max_Wh},
                E_nom_total={"units": "W*h"},
                fuse_margin={"units": "A"},
                fuse_max={"units": "A", "val": r.fuse_max_A},
                I_pack_static={"units": "A"},
                fuse_margin_C={"units": "A"},
                c_rating={"units": "1/h", "val": r.c_rating},
                capacity={"units": "A*h"},
            ),
        )
        m.connect(f"points.{p}.E_nom_total", f"rules_{p}.E_nom_total")
        m.connect(f"points.{p}.I_pack_static", f"rules_{p}.I_pack_static")
        if "capacity" in per:
            m.connect(f"capacity_{p}", f"rules_{p}.capacity")
        else:
            m.connect("capacity", f"rules_{p}.capacity")
    return plugin


def set_dv_defaults(m, mcfg):
    """Input defaults (start values and units) of every multipoint design variable.

    Parameters
    ----------
    m : om.Group
    mcfg : MultipointConfig
    """
    base = mcfg.base
    for name in mcfg.shared_dvs:
        bd = base.design_variables[name]
        start = base.propulsion.Kv if name == "Kv" else 0.5 * (bd.lower + bd.upper)
        m.set_input_defaults(name, base.initial.get(name, start), units=DV_UNITS.get(name))
    for p in POINTS:
        for name, bd in mcfg.point_dvs(p).items():
            k = name[: -len(p) - 1]
            start = mcfg.mp.per_point_initial.get(name, base.initial.get(k, 0.5 * (bd.lower + bd.upper)))
            m.set_input_defaults(name, start, units=PER_POINT_UNITS.get(k))
    for name, val in mcfg.extra_initial().items():
        m.set_input_defaults(name, val, units=EXTRA_UNITS[name])


def add_dvs(m, mcfg, only=None):
    """The multipoint design variables (bounds, units, refs) on a model, with the span capped at the rule.

    Parameters
    ----------
    m : om.Group
    mcfg : MultipointConfig
    only : iterable of str, optional
        A subset of the names (the rest stay model inputs at their set values; cost-scaling studies).
    """
    for name, bd in mcfg.all_dvs().items():
        if only is not None and name not in only:
            continue
        extra = name in EXTRA_UNITS
        k = name if name in mcfg.shared_dvs or extra else name.rsplit("_", 1)[0]
        upper = min(bd.upper, mcfg.mp.rules.span_max_ft * 0.3048) if name == "b" else bd.upper
        m.add_design_var(
            name,
            lower=bd.lower,
            upper=upper,
            units=EXTRA_UNITS[name] if extra else DV_UNITS.get(k),
            ref=EXTRA_REF.get(name, DV_REF.get(k)),
        )


def add_multipoint_driver(prob, mcfg, optimizer="SLSQP", only=None):
    """Set the optimiser, the design variables, the flight-score objective and the constraints on a problem.

    Parameters
    ----------
    prob : om.Problem
    mcfg : MultipointConfig
    optimizer : str
    only : iterable of str, optional
        As in add_dvs.
    """
    m = prob.model
    b = mcfg.base
    prob.driver = om.ScipyOptimizeDriver(optimizer=optimizer, tol=b.tol, maxiter=b.maxiter, disp=True)
    add_dvs(m, mcfg, only)
    m.add_objective("score", ref=-5.0)  # maximise the flight score (dbf2025 and dbf2026 both score about 5)
    for name, kw in multipoint_constraints(mcfg):
        m.add_constraint(name, **kw)


def apply_multipoint_values(prob, mcfg, prefix=""):
    """Put config values onto a set-up multipoint problem.

    Parameters
    ----------
    prob : om.Problem
    mcfg : MultipointConfig
    prefix : str
        Path to the multipoint group, e.g. "nodes.n3." for a robust-design node.
    """
    shared, per, extra = list(mcfg.shared_dvs), mcfg.mp.per_point, list(mcfg.extra_dvs())
    for p in POINTS:
        apply_full_values(_PointSetter(prob, p, shared, per, prefix, extra), mcfg.point_config(p), stores=True)
    for name, val in mcfg.mp.per_point_initial.items():
        prob.set_val(name, val)
    for name, val in mcfg.extra_initial().items():
        prob.set_val(name, val, units=EXTRA_UNITS[name])


def build_multipoint_problem(mcfg, driver=True, mda_retry=False):
    """The set-up multipoint problem of a config.

    Parameters
    ----------
    mcfg : MultipointConfig
    driver : bool
        Add the optimiser.
    mda_retry : bool
        The points' MDAs warm-start and fall back (rdo_problem.RetryNewton) before declaring failure.

    Returns
    -------
    om.Problem
    """
    prob = om.Problem(reports=False)
    build_multipoint_model(prob.model, mcfg, mda_retry=mda_retry)
    if driver:
        add_multipoint_driver(prob, mcfg)
    prob.setup()
    apply_multipoint_values(prob, mcfg)
    return prob


def initial_values(mcfg):
    """Start values of a multipoint config, as set_dv_defaults and apply_multipoint_values set them.

    Parameters
    ----------
    mcfg : MultipointConfig

    Returns
    -------
    dict
        {design variable: start value}.
    """
    base = mcfg.base
    out = {}
    for name in mcfg.shared_dvs:
        bd = base.design_variables[name]
        out[name] = base.initial.get(name, base.propulsion.Kv if name == "Kv" else 0.5 * (bd.lower + bd.upper))
    for pt in POINTS:
        for name, bd in mcfg.point_dvs(pt).items():
            k = name[: -len(pt) - 1]
            out[name] = mcfg.mp.per_point_initial.get(name, base.initial.get(k, 0.5 * (bd.lower + bd.upper)))
    out.update(mcfg.extra_initial())
    return out


def run_warm(prob, mcfg_start, dv, run=True):
    """The warm path to a design: converge at the config's unfixed start design, then move to the target.

    The start design converges cold. A cold run_model at a low-speed 2025-26 banner design can diverge; this path can't
    be worse (docs/DECISIONS.md#ff-d5, docs/DECISIONS.md#mr-f2).

    Parameters
    ----------
    prob : om.Problem
        A multipoint problem (build_multipoint_problem) or the robust one (build_robust_multipoint): both promote the
        design variables at the top level.
    mcfg_start : MultipointConfig
        The config whose start design to converge at.
    dv : dict
        {design variable: value} of the target.
    run : bool
        Run the model at the target as well.

    Raises the AnalysisError of the start run (run=False) or of either run.
    """
    for k, x in initial_values(mcfg_start).items():
        prob.set_val(k, x)
    prob.run_model()
    for k, x in dv.items():
        prob.set_val(k, x)
    if run:
        prob.run_model()


def design_values(prob, mcfg):
    """The design variables of a multipoint problem (shared, per point, the ruleset's extras).

    Parameters
    ----------
    prob : om.Problem
    mcfg : MultipointConfig

    Returns
    -------
    dict
        {design variable: value}.
    """
    return {k: float(np.ravel(prob.get_val(k))[0]) for k in mcfg.all_dvs()}


def m1_check(mcfg, prob):
    """Mission 1 on the final design: M3's prop, battery and throttle, no payload and no pylons, the airframe sized at
    M2.

    Passing means the rule (3 laps in the window) and the flight-safety constraints M1_SAFETY. The others, such as
    v_fit_margin (the light M1 aircraft can outrun the T(v) fit at M3's throttle), are listed but don't fail it. For
    2025-26, SM above SM_max doesn't fail it either (docs/DECISIONS.md#mr-m1-sm).

    Parameters
    ----------
    mcfg : MultipointConfig
    prob : om.Problem
        The run multipoint problem.

    Returns
    -------
    dict
        passed, laps_frac, ThreeLapTime [s], m_total [kg], violated, SM_high, constraints; or passed and error if the
        MDA fails.
    """
    from openmdao.core.analysis_error import AnalysisError

    cfg = mcfg.point_config("M1")
    robust = cfg.mda_robust  # RetryNewton robust mode + QuietDirectSolver (docs/DECISIONS.md#mr-f1)
    q = om.Problem(reports=False)
    build_full_model(q.model, cfg, sizing="external", stores=mcfg.store_set("M1"), mda_retry=robust)
    q.setup()
    apply_full_values(q, cfg, stores=True)
    for name in list(mcfg.shared_dvs) + mcfg.extra_points("M1"):
        q.set_val(name, prob.get_val(name))
    for k in mcfg.mp.per_point:
        q.set_val(k, prob.get_val(f"{k}_M3"))
    for src, dst in SIZING_LINKS:
        q.set_val(dst, prob.get_val(f"points.M2.{src}"))
    q.final_setup()  # a setup error is raised here, not reported as MDA non-convergence below
    errors = (AnalysisError, RuntimeError) if robust else AnalysisError  # robust: a singular Jacobian too
    try:
        q.run_model()
    except errors as exc:
        return dict(passed=False, error=f"MDA non-convergence: {str(exc)[:200]}")
    v = lambda k: float(np.ravel(q.get_val(k))[0])  # noqa: E731
    cons = {n: v(n) for n, _ in full_constraints(cfg)}
    viol = [
        n
        for n, kw in full_constraints(cfg)
        if (kw.get("lower") is not None and cons[n] < kw["lower"] - 1e-6)
        or (kw.get("upper") is not None and cons[n] > kw["upper"] + 1e-6)
    ]
    sm_kw = dict(full_constraints(cfg)).get("SM", {})
    sm_high = "SM" in cons and sm_kw.get("upper") is not None and cons["SM"] > sm_kw["upper"] + 1e-6
    safety = set(viol) & M1_SAFETY
    if sm_high and mcfg.mp.kind in M1_SM_ONE_SIDED:
        safety.discard("SM")  # above SM_max: listed, not failed (below SM_min still fails)
    ok = v("laps_frac") >= 3.0 and not safety
    return dict(
        passed=bool(ok),
        laps_frac=v("laps_frac"),
        ThreeLapTime=v("ThreeLapTime"),
        m_total=v("m_total"),
        violated=viol,
        SM_high=bool(sm_high),
        constraints=cons,
    )


def kv_settings_report(mcfg):
    """Whether Kv is a design variable, and its bounds.

    Parameters
    ----------
    mcfg : MultipointConfig

    Returns
    -------
    dict
        kv_dv, bounds.
    """
    return dict(kv_dv=kv_is_dv(mcfg.base), bounds=mcfg.base.design_variables.get("Kv"))


__all__ = [
    "MultipointConfig",
    "load_multipoint_config",
    "build_multipoint_problem",
    "build_multipoint_model",
    "add_multipoint_driver",
    "apply_multipoint_values",
    "multipoint_constraints",
    "m1_check",
    "design_values",
    "initial_values",
    "run_warm",
    "POINTS",
    "STORE_FLAGS",
    "STORE_FLAGS_2026",
]
