"""Multipoint problem of the 2024-25 ruleset: one airframe flying Mission 2 and Mission 3 (and checked on Mission 1).

    model
      points  (om.ParallelGroup; serial without MPI)
        M2   full coupled analysis (full_mdo.build_full_model) with the M2 stores: filled tanks, pylons, X-1.
             The structure is sized here (the heaviest case: maximum fuel and battery, the rules' wing-tip test case,
             Rules_2025 p. 9 and p. 17).
        M3   full coupled analysis with the M3 stores: empty tanks, pylons, X-1. Flies M2's airframe: its structural
             sizing inputs are M2's mass and speed (sizing="external"), so the structure is the same.
      scoring (dbf2025): F = M1 + M2 + M3 from M2's fuel weight and 3-lap time and M3's laps
      rules   TOGW < 55 lb (M2), total propulsion energy <= 100 Wh and fuse current <= min(100 A, C x capacity) at each
              point, 3 laps in the M2 window

Design variables:
    shared      the airframe of the base config (b, c, tc_scale, camber_scale, l_ht, S_ht, S_vt, x_cb_frac) and the
                motor Kv: "the airplane must be flown in the same configuration for all three missions" (p. 10)
    per point   capacity, x_batt, throttle (names suffixed _M2 / _M3): other inspected battery packs may be flown
                between attempts (p. 16), the battery station sets the CG per mission (Woombah: p. 19, Table 34), and the
                throttle is the pilot's setting for each flight
    M2 only     fuel_mass, up to the declared tanks' capacity (stores.StoreConfig.fuel_capacity)
    enumerated  the prop of each point (the prop may change per attempt, p. 14), cells, n_motors, the store set

M1 (3 laps in 5 min, no payload, no pylons; p. 10-11) is an analysis-only check of a final design (m1_check).
Formulation, rules and sources: docs/RESULTS_PLAN.md, docs/modules/scoring.md.
"""

from typing import Literal

import numpy as np
import openmdao.api as om
from pydantic import BaseModel, ConfigDict, Field

from ..stores import LB, StoreConfig
from .full_mdo import (DV_REF, DV_UNITS, FullMDOConfig, apply_full_values, build_full_model, full_constraints,
                       full_config_from_dict, kv_is_dv)
from .wing_aero import Bounds

POINTS = ("M2", "M3")
STORE_FLAGS = {"M2": dict(tanks=True, fuel=True, pylons=True, x1=True),     # Rules p. 11
               "M3": dict(tanks=True, fuel=False, pylons=True, x1=True),    # p. 11: tanks may be empty
               "M1": dict(tanks=False, fuel=False, pylons=False, x1=False)}  # p. 10-11: no payload, no pylons
PER_POINT_UNITS = {"capacity": "A*h", "x_batt": "m", "throttle": None}
# structural sizing inputs of an externally sized point, from the sizing point: mass, speed, wing Cm and CL_alpha
SIZING_LINKS = [("m_total", "m_sizing"), ("v", "v_sizing"), ("cruise.Cm_wing", "Cm_sizing"),
                ("cruise.CL_alpha", "CL_alpha_sizing")]


class PointConfig(BaseModel):
    model_config = ConfigDict(extra="forbid")
    prop: str = Field(description="APC prop of this mission point (enumerated; rules p. 14)")


class RulesConfig(BaseModel):
    """2024-25 rule limits (Rules_2025.pdf) and the battery / ESC data that turn them into constraints."""
    model_config = ConfigDict(extra="forbid")
    togw_max_lb: float = Field(55.0, description="TOGW must be less than 55 lb (p. 14)")
    energy_total_max_Wh: float = Field(100.0, description="propulsion total stored energy (p. 14); also <= 100 Wh "
                                                          "per pack, implied for packs that share the total")
    fuse_max_A: float = Field(100.0, description="arming fuse rating (p. 15)")
    c_rating: float = Field(70.0, description="pack C rating: fuse <= capacity x C (p. 15); Dualsky 70C (Woombah p. 22)")
    laps_m2_min: float = Field(3.0, description="M2: 3 laps within the 5-minute window (p. 11)")
    span_max_ft: float = Field(6.0, description="maximum wingspan (p. 8): the upper bound of b")


class MultipointSettings(BaseModel):
    model_config = ConfigDict(extra="forbid")
    points: dict[Literal["M2", "M3"], PointConfig]
    per_point: list[str] = Field(["capacity", "x_batt", "throttle"],
                                 description="design variables duplicated per point (suffix _M2 / _M3)")
    per_point_bounds: dict[str, Bounds] = Field({}, description="bounds of per-point DVs not in design_variables")
    per_point_initial: dict[str, float] = Field({}, description="start values by suffixed name, e.g. throttle_M3")
    fuel: Bounds = Field(Bounds(lower=0.0, upper=1e9), description="kg; the upper bound is capped at the tanks' "
                                                                   "capacity")
    fuel_initial: float = 3.0
    stores: StoreConfig = StoreConfig()
    scoring: dict = Field({}, description="dbf2025 options: R2, R3, bonus, x1_weight_lb, w2, w3")
    rules: RulesConfig = RulesConfig()


class MultipointConfig(BaseModel):
    """A base FullMDOConfig (airframe, powertrain, analysis settings) plus the multipoint block."""
    model_config = ConfigDict(extra="forbid")
    base: FullMDOConfig
    mp: MultipointSettings

    def point_config(self, name):
        cfg = self.base.model_copy(deep=True)
        cfg.propulsion.prop = self.mp.points["M3" if name == "M1" else name].prop
        return cfg

    @property
    def shared_dvs(self):
        return {k: b for k, b in self.base.design_variables.items() if k not in self.mp.per_point}

    def point_dvs(self, name):
        out = {}
        for k in self.mp.per_point:
            bd = self.base.design_variables.get(k) or self.mp.per_point_bounds.get(k)
            if bd is None:
                raise ValueError(f"per-point design variable {k!r} needs bounds (design_variables or per_point_bounds)")
            out[f"{k}_{name}"] = bd
        return out

    @property
    def fuel_bounds(self):
        return Bounds(lower=self.mp.fuel.lower, upper=min(self.mp.fuel.upper, self.mp.stores.fuel_capacity))

    def all_dvs(self):
        dvs = dict(self.shared_dvs)
        for p in POINTS:
            dvs.update(self.point_dvs(p))
        dvs["fuel_mass"] = self.fuel_bounds
        return dvs


def load_multipoint_config(path):
    from pathlib import Path

    from ..config.loader import load_yaml

    path = Path(path)
    data = load_yaml(path)
    mp = data.pop("multipoint")
    return MultipointConfig(base=full_config_from_dict(data, path), mp=MultipointSettings(**mp))


class _PointSetter:
    """set_val / get_val for one point: shared names at the top, per-point names suffixed, the rest prefixed."""

    def __init__(self, prob, point, shared, per_point):
        self.prob, self.point, self.shared, self.per_point = prob, point, set(shared), set(per_point)

    def _name(self, name):
        if name in self.shared or name == "fuel_mass":
            return name
        if name in self.per_point:
            return f"{name}_{self.point}"
        return f"points.{self.point}.{name}"

    def set_val(self, name, val, units=None, indices=None):
        self.prob.set_val(self._name(name), val, units=units, indices=indices)

    def get_val(self, name, units=None, indices=None):
        return self.prob.get_val(self._name(name), units=units, indices=indices)


def multipoint_constraints(mcfg):
    """[(absolute output name, add_constraint kwargs)]: each point's full-model constraints, then the rules."""
    cons = []
    for p in POINTS:
        for name, kw in full_constraints(mcfg.point_config(p)):
            cons.append((f"points.{p}.{name}", kw))
        cons += [(f"rules_{p}.energy_margin_Wh", dict(lower=0.0, ref=10.0)),
                 (f"rules_{p}.fuse_margin", dict(lower=0.0, ref=10.0)),
                 (f"rules_{p}.fuse_margin_C", dict(lower=0.0, ref=10.0))]
    cons += [("rules.togw_margin", dict(lower=0.0, ref=1.0)),
             ("points.M2.laps_frac", dict(lower=mcfg.mp.rules.laps_m2_min))]
    return cons


def build_multipoint_model(m, mcfg, mda_retry=False):
    """Add the two mission points, the score and the rule components to group m (promoted DVs at m's level)."""
    from ..scoring.dbf2025 import DBF2025, DBF2025Score

    shared = list(mcfg.shared_dvs)
    per = mcfg.mp.per_point
    points = m.add_subsystem("points", om.ParallelGroup(),
                             promotes_inputs=shared + [f"{k}_{p}" for p in POINTS for k in per] + ["fuel_mass"])
    for p in POINTS:
        g = om.Group()
        build_full_model(g, mcfg.point_config(p), mda_retry=mda_retry, sizing="own" if p == "M2" else "external",
                         stores=(mcfg.mp.stores, STORE_FLAGS[p]))
        promo = shared + [(k, f"{k}_{p}") for k in per] + (["fuel_mass"] if p == "M2" else [])
        points.add_subsystem(p, g, promotes_inputs=promo)
    for src, dst in SIZING_LINKS:  # M3 flies the airframe sized at M2 (same structural loads)
        m.connect(f"points.M2.{src}", f"points.M3.{dst}")

    base = mcfg.base
    for name in shared:
        bd = base.design_variables[name]
        start = base.propulsion.Kv if name == "Kv" else 0.5 * (bd.lower + bd.upper)
        m.set_input_defaults(name, base.initial.get(name, start), units=DV_UNITS.get(name))
    for p in POINTS:
        for name, bd in mcfg.point_dvs(p).items():
            k = name[: -len(p) - 1]
            start = mcfg.mp.per_point_initial.get(name, base.initial.get(k, 0.5 * (bd.lower + bd.upper)))
            m.set_input_defaults(name, start, units=PER_POINT_UNITS.get(k))
    m.set_input_defaults("fuel_mass", mcfg.mp.fuel_initial, units="kg")

    plugin = DBF2025(**mcfg.mp.scoring)
    m.add_subsystem("scoring", DBF2025Score(plugin=plugin), promotes_inputs=["fuel_mass"],
                    promotes_outputs=["score", "M1", "M2", "M3", "flight_score", "n_pre"])
    m.connect("points.M2.ThreeLapTime", "scoring.t3_M2")
    m.connect("points.M3.laps_frac", "scoring.laps_M3")

    r = mcfg.mp.rules
    m.add_subsystem("rules", om.ExecComp("togw_margin = togw_max - m_total", togw_margin={"units": "kg"},
                                         togw_max={"units": "kg", "val": r.togw_max_lb * LB},
                                         m_total={"units": "kg"}))
    m.connect("points.M2.m_total", "rules.m_total")
    for p in POINTS:
        m.add_subsystem(f"rules_{p}", om.ExecComp(
            ["energy_margin_Wh = e_max - E_nom_total", "fuse_margin = fuse_max - I_pack_static",
             "fuse_margin_C = c_rating * capacity - I_pack_static"],
            energy_margin_Wh={"units": "W*h"}, e_max={"units": "W*h", "val": r.energy_total_max_Wh},
            E_nom_total={"units": "W*h"}, fuse_margin={"units": "A"}, fuse_max={"units": "A", "val": r.fuse_max_A},
            I_pack_static={"units": "A"}, fuse_margin_C={"units": "A"}, c_rating={"units": "1/h", "val": r.c_rating},
            capacity={"units": "A*h"}))
        m.connect(f"points.{p}.E_nom_total", f"rules_{p}.E_nom_total")
        m.connect(f"points.{p}.I_pack_static", f"rules_{p}.I_pack_static")
        if "capacity" in per:
            m.connect(f"capacity_{p}", f"rules_{p}.capacity")
        else:
            m.connect("capacity", f"rules_{p}.capacity")
    return plugin


def add_multipoint_driver(prob, mcfg, optimizer="SLSQP"):
    m = prob.model
    b = mcfg.base
    prob.driver = om.ScipyOptimizeDriver(optimizer=optimizer, tol=b.tol, maxiter=b.maxiter, disp=True)
    for name, bd in mcfg.all_dvs().items():
        k = name if name in mcfg.shared_dvs or name == "fuel_mass" else name.rsplit("_", 1)[0]
        upper = min(bd.upper, mcfg.mp.rules.span_max_ft * 0.3048) if name == "b" else bd.upper
        m.add_design_var(name, lower=bd.lower, upper=upper, units=DV_UNITS.get(k) if k != "fuel_mass" else "kg",
                         ref=DV_REF.get(k))
    m.add_objective("score", ref=-5.0)  # maximise the flight score
    for name, kw in multipoint_constraints(mcfg):
        m.add_constraint(name, **kw)


def apply_multipoint_values(prob, mcfg):
    shared, per = list(mcfg.shared_dvs), mcfg.mp.per_point
    for p in POINTS:
        apply_full_values(_PointSetter(prob, p, shared, per), mcfg.point_config(p), stores=True)
    for name, val in mcfg.mp.per_point_initial.items():
        prob.set_val(name, val)
    prob.set_val("fuel_mass", mcfg.mp.fuel_initial, units="kg")


def build_multipoint_problem(mcfg, driver=True):
    prob = om.Problem(reports=False)
    build_multipoint_model(prob.model, mcfg)
    if driver:
        add_multipoint_driver(prob, mcfg)
    prob.setup()
    apply_multipoint_values(prob, mcfg)
    return prob


def design_values(prob, mcfg):
    """{design variable: value} of a multipoint problem (shared, per point, fuel)."""
    return {k: float(np.ravel(prob.get_val(k))[0]) for k in mcfg.all_dvs()}


def m1_check(mcfg, prob):
    """Mission 1 on the final design: M3's prop, battery and throttle, no payload and no pylons, the airframe sized at
    M2. Returns dict(passed, laps_frac, ThreeLapTime, constraint values)."""
    from openmdao.core.analysis_error import AnalysisError

    cfg = mcfg.point_config("M1")
    q = om.Problem(reports=False)
    build_full_model(q.model, cfg, sizing="external", stores=(mcfg.mp.stores, STORE_FLAGS["M1"]))
    q.setup()
    apply_full_values(q, cfg, stores=True)
    for name in mcfg.shared_dvs:
        q.set_val(name, prob.get_val(name))
    for k in mcfg.mp.per_point:
        q.set_val(k, prob.get_val(f"{k}_M3"))
    for src, dst in SIZING_LINKS:
        q.set_val(dst, prob.get_val(f"points.M2.{src}"))
    try:
        q.run_model()
    except AnalysisError as exc:
        return dict(passed=False, error=f"MDA non-convergence: {str(exc)[:200]}")
    v = lambda k: float(np.ravel(q.get_val(k))[0])  # noqa: E731
    cons = {n: v(n) for n, _ in full_constraints(cfg)}
    viol = [n for n, kw in full_constraints(cfg)
            if (kw.get("lower") is not None and cons[n] < kw["lower"] - 1e-6)
            or (kw.get("upper") is not None and cons[n] > kw["upper"] + 1e-6)]
    # pass = the rule (3 laps in the window) and the flight-safety constraints; model-validity limits (v_fit_margin:
    # the light M1 aircraft can outrun the T(v) fit at M3's throttle) and the others are reported, not failed on
    safety = {"stall_margin", "SM", "thrust_margin_static", "wall_ratio", "r_tube", "turn_speed_margin"}
    ok = v("laps_frac") >= 3.0 and not (set(viol) & safety)
    return dict(passed=bool(ok), laps_frac=v("laps_frac"), ThreeLapTime=v("ThreeLapTime"), m_total=v("m_total"),
                violated=viol, constraints=cons)


def kv_settings_report(mcfg):
    return dict(kv_dv=kv_is_dv(mcfg.base), bounds=mcfg.base.design_variables.get("Kv"))


__all__ = ["MultipointConfig", "load_multipoint_config", "build_multipoint_problem", "build_multipoint_model",
           "add_multipoint_driver", "apply_multipoint_values", "multipoint_constraints", "m1_check",
           "design_values", "POINTS", "STORE_FLAGS"]
