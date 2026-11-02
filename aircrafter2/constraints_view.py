"""Inventory of a problem's constraints, checks and limits (the UI's Constraints tab): everything but the design-variable
bounds, with the limit, its units and the file and line that set it.

    constraint_table(cfg, ...)   rows in five sections:
        1  optimiser constraints   the driver's list, read from the set-up problem (deterministic or RDO, single-point
                                   or multipoint), in the driver's order, with the raw lower / upper / equals / ref
        2  post-checks             check_fixed_limits, m1_check, oei_check: checked after a run, never constraints
        3  analysis limits         AnalysisError raise sites (the wind limit), report-only outputs, the G&G sizing
                                   criteria: not enforced by the optimiser
        4  MDA balances            L = W, T = D and the mass closure, solved by the MDA's own solvers
        5  unused limit fields     limit fields of the schema that no row of sections 1-4 reads for this problem

The optimiser section is never a hand-kept copy: the problem is built and set up (setup + final_setup, no run_model)
and its driver's constraints are joined to REGISTER. A driver constraint with no register entry raises
UnregisteredConstraint, and so does a margin output of the model that is neither constrained nor in the register:
a new constraint or check cannot drop out of the tab silently. To add a constraint: add it to the problem
(full_constraints, multipoint_constraints, a scoring plug-in) and add its Entry to REGISTER below.

RDO problems are set up with a one-node rule (udr, no active parameter): the driver's constraints do not depend on the
rule (they are the stats.* moments and the reference node n0), and the full rule takes 4-21 s to set up
(tests/test_constraints_view.py checks that both give the same list).
"""

import contextlib
import io
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Optional

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
INF = 1e30  # OpenMDAO stores an absent bound as +/- 1e30


class UnregisteredConstraint(ValueError):
    """A driver constraint (or an unconstrained margin output) with no REGISTER entry."""


# --- the register ---------------------------------------------------------------------------------------------------

@dataclass(frozen=True)
class Entry:
    """One constraint, check or limit.

    name         register key: the driver's promoted name without the RDO / multipoint prefixes (points.M2.,
                 nodes.n0., stats.), rules_M2. -> rules.; for sections 2-4 a descriptive key
    definition   the inequality, e.g. 'V - 1.2 V_stall >= 0'
    limit_paths  config paths that set the limit (dotted, as in the layer files)
    group        performance | propulsion | stability | structure | aero | twin | rules | scoring
    kind         safety | validity | rule | engineering (safety: multipoint.M1_SAFETY where it overlaps)
    hard_coded   ((file, pattern, what), ...): limits written in the code (file:line found by pattern)
    section      1-4 (5 is computed)
    applies      sections 2-4: ctx -> bool (ctx: problem_context); None = always
    outputs      section 3: the model outputs this entry covers (base names); the row appears when one exists
    anchors      ((file, pattern), ...): the source lines this entry accounts for (the section-3/4 source scan)
    """
    name: str
    label: str
    units: str
    definition: str
    group: str
    kind: str
    limit_paths: tuple = ()
    hard_coded: tuple = ()
    section: int = 1
    applies: Optional[Callable] = None
    outputs: tuple = ()
    anchors: tuple = ()
    note: str = ""


SL = "aircrafter2/mission/smooth_lap.py"
FM = "aircrafter2/problems/full_mdo.py"
MP = "aircrafter2/problems/multipoint.py"
GG = "aircrafter2/mass/garcia_gamboa"


def _zero(name, file=FM):
    """The hard-coded zero bound of a margin constraint, at its full_constraints line."""
    return (file, rf'"{re.escape(name)}", dict\(lower=0\.0', "lower bound 0")


_OPT = [
    Entry("stall_margin", "stall margin", "m/s", "V - 1.2 V_stall >= 0", "performance", "safety",
          hard_coded=((SL, r"stall_margin=V - 1\.2 \* Vstall", "1.2 V_stall"), _zero("stall_margin"))),
    Entry("turn_speed_margin", "turn speed margin", "m/s", "v after the 360 deg turn - V_stall >= 0", "performance",
          "safety", hard_coded=((SL, r"turn_speed_margin=v360 - Vstall", "1.0 V_stall"),
                                _zero("turn_speed_margin"))),
    Entry("thrust_margin_static", "take-off thrust margin", "N", "T(0) - T_static >= 0, T_static from Eq. 3.3",
          "performance", "safety", ("takeoff_distance", "mu_roll"),
          hard_coded=((SL, r"VR = 1\.2 \* np\.sqrt", "V_R = 1.2 V_stall(C_L,to) in Eq. 3.3"),
                      _zero("thrust_margin_static"))),
    Entry("energy_margin", "3-lap energy margin", "J", "(1 - reserve) E_usable - E_3laps >= 0", "performance",
          "engineering", ("energy_reserve", "require_3lap_energy"), hard_coded=(_zero("energy_margin"),),
          note="on/off: require_3lap_energy"),
    Entry("rpm_margin", "rpm margin at top speed", "rpm", "rpm_fraction x 150000/D - rpm(v) >= 0", "propulsion",
          "engineering", ("propulsion.rpm_fraction",),
          hard_coded=(("aircrafter2/propulsion/apc.py", r"rpm_limit=150000\.0", "APC 150000/D[in]"),
                      _zero("rpm_margin"))),
    Entry("I_margin", "static current margin", "A", "I_max - I_static >= 0 (per motor/ESC)", "propulsion",
          "engineering", ("propulsion.I_max",), hard_coded=(_zero("I_margin"),),
          note="a constraint only with Kv a design variable; always a post-check"),
    Entry("I_pack_margin", "pack current margin", "A", "pack_I_max - I_pack_static >= 0", "propulsion", "engineering",
          ("propulsion.pack_I_max",), hard_coded=(_zero("I_pack_margin"),)),
    Entry("v_fit_margin", "T(v) fit margin", "m/s", "v_top (fixed) or v_fit_max (throttle) - v >= 0", "propulsion",
          "validity", ("propulsion.v_top", "propulsion.f_fit"), hard_coded=(_zero("v_fit_margin"),)),
    Entry("SM", "static margin", "-", "SM_min <= SM <= SM_max", "stability", "safety",
          ("stability.SM_min", "stability.SM_max")),
    Entry("V_h", "HT volume coefficient", "-", "V_h_min <= V_h <= V_h_max", "stability", "engineering",
          ("stability.V_h_min", "stability.V_h_max")),
    Entry("V_v", "VT volume coefficient", "-", "V_v_min <= V_v <= V_v_max", "stability", "engineering",
          ("stability.V_v_min", "stability.V_v_max")),
    Entry("wall_ratio", "spar wall ratio", "-", "t_tube / r_tube <= wall_ratio_max (thin-wall validity)", "structure",
          "safety", ("wall_ratio_max",)),
    Entry("r_tube", "spar tube radius", "m", "r_tube >= r_tube_min", "structure", "safety", ("r_tube_min",)),
    Entry("cruise.cl_margin_sec", "cruise section C_l margin", "-", "C_l,max - C_l >= 0 on every strip (cruise)",
          "aero", "validity", hard_coded=((FM, r'"cruise\.cl_margin_sec", dict\(lower=0\.0', "lower bound 0"),)),
    Entry("fus_clear_margin", "prop-fuselage clearance", "m", "y_m - D/2 - (w_cb/2 + c_clear) >= 0", "twin",
          "engineering", ("propulsion.c_clear",), hard_coded=(_zero("fus_clear_margin"),)),
    Entry("tip_clear_margin", "prop disc inside the tip", "m", "b/2 - (y_m + D/2) >= 0", "twin", "engineering",
          hard_coded=(_zero("tip_clear_margin"),)),
    Entry("rules.energy_margin_Wh", "propulsion energy", "W*h", "energy_total_max_Wh - E_nom_total >= 0", "rules",
          "rule", ("multipoint.rules.energy_total_max_Wh",),
          hard_coded=((MP, r'f"rules_\{p\}\.energy_margin_Wh", dict\(lower=0\.0', "lower bound 0"),)),
    Entry("rules.fuse_margin", "arming fuse", "A", "fuse_max_A - I_pack_static >= 0", "rules", "rule",
          ("multipoint.rules.fuse_max_A",),
          hard_coded=((MP, r'f"rules_\{p\}\.fuse_margin", dict\(lower=0\.0', "lower bound 0"),)),
    Entry("rules.fuse_margin_C", "pack C rating", "A", "c_rating x capacity - I_pack_static >= 0", "rules", "rule",
          ("multipoint.rules.c_rating",),
          hard_coded=((MP, r'f"rules_\{p\}\.fuse_margin_C", dict\(lower=0\.0', "lower bound 0"),)),
    Entry("rules.togw_margin", "take-off gross weight", "kg", "togw_max_lb - m_total(M2) >= 0", "rules", "rule",
          ("multipoint.rules.togw_max_lb",),
          hard_coded=((MP, r'"rules\.togw_margin", dict\(lower=0\.0', "lower bound 0"),)),
    Entry("laps_frac", "M2 laps in the window", "-", "laps_frac(M2) >= laps_m2_min", "rules", "rule",
          ("multipoint.rules.laps_m2_min",)),
    Entry("rules.pax_margin", "passengers per cargo", "-", "n_p - pax_per_cargo_min x n_c >= 0", "rules", "rule",
          ("multipoint.rules.pax_per_cargo_min",),
          hard_coded=((MP, r'"rules\.pax_margin", dict\(lower=0\.0', "lower bound 0"),)),
    Entry("rules.fuselage_margin", "payload fuselage length", "m", "fuselage_span_ratio_max x b - l_fus >= 0",
          "rules", "engineering", ("multipoint.rules.fuselage_span_ratio_max",),
          hard_coded=((MP, r'"rules\.fuselage_margin", dict\(lower=0\.0', "lower bound 0"),)),
    Entry("stats.score_sigma", "score standard deviation", "-", "sigma(score) <= sigma_cap (objective mean_sigma_cap)",
          "scoring", "engineering", ("robust.sigma_cap",)),
]


def _mp(ctx):
    return ctx["multipoint"]


def _sp(ctx):
    return not ctx["multipoint"]


_POST = [
    Entry("post.I_margin", "static current (post-check)", "A", "I_max - I_static >= 0", "propulsion", "engineering",
          ("propulsion.I_max",), section=2, applies=_sp,
          hard_coded=((FM, r'ok = out\["I_margin_A"\] >= 0', "0 A"),),
          anchors=((FM, r"With a fixed motor the static current is set by the powertrain choice alone"),),
          note="check_fixed_limits, after every single-point run (the only current check with a fixed motor)"),
    Entry("post.rpm_margin", "rpm (post-check)", "rpm", "rpm_margin >= -1 rpm", "propulsion", "engineering",
          ("propulsion.rpm_fraction",), section=2, applies=_sp,
          hard_coded=((FM, r'out\["rpm_margin"\] >= -1\.0', "-1 rpm tolerance"),), note="check_fixed_limits"),
    Entry("post.I_pack_margin", "pack current (post-check)", "A", "pack_I_max - I_pack_static >= 0", "propulsion",
          "engineering", ("propulsion.pack_I_max",), section=2,
          applies=lambda c: _sp(c) and c["pack_I_max"], note="check_fixed_limits, when pack_I_max is set"),
    Entry("m1.laps", "M1 laps (post-check)", "-", "laps_frac(M1) >= 3", "rules", "rule", section=2, applies=_mp,
          hard_coded=((MP, r'ok = v\("laps_frac"\) >= 3\.0', "3 laps"),),
          note="m1_check: Mission 1 on the final design (M3's prop, battery and throttle; no payload)"),
    Entry("m1.safety", "M1 safety constraints (post-check)", "-",
          "the M1 point's safety constraints within their bounds (+/- 1e-6)", "rules", "safety",
          ("stability.SM_min", "stability.SM_max", "wall_ratio_max", "r_tube_min", "takeoff_distance", "mu_roll"),
          section=2, applies=_mp, hard_coded=((MP, r'kw\["lower"\] - 1e-6', "1e-6 tolerance"),),
          note="m1_check; the M1_SAFETY set: stall_margin, turn_speed_margin, thrust_margin_static, SM, wall_ratio, "
               "r_tube (the others are reported)"),
    Entry("oei.rudder", "one engine inoperative (post-check)", "deg",
          "rudder for the live unit's yaw at oei_speed_factor x V_stall <= rudder_max_deg", "stability", "safety",
          ("stability.rudder_max_deg",), section=2,
          applies=lambda c: c["twin"] and not c["multipoint"],
          anchors=((FM, r"OEI post-check only"), ("aircrafter2/stability/static.py", r"report-only post-check")),
          note="oei_check (report only, D8); model inputs eta_v, rudder_tau, AR_vt: Placeholders tab"),
    Entry("oei.speed", "OEI check speed (post-check)", "-", "the OEI check is flown at oei_speed_factor x V_stall",
          "stability", "safety", ("stability.oei_speed_factor",), section=2,
          applies=lambda c: c["twin"] and not c["multipoint"], note="oei_check"),
]

_ANALYSIS = [
    Entry("wind.lift_off", "headwind at rest below the lift-off speed", "m/s",
          "headwind along lap 1's first straight < k_lof V_stall, else AnalysisError (non-physical lap)",
          "performance", "validity", ("course.takeoff.k_lof",), section=3, applies=lambda c: c["takeoff"],
          anchors=((SL, r"raise AnalysisError\(f\"\{self\.pathname\}: non-physical lap \(headwind"),)),
    Entry("wind.lap", "ground speed stays positive", "m/s",
          "lap times finite and > 0 (headwind below the lap-1 start speed V_stall or V_LOF), else AnalysisError",
          "performance", "validity", section=3,
          anchors=((SL, r"raise AnalysisError\(f\"\{self\.pathname\}: non-physical lap \(lap times"),)),
    Entry("nf_confidence_min", "NeuralFoil confidence (min over strips)", "-",
          "report only: no threshold", "aero", "validity", section=3,
          outputs=("cruise.nf_confidence_min", "mid.nf_confidence_min", "turn.nf_confidence_min"),
          anchors=(("aircrafter2/aero/profile_drag.py", r'add_output\("nf_confidence_min"'),)),
    Entry("n_stalled", "strips clamped at C_l,max", "-", "report only: 0 expected", "aero", "validity", section=3,
          outputs=("cruise.n_stalled", "mid.n_stalled", "turn.n_stalled"),
          anchors=(("aircrafter2/aero/profile_drag.py", r'add_output\("n_stalled"'),)),
    Entry("cl_margin_sec.mid_turn", "mid and turn section C_l margin", "-",
          "C_l,max - C_l on every strip at the mid and turn polar points: computed, not constrained", "aero",
          "validity", section=3, outputs=("mid.cl_margin_sec", "turn.cl_margin_sec")),
    Entry("lapCount", "integer lap count", "-", "report only (step function; laps_frac is the smooth count)",
          "performance", "validity", section=3, outputs=("lapCount",),
          anchors=((SL, r"lapCount \(integer\) is reported only"), (SL, r'add_output\("lapCount"'))),
    Entry("takeoff_accel_margin", "smallest ground-roll acceleration", "m/s**2",
          "report only: > 0 for a roll that reaches V_LOF (smooth floor a_floor in the model)", "performance",
          "validity", section=3, applies=lambda c: c["takeoff"], outputs=("takeoff_accel_margin",),
          hard_coded=((SL, r'declare\("a_floor", default=0\.1', "a_floor 0.1 m/s^2"),),
          anchors=((SL, r"smallest acceleration over the roll \(report only\)"),)),
    Entry("takeoff_straight_margin", "first straight - ground roll", "m",
          "report only: >= 0 when lift-off is on lap 1's first straight (in-air remainder floored at rem_floor)",
          "performance", "validity", section=3, applies=lambda c: c["takeoff"], outputs=("takeoff_straight_margin",),
          hard_coded=((SL, r'declare\("rem_floor", default=1\.0', "rem_floor 1 m"),)),
    Entry("prop_clear_margin", "prop disc to disc", "m", "2 y_m - (D + c_clear) >= 0: reported only (implied by the "
          "fuselage clearance)", "twin", "engineering", ("propulsion.c_clear",), section=3, outputs=("prop_clear_margin",),
          anchors=((FM, r"prop-to-prop clearance is implied by the fuselage one: reported only"),)),
    Entry("rpm_max", "largest rotor speed over the fit nodes", "rpm", "report only (the top node is above the "
          "flight envelope; rpm_margin checks the top speed)", "propulsion", "validity", section=3,
          outputs=("rpm_max",), anchors=(("aircrafter2/propulsion/powertrain.py", r"speed over the nodes \(report only"),)),
    Entry("fit_residual_T", "T(v) fit residual", "N", "report only: max |fit - node|", "propulsion", "validity",
          section=3, outputs=("propulsion.fit_residual_T",),
          anchors=(("aircrafter2/propulsion/powertrain.py", r'add_output\("fit_residual_T"'),)),
    Entry("min_gauge_margin", "spar wall: load-sized - minimum gauge", "m",
          "report only: < 0 when the minimum wall t_tube_min governs", "structure", "validity",
          ("mass.inputs.t_tube_min",), section=3, outputs=("min_gauge_margin",)),
    Entry("t_tb_gauge_margin", "boom wall: load-sized - minimum gauge", "m",
          "report only: < 0 when the minimum wall t_tb_min governs", "structure", "validity",
          ("mass.inputs.t_tb_min",), section=3, outputs=("t_tb_gauge_margin",)),
    Entry("sizing.spar", "spar sizing criteria", "-",
          "enforced by the G&G sizing (by construction): stress <= ultimate / SF, tip deflection <= delta_tip_ratio "
          "b/2, wall >= t_tube_min", "structure", "safety",
          ("mass.inputs.SF", "mass.inputs.sigma_spar", "mass.inputs.tau_spar", "mass.inputs.delta_tip_ratio",
           "mass.inputs.t_tube_min"), section=3),
    Entry("sizing.boom", "tail-boom sizing criteria", "-",
          "enforced by the G&G sizing (by construction): stress <= ultimate / SF, twist <= theta_tb_max, "
          "wall >= t_tb_min", "structure", "safety",
          ("mass.inputs.SF", "mass.inputs.sigma_tb", "mass.inputs.tau_tb", "mass.inputs.theta_tb_max",
           "mass.inputs.t_tb_min"), section=3),
]

_MDA = [
    Entry("balance.alpha_cruise", "lift balance L = W", "N", "L(alpha_cruise) = W: solves alpha_cruise (deg)",
          "performance", "validity", ("mda_maxiter",), section=4,
          hard_coded=((FM, r"newton = om\.NewtonSolver\(solve_subsystems=True, maxiter=cfg\.mda_maxiter, atol=1e-10",
                       "Newton atol = rtol = 1e-10"),),
          anchors=((FM, r"err_on_non_converge=True"),
                   ("aircrafter2/robustness/rdo_problem.py", r"raise AnalysisError\(str\(exc\)\) from exc")),
          note="non-convergence raises AnalysisError (err_on_non_converge); with mda_retry, RetryNewton falls back to "
               "warm starts, and under mda_robust also to a continuation in the MDA inputs"),
    Entry("balance.v", "thrust balance T = D", "N", "T(v) = D(v): solves v (m/s), bounded 5 m/s <= v <= v_top",
          "performance", "validity", ("mda_maxiter", "propulsion.v_top"), section=4,
          hard_coded=((FM, r"lower=5\.0, upper=cfg\.propulsion\.v_top", "v in [5 m/s, v_top] (BoundsEnforceLS)"),),
          note="under mda_robust, a failed solve falls back to a continuation in the MDA inputs (the low-speed M3 "
               "banner designs have a second, sub-stall T = D root; docs/MDA_ROBUSTNESS_PLAN.md)"),
    Entry("mass.closure", "mass closure m = m_total", "kg", "m = m_total (G&G, block Gauss-Seidel with Aitken)",
          "structure", "validity", section=4, applies=lambda c: True,
          hard_coded=((f"{GG}/group.py", r"NonlinearBlockGS\(maxiter=50, atol=1e-12, rtol=1e-12",
                       "NLBGS maxiter 50, atol = rtol = 1e-12"),),
          anchors=((f"{GG}/group.py", r"err_on_non_converge=True"),
                   ("aircrafter2/problems/aero_mass.py", r"err_on_non_converge=True")),
          note="multipoint M3 has no closure: sized at M2's mass (sizing external)"),
]

REGISTER = {e.name: e for e in _OPT + _POST + _ANALYSIS + _MDA}

# the config value behind each editable cell of a row: lower / upper are the driver's bounds themselves; limit is the
# value behind a margin's 0 bound (the physical limit). Every path here is edited in the Constraints tab only.
CELLS = {
    "SM": dict(lower="stability.SM_min", upper="stability.SM_max"),
    "V_h": dict(lower="stability.V_h_min", upper="stability.V_h_max"),
    "V_v": dict(lower="stability.V_v_min", upper="stability.V_v_max"),
    "wall_ratio": dict(upper="wall_ratio_max"),
    "r_tube": dict(lower="r_tube_min"),
    "laps_frac": dict(lower="multipoint.rules.laps_m2_min"),
    "stats.score_sigma": dict(upper="robust.sigma_cap"),
    "thrust_margin_static": dict(limit="takeoff_distance"),
    "rpm_margin": dict(limit="propulsion.rpm_fraction"),
    "I_margin": dict(limit="propulsion.I_max"),
    "I_pack_margin": dict(limit="propulsion.pack_I_max"),
    "fus_clear_margin": dict(limit="propulsion.c_clear"),
    "rules.energy_margin_Wh": dict(limit="multipoint.rules.energy_total_max_Wh"),
    "rules.fuse_margin": dict(limit="multipoint.rules.fuse_max_A"),
    "rules.fuse_margin_C": dict(limit="multipoint.rules.c_rating"),
    "rules.togw_margin": dict(limit="multipoint.rules.togw_max_lb"),
    "rules.pax_margin": dict(limit="multipoint.rules.pax_per_cargo_min"),
    "rules.fuselage_margin": dict(limit="multipoint.rules.fuselage_span_ratio_max"),
    "post.I_margin": dict(limit="propulsion.I_max"),
    "post.rpm_margin": dict(limit="propulsion.rpm_fraction"),
    "post.I_pack_margin": dict(limit="propulsion.pack_I_max"),
    "oei.rudder": dict(limit="stability.rudder_max_deg"),
    "oei.speed": dict(limit="stability.oei_speed_factor"),
}
# config fields the tab edits through its on and robust columns (saved: they are config fields)
SWITCH_PATHS = ("require_3lap_energy", "robust.robust_constraints")

# config values that set a limit but are also model inputs: shown read-only here, edited where they are now
EDIT_ELSEWHERE = {
    "mu_roll": "Design and mission tab",
    "energy_reserve": "Placeholders tab",  # also the lap model's E_avail (laps_frac)
    "propulsion.v_top": "problem config (not in the UI)",
    "propulsion.f_fit": "problem config (not in the UI)",
    "course.takeoff.k_lof": "Placeholders tab",
    "robust.beta": "sidebar (RDO)",
    "mda_maxiter": "problem config (not in the UI)",
}
EDIT_ELSEWHERE_PREFIX = {"mass.inputs.": "Placeholders tab (or the mass config)"}
# limit fields of the schemas that no register entry reads (section 5 whenever they are set)
EXTRA_LIMIT_FIELDS = ("clmax_ratio_min",)
# lines the section-3 source scan finds that are not checks (tests/test_constraints_view.py)
SCAN_WAIVED = {
    ("aircrafter2/mass/statistical.py", "report-only column"): "the statistical mass estimate, not a check",
    ("aircrafter2/scoring/dbf2026.py", "Report only."): "the judges' floored score, not a check",
}


def edit_in(path):
    """Where a limit path is edited when not in the Constraints tab ('' = here)."""
    if path in EDIT_ELSEWHERE:
        return EDIT_ELSEWHERE[path]
    return next((v for k, v in EDIT_ELSEWHERE_PREFIX.items() if path.startswith(k)), "")


# --- source locations -------------------------------------------------------------------------------------------------

def code_lines(file, pattern):
    """['file:line', ...] of every line of file (repo-relative) matching the regex; raises if none matches."""
    hits = [f"{file}:{i}" for i, line in enumerate((ROOT / file).read_text().splitlines(), start=1)
            if re.search(pattern, line)]
    if not hits:
        raise LookupError(f"{file}: no line matches {pattern!r} (the code moved: update constraints_view.REGISTER)")
    return hits


def code_line(file, pattern):
    """'file:line' of the first line of file (repo-relative) matching the regex."""
    return code_lines(file, pattern)[0]


def _code_default(cls, path):
    """'code default (Class.field = value)' of a dotted path into a pydantic schema, or None."""
    parts = path.split(".")
    for i, k in enumerate(parts):
        f = cls.model_fields.get(k)
        if f is None:
            return None
        if i == len(parts) - 1:
            return f"code default ({cls.__name__}.{k} = {f.default!r})"
        ann = f.annotation
        if not (isinstance(ann, type) and hasattr(ann, "model_fields")):
            return None
        cls = ann
    return None


def limit_source(path, config_path, placeholders_path=None, ruleset_path=None):
    """Where the effective value of a limit path is set: dict(kind, file, line, status, Q, source, text).

    kind: 'problem config' (the config or its extends: chain), 'ruleset', 'placeholders', 'mass config', or
    'code default'. Reuses layer_files (set_in: the extends: chain; read_rows: the layer files)."""
    from .config.layer_files import read_rows, set_in, split_comment
    from .config.loader import load_yaml

    config_path = Path(config_path)
    base = config_path.parent

    def rel(f):
        f = Path(f).resolve()
        return str(f.relative_to(base.resolve())) if f.is_relative_to(base.resolve()) else f.name

    hit = set_in(config_path, path)
    if hit is not None:
        f, line = hit
        text = Path(f).read_text().splitlines()[line - 1]
        units, status, q, src = split_comment(text.split("#", 1)[1] if "#" in text else "")
        status = status or ("PLACEHOLDER" if "PLACEHOLDER" in text else "")
        return dict(kind="problem config", file=Path(f), line=line, status=status, Q=q, source=src,
                    text=f"{rel(f)}:{line}")
    for kind, layer in (("ruleset", ruleset_path), ("placeholders", placeholders_path)):
        if layer is None:
            continue
        row = next((r for r in read_rows(layer)[0] if r["parameter"] == path), None)
        if row is not None:
            return dict(kind=kind, file=Path(layer), line=row["line"], status=row["status"], Q=row["Q"],
                        source=row["source"], text=f"{rel(layer)}:{row['line']}")
    if path.startswith("mass."):
        mc = load_yaml(config_path).get("mass_config")
        if mc:
            hit = set_in(base / mc, path[len("mass."):])
            if hit is not None:
                return dict(kind="mass config", file=Path(hit[0]), line=hit[1], status="", Q="", source="",
                            text=f"{rel(hit[0])}:{hit[1]}")
    from .problems.full_mdo import FullMDOConfig
    from .problems.multipoint import MultipointSettings

    if path.startswith("multipoint."):
        text = _code_default(MultipointSettings, path[len("multipoint."):])
    else:
        text = _code_default(FullMDOConfig, path)
    return dict(kind="code default", file=None, line=None, status="", Q="", source="",
                text=text or "code default")


# --- the problem's context and values ----------------------------------------------------------------------------------

def _is_mp(cfg):
    from .problems.multipoint import MultipointConfig

    return isinstance(cfg, MultipointConfig)


def get_value(cfg, path):
    """The value of a dotted limit path in a FullMDOConfig or MultipointConfig (multipoint.* -> mcfg.mp)."""
    if _is_mp(cfg):
        if path.startswith("multipoint."):
            node, parts = cfg.mp, path.split(".")[1:]
        else:
            node, parts = cfg.base, path.split(".")
    else:
        node, parts = cfg, path.split(".")
    for k in parts:
        if node is None:
            return None
        node = node.get(k) if isinstance(node, dict) else getattr(node, k, None)
    return node


def problem_context(cfg, robust):
    base = cfg.base if _is_mp(cfg) else cfg
    from .problems.full_mdo import kv_is_dv

    return dict(multipoint=_is_mp(cfg), twin=base.propulsion.n_motors == 2, kv_dv=kv_is_dv(base),
                takeoff=base.course.takeoff.in_mission_time, robust=robust,
                pack_I_max=base.propulsion.pack_I_max is not None, fit_range=base.propulsion.fit_range,
                objective=base.robust.objective if robust else None)


def _point_cfg(cfg, point):
    if not _is_mp(cfg):
        return cfg
    return cfg.point_config(point) if point else cfg.base


# --- the set-up problem ---------------------------------------------------------------------------------------------

_CACHE = {}
CON_KEYS = ("lower", "upper", "equals", "ref", "units")


def _key(cfg, robust):
    return (type(cfg).__name__, cfg.model_dump_json(), bool(robust))


def setup_problem(cfg, robust=None, reduced=True):
    """The problem a run would build, set up (setup + final_setup; no run_model). robust None: the config's
    robust.enabled. reduced: RDO on a one-node rule (same driver constraints; module docstring)."""
    import openmdao.api as om

    mp = _is_mp(cfg)
    base = cfg.base if mp else cfg
    robust = base.robust.enabled if robust is None else robust
    c = cfg.model_copy(deep=True)
    cb = c.base if mp else c
    cb.robust.enabled = robust
    if robust and reduced:
        cb.robust.quadrature, cb.robust.active = "udr", []
    with contextlib.redirect_stdout(io.StringIO()):
        if robust and mp:
            from .robustness.rdo_multipoint import build_robust_multipoint

            prob = build_robust_multipoint(c)
        elif robust:
            from .robustness.rdo_problem import build_robust_problem

            prob = build_robust_problem(c)
        else:
            prob = om.Problem(reports=False)
            if mp:
                from .problems.multipoint import add_multipoint_driver, build_multipoint_model

                build_multipoint_model(prob.model, c)
                add_multipoint_driver(prob, c)
            else:
                from .problems.full_mdo import add_full_driver, build_full_model

                build_full_model(prob.model, c)
                add_full_driver(prob, c)
            prob.setup()
        prob.final_setup()
    return prob


def driver_constraints(cfg, robust=None, reduced=True):
    """[(driver name, dict(lower, upper, equals, ref, units))] of the set-up problem, in the driver's order (bounds
    raw; an absent bound None), and the model's output names. Cached per config and settings."""
    key = _key(cfg, robust) + (reduced,)
    if key not in _CACHE:
        prob = setup_problem(cfg, robust, reduced)
        cons = []
        for name, meta in prob.driver._cons.items():
            kw = {k: meta.get(k) for k in CON_KEYS}
            for k in ("lower", "upper"):
                if kw[k] is not None and abs(kw[k]) >= INF:
                    kw[k] = None
            cons.append((name, kw))
        outs = [m["prom_name"] for _, m in prob.model.list_outputs(prom_name=True, out_stream=None, val=False)]
        _CACHE[key] = (cons, outs)
    return _CACHE[key]


def parse_driver_name(name):
    """(register name, point, variant) of a driver constraint: variant '' (deterministic), 'n0' (RDO, deterministic
    at the reference node), 'lo' / 'hi' (RDO, mu -/+ beta sigma), 'sigma' (stats.score_sigma)."""
    variant = ""
    if name == "stats.score_sigma":
        return name, "", "sigma"
    if name.startswith("stats."):
        k, variant = name[len("stats."):].rsplit("_", 1)
        name = k.replace("__", ".")
    elif re.match(r"nodes\.n0\.", name):
        name, variant = name[len("nodes.n0."):], "n0"
    m = re.match(r"points\.(M\d)\.(.+)$", name)
    if m:
        return m.group(2), m.group(1), variant
    m = re.match(r"rules_(M\d)\.(.+)$", name)
    if m:
        return f"rules.{m.group(2)}", m.group(1), variant
    return name, "M2" if name.startswith("rules.") else "", variant


def _base_output(name):
    """An output's name without the RDO node and multipoint point prefixes, and its point."""
    name = re.sub(r"^nodes\.n\d+\.", "", name)
    m = re.match(r"points\.(M\d)\.(.+)$", name)
    return (m.group(2), m.group(1)) if m else (name, "")


# --- effective limits in physical units -------------------------------------------------------------------------------

def _prop_rpm_limit(c):
    from .propulsion.apc import INCH, load_prop

    try:
        data = load_prop(c.propulsion.prop, c.propulsion.apc_dir)
    except Exception:  # no APC data here: the formula without the number
        return None, None
    d_in = data.diameter / INCH
    return d_in, 150000.0 / d_in


def _g(x):
    return f"{x:g}" if isinstance(x, (int, float)) and not isinstance(x, bool) else str(x)


def effective_limit(name, c, mcfg=None, kw=None):
    """A readable limit in physical units for one register entry at point config c ('' if it needs a run)."""
    from .stores import LB

    st, pr = c.stability, c.propulsion
    rules = mcfg.mp.rules if mcfg is not None else None
    if name == "stall_margin":
        return "V >= 1.2 V_stall"
    if name == "turn_speed_margin":
        return "v after 360 deg >= V_stall"
    if name == "thrust_margin_static":
        return f"T(0) >= T_static (Eq. 3.3 at {_g(c.takeoff_distance)} m, mu_roll {_g(c.mu_roll)})"
    if name == "energy_margin":
        return f"E_usable >= E_3laps / (1 - {c.energy_reserve:.2f})"
    if name in ("rpm_margin", "post.rpm_margin"):
        d_in, lim = _prop_rpm_limit(c)
        if lim is None:
            return f"rpm(v) <= {_g(pr.rpm_fraction)} x 150000/D ({pr.prop})"
        return f"rpm(v) <= {_g(pr.rpm_fraction)} x 150000/{d_in:g} in = {pr.rpm_fraction * lim:,.0f} rpm ({pr.prop})"
    if name in ("I_margin", "post.I_margin"):
        return f"I_static <= {_g(pr.I_max)} A"
    if name in ("I_pack_margin", "post.I_pack_margin"):
        return f"I_pack <= {_g(pr.pack_I_max)} A"
    if name == "v_fit_margin":
        if pr.fit_range == "fixed":
            return f"v <= v_top = {_g(pr.v_top)} m/s"
        return f"v <= min(v_top = {_g(pr.v_top)} m/s, {_g(pr.f_fit)} J_max n0 D)"
    if name in ("SM", "V_h", "V_v"):
        lo, hi = getattr(st, f"{name}_min"), getattr(st, f"{name}_max")
        return f"{_g(lo)} <= {name} <= {_g(hi)}"
    if name == "wall_ratio":
        return f"t/r <= {_g(c.wall_ratio_max)}"
    if name == "r_tube":
        return f"r_tube >= {c.r_tube_min * 1e3:g} mm"
    if name == "cruise.cl_margin_sec":
        return "C_l <= C_l,max on every strip"
    if name in ("fus_clear_margin", "prop_clear_margin"):
        return f"gap >= c_clear = {pr.c_clear * 1e3:g} mm"
    if name == "tip_clear_margin":
        return "prop disc inside b/2"
    if rules is not None:
        if name == "rules.energy_margin_Wh":
            return f"E_nom <= {_g(rules.energy_total_max_Wh)} Wh"
        if name == "rules.fuse_margin":
            return f"I_pack <= {_g(rules.fuse_max_A)} A"
        if name == "rules.fuse_margin_C":
            return f"I_pack <= {_g(rules.c_rating)} C x capacity"
        if name == "rules.togw_margin":
            return f"TOGW < {_g(rules.togw_max_lb)} lb = {rules.togw_max_lb * LB:.3f} kg"
        if name == "laps_frac":
            return f"laps >= {_g(rules.laps_m2_min)}"
        if name == "rules.pax_margin":
            return f"n_p >= {_g(rules.pax_per_cargo_min)} n_c"
        if name == "rules.fuselage_margin":
            return f"l_fus <= {_g(rules.fuselage_span_ratio_max)} x b"
    if name == "stats.score_sigma":
        return f"sigma(score) <= {_g(c.robust.sigma_cap)}"
    if name == "m1.laps":
        return "laps >= 3 (M1: 3 laps in the window)"
    if name == "oei.speed":
        return f"V_check = {_g(st.oei_speed_factor)} V_stall"
    if name == "oei.rudder":
        return f"delta_rudder <= {_g(st.rudder_max_deg)} deg at {_g(st.oei_speed_factor)} V_stall"
    if name == "wind.lift_off":
        return f"headwind < {_g(c.course.takeoff.k_lof)} V_stall"
    if name == "balance.v":
        return f"5 <= v <= {_g(pr.v_top)} m/s"
    if name in ("balance.alpha_cruise",):
        return f"Newton, atol = rtol = 1e-10, maxiter {c.mda_maxiter}"
    if name == "mass.closure":
        return "NLBGS, atol = rtol = 1e-12, maxiter 50"
    return ""


def _robust_text(base_name, kw, beta, side):
    q = base_name.split(".")[-1]
    if side == "lo":
        return f"mu({q}) - {_g(beta)} sigma({q}) >= {_g(kw['lower'])}"
    return f"mu({q}) + {_g(beta)} sigma({q}) <= {_g(kw['upper'])}"


# --- the table --------------------------------------------------------------------------------------------------------

def _hard_text(e):
    return [f"hard-coded at {code_line(f, p)} ({what})" for f, p, what in e.hard_coded]


def _row(section, e, name, point, c, mcfg, paths, sources):
    srcs = [dict(path=p, edit_in=edit_in(p), **sources[p]) for p in paths]
    set_in_text = [f"{s['path']}: {s['text']}" + (f" (edit in: {s['edit_in']})" if s["edit_in"] else "")
                   for s in srcs] + _hard_text(e)
    first = next((s for s in srcs if not s["edit_in"]), srcs[0] if srcs else None)
    return dict(section=section, group=e.group, name=name, constraint=e.name, point=point, label=e.label,
                definition=e.definition, lower=None, upper=None, equals=None, ref=None, con_units=None, units=e.units,
                effective=effective_limit(e.name, c, mcfg), set_in="; ".join(set_in_text), sources=srcs,
                limit_paths=list(paths), editable=[s["path"] for s in srcs if not s["edit_in"]],
                status=first["status"] if first else "", Q=first["Q"] if first else "", kind=e.kind,
                robust=False, variant="", note=e.note, cells=row_cells(e.name), limit_field=None, limit=None)


def constraint_table(cfg, robust=None, config_path=None, placeholders_path=None, ruleset_path=None, reduced=True):
    """Rows of the five sections (module docstring) for a FullMDOConfig or MultipointConfig.

    robust        None: the config's robust.enabled; True / False: RDO on / off (the UI's switch)
    config_path   the problem config (its extends: chain); placeholders_path / ruleset_path: the layer files the
                  run uses (default: the ones the config names). Without config_path the 'set in' columns are empty.
    Raises UnregisteredConstraint for a driver constraint or an unconstrained margin output with no register entry.
    """
    from .config.layer_files import layer_files_of

    mp = _is_mp(cfg)
    base = cfg.base if mp else cfg
    robust = base.robust.enabled if robust is None else robust
    mcfg = cfg if mp else None
    if config_path is not None and placeholders_path is None and ruleset_path is None:
        placeholders_path, ruleset_path = layer_files_of(config_path)
    cons, outs = driver_constraints(cfg, robust, reduced)
    ctx = problem_context(cfg, robust)

    src_cache = {}

    def sources(paths):
        for p in paths:
            if p not in src_cache:
                src_cache[p] = (limit_source(p, config_path, placeholders_path, ruleset_path) if config_path
                                else dict(kind="", file=None, line=None, status="", Q="", source="", text=""))
        return src_cache

    rows = []
    for name, kw in cons:
        reg, point, variant = parse_driver_name(name)
        e = REGISTER.get(reg)
        if e is None or e.section != 1:
            raise UnregisteredConstraint(f"driver constraint {name!r} ({reg!r}) has no entry in "
                                         "aircrafter2/constraints_view.REGISTER: add one")
        c = _point_cfg(cfg, point)
        paths = list(e.limit_paths)
        if variant in ("lo", "hi"):
            paths.append("robust.beta")
        row = _row(1, e, name, point, c, mcfg, paths, sources(paths))
        row.update({k: kw[k] for k in ("lower", "upper", "equals", "ref")}, con_units=kw["units"],
                   units=kw["units"] or e.units, variant=variant,
                   robust=variant in ("lo", "hi"))
        if row["robust"]:
            row["effective"] = _robust_text(reg, kw, base.robust.beta, variant) + (
                f"  [{row['effective']}]" if row["effective"] else "")
        row["cells"] = row_cells(reg, variant)
        rows.append(row)

    # sections 2-4
    out_base = {}
    for o in outs:
        b, p = _base_output(o)
        if not b.startswith("stats."):
            out_base.setdefault(b, set()).add(p)
    for e in _POST + _ANALYSIS + _MDA:
        if e.outputs:
            points = sorted(set().union(*(out_base.get(o, set()) for o in e.outputs)))
            if not points or (e.applies is not None and not e.applies(ctx)):
                continue
        elif e.applies is not None and not e.applies(ctx):
            continue
        else:
            points = [""]
        if e.section == 4 and mp:
            points = ["M2"] if e.name == "mass.closure" else ["M2", "M3"]
        for p in points:
            rows.append(_row(e.section, e, e.name, p, _point_cfg(cfg, p), mcfg, list(e.limit_paths),
                             sources(e.limit_paths)))

    # unconstrained margin outputs must be accounted for
    constrained = {parse_driver_name(n)[0] for n, _ in cons}
    covered = {o for e in _ANALYSIS for o in e.outputs}
    for b in out_base:
        reg = parse_driver_name(b)[0]
        if "margin" in b and reg not in constrained and b not in covered and reg not in REGISTER:
            raise UnregisteredConstraint(f"model output {b!r} looks like a check (a margin) but is neither a driver "
                                         "constraint nor in aircrafter2/constraints_view.REGISTER: add an entry")

    # section 5: limit fields no row reads
    used = {p for r in rows for p in r["limit_paths"]}
    used |= {p for p, _ in dv_bound_rules(cfg)}
    if robust:
        used.add("robust.beta")
    for path in all_limit_fields(mp):
        if path in used:
            continue
        val = get_value(cfg, path)
        src = sources([path])[path]
        rows.append(dict(section=5, group="", name=path, constraint=path, point="", label=path,
                         definition="not used by this problem", lower=None, upper=None, equals=None, ref=None,
                         con_units=None, units="", effective="off (None)" if val is None else _g(val),
                         set_in=f"{path}: {src['text']}", sources=[dict(path=path, edit_in=edit_in(path), **src)],
                         limit_paths=[path], editable=[], status=src["status"], Q=src["Q"], kind="",
                         robust=False, variant="", note="", cells={}, limit_field=None, limit=None))
    for r in rows:
        if not mp:  # e.g. a single-point plug-in's laps_frac bound is the plug-in's own, not multipoint.rules
            r["cells"] = {k: v for k, v in r["cells"].items() if not v.startswith("multipoint.")}
        if r["cells"].get("limit"):
            r["limit_field"] = r["cells"]["limit"]
            r["limit"] = get_value(cfg, r["limit_field"])
    return rows


def all_limit_fields(multipoint):
    """Every limit path the register reads, plus the schema's other limit fields (EXTRA_LIMIT_FIELDS)."""
    paths = {p for e in REGISTER.values() for p in e.limit_paths} | set(EXTRA_LIMIT_FIELDS)
    paths |= {"robust.beta", "multipoint.rules.span_max_ft"}
    if not multipoint:
        paths = {p for p in paths if not p.startswith("multipoint.")}
    return sorted(paths)


def dv_bound_rules(cfg):
    """[(limit path or '', text)]: the rules enforced as design-variable bounds (not rows; the tab's caption)."""
    if not _is_mp(cfg):
        return []
    r, mp = cfg.mp.rules, cfg.mp
    out = [("multipoint.rules.span_max_ft", f"span <= {_g(r.span_max_ft)} ft: the upper bound of b "
                                             f"(min with design_variables.b.upper)")]
    if mp.kind == "dbf2026":
        b = cfg.base.design_variables.get("b")
        if b is not None:
            out.append(("", f"RAC >= 0.90: b >= {b.lower:g} m (design_variables.b.lower)"))
        for k, what in (("n_p", "passengers"), ("n_c", "cargo"), ("L_banner", "banner length (in)")):
            if k in mp.payload:
                out.append(("", f"{what} >= {_g(mp.payload[k].lower)} (multipoint.payload.{k}.lower)"))
    return out


SECTIONS = {1: "Optimiser constraints", 2: "Post-checks", 3: "Analysis limits and report-only checks",
            4: "MDA balances", 5: "Limit fields this problem doesn't use"}


def format_table(rows, sections=(1, 2, 3, 4, 5)):
    """Plain-text table: section, constraint, point, lower, upper, effective limit, set in, status."""
    def f(x):
        return "" if x is None else (f"{x:g}" if isinstance(x, float) else str(x))

    lines = []
    for s in sections:
        sec = [r for r in rows if r["section"] == s]
        lines.append(f"\n## {s}. {SECTIONS[s]} ({len(sec)})")
        for r in sec:
            name = r["name"] if s == 1 else r["constraint"]
            lines.append(f"  {name:40s} {r['point']:3s} lo={f(r['lower']):9s} hi={f(r['upper']):9s} "
                         f"| {r['effective'] or r['definition']} | {r['set_in'] or '-'} | {r['status']}")
    return "\n".join(lines)


# --- the tab: editable cells, edits, validation, saving ----------------------------------------------------------------

def row_cells(name, variant=""):
    """{'lower' | 'upper' | 'limit': config path} of a row's editable cells (robust lo / hi rows: their own bound)."""
    cells = {k: v for k, v in CELLS.get(name, {}).items() if not edit_in(v)}
    if variant == "lo":
        cells.pop("upper", None)
    elif variant == "hi":
        cells.pop("lower", None)
    return cells


def tab_paths():
    """Every config path the Constraints tab edits (the layer editors leave these rows to it)."""
    return {p for c in CELLS.values() for p in c.values() if not edit_in(p)} | set(SWITCH_PATHS)


def set_value(cfg, path, value):
    """Set a dotted limit path on a FullMDOConfig or MultipointConfig in place (multipoint.* -> mcfg.mp)."""
    if _is_mp(cfg):
        node, parts = (cfg.mp, path.split(".")[1:]) if path.startswith("multipoint.") else (cfg.base, path.split("."))
    else:
        node, parts = cfg, path.split(".")
    for k in parts[:-1]:
        node = node[k] if isinstance(node, dict) else getattr(node, k)
    if isinstance(node, dict):
        node[parts[-1]] = value
    else:
        setattr(node, parts[-1], value)


def config_off(cfg):
    """The constraints a single-point config switches off itself: disabled_constraints, energy_margin when
    require_3lap_energy is false."""
    return set(cfg.disabled_constraints) | (set() if cfg.require_3lap_energy else {"energy_margin"})


def apply_limits(cfg, edits=None, off=None, robust=None):
    """A copy of cfg with the tab's choices: edits {path: value}; off (single-point: the constraint names switched
    off; energy_margin is require_3lap_energy, the others disabled_constraints; None = unchanged); robust
    (robust_constraints; None = unchanged). With no edits, off = config_off(cfg) and robust None, the copy equals cfg."""
    c = cfg.model_copy(deep=True)
    for path, val in (edits or {}).items():
        set_value(c, path, val)
    if off is not None:
        if _is_mp(c):
            raise ValueError("constraints cannot be switched off in a multipoint config (the UI does not run them)")
        off = set(off)
        c.require_3lap_energy = "energy_margin" not in off
        c.disabled_constraints = sorted(off - {"energy_margin"})
    if robust is not None:
        (c.base if _is_mp(c) else c).robust.robust_constraints = list(robust)
    return c


FRACTIONS = ("propulsion.rpm_fraction", "wall_ratio_max")
NONNEGATIVE = ("multipoint.rules.laps_m2_min", "multipoint.rules.pax_per_cargo_min", "propulsion.c_clear")
PAIRS = [("stability.SM_min", "stability.SM_max"), ("stability.V_h_min", "stability.V_h_max"),
         ("stability.V_v_min", "stability.V_v_max")]


def validate_limits(cfg, paths=None):
    """Messages for invalid limit values of cfg (empty = valid): lower <= upper, fractions in (0, 1], positive
    physical limits (non-negative where 0 means 'none'). paths: the ones to check (default: every tab path)."""
    errs = []
    paths = sorted(tab_paths() - set(SWITCH_PATHS)) if paths is None else paths
    for path in paths:
        if path.startswith("multipoint.") and not _is_mp(cfg):
            continue
        v = get_value(cfg, path)
        if v is None:
            continue
        if not isinstance(v, (int, float)) or isinstance(v, bool) or v != v:
            errs.append(f"{path} must be a number (got {v!r}).")
        elif path in FRACTIONS and not 0.0 < v <= 1.0:
            errs.append(f"{path} is a fraction: it must lie in (0, 1] (got {v:g}).")
        elif path in NONNEGATIVE and v < 0.0:
            errs.append(f"{path} must be >= 0 (got {v:g}).")
        elif path not in FRACTIONS + NONNEGATIVE and v <= 0.0:
            errs.append(f"{path} must be > 0 (got {v:g}).")
    for lo, hi in PAIRS:
        a, b = get_value(cfg, lo), get_value(cfg, hi)
        if isinstance(a, (int, float)) and isinstance(b, (int, float)) and not a <= b:
            errs.append(f"{lo} ({a:g}) must not exceed {hi} ({b:g}).")
    return errs


def save_limits(config_path, values, comments=None, placeholders_path=None, ruleset_path=None):
    """Write limit values back to the file and line that sets each one's effective value: layer files with
    layer_files.save_rows (comment fields from `comments` {path: dict(status, Q, source)}), problem and mass configs
    in place with layer_files.save_config_value (comment kept). values: {path: value} (only changed ones).
    Returns dict(written={file: [lines]}, errors=[...]); a path set nowhere but in the code is an error."""
    from .config.layer_files import layer_files_of, read_rows, save_config_value, save_rows

    if placeholders_path is None and ruleset_path is None:
        placeholders_path, ruleset_path = layer_files_of(config_path)
    comments = comments or {}
    layer_rows, written, errors = {}, {}, []
    for path in sorted(set(values) | set(comments)):
        src = limit_source(path, config_path, placeholders_path, ruleset_path)
        if src["kind"] in ("placeholders", "ruleset"):
            f = src["file"]
            if f not in layer_rows:
                layer_rows[f] = ({r["line"]: r for r in read_rows(f)[0]}, set())
            rows, touched = layer_rows[f]
            row = rows[src["line"]]
            if path in values:
                row["value"] = values[path]
            row.update({k: v for k, v in comments.get(path, {}).items() if k in ("status", "Q", "source")})
            touched.add(src["line"])
        elif src["kind"] in ("problem config", "mass config"):
            if path in values:
                save_config_value(src["file"], src["line"], path.split(".")[-1], values[path])
                written.setdefault(src["file"], []).append(src["line"])
        else:
            errors.append(f"{path} is a {src['text']}: no file sets it, so it cannot be saved in place (add it "
                          f"to {Path(config_path).name} first)")
    for f, (rows, touched) in layer_rows.items():
        lines = save_rows(f, [rows[i] for i in sorted(touched)])
        if lines:
            written.setdefault(f, []).extend(lines)
    return dict(written=written, errors=errors)


# --- which study scripts read a file ----------------------------------------------------------------------------------

def _config_closure(f):
    """The files a load of config f reads: f, its extends: chain, its layer files, mass_config, uncertainty_config."""
    from .config.layer_files import _chain, layer_files_of
    from .config.loader import load_yaml

    out = set()
    try:
        chain = _chain(f)
    except Exception:  # not a config (a list, a course): itself only
        return {Path(f).resolve()}
    out |= {Path(c).resolve() for c in chain}
    try:
        out |= {Path(x).resolve() for x in layer_files_of(f) if x}
        data = load_yaml(f)
    except Exception:
        return out
    for key in ("mass_config",):
        if data.get(key):
            out.add((Path(f).parent / data[key]).resolve())
    unc = (data.get("robust") or {}).get("uncertainty_config")
    if unc:
        out.add((Path(f).parent / unc).resolve())
    return out


def study_readers(configs_dir, script_dirs=(ROOT / "studies", ROOT / "verification")):
    """{file (resolved): [script, ...]}: the files each study / verification script reads through the configs it
    names. Found by parsing the scripts (no import): string literals that are a .yaml file name, through the study
    modules each script imports, each config expanded to its extends: chain, layer files and the files it names."""
    import ast

    configs_dir = Path(configs_dir)
    scripts = {}
    for d in script_dirs:
        for f in sorted(Path(d).glob("*.py")):
            scripts.setdefault(f.stem, f)
    names, imports = {}, {}
    for stem, f in scripts.items():
        tree = ast.parse(f.read_text())
        names[stem] = {n.value for n in ast.walk(tree) if isinstance(n, ast.Constant) and isinstance(n.value, str)
                       and re.fullmatch(r"[\w./-]+\.ya?ml", n.value)}
        imp = set()
        for n in ast.walk(tree):
            if isinstance(n, ast.Import):
                imp |= {a.name.split(".")[-1] for a in n.names}
            elif isinstance(n, ast.ImportFrom) and n.module:
                imp.add(n.module.split(".")[-1])
        imports[stem] = imp & set(scripts)

    def all_names(stem, seen):
        seen.add(stem)
        out = set(names[stem])
        for m in imports[stem] - seen:
            out |= all_names(m, seen)
        return out

    readers = {}
    for stem, f in scripts.items():
        label = f"{f.parent.name}/{f.name}"
        for name in all_names(stem, set()):
            rel = name[len("configs/"):] if name.startswith("configs/") else name
            target = configs_dir / rel
            if not target.is_file():
                continue
            for g in _config_closure(target):
                readers.setdefault(g, set()).add(label)
    return {k: sorted(v) for k, v in readers.items()}


# --- run values -------------------------------------------------------------------------------------------------------

ACTIVE_TOL = 1e-4  # |margin| / ref below this: active (api.motor_report's tolerance)


def margin_of(value, lower=None, upper=None, equals=None):
    """Signed distance to the nearest bound (< 0: outside)."""
    m = []
    if lower is not None:
        m.append(value - lower)
    if upper is not None:
        m.append(upper - value)
    if equals is not None:
        m.append(-abs(value - equals))
    return min(m) if m else None


def run_states(rows, constraints_json, results_json=None):
    """Fill value, margin and state (active / inactive / violated) of the optimiser rows from a run's
    constraints.json (by driver name), and of the post-check rows from its results.json. In place; returns rows."""
    cj = constraints_json or {}
    tol = cj.get("tol", 1e-8)
    by = {c["name"]: c for c in cj.get("constraints", [])}
    post = {}
    if results_json:
        lim, oei = results_json.get("limits") or {}, results_json.get("oei") or {}
        post = {"post.I_margin": lim.get("I_margin_A"), "post.rpm_margin": lim.get("rpm_margin"),
                "post.I_pack_margin": lim.get("I_pack_margin_A"), "oei.rudder": oei.get("rudder_margin_deg")}
    for r in rows:
        r.update(value=None, margin=None, state="")
        if r["section"] == 1 and r["name"] in by:
            c = by[r["name"]]
            m = margin_of(c["value"], c["lower"], c["upper"], c["equals"])
            scaled = m / (c["ref"] or 1.0)
            r.update(value=c["value"], margin=m,
                     state="violated" if scaled < -tol else "active" if abs(scaled) < ACTIVE_TOL else "inactive")
        elif r["section"] == 2 and post.get(r["constraint"]) is not None:
            m = post[r["constraint"]] + (1.0 if r["constraint"] == "post.rpm_margin" else 0.0)  # -1 rpm tolerance
            r.update(value=post[r["constraint"]], margin=m, state="violated" if m < 0 else "ok")
    return rows


# --- the tab's table editor -------------------------------------------------------------------------------------------

OPTIONAL = ("multipoint.rules.laps_m2_min", "propulsion.pack_I_max", "robust.sigma_cap")  # None = no limit


def _num(x):
    return None if x is None or (isinstance(x, float) and x != x) else x


def editor_changes(rows, before, after, edits, off, robust):
    """What one section's table edit means: (edits {path: value}, off, robust, ignored messages). rows: the
    section's rows in table order; before / after: the frame given to st.data_editor and the one it returned.
    A lower / upper / limit cell edits the config path in row['cells']; a cell with no config value behind it is
    ignored (with a message), as is emptying a value that cannot be None."""
    edits, off, robust, ignored = dict(edits), set(off), list(robust), []
    for i, r in enumerate(rows):
        for col in ("lower", "upper", "limit"):
            if col not in before:
                continue
            old, new = _num(before.at[i, col]), _num(after.at[i, col])
            if old == new:
                continue
            path = r["cells"].get(col)
            name = r["name"] if r["section"] == 1 else r["constraint"]
            if path is None:
                ignored.append(f"{name}: its {col} is not a config value ({r['set_in'] or 'fixed'})")
            elif new is None and path not in OPTIONAL:
                ignored.append(f"{path} cannot be empty")
            else:
                edits[path] = None if new is None else float(new)
        if "on" in before and bool(after.at[i, "on"]) != bool(before.at[i, "on"]):
            (off.discard if after.at[i, "on"] else off.add)(r["constraint"])
        if "robust" in before and bool(after.at[i, "robust"]) != bool(before.at[i, "robust"]):
            robust = robust + [r["constraint"]] if after.at[i, "robust"] else [n for n in robust if n != r["constraint"]]
    return edits, off, list(dict.fromkeys(robust)), ignored
