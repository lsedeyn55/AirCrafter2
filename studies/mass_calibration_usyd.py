"""Mass-model calibration on three USYD aircraft: Orange Hornet, Woombah, Humpback (docs/MASS_CALIBRATION_PLAN.md).

    .venv/bin/python studies/mass_calibration_usyd.py fit      # fits -> outputs/results/mass_calibration_usyd_fit.json,
                                                               # configs/placeholders.yaml values (saved in place),
                                                               # configs/mass_usyd_loo_woombah.yaml
    .venv/bin/python studies/mass_calibration_usyd.py report   # docs/studies/mass_calibration_usyd.{md,json}

The published component masses are converted here from the report values (g, lb), never by hand. Each case is the
G&G mass group at the aircraft's sizing mass, with no closure (external sizing), on the optimiser's geometry
definitions (rectangular-equivalent chord, l_tb = l_ht - boom_offset, S_cb = 2(lw + lh + wh)), with section
coefficients from the airfoil module and Cm, CL_alpha from the wing aero module at the sizing weight. Eq. 127/128 as
relative residuals with equal weights (aircrafter2/mass/calibration.py). Nothing is tuned to these numbers.

Sources (printed pages): [OH] 2024 USYD DBF Design Report (Orange Hornet); [W] 2025 USYD DBF Design Report (Project
Woombah); [HB] 2026 USYD DBF Design Report (Project Humpback); [GG] Garcia & Gamboa (2022).
"""

import os

os.environ.setdefault("OMP_NUM_THREADS", "3")

import argparse  # noqa: E402
import json  # noqa: E402
import math  # noqa: E402
import subprocess  # noqa: E402
import warnings  # noqa: E402
from pathlib import Path  # noqa: E402

import numpy as np  # noqa: E402

warnings.filterwarnings("ignore")
ROOT = Path(__file__).resolve().parents[1]
CONFIGS = ROOT / "configs"
OUT = ROOT / "outputs" / "results"
DOCS = ROOT / "docs" / "studies"
FIT_JSON = OUT / "mass_calibration_usyd_fit.json"
G, LB, IN, FT, MPH, KMH = 1e-3, 0.45359237, 0.0254, 0.3048, 0.44704, 1 / 3.6

# --- the published weight-and-balance rows (value, report unit) -----------------------------------------------------
# target -> [(row as printed, value)]; the unit is the table's.
REPORTS = {
    "OH": dict(name="Orange Hornet", year="2023-24", unit=G, unit_name="g", table="Table 35, p. 42", rows={
        "m_wing": [("Wing", 1517.696)],
        "m_ht": [("Horizontal Stabaliser", 203.763)],
        "m_vt": [("Vertical Stabliser", 90.203)],
        "m_cb": [("Fuselage", 903.552), ("Nosecone (inc. crew restraint)", 356.884)],
        "m_tb": [("Empenage (the rear fuselage)", 231.598)],
        "m_gear": [("Undercarriage", 366.567)],
    }, systems=[("Servos", 290.2), ("Flight Battery", 594.0), ("ESC", 258.0), ("Reciever", 16.0),
                ("Recciever Battery", 132.0), ("Motor (inc. shaft, nut and washer)", 710.005), ("Propeller", 90.0)],
        payload_m2=[("Medical Supply Cabinet", 2912.087), ("Crew", 77.9), ("EMTs, Patient, gurney", 396.067)],
        m2_total=9146.522),
    "W": dict(name="Project Woombah", year="2024-25", unit=LB, unit_name="lb", table="Table 34, p. 45", rows={
        "m_wing": [("Wing", 3.41)],
        "m_ht": [("Horizontal Stabilizer", 0.41)],
        "m_vt": [("Vertical Stabilizer", 0.19)],
        "m_fus": [("Fuselage", 1.23), ("Fuselage nose", 0.15)],
        "m_gear": [("Undercarriage", 0.66)],
    }, systems=[("ESC", 0.56), ("Receiver", 0.19), ("Receiver Battery", 0.30),
                ("Motor (inc. shaft, nut, & washer) x2", 0.50), ("Propeller x2", 0.08), ("Battery x2 (M2)", 1.64)],
        payload_m2=[("Payload Structure", 0.44), ("Bottle x2", 13.5)], m2_total=26.02),
    "HB": dict(name="Project Humpback", year="2025-26", unit=LB, unit_name="lb", table="Table 36, p. 44", rows={
        "m_wing": [("Port Wing", 0.79), ("Starboard Wing", 0.79)],
        "m_ht": [("Port Horizontal Stab", 0.21), ("Starboard Horizontal Stab", 0.21)],
        "m_vt": [("Vertical Stab", 0.28)],
    }, systems=None, payload_m2=None, m2_total=18.54),
}

# --- the case inputs (SI after conversion) and their sources --------------------------------------------------------
CASES = {
    "OH": dict(b=(1.524, "Table 33, p. 32"), c=(0.435, "Table 33 (rectangular)"), S=(0.66294, "Table 33"),
               seed=("clarky", "p. 20"), n_panels=(1, "one-piece swivel wing, step-downs epoxied in (pp. 21, 34, 49)"),
               S_ht=(0.1909, "Table 24, p. 22"), S_vt=(0.0729, "Table 24"), l_ht=(1.00, "Table 24"),
               cb=((1.050, 0.167, 0.156), "constant-section midsection (pp. 19, 38-39)"),
               m_size=(8.0, "MTOW (M2), Table 36, p. 43"), V=(140 * KMH, "140 km/h cruise, Table 36"),
               alt=(400.0, "400 m (p. 25)"),
               # one unit per nacelle: motor + ESC + prop (Table 35 rows / 2); the battery in the nacelle stays at the
               # fuselage in the model (OPEN_QUESTIONS Q27)
               units=dict(m=((710.005 + 258.0 + 90.0) / 2 * G, "Table 35 (motor + ESC + prop) / 2"),
                          x=(-0.130, "130 mm ahead of the LE (p. 20)"),
                          y=(0.335, "PLACEHOLDER: not dimensioned; scaled from the three-view (p. 44)"))),
    "W": dict(b=(70.9 * IN, "70.9 in, Table 30, p. 31"), c=(914.5 * IN**2 / (70.9 * IN), "S/b (tapered 0.8: "
                                                                                    "modelling difference)"),
              S=(914.5 * IN**2, "914.5 in^2, Table 30"), seed=("clarky", "p. 20"), n_panels=(3, "p. 20"),
              S_ht=(0.1525, "236.4 in^2, Table 30 (woombah_2025.yaml)"), S_vt=(0.0670, "103.85 in^2, Table 30"),
              l_ht=(1.100, "3.61 ft, Table 21"), cb=((0.820, 0.1194, 0.1194), "nose + centre sections, "
                                                                            "1.64 + 1.05 ft x 4.7 in (Fig. 11, p. 19)"),
              m_size=(26.02 * LB, "Table 34 M2 (Table 29 design MTOW 26 lb)"),
              V=(87 * MPH, "87 mph M2 design, Table 29"), alt=(800.0, "Tucson 800 m (ruleset)"),
              units=dict(m=((0.50 + 0.56 + 0.08) / 2 * LB, "Table 34 (motor + ESC + prop) / 2"),
                         x=(-0.1397, "5.5 in ahead of the LE (p. 20)"), y=(9.9 * IN, "9.9 in (p. 32)"))),
    "HB": dict(b=(59 * IN, "59 in, p. 36"), c=(14.57 * IN, "14.57 in (rectangular), p. 36"),
               S=(868 * IN**2, "868 in^2, p. 36"), seed=("s3016", "Table 24, p. 26 (p. 36 prints 'SD3016')"),
               n_panels=(2, "'modular two-piece', pp. 26, 37"), S_ht=(230 * IN**2, "230 in^2, p. 36"),
               S_vt=(90.9 * IN**2, "90.9 in^2, p. 36"), l_ht=(1.1, "p. 26"), cb=None,
               m_size=(14.3 * LB, "MTOW of the V-n diagram, p. 36"), V=(100 * FT, "V_C = 100 ft/s, p. 36"),
               alt=(1300 * FT, "Wichita 1300 ft (p. 12)"), units=None),
}
ORDER = ["OH", "W", "HB"]
# Previous values of the Step 1 inputs (configs/placeholders.yaml before this calibration: G&G Table 13 and the 0.5 mm
# gauge placeholder)
BEFORE_STEP1 = {"t_tube_min": 5.0e-4, "sigma_spar": 7.2e8}
AREAL = ["k_wing", "a_ht", "a_vt", "f_cb", "f_tb", "f_gear"]
GG = ["k_wing", "f_ht", "f_vt", "f_cb", "f_tb", "f_gear"]
X0 = {"k_wing": 2.0, "a_ht": 1.0, "a_vt": 1.0, "f_ht": 0.6, "f_vt": 0.6, "f_cb": 1.0, "f_tb": 3.0, "f_gear": 0.03}
# the Woombah wing-unit options of the 2024-25 problem (configs/full_mdo_twin.yaml)
TWIN_OPTIONS = dict(motor_inertia_relief=False, nacelle_torsion=True)


def git_hash():
    return subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=ROOT, capture_output=True,
                          text=True).stdout.strip()


def real_masses(ac):
    r = REPORTS[ac]
    return {k: sum(v for _, v in rows) * r["unit"] for k, rows in r["rows"].items()}


def targets(ac):
    """Target keys for the calibration hook (m_cb and m_tb of [OH] separately; [W]'s fuselage lumped as m_fus)."""
    return real_masses(ac)


def gg_factors():
    """The published G&G factors (configs/mass_garcia_gamboa_2022.yaml: Table 14, f_tb refitted for the 1 mm boom)."""
    from aircrafter2.config import load_yaml

    inp = load_yaml(CONFIGS / "mass_garcia_gamboa_2022.yaml")["inputs"]
    return {k: inp[k] for k in ("f_wing", "f_ht", "f_vt", "f_cb", "f_tb", "f_gear")} | {"k_wing": 1.0}


def section(ac):
    from aircrafter2.airfoil import cst
    from aircrafter2.airfoil.seeds import load_seed

    s = load_seed(CASES[ac]["seed"][0])
    g = cst.section_geometry(s["upper_weights"], s["lower_weights"], s["leading_edge_weight"], s["TE_thickness"],
                             0.25)
    return {k: float(v) for k, v in g.items()}


def wing_aero(ac):
    """(alpha deg, Cm_wing, CL_alpha) of the wing at V_design and the site density, at L = sizing weight (1 g):
    the aero module the optimiser feeds the mass model with (cruise.Cm_wing, cruise.CL_alpha)."""
    import openmdao.api as om
    from scipy.optimize import brentq

    from aircrafter2.aero.wing import WingAero
    from aircrafter2.airfoil.seeds import load_seed
    from aircrafter2.atmosphere import isa
    from aircrafter2.config.aero import AeroConfig

    d = CASES[ac]
    a = AeroConfig(num_x=2, num_y=21)  # the 2024-25 problem's mesh (configs/full_mdo_twin.yaml aero)
    p = om.Problem(reports=False)
    p.model.add_subsystem("w", WingAero(num_x=a.num_x, num_y=a.num_y, spacing=a.spacing, nf=a.nf_settings()),
                          promotes=["*"])
    p.setup()
    s = load_seed(d["seed"][0])
    for k, v in [("upper_weights", s["upper_weights"]), ("lower_weights", s["lower_weights"]),
                 ("le_weight", s["leading_edge_weight"]), ("te_thickness", s["TE_thickness"])]:
        p.set_val(k, v)
    atm = isa(d["alt"][0])
    p.set_val("b", d["b"][0])
    p.set_val("c", d["S"][0] / d["b"][0])
    p.set_val("v", d["V"][0])
    p.set_val("rho", float(atm["rho"]))
    p.set_val("mu", float(atm["mu"]))
    W = d["m_size"][0] * 9.80665

    def lift(al):
        p.set_val("alpha", al, units="deg")
        p.run_model()
        return float(p.get_val("L")[0]) - W

    al = brentq(lift, -4.0, 12.0, xtol=1e-7)
    lift(al)
    return dict(alpha=al, Cm=float(p.get_val("Cm_wing")[0]), CL_alpha=float(p.get_val("CL_alpha")[0]))


def base_config(tail="areal"):
    """MassConfig: mass_garcia_gamboa_2022.yaml under configs/placeholders.yaml mass: (the layer order of every problem
    config), with the Step 1 physical values; the factors are set by the caller."""
    from aircrafter2.config import MassConfig, load_yaml
    from aircrafter2.config.loader import _merge, load_layer

    data = _merge(load_yaml(CONFIGS / "mass_garcia_gamboa_2022.yaml"), load_layer(CONFIGS / "placeholders.yaml")["mass"])
    data["options"]["tail_mass_model"] = tail
    return MassConfig(**data)


def make_case(ac, aero, tail="areal", inputs=None, sizing_mass=None, closure=None, y_motor=None, relief=False):
    """CalibrationCase of one aircraft. closure: dict(m_sys, m_pay) runs the mass closure instead (report only)."""
    from aircrafter2.atmosphere import isa
    from aircrafter2.config.loader import load_layer
    from aircrafter2.mass.calibration import CalibrationCase

    d, sec = CASES[ac], section(ac)
    cfg = base_config(tail)
    twin = d["units"] is not None
    cfg.options.n_panels = d["n_panels"][0]
    cfg.options.n_motor_pairs = 1 if twin else 0
    cfg.options.motor_inertia_relief = relief or TWIN_OPTIONS["motor_inertia_relief"]
    cfg.options.nacelle_torsion = TWIN_OPTIONS["nacelle_torsion"]
    boom_offset = load_layer(CONFIGS / "placeholders.yaml")["geometry"]["boom_offset"]
    c = d["S"][0] / d["b"][0] if ac == "W" else d["c"][0]
    l, w, h = d["cb"][0] if d["cb"] else (0.4, 0.12, 0.12)  # [HB]: no fuselage target, any box
    cfg.inputs.update(b=d["b"][0], c_mean=c, c_root=c, S_wing=d["S"][0], tc=sec["tc_max"], t_at_spar=sec["t_at_spar"],
                      A_c2=sec["A_c2"], P_c=sec["P_c"], n=5.0, SF=1.5, V_design=d["V"][0],
                      rho=float(isa(d["alt"][0])["rho"]), Cm=aero["Cm"], CL_alpha=aero["CL_alpha"],
                      S_ht=d["S_ht"][0], S_vt=d["S_vt"][0], l_ht=d["l_ht"][0], l_tb=d["l_ht"][0] - boom_offset,
                      S_cb=2 * (l * w + l * h + w * h), m_sys=0.0, m_pay=0.0, m_ballast=0.0)
    if closure:
        cfg.inputs.update(m_sys=closure["m_sys"], m_pay=closure["m_pay"])
    cfg.inputs.update(inputs or {})
    values = {}
    if twin:
        u = d["units"]
        values = {"m_motor_unit": (u["m"][0], "kg"), "x_motor": (u["x"][0], "m"),
                  "y_m": (y_motor if y_motor is not None else u["y"][0], "m")}
    return CalibrationCase(name=ac, config=cfg, targets=targets(ac), values=values,
                           sizing_mass=None if closure else (sizing_mass or d["m_size"][0]))


def identified(names, form):
    """(factors identified by these aircraft, {factor: value held}): the wing and tail factors always; f_cb and f_tb
    need [OH]'s separate cargo-box and rear-fuselage rows (with [W] alone only their sum is known: f_tb is held at
    the G&G 1 mm-boom value and f_cb fitted); f_gear needs [OH] or [W]."""
    tails = ["a_ht", "a_vt"] if form == "areal" else ["f_ht", "f_vt"]
    fac, held = ["k_wing"] + tails, {}
    gg = gg_factors()
    if "OH" in names:
        fac += ["f_cb", "f_tb"]
    elif "W" in names:
        fac += ["f_cb"]
        held["f_tb"] = gg["f_tb"]
    else:
        held.update(f_cb=gg["f_cb"], f_tb=gg["f_tb"])
    if "OH" in names or "W" in names:
        fac += ["f_gear"]
    else:
        held["f_gear"] = gg["f_gear"]
    return fac, held


def run_fits(aero, form, sizing=None):
    """Individual (Eq. 127), combined (Eq. 128) and leave-one-out fits for one tail form."""
    from aircrafter2.mass.calibration import fit_correction_factors, leave_one_out, predict

    sizing = sizing or {}
    cases = {ac: make_case(ac, aero[ac], form, sizing_mass=sizing.get(ac)) for ac in ORDER}
    out = dict(individual={}, combined=None, loo=None)
    for ac in ORDER:
        fac, held = identified([ac], form)
        case = make_case(ac, aero[ac], form, inputs=held, sizing_mass=sizing.get(ac))
        r = fit_correction_factors([case], fac, x0=[X0[f] for f in fac])
        out["individual"][ac] = dict(factors=r.factors, held=held, objective=r.objective, errors=r.relative_errors[0],
                                     success=r.success)
    fac, _ = identified(ORDER, form)
    r = fit_correction_factors([cases[ac] for ac in ORDER], fac, x0=[X0[f] for f in fac])
    out["combined"] = dict(factors=r.factors, objective=r.objective, success=r.success,
                           errors={ac: e for ac, e in zip(ORDER, r.relative_errors)})
    # in-sample structure errors of the combined fit
    out["combined"]["structure_error"] = {}
    for ac in ORDER:
        _, pred = predict(cases[ac], r.factors)
        real = real_masses(ac)
        out["combined"]["structure_error"][ac] = sum(pred.values()) / sum(real.values()) - 1.0

    def fold(rest):
        fac, held = identified([c.name for c in rest], form)
        return fac, held

    out["loo"] = leave_one_out([cases[ac] for ac in ORDER], fold, x0=X0)
    return out


def spar_check(aero):
    """Real two-tube CF spar mass ([OH], [W]) against the model's single tube (m_tube), k_fill 0.85 and the
    front-spar fits (docs/MASS_CALIBRATION_PLAN.md section 4)."""
    from aircrafter2.mass.calibration import predict

    rho = base_config().inputs["rho_spar"]

    def tube(od, idd, length):
        return math.pi / 4 * (od**2 - idd**2) * length * rho

    mm = 1e-3
    oh_out = CASES["OH"]["b"][0] / 2 - 0.5 + 0.05  # step-down at 0.5 m semi-span, 50 mm overlap (p. 34)
    w_mid = 2.95 * FT  # mid-wing (p. 20); outer panels stepped down 0.08 in with a 2 in overhang (p. 33)
    w_out = (CASES["W"]["b"][0] - w_mid) / 2 + 2 * IN
    real = {
        "OH": dict(front=tube(20 * mm, 18 * mm, 1.0) + 2 * tube(18 * mm, 16 * mm, oh_out),
                   rear=tube(18 * mm, 16 * mm, 1.0) + 2 * tube(16 * mm, 14 * mm, oh_out), od_front=20 * mm,
                   layout="20/18 + 18/16 mm (front), 18/16 + 16/14 mm (rear); 1 m centre tubes, outer tubes "
                          f"{oh_out:.3f} m with the 50 mm overlap (pp. 21, 34; outer sizes inferred)"),
        "W": dict(front=tube(0.87 * IN, 0.79 * IN, w_mid) + 2 * tube(0.79 * IN, 0.71 * IN, w_out),
                  rear=tube(0.79 * IN, 0.71 * IN, w_mid) + 2 * tube(0.71 * IN, 0.63 * IN, w_out), od_front=0.87 * IN,
                  layout="0.87/0.79 + 0.79/0.71 in (front), 0.79/0.71 + 0.71/0.63 in (rear); 2.95 ft mid-wing, "
                         f"outer {w_out:.3f} m with the 2 in overhang (pp. 20, 33)"),
    }
    out = {}
    for ac in ("OH", "W"):
        r = real[ac]
        tot = r["front"] + r["rear"]
        sec = section(ac)
        c = CASES[ac]["S"][0] / CASES[ac]["b"][0] if ac == "W" else CASES[ac]["c"][0]
        clearance = base_config().inputs["clearance"]
        rows = []
        for label, kf in [("0.85 (placeholder)", 0.85),
                          ("front-spar OD", (r["od_front"] / 2 + clearance) * 2 / (sec["t_at_spar"] * c)),
                          ("0.5", 0.5)]:
            got, _ = predict(make_case(ac, aero[ac], inputs={"k_fill": kf}), gg_factors())
            rows.append(dict(label=label, k_fill=kf, m_tube=got["m_tube"], error=got["m_tube"] / tot - 1.0))
        # r_tube and t_tube of the 0.85 tube and the thin-wall check for each row
        for row in rows:
            p = _problem(make_case(ac, aero[ac], inputs={"k_fill": row["k_fill"]}))
            row.update(r_tube=float(p.get_val("r_tube")[0]), t_tube=float(p.get_val("t_tube")[0]))
            row["t_over_r"] = row["t_tube"] / row["r_tube"]
        out[ac] = dict(front=r["front"], rear=r["rear"], total=tot, layout=r["layout"], rows=rows)
    return out


def _problem(case, factors=None):
    from aircrafter2.mass.calibration import _CaseProblem

    cp = _CaseProblem(case)
    cp.run(factors or gg_factors())
    return cp.prob


def uncalibrated(aero):
    """real / predicted with the G&G factors, before and after Step 1."""
    from aircrafter2.mass.calibration import predict

    out = {}
    for label, inputs in [("before", BEFORE_STEP1), ("after", {})]:
        out[label] = {}
        for ac in ORDER:
            case = make_case(ac, aero[ac], "gg", inputs=inputs)
            got, pred = predict(case, gg_factors())
            p = _problem(case)
            real = real_masses(ac)
            out[label][ac] = dict(ratio={k: real[k] / pred[k] for k in real}, pred=pred,
                                  t_tube=float(p.get_val("t_tube")[0]), r_tube=float(p.get_val("r_tube")[0]))
    return out


def sensitivity_E(aero):
    from aircrafter2.mass.calibration import predict

    out = {}
    for ac in ORDER:
        out[ac] = {f"{E / 1e9:g}": predict(make_case(ac, aero[ac], "gg", inputs={"E_spar": E}), gg_factors())[1]["m_wing"]
                   for E in (42.9e9, 84e9, 220e9)}
    return out


def y_motor_sensitivity(aero, factors):
    """[OH]'s wing prediction at y_motor 0.25 / 0.335 / 0.45 m, with the optimiser's options (no inertia relief) and
    with relief on."""
    from aircrafter2.mass.calibration import predict

    out = {}
    for relief in (False, True):
        out["relief_on" if relief else "relief_off"] = {
            f"{y:g}": predict(make_case("OH", aero["OH"], "areal", y_motor=y, relief=relief), factors)[1]["m_wing"]
            for y in (0.25, 0.335, 0.45)}
    return out


def closure_check(aero, factors):
    """[OH] and [W] with the calibrated factors and the closure on, the real m_sys and M2 payload: predicted M2 total
    against the weight-and-balance total."""
    from aircrafter2.mass.calibration import predict

    out = {}
    for ac in ("OH", "W"):
        r = REPORTS[ac]
        m_sys = sum(v for _, v in r["systems"]) * r["unit"]
        m_pay = sum(v for _, v in r["payload_m2"]) * r["unit"]
        got, _ = predict(make_case(ac, aero[ac], "areal", closure=dict(m_sys=m_sys, m_pay=m_pay)), factors)
        wb = r["m2_total"] * r["unit"]
        rows_sum = sum(real_masses(ac).values()) + m_sys + m_pay
        out[ac] = dict(m_sys=m_sys, m_pay=m_pay, structure_pred=got["m_empty"] - m_sys, m_total=got["m_total"],
                       wb_total=wb, wb_rows_sum=rows_sum, error=got["m_total"] / wb - 1.0,
                       error_rows=got["m_total"] / rows_sum - 1.0)
    return out


def loo_config(aero):
    """Factors fitted on [OH] and [HB] only (Woombah left out), areal tails."""
    from aircrafter2.mass.calibration import fit_correction_factors

    fac, held = identified(["OH", "HB"], "areal")
    r = fit_correction_factors([make_case(ac, aero[ac]) for ac in ("OH", "HB")], fac, x0=[X0[f] for f in fac])
    return dict(factors=r.factors, held=held, objective=r.objective, success=r.success)


def fbuild_spread(loo):
    worst = max(abs(r["structure_error"]) for r in loo.values())
    return worst, math.ceil(worst / 0.05 - 1e-9) * 0.05


def fit():
    import yaml  # noqa: F401

    from aircrafter2.config.layer_files import read_rows, save_rows

    aero = {}
    for ac in ORDER:
        aero[ac] = wing_aero(ac)
        print(ac, aero[ac], flush=True)
    res = dict(git=git_hash(), aero=aero, sections={ac: section(ac) for ac in ORDER},
               real={ac: real_masses(ac) for ac in ORDER})
    res["uncalibrated"] = uncalibrated(aero)
    res["E_sensitivity"] = sensitivity_E(aero)
    res["spar_check"] = spar_check(aero)
    res["fits"] = {form: run_fits(aero, form) for form in ("areal", "gg")}
    for form in ("areal", "gg"):
        f = res["fits"][form]
        print(form, {k: round(v, 4) for k, v in f["combined"]["factors"].items()}, "F", round(f["combined"]["objective"], 4),
              {ac: round(100 * r["structure_error"], 1) for ac, r in f["loo"].items()}, flush=True)
    # sizing mass at the weight-and-balance M2 totals ([W]'s sizing mass is already its M2 total)
    m2 = {"OH": REPORTS["OH"]["m2_total"] * G, "HB": REPORTS["HB"]["m2_total"] * LB}
    res["sensitivity_sizing_m2"] = {"sizing": m2, "areal": run_fits(aero, "areal", sizing=m2)}
    comb = res["fits"]["areal"]["combined"]["factors"]
    res["y_motor"] = y_motor_sensitivity(aero, comb)
    res["closure"] = closure_check(aero, comb)
    res["loo_woombah"] = loo_config(aero)
    worst, spread = fbuild_spread(res["fits"]["areal"]["loo"])
    res["f_build"] = dict(worst_loo=worst, spread=spread)
    # the gg form wins only if both its objective and its LOO errors are lower (decision 2026-10-23: judged on the
    # objective and the tail components' LOO errors; the structure totals are reported)
    tails_rms = {}
    for form in ("areal", "gg"):
        e = [r["errors"][k] for r in res["fits"][form]["loo"].values() for k in ("m_ht", "m_vt")]
        tails_rms[form] = float(np.sqrt(np.mean(np.square(e))))
    res["form_choice"] = dict(objective={f: res["fits"][f]["combined"]["objective"] for f in ("areal", "gg")},
                              tails_loo_rms=tails_rms,
                              structure_loo={f: {ac: r["structure_error"] for ac, r in res["fits"][f]["loo"].items()}
                                             for f in ("areal", "gg")})
    areal_wins = (tails_rms["areal"] < tails_rms["gg"]
                  and res["form_choice"]["objective"]["areal"] < res["form_choice"]["objective"]["gg"])
    res["form_choice"]["areal_wins"] = areal_wins
    OUT.mkdir(parents=True, exist_ok=True)
    FIT_JSON.write_text(json.dumps(res, indent=1))
    if not areal_wins:
        raise SystemExit("areal does not beat gg on the objective and the tail LOO errors: stop and ask (brief Step 3)")

    # configs/placeholders.yaml: the combined factors (areal, plus the gg tail factors for configs that pin gg) and the
    # f_build spread, saved in place (only those lines change)
    gg_comb = res["fits"]["gg"]["combined"]["factors"]
    new = {f"mass.inputs.{k}": round(float(v), 4 if k != "f_gear" else 5) for k, v in comb.items()}
    new.update({"mass.inputs.f_ht": round(gg_comb["f_ht"], 4), "mass.inputs.f_vt": round(gg_comb["f_vt"], 4)})
    rows, spreads = read_rows(CONFIGS / "placeholders.yaml")
    edits = [dict(r, value=new[r["parameter"]]) for r in rows if r["parameter"] in new]
    assert len(edits) == len(new), sorted(set(new) - {r["parameter"] for r in rows})
    sp = [dict(s, spread=round(spread, 2)) for s in spreads if s["name"] == "f_build"]
    save_rows(CONFIGS / "placeholders.yaml", edits, sp)

    lw = res["loo_woombah"]
    (CONFIGS / "mass_usyd_loo_woombah.yaml").write_text(
        "# GENERATED by studies/mass_calibration_usyd.py fit (do not edit): the USYD mass calibration WITHOUT Woombah,\n"
        "# fitted on [OH] Orange Hornet (2024 report, Table 35 p. 42) and [HB] Humpback (2026 report, Table 36 p. 44)\n"
        "# only (Eq. 128), for the out-of-sample Woombah validation (docs/studies/results/validation_2025.md). The\n"
        "# fuselage and gear factors (f_cb, f_tb, f_gear) rest on [OH] alone: one aircraft, an exact fit. The wing and\n"
        "# tail factors rest on [OH] and [HB]. Applied over the loaded problem config by studies/validation_2025.py (a\n"
        "# mass_config file would sit below configs/placeholders.yaml and lose to the calibrated factors there).\n"
        "# docs/studies/mass_calibration_usyd.md.\n"
        "extends: mass_garcia_gamboa_2022.yaml\noptions:\n  tail_mass_model: areal\ninputs:\n"
        + "".join(f"  {k}: {float(x):.6g}\n" for k, x in lw["factors"].items()))
    print("f_build spread", spread, "(worst LOO", round(100 * worst, 1), "%)")


# --- downstream: one deterministic 2024-25 optimum (studies/results_2025.py optimise) -------------------------------
RUNS = OUT / "runs"
PUBLISHED = "mp_6S_11x8E_10x7E"  # the published best deterministic case (results_2025.md §1), uncalibrated mass model
CALIBRATED = ["mpcal_6S_11x8E_10x7E", "mpcal_6S_11x8E_10x7E_warm"]  # the same case, calibrated: cold / warm start
RDO_RUNS = ["rdor_best", "rdor_ms0", "rdor_ms1"]  # the published reduced-set RDO (3 starts)


def _sec_optimum(w, res):
    """§9: the calibrated optimum against the published one, and the cost of rerunning the reduced-set RDO."""
    if not (RUNS / f"{PUBLISHED}.json").exists() or not any((RUNS / f"{t}.json").exists() for t in CALIBRATED):
        return
    pub = json.loads((RUNS / f"{PUBLISHED}.json").read_text())
    cal = [json.loads((RUNS / f"{t}.json").read_text()) for t in CALIBRATED if (RUNS / f"{t}.json").exists()]
    ok = [r for r in cal if r.get("ok") and "score" in r]
    best = max(ok, key=lambda r: r["score"]["flight_score"]) if ok else None
    res["optimum"] = dict(published=pub, calibrated=cal, best=best["tag"] if best else None)
    w("\n## 9. Downstream: one deterministic 2024-25 optimum\n")
    w(f"`studies/results_2025.py optimise` on `configs/dbf2025_multipoint.yaml` (now the calibrated mass model, "
      f"Woombah's n_panels and cargo box), the published best propulsion case (6S, {pub['props']['M2']} in M2, "
      f"{pub['props']['M3']} in M3, 1 L bottles, take-off off as published). Runs: "
      + "; ".join(f"`{r['tag']}` (start: {r.get('start') or 'config initial'}, {'converged' if r.get('ok') else 'NOT converged'}, "
                  f"{r.get('its')} iterations, {r.get('time', 0):.0f} s)" for r in cal)
      + f". Published: `{PUBLISHED}` (git {pub['git']}, uncalibrated, n_panels 1, 0.40 × 0.12 × 0.12 m box).\n")
    if not best:
        w("No calibrated run converged.\n")
        return
    w(f"Calibrated optimum: `{best['tag']}`.\n")
    rows = [("flight score F", lambda r: r["score"]["flight_score"], "{:.4f}"),
            ("M2 score", lambda r: r["score"]["M2"], "{:.4f}"), ("M3 score", lambda r: r["score"]["M3"], "{:.4f}"),
            ("fuel (lb)", lambda r: r["fuel_lb"], "{:.2f}"), ("M3 laps", lambda r: r["M3"]["laps_frac"], "{:.2f}"),
            ("M2 3-lap time (s)", lambda r: r["M2"]["ThreeLapTime"], "{:.1f}")]
    for pt in ("M2", "M3"):
        rows += [(f"{pt} m_total (kg)", lambda r, pt=pt: r[pt]["m_total"], "{:.3f}"),
                 (f"{pt} m_empty (kg)", lambda r, pt=pt: r[pt]["m_empty"], "{:.3f}")]
    for k in ("m_wing", "m_ht", "m_vt", "m_fus", "m_gear", "m_sys", "m_batt"):
        rows.append((f"M2 {k} (kg)", lambda r, k=k: r["M2"][k], "{:.3f}"))
    rows.append(("M2 structure, wing to gear (kg)", lambda r: sum(r["M2"][k] for k in ("m_wing", "m_ht", "m_vt", "m_fus",
                                                                                      "m_gear")), "{:.3f}"))
    for k in ("b", "c", "tc_scale", "l_ht", "S_ht", "S_vt", "Kv", "capacity_M2", "capacity_M3", "throttle_M2",
              "throttle_M3"):
        rows.append((k, lambda r, k=k: r["dv"][k], "{:.4g}"))
    rows += [("M2 speed (m/s)", lambda r: r["M2"]["v"], "{:.2f}"), ("M3 speed (m/s)", lambda r: r["M3"]["v"], "{:.2f}"),
             ("M2 SM", lambda r: r["M2"]["SM"], "{:.3f}")]
    w("| Quantity | published (uncalibrated) | calibrated | change |")
    w("|---|---:|---:|---:|")
    for lab, f, fmt in rows:
        a, b = f(pub), f(best)
        ch = f"{100 * (b / a - 1):+.1f} %" if a else "–"
        w(f"| {lab} | {fmt.format(a)} | {fmt.format(b)} | {ch} |")
    pa, ba = set(pub["active_constraints"]), set(best["active_constraints"])
    pb, bb = set(pub["active_bounds"]), set(best["active_bounds"])
    w("\nActive constraints (within 1e-4 of the reference): both " + (", ".join(sorted(pa & ba)) or "none")
      + "; published only: " + (", ".join(sorted(pa - ba)) or "none") + "; calibrated only: "
      + (", ".join(sorted(ba - pa)) or "none") + ".")
    w("Active bounds: both " + (", ".join(sorted(pb & bb)) or "none") + "; published only: "
      + (", ".join(sorted(pb - bb)) or "none") + "; calibrated only: " + (", ".join(sorted(bb - pb)) or "none")
      + f". Kv set by: published {pub['kv_set_by']}; calibrated {best['kv_set_by']}.\n")
    rdo = [json.loads((RUNS / f"{t}.json").read_text()) for t in RDO_RUNS if (RUNS / f"{t}.json").exists()]
    if rdo:
        res["rdo_cost"] = {r["tag"]: dict(time=r.get("time"), its=r.get("its"), start=r.get("start")) for r in rdo}
        tot = sum(r.get("time", 0) for r in rdo)
        w("**Cost of rerunning the reduced-set RDO (estimate, not run).** The published reduced-set RDO (d = 4, UDR "
          "N = 9 nodes × 2 points) took " + ", ".join(f"`{r['tag']}` {r.get('time', 0) / 60:.0f} min "
                                                      f"({r.get('its')} iterations, start {r.get('start')})" for r in rdo)
          + f", run in parallel with {rdo[0].get('threads', '3')} threads each: {max(r.get('time', 0) for r in rdo) / 60:.0f} "
          f"min wall, {tot / 3600:.1f} h of run time in total. A rerun on the calibrated model is the same problem size, "
          "so expect the same order (the wall time above for the three starts in parallel, if the iteration counts "
          "are similar), plus the downstream evaluations the results page needs (the common evaluation's Monte "
          "Carlo, robust_eval and propagation; docs/studies/results/compute_performance.md), which were not timed "
          "separately here. Not run (brief).\n")


# --- report ---------------------------------------------------------------------------------------------------------

def pct(x):
    return f"{100 * x:+.1f} %"


def report():
    res = json.loads(FIT_JSON.read_text())
    L = []
    w = L.append
    w("# Mass-model calibration on three USYD aircraft\n")
    w(f"Generated by `studies/mass_calibration_usyd.py report` from `fit` (git {res['git']}); no hand-typed numbers. "
      "Plan, data checks and decisions: `docs/MASS_CALIBRATION_PLAN.md`. Model: `aircrafter2/mass/` "
      "(`docs/modules/mass.md`); fitting hook: `aircrafter2/mass/calibration.py` (G&G Eqs. 127-128, relative "
      "residuals, equal weights).\n")
    w("**Three aircraft is a small sample.** The leave-one-out (LOO) errors in §5 are the evidence for the scatter; "
      "the in-sample fits (§4) are not validation. The fuselage factors (f_cb, f_tb) and f_gear rest on two aircraft "
      "each ([OH] and [W], one tricycle and one taildragger; [W]'s fuselage is one lumped row), so they are weak.\n")
    w("Sources: [OH] Orange Hornet, USYD DBF 2023-24; [W] Project Woombah, 2024-25; [HB] Project Humpback, 2025-26 "
      "(printed page numbers); [GG] Garcia & Gamboa (2022).\n")

    w("## 1. Data\n")
    w("Component masses as published, converted in the script (g → kg, lb → kg):\n")
    w("| Model output | " + " | ".join(f"[{ac}] {REPORTS[ac]['table']}" for ac in ORDER) + " |")
    w("|---|" + "---|" * len(ORDER))
    for key in ["m_wing", "m_ht", "m_vt", "m_cb", "m_tb", "m_fus", "m_gear"]:
        cells = []
        for ac in ORDER:
            r = REPORTS[ac]
            if key in r["rows"]:
                rows = r["rows"][key]
                txt = " + ".join(f"{n} {v:g}" for n, v in rows) + f" {r['unit_name']}"
                cells.append(f"{txt} = **{res['real'][ac][key]:.4f} kg**")
            elif key == "m_fus" and ac == "OH":
                cells.append("(m_cb + m_tb)")
            else:
                cells.append("not used" if ac == "HB" and key in ("m_fus", "m_gear") else "–")
        w(f"| `{key}` | " + " | ".join(cells) + " |")
    w("\nMapping notes:\n")
    w("- **[OH] 'Empenage' is the rear fuselage** (CG 898 mm aft of the LE datum; the 680 mm rear fuselage starts at "
      "the end of the 1050 mm midsection, whose centroid sits at the wing quarter chord (pp. 19, 39); the HS and VS "
      "rows are at 1169 and 1153 mm). It plays the model's tail boom. [OH]'s 'Fuselage' (the midsection) and the "
      "nosecone are the cargo box.")
    w("- **Cargo-box convention:** the constant-section payload fuselage, with the nose-cone mass lumped into it; the "
      "rear fuselage plays the tail boom. [W]'s fuselage is one row (fuselage + nose), fitted as a lumped `m_fus`.")
    w("- [OH] values are CAD estimates (Onshape density × volume, p. 41); P2 flew at 8.6 kg with the 2.8 kg M2 "
      "payload (Table 43, p. 59), 5.8 kg against Table 35's 5.84 kg M1: weak evidence that CAD ≈ built. P1 was "
      "heavier (7.9 kg 'airframe weight', p. 59, ambiguous).")
    w("- [W] values are CAD 'verified against the constructed structure' (p. 44). **Table 34 has no servo row**: the "
      "servos are inside the component rows or missing (flagged, not corrected).")
    w("- [HB] is used for the wing and tails only: Table 36's 6.14 lb 'Fuselage' also holds the gear, servos, "
      "receivers, receiver batteries, prop and wiring. Its mass basis is unclear (§8).\n")
    w("Case inputs (geometry as the optimiser defines it: `c_root = c_mean = c`, `l_tb = l_ht − boom_offset` with "
      "the placeholder boom_offset, `S_cb = 2(lw + lh + wh)`; n = 5 and SF = 1.5 from all three reports; the rest from "
      "`configs/placeholders.yaml` and `mass_garcia_gamboa_2022.yaml`):\n")
    w("| Input | " + " | ".join(f"[{ac}]" for ac in ORDER) + " |")
    w("|---|" + "---|" * len(ORDER))
    for key, lab, fmt in [("b", "b (m)", "{:.4f}"), ("c", "c (m)", "{:.4f}"), ("S", "S (m²)", "{:.4f}"),
                          ("seed", "airfoil seed", "{}"), ("n_panels", "n_panels", "{}"), ("S_ht", "S_ht (m²)", "{:.4f}"),
                          ("S_vt", "S_vt (m²)", "{:.4f}"), ("l_ht", "tail arm (m)", "{:.3f}"),
                          ("m_size", "sizing mass (kg)", "{:.3f}"), ("V", "V_design (m/s)", "{:.2f}"),
                          ("alt", "site altitude (m)", "{:.0f}")]:
        cells = []
        for ac in ORDER:
            v, src = CASES[ac][key]
            if key == "c" and ac == "W":
                v = CASES["W"]["S"][0] / CASES["W"]["b"][0]
            cells.append(fmt.format(v) + f" ({src})")
        w(f"| {lab} | " + " | ".join(cells) + " |")
    w("| cargo box l × w × h (m) | " + " | ".join(
        (" × ".join(f"{x:g}" for x in CASES[ac]["cb"][0]) + f" ({CASES[ac]['cb'][1]})") if CASES[ac]["cb"] else "–"
        for ac in ORDER) + " |")
    w("| wing units (per side) | " + " | ".join(
        (f"m {CASES[ac]['units']['m'][0]:.4f} kg ({CASES[ac]['units']['m'][1]}), x {CASES[ac]['units']['x'][0]:g} m "
         f"({CASES[ac]['units']['x'][1]}), y {CASES[ac]['units']['y'][0]:.4f} m ({CASES[ac]['units']['y'][1]})")
        if CASES[ac]["units"] else "none (one nose motor)" for ac in ORDER) + " |")
    w("| t/c, t/c at 0.25c | " + " | ".join(
        f"{res['sections'][ac]['tc_max']:.4f}, {res['sections'][ac]['t_at_spar']:.4f}" for ac in ORDER) + " |")
    w("| A/c², P/c | " + " | ".join(f"{res['sections'][ac]['A_c2']:.4f}, {res['sections'][ac]['P_c']:.3f}"
                                    for ac in ORDER) + " |")
    w("| Cm, CL_α (1/rad), α at L = W | " + " | ".join(
        f"{res['aero'][ac]['Cm']:.4f}, {res['aero'][ac]['CL_alpha']:.3f}, {res['aero'][ac]['alpha']:.2f}°"
        for ac in ORDER) + " |")
    w("\nSection coefficients from the airfoil module (`aircrafter2/airfoil/cst.py` on the seed); Cm and CL_α from the "
      "wing aero module (VLM + NeuralFoil strips, 21 spanwise nodes) at V_design, the site density and the α where "
      "the lift equals the sizing weight, as the optimiser feeds the mass model. The motor x and y stations only "
      "enter the nacelle torsion (`motor_inertia_relief` off, as the 2024-25 problem).\n")

    w("## 2. Step 1: the cited physical placeholders\n")
    w("| Input | Before | After | Source |")
    w("|---|---|---|---|")
    w("| `t_tube_min` | 0.5 mm (PLACEHOLDER) | **1.0 mm** | every CF tube of the three reports has a 1.0 mm wall: [OH] "
      "20/18, 18/16, 12/10, 10/8, 8/6 mm ('1 mm wall thickness … tubes', p. 36); [W] 0.87/0.79, 0.79/0.71, "
      "0.472/0.394, 0.394/0.315 in; [HB] 0.63/0.55, 0.39/0.31 in |")
    w("| `sigma_spar` | 720 MPa (G&G Table 13) | **313 MPa** | the team's measured COTS-tube strength, used as the "
      "design allowable ([OH] p. 35; §8.1.1 p. 55 prints '3.13 MPa', a typo). Spar Sizer 3.1 uses 340.7 MPa (9 % "
      "higher) |")
    w("| `E_spar`, `tau_spar`, `G_spar`, `rho_spar` | G&G | G&G (unchanged) | the USYD stiffness sources disagree "
      "([OH] 42.9 GPa flexural, Spar Sizer 220 GPa); E changes no wing mass (below) |")
    w("| `k_fill` | 0.85 (PLACEHOLDER) | 0.85, checked (§3) | |")
    w("| `n`, `SF` | 5, 1.5 | unchanged | [OH] pp. 20, 34; [W] p. 20; [HB] p. 36 |\n")
    E = res["E_sensitivity"]
    w("Wing mass (G&G factors, after Step 1) at E_spar = 42.9 / 84 / 220 GPa: " + "; ".join(
        f"[{ac}] " + " / ".join(f"{E[ac][k]:.4f}" for k in ("42.9", "84", "220")) + " kg" for ac in ORDER)
      + ". The deflection limit never binds, so E does not change any wing mass.\n")
    u = res["uncalibrated"]
    w("Real / predicted with the G&G factors (`mass_garcia_gamboa_2022.yaml`, k_wing = 1, gg tails), before and "
      "after Step 1:\n")
    keys = ["m_wing", "m_ht", "m_vt", "m_cb", "m_tb", "m_fus", "m_gear"]
    w("| Component | " + " | ".join(f"[{ac}] before | [{ac}] after" for ac in ORDER) + " |")
    w("|---|" + "---:|" * (2 * len(ORDER)))
    for k in keys:
        cells = []
        for ac in ORDER:
            for lab in ("before", "after"):
                r = u[lab][ac]["ratio"]
                cells.append(f"{r[k]:.2f}" if k in r else "–")
        w(f"| {k} | " + " | ".join(cells) + " |")
    w("| spar wall t (mm) | " + " | ".join(f"{1e3 * u[lab][ac]['t_tube']:.2f}" for ac in ORDER
                                          for lab in ("before", "after")) + " |\n")

    w("## 3. Spar-mass check (k_fill)\n")
    w("USYD carry the wing bending in two CF tubes; the model has one tube, as deep as `k_fill` of the section allows. "
      "Real spar masses from the tube sizes and spans (ρ = `rho_spar`):\n")
    sc = res["spar_check"]
    for ac in ("OH", "W"):
        s = sc[ac]
        w(f"- [{ac}] {s['layout']}: front {s['front']:.4f} + rear {s['rear']:.4f} = **{s['total']:.4f} kg**.")
    w("\n| k_fill | " + " | ".join(f"[{ac}] r, t (t/r) | [{ac}] m_tube vs real" for ac in ("OH", "W")) + " |")
    w("|---|---|---|---|---|")
    for i in range(3):
        cells = []
        for ac in ("OH", "W"):
            r = sc[ac]["rows"][i]
            cells += [f"{1e3 * r['r_tube']:.1f} mm, {1e3 * r['t_tube']:.2f} mm ({r['t_over_r']:.2f})",
                      f"{r['m_tube']:.4f} kg, {pct(r['error'])}"]
        lab = sc["OH"]["rows"][i]["label"]
        if lab == "front-spar OD":
            lab = f"front-spar OD ({sc['OH']['rows'][i]['k_fill']:.3f} / {sc['W']['rows'][i]['k_fill']:.3f})"
        w(f"| {lab} | " + " | ".join(cells) + " |")
    w("\n**k_fill stays 0.85, as a checked equivalence** (decision 2026-10-23). The model's single deep tube gives the "
      "two-tube spar mass within the errors above. A k_fill fitted to the front spar alone needs a thick-walled tube "
      "([W] at 0.5: t/r above `wall_ratio_max` 0.2), outside the thin-wall equations (OPEN_QUESTIONS Q5).\n")

    w("## 4. Fits\n")
    for form in ("areal", "gg"):
        f = res["fits"][form]
        fac = AREAL if form == "areal" else GG
        w(f"### Tail form `{form}`" + (" (adopted)" if form == "areal" else " (G&G Eqs. 34-35, for comparison)") + "\n")
        w("| Fit | " + " | ".join(f"`{k}`" for k in fac) + " | Eq. 127/128 |")
        w("|---|" + "---:|" * (len(fac) + 1))
        for ac in ORDER:
            ind = f["individual"][ac]
            cells = [f"{ind['factors'][k]:.4g}" if k in ind["factors"] else
                     (f"({ind['held'][k]:.4g} held)" if k in ind["held"] else "–") for k in fac]
            w(f"| individual [{ac}] | " + " | ".join(cells) + f" | {ind['objective']:.2e} |")
        c = f["combined"]
        w("| **combined (Eq. 128)** | " + " | ".join(f"**{c['factors'][k]:.4g}**" for k in fac)
          + f" | **{c['objective']:.4f}** |")
        w("\nIn-sample relative errors of the combined fit (m_calc / m_real − 1):\n")
        w("| Aircraft | " + " | ".join(keys) + " | structure (wing to gear) |")
        w("|---|" + "---:|" * (len(keys) + 1))
        for ac in ORDER:
            e = c["errors"][ac]
            w(f"| [{ac}] | " + " | ".join(pct(e[k]) if k in e else "–" for k in keys)
              + f" | {pct(c['structure_error'][ac])} |")
        w("")
    w("Units: `a_ht`, `a_vt`, `f_cb` in kg/m². Individual fits use the factors each aircraft identifies: [W]'s "
      "lumped fuselage fixes only f_cb with f_tb held at the G&G 1 mm-boom value; [HB] identifies the wing and tails "
      "only. [OH]'s individual fit is exact (six factors, six masses). With k_wing, f_wing stays G&G's 1.12 per "
      "joint, and each aircraft has its real n_panels (1, 3, 2).\n")

    w("## 5. Leave-one-out\n")
    w("Refit without each aircraft and predict it. A factor the remaining aircraft do not identify is held at its "
      "G&G value (without [OH], [W]'s lumped fuselage gives f_cb with f_tb at 0.847).\n")
    for form in ("areal", "gg"):
        lo = res["fits"][form]["loo"]
        w(f"**`{form}`**\n")
        w("| Left out | factors fitted on the other two | " + " | ".join(keys) + " | **structure** |")
        w("|---|---|" + "---:|" * (len(keys) + 1))
        for ac in ORDER:
            r = lo[ac]
            fx = ", ".join(f"{k} {v:.3g}" for k, v in r["factors"].items())
            if r["held_at"]:
                fx += "; held " + ", ".join(f"{k} {v:.3g}" for k, v in r["held_at"].items())
            w(f"| [{ac}] | {fx} | " + " | ".join(pct(r["errors"][k]) if k in r["errors"] else "–" for k in keys)
              + f" | **{pct(r['structure_error'])}** |")
        w("")
    fb = res["f_build"]
    w(f"**f_build spread:** the largest |LOO structure error| (`areal`) is {100 * fb['worst_loo']:.1f} %, rounded up "
      f"to the next 5 %: **±{100 * fb['spread']:.0f} %** about the nominal 1.0 (`configs/placeholders.yaml`, Q39). "
      "The one-sided [1.0, 1.3] variant stays as a sensitivity, now cited to [HB]'s ≈20 % CAD → built (p. 43).\n")

    w("## 6. `gg` against `areal`\n")
    fc = res["form_choice"]
    w("| | `areal` | `gg` |")
    w("|---|---:|---:|")
    w(f"| Eq. 128 objective | {fc['objective']['areal']:.4f} | {fc['objective']['gg']:.4f} |")
    w(f"| LOO RMS, HT and VT | {100 * fc['tails_loo_rms']['areal']:.1f} % | {100 * fc['tails_loo_rms']['gg']:.1f} % |")
    for ac in ORDER:
        w(f"| LOO structure, [{ac}] left out | {pct(fc['structure_loo']['areal'][ac])} | "
          f"{pct(fc['structure_loo']['gg'][ac])} |")
    rms = {f: float(np.sqrt(np.mean(np.square(list(fc["structure_loo"][f].values()))))) for f in ("areal", "gg")}
    w(f"| LOO structure RMS | {100 * rms['areal']:.1f} % | {100 * rms['gg']:.1f} % |\n")
    w("G&G scale the tails with the wing mass; the USYD tails do not follow their wings, but their areal masses are "
      "consistent (individual a_ht, a_vt above). `areal` has the lower objective and the lower tail LOO errors and "
      "is adopted (decision 2026-10-23); on the structure totals the two forms are mixed (the maximum favours "
      "`areal`, the RMS `gg`) and the f_build spread is the same. `placeholders.yaml` also carries the `gg` tail "
      "factors for the published full-set RDO configs, which pin `tail_mass_model: gg` (their f_ht/f_vt spreads).\n")

    w("## 7. Sensitivity and closure check (reported, not used)\n")
    sm = res["sensitivity_sizing_m2"]
    c0, c1 = res["fits"]["areal"]["combined"]["factors"], sm["areal"]["combined"]["factors"]
    w(f"**Sizing mass at the W&B M2 totals** ([OH] {sm['sizing']['OH']:.3f} kg, [HB] {sm['sizing']['HB']:.3f} kg; "
      "[W]'s sizing mass is already its M2 total): combined factors " + ", ".join(
          f"{k} {c1[k]:.4g} ({pct(c1[k] / c0[k] - 1)})" for k in AREAL) + "; LOO structure errors " + ", ".join(
          f"[{ac}] {pct(r['structure_error'])}" for ac, r in sm["areal"]["loo"].items()) + ".\n")
    ym = res["y_motor"]
    on = list(ym["relief_on"].values())
    w("**[OH] y_motor** (not dimensioned; 0.335 m scaled from the three-view): predicted wing (calibrated) at y = "
      + ", ".join(f"{y} m" for y in ym["relief_off"]) + ": " + " / ".join(f"{v:.4f}" for v in ym["relief_off"].values())
      + " kg with the optimiser's options (no inertia relief: y does not enter), " + " / ".join(f"{v:.4f}" for v in on)
      + " kg with `motor_inertia_relief` on" + (" (unchanged: [OH]'s spar sits on the 1 mm minimum gauge, so the "
                                                "root-bending relief does not reach it)" if max(on) - min(on) < 1e-9
                                                else "") + ". The placeholder does not move the calibration.\n")
    w("**Closure check** (calibrated `areal` factors, closure on, the real m_sys and M2 payload):\n")
    w("| Aircraft | m_sys (kg) | M2 payload (kg) | predicted structure (kg) | predicted M2 (kg) | W&B M2 (kg) | error | "
      "sum of the W&B rows (kg) | error |")
    w("|---|---:|---:|---:|---:|---:|---:|---:|---:|")
    for ac, r in res["closure"].items():
        w(f"| [{ac}] | {r['m_sys']:.3f} | {r['m_pay']:.3f} | {r['structure_pred']:.3f} | {r['m_total']:.3f} | "
          f"{r['wb_total']:.3f} | {pct(r['error'])} | {r['wb_rows_sum']:.3f} | {pct(r['error_rows'])} |")
    w("\n[OH]'s M2 total includes the crew and the EMTs/patient/gurney rows as payload (Table 35's M2 is their sum). "
      "[W]'s printed M2 (26.02 lb) is above the sum of its M2 rows (validation_2025.md §2). The closure re-sizes "
      "the spar and the gear at the closed mass instead of the design MTOW.\n")
    lw = res["loo_woombah"]
    w("**LOO config for the Woombah validation** (`configs/mass_usyd_loo_woombah.yaml`, fitted on [OH] and [HB]): "
      + ", ".join(f"{k} {v:.4g}" for k, v in lw["factors"].items())
      + ". Its fuselage and gear factors rest on [OH] alone.\n")

    w("## 8. Report inconsistencies\n")
    for item in [
        "[HB] mass basis: CAD values 'validated against the measured weights … with a ≈20 % increase from CAD to "
        "manufacturing' (p. 43); whether Table 36 includes the 20 % is not stated (OPEN_QUESTIONS Q47).",
        "[HB] Table 36: the 'Mission 1' row reads 1.22 lb (the battery alone); M2 is 18.54 lb against 18.24 lb from "
        "its own rows; empty 9.93 + battery 1.22 = 11.15 lb against an M1 take-off weight of 8.6 lb (Table 33, p. 35) "
        "and an MTOW of 14.3 lb (pp. 3, 36).",
        "[HB] spar sizes: p. 26 text 0.71/0.63 and 0.55/0.47 in; Table 24 0.63/0.55 and 0.39/0.31 in; p. 37 5/8 and "
        "3/8 in (all 0.04 in walls).",
        "[HB] airfoil 'SD3016' (p. 36) against S3016 (Table 24, p. 26; p. 37): the UIUC S3016 is used (Q51).",
        "[HB] wing area 868 in² against 59 × 14.57 = 859.6 in².",
        "[W] Table 34 has no servo row (Q48).",
        "[W] cargo-box width 4.7 in (Fig. 11, p. 19) against 4.72 in maximum width (Table 30, p. 31).",
        "[OH] midsection longerons: four 12/10 mm (p. 19) against four 10/8 mm (p. 39).",
        "[OH] M2 take-off weight 8.6 kg (Table 43) against 8.9 kg 'actual' (Table 44, p. 59).",
        "[OH] tube strength '3.13 MPa' (p. 55) against 313 MPa (p. 35): a typo.",
        "[OH] the wing has 1 mm plywood endplates (p. 21), not modelled (`winglet_ratio` 0; k_wing absorbs them).",
        "[OH] y_motor is not dimensioned in the drawings (scaled: 0.335 m, PLACEHOLDER).",
        "[OH] site: p. 25 uses 400 m (Wichita); the case uses 400 m.",
    ]:
        w(f"- {item}")
    _sec_optimum(w, res)
    DOCS.mkdir(parents=True, exist_ok=True)
    (DOCS / "mass_calibration_usyd.md").write_text("\n".join(L) + "\n")
    (DOCS / "mass_calibration_usyd.json").write_text(json.dumps(res, indent=1))
    print("\n".join(L))


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("part", choices=["fit", "report"])
    {"fit": fit, "report": report}[ap.parse_args().part]()
