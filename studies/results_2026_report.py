"""docs/studies/results_2026/results_2026.{md,json}: the thesis results page, from the campaign's JSONs only.

    .venv/bin/python studies/results_2026_report.py

collect() reads every run into one dict (written as results_2026.json); tables() turns it into named tables, which both
the page and the exhibits (studies/results_2026_exhibits.py: .tex, .csv) print, so the page and the thesis tables can't
disagree. A number in the text comes from the same dict. A run that is missing shows as "missing" with its reason. The
only typed numbers are citations ([RES26], [HB]) and the thresholds, in the constants below.
"""

import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "studies"))
import results_2026 as S  # noqa: E402
from results_2026 import OUT, RUNS  # noqa: E402

DOCS = ROOT / "docs" / "studies" / "results_2026"
LABELS = {"humpback": "Humpback", "humpback_loo": "Humpback (LOO factors)", "ac1_direct": "AC1 direct (a)",
          "ac1_completed": "AC1 completed (b)", "mdo": "MDO", "mdo_wind": "MDO, mean wind", "rdo": "MDO + RDO"}
MAIN = ["humpback", "ac1_direct", "ac1_completed", "mdo", "mdo_wind", "rdo"]
ALL = ["humpback", "humpback_loo", "ac1_direct", "ac1_completed", "mdo", "mdo_wind", "rdo"]
FOUR = ["humpback", "ac1_completed", "mdo", "rdo"]
RES26 = dict(M2=1.54, M3=2.52, GM=0.01, source="[RES26] 2026 DBF Competition Summary (USYD, rank 14)")
NF_FLAG = 0.8  # below it a NeuralFoil section is flagged untrustworthy (Q10)
TOL_ACTIVE = "within 1e-4 of ref"
OK_F = 1e-3  # two converged runs within this F are the same optimum

# What sets each active constraint or bound: (kind, where).
CONSTRAINT_SOURCE = {
    "V_h": ("PLACEHOLDER", "Q9 (lower 0.35; flown 0.66-0.80)"), "V_v": ("PLACEHOLDER", "Q9 (lower 0.02; flown 0.059-0.08)"),
    "SM": ("cited, model frame", "Q9, Q62"), "energy_margin_Wh": ("rule", "100 Wh [R26 p. 16]"),
    "energy_margin": ("model", "the mission's energy"), "pax_margin": ("rule", "3 passengers per cargo [R26 p. 10]"),
    "fuselage_margin": ("PLACEHOLDER", "Q55 (l_cb <= 1.5 b)"), "tail_margin": ("PLACEHOLDER", "Q64"),
    "batt_fwd_margin": ("cited", "Q64"), "batt_aft_margin": ("cited", "Q64"), "rpm_margin": ("cited", "[APC] 150000/D"),
    "I_margin": ("cited", "ESC 60 A [HB p. 28]"), "fuse_margin": ("rule", "100 A [R26 p. 15]"),
    "fuse_margin_C": ("cited", "Q8"), "togw_margin": ("rule", "55 lb [R26 p. 13]"), "stall_margin": ("model", "1.2 V_stall"),
    "turn_speed_margin": ("PLACEHOLDER", "Q18"), "thrust_margin_static": ("model", "static thrust against the 350 ft roll"),
    "v_fit_margin": ("model", "T(v) fit range"), "wall_ratio": ("model", "spar wall"), "r_tube": ("PLACEHOLDER", "Q3"),
    "cl_margin_sec": ("model", "section c_l"),
}
BOUND_SOURCE = {
    ("b", "upper"): ("rule", "5 ft [R26 p. 10]"), ("b", "lower"): ("rule", "3 ft [R26 p. 10]"),
    ("c", "upper"): ("PLACEHOLDER", "configs/full_mdo.yaml, no Q (sensitivity i)"),
    ("camber_scale", "upper"): ("PLACEHOLDER", "configs/full_mdo.yaml, Q10 (sensitivity i)"),
    ("L_banner", "upper"): ("PLACEHOLDER", "Q56 (275 in)"), ("L_banner", "lower"): ("rule", "10 in [R26 p. 12]"),
    ("Kv", "upper"): ("cited", "[TM7] catalogue 270"), ("Kv", "lower"): ("cited", "[TM7] catalogue 200"),
    ("throttle_M2", "upper"): ("physical", "full throttle"), ("throttle_M3", "upper"): ("physical", "full throttle"),
    ("throttle_M2", "lower"): ("PLACEHOLDER", "0.5, no Q"), ("throttle_M3", "lower"): ("PLACEHOLDER", "0.5, no Q"),
}
BOUND_DEFAULT = ("PLACEHOLDER", "config bound")


def rd(tag):
    f = RUNS / f"{tag}.json"
    return json.loads(f.read_text()) if f.exists() else None


def score_of(s):
    """Capped, uncapped and floored F of a summary dict."""
    sc = s["score"]
    return dict(F=sc["flight_score"], uncapped=sc["M1"] + 1.0 + sc["r_M2"] + 2.0 + sc["r_M3"], floored=s["floored"]["flight_score"],
                M2=sc["M2"], M3=sc["M3"], r_M2=sc["r_M2"], r_M3=sc["r_M3"])


def short(d):
    """A deterministic run in the form the tables print."""
    if d is None:
        return None
    out = {k: d.get(k) for k in ("tag", "ok", "its", "nit", "error", "time", "git", "start", "fixed", "wind", "overrides",
                                 "bounds", "cells", "props", "active_constraints", "active_bounds", "violated", "kv_set_by",
                                 "dv", "counters", "mda_fallbacks", "threads", "peak_rss_gb", "M1_check", "cap")}
    if "score" in d:
        out.update(score_of(d))
        for pt in ("M2", "M3"):
            out[f"laps_{pt}"] = d[pt]["laps_frac"]
            out[f"nf_{pt}"] = d[pt]["cruise.nf_confidence_min"]
    return out


def collect():
    designs_ = {}
    for lab in ALL:
        c = rd(f"fr_common_{lab}")
        designs_[lab] = dict(common=c, mc_in=rd(f"fr_mc_in_{lab}"), mc_out=rd(f"fr_mc_out_{lab}"),
                             wind_dir=rd(f"fr_wind_dir_{lab}"), wind_env=rd(f"fr_wind_env_{lab}"),
                             reanalysis=rd(f"fr_reanalysis_{lab}"), screen=rd(f"fr_screen_{lab}"))
    enum = [short(d) for d in S.enumeration_runs()]
    ranked = S.ranked_cases()
    best = ranked[0] if ranked else None
    case = best["tag"] if best else None
    int_runs = [short(json.loads(f.read_text())) for f in sorted(RUNS.glob(f"{case}_int_*p*c.json"))] if case else []
    mdo = S.best_integer(case) if case else None
    wind_runs = [short(json.loads(f.read_text())) for f in sorted(RUNS.glob("fr_wind_int_*p*c.json"))]
    rdo_all = []
    for r in S.rdo_runs():
        rdo_all.append({k: r.get(k) for k in ("tag", "case", "start", "ok", "its", "nit", "error", "time", "N", "active", "dv",
                                              "summary", "fixed", "fallbacks", "counters", "peak_rss_gb", "k", "beta", "git",
                                              "threads", "maxiter")})
    ac1 = (rd("fr_ac1_direct") or {}).get("ac1") or (rd("fr_ac1_completed") or {}).get("ac1")
    R = dict(page_git=S.git_hash(), freeze=_freeze(), designs=designs_, enumeration=enum, best_case=case,
             second_case=ranked[1]["tag"] if len(ranked) > 1 else None, integer=int_runs, mdo_tag=mdo["tag"] if mdo else None,
             fr_check=short(rd("fr_check")), wind_cont=short(rd("fr_wind")), wind_integer=wind_runs,
             multistart=[short(rd(f"fr_ms{i}")) for i in range(10) if rd(f"fr_ms{i}")],
             bookends={k: short(rd(f"fr_{k}")) for k in ("M2only", "M3only")},
             normalisation={k: short(rd(f"fr_norm_{k}")) for k in ("NI0.5", "NI2", "B0.5", "B2")},
             bounds={k: short(rd(f"fr_bound_{k}")) for k in ("i", "ii", "iii", "banner")},
             others={k: short(rd(f"fr_{k}")) for k in ("old_constraints", "takeoff_off")}, rdo=rdo_all,
             rdo_best=(S.best_rdo() or {}).get("tag"), rdo_best_cont=(S.best_rdo(rounded=False) or {}).get("tag"),
             propagation=rd("fr_propagation"), node_check=rd("fr_node_check"), placeholders=rd("fr_placeholders_mdo"),
             ac1=ac1, ac1_table=(rd("fr_ac1_direct") or {}).get("table"), ac1_throttle=(rd("fr_ac1_direct") or {}).get("throttle_notes"),
             validation=_validation(), y2025=_results_2025(), res26=RES26, nf_flag=NF_FLAG)
    R["provenance"] = provenance(R)
    return R


def _freeze():
    import subprocess

    return subprocess.run(["git", "rev-parse", "--short", S.FREEZE_TAG], cwd=ROOT, capture_output=True, text=True).stdout.strip()


def _validation():
    f = DOCS / "validation_2026.json"
    return json.loads(f.read_text()) if f.exists() else None


def _results_2025():
    """The 2024-25 headline numbers (Appendix A, frozen), for the comparison of conclusions."""
    f = ROOT / "docs" / "studies" / "results" / "results_2025.json"
    if not f.exists():
        return None
    d = json.loads(f.read_text())
    uq = d["uq"]["reduced"]
    b = next(x for x in d["enumeration"] if x["tag"] == d["best_case"])
    best = next((x for x in uq["rdo"] if x["tag"] == uq["rdo_best"]), None)
    nom = uq["nominal"]
    mc = uq["mc"]
    return dict(best_case=d["best_case"], F=b["F"], M2=b["M2"], M3=b["M3"], rdo_robust=best["summary"]["flight_score"]["robust"] if best else None,
                nominal_cost=nom["multipoint"]["score"]["flight_score"] - nom["rdo"]["score"]["flight_score"],
                max_pv_mdo=max(mc["multipoint"]["p_violation"].values()), max_pv_rdo=max(mc["rdo"]["p_violation"].values()),
                sigma_mdo=mc["multipoint"]["stats"]["flight_score"]["sigma"], sigma_rdo=mc["rdo"]["stats"]["flight_score"]["sigma"],
                woombah_F=nom.get("woombah", {}).get("score", {}).get("flight_score"),
                ac1_direct_F=nom.get("ac1_direct", {}).get("score", {}).get("flight_score"),
                ac1_completed_F=nom.get("ac1_completed", {}).get("score", {}).get("flight_score"))


def provenance(R):
    """Every run JSON a page uses: its git field against the freeze commit."""
    bad = []
    n = 0
    for f in sorted(RUNS.glob("fr_*.json")):
        d = json.loads(f.read_text())
        n += 1
        if d.get("git") != R["freeze"]:
            bad.append(f"{f.stem}: {d.get('git')}")
    for m in ("envelope", "humpback_loo", "humpback_gg", "humpback_cal"):
        f = OUT / f"validation_{m}.json"
        if f.exists():
            n += 1
            g = json.loads(f.read_text()).get("git")
            if g != R["freeze"]:
                bad.append(f"validation_{m}: {g}")
    return dict(n=n, not_at_freeze=bad)


# ------------------------------------------------------------------------------------------------------- formatting
def f(x, nd=3):
    if x is None:
        return "–"
    if isinstance(x, str):
        return x
    if isinstance(x, (bool, np.bool_)):
        return "yes" if x else "no"
    if not np.isfinite(x):
        return "nan"
    return f"{x:.{nd}f}"


def signed(x, nd=4):
    return "–" if x is None else f"{x:+.{nd}f}"


def tail(name):
    return name.split(".")[-1]


def mark_constraint(c):
    """'name (side)' to 'name (side) [kind: where]'."""
    name, side = c.rsplit(" (", 1)
    kind, where = CONSTRAINT_SOURCE.get(tail(name), ("model", "unclassified"))
    return f"{name.replace('points.', '')} ({side} [{kind}: {where}]"


def mark_bound(c):
    name, side = c.rsplit(" (", 1)
    kind, where = BOUND_SOURCE.get((name, side.rstrip(")")), BOUND_DEFAULT)
    return f"{name} ({side} [{kind}: {where}]"


def active_list(s):
    return [mark_constraint(c) for c in s.get("active_constraints", [])] + [mark_bound(b) for b in s.get("active_bounds", [])]


def nominal(R, lab):
    c = (R["designs"][lab] or {}).get("common")
    return c["nominal"] if c and c["nominal"].get("ok") else None


def stat(R, lab, which="mc_in"):
    m = R["designs"][lab].get(which)
    return m["stats"]["flight_score"] if m else {}


class T:
    """A table: the page prints markdown, the exhibits .tex and .csv."""

    def __init__(self, name, caption, header, rows, note=""):
        self.name, self.caption, self.header, self.rows, self.note = name, caption, header, rows, note

    def md(self):
        s = "| " + " | ".join(self.header) + " |\n|" + "|".join("---" if i == 0 else "---:" for i in range(len(self.header))) + "|\n"
        return s + "".join("| " + " | ".join(str(c) for c in r) + " |\n" for r in self.rows)


# ---------------------------------------------------------------------------------------------------------- tables
def t_method(R):
    rows = []
    for lab in MAIN:
        n, m = nominal(R, lab), R["designs"][lab].get("mc_in")
        sc = score_of(n) if n else {}
        st = (m or {}).get("stats", {}).get("flight_score", {})
        rows.append([LABELS[lab], f(sc.get("F"), 4), f(sc.get("uncapped"), 4), f(sc.get("floored"), 4), f(sc.get("M2"), 4),
                     f(sc.get("M3"), 4), f(n["M2"]["laps_frac"], 2) if n else "–", f(n["M3"]["laps_frac"], 2) if n else "–",
                     f(st.get("mu"), 4), f(st.get("sigma"), 4), f(st.get("robust"), 4), f((m or {}).get("p_any"), 3),
                     (f"{m['mda_failed']} / {m['wind_limit']}" if m else "–")])
    return T("method_comparison", "The designs in the common evaluation (AirCrafter 2.0's model): nominal scores and the in-set "
             "Monte Carlo. F capped, uncapped and floored (completed laps, hard cap); MC over the five active parameters, 1024 "
             "scrambled Sobol' points, the same points for every design; P(any) counts constraint violations over converged "
             "samples; analysis failures are counted apart.",
             ["design", "F capped", "F uncapped", "F floored", "M2", "M3", "M2 laps", "M3 laps", "MC mean", "MC sigma",
              "mean - k sigma", "P(any violation)", "MDA failures / wind limits"], rows)


def t_nominal_loo(R):
    rows = []
    for lab in ("humpback", "humpback_loo"):
        n = nominal(R, lab)
        if n:
            sc = score_of(n)
            m = R["designs"][lab].get("mc_in") or {}
            st = m.get("stats", {}).get("flight_score", {})
            rows.append([LABELS[lab], f(sc["F"], 4), f(sc["floored"], 4), f(sc["M2"], 4), f(sc["M3"], 4), f(n["M2"]["m_total"], 3),
                         f(n["M2"]["m_empty"], 3), f(st.get("mu"), 4), f(st.get("sigma"), 4), f(m.get("p_any"), 3)])
    return T("humpback_factors", "Humpback as built in the common evaluation: the calibrated factors against the factors fitted "
             "without it.", ["mass factors", "F capped", "F floored", "M2", "M3", "M2 take-off mass (kg)", "empty mass (kg)", "MC mean",
                             "MC sigma", "P(any violation)"], rows)


def t_nominal(R):
    rows = []
    for lab in MAIN:
        n = nominal(R, lab)
        if not n:
            rows.append([LABELS[lab]] + ["missing"] * 8)
            continue
        m1 = n["M1_check"]
        rows.append([LABELS[lab], f(n["M2"]["laps_frac"], 2), f(n["M3"]["laps_frac"], 2), ", ".join(n["violated"]).replace("points.", "") or "none",
                     f"{'pass' if m1['passed'] else 'FAIL'}, SM {f(m1.get('constraints', {}).get('SM'), 3)}"
                     + (", SM_high" if m1.get("SM_high") else ""), f(m1.get("laps_frac"), 2),
                     f"{n['mda_fallbacks']['M2']['continuation']} / {n['mda_fallbacks']['M3']['continuation']}",
                     f(R["designs"][lab]["common"]["mean_wind"].get("score", {}).get("flight_score"), 4),
                     ", ".join(str(x) for x in n["cap"].values() if isinstance(x, bool)) if False else
                     ("M2" if n["cap"]["M2_capped"] else "") + (" M3" if n["cap"]["M3_capped"] else "") or "none"])
    return T("nominal_detail", "Nominal analysis: laps, violated constraints, the M1 check, continuation fallbacks (M2 / M3), F at "
             "the set's mean wind (5 m/s, direction 0), and which score sits at its cap.",
             ["design", "M2 laps", "M3 laps", "violated", "M1 check", "M1 laps", "continuations", "F at mean wind", "at the cap"], rows)


def t_constraints(R):
    names = []
    for lab in MAIN:
        n = nominal(R, lab)
        if n:
            for c in n["details"]["constraints"]:
                if c["name"] not in names:
                    names.append(c["name"])
    rows = []
    for name in names:
        row = [name.replace("points.", "")]
        for lab in MAIN:
            n = nominal(R, lab)
            c = next((x for x in n["details"]["constraints"] if x["name"] == name), None) if n else None
            row.append("–" if c is None else f"{c['margin']:.4g}" + (" **violated**" if c["margin"] < -1e-6 * max(abs(c.get("ref") or 0), 1.0) else ""))
        rows.append(row)
    return T("constraint_margins", "Every constraint's margin at each design, in its own units (distance to the nearest bound; "
             "negative = violated).", ["constraint"] + [LABELS[x] for x in MAIN], rows)


def _mc_labels(R, which):
    return [lab for lab in ALL if R["designs"][lab].get(which)]


def t_mc(R, which):
    rows = []
    for lab in _mc_labels(R, which):
        m = R["designs"][lab][which]
        st = m["stats"]["flight_score"]
        rows.append([LABELS[lab], f"{m['converged']}/{m['N']}", f(st["mu"], 4), f(st["sigma"], 4), f(st["robust"], 4), f(st["p5"], 4),
                     f(st["p95"], 4), f(m["stats"]["M2"]["mu"], 4), f(m["stats"]["M3"]["mu"], 4), f(m["p_any"], 3), m["mda_failed"],
                     m["wind_limit"], m["retried"], f(m["wall"] / 60, 1)])
    word = "in-set: the five active parameters" if which == "mc_in" else "out-of-set: the 17 parameters of the extended list"
    return T(f"{which}_summary", f"Monte Carlo of F ({word}), 1024 scrambled Sobol' points, seed 0, the same points for every design. "
             "MDA failures and wind limits are excluded from the statistics and are not constraint violations.",
             ["design", "converged", "mean", "sigma", "mean - k sigma", "5th pct", "95th pct", "M2 mean", "M3 mean", "P(any violation)",
              "MDA failures", "wind limits", "recovered from nominal", "wall (min)"], rows)


def t_pv(R, which):
    labs = _mc_labels(R, which)
    names = sorted({k for lab in labs for k, p in R["designs"][lab][which]["p_violation"].items() if p > 0})
    rows = [[n.replace("points.", "")] + [f(R["designs"][lab][which]["p_violation"].get(n), 3) for lab in labs] for n in names]
    rows.append(["**any constraint**"] + [f(R["designs"][lab][which]["p_any"], 3) for lab in labs])
    return T(f"{which}_p_violation", f"P(violation) per constraint ({'in-set' if which == 'mc_in' else 'out-of-set'} Monte Carlo, "
             "converged samples; constraints never violated are omitted).", ["constraint"] + [LABELS[x] for x in labs], rows)


def t_oos_params(R):
    m = next((R["designs"][lab]["mc_out"] for lab in ALL if R["designs"][lab].get("mc_out")), None)
    if not m:
        return T("out_of_set_parameters", "The extended list.", ["parameter"], [])
    src = m["sources"]
    active = {"f_build", "banner_drag_factor", "m_avionics", "lap.wind", "lap.wind_dir"}
    why = {"f_build": "Q39 (USYD LOO)", "banner_drag_factor": "Q56 ([HB] Table 41)", "lap.wind": "Q39", "lap.wind_dir": "Q59", "m_avionics": "Q32",
           "f_tb": "Q32", "a_ht": "Q32 (±30 %, as f_ht)", "a_vt": "Q32 (±30 %, as f_vt)"}
    rows = [[p["name"], "in set" if p["name"] in active else "out of set", f(p["mean"], 4), f(p["std"], 4), p["kind"], src.get(p["name"]),
             why.get(p["name"], "Q32 (configs/uncertainty.yaml)")] for p in m["params"]]
    return T("out_of_set_parameters", "The parameters of the out-of-set Monte Carlo, with their sources.",
             ["parameter", "set", "mean", "sigma", "distribution", "source", "question"], rows)


def t_propagation(R):
    p = R.get("propagation")
    if not p:
        return T("propagation", "Propagation check.", ["quantity"], [])
    meth = p["methods"]
    q = list(meth["udr"]["stats"]) or list(meth["sobol"]["stats"])
    rows = []
    for name in q:
        row = [name.replace("points.", "")]
        for k in ("udr", "smolyak", "sobol"):
            st = meth.get(k, {}).get("stats", {}).get(name)
            row += [f"{st['mu']:.5g}", f"{st['sigma']:.4g}"] if st else ["–", "–"]
        s = meth.get("sobol", {}).get("stats", {}).get(name)
        row += [f"{s['se_mu']:.3g}"] if s else ["–"]
        rows.append(row)
    return T("propagation", f"Propagation at the robust optimum: UDR (N = {meth['udr']['N']}, the RDO's own rule), Smolyak level 2 "
             f"(N = {meth['smolyak']['N']}) and the in-set Sobol' Monte Carlo (N = {meth.get('sobol', {}).get('N')}); se = the plain Monte "
             "Carlo standard error of the Sobol' mean (conservative).",
             ["quantity", "UDR mean", "UDR sigma", "Smolyak mean", "Smolyak sigma", "Sobol' mean", "Sobol' sigma", "se(mean)"], rows)


def t_wind_dir(R):
    labs = [x for x in MAIN if R["designs"][x].get("wind_dir")]
    if not labs:
        return T("wind_direction", "F against direction.", ["direction"], [])
    th = R["designs"][labs[0]]["wind_dir"]["directions"]
    rows = [[f"{t:+.1f}"] + [f(R["designs"][x]["wind_dir"]["values"][i][0], 4) for x in labs] for i, t in enumerate(th)]
    return T("wind_direction", "F against the wind direction at the mean wind speed (5 m/s).", ["direction (deg)"] + [LABELS[x] for x in labs], rows)


def t_wind_env(R):
    labs = [x for x in ("mdo", "rdo") if R["designs"][x].get("wind_env") and "rows" in R["designs"][x]["wind_env"]]
    if not labs:
        return T("wind_envelope", "Wind limit.", ["direction"], [])
    th = [r["theta"] for r in R["designs"][labs[0]]["wind_env"]["rows"]]
    rows = []
    for i, t in enumerate(th):
        row = [f"{t:+.1f}"]
        for x in labs:
            r = R["designs"][x]["wind_env"]["rows"][i]
            row += [f(r["limit"], 2) if r["limit"] is not None else f"> {r['above']:g}", r["stopped_by"] or "–"]
        rows.append(row)
    return T("wind_envelope", "The wind limit against direction (bisection, 0.05 m/s): the largest speed whose analysis succeeds.",
             ["direction (deg)"] + [c for x in labs for c in (f"{LABELS[x]} limit (m/s)", "stopped by")], rows)


GEOM_ROWS = [("span b (m)", lambda n: n["dv"]["b"], 3), ("chord c (m)", lambda n: n["dv"]["c"], 3),
             ("AR", lambda n: n["dv"]["b"] / n["dv"]["c"], 2), ("wing area (m²)", lambda n: n["dv"]["b"] * n["dv"]["c"], 3),
             ("t/c scale", lambda n: n["dv"]["tc_scale"], 3), ("camber scale", lambda n: n["dv"]["camber_scale"], 3),
             ("S_ht (m²)", lambda n: n["dv"]["S_ht"], 4), ("S_vt (m²)", lambda n: n["dv"]["S_vt"], 4),
             ("tail arm l_ht (m)", lambda n: n["dv"]["l_ht"], 3), ("V_h", lambda n: n["M2"]["V_h"], 3), ("V_v", lambda n: n["M2"]["V_v"], 3),
             ("Kv (rpm/V)", lambda n: n["dv"]["Kv"], 1), ("cells", lambda n: n["cells"], 0),
             ("M2 pack (Ah)", lambda n: n["dv"]["capacity_M2"], 3), ("M3 pack (Ah)", lambda n: n["dv"]["capacity_M3"], 3),
             ("M2 pack (Wh)", lambda n: n["M2"]["E_nom_total"], 1), ("M3 pack (Wh)", lambda n: n["M3"]["E_nom_total"], 1),
             ("passengers n_p", lambda n: n["dv"]["n_p"], 1), ("cargo n_c", lambda n: n["dv"]["n_c"], 1),
             ("banner length (in)", lambda n: n["dv"]["L_banner"], 1), ("payload fuselage l_cb (m)", lambda n: n["M2"]["l_cb"], 3),
             ("x_motor (m)", lambda n: n["M2"]["x_motor"], 3),
             ("M2 take-off mass (kg)", lambda n: n["M2"]["m_total"], 3), ("M3 take-off mass (kg)", lambda n: n["M3"]["m_total"], 3),
             ("empty mass, M2 pack (kg)", lambda n: n["M2"]["m_empty"], 3),
             ("SM M2", lambda n: n["M2"]["SM"], 3), ("SM M3", lambda n: n["M3"]["SM"], 3), ("cruise M2 (m/s)", lambda n: n["M2"]["v"], 1),
             ("cruise M3 (m/s)", lambda n: n["M3"]["v"], 1), ("throttle M2", lambda n: n["dv"]["throttle_M2"], 3),
             ("throttle M3", lambda n: n["dv"]["throttle_M3"], 3), ("M2 laps", lambda n: n["M2"]["laps_frac"], 2),
             ("M3 laps", lambda n: n["M3"]["laps_frac"], 2), ("M2 score", lambda n: n["score"]["M2"], 4), ("M3 score", lambda n: n["score"]["M3"], 4),
             ("NeuralFoil confidence M2", lambda n: n["M2"]["cruise.nf_confidence_min"], 3),
             ("NeuralFoil confidence M3", lambda n: n["M3"]["cruise.nf_confidence_min"], 3)]


def t_designs(R):
    rows = []
    labs = [x for x in MAIN if nominal(R, x)]
    for name, fn, nd in GEOM_ROWS:
        rows.append([name] + [f(float(fn(nominal(R, x))), nd) for x in labs])
    rows.insert(12, ["props M2 / M3"] + [f"{nominal(R, x)['props']['M2']} / {nominal(R, x)['props']['M3']}" for x in labs])
    rows.insert(7, ["vertical tail height / chord (m)"] + [
        f"{(nominal(R, x)['dv']['S_vt'] * nominal(R, x)['details']['geometry']['AR_vt']) ** 0.5:.3f} / "
        f"{(nominal(R, x)['dv']['S_vt'] / nominal(R, x)['details']['geometry']['AR_vt']) ** 0.5:.3f}" for x in labs])
    return T("design_table", "The designs side by side.", ["quantity"] + [LABELS[x] for x in labs], rows)


def t_active(R):
    rows = []
    for lab in MAIN:
        n = nominal(R, lab)
        if n:
            rows.append([LABELS[lab], "; ".join(active_list(n)) or "none"])
    return T("active_constraints", f"Active constraints and bounds ({TOL_ACTIVE}), each marked as rule, cited, model or PLACEHOLDER.",
             ["design", "active"], rows)


MASS_ORDER = [("wing spar tube", lambda d, n: (d["M2"]["m_tube"] or 0) * _wf(d, n)),
              ("wing ribs, LE/TE, film", lambda d, n: ((d["M2"]["m_rib"] or 0) + (d["M2"]["m_le"] or 0) + (d["M2"]["m_te"] or 0) + (d["M2"]["m_film"] or 0)) * _wf(d, n)),
              ("tails", lambda d, n: n["M2"]["m_ht"] + n["M2"]["m_vt"]), ("tail boom", lambda d, n: d["M2"]["m_tb"]),
              ("payload fuselage", lambda d, n: d["M2"]["m_cb"]), ("landing gear", lambda d, n: n["M2"]["m_gear"]),
              ("motor, ESC, prop", lambda d, n: d["M2"]["m_motor_group"]), ("battery (M2 pack)", lambda d, n: n["M2"]["m_batt"]),
              ("avionics", lambda d, n: d["M2"]["m_avionics"]), ("M2 payload", lambda d, n: n["M2"]["m_pay"])]


def _wf(d, n):
    bare = d["M2"]["m_wing_bare"]
    return n["M2"]["m_wing"] / bare if bare else 1.0


def mass_breakdown(R, lab):
    n = nominal(R, lab)
    d = n["details"]
    return [(name, float(fn(d, n))) for name, fn in MASS_ORDER]


def t_mass(R):
    labs = [x for x in MAIN if nominal(R, x)]
    rows = []
    br = {x: mass_breakdown(R, x) for x in labs}
    for i, (name, _) in enumerate(MASS_ORDER):
        rows.append([name] + [f(br[x][i][1], 3) for x in labs])
    rows.append(["**sum (M2 take-off)**"] + [f(sum(v for _, v in br[x]), 3) for x in labs])
    rows.append(["model M2 take-off mass"] + [f(nominal(R, x)["M2"]["m_total"], 3) for x in labs])
    return T("mass_breakdown", "Mass breakdown at M2 (kg), in the order of `plotting.figures.mass_breakdown`.", ["component"] + [LABELS[x] for x in labs], rows)


DRAG_PARTS = [("induced (VLM)", "cruise.vlm.wing_perf.CDi"), ("wing profile", "cruise.CDp"), ("tails", "CD_tail"), ("payload fuselage", "parasite.CD_cb"),
              ("tail boom", "parasite.CD_boom"), ("gear + leakage areas", "CD_areas"), ("banner (deployed)", "CD_stores")]


def drag_parts(R, lab, pt):
    d = nominal(R, lab)["details"][pt]
    return [(name, float(d.get(k) or 0.0)) for name, k in DRAG_PARTS]


def t_drag(R):
    labs = [x for x in MAIN if nominal(R, x)]
    rows = []
    for pt in ("M2", "M3"):
        br = {x: drag_parts(R, x, pt) for x in labs}
        for i, (name, _) in enumerate(DRAG_PARTS):
            if pt == "M2" and name.startswith("banner"):
                continue
            rows.append([f"{pt} {name}"] + [f(br[x][i][1], 4) for x in labs])
        rows.append([f"**{pt} total at cruise**"] + [f(sum(v for _, v in br[x]), 4) for x in labs])
    return T("drag_breakdown", "Drag breakdown at cruise, on the wing area: M2 and M3 (banner deployed).", ["component"] + [LABELS[x] for x in labs], rows)


def t_enum(R):
    rows = []
    for d in R["enumeration"]:
        rows.append([d["tag"].replace("fr_", ""), f(d["cells"], 0), d["props"]["M2"] if d.get("props") else "–", d["props"]["M3"] if d.get("props") else "–", f(d.get("ok")),
                     d.get("its"), f(d.get("F"), 4), f(d.get("floored"), 4), f(d.get("M2"), 4), f(d.get("M3"), 4), f((d.get("dv") or {}).get("n_p"), 1),
                     f((d.get("dv") or {}).get("L_banner"), 0), f((d.get("dv") or {}).get("Kv"), 0), d.get("kv_set_by") or "–",
                     ", ".join(_short_active(x) for x in (d.get("active_constraints") or [])) + (" | " + ", ".join(d.get("active_bounds") or []) if d.get("active_bounds") else "")
                     or (d.get("error") or "")[:60]])
    return T("enumeration", "The propulsion enumeration: 9 prop pairs at 8S and the best pair at 6S, each from Humpback's start.",
             ["case", "cells", "M2 prop", "M3 prop", "converged", "its", "F", "F floored", "M2", "M3", "n_p", "banner (in)", "Kv", "Kv set by", "active constraints | bounds"], rows)


def _short_active(c):
    return c.replace("points.", "").replace(" (lower)", "").replace(" (upper)", " up")


def t_integer(R):
    rows = []
    for d in R["integer"] + R["wind_integer"]:
        rows.append([d["tag"].replace("fr_", ""), f(d.get("ok")), d.get("its"), f(d.get("F"), 4), f(d.get("uncapped"), 4), f(d.get("floored"), 4),
                     f(d.get("M2"), 4), f(d.get("M3"), 4), ", ".join(d.get("violated") or []) or "none", f(d.get("time"), 0)])
    return T("integer_rounding", "Payload rounding: each candidate that fits, re-optimised with the counts fixed.",
             ["run", "converged", "its", "F", "F uncapped", "F floored", "M2", "M3", "violated", "wall (s)"], rows)


def t_multistart(R):
    rows = []
    for d in R["multistart"]:
        rows.append([d["tag"].replace("fr_", ""), f(d.get("ok")), d.get("its"), f(d.get("F"), 4), f(d.get("floored"), 4), f((d.get("dv") or {}).get("n_p"), 1),
                     f((d.get("dv") or {}).get("L_banner"), 0), f((d.get("dv") or {}).get("Kv"), 0), f(d.get("time"), 0), (d.get("error") or "")[:70]])
    return T("multistart", "Multistart: 10 scrambled Sobol' starts in the design box, the best case.",
             ["start", "converged", "its", "F", "F floored", "n_p", "banner (in)", "Kv", "wall (s)", "error"], rows)


def multistart_summary(R):
    ms = R["multistart"]
    if not ms:
        return None
    best = next((d for d in R["enumeration"] if d["tag"] == R["best_case"]), None)
    good = [d for d in ms if d.get("ok") and d.get("F") is not None and not d.get("violated")]
    fs = sorted(d["F"] for d in good)
    clusters = []
    for x in fs:
        if not clusters or x - clusters[-1][-1] > OK_F:
            clusters.append([x])
        else:
            clusters[-1].append(x)
    dvs = list(good[0]["dv"]) if good else []
    box = S.config().all_dvs()
    top = [d for d in good if abs(d["F"] - fs[-1]) <= OK_F]
    spread = {}
    for k in dvs:
        vals = [d["dv"][k] for d in top]
        rng = box[k].upper - box[k].lower
        spread[k] = dict(min=min(vals), max=max(vals), rel=(max(vals) - min(vals)) / rng if rng else 0.0)
    return dict(n=len(ms), converged=len(good), failed=[d["tag"] for d in ms if d not in good],
                errors={d["tag"]: (d.get("error") or ("violated: " + "; ".join(d.get("violated") or [])) or "SLSQP did not converge") for d in ms if d not in good},
                clusters=[dict(F_min=c[0], F_max=c[-1], n=len(c)) for c in clusters], best_F=fs[-1] if fs else None,
                reference_F=best["F"] if best else None, spread=spread)


def t_bookends(R):
    rows = []
    base = next((d for d in R["enumeration"] if d["tag"] == R["best_case"]), None)
    for name, d in (("multipoint (M2 + M3)", base), ("M2 only", R["bookends"].get("M2only")), ("M3 only", R["bookends"].get("M3only"))):
        if d:
            dv = d.get("dv") or {}
            rows.append([name, f(d.get("ok")), d.get("its"), f(d.get("M2"), 4), f(d.get("M3"), 4), f(d.get("r_M2"), 3), f(d.get("r_M3"), 3), f(dv.get("n_p"), 1),
                         f(dv.get("L_banner"), 0), f(dv.get("c"), 3), f(d.get("laps_M2"), 2), f(d.get("laps_M3"), 2)])
    return T("bookends", "Mission bookends: the same problem with the objective reduced to one mission's score. r = the uncapped ratio.",
             ["objective", "converged", "its", "M2", "M3", "r_M2", "r_M3", "n_p", "banner (in)", "c (m)", "M2 laps", "M3 laps"], rows)


def t_norm(R):
    rows = []
    base = next((d for d in R["enumeration"] if d["tag"] == R["best_case"]), None)
    if base:
        rows.append(["NI_max 2800, B_max 1500 (main)", f(base["ok"]), base["its"], f(base["F"], 4), f(base["M2"], 4), f(base["M3"], 4), f(base["dv"]["n_p"], 1), f(base["dv"]["L_banner"], 0), "–"])
    names = {"NI0.5": "NI_max x0.5 (1400)", "NI2": "NI_max x2 (5600)", "B0.5": "B_max x0.5 (750)", "B2": "B_max x2 (3000)"}
    for k, name in names.items():
        d = R["normalisation"].get(k)
        if d:
            rows.append([name, f(d.get("ok")), d.get("its"), f(d.get("F"), 4), f(d.get("M2"), 4), f(d.get("M3"), 4), f((d.get("dv") or {}).get("n_p"), 1),
                         f((d.get("dv") or {}).get("L_banner"), 0), signed(d["F"] - base["F"]) if base and d.get("F") else "–"])
    return T("normalisation", "Normalisation (PLACEHOLDERS, Q52): re-optimised from the best case.", ["normalisation", "converged", "its", "F", "M2", "M3", "n_p", "banner (in)", "ΔF"], rows)


def bound_lines(R):
    """Each bound sensitivity as one line for the thesis, with the design's move."""
    mdo = rd(R["mdo_tag"]) if R["mdo_tag"] else None
    if not mdo:
        return []
    F0 = mdo["score"]["flight_score"]
    names = {"i": "(i) c <= 0.60 m and camber_scale <= 2.5", "ii": "(ii) V_h >= 0.50, V_v >= 0.04 (Raymer 6th ed., Table 6.4, p. 160, homebuilt)",
             "iii": "(iii) V_h >= 0.66 ([OH]), V_v >= 0.059 ([W]), the lowest flown",
             "banner": "(added) banner <= 500 in, AirCrafter 1.0's grid limit ([HB] Table 7), against the 275 in the team built (Q56)"}
    out = []
    for k, name in names.items():
        d = R["bounds"].get(k)
        if not d or d.get("F") is None:
            out.append(dict(key=k, name=name, missing=(d or {}).get("error") or "missing"))
            continue
        moved = {x: (mdo["dv"][x], d["dv"][x]) for x in mdo["dv"] if abs(mdo["dv"][x] - d["dv"][x]) > 0.02 * max(abs(mdo["dv"][x]), 1e-6)}
        nf = min(d["nf_M2"], d["nf_M3"])
        out.append(dict(key=k, name=name, ok=d["ok"], its=d["its"], F=d["F"], dF=d["F"] - F0, uncapped=d["uncapped"], floored=d["floored"], moved=moved, nf=nf,
                        flag=nf < NF_FLAG, violated=d["violated"], c=d["dv"]["c"], camber=d["dv"]["camber_scale"], S_ht=d["dv"]["S_ht"], S_vt=d["dv"]["S_vt"], V_h=None))
    return out


def t_bounds(R):
    rows = []
    for x in bound_lines(R):
        if "missing" in x:
            rows.append([x["name"], "missing"] + [""] * 7)
            continue
        rows.append([x["name"], f(x["ok"]), f(x["F"], 4), signed(x["dF"]), f(x["floored"], 4), f(x["c"], 3), f(x["camber"], 3), f(x["nf"], 3) + (" **untrustworthy**" if x["flag"] else ""),
                     ", ".join(x["violated"]) or "none"])
    return T("bound_sensitivities", "Bound sensitivities: warm re-optimisations from the MDO design, payload counts fixed. NeuralFoil's "
             f"confidence below {NF_FLAG} flags a result as untrustworthy (Q10).",
             ["bound", "converged", "F", "ΔF", "F floored", "c (m)", "camber scale", "NeuralFoil confidence", "violated"], rows)


def t_others(R):
    rows = []
    mdo = rd(R["mdo_tag"]) if R["mdo_tag"] else None
    F0 = mdo["score"]["flight_score"] if mdo else None
    for k, name in (("old_constraints", "old constraint set (rpm 0.85, SM_max 0.40, V_h_max 0.65, V_v_max 0.06), re-optimised"), ("takeoff_off", "take-off off, re-optimised")):
        d = R["others"].get(k)
        if d:
            rows.append([name, "MDO", f(d.get("ok")), f(d.get("F"), 4), signed(d["F"] - F0) if d.get("F") else "–", f(d.get("floored"), 4), ", ".join(d.get("violated") or []) or "none"])
    for lab in ("mdo", "rdo"):
        re_ = R["designs"][lab].get("reanalysis")
        n = nominal(R, lab)
        if re_ and n:
            F = n["score"]["flight_score"]
            for name, x in re_["cases"].items():
                nm = f"C_D,b {name}" if "rpm" not in name else name
                rows.append([nm + " (re-analysis)", LABELS[lab], f(x.get("ok")), f(x.get("F"), 4), signed(x["F"] - F) if x.get("F") else "–", f(x.get("floored"), 4),
                             ", ".join(y.replace("points.", "") for y in x.get("violated") or []) or x.get("error") or "none"])
    return T("other_sensitivities", "Other sensitivities. Re-analyses hold the design (and its M3 throttle, unless stated); a sub-stall balance shows as a violated stall margin.",
             ["case", "design", "ok", "F", "ΔF", "F floored", "violated"], rows)


def placeholder_rows(R):
    ph = R.get("placeholders")
    if not ph:
        return []
    F0, U0 = ph["nominal"]["F"], ph["nominal"]["F_uncapped"]
    out = []
    for rng in ph["ranges"]:
        lo = next(r for r in ph["rows"] if r["path"] == rng["path"] and r["side"] == "lo")
        hi = next(r for r in ph["rows"] if r["path"] == rng["path"] and r["side"] == "hi")
        p0 = ph["nominal_values"][rng["path"]]
        ok = lo.get("ok") and hi.get("ok")
        row = dict(name=rng["name"], path=rng["path"], lo=rng["lo"], hi=rng["hi"], nominal=p0, why=rng["why"], ok=ok, error=lo.get("error") or hi.get("error"))
        if ok:
            slope = (hi["F"] - lo["F"]) / (rng["hi"] - rng["lo"])
            slope_u = (hi["F_uncapped"] - lo["F_uncapped"]) / (rng["hi"] - rng["lo"])
            row.update(F_lo=lo["F"], F_hi=hi["F"], effect=max(abs(hi["F"] - F0), abs(lo["F"] - F0), abs(hi["F"] - lo["F"])), range_effect=abs(hi["F"] - lo["F"]),
                       range_effect_u=abs(hi["F_uncapped"] - lo["F_uncapped"]), normalised=slope * p0 / F0, normalised_u=slope_u * p0 / U0,
                       violated_lo=lo["violated"], violated_hi=hi["violated"])
        out.append(row)
    return out


def ranking(R):
    """The 2025-26 PLACEHOLDER ranking by contribution to F. Measures differ by row and are stated (the method of the
    2024-25 INDEX): a re-analysis over the range (|dF/dp| x range as a finite difference, at the MDO design); a
    re-optimisation; or the first-order sigma of F that a PLACEHOLDER spread gives."""
    rows = []
    for r in placeholder_rows(R):
        if r["ok"]:
            rows.append(dict(name=r["name"], effect=r["range_effect"], effect_uncapped=r["range_effect_u"], measure=f"re-analysis over {r['lo']:g} to {r['hi']:g} ({r['why']})"))
    base = next((d for d in R["enumeration"] if d["tag"] == R["best_case"]), None)
    for k, d in R["normalisation"].items():
        if d and d.get("F") and base:
            rows.append(dict(name=f"Q52 normalisation {k}", effect=abs(d["F"] - base["F"]), effect_uncapped=abs(d["uncapped"] - base["uncapped"]), measure="F re-optimised; mostly a rescaling of the score"))
    for x in bound_lines(R):
        if "missing" not in x:
            rows.append(dict(name=f"bound {x['name']}", effect=abs(x["dF"]), effect_uncapped=None, measure="F re-optimised, payload counts fixed"))
    sc = R["designs"]["mdo"].get("screen")
    if sc:
        for p, var in sc["oned"].items():
            if sc["sources"].get(p) == "PLACEHOLDER" and var is not None:
                rows.append(dict(name=f"uncertainty spread {p} (Q32, Q39)", effect=float(np.sqrt(var)), effect_uncapped=None, measure="sigma of F, 3-point Gauss, this parameter alone, at the MDO design"))
    return sorted(rows, key=lambda r: -r["effect"])


def t_ranking(R):
    return T("placeholder_ranking", "The 2025-26 PLACEHOLDER ranking by contribution to F at the MDO design. Measures differ by row, as stated. M2 sits in the cap's blend at the optimum, so a "
             "re-analysis that raises M2 barely moves capped F: the uncapped column shows it.",
             ["rank", "PLACEHOLDER", "effect on F", "effect on uncapped F", "measure"],
             [[i + 1, r["name"], f(r["effect"], 4), f(r.get("effect_uncapped"), 4), r["measure"]] for i, r in enumerate(ranking(R))])


def t_normalised(R):
    rows = [[r["name"], r["path"], f"{r['nominal']:g}", f"{r['lo']:g}", f"{r['hi']:g}", f(r.get("F_lo"), 4), f(r.get("F_hi"), 4), f(r.get("normalised"), 4), f(r.get("normalised_u"), 4), r["why"] if r["ok"] else r["error"]]
            for r in sorted(placeholder_rows(R), key=lambda r: -abs(r.get("normalised") or 0.0))]
    return T("normalised_sensitivities", "Normalised sensitivities (p/F) dF/dp of F to the placeholders, by finite difference over each range at the MDO design, capped and uncapped.",
             ["placeholder", "config path", "nominal", "low", "high", "F low", "F high", "(p/F) dF/dp", "uncapped", "range"], rows)


def t_rdo(R):
    rows = []
    for r in R["rdo"]:
        s = (r.get("summary") or {}).get("flight_score", {})
        fb = r.get("fallbacks") or []
        rows.append([r["tag"].replace("fr_", ""), (r.get("start") or "").replace("fr_", ""), f(r.get("ok")), r.get("its"), r.get("nit"), f((r.get("time") or 0) / 3600, 2), f(s.get("mu"), 4), f(s.get("sigma"), 4),
                     f(s.get("robust"), 4), f((r.get("dv") or {}).get("n_p"), 2), f((r.get("dv") or {}).get("n_c"), 2), f((r.get("dv") or {}).get("L_banner"), 0), sum(x["continuation"] for x in fb), sum(x["failed_attempts"] for x in fb), (r.get("error") or "")[:60]])
    return T("rdo_starts", "The robust runs: three starts and the robust rounding. Fallbacks are summed over the 11 nodes x 2 points.",
             ["run", "start", "converged", "its", "nit", "wall (h)", "mean", "sigma", "mean - k sigma", "n_p", "n_c", "banner (in)", "continuations", "failed attempts", "error"], rows)


def t_tradeoff(R):
    rows = []
    wall = design_walls(R)
    for lab in ("mdo", "mdo_wind", "rdo"):
        n, m = nominal(R, lab), R["designs"][lab].get("mc_in")
        st = stat(R, lab)
        rows.append([LABELS[lab], f(n["score"]["flight_score"], 4) if n else "–", f(st.get("mu"), 4), f(st.get("sigma"), 4), f(st.get("robust"), 4), f((m or {}).get("p_any"), 3), f((wall.get(lab) or 0) / 60, 1)])
    return T("robustness_tradeoff", "The trade-off: nominal F, Monte Carlo statistics, P(any violation) and the wall time of the runs that produced the design (3 threads, not clean; clean timings in compute_performance_2026.md).",
             ["design", "F nominal", "MC mean", "MC sigma", "mean - k sigma", "P(any violation)", "wall (min)"], rows)


def design_walls(R):
    out = {}
    base = next((d for d in R["enumeration"] if d["tag"] == R["best_case"]), None)
    if base:
        out["mdo"] = (base.get("time") or 0) + sum(d.get("time") or 0 for d in R["integer"])
    if R["wind_cont"]:
        out["mdo_wind"] = (R["wind_cont"].get("time") or 0) + sum(d.get("time") or 0 for d in R["wind_integer"])
    runs = {r["tag"]: r for r in R["rdo"]}
    if R["rdo_best"]:
        cont = R["rdo_best"].split("_int_")[0]
        out["rdo"] = (runs[cont].get("time") or 0) + (sum(r.get("time") or 0 for t, r in runs.items() if t.startswith(f"{cont}_int_")))
    return out


def decomposition(R):
    a, b, c = (nominal(R, x) for x in ("mdo", "mdo_wind", "rdo"))
    sa, sb, sc = (stat(R, x) for x in ("mdo", "mdo_wind", "rdo"))
    if not (a and b and c and sa and sb and sc):
        return None
    pa, pb, pc = (R["designs"][x]["mc_in"]["p_any"] for x in ("mdo", "mdo_wind", "rdo"))
    Fa, Fb, Fc = (x["score"]["flight_score"] for x in (a, b, c))
    return dict(nominal=dict(mdo=Fa, mdo_wind=Fb, rdo=Fc, wind=Fb - Fa, robust=Fc - Fb, total=Fc - Fa), mu=dict(mdo=sa["mu"], mdo_wind=sb["mu"], rdo=sc["mu"], wind=sb["mu"] - sa["mu"], robust=sc["mu"] - sb["mu"], total=sc["mu"] - sa["mu"]),
                sigma=dict(mdo=sa["sigma"], mdo_wind=sb["sigma"], rdo=sc["sigma"], wind=sb["sigma"] - sa["sigma"], robust=sc["sigma"] - sb["sigma"], total=sc["sigma"] - sa["sigma"]),
                p_any=dict(mdo=pa, mdo_wind=pb, rdo=pc, wind=pb - pa, robust=pc - pb, total=pc - pa))


def t_decomp(R):
    d = decomposition(R)
    if not d:
        return T("robustness_decomposition", "Decomposition.", ["quantity"], [])
    rows = [[name, f(d[k]["mdo"], 4), f(d[k]["mdo_wind"], 4), f(d[k]["rdo"], 4), signed(d[k]["wind"]), signed(d[k]["robust"]), signed(d[k]["total"])]
            for name, k in (("F nominal (calm air)", "nominal"), ("MC mean of F", "mu"), ("MC sigma of F", "sigma"), ("P(any violation)", "p_any"))]
    return T("robustness_decomposition", "What robustness costs: mdo_wind - mdo is the effect of designing at the mean wind, rdo - mdo_wind the effect of robustness itself.",
             ["quantity", "MDO", "MDO, mean wind", "MDO + RDO", "designing at the mean wind", "robustness itself", "total"], rows)


def t_screen(R):
    labs = [x for x in ("mdo", "rdo") if R["designs"][x].get("screen")]
    if not labs:
        return T("screening", "Screening.", ["parameter"], [])
    first = R["designs"][labs[0]]["screen"]
    params = sorted(first["contrib"]["flight_score"], key=lambda p: -(first["oned"].get(p) or 0.0))
    active = set(first["active"])
    rows = []
    for p in params:
        row = [p, "in set" if p in active else "out of set", first["sources"].get(p)]
        for x in labs:
            sc = R["designs"][x]["screen"]
            row += [f"{sc['contrib']['flight_score'][p] ** 0.5:.4f}", f"{sc['oned'][p] ** 0.5:.4f}" if sc["oned"].get(p) is not None else "failed"]
        rows.append(row)
    return T("screening", "Each parameter's contribution to the sigma of F at the set's means: first order (|dF/dp| sigma) and 3-point Gauss (this parameter alone, sees the cap and the curvature).",
             ["parameter", "set", "source"] + [c for x in labs for c in (f"{LABELS[x]}: first order", f"{LABELS[x]}: 1-D Gauss")], rows)


def t_mapping(R):
    return T("ac1_mapping", "What AirCrafter 1.0 sets and what AirCrafter 2.0 fills in.", ["item", "mapping (a), direct", "mapping (b), completed"], [list(r) for r in (R.get("ac1_table") or [])])


def overlap(R, lab):
    """The box-boom overlap (Q31): the boom's length inside the payload fuselage, and the mass and drag counted twice."""
    n = nominal(R, lab)
    if not n:
        return None
    g, d = n["details"]["geometry"], n["details"]["M2"]
    x0 = 0.25 * g["c"] + g["boom_offset"]
    length = max(0.0, min(g["x_cb_aft"] - x0, g["l_tb"]))
    share = length / g["l_tb"]
    return dict(length=length, share=share, mass=(d["m_tb"] or 0.0) * share, drag=(d.get("parasite.CD_boom") or 0.0) * share,
                x_cb_aft=g["x_cb_aft"], boom_start=x0)


def t_overlap(R):
    rows = []
    for lab in MAIN:
        o = overlap(R, lab)
        if o:
            rows.append([LABELS[lab], f(o["length"], 3), f(100 * o["share"], 0), f(1000 * o["mass"], 0), f(1e4 * o["drag"], 1)])
    return T("q31_overlap", "The box-boom overlap (Q31): boom length inside the payload fuselage and what is counted twice (linear share of the boom).",
             ["design", "overlap (m)", "share of the boom (%)", "mass counted twice (g)", "M2 drag counted twice (counts)"], rows)


def tables(R):
    ts = [t_method(R), t_nominal_loo(R), t_nominal(R), t_constraints(R), t_mc(R, "mc_in"), t_pv(R, "mc_in"), t_mc(R, "mc_out"), t_pv(R, "mc_out"), t_oos_params(R), t_propagation(R),
          t_wind_dir(R), t_wind_env(R), t_designs(R), t_active(R), t_mass(R), t_drag(R), t_enum(R), t_integer(R), t_multistart(R), t_bookends(R), t_norm(R), t_bounds(R), t_others(R), t_ranking(R),
          t_normalised(R), t_rdo(R), t_tradeoff(R), t_decomp(R), t_screen(R), t_mapping(R), t_overlap(R)]
    return {t.name: t for t in ts}


# ----------------------------------------------------------------------------------------------------- generated text
def sub_stall(R, lab):
    """How many converged in-set samples sit on the low-speed balance (M3 stall margin below zero), from the arrays."""
    fz = S.MC_DIR / f"fr_mc_in_{lab}.npz"
    if not fz.exists():
        return None
    z = np.load(fz)
    names = [str(x) for x in z["outputs"]]
    V = z["values"]
    ok = ~np.isnan(V).any(axis=1)
    bad = V[ok, names.index("points.M3.stall_margin")] < 0
    F = V[ok, 0]
    return dict(n=int(bad.sum()), of=int(ok.sum()), mu_without=float(F[~bad].mean()) if (~bad).any() else None, mu=float(F.mean()), F_max_bad=float(F[bad].max()) if bad.any() else None)


def design_paragraph(R, lab):
    n, ref = nominal(R, lab), nominal(R, "mdo")
    if not n:
        return f"**{LABELS[lab]}:** missing."
    limit = "; ".join(active_list(n)) or "no constraint or bound is active"
    bits = []
    if ref and lab != "mdo":
        for name, k, nd in (("span", "b", 2), ("chord", "c", 3), ("passengers", "n_p", 0), ("cargo", "n_c", 0), ("banner", "L_banner", 0), ("S_ht", "S_ht", 3), ("S_vt", "S_vt", 3)):
            a, b = n["dv"][k], ref["dv"][k]
            if abs(a - b) > 0.02 * max(abs(b), 1e-9):
                bits.append(f"{name} {a:.{nd}f} against {b:.{nd}f}")
    sc = score_of(n)
    return (f"**{LABELS[lab]}** (F {sc['F']:.4f}; M2 {sc['M2']:.3f}, r {sc['r_M2']:.3f}; M3 {sc['M3']:.3f}, r {sc['r_M3']:.3f}; floored {sc['floored']:.4f}). What limits it: {limit}."
            + (f" Against MDO: {', '.join(bits)}." if bits else ""))


def findings(R):
    F = []
    get = lambda lab: nominal(R, lab)  # noqa: E731
    mdo, wind, rob, hb, a1, a2 = (get(x) for x in ("mdo", "mdo_wind", "rdo", "humpback", "ac1_direct", "ac1_completed"))
    base = next((d for d in R["enumeration"] if d["tag"] == R["best_case"]), None)
    if base and mdo:
        F.append(f"**Best propulsion case: {base['tag'].replace('fr_', '')}.** After payload rounding (MDO): F = {mdo['score']['flight_score']:.4f} capped, "
                 f"{score_of(mdo)['uncapped']:.4f} uncapped, {score_of(mdo)['floored']:.4f} floored, with M2 {mdo['score']['M2']:.4f}, M3 {mdo['score']['M3']:.4f}, "
                 f"{mdo['dv']['n_p']:.0f} passengers, {mdo['dv']['n_c']:.0f} cargo, a {mdo['dv']['L_banner']:.0f} in banner.")
    good = [d for d in R["enumeration"] if d.get("F") is not None and d.get("ok")]
    if len(good) > 1:
        fs = sorted(good, key=lambda d: -d["F"])
        F.append(f"The enumeration's spread is {fs[0]['F'] - fs[-1]['F']:.4f} in F over {len(good)} converged cases ({fs[0]['tag'].replace('fr_', '')} first, {fs[-1]['tag'].replace('fr_', '')} last); "
                 f"{len(R['enumeration']) - len(good)} did not converge.")
    if hb and mdo:
        F.append(f"**Against Humpback as built ({hb['score']['flight_score']:.4f} in the same model):** MDO gains {mdo['score']['flight_score'] - hb['score']['flight_score']:+.4f}, through payload ({hb['dv']['n_p']:.0f}/{hb['dv']['n_c']:.0f} to "
                 f"{mdo['dv']['n_p']:.0f}/{mdo['dv']['n_c']:.0f}) and banner ({hb['dv']['L_banner']:.0f} to {mdo['dv']['L_banner']:.0f} in). The common evaluation is AirCrafter 2.0's own model, so it favours this; and the Ground Mission, which grows with the counts, isn't in F (Q53).")
    if a1 and a2 and mdo:
        F.append(f"**Against AirCrafter 1.0's selection in the same model:** direct (a) {a1['score']['flight_score']:.4f}, completed (b) {a2['score']['flight_score']:.4f}, MDO {mdo['score']['flight_score']:.4f}: "
                 f"{mdo['score']['flight_score'] - a2['score']['flight_score']:+.4f} over (b), the like-for-like comparison (the same model, optimised around AirCrafter 1.0's span, chord, counts and banner).")
    d = decomposition(R)
    if d:
        F.append(f"**What robustness costs.** Nominal F: {d['nominal']['wind']:+.4f} from designing at the mean wind, {d['nominal']['robust']:+.4f} from robustness itself. Monte Carlo mean: {d['mu']['wind']:+.4f} and {d['mu']['robust']:+.4f}. "
                 f"Sigma: {d['sigma']['mdo']:.4f} (MDO) to {d['sigma']['rdo']:.4f} (RDO). P(any violation): {d['p_any']['mdo']:.3f} to {d['p_any']['mdo_wind']:.3f} to {d['p_any']['rdo']:.3f}.")
    ms = multistart_summary(R)
    if ms:
        F.append(f"**Multistart ({ms['n']} Sobol' starts):** {ms['converged']} converge feasibly, to {len(ms['clusters'])} distinct optimum/optima (F within {OK_F:g}): "
                 + "; ".join(f"F {c['F_min']:.4f}–{c['F_max']:.4f} ({c['n']})" for c in ms["clusters"]) + f". The default start's F is {f(ms['reference_F'], 4)}. The others: "
                 + ("; ".join(f"{k.replace('fr_', '')}: {str(v)[:70]}" for k, v in ms["errors"].items()) or "none") + ".")
    bl = bound_lines(R)
    if bl:
        F.append("**The bound-driven part.** " + " ".join(
            (f"{x['name']}: ΔF {x['dF']:+.4f}" + (f" (NeuralFoil {x['nf']:.2f}: untrustworthy)" if x.get("flag") else "") + ".") if "missing" not in x else f"{x['name']}: missing." for x in bl))
    p = R.get("propagation")
    if p and p["methods"]["udr"]["stats"] and "sobol" in p["methods"]:
        u, sm, so = (p["methods"][k]["stats"]["flight_score"] for k in ("udr", "smolyak", "sobol"))
        F.append(f"**Propagation at the robust optimum:** mean of F {u['mu']:.4f} (UDR), {sm['mu']:.4f} (Smolyak), {so['mu']:.4f} (Sobol'); sigma {u['sigma']:.4f}, {sm['sigma']:.4f}, {so['sigma']:.4f}.")
    tot = {x: (R["designs"][x]["mc_in"]["mda_failed"], R["designs"][x]["mc_in"]["wind_limit"], R["designs"][x]["mc_in"]["N"]) for x in ("humpback", "mdo", "mdo_wind", "rdo") if R["designs"][x].get("mc_in")}
    if tot:
        F.append("**Analysis failures are counted apart from violations:** in-set Monte Carlo, MDA failures / lap-model limits out of N: " + "; ".join(f"{LABELS[x]} {a} / {b} of {n}" for x, (a, b, n) in tot.items()) + ". The statistics rest on the samples that converged.")
    re_m, re_r = (R["designs"][x].get("reanalysis") for x in ("mdo", "rdo"))
    if re_m and re_r:
        bad = lambda r: [k for k, x in r["cases"].items() if "rpm" not in k and "full" not in k and (not x.get("ok") or x.get("violated"))]  # noqa: E731
        F.append(f"**Banner drag at fixed throttle:** across the {len([k for k in re_m['cases'] if 'rpm' not in k and 'full' not in k])} C_D,b values re-analysed, MDO loses level flight or a constraint at {len(bad(re_m))} and RDO at {len(bad(re_r))} (§5.4). MDO's limit on the placeholder banner bound leaves it no margin on M3's low-speed balance.")
    y = R.get("y2025")
    if y and mdo and d:
        F.append(f"**Where this differs from 2024-25 (Appendix A).** There the multipoint gain over the as-built aircraft was small (Woombah {f(y['woombah_F'], 4)}, MDO {y['F']:.4f}) and the optimum was tank-limited on M2; here MDO is {mdo['score']['flight_score'] - hb['score']['flight_score']:+.4f} over Humpback, and M2 sits at the score's cap, so the "
                 f"cap and the fuselage's payload law set the payload. There robustness cost {y['nominal_cost']:.4f} in nominal F and took the largest P(violation) from {y['max_pv_mdo']:.3f} to {y['max_pv_rdo']:.3f}; here it costs {-(d['nominal']['total']):.4f} and takes P(any) from {d['p_any']['mdo']:.3f} to {d['p_any']['rdo']:.3f}.")
    return F


def limitations(R):
    """Biases and limitations, one row per item, with the measured size at the final designs."""
    rows = []
    V = R.get("validation") or {}
    loo = (V.get("variants") or {}).get("loo", {}).get("cases", {})
    gaps = []
    for name in ("M3_test1", "M3_test2", "M3_test3", "M2_test1", "M2_test2"):
        c = loo.get(name, {})
        s = c.get("snapshot", {}).get(c.get("table42", {}).get("point"), {})
        if s:
            gaps.append(f"{name.replace('_', ' ')} {100 * (s['otherLapTime'] / c['table42']['lap_s'] - 1):+.0f} %")
    cd0 = loo.get("M3_test3", {}).get("snapshot", {}).get("M3", {}).get("CD0")
    rows.append(["M3 lap-time and C_D0 gap", "validation_2026.md §2-3; Q56, Q57", f"Lap errors (model minus test, negative = model too fast): {', '.join(gaps) or 'missing'}. Deployed C_D0 {f(cd0, 3)} at the 197 in test against Table 29's 0.36."])
    bl = {x["key"]: x for x in bound_lines(R)}
    vals = "; ".join(f"{LABELS[x]} V_h {nominal(R, x)['M2']['V_h']:.3f}, V_v {nominal(R, x)['M2']['V_v']:.3f}" for x in MAIN if nominal(R, x))
    ii, iii = bl.get("ii", {}), bl.get("iii", {})
    rows.append(["Tail volumes at their PLACEHOLDER minimums; no directional-stability or crosswind check", "§5.3; Q9, Q60", f"{vals}. Raymer's typical values cost ΔF {signed(ii.get('dF'))}, the lowest flown {signed(iii.get('dF'))}. Directional stability isn't checked: not measured."])
    i = bl.get("i", {})
    rows.append(["Chord and camber bounds", "§5.3; Q10", f"Relaxing c to 0.60 m and camber_scale to 2.5 is worth ΔF {signed(i.get('dF'))}, NeuralFoil confidence {f(i.get('nf'), 3)}" + (" (below 0.8: untrustworthy)." if i.get("flag") else ".")])
    ov = "; ".join(f"{LABELS[x]} {overlap(R, x)['length']:.2f} m, {1000 * overlap(R, x)['mass']:.0f} g, {1e4 * overlap(R, x)['drag']:.1f} counts" for x in MAIN if overlap(R, x))
    rows.append(["The box-boom overlap counted twice", "§7 table; Q31", f"Overlap, mass and M2 drag counted twice: {ov}."])
    counts = "; ".join(f"{LABELS[x]} {nominal(R, x)['dv']['n_p']:.0f}/{nominal(R, x)['dv']['n_c']:.0f}" for x in MAIN if nominal(R, x))
    rows.append(["The Ground Mission not modelled", "Q53", f"Not in F. Its loading time grows with the counts ({counts}); USYD's GM was {RES26['GM']} ({RES26['source']}): size not measured."])
    norm = R["normalisation"]
    base = next((d for d in R["enumeration"] if d["tag"] == R["best_case"]), None)
    rng = "; ".join(f"{k} ΔF {signed(d['F'] - base['F'])}" for k, d in norm.items() if d and d.get("F") and base)
    rows.append(["The normalisation PLACEHOLDERS", "§5.2; Q52", f"Re-optimised: {rng or 'missing'}."])
    sm = "; ".join(f"{LABELS[x]} {nominal(R, x)['M2']['SM']:.2f}/{nominal(R, x)['M3']['SM']:.2f}" for x in MAIN if nominal(R, x))
    rows.append(["The SM frame offset", "validation_2026.md §4; Q9, Q62", f"The model's neutral point sits about 0.2 c aft of the teams' eCalc, so its SM isn't comparable to the reports' 10-20 %. Modelled SM M2/M3: {sm}."])
    cdi = "; ".join(f"{LABELS[x]} {1e4 * (nominal(R, x)['details']['M3'].get('cruise.vlm.wing_perf.CDi') or 0):.0f} counts" for x in MAIN if nominal(R, x))
    rows.append(["The rectangular wing against Humpback's end-plated wing", "validation_2026.md, stated up front", f"End-plates aren't modelled: every design's induced drag is the plain rectangular wing's (M3 induced drag {cdi}). Effect of end-plates not measured."])
    sc = R["designs"]["mdo"].get("screen") or {}
    sg = (sc.get("oned") or {}).get("f_build")
    rows.append(["The three-aircraft calibration", "mass_calibration_usyd.md §5; Q46", f"LOO structure errors -26 / +12 / +24 %; f_build's ±30 % gives sigma of F {f(float(np.sqrt(sg)) if sg is not None else None, 4)} at MDO. A payload fuselage this long is an extrapolation: l_cb {f(nominal(R, 'mdo')['M2']['l_cb'] if nominal(R, 'mdo') else None, 2)} m against Humpback's {f(nominal(R, 'humpback')['M2']['l_cb'] if nominal(R, 'humpback') else None, 2)} m."])
    ph = {r["path"]: r for r in placeholder_rows(R)}
    g, lp = ph.get("aero.cd_area_gear", {}), ph.get("aero.cd_area_lp", {})
    rows.append(["The constant gear and leakage drag areas", "§5.5; Q63", f"×0.5 to ×2 at MDO: gear F {f(g.get('F_lo'), 4)} to {f(g.get('F_hi'), 4)}, leakage F {f(lp.get('F_lo'), 4)} to {f(lp.get('F_hi'), 4)}."])
    d = decomposition(R)
    mdo = R["designs"]["mdo"].get("common")
    rows.append(["Calm air in the deterministic optimum", "§4", f"MDO F {f(mdo['nominal']['score']['flight_score'], 4) if mdo else '–'} in calm air, {f(mdo['mean_wind'].get('score', {}).get('flight_score'), 4) if mdo else '–'} at the mean wind; designing at the mean wind changes the Monte Carlo mean by {signed(d['mu']['wind']) if d else '–'}."])
    ss = sub_stall(R, "mdo")
    if ss:
        rows.append(["Fixed-throttle analysis can land on the sub-stall balance", "docs/MDA_ROBUSTNESS_PLAN.md §0", f"{ss['n']} of {ss['of']} in-set samples of MDO have M3's level speed below 1.2 V_stall at its fixed throttle. They count as stall-margin violations. Without them the mean of F is {f(ss['mu_without'], 4)} against {f(ss['mu'], 4)}."])
    return T("limitations", "Biases and limitations, with their measured size at the final designs.", ["item", "measured in", "size"], rows)


def rewind_errors():
    """The rewind law's error at the AT7215 catalogue windings ([TM7], the hand-off of RESULTS_2026_PLAN §4.4)."""
    import csv

    f = ROOT.parent / "3_LITERATURE" / "26 competition" / "T-Motor AT7215" / "at7215_specifications.csv"
    rows = list(csv.DictReader(f.open()))
    ref = next(r for r in rows if r["kv"] == "245")
    out = []
    for r in rows:
        kv = float(r["kv"])
        Rm = float(ref["internal_resistance_mohm"]) * (245.0 / kv) ** 2
        I0 = float(ref["idle_current_A"]) * kv / 245.0
        out.append(dict(kv=kv, Rm_cat=float(r["internal_resistance_mohm"]), Rm_model=Rm, Rm_err=Rm / float(r["internal_resistance_mohm"]) - 1, I0_cat=float(r["idle_current_A"]), I0_model=I0,
                        I0_err=I0 / float(r["idle_current_A"]) - 1))
    return out


# ------------------------------------------------------------------------------------------------------------- page
def render(R, ts, recs):
    L = []
    w = L.append
    w("# 2025-26 results: Humpback, AirCrafter 1.0, MDO and MDO + RDO\n")
    w(f"Generated by `studies/results_2026_report.py` (page commit `{R['page_git']}`) from the runs at the freeze tag `{S.FREEZE_TAG}` (`{R['freeze']}`), `outputs/results_2026/runs/fr_*.json` and `mc/`. "
      f"Every number below is read from `docs/studies/results_2026/results_2026.json`. Provenance: {R['provenance']['n']} run files, {len(R['provenance']['not_at_freeze'])} not at the freeze commit"
      + (f" ({', '.join(R['provenance']['not_at_freeze'])})" if R["provenance"]["not_at_freeze"] else "") + ". Configs: `configs/dbf2026_multipoint.yaml`, `configs/dbf2026_multipoint_rdo.yaml`, `configs/humpback_2026.yaml`, `configs/mass_usyd_loo_humpback.yaml`, `configs/rulesets/dbf_2025_26.yaml`, `configs/placeholders.yaml`.\n")
    w("**Biases stated up front.**\n")
    for s in ["**In-sample is not validation.** Humpback's column here uses the calibrated factors, fitted with it; the out-of-sample check is `validation_2026.md`, and its factors are the second Humpback column.",
              "**The common evaluation is AirCrafter 2.0's own model, so it favours AirCrafter 2.0's own optima.** AirCrafter 1.0's design is judged by a model it wasn't designed in. The Monte Carlo favours RDO by construction: it optimised a statistic of the same model over the same parameters. The out-of-set Monte Carlo (§2.5) is the check on that.",
              "**Analysis failures are not infeasibility.** MDA failures and wind limits are counted apart everywhere, and never as constraint violations.",
              "**A bound-driven optimum is bound-driven.** Active PLACEHOLDER bounds are marked in §3, and their effect is measured in §5.3.",
              "**The model flies Humpback's test laps too fast,** M3 by 20-46 % (validation_2026.md), and its M2 and M3 laps set F. F is a comparison inside this model, not a prediction of a competition score.",
              "**F leaves out the Ground Mission** (Q53) and uses PLACEHOLDER normalisations (Q52). Scores: optimised on continuous laps and the smooth cap; the capped, uncapped and floored (completed laps, hard cap) values are all reported."]:
        w(f"- {s}")
    w("\n## Findings\n")
    for i, s in enumerate(findings(R), 1):
        w(f"{i}. {s}")
    w("\n## 1. The designs compared\n")
    w(ts["method_comparison"].md())
    y = R["res26"]
    w(f"\nWhat each method claims in its own model: AirCrafter 1.0's selected design {f(1 + R['ac1']['score'], 4) if R.get('ac1') else '–'} (M2 {f((R.get('ac1') or {}).get('M2score'), 3)}, M3 {f((R.get('ac1') or {}).get('M3score'), 3)}; source: {(R.get('ac1') or {}).get('source_label')}). USYD's official 2026 result: M1 + M2 + M3 = {1 + y['M2'] + y['M3']:.2f} (M2 {y['M2']}, M3 {y['M3']}, GM {y['GM']}; {y['source']}), against Humpback's {f(nominal(R, 'humpback')['score']['flight_score'] if nominal(R, 'humpback') else None, 4)} in the model: the model's gap, not a method's.\n")
    w("![Method comparison](figures/method_comparison.png)\n")
    w("## 2. Common evaluation\n\n### 2.1 The AirCrafter 1.0 mappings\n")
    if R.get("ac1"):
        a = R["ac1"]
        w(f"AirCrafter 1.0 run: {a['source_label']} (`{a['source']}`); {a.get('n_points')} grid points, {a.get('n_calls')} mission evaluations, {a.get('n_feasible')} feasible. Selected: m {a['m2']:g} kg, b {a['b']:g} m, AR {a['AR']:g}, {a['n_p']}/{a['n_c']}, banner {a['banner_in']:g} in, battery fraction {a['f']:g}, V {a['V2']:g} / {a['V3']:g} m/s, props {a['props']['M2']} / {a['props']['M3']}. Mapping (a) throttle notes: {R.get('ac1_throttle')}.\n")
        w(ts["ac1_mapping"].md())
    else:
        w("Missing.\n")
    w("\n### 2.2 Nominal\n")
    w(ts["nominal_detail"].md())
    w("\n" + ts["humpback_factors"].md())
    w("\n**Every constraint's margin:**\n\n" + ts["constraint_margins"].md())
    w("\n### 2.3 In-set Monte Carlo\n")
    w("These Monte Carlos ran in batches of 384 samples, each on fresh workers (`studies/results_2026_mc_batched.py`), with the frozen model and study code. A first run in one pool lost 17-30 % of its samples to a failure that depends on how many evaluations a worker has done, not on the sample (RESULTS_LOG.md, Decision 13); on the samples both runs converged, F agrees to 2e-12.\n")
    w(ts["mc_in_summary"].md())
    w("\n" + ts["mc_in_p_violation"].md())
    w("\n![CDF in set](figures/mc_cdf_in_set.png)\n\n![P(violation) in set](figures/p_violation_in_set.png)\n")
    ss = [(lab, sub_stall(R, lab)) for lab in MAIN]
    ss = [(lab, x) for lab, x in ss if x]
    if ss:
        w("**Samples on the low-speed balance.** At a fixed throttle, a sample with enough drag or mass has no level-flight root above stall, and the T = D balance can land on the sub-stall root (docs/MDA_ROBUSTNESS_PLAN.md §0). Those samples converge, count as violations of the stall margin, and still carry an F from the lap model: "
          + "; ".join(f"{LABELS[lab]} {x['n']}/{x['of']}, mean of F without them {f(x['mu_without'], 4)} (with {f(x['mu'], 4)})" for lab, x in ss) + ".\n")
    w("### 2.4 Propagation check\n")
    w(ts["propagation"].md())
    p = R.get("propagation")
    w("\n### 2.5 Out-of-set Monte Carlo\n\nThis shows whether the robustness is tuned to the five parameters RDO saw.\n")
    w(ts["out_of_set_parameters"].md())
    w("\n" + ts["mc_out_summary"].md())
    w("\n" + ts["mc_out_p_violation"].md())
    w("\n![CDF out of set](figures/mc_cdf_out_of_set.png)\n\n![P(violation) out of set](figures/p_violation_out_of_set.png)\n")
    w("### 2.6 Wind\n")
    w(ts["wind_envelope"].md())
    w("\n" + ts["wind_direction"].md())
    w("\nThe upwind ground speed stays positive while v > w for every direction, so the limit barely depends on direction; F rises with |direction| because the along-course component falls (crosswind isn't checked: no directional-stability model, Q60).\n")
    w("\n![Wind envelope](figures/wind_envelope.png)\n\n![F against direction](figures/f_against_direction.png)\n")
    w("## 3. Why the designs differ\n")
    w(ts["design_table"].md())
    w("\n" + ts["active_constraints"].md())
    w("\n**Planform overlay.** AirCrafter 1.0 sets only span, AR, mass and the mission choices, so its tails and fuselage in the figure are AirCrafter 2.0's mapping (b).\n\n![Planform overlay](figures/planform_overlay.png)\n\n![Aligned at the prop plane](figures/planform_overlay_motor.png)\n\n![All six](figures/planform_overlay_all.png)\n")
    w(ts["mass_breakdown"].md())
    w("\n![Mass breakdown](figures/mass_breakdown.png)\n\n![Score split](figures/score_split.png)\n")
    w(ts["drag_breakdown"].md())
    w("\n![Drag breakdown](figures/drag_breakdown.png)\n\n![Propulsion enumeration](figures/propulsion_enumeration.png)\n")
    for lab in MAIN:
        w(design_paragraph(R, lab) + "\n")
    w("## 4. What robustness costs\n")
    w(ts["robustness_decomposition"].md())
    w("\n![Decomposition](figures/robustness_decomposition.png)\n")
    w(ts["robustness_tradeoff"].md())
    w("\n" + ts["rdo_starts"].md())
    w(f"\n`rdo` in the comparison is `{R['rdo_best']}`. Command for Appendix C: `.venv/bin/python studies/results_2026.py rdo --tag fr_rdo_mdo --case {R['best_case']} --start {R['mdo_tag']} --free` (the driver: `studies/results_2026_campaign.py`; the config: `configs/dbf2026_multipoint_rdo.yaml`).\n")
    w("\n**The screening** at the MDO and RDO designs, over the extended list (f_tb included):\n\n" + ts["screening"].md())
    for x in ("mdo", "rdo"):
        sc = R["designs"][x].get("screen")
        if sc:
            cov = ", ".join(f"{k.replace('points.', '')} {v:.2f}" for k, v in sc["coverage"].items() if v < sc["threshold"])
            w(f"\nAt {LABELS[x]}, the first-order screen (threshold {sc['threshold']:g}, at most {sc['max_dim']} parameters) would select {', '.join(sc['selected'])}" + (f"; below the threshold: {cov}." if cov else "."))
    nc = R.get("node_check")
    if nc:
        w(f"\n\n**The node check** (every UDR node, both points; N = {nc['N']}): " + "; ".join(f"{r['design']} {len(r['failed'])}/{nc['N']} failed ({len(r['wind_limit'])} wind limit)" for r in nc["results"]) + ".\n")
    w("\n## 5. Is it a real optimum?\n\n### 5.1 Enumeration, rounding, multistart, bookends\n")
    w(ts["enumeration"].md())
    w("\n" + ts["integer_rounding"].md())
    w("\n" + ts["multistart"].md())
    ms = multistart_summary(R)
    if ms:
        moved = {k: x for k, x in ms["spread"].items() if x["rel"] > 0.01}
        w(f"\n{ms['converged']}/{ms['n']} converge feasibly; distinct optima: " + "; ".join(f"F {c['F_min']:.4f}–{c['F_max']:.4f} ({c['n']})" for c in ms["clusters"]) + ". Design spread at the best optimum (more than 1 % of the box): "
          + (", ".join(f"{k} {x['min']:.4g}–{x['max']:.4g}" for k, x in moved.items()) or "none") + ".\n")
    w("\n![Multistart](figures/multistart_spread.png)\n")
    w(ts["bookends"].md())
    w("\n### 5.2 Normalisation\n\n" + ts["normalisation"].md())
    w("\n### 5.3 Bound sensitivities\n\n" + ts["bound_sensitivities"].md() + "\n")
    for x in bound_lines(R):
        if "missing" in x:
            w(f"- {x['name']}: missing ({x['missing']}).")
        else:
            verb = "relaxing it is worth" if x["dF"] > 0 else "a realistic floor costs"
            mv = ", ".join(f"{k} {a:.3g} to {b:.3g}" for k, (a, b) in x["moved"].items())
            w(f"- {x['name']}: {verb} ΔF = {abs(x['dF']):.4f}" + (f" (NeuralFoil {x['nf']:.2f}: **untrustworthy**)" if x["flag"] else "") + f". Design: {mv or 'barely moves'}.")
    w("\n### 5.4 Other sensitivities\n\n" + ts["other_sensitivities"].md())
    w("\n### 5.5 Placeholders\n\n" + ts["placeholder_ranking"].md() + "\n" + ts["normalised_sensitivities"].md())
    w("\n![Sensitivities](figures/sensitivities.png)\n\n![Ranking](figures/placeholder_ranking.png)\n")
    w("## 7. Biases and limitations\n")
    w(limitations(R).md())
    w("\n" + ts["q31_overlap"].md())
    w("\n## 8. Recommendations for the team\n\nModel-based: each rests on the named result, with its confidence given §7. Nothing here rests only on a PLACEHOLDER-driven result; §5.3's tail and chord bounds are why there is no recommendation on tail volume or chord.\n")
    for r in recs:
        w(f"- {r}")
    w("")
    return "\n".join(L)


def recommendations(R):
    from results_2026_recommendations import build

    return build(R, sys.modules[__name__])


def report():
    R = collect()
    ts = tables(R)
    DOCS.mkdir(parents=True, exist_ok=True)
    recs = recommendations(R)
    R["recommendations"] = recs
    R["findings"] = findings(R)
    R["limitations"] = limitations(R).rows
    R["ranking"] = ranking(R)
    R["decomposition"] = decomposition(R)
    R["multistart_summary"] = multistart_summary(R)
    R["bound_lines"] = bound_lines(R)
    R["overlap"] = {x: overlap(R, x) for x in MAIN}
    R["rewind_errors"] = rewind_errors()
    R["sub_stall"] = {x: sub_stall(R, x) for x in MAIN}
    R["design_walls"] = design_walls(R)
    R["mass_breakdown"] = {x: mass_breakdown(R, x) for x in MAIN if nominal(R, x)}
    R["drag_breakdown"] = {x: {pt: drag_parts(R, x, pt) for pt in ("M2", "M3")} for x in MAIN if nominal(R, x)}
    (DOCS / "results_2026.json").write_text(json.dumps(R, indent=1, default=lambda o: o.tolist()))
    (DOCS / "results_2026.md").write_text(render(R, ts, recs))
    print("wrote docs/studies/results_2026/results_2026.{md,json}")
    return R, ts


if __name__ == "__main__":
    report()
