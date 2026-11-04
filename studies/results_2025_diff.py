"""thesis_number_diff.{md,json}: every 2024-25 number the thesis quotes, old against new. Old and new values go through the
same extraction functions, old from docs/studies/results/{results_2025,validation_2025}.json (the published pages, unchanged
between the tags) and run files, new from docs/studies/results_2025_final/ and outputs/results_2025_final/.

"Still true?": yes = unchanged at the quoted precision; "number changes" = the statement holds but its number must be
updated; NO = the statement no longer holds. A claim whose stage did not finish is "not run". `src` says where the thesis
quotes it beyond the brief's list.

Usage: results_2025_diff.py [--selftest]    (selftest: new = old, to check the extraction against the quoted values)
"""

import argparse
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DOCS_OLD, DOCS_NEW = ROOT / "docs/studies/results", ROOT / "docs/studies/results_2025_final"
RUNS_OLD = Path.home() / "University/THESIS/AirCrafter2/outputs/results/runs"
RUNS_NEW = ROOT / "outputs/results_2025_final/runs"
OUT_NEW = ROOT / "outputs/results_2025_final"
LB = 0.45359237
T34 = dict(m_wing=3.41, m_ht=0.41, m_vt=0.19, m_fus=1.38, m_gear=0.66)


class D:
    """One study: its page JSON, validation JSON and run folder."""

    def __init__(self, page, val, runs):
        self.R = json.loads(Path(page).read_text())
        self.V = json.loads(Path(val).read_text())
        self.runs = Path(runs)
        self.mcdir = (Path.home() / "University/THESIS/AirCrafter2/outputs/results/reduced") if self.runs == RUNS_OLD else OUT_NEW / "reduced_final"
        self.B = self.R["uq"]["reduced"]
        self.w = self.V["woombah_loo"]["cases"]
        self.nom = self.B["nominal"]
        self.mc = self.B["mc"]
        self.best = self.R["best_case"]
        self.enum = [c for c in self.R["enumeration"] if c.get("ok")]

    def run(self, tag):
        f = self.runs / f"{tag}.json"
        return json.loads(f.read_text()) if f.exists() else None

    def F(self, lab):
        return self.nom[lab]["score"]["flight_score"]

    def bestrow(self):
        return max(self.enum, key=lambda c: c["F"])

    def ms(self):
        return self.R["multistart"]["runs"]

    def msok(self):
        return [r for r in self.ms() if r["ok"]]

    def kt_ms(self):
        ds = [self.run(f"{self.best}_ms{r['index']}") for r in self.msok()]
        return [d["dv"]["Kv"] * d["dv"]["throttle_M2"] for d in ds]

    def kv_rdo(self):
        return [r["dv"]["Kv"] for r in self.B["rdo"] if r.get("ok")]

    def kt_rdo(self):
        return [r["dv"]["Kv"] * r["dv"]["throttle_M2"] for r in self.B["rdo"] if r.get("ok")]

    def rob(self):
        return [r["summary"]["flight_score"]["robust"] for r in self.B["rdo"] if r.get("ok")]

    def spread_m2(self):
        s = [c for c in self.enum if c["cells"] == self.bestrow()["cells"]]
        g = {}
        for c in s:
            g.setdefault(c["props"]["M2"], []).append(c["F"])
        return max(max(x) for x in g.values()) - min(max(x) for x in g.values())

    def spread_m3(self):
        s = [c for c in self.enum if c["cells"] == self.bestrow()["cells"]]
        g = {}
        for c in s:
            g.setdefault(c["props"]["M2"], []).append(c["F"])
        return max(max(x) - min(x) for x in g.values())

    def spread_cells(self):
        b = self.bestrow()
        s = [c["F"] for c in self.enum if c["props"] == b["props"]]
        return max(s) - min(s)

    def variance_share(self):
        c = self.B["screen"]["contrib"]["flight_score"]
        tot = sum(c.values())
        return c

    def share(self, names):
        c = self.variance_share()
        return sum(c[n] for n in names) / sum(c.values())

    def norm_move(self):
        g = self.R["normalisation"]["grid"]
        c0 = next(r for r in g if r["f2"] == 1 and r["f3"] == 1)
        off = [r for r in g if "F" in r and r["f2"] != r["f3"]]
        return dict(fuel=max(abs(r["fuel_lb"] - c0["fuel_lb"]) for r in off), laps=max(abs(r["laps"] - c0["laps"]) for r in off),
                    n=len(off))

    def prop(self, m, q="flight_score", s="mu"):
        return self.B["propagation"]["methods"][m]["stats"][q][s]

    def prop_max_constraint(self):
        """The largest relative disagreement among the three methods in sigma over the constraints with a real spread
        (the mean agrees far closer): the thesis's 'about 2 %' is this reading or tighter."""
        M = self.B["propagation"]["methods"]
        out = 0.0
        for q in M["udr"]["stats"]:
            if q == "flight_score":
                continue
            x = [M[m]["stats"][q]["sigma"] for m in ("udr", "smolyak", "sobol") if q in M[m]["stats"]]
            if sum(x) / len(x) > 1e-3:
                out = max(out, (max(x) - min(x)) / (sum(x) / len(x)))
        return out

    def laps_quartiles(self, lab, col):
        """P(missing M2's 3 laps) in the lowest and highest quartile of one parameter (0 f_build, 3 lap.wind), from the
        Monte Carlo array."""
        import numpy as np

        z = np.load(self.mcdir / f"mc_{lab}.npz")
        names = [str(x) for x in z["outputs"]]
        laps, u = z["values"][:, names.index("points.M2.laps_frac")], z["u"]
        q = np.minimum((u[:, col] * 4).astype(int), 3)
        return float((laps[q == 0] < 3 - 1e-6).mean()), float((laps[q == 3] < 3 - 1e-6).mean())

    def p_any(self, lab):
        """P(any violation) from the Monte Carlo array, one rule for old and new (the published page did not report it)."""
        import numpy as np

        z = np.load(self.mcdir / f"mc_{lab}.npz")
        names = [str(x) for x in z["outputs"]]
        V = z["values"]
        ok = ~np.isnan(V).any(axis=1)
        bad = np.zeros(int(ok.sum()), bool)
        for name, kw in self.R["constraints"].items():
            if name not in names:
                continue
            col = V[ok, names.index(name)]
            for side, sgn in (("lower", -1.0), ("upper", 1.0)):
                b = kw.get(side)
                if b is not None:
                    bad |= sgn * (col - b) > 1e-6 * max(abs(b), abs(kw.get("ref") or 0.0), 1.0)
        return float(bad.mean())


def pct(x, ref):
    return 100 * (x / ref - 1)


def rows(d_old, d_new, cont, timing):
    """(section, claim, extractor, format, tolerance at the quoted precision, meaning check, source)."""
    r = []

    def add(sec, claim, f, fmt="{:.4f}", tol=None, holds=None, src=""):
        r.append(dict(sec=sec, claim=claim, f=f, fmt=fmt, tol=tol, holds=holds, src=src))

    val = "Validation (Woombah, out of sample)"
    for key, nm, q in (("m_wing", "Wing", -41), ("m_ht", "Horizontal tail", -5), ("m_vt", "Vertical tail", 14),
                       ("m_fus", "Fuselage", 57), ("m_gear", "Undercarriage", 63)):
        add(val, f"{nm} {q:+d} % (full-throttle case on the published page)", lambda d, k=key: pct(d.w["full_throttle"]["M2"][k] / LB, T34[k]), "{:+.1f} %", 0.5)
    add(val, "Structure −3 %", lambda d: pct(sum(d.w["full_throttle"]["M2"][k] / LB for k in T34), sum(T34.values())), "{:+.1f} %", 0.5)
    add(val, "Structure 5.86 lb against 6.05 lb", lambda d: sum(d.w["full_throttle"]["M2"][k] / LB for k in T34), "{:.2f} lb", 0.005, src="App. A")
    add(val, "M2 take-off mass −10 %", lambda d: pct(d.w["full_throttle"]["M2"]["m_total"] / LB, 26.02), "{:+.1f} %", 0.5)
    add(val, "M2 average lap 27.3 s (lap 2+, logged current)", lambda d: d.w["logged_current"]["M2"]["otherLapTime"], "{:.1f} s", 0.05)
    add(val, "M2 average lap −27 % against 37.4 s", lambda d: pct(d.w["logged_current"]["M2"]["otherLapTime"], 37.4), "{:+.1f} %", 0.5)
    add(val, "M2 3-lap time / 3, take-off included in the new run (not quoted; the fair reading of the 37.4 s)", lambda d: d.w["logged_current"]["M2"]["ThreeLapTime"] / 3, "{:.1f} s", 0.05)
    add(val, "M2 average speed +41 % (72.2 mph against 51.1)", lambda d: pct(d.w["logged_current"]["M2"]["avg_speed_mph_3laps"], 51.1), "{:+.1f} %", 0.5)
    add(val, "M3 average speed +39 % (72.9 mph against 52.3)", lambda d: pct(d.w["logged_current"]["M3"]["avg_speed_mph_3laps"], 52.3), "{:+.1f} %", 0.5)
    add(val, "M3 laps 9.8 against 7", lambda d: d.w["logged_current"]["M3"]["laps_frac"], "{:.2f}", 0.05)
    add(val, "M3 laps +40 %", lambda d: pct(d.w["logged_current"]["M3"]["laps_frac"], 7), "{:+.1f} %", 0.5, src="val_criteria_2025.tex")
    add(val, "Flight rows fail the 10 % criterion (M2 lap, M2 speed, M3 laps, M3 speed)", lambda d: "{:+.0f} / {:+.0f} / {:+.0f} / {:+.0f} %".format(
        pct(d.w["logged_current"]["M2"]["otherLapTime"], 37.4), pct(d.w["logged_current"]["M2"]["avg_speed_mph_3laps"], 51.1),
        pct(d.w["logged_current"]["M3"]["laps_frac"], 7), pct(d.w["logged_current"]["M3"]["avg_speed_mph_3laps"], 52.3)), "{}", None,
        lambda d: abs(pct(d.w["logged_current"]["M2"]["otherLapTime"], 37.4)) > 10 and abs(pct(d.w["logged_current"]["M2"]["avg_speed_mph_3laps"], 51.1)) > 10, src="val_criteria_2025.tex")
    add(val, "Logged-current throttle, M2 (input condition, not quoted)", lambda d: d.V["woombah_loo"]["logged_throttle"]["M2"]["throttle"], "{:.3f}", 0.0005)

    det = "Deterministic optimum and enumeration"
    add(det, "Deterministic optimum F = 5.0993", lambda d: d.bestrow()["F"], "{:.4f}", 5e-5)
    add(det, "Fuel at the 13.0 lb bottle capacity in every case", lambda d: f"{sum('fuel_mass (upper)' in c['bounds'] for c in d.enum)}/{len(d.enum)} at {d.bestrow()['fuel_lb']:.1f} lb", "{}", None,
        lambda d: all("fuel_mass (upper)" in c["bounds"] for c in d.enum))
    add(det, "Best case 6S, 11x8E / 10x7E", lambda d: f"{d.bestrow()['cells']:g}S, {d.bestrow()['props']['M2']} / {d.bestrow()['props']['M3']}", "{}")
    add(det, "Spread across M2 propellers 0.056", lambda d: d.spread_m2(), "{:.3f}", 5e-4)
    add(det, "M3 propeller ≤ 0.002", lambda d: d.spread_m3(), "{:.4f}", None, lambda d: d.spread_m3() <= 0.002)
    add(det, "Cell count 0.003", lambda d: d.spread_cells(), "{:.4f}", 5e-4)
    add(det, "Exactly three M2 laps in calm air (laps constraint active)", lambda d: d.run(d.best)["M2"]["laps_frac"], "{:.3f}", 5e-4,
        lambda d: "points.M2.laps_frac (lower)" in d.run(d.best)["active_constraints"], src="§4 Results, App. A")
    add(det, "M3 laps 10.01 (not quoted in the prose; page)", lambda d: d.run(d.best)["M3"]["laps_frac"], "{:.2f}", 0.005)
    add(det, "M2 3-lap time 84.9 s (page)", lambda d: d.run(d.best)["M2"]["ThreeLapTime"], "{:.1f} s", 0.05)

    ms = "Multistart"
    add(ms, "6 of 10 converged (\"in both years\", §4 Results, App. A)", lambda d: len(d.msok()), "{:d}", 0, lambda d: len(d.msok()) == 6)
    add(ms, "All to one optimum, F 5.0992–5.0993 (within about 1e-4, §4 Results)", lambda d: f"{min(x['F'] for x in d.msok()):.4f}–{max(x['F'] for x in d.msok()):.4f}", "{}", None,
        lambda d: max(x["F"] for x in d.msok()) - min(x["F"] for x in d.msok()) <= 1.5e-4)
    add(ms, "4 failed at their first analysis (brief, App. A); §4 says two at the start design, two after 5 and 50 evaluations",
        lambda d: "its " + ", ".join(str(x["its"]) for x in d.ms() if not x["ok"]), "{}", None,
        lambda d: len(d.ms()) - len(d.msok()) == 4)
    add(ms, "K_V × M2 throttle constant across converged designs, about 598", lambda d: f"{min(d.kt_ms()):.1f}–{max(d.kt_ms()):.1f}", "{}", None,
        lambda d: max(d.kt_ms()) - min(d.kt_ms()) < 0.01 * min(d.kt_ms()))
    add(ms, "Robust starts K_V 634 to 920 rpm/V", lambda d: f"{min(d.kv_rdo()):.0f}–{max(d.kv_rdo()):.0f}", "{}", src="§4 Results")
    add(ms, "Robust starts K_V × M2 throttle 598–598", lambda d: f"{min(d.kt_rdo()):.1f}–{max(d.kt_rdo()):.1f}", "{}", None,
        lambda d: max(d.kt_rdo()) - min(d.kt_rdo()) < 0.01 * min(d.kt_rdo()), src="page")

    bk = "Bookends"
    add(bk, "M2-only gains 0.0013 in M2", lambda d: d.R["bookends"]["M2only"]["M2"] - d.bestrow()["M2"], "{:+.4f}", 5e-5)
    add(bk, "M3-only gives up 0.722 in M2", lambda d: d.bestrow()["M2"] - d.R["bookends"]["M3only"]["M2"], "{:.3f}", 5e-4)
    add(bk, "M3-only gains 0.0147 in M3", lambda d: d.R["bookends"]["M3only"]["M3"] - d.bestrow()["M3"], "{:.4f}", 5e-5)

    com = "Common evaluation"
    add(com, "Woombah 5.1020", lambda d: d.F("woombah"), "{:.4f}", 5e-5)
    add(com, "Mapping (a) 5.3735", lambda d: d.F("ac1_direct"), "{:.4f}", 5e-5)
    add(com, "Mapping (b) 5.2879", lambda d: d.F("ac1_completed"), "{:.4f}", 5e-5)
    add(com, "Robust 5.0986", lambda d: d.F("rdo"), "{:.4f}", 5e-5, src="y2025.tex")
    add(com, "2 L bottles 5.4710", lambda d: d.F("multipoint_2L"), "{:.4f}", 5e-5)
    add(com, "Gain of 0.183 over mapping (b) (also 0.18 in Discussion, Conclusion)", lambda d: d.F("multipoint_2L") - d.F("ac1_completed"), "{:.3f}", 5e-4,
        lambda d: d.F("multipoint_2L") > d.F("ac1_completed"))
    add(com, "0.5 L bottles 4.7598", lambda d: next(b["F"] for b in d.R["bottles"] if b["bottle"] == "0.5L"), "{:.4f}", 5e-5)
    add(com, "The deterministic optimum 0.0027 below Woombah", lambda d: d.F("woombah") - d.F("multipoint"), "{:+.4f}", 5e-5,
        lambda d: d.F("woombah") > d.F("multipoint"))
    add(com, "Ranking of the six designs unchanged (take-off did not change the ranking, §4 Results)", lambda d: " > ".join(k for k, _ in sorted(((x, d.F(x)) for x in ("woombah", "ac1_direct", "ac1_completed", "multipoint", "multipoint_2L", "rdo")), key=lambda t: -t[1])), "{}")
    add(com, "Tank size the largest design effect (4.760, 5.099, 5.471)", lambda d: "; ".join(f"{b['bottle']} {b['F']:.3f}" for b in d.R["bottles"] if "F" in b), "{}", src="§4 Results")
    for lab, nm, mu, sg in (("woombah", "Woombah", 5.0932, 0.0143), ("ac1_direct", "Mapping (a)", 5.3652, 0.0187), ("ac1_completed", "Mapping (b)", 5.2756, 0.0266),
                            ("multipoint", "Deterministic", 5.0905, 0.0131), ("multipoint_2L", "2 L", 5.4569, 0.0323), ("rdo", "Robust", 5.0900, 0.0130)):
        add(com, f"{nm} Monte Carlo μ {mu:.4f}", lambda d, k=lab: d.mc[k]["stats"]["flight_score"]["mu"], "{:.4f}", 5e-5, src="y2025.tex")
        add(com, f"{nm} Monte Carlo σ {sg:.4f}", lambda d, k=lab: d.mc[k]["stats"]["flight_score"]["sigma"], "{:.4f}", 5e-5, src="y2025.tex")
        add(com, f"{nm} Monte Carlo 5th percentile", lambda d, k=lab: d.mc[k]["stats"]["flight_score"]["p5"], "{:.4f}", 5e-5, src="mc_2025.tex")
        add(com, f"{nm} largest single-constraint P", lambda d, k=lab: max(d.mc[k]["p_violation"].values()), "{:.3f}", 5e-4, src="mc_2025.tex")
    add(com, "Analysis failures across the six Monte Carlos: none (§App. A)", lambda d: sum(m.get("failed", 0) for m in d.mc.values()), "{:d}", 0)

    rb = "Robustness"
    add(rb, "RDO nominal cost 0.0007", lambda d: d.F("multipoint") - d.F("rdo"), "{:.4f}", 5e-5)
    add(rb, "All 3 robust starts converged", lambda d: f"{sum(bool(x.get('ok')) for x in d.B['rdo'])}/{len(d.B['rdo'])}", "{}", None, lambda d: all(x.get("ok") for x in d.B["rdo"]))
    add(rb, "Within 1e-6", lambda d: max(d.rob()) - min(d.rob()), "{:.1e}", None, lambda d: max(d.rob()) - min(d.rob()) <= 1e-6)
    add(rb, "σ_F 0.0131 → 0.0130", lambda d: f"{d.mc['multipoint']['stats']['flight_score']['sigma']:.4f} → {d.mc['rdo']['stats']['flight_score']['sigma']:.4f}", "{}")
    add(rb, "Largest single-constraint P(violation) 0.621 → 0.016", lambda d: f"{max(d.mc['multipoint']['p_violation'].values()):.3f} → {max(d.mc['rdo']['p_violation'].values()):.3f}", "{}", None,
        lambda d: max(d.mc["rdo"]["p_violation"].values()) < max(d.mc["multipoint"]["p_violation"].values()))
    add(rb, "5th percentile unchanged", lambda d: f"{d.mc['multipoint']['stats']['flight_score']['p5']:.4f} → {d.mc['rdo']['stats']['flight_score']['p5']:.4f}", "{}", None,
        lambda d: abs(d.mc["multipoint"]["stats"]["flight_score"]["p5"] - d.mc["rdo"]["stats"]["flight_score"]["p5"]) < 0.001)
    add(rb, "The deterministic optimum misses the 3 M2 laps in 62 % of samples", lambda d: 100 * d.mc["multipoint"]["p_violation"]["points.M2.laps_frac"], "{:.1f} %", 0.5)
    add(rb, "Any headwind costs it laps (M2 laps at the mean wind and mean inputs, calm air 3.000)", lambda d: d.B["screen"]["values"]["points.M2.laps_frac"], "{:.3f}", 5e-4,
        lambda d: d.B["screen"]["values"]["points.M2.laps_frac"] < 3.0)
    add(rb, "Wind was what robustness bought: P(miss M2 laps), calmest to windiest quartile", lambda d: "{:.0f} % → {:.0f} %".format(*(100 * x for x in d.laps_quartiles("multipoint", 3))), "{}", None,
        lambda d: d.laps_quartiles("multipoint", 3)[1] - d.laps_quartiles("multipoint", 3)[0] > 0.15)
    add(rb, "(For comparison) P(miss M2 laps), lightest to heaviest build-mass quartile", lambda d: "{:.0f} % → {:.0f} %".format(*(100 * x for x in d.laps_quartiles("multipoint", 0))), "{}")
    add(rb, "P(any violation), deterministic → robust (not quoted; computed from the Monte Carlo arrays, old and new)",
        lambda d: f"{d.p_any('multipoint'):.3f} → {d.p_any('rdo'):.3f}", "{}")

    pr = "Propagation and variance"
    add(pr, "UDR, Smolyak and Monte Carlo agree: 9, 49, 1024 analyses", lambda d: " / ".join(str(d.B["propagation"]["methods"][m]["N"]) for m in ("udr", "smolyak", "sobol")), "{}")
    add(pr, "Mean of F by the three methods agree (to the precision reported)", lambda d: " / ".join(f"{d.prop(m):.4f}" for m in ("udr", "smolyak", "sobol")), "{}", None,
        lambda d: max(d.prop(m) for m in ("udr", "smolyak", "sobol")) - min(d.prop(m) for m in ("udr", "smolyak", "sobol")) < 2 * d.prop("sobol", s="sigma") / 1024 ** 0.5 + 1e-4)
    add(pr, "σ of F by the three methods", lambda d: " / ".join(f"{d.prop(m, s='sigma'):.4f}" for m in ("udr", "smolyak", "sobol")), "{}")
    add(pr, "Every constraint to about 2 % (largest spread in σ among the methods; the published data give 2.8 %)", lambda d: 100 * d.prop_max_constraint(), "{:.1f} %", None, lambda d: d.prop_max_constraint() <= 0.05, src="§4 Results")
    add(pr, "Build mass about 2/3 of the variance of F", lambda d: d.share(["f_build"]), "{:.2f}", 0.03)
    add(pr, "Wind about 1/3", lambda d: d.share(["lap.wind"]), "{:.2f}", 0.03)

    se = "Sensitivities"
    add(se, "Normalisation barely moves the design", lambda d: f"largest off-diagonal move: fuel {d.norm_move()['fuel']:.2f} lb, M3 laps {d.norm_move()['laps']:.2f} ({d.norm_move()['n']} runs)", "{}", None,
        lambda d: d.norm_move()["fuel"] < 0.5 and d.norm_move()["laps"] < 0.2)
    return r


def evaluate(row, d_old, d_new):
    out = dict(sec=row["sec"], claim=row["claim"], src=row["src"])
    fmt = row["fmt"]

    def val(d):
        try:
            return row["f"](d)
        except (KeyError, StopIteration, TypeError, ValueError, AttributeError, IndexError, OSError):
            return None

    o, n = val(d_old), val(d_new)
    out["old"] = "–" if o is None else fmt.format(o)
    out["new"] = "not run" if n is None else fmt.format(n)
    if n is None:
        out["change"], out["true"] = "–", "not run"
        return out
    if isinstance(n, (int, float)) and isinstance(o, (int, float)):
        out["change"] = f"{n - o:+.4g}"
        same = abs(n - o) <= (row["tol"] if row["tol"] is not None else 0)
    else:
        same = str(o) == str(n)
        out["change"] = "same" if same else "changed"
    holds = True
    if row["holds"] is not None:
        try:
            holds = bool(row["holds"](d_new))
        except (KeyError, StopIteration, TypeError, ValueError, AttributeError, IndexError):
            holds = False
    if not holds:
        out["true"] = "**NO**"
    elif same:
        out["true"] = "yes"
    else:
        out["true"] = "number changes"
    return out


def main(selftest=False):
    old = D(DOCS_OLD / "results_2025.json", DOCS_OLD / "validation_2025.json", RUNS_OLD)
    new = old if selftest else D(DOCS_NEW / "results_2025_final.json", DOCS_NEW / "validation_2025_final.json", RUNS_NEW)
    cont = new.R.get("continuity")
    timing = new.R.get("timing") or {}
    rr = [evaluate(x, old, new) for x in rows(old, new, cont, timing)]
    if selftest:
        for x in rr:
            print(f"{x['claim'][:70]:70s} | {x['old']:>28s} | {x['true']}")
        return
    L = ["# 2024-25 thesis claims: old against new\n",
         f"Old: tag `results-2024-25-calibrated` (take-off off). New: `rerun-2025-final`, run commit `{new.R['run_commit']}`, take-off in mission time on, "
         "mass model USYD-calibrated. Generated by `studies/results_2025_diff.py`; old and new values go through the same extraction.\n",
         "**Still true?** yes = unchanged at the quoted precision; number changes = the statement holds, its number needs updating; **NO** = the statement no longer holds; not run = its stage did not finish.\n"]
    from collections import Counter

    cnt = Counter(x["true"] for x in rr)
    L.append("Count: " + ", ".join(f"{k} {v}" for k, v in cnt.items()) + ".\n")
    L.append("| thesis claim | old value | new value | change | still true? |")
    L.append("|---|---:|---:|---:|---|")
    sec = None
    for x in rr:
        if x["sec"] != sec:
            sec = x["sec"]
            L.append(f"| **{sec}** | | | | |")
        src = f" ({x['src']})" if x["src"] else ""
        L.append(f"| {x['claim']}{src} | {x['old']} | {x['new']} | {x['change']} | {x['true']} |")
    t = new.R.get("timing") or {}
    dets = [v for k, v in t.items() if k.startswith("timing_det") and v]
    L.append("| **Timing** | | | | |")
    L.append(f"| Deterministic optimisation 3.1 min | 3.1 min | {f'{sorted(dets)[len(dets) // 2] / 60:.1f} min (median of {len(dets)})' if dets else 'not run'} | | {'number changes' if dets else 'not run'} |")
    L.append(f"| RDO from the optimum 13 min | 13 min | {f'{t['timing_rdo'] / 60:.1f} min' if t.get('timing_rdo') else 'not run'} | | {'number changes' if t.get('timing_rdo') else 'not run'} |")
    if cont:
        L.append("\n## How much of each change is the take-off?\n")
        L.append("The published designs, unchanged, at the final code. The first step is the code change, the second the take-off, at fixed design.\n")
        L.append("| design | published F | final code, take-off off | take-off on | take-off effect | violated, take-off on |")
        L.append("|---|---:|---:|---:|---:|---|")
        for k, c in cont.items():
            off, on = c.get("takeoff_off", {}), c.get("takeoff_on", {})
            L.append(f"| {k} | {c['old_F']:.4f} | {off.get('F', float('nan')):.4f} | {on.get('F', float('nan')):.4f} | {on.get('F', float('nan')) - off.get('F', float('nan')):+.4f} | {', '.join(v.replace('points.', '') for v in on.get('violated', [])) or 'none'} |")
    if cont:
        eff = [c["takeoff_on"]["F"] - c["takeoff_off"]["F"] for c in cont.values() if "F" in c.get("takeoff_on", {})]
        code = [abs(c["takeoff_off"]["F"] - c["old_F"]) for c in cont.values() if "F" in c.get("takeoff_off", {})]
        act_old = old.run(old.best)["active_constraints"]
        act_new = new.run(new.best)["active_constraints"]
        p3 = new.w["logged_current"]["M2"]["ThreeLapTime"] / 3
        L.append(f"\nThe code change moves F by at most {max(code):.1e}, so **every change on this page comes from the take-off**, plus the re-optimisation it causes "
                 f"and, for the multistart count only, the fallback. At fixed design the take-off costs {-max(eff):.3f} to {-min(eff):.3f} in F. Re-optimised, the best case loses "
                 f"{old.bestrow()['F'] - new.bestrow()['F']:.4f}. Its active constraints are {'the same set' if set(act_old) == set(act_new) else 'a different set'} as the published optimum's, "
                 "but the published optima, unchanged, violate M2's energy and laps constraints with take-off on. This replaces §4's sentence on the take-off (about 0.02, from the uncalibrated model).\n")
        L.append("## Where the claims that no longer hold are, and what the data say\n")
        L.append("1. **Wind.** §4 Results (\"any headwind costs it laps\", robust-design paragraph), App. A (\"Wind was what robustness bought\"). A headwind now raises M2's laps at the mean wind, because the ground roll starts at an "
                 "airspeed equal to the headwind and saves take-off energy (`mission/smooth_lap.py`, take-off roll). The misses follow build mass. Even in the published arrays build mass was the stronger driver of M2 laps (rank correlation −0.94 against −0.32 for wind), so the old wording was already generous.")
        L.append("2. **Multistart.** §4 Results (\"In both years, six of the ten starts converged\", \"same score within about 1e-4\", Reliability paragraph \"four of the ten\"), App. A (\"As in 2025-26, six of ten\", \"all to one optimum\", \"the other four failed in analysis at the start design\"). "
                 "Now 8 of 10, to two airframes 0.0003 apart; the default start is in the lower group, so the best known deterministic F is 5.0740, not 5.0737. Attribution of 6 → 8: multistart 0 converges with the fallback off (take-off and start path), multistart 7 fails with it off at evaluation 5, as published (fallback).")
        L.append("3. **\"4 failed at their first analysis\"** (brief, App. A) was already wrong in the published data: evaluations 50, 0, 0 and 5, as §4 correctly says.\n")
        L.append("## Other edits the text needs\n")
        L.append("- **Statements of the take-off being off:** caption of `tab:res_criteria_2025`, App. A's opening sentence (tag, \"take-off not in the mission time\") and its Validation paragraph, §4's take-off paragraph.")
        L.append(f"- **The Woombah lap error.** The 27 % (§4, Discussion, Conclusion) compares lap 2+ with the report's 37.4 s average, so it does not see the take-off and hardly moves. Over all three laps, take-off included, the model gives {p3:.1f} s, {pct(p3, 37.4):+.0f} %. Which reading to quote is a decision.")
        L.append("- **Rounded repeats:** Executive Summary 0.62 → 0.02, Discussion and Conclusion 0.621 → 0.016, 0.18 and 0.0007, §4 5.099, 5.10, 0.001, 0.72, 0.003, 4.760 / 5.099 / 5.471.")
        L.append("- **Largest P(violation) changes constraint:** the deterministic optimum's 0.499 is now the M2 rpm margin (laps 0.476); the published 0.621 was laps and energy.")
        L.append("- **RDO nominal cost** is 0.0005 against the default-start optimum and 0.0007 against the best known (5.0740).")
        L.append("- **App. C cites the wrong config for the published study:** `dbf2025_multipoint_rdo_calibrated.yaml` is the superseded f_build 2.74 file; the published page used `dbf2025_multipoint_rdo.yaml`. After this re-run the configs are `dbf2025_multipoint_final.yaml` and `dbf2025_multipoint_rdo_final.yaml`, and `tab:rulesets`' study tag changes.")
        L.append("- **Generated tables** (`val_criteria_2025.tex`, `y2025.tex`, `mc_2025.tex`, `enumeration_2025.tex`, from `scripts/make_results_tables.py`) need the new JSON. Not touched here.")
        L.append("- **Published-data mismatches, independent of the re-run:** \"about 2 %\" for the constraints is 2.8 % in σ in the published propagation data; the multistart K_V × throttle is 598.7–599.3 (598.2 is the robust starts').")
    DOCS_NEW.mkdir(parents=True, exist_ok=True)
    (DOCS_NEW / "thesis_number_diff.md").write_text("\n".join(L) + "\n")
    (DOCS_NEW / "thesis_number_diff.json").write_text(json.dumps(rr, indent=1))
    print("\n".join(L))


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--selftest", action="store_true")
    main(ap.parse_args().selftest)
