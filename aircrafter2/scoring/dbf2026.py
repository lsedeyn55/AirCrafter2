"""AIAA DBF 2025-26 (Banner Towing Bush Plane) flight score: the `dbf2026` plug-in.

Verbatim (2026-DBF-Rules.pdf, 2025-26 Design, Build, Fly Rules; printed page "N of 34" = PDF page):
    p. 10  RAC = 0.05 * WS + 0.75 (WS = wingspan in ft, measured to the nearest inch); minimum RAC = 0.90 (3 ft span)
    p. 19  M1 = 1.0 for successful mission.
    p. 19  Net Income = Income - Cost
           Income = (#passengers * (Ip1 + (Ip2 * #laps))) + (#cargo * (Ic1 + (Ic2 * #laps)))
           Ip1 6, Ip2 2, Ic1 10, Ic2 8
    p. 20  Cost = #laps * (Ce + (#passengers * Cp) + (#cargo * Cc)) * EF
           EF = Efficiency Factor = Total Propulsion Battery Capacity (W-hrs) / 100;  Ce 10, Cp 0.5, Cc 2
    p. 20  M2 = 1 + [N_Net_Income / Max_Net_Income]
    p. 20  M3 = 2 + [N_(#laps X banner length / RAC) / Max_(#laps X banner length / RAC)]
           (Max over all teams, so each ratio is at most 1: the model caps it, smoothly, at 1; cap(), Q52)
    p. 21  GM = [Min_(mission time) / N_(mission time)]
    p. 32  Total Mission Score = M1 + M2 + M3 + GM; times to 0.01 s, lengths to 0.01 in, weights to 0.01 lb/oz
           (the banner length is recorded rounded down to 1/4 in, p. 12)

Model (docs/modules/scoring.md, docs/RESULTS_2026_PLAN.md §4.1):
    objective  F = M1 + M2 + M3 (the flight score; GM is not modelled: it measures the crew, and its time grows with
               the payload count)
    M1 = 1     the design is required to pass the M1 check (analysis only)
    N2, N3     laps in the 5-min window of the M2 and M3 points: the lap model's continuous laps_frac
    EF         E_M2 / 100, E_M2 = the M2 pack's rated energy (capacity x rated voltage, Wh): the pack is a per-mission
               choice (p. 24, p. 33), so the M2 pack size trades laps against EF
    n_p, n_c   passengers and cargo flown (continuous design variables; integers in the final design)
    L_banner   banner length in inches (design variable); RAC from the span design variable b
    NI_max, B_max  the best teams' Net_Income and laps x length / RAC: PLACEHOLDERS (OPEN_QUESTIONS Q52)
    weights    (w2, w3) multiply the M2 and M3 terms, for the single-mission bookends (default 1, 1)
`floored()` gives the same parts with the laps floored to completed laps and the banner and span recorded as the
rules record them (reported next to the continuous score; not differentiable).
"""

import numpy as np
import openmdao.api as om

from .base import ScoringPlugin, register

FT = 0.3048
RULES = dict(Ip1=6.0, Ip2=2.0, Ic1=10.0, Ic2=8.0, Ce=10.0, Cp=0.5, Cc=2.0)  # [R26] pp. 19-20


def net_income(n_p, n_c, laps, E_Wh, k=RULES):
    """Net Income = Income - Cost, as the rules write it (pp. 19-20)."""
    income = n_p * (k["Ip1"] + k["Ip2"] * laps) + n_c * (k["Ic1"] + k["Ic2"] * laps)
    cost = laps * (k["Ce"] + n_p * k["Cp"] + n_c * k["Cc"]) * (E_Wh / 100.0)
    return income - cost


def cap(r, delta):
    """The rules' cap of a mission ratio N / Max at 1 (M2 = 1 + NI / Max_NI with Max the best team's, so the ratio is
    at most 1: [R26] p. 20), smoothed for the optimiser: exactly r for r <= 1 - delta (every score below the cap is
    unchanged bit for bit), exactly 1 for r >= 1 + delta, and the C1 quadratic blend r - (r - 1 + delta)^2 / (4 delta)
    between (bias delta / 4 at r = 1). Branches on the real part, so complex-step safe. delta = 0: the hard min(r, 1)."""
    r = np.asarray(r)
    re = np.real(r)
    if delta <= 0.0:
        return np.where(re < 1.0, r, 1.0 + 0.0 * r)
    blend = r - (r - 1.0 + delta) ** 2 / (4.0 * delta)
    return np.where(re <= 1.0 - delta, r, np.where(re >= 1.0 + delta, 1.0 + 0.0 * r, blend))


def rac(span_ft):
    """Rated airplane cost (p. 10); the 0.90 minimum is the 3 ft span bound of the design."""
    return 0.05 * span_ft + 0.75


def m3_term(laps, banner_in, span_ft):
    """#laps x banner length / RAC (p. 20)."""
    return laps * banner_in / rac(span_ft)


@register("dbf2026")
class DBF2026(ScoringPlugin):
    """Flight score M1 + M2 + M3 of the 2025-26 rules, from the multipoint problem's M2 and M3 points."""

    sense = "max"
    ref = 5.0
    multipoint = True
    inputs = {"n_p": None, "n_c": None, "laps_M2": None, "E_M2": "W*h", "laps_M3": None, "L_banner": "inch", "b": "m"}

    def __init__(self, NI_max=2800.0, B_max=1500.0, w2=1.0, w3=1.0, m1=1.0, cap_delta=0.005, **rules):
        k = dict(RULES)
        k.update(rules)
        super().__init__(NI_max=NI_max, B_max=B_max, w2=w2, w3=w3, m1=m1, cap_delta=cap_delta, **k)

    def parts(self, x, hard=False):
        """hard: the hard cap min(r, 1), as the judges would record it (floored); else the smooth cap (cap_delta)."""
        o = self.options
        ni = net_income(x["n_p"], x["n_c"], x["laps_M2"], x["E_M2"], o)
        t3 = m3_term(x["laps_M3"], x["L_banner"], x["b"] / FT)
        r2, r3 = ni / o["NI_max"], t3 / o["B_max"]  # the uncapped ratios N / Max (Max: the best team, Q52)
        d = 0.0 if hard else o["cap_delta"]
        m2 = 1.0 + cap(r2, d)
        m3 = 2.0 + cap(r3, d)
        return dict(
            M1=o["m1"] + 0.0 * m2,
            M2=m2,
            M3=m3,
            net_income=ni,
            m3_term=t3,
            RAC=rac(x["b"] / FT),
            EF=x["E_M2"] / 100.0,
            flight_score=o["m1"] + m2 + m3,
            score=o["m1"] + o["w2"] * m2 + o["w3"] * m3,
            r_M2=r2,
            r_M3=r3,
        )

    def score(self, x):
        return self.parts(x)["score"]

    def floored(self, x):
        """The parts with the laps floored to completed laps, the banner length rounded down to 1/4 in (p. 12) and the
        span to the nearest inch (p. 10): what the judges would record. Report only."""
        y = {k: np.real(np.asarray(v, dtype=complex)) for k, v in x.items()}
        y["laps_M2"] = np.floor(y["laps_M2"] + 1e-9)
        y["laps_M3"] = np.floor(y["laps_M3"] + 1e-9)
        y["L_banner"] = np.floor(4.0 * y["L_banner"] + 1e-9) / 4.0
        y["b"] = np.round(y["b"] / 0.0254) * 0.0254
        y["n_p"] = np.round(y["n_p"])
        y["n_c"] = np.round(y["n_c"])
        return self.parts(y, hard=True)

    def component(self):
        return DBF2026Score(plugin=self)


class DBF2026Score(om.ExplicitComponent):
    """score (the objective, with the bookend weights), plus M1, M2, M3, flight_score, net_income, m3_term, RAC, EF."""

    OUT = ["score", "M1", "M2", "M3", "flight_score", "net_income", "m3_term", "RAC", "EF", "r_M2", "r_M3"]

    def initialize(self):
        self.options.declare("plugin")

    def setup(self):
        for name, units in self.options["plugin"].inputs.items():
            self.add_input(name, 1.0, units=units)
        for name in self.OUT:
            self.add_output(name, 1.0)
        self.declare_partials("*", "*", method="cs")
        self.declare_partials("M1", "*", dependent=False)

    def compute(self, inputs, outputs):
        p = self.options["plugin"].parts({k: inputs[k] for k in inputs})
        for k in self.OUT:
            outputs[k] = np.asarray(p[k])
