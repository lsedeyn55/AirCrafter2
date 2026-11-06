"""AIAA DBF 2025-26 (Banner Towing Bush Plane) flight score: the `dbf2026` plug-in.

Objective F = M1 + M2 + M3 from the multipoint problem's lap counts, payload, M2 pack energy and banner
(problems/multipoint.py), with each mission ratio capped at 1 (docs/DECISIONS.md#ff-d1). NI_max and B_max, the best
teams' values, are placeholders (docs/PLACEHOLDERS.md Q52). Source: [R26] pp. 10, 19-21, 32
(docs/DECISIONS.md#r26-scoring). The rules as written and the model: docs/modules/scoring.md, "dbf2026".
"""

import numpy as np
import openmdao.api as om

from .base import ScoringPlugin, register

FT = 0.3048
RULES = dict(Ip1=6.0, Ip2=2.0, Ic1=10.0, Ic2=8.0, Ce=10.0, Cp=0.5, Cc=2.0)  # [R26] pp. 19-20


def net_income(n_p, n_c, laps, E_Wh, k=RULES):
    """Net Income = Income - Cost, as the rules write it (pp. 19-20).

    Parameters
    ----------
    n_p, n_c : float or array
        Passengers and cargo.
    laps : float or array
    E_Wh : float or array
        Pack energy [W*h]; EF = E_Wh / 100.
    k : dict
        The rule's constants, Ip1, Ip2, Ic1, Ic2, Ce, Cp, Cc.

    Returns
    -------
    float or array
    """
    income = n_p * (k["Ip1"] + k["Ip2"] * laps) + n_c * (k["Ic1"] + k["Ic2"] * laps)
    cost = laps * (k["Ce"] + n_p * k["Cp"] + n_c * k["Cc"]) * (E_Wh / 100.0)
    return income - cost


def cap(r, delta):
    """The rules' cap of a mission ratio N / Max at 1, smoothed for the optimiser.

    M2 = 1 + NI / Max_NI with Max the best team's, so the ratio is at most 1 ([R26] p. 20). Exactly r for r <= 1 -
    delta, so every score below the cap is unchanged bit for bit; exactly 1 for r >= 1 + delta; and the C1 quadratic
    blend r - (r - 1 + delta)^2 / (4 delta) between (bias delta / 4 at r = 1). Branches on the real part, so
    complex-step safe.

    Parameters
    ----------
    r : float or array
    delta : float
        Half-width of the blend; 0 gives the hard min(r, 1).

    Returns
    -------
    array
    """
    r = np.asarray(r)
    re = np.real(r)
    if delta <= 0.0:
        return np.where(re < 1.0, r, 1.0 + 0.0 * r)
    blend = r - (r - 1.0 + delta) ** 2 / (4.0 * delta)
    return np.where(re <= 1.0 - delta, r, np.where(re >= 1.0 + delta, 1.0 + 0.0 * r, blend))


def rac(span_ft):
    """Rated airplane cost (p. 10); the 0.90 minimum is the 3 ft span bound of the design.

    Parameters
    ----------
    span_ft : float or array

    Returns
    -------
    float or array
    """
    return 0.05 * span_ft + 0.75


def m3_term(laps, banner_in, span_ft):
    """#laps x banner length / RAC (p. 20).

    Parameters
    ----------
    laps : float or array
    banner_in : float or array
        Banner length [inch].
    span_ft : float or array

    Returns
    -------
    float or array
    """
    return laps * banner_in / rac(span_ft)


@register("dbf2026")
class DBF2026(ScoringPlugin):
    """Flight score M1 + M2 + M3 of the 2025-26 rules, from the multipoint problem's M2 and M3 points."""

    sense = "max"
    ref = 5.0
    multipoint = True
    inputs = {"n_p": None, "n_c": None, "laps_M2": None, "E_M2": "W*h", "laps_M3": None, "L_banner": "inch", "b": "m"}

    def __init__(self, NI_max=2800.0, B_max=1500.0, w2=1.0, w3=1.0, m1=1.0, cap_delta=0.005, **rules):
        """Set the options.

        Parameters
        ----------
        NI_max, B_max : float
            The best teams' Net Income and laps x length / RAC, placeholders (Q52).
        w2, w3 : float
            Weights on the M2 and M3 terms, for the single-mission bookends (default 1, 1).
        m1 : float
            M1: the design is required to pass the M1 check (analysis only).
        cap_delta : float
            Half-width of the smooth cap.
        **rules
            Overrides of RULES.
        """
        k = dict(RULES)
        k.update(rules)
        super().__init__(NI_max=NI_max, B_max=B_max, w2=w2, w3=w3, m1=m1, cap_delta=cap_delta, **k)

    def parts(self, x, hard=False):
        """Every part of the score.

        Parameters
        ----------
        x : dict
            n_p, n_c, laps_M2, E_M2 [W*h], laps_M3, L_banner [inch], b [m].
        hard : bool
            The hard cap min(r, 1), as the judges would record it (floored). Otherwise the smooth cap (cap_delta).

        Returns
        -------
        dict
            M1, M2, M3, net_income, m3_term, RAC, EF, flight_score, score, r_M2, r_M3.
        """
        opts = self.options
        ni = net_income(x["n_p"], x["n_c"], x["laps_M2"], x["E_M2"], opts)
        t3 = m3_term(x["laps_M3"], x["L_banner"], x["b"] / FT)
        r2, r3 = ni / opts["NI_max"], t3 / opts["B_max"]  # the uncapped ratios N / Max (Max: the best team, Q52)
        d = 0.0 if hard else opts["cap_delta"]
        m2 = 1.0 + cap(r2, d)
        m3 = 2.0 + cap(r3, d)
        return dict(
            M1=opts["m1"] + 0.0 * m2,
            M2=m2,
            M3=m3,
            net_income=ni,
            m3_term=t3,
            RAC=rac(x["b"] / FT),
            EF=x["E_M2"] / 100.0,
            flight_score=opts["m1"] + m2 + m3,
            score=opts["m1"] + opts["w2"] * m2 + opts["w3"] * m3,
            r_M2=r2,
            r_M3=r3,
        )

    def score(self, x):
        """The objective."""
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
        """The OpenMDAO component with the score's parts."""
        return DBF2026Score(plugin=self)


class DBF2026Score(om.ExplicitComponent):
    """score (the objective, with the bookend weights), plus M1, M2, M3, flight_score, net_income, m3_term, RAC, EF,
    r_M2, r_M3.

    Partials are complex step: the plug-in is complex-step safe.

    Options
    -------
    plugin : DBF2026

    Inputs
    ------
    n_p, n_c, laps_M2, E_M2 [W*h], laps_M3, L_banner [inch], b [m]

    Outputs
    -------
    OUT
    """

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
