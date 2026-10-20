"""AIAA DBF 2024-25 (X-1 Supersonic Flight Program) flight score: the `dbf2025` plug-in.

Verbatim (Rules_2025.pdf, 2024-25 Design, Build, Fly Rules):
    p. 11  M1 = 1.0 for successful mission.
    p. 11  M2 = 1 + [N_(fuel weight / time) / Max_(fuel weight / time)], where Max_(fuel weight / time) is the highest
           (fuel weight / time) score of all teams.            ("time": to fly 3 laps, from first throttle advance)
    p. 12  M3 = 2 + [N_(# laps flown + (bonus box score / X-1 test vehicle weight)) / Max_(# laps flown + (bonus box
           score / X-1 test vehicle weight))]                   ("# laps flown": prior to launch of the X-1)
    p. 12  GM = [Min_(mission time) / N_(mission time)]
    p. 27  Total Mission Score = M1 + M2 + M3 + GM
    p. 26  units: weights in lb and times in s, both rounded to 2 decimals.

Model (docs/modules/scoring.md):
    objective  F = M1 + M2 + M3 (the flight score; GM measures the crew, not the aircraft, and is left out)
    M1 = 1     the design is required to pass the M1 check (3 laps in 5 min, no payload, no pylons; analysis only)
    M2 = 1 + (W_fuel / t_3laps) / R2          W_fuel in lb (from kg), t_3laps = the lap model's ThreeLapTime (s;
                                              take-off not included, see the module docs)
    M3 = 2 + (n_pre + B / W_X1) / R3          n_pre = laps in the window minus the release lap: softplus(laps_frac - 1)
                                              (smooth floor at 0; equals laps_frac - 1 to 1e-9 for laps_frac > 1.5)
    R2, R3     best-team values, PLACEHOLDERS (config; OPEN_QUESTIONS Q35); B = bonus box score (0 main scenario,
               2.5 second scenario), W_X1 = X-1 weight in lb (Woombah: 0.37)
    weights    (w2, w3) multiply the M2 and M3 terms, for the single-mission bookends (default 1, 1)
"""

import numpy as np
import openmdao.api as om

from ..smooth import softplus
from .base import ScoringPlugin, register

LB = 0.45359237
LAPS_SCALE = 0.05  # softplus width (laps) of the n_pre floor


def m2_verbatim(fuel_lb, time_s, best):
    """M2 as the rules write it (p. 11), with best = Max_(fuel weight / time) in lb/s."""
    return 1.0 + (fuel_lb / time_s) / best


def m3_verbatim(laps_pre, bonus, x1_lb, best):
    """M3 as the rules write it (p. 12), laps_pre = laps flown before the X-1 launch."""
    return 2.0 + (laps_pre + bonus / x1_lb) / best


@register("dbf2025")
class DBF2025(ScoringPlugin):
    """Flight score M1 + M2 + M3 of the 2024-25 rules, from the multipoint problem's M2 and M3 points."""
    sense = "max"
    ref = 5.0
    inputs = {"fuel_mass": "kg", "t3_M2": "s", "laps_M3": None}

    def __init__(self, R2=0.20, R3=27.0, bonus=0.0, x1_weight_lb=0.37, w2=1.0, w3=1.0, m1=1.0):
        super().__init__(R2=R2, R3=R3, bonus=bonus, x1_weight_lb=x1_weight_lb, w2=w2, w3=w3, m1=m1)

    def parts(self, x):
        o = self.options
        n_pre = softplus(x["laps_M3"] - 1.0, LAPS_SCALE)[0]
        m2 = 1.0 + (x["fuel_mass"] / LB / x["t3_M2"]) / o["R2"]
        m3 = 2.0 + (n_pre + o["bonus"] / o["x1_weight_lb"]) / o["R3"]
        return dict(M1=o["m1"] + 0.0 * m2, M2=m2, M3=m3, n_pre=n_pre,
                    flight_score=o["m1"] + m2 + m3, score=o["m1"] + o["w2"] * m2 + o["w3"] * m3)

    def score(self, x):
        return self.parts(x)["score"]

    def component(self):
        return DBF2025Score(plugin=self)


class DBF2025Score(om.ExplicitComponent):
    """score (the objective, with the bookend weights), plus M1, M2, M3, flight_score and n_pre for reporting."""

    def initialize(self):
        self.options.declare("plugin")

    def setup(self):
        for name, units in self.options["plugin"].inputs.items():
            self.add_input(name, 1.0, units=units)
        for name in ["score", "M1", "M2", "M3", "flight_score", "n_pre"]:
            self.add_output(name, 1.0)
        self.declare_partials("*", "*", method="cs")
        self.declare_partials("M1", "*", dependent=False)

    def compute(self, inputs, outputs):
        p = self.options["plugin"].parts({k: inputs[k] for k in inputs})
        for k, v in p.items():
            outputs[k] = np.asarray(v)
