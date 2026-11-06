"""AIAA DBF 2024-25 (X-1 Supersonic Flight Program) flight score: the `dbf2025` plug-in.

Objective F = M1 + M2 + M3 from the multipoint problem's M2 fuel weight and 3-lap time and M3 laps
(problems/multipoint.py). GM measures the crew, not the aircraft, so it's left out. R2 and R3, the best teams' values,
are placeholders (docs/PLACEHOLDERS.md Q35). Source: Rules_2025.pdf. The rules as written and the model:
docs/modules/scoring.md, "dbf2025".
"""

import numpy as np
import openmdao.api as om

from ..smooth import softplus
from .base import ScoringPlugin, register

LB = 0.45359237
LAPS_SCALE = 0.05  # softplus width (laps) of the n_pre floor


def m2_verbatim(fuel_lb, time_s, best):
    """M2 as the rules write it (p. 11).

    Parameters
    ----------
    fuel_lb : float
        Fuel weight [lb].
    time_s : float
        Time to fly 3 laps [s].
    best : float
        Max_(fuel weight / time) [lb/s].

    Returns
    -------
    float
    """
    return 1.0 + (fuel_lb / time_s) / best


def m3_verbatim(laps_pre, bonus, x1_lb, best):
    """M3 as the rules write it (p. 12).

    Parameters
    ----------
    laps_pre : float
        Laps flown before the X-1 launch.
    bonus : float
        Bonus box score.
    x1_lb : float
        X-1 weight [lb].
    best : float
        The best team's value.

    Returns
    -------
    float
    """
    return 2.0 + (laps_pre + bonus / x1_lb) / best


@register("dbf2025")
class DBF2025(ScoringPlugin):
    """Flight score M1 + M2 + M3 of the 2024-25 rules, from the multipoint problem's M2 and M3 points."""

    sense = "max"
    ref = 5.0
    multipoint = True  # reads two mission points: problems/multipoint.py only (not offered for single-point runs)
    inputs = {"fuel_mass": "kg", "t3_M2": "s", "laps_M3": None}

    def __init__(self, R2=0.20, R3=27.0, bonus=0.0, x1_weight_lb=0.37, w2=1.0, w3=1.0, m1=1.0):
        """Set the options.

        Parameters
        ----------
        R2, R3 : float
            Best-team values, placeholders.
        bonus : float
            Bonus box score (0 main scenario, 2.5 second scenario).
        x1_weight_lb : float
            X-1 weight (Woombah: 0.37).
        w2, w3 : float
            Weights on the M2 and M3 terms, for the single-mission bookends (default 1, 1).
        m1 : float
            M1: the design is required to pass the M1 check.
        """
        super().__init__(R2=R2, R3=R3, bonus=bonus, x1_weight_lb=x1_weight_lb, w2=w2, w3=w3, m1=m1)

    def parts(self, x):
        """Every part of the score.

        M2 = 1 + (W_fuel / t_3laps) / R2, with W_fuel in lb (from kg) and t_3laps the lap model's ThreeLapTime [s].
        M3 = 2 + (n_pre + B / W_X1) / R3, with n_pre the laps in the window minus the release lap, softplus(laps_frac -
        1), a smooth floor at 0 that equals laps_frac - 1 to 1e-9 for laps_frac > 1.5.

        Parameters
        ----------
        x : dict
            fuel_mass [kg], t3_M2 [s], laps_M3.

        Returns
        -------
        dict
            M1, M2, M3, n_pre, flight_score, score.
        """
        opts = self.options
        n_pre = softplus(x["laps_M3"] - 1.0, LAPS_SCALE)[0]
        m2 = 1.0 + (x["fuel_mass"] / LB / x["t3_M2"]) / opts["R2"]
        m3 = 2.0 + (n_pre + opts["bonus"] / opts["x1_weight_lb"]) / opts["R3"]
        return dict(
            M1=opts["m1"] + 0.0 * m2,
            M2=m2,
            M3=m3,
            n_pre=n_pre,
            flight_score=opts["m1"] + m2 + m3,
            score=opts["m1"] + opts["w2"] * m2 + opts["w3"] * m3,
        )

    def score(self, x):
        """The objective."""
        return self.parts(x)["score"]

    def component(self):
        """The OpenMDAO component with the score's parts."""
        return DBF2025Score(plugin=self)


class DBF2025Score(om.ExplicitComponent):
    """score (the objective, with the bookend weights), plus M1, M2, M3, flight_score and n_pre for reporting.

    Partials are complex step: the plug-in is complex-step safe.

    Options
    -------
    plugin : DBF2025

    Inputs
    ------
    fuel_mass [kg], t3_M2 [s], laps_M3

    Outputs
    -------
    score, M1, M2, M3, flight_score, n_pre
    """

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
