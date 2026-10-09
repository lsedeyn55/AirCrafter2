"""Generic placeholder objectives (no year-specific DBF rules).

three_lap_time   minimise the 3-lap time (AirCrafter 1.0's speed-mission figure of merit).
laps_in_window   maximise the continuous lap count within the mission time and the usable
                 battery energy (laps_frac, the fractional-lap surrogate of the lap model).
"""

from .base import ScoringPlugin, register


@register("three_lap_time")
class ThreeLapTime(ScoringPlugin):
    sense = "min"
    inputs = {"ThreeLapTime": "s"}
    ref = 100.0

    def score(self, x):
        return x["ThreeLapTime"]


@register("laps_in_window")
class LapsInWindow(ScoringPlugin):
    sense = "max"
    inputs = {"laps_frac": None}
    ref = 10.0

    def score(self, x):
        return x["laps_frac"]
