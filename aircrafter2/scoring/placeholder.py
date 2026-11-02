"""Generic placeholder objectives, with no year-specific DBF rules, for the single-point problems.

three_lap_time   minimise the 3-lap time (AirCrafter 1.0's speed-mission figure of merit).
laps_in_window   maximise the continuous lap count within the mission time and the usable battery energy (laps_frac, the
                 fractional-lap surrogate of the lap model).
See docs/modules/scoring.md.
"""

from .base import ScoringPlugin, register


@register("three_lap_time")
class ThreeLapTime(ScoringPlugin):
    """Minimise the 3-lap time [s]."""

    sense = "min"
    inputs = {"ThreeLapTime": "s"}
    ref = 100.0

    def score(self, x):
        """The 3-lap time."""
        return x["ThreeLapTime"]


@register("laps_in_window")
class LapsInWindow(ScoringPlugin):
    """Maximise the continuous lap count in the window."""

    sense = "max"
    inputs = {"laps_frac": None}
    ref = 10.0

    def score(self, x):
        """The continuous lap count."""
        return x["laps_frac"]
