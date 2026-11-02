"""Simple oval lap at constant speed, used by the oval_lap problem.

Two straights and two 180-degree level turns at constant speed. It takes the load factor from the turn analysis point
(problems/oval_lap.py) and returns the lap time, a teaching and verification case for the coupled aero.
Developed from the LapTime component in archive/legacy/basic_perf.py.
Details: docs/modules/mission.md, "Oval".
"""

import numpy as np

from ..analytic import AnalyticComponent, D
from ..smooth import ks_min, softplus

_N_FLOOR_WIDTH = 0.01


class LoadFactor(AnalyticComponent):
    """Load factor from the turn point, smoothly capped at the structural limit: n = KSmin(L_turn / W, n_struct).

    Partials are analytic.

    Options
    -------
    max_mode : "ks" or "hard"
        Smooth or hard minimum.
    rho_ks : float
        Sharpness of the KS (Kreisselmeier-Steinhauser) smooth minimum.

    Inputs
    ------
    L_turn [N], W [N], n_struct

    Outputs
    -------
    n_aero : L_turn / W
    n : capped load factor

    Symbols
    -------
    L, W : lift and weight at the turn point [N]
    na, ns : aerodynamic load factor and structural limit
    """

    def initialize(self):
        self.options.declare("max_mode", default="ks", values=["ks", "hard"])
        self.options.declare("rho_ks", default=100.0)

    def io(self):
        return (
            {"L_turn": (100.0, "N"), "W": (49.0, "N"), "n_struct": (5.0, None)},
            {"n_aero": (2.0, None), "n": (2.0, None)},
        )

    def evaluate(self, x):
        L, W, ns = x["L_turn"], x["W"], x["n_struct"]
        na = L / W
        g_na = D({"L_turn": 1.0 / W, "W": -na / W})
        n, (wa, ws) = ks_min([na, ns], self.options["rho_ks"], self.options["max_mode"])
        return {"n_aero": na, "n": n}, {"n_aero": g_na, "n": wa * g_na + D({"n_struct": ws})}


class OvalLapTime(AnalyticComponent):
    """Lap time of the oval: R = V^2 / (g sqrt(n^2 - 1)), t_lap = (2 L_s + 2 pi R) / V.

    Partials are analytic.

    Inputs
    ------
    V [m/s], n, straight_length [m], g [m/s^2]

    Outputs
    -------
    turn_radius [m], turn_rate [rad/s], lap_time [s]

    Symbols
    -------
    V : flight speed [m/s]
    n, ne : load factor, and the same floored smoothly just above 1
    Ls : length of one straight (L_s) [m]
    R : level coordinated turn radius [m]
    s : sqrt(ne^2 - 1)
    """

    def io(self):
        ins = {"V": (20.0, "m/s"), "n": (2.0, None), "straight_length": (304.8, "m"), "g": (9.80665, "m/s**2")}
        outs = {"turn_radius": (10.0, "m"), "turn_rate": (1.0, "rad/s"), "lap_time": (30.0, "s")}
        return ins, outs

    def evaluate(self, x):
        V, n, Ls, g = x["V"], x["n"], x["straight_length"], x["g"]
        # floor n just above 1, so the radius stays finite at infeasible iterates; feasible designs have n well above 1
        ne, dne = softplus(n - 1.0, _N_FLOOR_WIDTH)
        ne = 1.0 + ne
        s = np.sqrt(ne**2 - 1.0)
        ds_dn = ne * dne / s

        R = V**2 / (g * s)
        gR = D({"V": 2.0 * V / (g * s), "g": -R / g, "n": -R / s * ds_dn})
        t = (2.0 * Ls + 2.0 * np.pi * R) / V
        gt = (2.0 * np.pi / V) * gR + D({"straight_length": 2.0 / V, "V": -t / V})
        omega = V / R
        gw = D({"V": 1.0 / R}) + (-V / R**2) * gR
        return (
            {"turn_radius": R, "turn_rate": omega, "lap_time": t},
            {"turn_radius": gR, "turn_rate": gw, "lap_time": gt},
        )
