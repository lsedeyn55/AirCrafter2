"""Simple oval lap at constant speed (step 4).

Two straights of length L_s and two 180-degree level turns at constant speed V and load
factor n (developed from the LapTime component in archive/legacy/basic_perf.py):

    R     = V^2 / (g sqrt(n^2 - 1))       level coordinated turn radius
    t_lap = (2 L_s + 2 pi R) / V

The load factor comes from the turn analysis point, n = L_turn / W, smoothly capped at the
structural limit n_struct (KS-min). Its C_L is limited by the C_L,max margin constraint on the
turn point, so "the turn radius is set by the C_L margin to C_L,max".

n is floored smoothly just above 1 (1 + softplus(n - 1)), so the radius stays finite even at
infeasible iterates; feasible designs have n well above 1.
"""

import numpy as np

from ..analytic import AnalyticComponent, D
from ..smooth import ks_min, softplus

_N_FLOOR_WIDTH = 0.01


class LoadFactor(AnalyticComponent):
    """n = KSmin(L_turn / W, n_struct)."""

    def initialize(self):
        self.options.declare("max_mode", default="ks", values=["ks", "hard"])
        self.options.declare("rho_ks", default=100.0)

    def io(self):
        return ({"L_turn": (100.0, "N"), "W": (49.0, "N"), "n_struct": (5.0, None)},
                {"n_aero": (2.0, None), "n": (2.0, None)})

    def evaluate(self, x):
        L, W, ns = x["L_turn"], x["W"], x["n_struct"]
        na = L / W
        g_na = D({"L_turn": 1.0 / W, "W": -na / W})
        n, (wa, ws) = ks_min([na, ns], self.options["rho_ks"], self.options["max_mode"])
        return {"n_aero": na, "n": n}, {"n_aero": g_na, "n": wa * g_na + D({"n_struct": ws})}


class OvalLapTime(AnalyticComponent):
    def io(self):
        ins = {"V": (20.0, "m/s"), "n": (2.0, None), "straight_length": (304.8, "m"), "g": (9.80665, "m/s**2")}
        outs = {"turn_radius": (10.0, "m"), "turn_rate": (1.0, "rad/s"), "lap_time": (30.0, "s")}
        return ins, outs

    def evaluate(self, x):
        V, n, Ls, g = x["V"], x["n"], x["straight_length"], x["g"]
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
        return ({"turn_radius": R, "turn_rate": omega, "lap_time": t},
                {"turn_radius": gR, "turn_rate": gw, "lap_time": gt})
