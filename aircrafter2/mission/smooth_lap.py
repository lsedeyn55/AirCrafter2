"""Smooth twin of the AirCrafter lap modeller (decision D2).

Same physics as ``lap_modeller.performance_modeller`` (Kallarackel 2025, Ch. 3, and
performance_modeller.m), reformulated so that gradients are exact:

* Constant electrical power P = D(V) V / eta (Eq. 3.4); thrust T = P / v.
* Drag polar CD = CD0 + CD1 CL + k CL^2. With CD1 = 0 this is Eqs. 3.5-3.7 (static
  assumptions); the coupled model supplies a three-point fit (aero/polar.py).
* Straights: dv/ds = a(v) / v and dt/ds = 1 / v, a(v) = (T - D) / m, integrated in distance
  with fixed-step RK4 (no lookup tables, no while loops). The MATLAB tables stop accelerating
  at V (the pilot holds the target speed); here the acceleration is a smooth
  min(a(v), (V - v) / tau_hold), so v relaxes onto V and stays there (a plain multiplicative
  cap leaks: thrust exceeds drag at V because P includes 1/eta).
* Turns: duration = angle / omega(v_entry), with omega = g sqrt(n^2 - 1) / v and
  n = KSmin(q S CL_max / W, n_struct) (Eqs. 3.10-3.12: stall-limited turn rate, capped at the
  structural limit). During the turn the speed follows dv/dt = min(a_turn(v), 0) with drag at
  CL = CL_max, as in the MATLAB deceleration tables (smooth min), integrated in time with RK4.
* Lap 1 starts at V_stall; legs as in Table 3.5. Take-off time is not added (as in the MATLAB).
* Lap count: continuous surrogate laps_frac = KSmin(laps limited by time, laps limited by energy),
  where laps(t) = 1 + (t - (lap1 + t_r)) / (lap_other + t_r) mirrors the MATLAB counting loop.
  lapCount (integer) is reported only.

All arithmetic is complex-step safe; the component uses method="cs" partials (exact).
"""

import numpy as np
import openmdao.api as om

from ..smooth import ks_min, softplus

FT2M = 0.3048
LAP1 = [("s", 500), ("t", 180), ("t", 360), ("s", 1000), ("t", 180), ("s", 500)]  # Table 3.5, Lap 1
LAP_OTHER = [("s", 500), ("t", 180), ("t", 360), ("s", 1000), ("t", 180), ("s", 500)]  # Lap 2+


def _sigmoid(z):
    zr = np.real(z)
    return np.where(zr >= 0, 1.0 / (1.0 + np.exp(-np.where(zr >= 0, z, 0.0))),
                    np.exp(np.where(zr >= 0, 0.0, z)) / (1.0 + np.exp(np.where(zr >= 0, 0.0, z))))


class SmoothLapModel(om.ExplicitComponent):
    def initialize(self):
        self.options.declare("n_steps_straight", default=40, types=int)
        self.options.declare("n_steps_turn", default=20, types=int)
        self.options.declare("tau_hold", default=0.5, desc="s, relaxation time of the speed hold at V")
        self.options.declare("a_smooth", default=0.01, desc="m/s^2, width of the smooth min(a, 0) in turns")
        self.options.declare("rho_ks", default=100.0)
        self.options.declare("pilot_reaction_time", default=3.0, desc="s per lap, performance_modeller.m")
        self.options.declare("lap1", default=LAP1)
        self.options.declare("lap_other", default=LAP_OTHER)

    def setup(self):
        for name, val, units in [
            ("m", 5.0, "kg"), ("S", 0.24, "m**2"), ("V", 20.0, "m/s"), ("rho", 1.18, "kg/m**3"),
            ("CD0", 0.012, None), ("CD1", 0.0, None), ("k", 0.06, None), ("CL_max", 1.4, None), ("CL_to", 1.5, None),
            ("eta", 0.7, None), ("n_struct", 5.0, None), ("mu_roll", 0.02, None),
            ("takeoff_distance", 100.0, "m"), ("mission_time", 300.0, "s"), ("E_avail", 288000.0, "J"),
            ("g", 9.81, "m/s**2"),
        ]:
            self.add_input(name, val, units=units)
        for name, val, units in [
            ("Vstall", 15.0, "m/s"), ("Preq", 100.0, "W"), ("Tstatic", 15.0, "N"), ("Lap1time", 60.0, "s"),
            ("otherLapTime", 55.0, "s"), ("ThreeLapTime", 170.0, "s"), ("laps_frac", 5.0, None),
            ("stall_margin", 1.0, "m/s"), ("turn_speed_margin", 1.0, "m/s"), ("energy_3laps", 1e4, "J"),
        ]:
            self.add_output(name, val, units=units)
        self.add_output("lapCount", 5.0, desc="report only (step function)")
        self.declare_partials("*", "*", method="cs")
        self.declare_partials("lapCount", "*", dependent=False)

    # --- physics -------------------------------------------------------------------------
    def _model(self, x):
        o = self.options
        m, S, V, rho = x["m"], x["S"], x["V"], x["rho"]
        CD0, CD1, k, CLmax, g = x["CD0"], x["CD1"], x["k"], x["CL_max"], x["g"]
        cd = lambda cl: CD0 + CD1 * cl + k * cl**2  # noqa: E731
        W = m * g
        qS = lambda v: 0.5 * rho * v**2 * S  # noqa: E731

        Vstall = np.sqrt(W / (0.5 * rho * CLmax * S))
        D_cruise = qS(V) * cd(W / qS(V))
        P = D_cruise * V / x["eta"]

        def a_level(v):  # straight, level: CL = W / qS
            return (P / v - qS(v) * cd(W / qS(v))) / m

        def a_capped(v):  # smooth min(a_level, (V - v) / tau): the pilot holds V
            a = a_level(v)
            return a - softplus(a - (V - v) / o["tau_hold"], o["a_smooth"])[0]

        def a_turn(v):  # drag at CL_max (MATLAB deceleration tables), only the decelerating part
            a = (P / v - qS(v) * cd(CLmax)) / m
            return -softplus(-a, o["a_smooth"])[0]

        def omega(v):
            n_stall = qS(v) * CLmax / W
            n, _ = ks_min([n_stall, x["n_struct"]], o["rho_ks"])
            n = 1.0 + softplus(n - 1.0, 0.01)[0]
            return g * np.sqrt(n**2 - 1.0) / v

        def straight(v, dist):
            N = o["n_steps_straight"]
            h = dist / N
            t = 0.0 * v
            for _ in range(N):  # RK4 on y = (v, t) with ds steps
                f = lambda vv: (a_capped(vv) / vv, 1.0 / vv)  # noqa: E731
                k1 = f(v)
                k2 = f(v + 0.5 * h * k1[0])
                k3 = f(v + 0.5 * h * k2[0])
                k4 = f(v + h * k3[0])
                v, t = (v + h / 6 * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]),
                        t + h / 6 * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1]))
            return v, t

        def turn(v, angle_deg):
            t_turn = np.radians(angle_deg) / omega(v)
            N = o["n_steps_turn"]
            h = t_turn / N
            for _ in range(N):
                k1 = a_turn(v)
                k2 = a_turn(v + 0.5 * h * k1)
                k3 = a_turn(v + 0.5 * h * k2)
                k4 = a_turn(v + h * k3)
                v = v + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
            return v, t_turn

        def lap(v, legs, track_360=False):
            total, v_after_360 = 0.0 * v, None
            for kind, size in legs:
                if kind == "s":
                    v, t = straight(v, size * FT2M)
                else:
                    v, t = turn(v, size)
                    if size == 360 and v_after_360 is None:
                        v_after_360 = v
                total = total + t
            return v, total, v_after_360

        v, lap1, v360 = lap(Vstall, o["lap1"])
        if v360 is None:  # custom courses without a 360-degree turn: use the lap-1 exit speed
            v360 = v
        _, other, _ = lap(v, o["lap_other"])

        # static thrust for the take-off ground roll (Eqs. 3.1-3.3, static_thrust_calculator.m)
        CLto = x["CL_to"]
        VR = 1.2 * np.sqrt(W / (0.5 * rho * S * CLto))
        Tstatic = m * VR**2 / (2.0 * x["takeoff_distance"]) + qS(VR) * cd(CLto) + x["mu_roll"] * W

        tr = o["pilot_reaction_time"]
        laps_time = 1.0 + (x["mission_time"] - (lap1 + tr)) / (other + tr)
        laps_energy = 1.0 + (x["E_avail"] / P - (lap1 + tr)) / (other + tr)
        laps_frac, _ = ks_min([1.0 + softplus(laps_time - 1.0, 0.05)[0],
                               1.0 + softplus(laps_energy - 1.0, 0.05)[0]], o["rho_ks"])
        return dict(Vstall=Vstall, Preq=P, Tstatic=Tstatic, Lap1time=lap1, otherLapTime=other,
                    ThreeLapTime=lap1 + 2.0 * other, laps_frac=laps_frac, stall_margin=V - 1.2 * Vstall,
                    turn_speed_margin=v360 - Vstall, energy_3laps=P * (lap1 + 2.0 * other + 3.0 * tr))

    def compute(self, inputs, outputs):
        res = self._model({k: inputs[k] for k in inputs})
        for k, v in res.items():
            outputs[k] = v
        # integer count for reporting, mirroring the MATLAB loop's strict inequalities
        outputs["lapCount"] = max(1.0, np.ceil(np.real(res["laps_frac"])) - 1.0)
