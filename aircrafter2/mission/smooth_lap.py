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
* Wind (robustness): a steady wind of speed `wind` from direction `wind_dir` (deg from the course axis; 0 = along
  it, the original model). Valid while wind < V_stall: lap 1 starts at V_stall into the wind; a non-physical lap
  raises AnalysisError. On a straight the along-track component a = wind cos(wind_dir) is a head- or tailwind by the
  leg's direction (course.py: head +1, tail -1, none 0) and the crosswind c = wind sin(wind_dir) is crabbed out:
  ground speed = v sqrt(1 - (c/v)^2) - sign a (energy and power on airspeed). At wind_dir = 0 this is v - sign wind,
  bit for bit. Legacy legs ("s", length_ft) without a direction alternate head, tail, head, ... as before.
  Turns are flown at constant airspeed in the air-mass frame; with turn_drift on (course.turn_drift; off by default,
  the original model) the drift wind x t_turn is carried to the next straight: its along-track part shortens it (drift
  in its direction of flight) or lengthens it, and its cross-track part d is recovered on it (flown length
  sqrt((L - delta)^2 + d^2)). The take-off ground roll sees the along-runway component only.
* Banner (2025-26 M3): the point's drag polar is the towing state; lap 1's legs in banner_stowed_legs (before the first
  downwind leg) are flown with CD0 + dCD_stowed (stowed instead of deployed banner), the ground roll included.
* Turns: duration = angle / omega(v_entry), with omega = g sqrt(n^2 - 1) / v and
  n = KSmin(q S CL_max / W, n_struct) (Eqs. 3.10-3.12: stall-limited turn rate, capped at the
  structural limit). During the turn the speed follows dv/dt = min(a_turn(v), 0) with drag at
  CL = CL_max, as in the MATLAB deceleration tables (smooth min), integrated in time with RK4.
* Lap 1 starts at V_stall; legs from the course (default Table 3.5). Take-off time is not added (as in the MATLAB),
  unless option takeoff=True (propulsion mode only; docs/TAKEOFF_COURSE_PLAN.md):
    ground roll from rest (airspeed = headwind w) to lift-off at V_LOF = k_lof V_stall on lap 1's first straight,
    m dv/dt = T(v) - D(v) - mu_roll (W - L(v)),  L, D at CL_ground with the polar, no ground effect,
    integrated in airspeed (no event): dt/dv = 1/a, ds/dv = (v - w)/a, dE/dv = P_batt(v)/a, fixed-step RK4.
    a is floored smoothly (a_floor + softplus) so poor designs stay finite; takeoff_accel_margin is the smallest
    unfloored acceleration over the roll. The rest of the first straight (its length + start_offset_ft - ground roll,
    kept positive smoothly; takeoff_straight_margin = length - roll) is flown in the air from V_LOF. The roll's time
    and energy go into Lap1time and the lap-1 energy. Rotation, transition and climb are not modelled.
* Lap count: continuous surrogate laps_frac = KSmin(laps limited by time, laps limited by energy),
  where laps(t) = 1 + (t - (lap1 + t_r)) / (lap_other + t_r) mirrors the MATLAB counting loop.
  lapCount (integer) is reported only.

power_mode="propulsion" (decision B, used by full_mdo) replaces AirCrafter's power rule:
* thrust T(v) and battery power P(v) come from the powertrain (cubic fits in v, propulsion/
  powertrain.py) at a fixed throttle; there is no speed cap, so the aircraft accelerates on the
  straights towards the speed where T = D. Input V is that equilibrium cruise speed (solved in
  the MDA) and is used for the stall margin and the reaction-time energy only.
* Battery energy is integrated along every leg; laps_frac uses the usable battery energy.
* thrust_margin_static = T(0) - Tstatic (take-off ground roll).
power_mode="aircrafter" keeps the AirCrafter 1.0 rule (P = D(V) V / eta, T = P / v, speed held at V).

All arithmetic is complex-step safe; the component uses method="cs" partials (exact).
"""

import numpy as np
import openmdao.api as om

from ..smooth import ks_min, softplus

FT2M = 0.3048
# Table 3.5 (configs/courses/dbf_2025.yaml): (kind, size, wind sign) with size in ft or deg; +1 head, -1 tail, 0 none
LAP1 = [("s", 500, 1.0), ("t", 180, None), ("t", 360, None), ("s", 1000, -1.0), ("t", 180, None), ("s", 500, 1.0)]
LAP_OTHER = list(LAP1)  # Lap 2+
TAKEOFF_OUTPUTS = [
    ("t_takeoff", "s"),
    ("s_ground_roll", "m"),
    ("V_lof", "m/s"),
    ("E_takeoff", "J"),
    ("takeoff_accel_margin", "m/s**2"),
    ("takeoff_straight_margin", "m"),
]


def normalise_legs(legs):
    """(kind, size, wind_sign) legs; legacy (kind, size) straights alternate head, tail, head, ... (the old rule)."""
    out, n_straight = [], 0
    for leg in legs:
        kind, size = leg[0], leg[1]
        if kind == "s":
            sign = leg[2] if len(leg) > 2 else (1.0 if n_straight % 2 == 0 else -1.0)
            n_straight += 1
            out.append(("s", size, sign))
        else:
            out.append(("t", size, None))
    return out


def _sigmoid(z):
    zr = np.real(z)
    return np.where(
        zr >= 0,
        1.0 / (1.0 + np.exp(-np.where(zr >= 0, z, 0.0))),
        np.exp(np.where(zr >= 0, 0.0, z)) / (1.0 + np.exp(np.where(zr >= 0, 0.0, z))),
    )


class SmoothLapModel(om.ExplicitComponent):
    def initialize(self):
        self.options.declare("n_steps_straight", default=40, types=int)
        self.options.declare("n_steps_turn", default=20, types=int)
        self.options.declare(
            "turn_drift",
            default=False,
            types=bool,
            desc="carry the wind drift of each turn to the next straight (course.turn_drift)",
        )
        self.options.declare(
            "banner_stowed_legs",
            default=(),
            desc="lap-1 leg indices flown with the banner stowed "
            "(CD0 + dCD_stowed; course.banner_stowed_legs); () = no banner",
        )
        self.options.declare(
            "lap1_cluster",
            default=2.0,
            desc="grid exponent of lap 1's first straight: nodes at "
            "s_i = L (i/N)^p, clustered at its start where v - wind is smallest (Q45); 1 = uniform",
        )
        self.options.declare("tau_hold", default=0.5, desc="s, relaxation time of the speed hold at V")
        self.options.declare("a_smooth", default=0.01, desc="m/s^2, width of the smooth min(a, 0) in turns")
        self.options.declare("rho_ks", default=100.0)
        self.options.declare("pilot_reaction_time", default=3.0, desc="s per lap, performance_modeller.m")
        self.options.declare("lap1", default=LAP1)
        self.options.declare("lap_other", default=LAP_OTHER)
        self.options.declare("power_mode", default="aircrafter", values=["aircrafter", "propulsion"])
        self.options.declare("n_poly", default=4, types=int, desc="coefficients in the T(v), P(v) fits")
        self.options.declare(
            "takeoff",
            default=False,
            types=bool,
            desc="add the ground roll to lap 1 (propulsion mode only; course.TakeoffConfig)",
        )
        self.options.declare("start_offset_ft", default=0.0, desc="brake release behind the start line (take-off on)")
        self.options.declare("n_steps_takeoff", default=40, types=int)
        self.options.declare("a_floor", default=0.1, desc="m/s^2, smooth floor of the ground-roll acceleration")
        self.options.declare("a_floor_width", default=0.05, desc="m/s^2, width of that floor")
        self.options.declare("rem_floor", default=1.0, desc="m, smallest in-air remainder of lap 1's first straight")
        self.options.declare("rem_floor_width", default=0.5, desc="m, width of that floor")

    def setup(self):
        o = self.options
        self._lap1, self._other = normalise_legs(o["lap1"]), normalise_legs(o["lap_other"])
        if o["takeoff"]:
            if o["power_mode"] != "propulsion":
                raise ValueError(
                    "take-off in the mission time needs power_mode='propulsion': in 'aircrafter' mode "
                    "T = P / v is infinite at rest (course.takeoff.in_mission_time; lap_mdo keeps it off)"
                )
            first = self._lap1[0]
            if not (first[0] == "s" and first[2] == 1.0):
                raise ValueError("take-off in the mission time needs lap 1 to start with a head-wind straight")
        for name, val, units in [
            ("m", 5.0, "kg"),
            ("S", 0.24, "m**2"),
            ("V", 20.0, "m/s"),
            ("rho", 1.18, "kg/m**3"),
            ("CD0", 0.012, None),
            ("CD1", 0.0, None),
            ("k", 0.06, None),
            ("CL_max", 1.4, None),
            ("CL_to", 1.5, None),
            ("eta", 0.7, None),
            ("n_struct", 5.0, None),
            ("mu_roll", 0.02, None),
            ("takeoff_distance", 100.0, "m"),
            ("mission_time", 300.0, "s"),
            ("E_avail", 288000.0, "J"),
            ("g", 9.81, "m/s**2"),
            ("wind", 0.0, "m/s"),
            ("thrust_factor", 1.0, None),
            ("wind_dir", 0.0, "deg"),
            ("dCD_stowed", 0.0, None),
        ]:
            self.add_input(name, val, units=units)
        if self.options["power_mode"] == "propulsion":
            npoly = self.options["n_poly"]
            self.add_input("T_coeffs", np.zeros(npoly), desc="T(v) = sum t_i v^i (N, v in m/s)")
            self.add_input("P_coeffs", np.zeros(npoly), desc="P_batt(v) = sum p_i v^i (W)")
            self.add_output("thrust_margin_static", 1.0, units="N")
        if o["takeoff"]:
            self.add_input("k_lof", 1.1, desc="lift-off speed / V_stall")
            self.add_input("CL_ground", 0.15, desc="ground-roll lift coefficient (PLACEHOLDER)")
        for name, val, units in [
            ("Vstall", 15.0, "m/s"),
            ("Preq", 100.0, "W"),
            ("Tstatic", 15.0, "N"),
            ("Lap1time", 60.0, "s"),
            ("otherLapTime", 55.0, "s"),
            ("ThreeLapTime", 170.0, "s"),
            ("laps_frac", 5.0, None),
            ("stall_margin", 1.0, "m/s"),
            ("turn_speed_margin", 1.0, "m/s"),
            ("energy_3laps", 1e4, "J"),
        ]:
            self.add_output(name, val, units=units)
        for name, units in TAKEOFF_OUTPUTS:  # always declared; zero when take-off is off
            self.add_output(name, 0.0, units=units)
        self.add_output("lapCount", 5.0, desc="report only (step function)")
        self.declare_partials("*", "*", method="cs")
        self.declare_partials("lapCount", "*", dependent=False)
        if not o["takeoff"]:
            for name, _ in TAKEOFF_OUTPUTS:
                self.declare_partials(name, "*", dependent=False)

    # --- physics -------------------------------------------------------------------------
    def _model(self, x, record=None):
        """The lap model. record: a list to append one row per leg to (leg_breakdown); None in the gradient path."""
        o = self.options
        m, S, V, rho = x["m"], x["S"], x["V"], x["rho"]
        CD0, CD1, k, CLmax, g = x["CD0"], x["CD1"], x["k"], x["CL_max"], x["g"]
        cd = lambda cl: CD0 + CD1 * cl + k * cl**2  # noqa: E731
        W = m * g
        qS = lambda v: 0.5 * rho * v**2 * S  # noqa: E731

        Vstall = np.sqrt(W / (0.5 * rho * CLmax * S))
        propulsion = o["power_mode"] == "propulsion"
        if propulsion:
            tc, pc = x["T_coeffs"], x["P_coeffs"]
            thrust = lambda v: x["thrust_factor"] * sum(tc[i] * v**i for i in range(len(tc)))  # noqa: E731
            p_batt = lambda v: sum(pc[i] * v**i for i in range(len(pc)))  # noqa: E731
            P = p_batt(V)  # battery power at the equilibrium cruise speed
        else:
            D_cruise = qS(V) * cd(W / qS(V))
            P = D_cruise * V / x["eta"]
            thrust = lambda v: P / v  # noqa: E731
            p_batt = lambda v: P + 0.0 * v  # noqa: E731

        th = x["wind_dir"] * (np.pi / 180.0)
        along, cross = x["wind"] * np.cos(th), x["wind"] * np.sin(th)
        stowed = set(o["banner_stowed_legs"])
        cd_stowed = lambda cl: CD0 + x["dCD_stowed"] + CD1 * cl + k * cl**2  # noqa: E731

        def a_level(v, cdf=cd):  # straight, level: CL = W / qS
            return (thrust(v) - qS(v) * cdf(W / qS(v))) / m

        def a_capped(v, cdf=cd):
            a = a_level(v, cdf)
            if propulsion:  # fixed throttle: no speed hold, T = D sets the top speed
                return a
            # smooth min(a_level, (V - v) / tau): the pilot holds V
            return a - softplus(a - (V - v) / o["tau_hold"], o["a_smooth"])[0]

        def a_turn(v, cdf=cd):  # drag at CL_max (MATLAB deceleration tables), only the decelerating part
            a = (thrust(v) - qS(v) * cdf(CLmax)) / m
            return -softplus(-a, o["a_smooth"])[0]

        def omega(v):
            n_stall = qS(v) * CLmax / W
            n, _ = ks_min([n_stall, x["n_struct"]], o["rho_ks"])
            n = 1.0 + softplus(n - 1.0, 0.01)[0]
            return g * np.sqrt(n**2 - 1.0) / v

        def log(lap_no, kind, size, wind, v0, v1, t, e, banner="", drift=0.0):
            if record is not None:
                r = lambda z: float(np.ravel(np.real(z))[0])  # noqa: E731
                record.append(
                    dict(
                        lap=lap_no,
                        kind=kind,
                        size=r(size),
                        wind=wind,
                        v_entry=r(v0),
                        v_exit=r(v1),
                        time=r(t),
                        energy=r(e),
                        banner=banner,
                        drift_ft=r(drift),
                    )
                )

        def straight(v, dist, wind_sign, cluster=1.0, cdf=cd):
            # s is ground distance; ground speed = airspeed (crabbed) - wind_sign * along-track wind (+1 = headwind leg)
            N = o["n_steps_straight"]
            h = dist / N
            t = 0.0 * v
            e = 0.0 * v
            w = wind_sign * along
            if wind_sign == 0.0:  # a leg across the course axis: no along-track wind (as before)
                gs = lambda vv: vv - w  # noqa: E731
            else:
                gs = lambda vv: vv * np.sqrt(1.0 - (cross / vv) ** 2) - w  # noqa: E731
            f = lambda vv: (a_capped(vv, cdf) / gs(vv), 1.0 / gs(vv), p_batt(vv) / gs(vv))  # noqa: E731
            for i in range(N):  # RK4 in distance
                if cluster != 1.0:  # nodes s_i = dist (i/N)^p: short steps where the integrand is stiff (Q45)
                    h = dist * (((i + 1) / N) ** cluster - (i / N) ** cluster)
                k1 = f(v)
                k2 = f(v + 0.5 * h * k1[0])
                k3 = f(v + 0.5 * h * k2[0])
                k4 = f(v + h * k3[0])
                v, t, e = (
                    y + h / 6 * (a1 + 2 * a2 + 2 * a3 + a4) for y, a1, a2, a3, a4 in zip((v, t, e), k1, k2, k3, k4)
                )
            return v, t, e

        def turn(v, angle_deg, cdf=cd):
            t_turn = np.radians(angle_deg) / omega(v)
            N = o["n_steps_turn"]
            h = t_turn / N
            e = 0.0 * v
            f = lambda vv: (a_turn(vv, cdf), p_batt(vv))  # noqa: E731  d(v, E)/dt
            for _ in range(N):
                k1 = f(v)
                k2 = f(v + 0.5 * h * k1[0])
                k3 = f(v + 0.5 * h * k2[0])
                k4 = f(v + h * k3[0])
                v, e = (y + h / 6 * (a1 + 2 * a2 + 2 * a3 + a4) for y, a1, a2, a3, a4 in zip((v, e), k1, k2, k3, k4))
            return v, t_turn, e

        def ground_roll(w):
            """Rest to lift-off into a headwind w, in airspeed v from w to V_LOF: (t, s, E, smallest a)."""
            CLg = x["CL_ground"]

            cdg = cd_stowed if 0 in stowed else cd

            def a_raw(vv):
                return (thrust(vv) - qS(vv) * cdg(CLg) - x["mu_roll"] * (W - qS(vv) * CLg)) / m

            def f(vv):
                a = o["a_floor"] + softplus(a_raw(vv) - o["a_floor"], o["a_floor_width"])[0]
                return 1.0 / a, (vv - w) / a, p_batt(vv) / a

            N = o["n_steps_takeoff"]
            h = (V_lof - w) / N
            t, s, e = 0.0 * h, 0.0 * h, 0.0 * h
            a_nodes = [a_raw(w + 0.0 * h)]
            for i in range(N):  # RK4 in airspeed (the integrand does not depend on t, s, E: Simpson per step)
                v0 = w + i * h
                k1, k2, k4 = f(v0), f(v0 + 0.5 * h), f(v0 + h)
                t, s, e = (y + h / 6 * (a1 + 4 * a2 + a4) for y, a1, a2, a4 in zip((t, s, e), k1, k2, k4))
                a_nodes.append(a_raw(v0 + h))
            # smallest acceleration over the roll (report only): the node selected by real part
            i_min = int(np.argmin([np.real(np.ravel(a)[0]) for a in a_nodes]))
            return t, s, e, a_nodes[i_min]

        def lap(v, legs, lap_no, roll=None):
            total, energy, v_after_360 = 0.0 * v, 0.0 * v, None
            dx = dy = 0.0 * v  # wind drift of the turns since the last straight (course axis: +x = into the wind)
            for i, (kind, size, sign) in enumerate(legs):
                st = lap_no == 1 and i in stowed
                cdf = cd_stowed if st else cd
                ban = ("stowed" if st else "deployed") if stowed else ""
                if kind == "s":
                    wind = {1.0: "head", -1.0: "tail", 0.0: "none"}.get(sign, f"{sign:g}")
                    L = size * FT2M
                    delta = 0.0 * v
                    if o["turn_drift"] and sign != 0.0:  # the drift's along-track part shortens (or lengthens) the
                        delta = dx * sign  # leg, its cross-track part is recovered on it
                        L = np.sqrt((L - delta) ** 2 + dy**2)
                    dx = dy = 0.0 * v
                    if roll is not None and i == 0:  # lap 1's first straight: ground roll, then in the air from V_LOF
                        t_r, s_r, e_r = roll
                        log(lap_no, "ground roll", s_r / FT2M, wind, w_lap1 + 0.0 * v, v, t_r, e_r, ban)
                        v0 = v
                        v, t, e = straight(v, rem_eff, sign, o["lap1_cluster"], cdf)
                        log(lap_no, "straight", rem_eff / FT2M, wind, v0, v, t, e, ban)
                        t, e = t + t_r, e + e_r
                    else:
                        v0 = v
                        first = lap_no == 1 and i == 0  # lap 1 starts slow (V_stall or V_LOF): cluster its grid
                        v, t, e = straight(v, L, sign, o["lap1_cluster"] if first else 1.0, cdf)
                        log(lap_no, "straight", L / FT2M, wind, v0, v, t, e, ban, -delta / FT2M)
                else:
                    v0 = v
                    v, t, e = turn(v, size, cdf)
                    if o["turn_drift"]:  # the air mass moves the aircraft by (-along, -cross) t during the turn
                        dx, dy = dx - along * t, dy - cross * t
                    log(lap_no, "turn", size, "", v0, v, t, e, ban)
                    if size == 360 and v_after_360 is None:
                        v_after_360 = v
                total = total + t
                energy = energy + e
            return v, total, energy, v_after_360

        zero = 0.0 * Vstall
        to = dict(
            t_takeoff=zero,
            s_ground_roll=zero,
            V_lof=zero,
            E_takeoff=zero,
            takeoff_accel_margin=zero,
            takeoff_straight_margin=zero,
        )
        if o["takeoff"]:
            first = self._lap1[0]
            w_lap1 = first[2] * along  # along-runway component only (crosswind take-off limits not modelled)
            V_lof = x["k_lof"] * Vstall
            t_r, s_r, e_r, a_min = ground_roll(w_lap1)
            L1 = (first[1] + o["start_offset_ft"]) * FT2M  # the brake-release offset lengthens lap 1's first straight
            margin = L1 - s_r
            rem_eff = o["rem_floor"] + softplus(margin - o["rem_floor"], o["rem_floor_width"])[0]
            to = dict(
                t_takeoff=t_r,
                s_ground_roll=s_r,
                V_lof=V_lof,
                E_takeoff=e_r,
                takeoff_accel_margin=a_min,
                takeoff_straight_margin=margin,
            )
            v, lap1, e_lap1, v360 = lap(V_lof, self._lap1, 1, roll=(t_r, s_r, e_r))
        else:
            v, lap1, e_lap1, v360 = lap(Vstall, self._lap1, 1)
        if v360 is None:  # custom courses without a 360-degree turn: use the lap-1 exit speed
            v360 = v
        _, other, e_other, _ = lap(v, self._other, 2)

        # static thrust for the take-off ground roll (Eqs. 3.1-3.3, static_thrust_calculator.m)
        CLto = x["CL_to"]
        VR = 1.2 * np.sqrt(W / (0.5 * rho * S * CLto))
        Tstatic = m * VR**2 / (2.0 * x["takeoff_distance"]) + qS(VR) * cd(CLto) + x["mu_roll"] * W

        tr = o["pilot_reaction_time"]
        laps_time = 1.0 + (x["mission_time"] - (lap1 + tr)) / (other + tr)
        e_react = P * tr  # reaction time flown at cruise power
        laps_energy = 1.0 + (x["E_avail"] - (e_lap1 + e_react)) / (e_other + e_react)
        laps_frac, _ = ks_min(
            [1.0 + softplus(laps_time - 1.0, 0.05)[0], 1.0 + softplus(laps_energy - 1.0, 0.05)[0]], o["rho_ks"]
        )
        return dict(
            Vstall=Vstall,
            Preq=P,
            Tstatic=Tstatic,
            Lap1time=lap1,
            otherLapTime=other,
            ThreeLapTime=lap1 + 2.0 * other,
            laps_frac=laps_frac,
            stall_margin=V - 1.2 * Vstall,
            turn_speed_margin=v360 - Vstall,
            energy_3laps=e_lap1 + 2.0 * e_other + 3.0 * e_react,
            **to,
            **({"thrust_margin_static": thrust(0.0 * V) - Tstatic} if propulsion else {}),
        )

    def compute(self, inputs, outputs):
        if self.options["takeoff"]:
            # at rest into a headwind at or above the lift-off speed the aircraft is already flying: non-physical
            w = float(np.real(self._lap1[0][2] * inputs["wind"][0] * np.cos(inputs["wind_dir"][0] * np.pi / 180.0)))
            v_lof = float(
                np.real(inputs["k_lof"][0])
                * np.sqrt(
                    np.real(
                        inputs["m"][0]
                        * inputs["g"][0]
                        / (0.5 * inputs["rho"][0] * inputs["CL_max"][0] * inputs["S"][0])
                    )
                )
            )
            if not w < v_lof:
                from openmdao.core.analysis_error import AnalysisError

                # constraints_view.REGISTER matches this text line by line

                # fmt: off
                raise AnalysisError(f"{self.pathname}: non-physical lap (headwind {w:.2f} m/s >= lift-off speed "
                                    f"{v_lof:.2f} m/s): outside the take-off model's validity")
                # fmt: on
        res = self._model({k: inputs[k] for k in inputs})
        # a strong headwind near the lap-1 start speed (V_stall) drives the ground speed v - wind to zero and the
        # distance integration breaks down: report it as an analysis failure, never as a score
        lap_t = np.real(np.concatenate([np.ravel(res["Lap1time"]), np.ravel(res["otherLapTime"])]))
        to_vals = np.real(np.concatenate([np.ravel(res[k]) for k, _ in TAKEOFF_OUTPUTS]))
        if not (
            np.all(np.isfinite(lap_t))
            and np.all(lap_t > 0.0)
            and np.all(np.isfinite(np.real(res["laps_frac"])))
            and np.all(np.isfinite(to_vals))
        ):
            from openmdao.core.analysis_error import AnalysisError

            # constraints_view.REGISTER matches this text line by line

            # fmt: off
            raise AnalysisError(f"{self.pathname}: non-physical lap (lap times {lap_t}, wind "
                                f"{float(np.real(inputs['wind'][0])):.2f} m/s against V_stall "
                                f"{float(np.real(res['Vstall'][0])):.2f} m/s): outside the lap model's validity")
            # fmt: on
        for k, v in res.items():
            outputs[k] = v
        # integer count for reporting, mirroring the MATLAB loop's strict inequalities
        outputs["lapCount"] = max(1.0, np.ceil(np.real(res["laps_frac"])) - 1.0)

    def leg_breakdown(self):
        """Per-leg table of the last evaluated point (post-run; not an OpenMDAO output): one row per leg of lap 1 (with
        the ground roll as its own row when take-off is on) and of a lap 2+: lap, leg, kind, size (ft or deg; a
        straight's flown ground length), wind, v_entry, v_exit (m/s), time (s), energy (J), banner (stowed / deployed
        / empty without a banner), drift_ft (the turn drift that lengthened (+) or shortened (-) a straight)."""
        rows = []
        self._model({k: self._inputs[k] for k in self._inputs}, record=rows)
        leg = {}
        for r in rows:
            leg[r["lap"]] = leg.get(r["lap"], 0) + 1
            r["leg"] = leg[r["lap"]]
        return rows
