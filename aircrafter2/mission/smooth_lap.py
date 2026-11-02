"""Smooth lap model: flies a mission's laps leg by leg, with exact gradients for the optimiser.

Takes the drag polar (aero/polar.py), the mass (mass/) and either the powertrain's thrust and battery power
(propulsion/powertrain.py) or AirCrafter's constant-power rule. Returns the lap times, the mission energy, the
fractional lap count and the stall, turn-speed and take-off thrust margins, which feed the score and the constraints.

Same physics as lap_modeller.performance_modeller (Kallarackel 2025, Ch. 3; performance_modeller.m), without tables or
while loops, so gradients are exact (docs/DECISIONS.md#plan-d2). In the full model the powertrain sets the power
(docs/DECISIONS.md#plan-b). Details: docs/modules/mission.md, "Smooth twin".
Abbreviations: MDA = multidisciplinary analysis.
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
    """Put a leg list into the (kind, size, wind_sign) form the model flies.

    Parameters
    ----------
    legs : list of tuple
        ("s", length_ft[, wind_sign]) for a straight, ("t", angle_deg[, ...]) for a turn. wind_sign is +1 head, -1 tail,
        0 none. Straights without a sign alternate head, tail, head, ...

    Returns
    -------
    list of tuple
        (kind, size, wind_sign), with wind_sign None for a turn.
    """
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
    """Logistic function of z, written so neither tail overflows. Complex-step safe. Nothing calls it."""
    zr = np.real(z)
    return np.where(
        zr >= 0,
        1.0 / (1.0 + np.exp(-np.where(zr >= 0, z, 0.0))),
        np.exp(np.where(zr >= 0, 0.0, z)) / (1.0 + np.exp(np.where(zr >= 0, 0.0, z))),
    )


class SmoothLapModel(om.ExplicitComponent):
    """Lap times, energy, lap count and margins of one mission point.

    Each leg is integrated with fixed-step RK4: straights in distance, turns in time, the ground roll in airspeed. The
    equations are in docs/modules/mission.md. Partials are complex step (method="cs"): exact, because all the arithmetic
    is complex-step safe, and the model is too branched to differentiate by hand.

    Options
    -------
    power_mode : "aircrafter" or "propulsion"
        "aircrafter": P = D(V) V / eta, T = P / v, speed held at V. "propulsion": T(v), P(v) from the powertrain fits.
    lap1, lap_other : list of tuple
        Legs of lap 1 and of later laps (normalise_legs). Default: Kallarackel 2025 Table 3.5.
    pilot_reaction_time : float
        s per lap (performance_modeller.m).
    takeoff : bool
        Add the ground roll to lap 1 (propulsion mode only; docs/DECISIONS.md#tc-takeoff).
    start_offset_ft : float
        ft, brake release behind the start line (take-off on).
    turn_drift : bool
        Carry each turn's wind drift to the next straight (docs/DECISIONS.md#r26-wind).
    banner_stowed_legs : tuple of int
        Lap-1 leg indices flown with the banner stowed; () = no banner (docs/DECISIONS.md#r26-banner).
    lap1_cluster : float
        Grid exponent of lap 1's first straight; 1 = uniform (placeholder, docs/OPEN_QUESTIONS.md Q45).
    n_steps_straight, n_steps_turn, n_steps_takeoff : int
        RK4 steps per leg.
    n_poly : int
        Coefficients in the T(v) and P(v) fits.
    tau_hold, a_smooth, rho_ks, a_floor, a_floor_width, rem_floor, rem_floor_width : float
        Smoothing widths, in the units their `desc` gives.

    Inputs
    ------
    m [kg], S [m**2], V [m/s], rho [kg/m**3], g [m/s**2]
        See Symbols.
    CD0, CD1, k, CL_max, CL_to, eta, n_struct, mu_roll : float
        See Symbols. CL_to: lift coefficient of the Eq. 3.3 take-off.
    takeoff_distance [m], mission_time [s], E_avail [J]
        Eq. 3.3 distance, mission window, energy available for the lap count.
    wind [m/s], wind_dir [deg], thrust_factor, dCD_stowed
        Robustness inputs and the stowed-banner drag increment.
    T_coeffs, P_coeffs : array, shape (n_poly,)
        Propulsion mode only. T(v) = sum t_i v^i [N], P_batt(v) = sum p_i v^i [W], v in m/s.
    k_lof, CL_ground : float
        Take-off on only. V_LOF / V_stall, and the ground-roll lift coefficient (placeholder).

    Outputs
    -------
    Vstall [m/s], Preq [W], Tstatic [N]
        Stall speed, power at V, the static thrust of Eqs. 3.1-3.3.
    Lap1time, otherLapTime, ThreeLapTime [s], energy_3laps [J]
        Lap times, and the energy for three laps with the reaction time.
    laps_frac : float
        Continuous lap count, KSmin of the laps limited by time and by energy.
    lapCount : float
        lapCount (integer) is reported only.
    stall_margin, turn_speed_margin [m/s], thrust_margin_static [N]
        V - 1.2 V_stall, speed after the 360 deg turn - V_stall, T(0) - Tstatic (propulsion mode only).
    t_takeoff [s], s_ground_roll [m], V_lof [m/s], E_takeoff [J], takeoff_accel_margin [m/s**2],
    takeoff_straight_margin [m]
        Ground roll results; always declared, zero when take-off is off.

    Symbols
    -------
    m : aircraft mass [kg]
    S : wing area [m**2]
    rho : air density [kg/m**3]
    g : gravity [m/s**2]
    W : weight, m g [N]
    V : cruise speed: the target speed (aircrafter mode) or the T = D speed from the MDA (propulsion mode) [m/s]
    v : airspeed along a leg [m/s]
    qS : dynamic pressure x wing area, 0.5 rho v^2 S [N]
    CD0, CD1, k : drag polar, CD = CD0 + CD1 CL + k CL^2 (Eqs. 3.5-3.7 with CD1 = 0)
    CLmax : maximum lift coefficient, the turn's lift limit
    Vstall : stall speed, sqrt(W / (0.5 rho CLmax S)) [m/s]
    eta : propulsive efficiency (aircrafter mode), Eq. 3.4
    P : battery power at V (propulsion mode) or D(V) V / eta (aircrafter mode) [W]
    n_struct : structural load factor limit
    n : turn load factor, KSmin(qS CLmax / W, n_struct)
    omega : turn rate, g sqrt(n^2 - 1) / v, Eqs. 3.10-3.12 [rad/s]
    along, cross : wind components along and across the course axis [m/s]
    L : flown length of a straight [m]
    N, h, k1..k4 : RK4 steps, step size and stages
    mu_roll : rolling friction coefficient
    VR : rotation speed of Eq. 3.3, 1.2 V_stall(CL_to) [m/s]
    """

    def initialize(self):
        """Declare the options listed in the class docstring."""
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
        """Declare inputs, outputs and partials. Raises ValueError if the course or mode can't fly a take-off."""
        opts = self.options
        self._lap1, self._other = normalise_legs(opts["lap1"]), normalise_legs(opts["lap_other"])
        if opts["takeoff"]:
            if opts["power_mode"] != "propulsion":
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
        if opts["takeoff"]:
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
        if not opts["takeoff"]:
            for name, _ in TAKEOFF_OUTPUTS:
                self.declare_partials(name, "*", dependent=False)

    def _model(self, inputs, record=None):
        """Fly lap 1 and a later lap, and return every output as a dict.

        Parameters
        ----------
        inputs : dict
            Input name -> value, as in the class docstring. Complex values are allowed (complex step).
        record : list, optional
            Appends one row per leg (leg_breakdown). None in the gradient path.

        Returns
        -------
        dict
            Output name -> value, without lapCount.
        """
        # --- polar and speeds ---
        opts = self.options
        m, S, V, rho = inputs["m"], inputs["S"], inputs["V"], inputs["rho"]
        CD0, CD1, k, CLmax, g = inputs["CD0"], inputs["CD1"], inputs["k"], inputs["CL_max"], inputs["g"]

        def cd(cl):
            """Drag coefficient at lift coefficient cl (the towing state, if there is a banner)."""
            return CD0 + CD1 * cl + k * cl**2

        W = m * g

        def qS(v):
            """Dynamic pressure x wing area at airspeed v [N]."""
            return 0.5 * rho * v**2 * S

        Vstall = np.sqrt(W / (0.5 * rho * CLmax * S))

        # --- thrust and battery power as functions of airspeed ---
        propulsion = opts["power_mode"] == "propulsion"
        if propulsion:
            thrust_coeffs, power_coeffs = inputs["T_coeffs"], inputs["P_coeffs"]

            def thrust(v):
                """Thrust at airspeed v, from the powertrain fit [N]."""
                return inputs["thrust_factor"] * sum(thrust_coeffs[i] * v**i for i in range(len(thrust_coeffs)))

            def p_batt(v):
                """Battery power at airspeed v, from the powertrain fit [W]."""
                return sum(power_coeffs[i] * v**i for i in range(len(power_coeffs)))

            P = p_batt(V)  # battery power at the equilibrium cruise speed
        else:
            D_cruise = qS(V) * cd(W / qS(V))
            P = D_cruise * V / inputs["eta"]

            def thrust(v):
                """Thrust at airspeed v under constant power [N]."""
                return P / v

            def p_batt(v):
                """Constant power, with the shape of v [W]."""
                return P + 0.0 * v

        # --- wind and banner ---
        wind_dir_rad = inputs["wind_dir"] * (np.pi / 180.0)
        along, cross = inputs["wind"] * np.cos(wind_dir_rad), inputs["wind"] * np.sin(wind_dir_rad)
        stowed = set(opts["banner_stowed_legs"])

        def cd_stowed(cl):
            """Drag coefficient with the banner stowed."""
            return CD0 + inputs["dCD_stowed"] + CD1 * cl + k * cl**2

        # --- accelerations and turn rate ---
        def a_level(v, polar=cd):
            """Acceleration in straight, level flight, where CL = W / qS [m/s^2]. polar: the CD(CL) to use."""
            return (thrust(v) - qS(v) * polar(W / qS(v))) / m

        def a_capped(v, polar=cd):
            """Acceleration on a straight, with the pilot's speed hold in aircrafter mode [m/s^2]."""
            a = a_level(v, polar)
            if propulsion:  # fixed throttle: no speed hold, T = D sets the top speed
                return a
            # smooth min(a_level, (V - v) / tau_hold): the pilot holds V
            return a - softplus(a - (V - v) / opts["tau_hold"], opts["a_smooth"])[0]

        def a_turn(v, polar=cd):
            """Turn deceleration: drag at CL_max, as in the MATLAB deceleration tables; negative part only [m/s^2]."""
            a = (thrust(v) - qS(v) * polar(CLmax)) / m
            return -softplus(-a, opts["a_smooth"])[0]

        def omega(v):
            """Turn rate at airspeed v: stall-limited, capped at n_struct (Eqs. 3.10-3.12) [rad/s]."""
            n_stall = qS(v) * CLmax / W
            n, _ = ks_min([n_stall, inputs["n_struct"]], opts["rho_ks"])
            n = 1.0 + softplus(n - 1.0, 0.01)[0]
            return g * np.sqrt(n**2 - 1.0) / v

        def log(lap_no, kind, size, wind, v0, v1, t, e, banner="", drift=0.0):
            """Append one leg to the breakdown when recording; does nothing in the gradient path."""
            if record is not None:

                def to_float(z):
                    """Real part of z as a float."""
                    return float(np.ravel(np.real(z))[0])

                record.append(
                    dict(
                        lap=lap_no,
                        kind=kind,
                        size=to_float(size),
                        wind=wind,
                        v_entry=to_float(v0),
                        v_exit=to_float(v1),
                        time=to_float(t),
                        energy=to_float(e),
                        banner=banner,
                        drift_ft=to_float(drift),
                    )
                )

        # --- the legs ---
        def straight(v, dist, wind_sign, cluster=1.0, polar=cd):
            """Fly a straight of ground length dist [m] from airspeed v, with RK4 in distance.

            s is ground distance. Ground speed = airspeed (crabbed) - wind_sign * along-track wind (+1 = headwind leg).
            Returns (exit speed [m/s], time [s], battery energy [J]).
            """
            N = opts["n_steps_straight"]
            h = dist / N
            t = 0.0 * v
            e = 0.0 * v
            w = wind_sign * along
            if wind_sign == 0.0:  # a leg across the course axis: no along-track wind

                def ground_speed(vv):
                    """Ground speed on this leg at airspeed vv [m/s]."""
                    return vv - w
            else:

                def ground_speed(vv):
                    """Ground speed on this leg at airspeed vv, crabbing out the crosswind [m/s]."""
                    return vv * np.sqrt(1.0 - (cross / vv) ** 2) - w

            def rates(vv):
                """d(v, t, E)/ds at airspeed vv."""
                return (a_capped(vv, polar) / ground_speed(vv), 1.0 / ground_speed(vv), p_batt(vv) / ground_speed(vv))

            for i in range(N):  # RK4 in distance
                if cluster != 1.0:  # nodes s_i = dist (i/N)^p: short steps where the integrand is stiff (Q45)
                    h = dist * (((i + 1) / N) ** cluster - (i / N) ** cluster)
                k1 = rates(v)
                k2 = rates(v + 0.5 * h * k1[0])
                k3 = rates(v + 0.5 * h * k2[0])
                k4 = rates(v + h * k3[0])
                v, t, e = (
                    y + h / 6 * (a1 + 2 * a2 + 2 * a3 + a4) for y, a1, a2, a3, a4 in zip((v, t, e), k1, k2, k3, k4)
                )
            return v, t, e

        def turn(v, angle_deg, polar=cd):
            """Fly a turn of angle_deg with RK4 in time. Returns (exit speed [m/s], time [s], battery energy [J])."""
            t_turn = np.radians(angle_deg) / omega(v)
            N = opts["n_steps_turn"]
            h = t_turn / N
            e = 0.0 * v

            def rates(vv):
                """d(v, E)/dt at airspeed vv."""
                return a_turn(vv, polar), p_batt(vv)

            for _ in range(N):
                k1 = rates(v)
                k2 = rates(v + 0.5 * h * k1[0])
                k3 = rates(v + 0.5 * h * k2[0])
                k4 = rates(v + h * k3[0])
                v, e = (y + h / 6 * (a1 + 2 * a2 + 2 * a3 + a4) for y, a1, a2, a3, a4 in zip((v, e), k1, k2, k3, k4))
            return v, t_turn, e

        def ground_roll(w):
            """Rest to lift-off into a headwind w, in airspeed v from w to V_LOF: (t, s, E, smallest a)."""
            CLg = inputs["CL_ground"]

            cd_ground = cd_stowed if 0 in stowed else cd

            def a_raw(vv):
                """Unfloored ground-roll acceleration at airspeed vv [m/s^2]."""
                return (thrust(vv) - qS(vv) * cd_ground(CLg) - inputs["mu_roll"] * (W - qS(vv) * CLg)) / m

            def rates(vv):
                """d(t, s, E)/dv at airspeed vv, with the acceleration floored smoothly."""
                a = opts["a_floor"] + softplus(a_raw(vv) - opts["a_floor"], opts["a_floor_width"])[0]
                return 1.0 / a, (vv - w) / a, p_batt(vv) / a

            N = opts["n_steps_takeoff"]
            h = (V_lof - w) / N
            t, s, e = 0.0 * h, 0.0 * h, 0.0 * h
            a_nodes = [a_raw(w + 0.0 * h)]
            for i in range(N):  # RK4 in airspeed (the integrand does not depend on t, s, E: Simpson per step)
                v0 = w + i * h
                k1, k2, k4 = rates(v0), rates(v0 + 0.5 * h), rates(v0 + h)
                t, s, e = (y + h / 6 * (a1 + 4 * a2 + a4) for y, a1, a2, a4 in zip((t, s, e), k1, k2, k4))
                a_nodes.append(a_raw(v0 + h))
            # smallest acceleration over the roll (report only): the node selected by real part
            i_min = int(np.argmin([np.real(np.ravel(a)[0]) for a in a_nodes]))
            return t, s, e, a_nodes[i_min]

        def lap(v, legs, lap_no, roll=None):
            """Fly the legs from speed v. roll: the ground roll's (t, s, E) on lap 1, or None.

            Returns (exit speed [m/s], lap time [s], lap energy [J], speed after the first 360 deg turn or None).
            """
            total, energy, v_after_360 = 0.0 * v, 0.0 * v, None
            dx = dy = 0.0 * v  # wind drift of the turns since the last straight (course axis: +x = into the wind)
            for i, (kind, size, sign) in enumerate(legs):
                is_stowed = lap_no == 1 and i in stowed
                polar = cd_stowed if is_stowed else cd
                banner_state = ("stowed" if is_stowed else "deployed") if stowed else ""
                if kind == "s":
                    wind = {1.0: "head", -1.0: "tail", 0.0: "none"}.get(sign, f"{sign:g}")
                    L = size * FT2M
                    delta = 0.0 * v
                    if opts["turn_drift"] and sign != 0.0:  # the drift's along-track part shortens (or lengthens) the
                        delta = dx * sign  # leg, its cross-track part is recovered on it
                        L = np.sqrt((L - delta) ** 2 + dy**2)
                    dx = dy = 0.0 * v
                    if roll is not None and i == 0:  # lap 1's first straight: ground roll, then in the air from V_LOF
                        t_r, s_r, e_r = roll
                        log(lap_no, "ground roll", s_r / FT2M, wind, w_lap1 + 0.0 * v, v, t_r, e_r, banner_state)
                        v0 = v
                        v, t, e = straight(v, rem_eff, sign, opts["lap1_cluster"], polar)
                        log(lap_no, "straight", rem_eff / FT2M, wind, v0, v, t, e, banner_state)
                        t, e = t + t_r, e + e_r
                    else:
                        v0 = v
                        first = lap_no == 1 and i == 0  # lap 1 starts slow (V_stall or V_LOF): cluster its grid
                        v, t, e = straight(v, L, sign, opts["lap1_cluster"] if first else 1.0, polar)
                        log(lap_no, "straight", L / FT2M, wind, v0, v, t, e, banner_state, -delta / FT2M)
                else:
                    v0 = v
                    v, t, e = turn(v, size, polar)
                    if opts["turn_drift"]:  # the air mass moves the aircraft by (-along, -cross) t during the turn
                        dx, dy = dx - along * t, dy - cross * t
                    log(lap_no, "turn", size, "", v0, v, t, e, banner_state)
                    if size == 360 and v_after_360 is None:
                        v_after_360 = v
                total = total + t
                energy = energy + e
            return v, total, energy, v_after_360

        # --- fly lap 1 (from the ground roll, or from V_stall) and a later lap ---
        zero = 0.0 * Vstall
        takeoff_out = dict(
            t_takeoff=zero,
            s_ground_roll=zero,
            V_lof=zero,
            E_takeoff=zero,
            takeoff_accel_margin=zero,
            takeoff_straight_margin=zero,
        )
        if opts["takeoff"]:
            first = self._lap1[0]
            w_lap1 = first[2] * along  # along-runway component only (crosswind take-off limits not modelled)
            V_lof = inputs["k_lof"] * Vstall
            t_r, s_r, e_r, a_min = ground_roll(w_lap1)
            L1 = (
                first[1] + opts["start_offset_ft"]
            ) * FT2M  # the brake-release offset lengthens lap 1's first straight
            margin = L1 - s_r
            rem_eff = opts["rem_floor"] + softplus(margin - opts["rem_floor"], opts["rem_floor_width"])[0]
            takeoff_out = dict(
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
        CLto = inputs["CL_to"]
        VR = 1.2 * np.sqrt(W / (0.5 * rho * S * CLto))
        Tstatic = m * VR**2 / (2.0 * inputs["takeoff_distance"]) + qS(VR) * cd(CLto) + inputs["mu_roll"] * W

        # --- lap count ---
        t_react = opts["pilot_reaction_time"]
        laps_time = 1.0 + (inputs["mission_time"] - (lap1 + t_react)) / (other + t_react)
        e_react = P * t_react  # reaction time flown at cruise power
        laps_energy = 1.0 + (inputs["E_avail"] - (e_lap1 + e_react)) / (e_other + e_react)
        laps_frac, _ = ks_min(
            [1.0 + softplus(laps_time - 1.0, 0.05)[0], 1.0 + softplus(laps_energy - 1.0, 0.05)[0]], opts["rho_ks"]
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
            **takeoff_out,
            **({"thrust_margin_static": thrust(0.0 * V) - Tstatic} if propulsion else {}),
        )

    def compute(self, inputs, outputs):
        """Run the model. Raises AnalysisError for a lap outside the model's validity (headwind, lap times)."""
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

                # fmt: off
                # constraints_view.REGISTER matches this text line by line
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

            # fmt: off
            # constraints_view.REGISTER matches this text line by line
            raise AnalysisError(f"{self.pathname}: non-physical lap (lap times {lap_t}, wind "
                                f"{float(np.real(inputs['wind'][0])):.2f} m/s against V_stall "
                                f"{float(np.real(res['Vstall'][0])):.2f} m/s): outside the lap model's validity")
            # fmt: on
        for k, v in res.items():
            outputs[k] = v
        # integer count for reporting, mirroring the MATLAB loop's strict inequalities
        outputs["lapCount"] = max(1.0, np.ceil(np.real(res["laps_frac"])) - 1.0)

    def leg_breakdown(self):
        """Per-leg table of the last evaluated point: post-run, not an OpenMDAO output.

        Returns
        -------
        list of dict
            One row per leg of lap 1 (with the ground roll as its own row when take-off is on) and of a lap 2+.
            Keys: lap, leg, kind, size (ft or deg; a straight's flown ground length), wind, v_entry, v_exit [m/s],
            time [s], energy [J], banner (stowed, deployed, or empty without a banner), drift_ft (the turn drift that
            lengthened (+) or shortened (-) a straight).
        """
        rows = []
        self._model({k: self._inputs[k] for k in self._inputs}, record=rows)
        leg = {}
        for r in rows:
            leg[r["lap"]] = leg.get(r["lap"], 0) + 1
            r["leg"] = leg[r["lap"]]
        return rows
