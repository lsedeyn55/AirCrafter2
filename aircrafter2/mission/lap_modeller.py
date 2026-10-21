"""AirCrafter DBF lap-performance modeller, ported to Python/OpenMDAO.

This is a from-scratch Python re-implementation of the methodology described
in Joshua Kallarackel's thesis, Chapter 3 ("AirCrafter Performance
Modeller"), and of the MATLAB source it documents:

    THESIS/AirCrafter/performance_modeller.m
    THESIS/AirCrafter/static_thrust_calculator.m
    THESIS/AirCrafter/leg_calc_straight.m
    THESIS/AirCrafter/leg_calc_turn.m

The physics is unchanged from the original:
  1. Take-off static thrust required for a given ground roll (Sec 3.2.2).
  2. Cruise power required from the drag polar at constant power (3.2.3).
  3. Constant-power acceleration/deceleration lookup tables built by Euler-
     integrating velocity vs time, then integrating to get distance vs time
     (3.2.3).
  4. A turn-rate-vs-speed table, stall-limited up to the manoeuvre speed and
     capped at the structural (load-factor) limit above it (3.2.3).
  5. A lap assembled leg-by-leg from Table 3.5 (straights use the
     accel table, turns use the decel + turn-rate tables), giving Lap 1 time,
     "other lap" time, and 3-lap time.
  6. Lap count achievable within the mission time/energy budget.

Deliberate deviations from the MATLAB, for OpenMDAO use
---------------------------------------------------------
The MATLAB version screens infeasible design points by setting
`output.NaNflag = true` and returning early (e.g. cruise speed too close to
stall, or the aircraft decelerating below stall speed in the 360 turn). Hard
branches/NaNs are fine for the original's brute-force grid search over design
points (see AirCrafter.m), but they give a gradient-based OpenMDAO driver
(e.g. SLSQP) nothing to climb. Instead, this module always returns a value
and additionally reports continuous *margins*:

  - `stall_margin`      = V - 1.2 * Vstall        (>= 0 is feasible)
  - `turn_speed_margin` = v_after_360turn - Vstall (>= 0 is feasible)

Use these as `>= 0` constraints in the optimisation problem in place of the
MATLAB NaNflag check.

The banner-mass/payload-fraction block in performance_modeller.m (specific
to a banner-carrying mission from a past competition year) has been left
out, since it is competition-rule-specific and not part of the general
methodology. Add a mission-specific mass term to `design["m"]` yourself if
your current rules need it.

Note also that in the original MATLAB, "Lap 1 Leg 1" (take-off ground roll)
is listed in Table 3.5 but its contribution to Lap1time is commented out
(`% Lap1time = Lap1time + t`) -- take-off time is never actually added.
`Tstatic` is computed but otherwise unused downstream. That gap is
reproduced as-is here (this port stays the MATLAB reference). The smooth
lap model used by the optimisation problems has a switch for it:
`course.takeoff.in_mission_time` (smooth_lap.py, option `takeoff`; off by
default; docs/TAKEOFF_COURSE_PLAN.md).

`lapCount` is inherently a step function of the inputs (see `_lap_count`)
and its finite-difference partials will mostly be zero with occasional large
jumps. Don't use it as a gradient-based objective/constraint; use it for
reporting, or drive it with a derivative-free OpenMDAO driver (e.g.
`om.SimpleGADriver`). For gradient-based optimisation, target the continuous
`ThreeLapTime` output instead.
"""

import numpy as np
import openmdao.api as om

G = 9.81  # m/s^2, matches the `g = 9.81` used throughout performance_modeller.m
FT2M = 0.3048

DEFAULT_ASSUMPTIONS = dict(
    Cd0=0.012,
    Clmax=1.4,
    Clto=1.5,
    e=0.85,
    n=5.0,
    mu=0.02,
    eta=0.7,
    E=0.8 * 100 * 3600.0,  # W-s -- 80% usable capacity of a 100 Wh pack
    payload_fraction=0.4,
)


def isa_density(altitude, T0=288.15, p0=101325.0, lapse_rate=0.0065, R=287.05287, g0=9.80665):
    """International Standard Atmosphere density, troposphere model (< 11 km).

    Equivalent to MATLAB's `atmosisa(altitude, "extended", "on")` for the
    altitudes relevant to this modeller.
    """
    T = T0 - lapse_rate * altitude
    p = p0 * (T / T0) ** (g0 / (lapse_rate * R))
    return p / (R * T)


def _cumtrapz(t, y):
    """Cumulative trapezoidal integral of y wrt t, starting at 0 (like MATLAB cumtrapz)."""
    s = np.zeros_like(t)
    s[1:] = np.cumsum(0.5 * (y[1:] + y[:-1]) * np.diff(t))
    return s


def _interp_monotonic(x, xp, fp):
    """np.interp that tolerates a descending xp (unlike np.interp itself)."""
    xp = np.asarray(xp)
    fp = np.asarray(fp)
    if xp[0] > xp[-1]:
        xp = xp[::-1]
        fp = fp[::-1]
    return np.interp(x, xp, fp)


def static_thrust_calculator(m, b, AR, sg, altitude, Cd0, Clto, e, mu, g=G):
    """Required static thrust for a given ground roll (Eq. 3.1-3.3)."""
    rho = isa_density(altitude)
    W = m * g
    S = b**2 / AR
    k = 1.0 / (np.pi * AR * e)
    Cdto = Cd0 + k * Clto**2
    VR = 1.2 * np.sqrt(2.0 * W / (rho * S * Clto))
    D = 0.5 * rho * VR**2 * S * Cdto
    return m * VR**2 / (2.0 * sg) + D + mu * W


def _build_accel_table(Vstall, V, Preq, W, m, rho, S, Cd0, k, t_step=0.1, max_count=100_000):
    """Constant-power acceleration lookup table, Vstall -> V (Sec 3.2.3)."""
    v_src = np.arange(Vstall, V, 0.01)
    v_src = np.append(v_src, V)
    T_src = Preq / v_src
    Cl_src = W / (0.5 * rho * v_src**2 * S)
    Cd_src = Cd0 + k * Cl_src**2
    D_src = Cd_src * 0.5 * rho * v_src**2 * S
    a_src = (T_src - D_src) / m

    v_time = [Vstall]
    t_time = [0.0]
    count = 0
    while v_time[-1] < V:
        a_current = np.interp(v_time[-1], v_src, a_src)
        if a_current <= 0.0:
            # Aircraft cannot reach the target cruise speed at this power setting.
            break
        v_time.append(v_time[-1] + a_current * t_step)
        t_time.append(t_time[-1] + t_step)
        count += 1
        if count > max_count:
            raise RuntimeError("Acceleration lookup table exceeded max iterations")

    v_time = np.array(v_time)
    t_time = np.array(t_time)
    # Extend by 100s at constant cruise speed, as in the MATLAB (allows
    # interpolation past the point where the aircraft holds V).
    v_time[-1] = V
    t_time[-1] += 100.0
    s_time = _cumtrapz(t_time, v_time)
    return v_time, t_time, s_time


def _build_decel_table(Vstall, V, Preq, W, m, rho, S, Cd0, k, Clmax, t_step=0.1, max_count=10_000):
    """Stall-limited-turn deceleration lookup table, V -> Vstall (Sec 3.2.3).

    Returns (v_time, t_time), or (None, None) if the aircraft has enough
    thrust to hold Clmax without net deceleration anywhere in [Vstall, V].
    """
    n_pts = max(int(round((V - Vstall) / 0.01)), 1) + 1
    v_src = np.linspace(V, Vstall, n_pts)  # descending
    T_src = Preq / v_src
    Cd_const = Cd0 + k * Clmax**2  # Cl fixed at Clmax -> highest-drag case
    D_src = Cd_const * 0.5 * rho * v_src**2 * S
    a_src = (T_src - D_src) / m

    mask = a_src < 0.0
    if not np.any(mask):
        return None, None
    v_src = v_src[mask]
    a_src = a_src[mask]
    v_min = v_src[-1]

    v_asc = v_src[::-1]
    a_asc = a_src[::-1]

    v_time = [V]
    t_time = [0.0]
    count = 0
    while v_time[-1] >= v_min:
        a_current = np.interp(v_time[-1], v_asc, a_asc)
        v_time.append(v_time[-1] + a_current * t_step)
        t_time.append(t_time[-1] + t_step)
        count += 1
        if count > max_count:
            raise RuntimeError("Deceleration lookup table exceeded max iterations")

    v_time = np.array(v_time)
    t_time = np.array(t_time)
    v_time[-1] = v_min
    t_time[-1] += 100.0
    return v_time, t_time


def _build_turn_rate_table(Vstall, V, W, rho, Clmax, S, n, g=G, v_broad_start=None):
    """Turn-rate vs speed, stall-limited then capped at the structural limit (Eq 3.10-3.12)."""

    def turn_rate(v):
        term = ((0.5 * rho * Clmax * S) / W) ** 2 * v**2 - 1.0 / v**2
        return g * np.sqrt(np.abs(term))

    # MATLAB starts this table at the *unrounded* stall speed (matters only with matlab_compat)
    start = Vstall if v_broad_start is None else v_broad_start
    v_broad = np.arange(start, 1.1 * V, 0.1)
    v_broad = np.append(v_broad, 1.1 * V)

    bank_stall = np.degrees(
        np.arccos(np.clip(W / (0.5 * rho * v_broad**2 * Clmax * S), -1.0, 1.0))
    )
    bank_load = np.degrees(np.arccos(np.clip(1.0 / n, -1.0, 1.0)))

    if bank_load <= bank_stall[0]:
        va = Vstall
    elif bank_load >= bank_stall[-1]:
        va = V  # structural limit never binds within the speed envelope
    else:
        va = _interp_monotonic(bank_load, bank_stall, v_broad)

    v_fine = np.arange(Vstall, va, 0.1)
    v_fine = np.append(v_fine, va)
    omega_fine = turn_rate(v_fine)

    v_final = np.append(v_fine, 1.1 * V)
    omega_final = np.append(omega_fine, omega_fine[-1])
    return v_final, omega_final


def _interp1(x, xp, fp, matlab_compat):
    """Linear interpolation. matlab_compat: NaN outside [min(xp), max(xp)] like MATLAB's interp1;
    otherwise numpy's clamping."""
    if matlab_compat:
        lo, hi = np.min(xp), np.max(xp)
        if not (lo <= x <= hi):
            return np.nan
    return _interp_monotonic(x, xp, fp)


def leg_calc_straight(v_time, s_time, t_time, V1, leg_dist, matlab_compat=False):
    """Exit velocity + time to cover leg_dist, entering a straight at V1."""
    initial_s = _interp1(V1, v_time, s_time, matlab_compat)
    initial_t = _interp1(V1, v_time, t_time, matlab_compat)
    dist = initial_s + leg_dist
    v = _interp1(dist, s_time, v_time, matlab_compat)
    final_t = _interp1(dist, s_time, t_time, matlab_compat)
    return v, final_t - initial_t


def leg_calc_turn(v_time_decel, omega_vs_v, t_time_decel, V1, turn_angle, v_for_omega, matlab_compat=False):
    """Exit velocity + time to complete turn_angle radians, entering at V1.

    If V1 is below the slowest speed in the deceleration table (where the C_L,max-drag
    deceleration ends), the aircraft does not slow down in the turn: v = V1. MATLAB's interp1
    returns NaN there instead, and nothing flags it (PLAN §8, M7); matlab_compat reproduces that.
    The original port clamped to the table edge, which spuriously sped the aircraft up.
    """
    omega = _interp1(V1, v_for_omega, omega_vs_v, matlab_compat)
    t = turn_angle / omega

    if v_time_decel is None or len(v_time_decel) == 0:
        return V1, t
    if not matlab_compat and V1 < np.min(v_time_decel):
        return V1, t

    initial_t = _interp1(V1, v_time_decel, t_time_decel, matlab_compat)
    v = _interp1(initial_t + t, t_time_decel, v_time_decel, matlab_compat)
    return v, t


def _lap_performance(Vstall, accel_tables, decel_tables, turn_tables, matlab_compat=False):
    """Assemble Lap 1 and "other lap" times leg-by-leg, per Table 3.5."""
    v_time, t_time, s_time = accel_tables
    v_time_decel, t_time_decel = decel_tables
    v_turn, omega_turn = turn_tables

    straight500 = 500 * FT2M

    def straight(v0, dist):
        return leg_calc_straight(v_time, s_time, t_time, v0, dist, matlab_compat)

    def turn(v0, angle):
        return leg_calc_turn(v_time_decel, omega_turn, t_time_decel, v0, angle, v_turn, matlab_compat)

    lap1 = 0.0
    v = Vstall
    v, t = straight(v, straight500); lap1 += t                # Lap1 Leg2 - 500ft
    v, t = turn(v, np.pi); lap1 += t                           # Lap1 Leg3 - 180deg
    v, t = turn(v, 2 * np.pi); lap1 += t                       # Lap1 Leg4 - 360deg
    v_after_360 = v
    v, t = straight(v, 2 * straight500); lap1 += t             # Lap1 Leg5 - 1000ft
    v, t = turn(v, np.pi); lap1 += t                           # Lap1 Leg6 - 180deg
    v, t = straight(v, straight500); lap1 += t                 # Lap1 Leg7 - 500ft

    other = 0.0
    v, t = straight(v, straight500); other += t                # Lap2+ Leg1 - 500ft
    v, t = turn(v, np.pi); other += t                          # Lap2+ Leg2 - 180deg
    v, t = turn(v, 2 * np.pi); other += t                      # Lap2+ Leg3 - 360deg
    v, t = straight(v, 2 * straight500); other += t            # Lap2+ Leg4 - 1000ft
    v, t = turn(v, np.pi); other += t                          # Lap2+ Leg5 - 180deg
    v, t = straight(v, straight500); other += t                # Lap2+ Leg6 - 500ft

    return lap1, other, v_after_360


def _lap_count(Preq, E, mission_time, lap1_time, other_lap_time, pilot_reaction_time=3.0, max_laps=1000):
    lap_count = 1
    flight_time = lap1_time + pilot_reaction_time
    while flight_time < mission_time and lap_count < max_laps:
        flight_time += other_lap_time + pilot_reaction_time
        if flight_time >= mission_time:
            break
        if Preq * flight_time > E:
            break
        lap_count += 1
    return lap_count


def _matlab_round(x):
    """MATLAB round(): halves away from zero (numpy rounds halves to even)."""
    return np.sign(x) * np.floor(np.abs(x) + 0.5)


def performance_modeller(assumptions, design, matlab_compat=False):
    """Full port of performance_modeller.m. See module docstring for outputs.

    matlab_compat=True reproduces the MATLAB exactly where it differs from the cleaned-up port:
    the stall speed is rounded to an integer, Vstall = round(sqrt(W / (0.5 rho Clmax S))), and
    that rounded value drives the stall check (1.2 round(Vstall) >= V), the lookup tables and the
    Vstall output (PLAN §8, M1). Use it only for the code check against MATLAB (lap_mdo report).

    `assumptions`: dict with Cd0, Clmax, Clto, e, n, mu, eta, E, payload_fraction.
    `design`: dict with m [kg], b [m], AR, V [m/s], altitude [m],
              takeoff_distance [m], mission_time [s], battery_fraction.
    """
    altitude = design["altitude"]
    rho = isa_density(altitude)

    Cd0 = assumptions["Cd0"]
    Clmax = assumptions["Clmax"]
    Clto = assumptions["Clto"]
    e = assumptions["e"]
    n = assumptions["n"]
    mu = assumptions["mu"]
    eta = assumptions["eta"]
    E = design["battery_fraction"] * assumptions["E"]

    m = design["m"]
    b = design["b"]
    AR = design["AR"]
    V = design["V"]
    sg = design["takeoff_distance"]
    mission_time = design["mission_time"]

    W = m * G
    S = b**2 / AR
    k = 1.0 / (np.pi * AR * e)

    Tstatic = static_thrust_calculator(m, b, AR, sg, altitude, Cd0, Clto, e, mu)

    Vstall_exact = np.sqrt(W / (0.5 * rho * Clmax * S))
    Vstall = _matlab_round(Vstall_exact) if matlab_compat else Vstall_exact
    stall_margin = V - 1.2 * Vstall

    q = 0.5 * rho * V**2
    Cl = W / (q * S)
    Cd = Cd0 + k * Cl**2
    D = Cd * q * S
    Preq = D * V / eta

    accel_tables = _build_accel_table(Vstall, V, Preq, W, m, rho, S, Cd0, k)
    v_time_decel, t_time_decel = _build_decel_table(Vstall, V, Preq, W, m, rho, S, Cd0, k, Clmax)
    turn_tables = _build_turn_rate_table(Vstall, V, W, rho, Clmax, S, n,
                                         v_broad_start=Vstall_exact if matlab_compat else None)

    lap1, other, v_after_360 = _lap_performance(
        Vstall, accel_tables, (v_time_decel, t_time_decel), turn_tables, matlab_compat
    )
    turn_speed_margin = v_after_360 - Vstall

    lap_count = _lap_count(Preq, E, mission_time, lap1, other)

    return dict(
        Tstatic=Tstatic,
        Preq=Preq,
        Vstall=Vstall,
        Lap1time=lap1,
        otherLapTime=other,
        ThreeLapTime=lap1 + 2.0 * other,
        lapCount=lap_count,
        stall_margin=stall_margin,
        turn_speed_margin=turn_speed_margin,
    )


class AirCrafterPerformance(om.ExplicitComponent):
    """OpenMDAO wrapper around `performance_modeller`.

    Assumptions (Cd0, Clmax, Clto, e, n, mu, eta, E, payload_fraction) are
    fixed physical/competition constants, so they are a component `option`
    rather than an OpenMDAO input -- pass a dict overriding any of
    DEFAULT_ASSUMPTIONS at instantiation, e.g.:

        AirCrafterPerformance(assumptions={"Clmax": 1.3, "n": 4})

    FD partials are used (not complex-step) because the underlying algorithm
    is a while-loop + linear interpolation, not a complex-safe analytic
    expression.
    """

    def initialize(self):
        self.options.declare("assumptions", types=dict, default=None)

    def setup(self):
        assumptions = DEFAULT_ASSUMPTIONS.copy()
        if self.options["assumptions"]:
            assumptions.update(self.options["assumptions"])
        self._assumptions = assumptions

        self.add_input("m", val=5.0, units="kg")
        self.add_input("b", val=1.2, units="m")
        self.add_input("AR", val=6.0)
        self.add_input("V", val=20.0, units="m/s")
        self.add_input("altitude", val=400.0, units="m")
        self.add_input("takeoff_distance", val=100.0, units="m")
        self.add_input("mission_time", val=300.0, units="s")
        self.add_input("battery_fraction", val=1.0)

        self.add_output("Tstatic", val=1.0, units="N")
        self.add_output("Preq", val=1.0, units="W")
        self.add_output("Vstall", val=1.0, units="m/s")
        self.add_output("Lap1time", val=1.0, units="s")
        self.add_output("otherLapTime", val=1.0, units="s")
        self.add_output("ThreeLapTime", val=1.0, units="s")
        self.add_output("lapCount", val=1.0)
        self.add_output("stall_margin", val=0.0, units="m/s")
        self.add_output("turn_speed_margin", val=0.0, units="m/s")

        self.declare_partials(
            of=["Tstatic", "Preq", "Vstall", "Lap1time", "otherLapTime",
                "ThreeLapTime", "stall_margin", "turn_speed_margin"],
            wrt=["m", "b", "AR", "V", "altitude", "takeoff_distance",
                 "mission_time", "battery_fraction"],
            method="fd", step=1e-4, form="forward",
        )
        # lapCount is a step function -- report it, but don't ask the driver
        # to differentiate it (partials left at the zero default).

    def compute(self, inputs, outputs):
        design = dict(
            m=inputs["m"][0],
            b=inputs["b"][0],
            AR=inputs["AR"][0],
            V=inputs["V"][0],
            altitude=inputs["altitude"][0],
            takeoff_distance=inputs["takeoff_distance"][0],
            mission_time=inputs["mission_time"][0],
            battery_fraction=inputs["battery_fraction"][0],
        )
        result = performance_modeller(self._assumptions, design)
        for name, value in result.items():
            outputs[name] = value


if __name__ == "__main__":
    # Demo: minimise 3-lap time subject to stall/turn feasibility margins,
    # over the ranges from inputs_file.m.
    prob = om.Problem(reports=False)
    model = prob.model

    model.add_subsystem("perf", AirCrafterPerformance(), promotes=["*"])

    prob.driver = om.ScipyOptimizeDriver()
    prob.driver.options["optimizer"] = "SLSQP"
    prob.driver.options["tol"] = 1e-6
    prob.driver.options["disp"] = True

    model.add_design_var("m", lower=3.0, upper=7.0)
    model.add_design_var("b", lower=0.9, upper=1.5)
    model.add_design_var("AR", lower=4.0, upper=9.0)
    model.add_design_var("V", lower=10.0, upper=30.0)

    model.add_objective("ThreeLapTime")
    model.add_constraint("stall_margin", lower=0.0)
    model.add_constraint("turn_speed_margin", lower=0.0)

    prob.setup()
    prob.set_val("altitude", 400.0)
    prob.set_val("takeoff_distance", 100.0)
    prob.set_val("mission_time", 300.0)
    prob.set_val("battery_fraction", 1.0)

    prob.set_val("m", 5.0)
    prob.set_val("b", 1.2)
    prob.set_val("AR", 6.0)
    prob.set_val("V", 20.0)

    prob.run_driver()

    print(f"Mass:           {prob['m'][0]:.3f} kg")
    print(f"Wingspan:       {prob['b'][0]:.3f} m")
    print(f"Aspect Ratio:   {prob['AR'][0]:.3f}")
    print(f"Cruise Speed:   {prob['V'][0]:.3f} m/s")
    print(f"Stall Speed:    {prob['Vstall'][0]:.3f} m/s")
    print(f"Required Power: {prob['Preq'][0]:.2f} W")
    print(f"Static Thrust:  {prob['Tstatic'][0]:.2f} N")
    print(f"Lap 1 Time:     {prob['Lap1time'][0]:.2f} s")
    print(f"Other Lap Time: {prob['otherLapTime'][0]:.2f} s")
    print(f"3-Lap Time:     {prob['ThreeLapTime'][0]:.2f} s")
    print(f"Lap Count:      {prob['lapCount'][0]:.0f}")
