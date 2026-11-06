# Mission / performance

Code: `aircrafter2/mission/`. Source: Kallarackel (2025), *AirCrafter*, Ch. 3, and the MATLAB in
`THESIS/AirCrafter/`.

## Reference port (`lap_modeller.py`)

A faithful port of `performance_modeller.m`:
- take-off static thrust (Eqs. 3.1–3.3);
- constant-power cruise (Eqs. 3.4–3.7);
- Euler acceleration and deceleration lookup tables;
- stall-limited turn rate capped at n_struct;
- lap assembly from Table 3.5;
- lap count within the time and energy budget.

NaN flags are replaced by continuous margins (`stall_margin`, `turn_speed_margin`).

`matlab_compat=True` reproduces the MATLAB exactly. That includes its rounded V_stall
([plan-m1](../DECISIONS.md#plan-m1)) and its unflagged NaN when a turn is entered below the deceleration table
([plan-m7](../DECISIONS.md#plan-m7)). The port matches `performance_modeller.m`, run in MATLAB R2026a, to ~1e-8
(`docs/problems/lap_mdo.md`, `verification/matlab/run_reference.m`).

**Source files:** `performance_modeller.m`, `static_thrust_calculator.m`, `leg_calc_straight.m` and `leg_calc_turn.m`, in
`THESIS/AirCrafter/`. The method is Kallarackel (2025), Ch. 3:
1. Take-off static thrust required for a given ground roll (Sec. 3.2.2).
2. Cruise power required from the drag polar at constant power (Sec. 3.2.3).
3. Constant-power acceleration and deceleration lookup tables, Euler-integrating velocity against time, then integrating
   to get distance against time (Sec. 3.2.3).
4. A turn-rate-against-speed table, stall-limited up to the manoeuvre speed and capped at the structural (load-factor)
   limit above it (Sec. 3.2.3).
5. A lap assembled leg by leg from Table 3.5: straights use the acceleration table, turns use the deceleration and
   turn-rate tables. This gives the lap 1 time, the "other lap" time and the 3-lap time.
6. The lap count achievable within the mission time and energy budget.

**Deliberate deviations from the MATLAB, for OpenMDAO:**
- **Margins instead of a NaN flag.** The MATLAB screens infeasible points by setting `output.NaNflag = true` and returning
  early (for example cruise too close to stall, or decelerating below stall in the 360° turn). Hard branches and NaNs
  suit its brute-force grid search (`AirCrafter.m`), but give a gradient-based driver such as SLSQP nothing to climb. So
  this port always returns a value and reports two margins, feasible when ≥ 0: `stall_margin` = V − 1.2 V_stall and
  `turn_speed_margin` = v after the 360° turn − V_stall. Use them as constraints in place of the NaN flag.
- **No banner block.** The banner-mass and payload-fraction block in `performance_modeller.m` is specific to a past
  competition, not part of the general method, so it's left out. Add a mission-specific mass term to `design["m"]` if the
  rules need one.

**Reproduced as they are:**
- "Lap 1 Leg 1" (the take-off ground roll) is listed in Table 3.5, but its contribution to `Lap1time` is commented out
  (`% Lap1time = Lap1time + t`), so take-off time is never added. The smooth model has a switch for it
  (`course.takeoff.in_mission_time`, option `takeoff`, off by default; [tc-takeoff](../DECISIONS.md#tc-takeoff)).
- `Tstatic` is computed but otherwise unused downstream.

**`lapCount` is a step function of the inputs** (`_lap_count`). Its finite-difference partials are mostly zero, with
occasional large jumps. Don't use it as a gradient-based objective or constraint: use it for reporting, or drive it with
a derivative-free driver (such as `om.SimpleGADriver`). For gradient-based optimisation, use the continuous `ThreeLapTime`.

## Smooth twin (`smooth_lap.py`)

A smooth version of the lap model, so that gradients are exact ([DECISIONS plan-d2](../DECISIONS.md#plan-d2)).

The same physics without tables or loops, with exact complex-step partials:
- **Straights:** RK4 in distance.
- **Turns:** entry turn rate ω = g√(n² − 1)/v, with n = KSmin(q S C_L,max / W, n_struct). The C_L,max-drag
  deceleration is integrated in time, keeping only its negative part.
- **Lap count:** continuous `laps_frac`; the integer `lapCount` is for reporting only.

There are two power modes:

| `power_mode` | Thrust | Speed | Energy |
|---|---|---|---|
| `aircrafter` | P = D(V)V/η, T = P/v | held at V (smooth min) | P · t |
| `propulsion` ([plan-b](../DECISIONS.md#plan-b)) | T(v) from the powertrain | no cap; settles where T = D | ∫ P_batt(v) dt per leg |

### How each part of the lap is modelled

**Same physics as the reference port.** `SmoothLapModel` follows `lap_modeller.performance_modeller` (Kallarackel 2025,
Ch. 3, and `performance_modeller.m`), reformulated so that gradients are exact. All arithmetic is complex-step safe, and
the component uses `method="cs"` partials, which are exact.

**Drag polar.** C_D = CD0 + CD1 C_L + k C_L². With CD1 = 0 this is Eqs. 3.5–3.7 (static assumptions). The coupled model
supplies a three-point fit (`aero/polar.py`).

**Power, `aircrafter` mode.** Constant electrical power P = D(V) V / η (Eq. 3.4), thrust T = P / v, speed held at V.

**Power, `propulsion` mode** (used by `full_mdo`):
- Thrust T(v) and battery power P(v) come from the powertrain, as cubic fits in v (`propulsion/powertrain.py`), at a
  fixed throttle.
- There's no speed cap, so the aircraft accelerates on the straights towards the speed where T = D.
- The input V is that equilibrium cruise speed, solved in the MDA. It's used for the stall margin and the reaction-time
  energy only.
- Battery energy is integrated along every leg, and `laps_frac` uses the usable battery energy.
- `thrust_margin_static` = T(0) − Tstatic, for the take-off ground roll.

**Straights.** dv/ds = a(v) / v and dt/ds = 1 / v, with a(v) = (T − D) / m, integrated in distance with fixed-step RK4 (no
lookup tables, no while loops). The MATLAB tables stop accelerating at V, because the pilot holds the target speed. Here,
in `aircrafter` mode, the acceleration is a smooth min(a(v), (V − v) / `tau_hold`), so v relaxes onto V and stays there. A
plain multiplicative cap leaks: thrust exceeds drag at V, because P includes 1/η.

**Turns.** Duration = angle / ω(v_entry), with ω = g √(n² − 1) / v and n = KSmin(q S CL_max / W, n_struct) (Eqs. 3.10–3.12:
stall-limited turn rate, capped at the structural limit). During the turn the speed follows dv/dt = min(a_turn(v), 0) with
drag at C_L = CL_max, as in the MATLAB deceleration tables (a smooth min), integrated in time with RK4.

**Wind** ([plan: r26-wind](../DECISIONS.md#r26-wind)). A steady wind of speed `wind` from direction `wind_dir` (degrees
from the course axis; 0 = along it, the original model). It's valid while wind < V_stall: lap 1 starts at V_stall into the
wind, and a non-physical lap raises `AnalysisError`.
- **On a straight,** the along-track component a = wind cos(wind_dir) is a head- or tailwind by the leg's direction
  (`course.py`: head +1, tail −1, none 0), and the crosswind c = wind sin(wind_dir) is crabbed out: ground speed
  = v √(1 − (c/v)²) − sign · a, with energy and power on airspeed. At wind_dir = 0 this is v − sign · wind, bit for bit.
- **Legs without a direction.** Legs given as `("s", length_ft)` alternate head, tail, head, ...
- **Turns** are flown at constant airspeed in the air-mass frame. With `turn_drift` on (`course.turn_drift`; off by
  default, the original model), the drift wind × t_turn is carried to the next straight. Its along-track part shortens
  that straight (drift in its direction of flight) or lengthens it, and its cross-track part d is recovered on it, so the
  flown length is √((L − δ)² + d²).
- **The take-off ground roll** sees the along-runway component only.

**Banner (2025-26 M3).** The point's drag polar is the towing state. Lap 1's legs in `banner_stowed_legs` (before the first
downwind leg) are flown with CD0 + `dCD_stowed` (stowed instead of deployed banner), the ground roll included.

**Lap 1.** It starts at V_stall, with legs from the course (default Table 3.5). Take-off time isn't added, as in the MATLAB,
unless `takeoff=True` (next section). With it on, the ground roll is integrated in airspeed, with no event to detect, and
the acceleration floor keeps poor designs finite. `takeoff_accel_margin` is the smallest unfloored acceleration over the
roll. The rest of the first straight (its length + `start_offset_ft` − ground roll, kept positive smoothly;
`takeoff_straight_margin` = length − roll) is flown in the air from V_LOF.

**Lap count.** A continuous surrogate: `laps_frac` = KSmin(laps limited by time, laps limited by energy), where
laps(t) = 1 + (t − (lap1 + t_r)) / (lap_other + t_r) mirrors the MATLAB counting loop. The integer `lapCount` is for
reporting only.

**Finding L1:** `aircrafter` mode rewards cruise drag. More drag means more power on every leg, which the
optimiser exploited. That's why the full model uses `propulsion` mode.

Robustness hooks: `wind` (applied to each straight by its course wind direction; on `dbf_2025`, headwind on the
500 ft legs and tailwind on the 1000 ft leg) and `thrust_factor`. **Wind limit:** lap 1 starts at V_stall on the
into-wind leg (take-off off), and the straights are integrated in ground distance
(dt/ds = 1/(v − wind)). As the headwind nears V_stall the ground speed after lift-off goes to zero: the aircraft cannot
make progress into the wind and would not fly. That is the aircraft's wind limit, a property of the design, not a
model error. It is about 7.5 m/s for the light M3 aircraft of the multipoint optimum (V_stall 8.0–8.3 m/s) and lower
for bigger wings. A lap past it (non-finite, or a lap time ≤ 0) raises `AnalysisError`. Sampling records it as
`lap_invalid`, reported as "wind limit (no flight)": separate from MDA non-convergence and from constraint violation,
and never scored.

Agreement with the reference port: ≤ 0.65% on the 3-lap time across 8 points.

**Step resolution near the wind limit** (found in `docs/studies/takeoff_study.md` §6; PLACEHOLDERS Q45). The first
straight uses a fixed 40-step RK4 in ground distance, and dt/ds = 1/(v − w) grows as the ground speed falls.
- Below 5 m/s, M3's lap-1 time is within 0.15 % of a 640-step solution.
- Close to the limit the error grows. For the heavy M3 aircraft of multipoint_2L, with take-off off, it is +0.8 % at
  6 m/s and +16 % at 7 m/s.
- The validity edge is not always monotonic in wind.

## Take-off (`course.takeoff`, off by default)

The rules start every mission clock at the first throttle advance [R p. 11–12]. `performance_modeller.m` lists the
ground roll (l. 405) but leaves its time out (l. 435 commented). With `in_mission_time: false`, which is the default
and what every published result uses, the lap model does the same: lap 1 starts in the air at V_stall.

**Switched on**, lap 1's first straight starts with a ground roll from rest:
- **Equation of motion:** m dv/dt = T(v) − D(v) − μ_roll (W − L(v)).
  - L = q S `CL_ground` and D = q S (CD0 + CD1 C_L + k C_L²) at that C_L, from the point's polar. No ground effect.
  - T(v) and P_batt(v) are the point's powertrain fits, at the point's throttle.
- **Lift-off** at V_LOF = `k_lof` V_stall (`k_lof` 1.1, from the comment at `performance_modeller.m` l. 405).
- **Integration:** in airspeed from v = w (at rest into a headwind w) to V_LOF, with fixed-step RK4 (40 steps). There
  is no event to detect:
  - dt/dv = 1/a;
  - ds/dv = (v − w)/a, the ground distance;
  - dE/dv = P_batt/a.
- **The rest of the straight** (its length plus `start_offset_ft`, minus the roll) is flown in the air from V_LOF by
  the ordinary straight. The roll is part of the straight, not an extra leg.
- **Accounting:** the roll's time and energy go into `Lap1time` and the lap-1 energy. So `ThreeLapTime`, `laps_frac`,
  `energy_3laps` and the energy constraint all include it, for M1, M2 and M3 alike, because it is one base-config
  setting.
- **Robustness:**
  - the acceleration is floored smoothly with a softplus (0.1 m/s², width 0.05);
  - the in-air remainder is floored smoothly at 1 m (width 0.5);
  - a headwind at or above V_LOF, or a non-finite result, raises `AnalysisError` (same path as the wind limit). With
    take-off on, the wind limit moves from about V_stall to just under V_LOF.
- **Outputs**, always declared and zero when off: `t_takeoff`, `s_ground_roll`, `V_lof`, `E_takeoff`,
  `takeoff_accel_margin` (smallest acceleration over the roll) and `takeoff_straight_margin` (straight length − roll).
  They are reported, not constrained.
- **Constraint:** `thrust_margin_static` (Eq. 3.3, V_R = 1.2 V_stall) stays the take-off constraint, unchanged.
- **Assumptions:**
  - the aircraft is level at lift-off: no rotation, transition or climb is modelled;
  - the roll is at the point's throttle, while pilots take off at full throttle (a separate take-off throttle is not
    built: `docs/development/TAKEOFF_COURSE_PLAN.md`);
  - `CL_ground` = 0.15 is a PLACEHOLDER: a taildragger rolling tail-up at Woombah's 2° wing setting, on the model's
    own lift curve;
  - `start_offset_ft` = 0 (brake release on the start line) is a PLACEHOLDER.

  See PLACEHOLDERS Q40–Q44.
- **Power mode:** `propulsion` only. In `aircrafter` mode T = P/v is infinite at rest, so switching take-off on raises
  a config error. `lap_mdo` and the reference port (`lap_modeller.py`) keep take-off out.
- **Verification:** closed form to 1e-10; `solve_ivp` within 0.1 % at 6 cases; FD partials
  (`tests/test_takeoff_course.py`). Re-analysis of the published designs: `docs/studies/takeoff_study.md`.

## Course (`course`)

The course is data, not code (`aircrafter2/mission/course.py`). One lap from the start line is an ordered list of
legs:
- `{kind: straight, length_ft, wind: head | tail | none}`: `none` is a crosswind leg, with no along-track component;
- `{kind: turn, angle_deg}`: turns ignore drift.

Lengths are in feet and angles in degrees, as the rules write them.

**Fields:**
- `preset`: a file in `configs/courses/`;
- `legs`: inline legs, which override the preset;
- `lap1_legs`: an optional different first lap;
- `pilot_reaction_time_s`: 3.0 s per lap, from `performance_modeller.m`.

**Presets** (each file carries its citation):

| preset | legs | source |
|---|---|---|
| `dbf_2025` (default) | 500 ft head, 180°, 360°, 1000 ft tail, 180°, 500 ft head | Kallarackel (2025) Table 3.5; the course the lap model always flew, bit for bit |
| `dbf_2025_360_mid` | 500 head, 180, 500 tail, 360, 500 tail, 180, 500 head | Rules_2025 Figure 1 (p. 3): the 360 halfway down the back straight |
| `oval` | 500 head, 180, 1000 tail, 180, 500 head | the same markers (500 ft from the start line, p. 18), no 360 |

A preset file holds `name`, `citation`, `legs` and an optional `lap1_legs`. Legs given inline override the preset.

**Validation (`check_course`):**
- **Errors:**
  - an empty lap;
  - a lap with no straight;
  - a length or angle ≤ 0;
  - with take-off on, a lap 1 that does not start with a head-wind straight.
- **Warnings only,** because the lap model does not need a closed course:
  - total turning that is not a non-zero multiple of 360°;
  - head-wind and tail-wind distances that differ (the lap wouldn't end at the start line).

**Other rules:**
- `turn_speed_margin` is unchanged: the speed after the first 360° turn, or the lap-1 exit speed if there is none.
- `lap_mdo` keeps the fixed Table 3.5 course and has no course block.

**Leg breakdown:** `SmoothLapModel.leg_breakdown()` is a post-run per-leg table, not an OpenMDAO output. It covers
lap 1 and lap 2+, with columns lap, leg, kind, size, wind, entry and exit speed, time and energy, and gives the ground
roll its own row. `api.run_design` writes it to `summary.md` and `results.json` together with the course used.

**UI** (`aircrafter2/ui/app.py`, "Mission" and "Mission: course"):
- **Take-off:**
  - the take-off checkbox, with `k_lof`, `CL_ground` (PLACEHOLDER) and the start offset, all disabled while take-off
    is off;
  - `mu_roll` and the take-off distance stay editable either way, because the Eq. 3.3 constraint uses them.
- **Course:**
  - a preset selector, an editable leg table (an order column, rows added or deleted in the table or with "Add leg")
    and a second table for a different lap 1;
  - the pilot reaction time;
  - a top-view sketch (`aircrafter2/plotting/course_sketch.py`: straights to scale, schematic turn radii, start line,
    runway, wind arrow, leg numbers, lap length and total turning).
- **Validation:** errors disable Run, and warnings show above the sketch.
- **Save as preset** writes `configs/courses/<name>.yaml` and refuses the shipped names.
- **After a run:** t_takeoff, s_ground_roll and V_LOF are shown above the summary, which carries the course used and
  the leg breakdown.

## Oval (`oval.py`, used by `oval_lap`)

Constant-speed oval: R = V²/(g√(n² − 1)), lap = (2L_s + 2πR)/V, with two straights of length L_s and two 180° level
turns at constant speed V. Kept as a teaching and verification case.

The load factor comes from the turn analysis point, n = L_turn / W, smoothly capped at the structural limit n_struct
(KS-min). Its C_L is limited by the C_L,max margin constraint on the turn point, so the turn radius is set by the C_L
margin to C_L,max. n is floored smoothly just above 1 (1 + softplus(n − 1)), so the radius stays finite even at infeasible
iterates; feasible designs have n well above 1.
