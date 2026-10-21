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

`matlab_compat=True` reproduces the MATLAB exactly. That includes its rounded V_stall (PLAN M1) and its unflagged
NaN when a turn is entered below the deceleration table (M7). The port matches `performance_modeller.m`, run in
MATLAB R2026a, to ~1e-8 (`docs/problems/lap_mdo.md`, `verification/matlab/run_reference.m`).

## Smooth twin (`smooth_lap.py`, decision D2)

The same physics without tables or loops, with exact complex-step partials:
- **Straights:** RK4 in distance.
- **Turns:** entry turn rate ω = g√(n² − 1)/v, with n = KSmin(q S C_L,max / W, n_struct). The C_L,max-drag
  deceleration is integrated in time, keeping only its negative part.
- **Lap count:** continuous `laps_frac`; the integer `lapCount` is for reporting only.

There are two power modes:

| `power_mode` | Thrust | Speed | Energy |
|---|---|---|---|
| `aircrafter` | P = D(V)V/η, T = P/v | held at V (smooth min) | P · t |
| `propulsion` (decision B) | T(v) from the powertrain | no cap; settles where T = D | ∫ P_batt(v) dt per leg |

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

**Step resolution near the wind limit** (found in `docs/studies/takeoff_study.md` §6; OPEN_QUESTIONS Q45). The first
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
    built: `docs/TAKEOFF_COURSE_PLAN.md`);
  - `CL_ground` = 0.15 is a PLACEHOLDER: a taildragger rolling tail-up at Woombah's 2° wing setting, on the model's
    own lift curve;
  - `start_offset_ft` = 0 (brake release on the start line) is a PLACEHOLDER.

  See OPEN_QUESTIONS Q40–Q44.
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

**Validation:**
- **Errors:**
  - an empty lap;
  - a lap with no straight;
  - a length or angle ≤ 0;
  - with take-off on, a lap 1 that does not start with a head-wind straight.
- **Warnings only,** because the lap model does not need a closed course:
  - total turning that is not a non-zero multiple of 360°;
  - head-wind and tail-wind distances that differ.

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

Constant-speed oval: R = V²/(g√(n² − 1)), lap = (2L_s + 2πR)/V. Kept as a teaching and verification case.
