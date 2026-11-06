# Take-off in the mission time and an editable course

Design notes for counting the take-off ground roll in the mission time and making the course editable (branch
`takeoff-course`): why, what was built, the ground lift coefficient, the check against the integrated ground roll,
verification and re-optimisation.
Re-analysis data: `outputs/takeoff/reanalysis.json` (`studies/takeoff_study.py`). Nothing published was rerun or
overwritten.

## Why

The rules start the clock "when the airplane throttle is advanced for the first take-off (or attempt)" for M1, M2 and
M3 (Rules_2025.pdf p. 11–12). There is no take-off distance limit (p. 10). The lap model inherited AirCrafter's
gap: `performance_modeller.m` l. 405 lists the ground roll as lap 1 leg 1, but l. 435 (`% Lap1time = Lap1time + t`)
is commented out. So every model M2 time is too short and every M3 lap count too high, by an amount that depends on
the design (below).

## What was built

**Course (`aircrafter2/mission/course.py`, `configs/courses/`)**
- `course` block in `FullMDOConfig` (pydantic, `extra="forbid"`): `preset`, `legs`, `lap1_legs`,
  `pilot_reaction_time_s`, `takeoff`. Legs: `{kind: straight, length_ft, wind: head|tail|none}` or
  `{kind: turn, angle_deg}`. Inline legs override the preset.
- Presets, each with a citation line:
  - `dbf_2025` (default): Table 3.5, every lap;
  - `dbf_2025_360_mid`: the 360° on the back straight, as Figure 1 draws it;
  - `oval`: no 360° turn.
- Each straight carries its own wind sign (+1 head, −1 tail, 0 none). The position-based alternation in `lap()` is
  gone; legacy 2-tuples `("s", 500)` still alternate, so old callers are unchanged.
- Validation errors: an empty lap, no straight, a length or angle ≤ 0, and (take-off on) a lap 1 that doesn't start with
  a head-wind straight. Warnings: turning not a multiple of 360°, unequal head/tail distances.
- `SmoothLapModel.leg_breakdown()`: a per-leg table (lap 1 with the ground roll as its own row, and lap 2+) after a
  run. `api.run_design` writes it to `summary.md` and `results.json`, together with the course used.
- `mu_roll` and `takeoff_distance` stay top-level (the Eq. 3.3 constraint uses them); `lap_mdo` has no course block
  and keeps Table 3.5.

**Take-off (`smooth_lap.py`, option `takeoff`; config `course.takeoff.in_mission_time`, default false)**
- One setting in the base config, so M2, M3 and `m1_check` all use it.
- Ground roll from rest to lift-off on lap 1's first straight, into the wind, at the point's throttle:
  m dv/dt = T(v) − D(v) − μ_roll (W − L(v)), with L and D at `CL_ground` on the existing polar (no ground effect).
  Lift-off at V_LOF = `k_lof` V_stall (`k_lof` = 1.1, `performance_modeller.m` l. 405).
- Integrated in airspeed from v = w (at rest into a headwind w) to V_LOF with fixed-step RK4 (40 steps):
  dt/dv = 1/a, ds/dv = (v − w)/a, dE/dv = P_batt(v)/a. No event detection; complex-step safe, `method="cs"` partials.
- The rest of the first straight (length + `start_offset_ft` − roll) is flown in the air from V_LOF with the existing
  `straight()`. The roll's time and energy go into `Lap1time` and the lap-1 energy, so `ThreeLapTime`, `laps_frac`,
  `energy_3laps` and the energy constraint all include it.
- Robustness for poor designs:
  - a = a_floor + softplus(a_raw − a_floor) with a_floor = 0.1 m/s² and width 0.05;
  - the in-air remainder is floored the same way at 1 m;
  - `takeoff_accel_margin` (smallest unfloored a over the roll) and `takeoff_straight_margin` (straight − roll) are
    reported, not constrained.
  - `AnalysisError` only for non-physical cases: a headwind ≥ V_LOF, or a non-finite result.
- New outputs, always declared and zero when off: `t_takeoff`, `s_ground_roll`, `V_lof`, `E_takeoff` and the two
  margins.
- `power_mode="aircrafter"` with take-off on raises a config error (T = P/v is infinite at rest).
- Not modelled: rotation, transition and climb (the aircraft is level at lift-off).
- `thrust_margin_static` (Eq. 3.3) is unchanged and stays the take-off constraint.
- `configs/dbf2025_multipoint_takeoff.yaml` extends `dbf2025_multipoint.yaml` with the switch on.

## `CL_ground` proposal: 0.15 (PLACEHOLDER)

- **Woombah:**
  - taildragger, 6.8 in mains, 2.0 in tailwheel, tail-down angle 15° (report §4.3.5);
  - 2° wing setting angle (§4.3.2, "A 2° wing setting angle, optimized via XFLR5, minimizes cruise drag");
  - takes off at full throttle with flaps (§4.7.1).
- **Assumption:** a taildragger lifts its tail early and rolls on the mains roughly level, so for most of the roll
  the wing sits at its setting angle.
- **The model's lift:** the framework has no incidence, and its VLM lift is linear through zero (C_L = 0 at α = 0).
  At α = 2°, C_L = C_L,α × 2° = 0.144 (Woombah), 0.139 (AirCrafter 1.0 direct) and 0.151 (multipoint), from each
  design's own lift curve. So the proposal is **0.15**.
- **What it leaves out:** flaps and Clark-Y camber would give the real wing more lift. Neither is in the model's
  C_L(α).
- **It hardly matters.** At μ = 0.02 friction is small, and more C_L adds induced drag (k C_L²) roughly as fast as
  it removes friction.

  | CL_ground | 0 | 0.15 | 0.5 | 1.0 |
  |---|---:|---:|---:|---:|
  | multipoint M2 roll (ft) | 78 | 78 | 78 | 80 |
  | multipoint_2L M2 roll (ft) | 159 | 158 | 160 | 177 |

  The M3 rolls change by under 1 ft.

  The first pass of the re-analysis used 0.5. The default was set to 0.15 on the reasoning above, before any
  write-up. Both passes are in the JSON (`CLg0.5`). They differ by ≤ 0.0006 in F.

## Eq. 3.3 against the integrated ground roll (published designs)

Eq. 3.3 solved for s_g with each design's own static thrust T(0):
s_g = m V_R² / (2 (T(0) − q(V_R) S C_D(C_L,max) − μW)), with V_R = 1.2 V_stall. Integrated: take-off on, at the
point's throttle, `CL_ground` = 0.15.

| design | pt | throttle | integrated roll (ft) | t (s) | V_LOF (m/s) | Eq. 3.3 roll (ft) | V_R (m/s) | full-throttle roll (ft) / t (s) |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| woombah | M2 | 0.678 | 79 | 2.91 | 15.82 | 104 | 17.26 | 42 / 1.58 |
| woombah | M3 | 0.537 | 12 | 0.78 | 9.23 | 15 | 10.07 | 4 / 0.29 |
| ac1_direct | M2 | 0.760 | 100 | 3.40 | 17.04 | 139 | 18.59 | 64 / 2.24 |
| ac1_direct | M3 | 0.597 | 8 | 0.57 | 8.56 | 10 | 9.34 | 4 / 0.26 |
| ac1_completed | M2 | 0.825 | 132 | 4.59 | 16.47 | 207 | 17.97 | 92 / 3.24 |
| ac1_completed | M3 | 0.729 | 9 | 0.65 | 8.32 | 11 | 9.07 | 5 / 0.38 |
| multipoint | M2 | 0.823 | 78 | 2.83 | 15.88 | 97 | 17.33 | 55 / 2.03 |
| multipoint | M3 | 0.808 | 10 | 0.64 | 9.13 | 12 | 9.96 | 7 / 0.44 |
| multipoint_2L | M2 | 0.872 | 158 | 5.38 | 16.88 | 296 | 18.41 | 122 / 4.21 |
| multipoint_2L | M3 | 0.802 | 9 | 0.66 | 7.78 | 11 | 8.48 | 6 / 0.45 |
| rdo (rdor_ms0) | M2 | 0.947 | 82 | 2.91 | 16.24 | 101 | 17.71 | 74 / 2.64 |
| rdo (rdor_ms0) | M3 | 0.934 | 10 | 0.64 | 9.32 | 12 | 10.16 | 9 / 0.57 |
| bottle 0.5 L | M2 | 0.794 | 59 | 1.95 | 17.39 | 65 | 18.97 | 39 / 1.31 |
| bottle 0.5 L | M3 | 0.846 | 15 | 0.73 | 12.33 | 17 | 13.45 | 11 / 0.55 |

- **Eq. 3.3 is longer every time:** 10–25 % for the light cases and up to 1.9× for multipoint_2L. Three things make
  it so: it rolls to V_R = 1.2 V_stall (against 1.1), it holds the drag at C_L,max for the whole roll, and it takes
  T(0) for the whole roll. On the heavy, slow-accelerating 2 L design the last one is a better approximation than it
  looks, but the drag at C_L,max is not.
- **Eq. 3.3 stays a conservative constraint.** Every roll is far inside its 100 m (328 ft) take-off distance, so the
  constraint is inactive at all of these designs, as it was in the published results.
- **Lap 2+ is unaffected.** The ground roll is 2–5 s at M2 and under 1 s at M3, where the aircraft is light and
  V_stall is low.
- **The one-line re-analysis** (written up in `docs/studies/takeoff_study.md`, which also covers the robust
  statistics: with take-off on, the RDO design violates two robust constraints and its mu − kσ falls 5e-4 below
  the deterministic optimum's):
  - ΔF ranges from −0.0069 (0.5 L) to −0.053 (2 L), almost all of it from M2; ΔM3 = −0.0007 everywhere
    (−0.02 laps).
  - The ranking doesn't change: bottles 0.5 L < 1 L < 2 L; AirCrafter 1.0 direct > completed; Woombah > multipoint >
    RDO at nominal, as before.
  - **Every published optimum whose M2 energy constraint was active now violates it**, by the take-off energy:
    −3.1 kJ (−0.87 Wh) for multipoint, −6.2 kJ for multipoint_2L. Re-optimising would buy it back with throttle or
    capacity; the cost estimate is below.

## Take-off throttle (assumption to flag; not built)

The roll uses the point's throttle (0.54–0.95 here). Woombah takes off at full throttle with flaps (§4.7.1) and
budgets "a 5-second take-off at 99 % throttle" (§4.7.3).

**Estimate of the effect.** Each point was re-analysed at throttle 1.0 and only its roll read. That's an upper bound
on the change: the polar and the masses barely move with throttle.
- M2 roll time: −0.8 s for multipoint and −1.3 s for Woombah, about 30–45 % shorter.
- M3 roll time: −0.2 to −0.5 s.
- So ΔF from take-off would shrink by roughly 30–45 %, but not vanish.

**What building it would cost.**
- A second T(v)/P_batt(v) fit per point: one more `Powertrain` evaluation per point, at a `throttle_TO` input
  (default 1.0), feeding only the ground roll.
- Its cost is one more powertrain compute + cs linearisation per point per evaluation. Measured at the multipoint optimum, single-threaded: 0.004 s compute + 0.022 s linearise against 0.44 s per model run + 2.43 s per `compute_totals`. That's **+1.8 % per optimiser iteration**: cheap.
- The rule constraints wouldn't change. The take-off current is the static current at full throttle, which
  `I_margin` / `fuse_margin` already check at the point's throttle, so a full-throttle take-off would make those
  checks optimistic. That interaction is the real cost and would have to be decided with it.

## Verification (all in `tests/test_takeoff_course.py`; 24 tests)

- **Off = unchanged:**
  - bit for bit against `configs_reference.json`, `rdo_off_reference.json`, `kv_dv_reference.json` and
    `multipoint_nominal_reference.json`, through the existing tests, none regenerated;
  - the default preset with explicit wind signs equals the old alternation bit for bit, at 0 and 3 m/s;
  - the off-re-analysis reproduces the published nominal F of every design (e.g. rdo 5.138586, multipoint 5.1394).
- **Closed form:** with constant T and no drag or lift, t, s and E are correct to 1e-10, at 0 and 3 m/s headwind.
- **Independent integrator:** `solve_ivp` in time (event at V_LOF, rtol 1e-10) at 6 cases, 2 of them with wind,
  agrees within 0.1 % on t, s and E.
- **Partials:** central FD against the cs partials (`check_partials_fd`, as for the existing lap tests), with take-off
  on, with wind, and on a custom course with a start offset.
- **Courses:**
  - a custom course runs;
  - `oval` gives a shorter lap than `dbf_2025`;
  - the error cases: empty lap, no straight, a zero length, and a tail-first lap 1 with take-off on;
  - the warning case;
  - save-as-preset refuses the shipped names.
- **Smoke:** the multipoint problem and `m1_check` with take-off on; one RDO problem (d = 1, 3 nodes) with take-off
  on, where the headwind changes the roll.
- **Robustness:** a roll with friction above thrust at rest (floored), longer than the straight, stays finite. A
  headwind ≥ V_LOF raises `AnalysisError`.

## Re-optimisation with take-off on

**Done for the best case and the reduced-set RDO** (`studies/takeoff_study.py reopt`; results in
`docs/studies/takeoff_study.md` §9). Actual times: deterministic best case 61 s warm, 266 s cold; RDO 500 s (one
start). The full enumeration was not run. The estimate below, made before those runs, was conservative.

### Estimate (before the runs)

One deterministic multipoint run from the published optimum costs about the same as the published run. Take-off on costs +2 % per model run (0.438 → 0.446 s) and +10 % per `compute_totals`
(2.43 → 2.68 s). The lap component's cs linearisation goes from 0.85 to 0.98 s per point (two more inputs and the roll in every
complex evaluation).
- Best case: about 20 min.
- The full 11-case propulsion enumeration plus the bottle runs: about 4–5 h on 4 workers.
- The reduced-set RDO (3 starts): about 1.5 h.

## Open questions (PLACEHOLDERS Q40–Q45)

- `CL_ground` (0.15, PLACEHOLDER) and `k_lof` (1.1, from the MATLAB comment, not a measurement).
- The take-off throttle.
- `start_offset_ft` (0, PLACEHOLDER: brake release on the start line).
- Climb and transition are not modelled.
- Step resolution of lap 1's first straight near the wind limit (Q45, found in the re-analysis; affects the take-off-off
  model too).

## Status

- **Stage 1** (`7fa5148`): model, course data and presets, verification.
- **Stage 2** (`77ff476`): re-analysis (`docs/studies/takeoff_study.md`), wording fixes, docs and PLACEHOLDERS.
- **Stage 3:** the UI.
  - **Mission panel:** the Mission column (mission time, take-off, `mu_roll`, the Eq. 3.3 distance, reaction time) and
    the course editor.
  - **Course editor:** preset selector, leg table with an order column and "Add leg" / "Sort by order", a different
    lap 1, the sketch, validation, save-as-preset.
  - **After a run:** the take-off metrics.
  - **Tests:** `tests/test_ui.py` covers take-off on, a leg added, Analyse at num_y 11, the take-off time in the
    summary and the course recorded in the outputs, plus a validation case that blocks Run and the shipped-name
    refusal. AppTest cannot edit an `st.data_editor`, so the UI test adds the leg with the "Add leg" button.
