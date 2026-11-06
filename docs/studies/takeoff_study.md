# Take-off in the mission time: re-analysis of the 2024-25 designs

> **Uncalibrated (2026-10-26).** This page was computed on the uncalibrated (G&G) mass model and has not been rerun on the calibrated one. The 2025-26 pages (`docs/studies/results_2026/`, take-off on, calibrated mass model) supersede it for the thesis.

Script: `studies/takeoff_study.py` (`run`, `woombah`, `wind`, `robust`, `resolution`, `reopt`, `report`). Its outputs are in
`outputs/takeoff/`, which git ignores. Config: `configs/dbf2025_multipoint_takeoff.yaml`. It extends
`dbf2025_multipoint.yaml` and switches `course.takeoff.in_mission_time` on. The model and its decisions are in
`docs/development/TAKEOFF_COURSE_PLAN.md` and `docs/modules/mission.md`.

Sections 1–8 are **analysis only**: every design is re-analysed at its published design variables. Section 9
re-optimises the best deterministic case and the reduced-set RDO with take-off on; its runs are new files
(`outputs/results/runs/to_*.json`). No published result, figure or JSON is changed. Take-off stays **off** in every
published result.

## Setup

- **Rule:** the 2024-25 rules start the clock for M1, M2 and M3 "when the airplane throttle is advanced for the first
  take-off (or attempt)" [R p. 11–12].
- **Model, take-off on:**
  - a ground roll from rest to lift-off on lap 1's first straight, into the wind, at the point's throttle;
  - m dv/dt = T(v) − D(v) − μ_roll (W − L(v)), with L and D at `CL_ground` = 0.15 (PLACEHOLDER);
  - lift-off at V_LOF = 1.1 V_stall (`performance_modeller.m` l. 405);
  - the rest of the straight is flown in the air from V_LOF;
  - rotation, transition and climb are not modelled.
- **Designs:** the six of `results_2025.md` §2, plus the bottle optima of §1b. The 1 L optimum is `multipoint` and the
  2 L optimum is `multipoint_2L`, so only 0.5 L is new.
- **Off reproduces the published numbers:** every nominal F in the "F off" column equals `results_2025.md` §2 and §1b,
  and the UDR statistics with take-off off equal §7.

## 1. Re-analysis at nominal inputs

| design | m M2 (kg) | W/S M2 (N/m²) | t_TO M2 (s) | roll M2 (ft) | V_LOF M2 (m/s) | t_TO M3 (s) | roll M3 (ft) | V_LOF M3 (m/s) | Δ3-lap time (s) | ΔM2 | ΔM3 laps | ΔM3 | F off | F on | ΔF |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| woombah | 8.97 | 149 | 2.91 | 79 | 15.82 | 0.78 | 12 | 9.23 | +2.04 | -0.0199 | -0.020 | -0.0007 | 5.1447 | 5.1241 | -0.0207 |
| ac1_direct | 11.55 | 175 | 3.40 | 100 | 17.04 | 0.57 | 8 | 8.56 | +2.51 | -0.0344 | -0.020 | -0.0007 | 5.4295 | 5.3944 | -0.0351 |
| ac1_completed | 11.17 | 169 | 4.59 | 132 | 16.47 | 0.65 | 9 | 8.32 | +3.26 | -0.0360 | -0.018 | -0.0007 | 5.3777 | 5.3411 | -0.0366 |
| multipoint (= 1 L bottle) | 8.16 | 151 | 2.83 | 78 | 15.88 | 0.64 | 10 | 9.13 | +2.03 | -0.0194 | -0.018 | -0.0007 | 5.1394 | 5.1194 | -0.0201 |
| multipoint_2L (= 2 L bottle) | 14.58 | 174 | 5.38 | 158 | 16.88 | 0.66 | 9 | 7.78 | +3.84 | -0.0525 | -0.018 | -0.0007 | 5.6868 | 5.6336 | -0.0531 |
| rdo (rdor_ms0) | 8.15 | 157 | 2.91 | 82 | 16.24 | 0.64 | 10 | 9.32 | +2.12 | -0.0202 | -0.018 | -0.0007 | 5.1386 | 5.1177 | -0.0209 |
| 0.5 L bottle | 4.93 | 158 | 1.95 | 59 | 17.39 | 0.73 | 15 | 12.33 | +1.30 | -0.0069 | -0.021 | -0.0008 | 4.7685 | 4.7609 | -0.0076 |

Notes on the table:
- Δ3-lap time is the M2 3-lap time with take-off on minus with take-off off.
- The 3-lap time grows by less than t_TO. With take-off off, lap 1 starts at V_stall and accelerates through the same
  first metres in the air, so the roll replaces part of that time rather than adding to it.
- ΔM3 laps is the change in M3 laps in the 300 s window.

**The bias is not uniform; it favours heavy designs.** ΔF runs from −0.008 (0.5 L) to −0.053 (2 L), a factor of 7.
- **M2 drives it.** M2 is fuel weight over 3-lap time. The heavy M2 aircraft roll longest (more mass to accelerate,
  higher V_LOF), and their large fuel weight multiplies the time lost. So ΔM2 grows with M2 mass: 0.5 L −0.007,
  1 L −0.019, 2 L −0.053.
- **M3 barely moves.** ΔM3 is −0.0007 everywhere: the M3 aircraft is light, its roll is under 1 s, and it loses
  0.02 of a lap in the 300 s window.
- **Earlier wording was wrong.** "Biases every M2 high alike" (`INDEX.md`, `results_2025.md` caveats) is corrected
  to this.

**Rankings at nominal: unchanged.** With take-off on the order is still:

2 L > ac1_direct > ac1_completed > woombah > multipoint > rdo > 0.5 L

The gaps move:
- the 2 L lead over 1 L shrinks from 0.547 to 0.514;
- woombah − multipoint goes from 0.0053 to 0.0047;
- multipoint − rdo goes from 0.0008 to 0.0017.

### Lap 1 with take-off on (multipoint, M2)

This is the leg breakdown the lap component now reports (`SmoothLapModel.leg_breakdown()`). Lap 1 takes 30.49 s
with take-off on, against 28.46 s with it off.

| leg | kind | size | wind | v in (m/s) | v out (m/s) | t (s) | E (kJ) |
|---:|---|---:|---|---:|---:|---:|---:|
| 1 | ground roll | 78 ft | head | 0.00 | 15.88 | 2.83 | 4.14 |
| 2 | straight | 422 ft | head | 15.88 | 31.55 | 5.18 | 7.24 |
| 3 | turn | 180 ° | – | 31.55 | 29.64 | 2.17 | 2.79 |
| 4 | turn | 360 ° | – | 29.64 | 28.49 | 4.64 | 6.17 |
| 5 | straight | 1000 ft | tail | 28.49 | 38.00 | 8.78 | 10.03 |
| 6 | turn | 180 ° | – | 38.00 | 31.68 | 2.48 | 2.89 |
| 7 | straight | 500 ft | head | 31.68 | 36.64 | 4.41 | 5.08 |

## 2. Feasibility: the published optima no longer complete M2 on their energy

| design | M2 energy margin off (J) | on (J) | M2 laps (energy) off | on | M1 check off | on |
|---|---:|---:|---:|---:|---|---|
| woombah | 107475 | 103905 | 5.743 | 5.652 | fail | fail |
| ac1_direct | 55515 | 49756 | 3.994 | 3.891 | fail | fail |
| ac1_completed | 0 | -4453 | 3.000 | 2.875 | pass | pass |
| multipoint (= 1 L bottle) | -0 | -3146 | 3.000 | 2.906 | fail | fail |
| multipoint_2L (= 2 L bottle) | -0 | -6181 | 3.000 | 2.868 | pass | pass |
| rdo (rdor_ms0) | 2298 | -983 | 3.069 | 2.970 | pass | pass |
| 0.5 L bottle | 0 | -2015 | 3.000 | 2.920 | pass | pass |

- **This is the larger effect of take-off.**
  - Every optimised design flies M2 on exactly its energy budget with take-off off: the energy margin and
    `laps_frac` ≥ 3 constraints are active.
  - Take-off adds 2.0–6.2 kJ net (E_takeoff 2.7–8.4 kJ, less the in-air acceleration it replaces).
  - So with take-off on, every one of them violates M2's energy constraint at nominal inputs. Its energy-limited M2
    laps fall to 2.87–2.97.
  - That includes the RDO design: its calm-air M2 energy margin (2.3 kJ) was smaller than the take-off energy.
- **The reserve is a placeholder.** The energy budget holds back 20 % of usable energy (`energy_reserve`, a
  PLACEHOLDER, PLACEHOLDERS Q24), so these aircraft would finish M2 into the reserve. Against the model's own
  constraints, though, they are infeasible.
- **Re-optimising restores the energy almost for free (section 9).** The re-analysis ΔF was expected to be a lower
  bound on the cost. It is not: the re-optimised best case scores 5.1200, slightly above its infeasible re-analysis
  (5.1194). The optimiser buys the energy back with a bigger M2 pack (1.03 → 1.07 Ah), not a slower M2.
- **The M1 check doesn't change:** every design passes or fails it as before. M1 is far inside its window and energy.

## 3. Robust statistics at the published designs: the RDO's advantage does not survive (until re-optimised, §9)

These are the reduced set's UDR statistics (f_build, f_tb, m_avionics, lap.wind; d = 4, 9 nodes; k and β as in
`configs/dbf2025_multipoint_rdo.yaml`) at each design. The take-off off columns reproduce `results_2025.md` §7. The
Monte Carlo was not rerun.

| design | mu F off | mu F on | sigma off | sigma on | mu − kσ off | mu − kσ on | robust constraints violated, off | on |
|---|---:|---:|---:|---:|---:|---:|---|---|
| woombah | 5.1333 | 5.1174 | 0.0103 | 0.0081 | 5.1231 | 5.1092 | M2.rpm_margin, M2.SM, M3.SM | M2.rpm_margin, M2.SM, M3.SM |
| ac1_direct | 5.4198 | 5.3923 | 0.0102 | 0.0069 | 5.4097 | 5.3854 | M2.rpm_margin, M3.SM | M2.rpm_margin, M3.SM |
| ac1_completed | 5.3635 | 5.3348 | 0.0140 | 0.0103 | 5.3495 | 5.3245 | M2.energy_margin, M2.rpm_margin, M2.laps_frac | M2.energy_margin, M2.rpm_margin, M2.laps_frac |
| multipoint (= 1 L bottle) | 5.1284 | 5.1130 | 0.0097 | 0.0074 | 5.1187 | 5.1056 | M2.energy_margin, M2.rpm_margin, M3.SM, M2.laps_frac | M2.energy_margin, M2.rpm_margin, M3.SM, M2.laps_frac |
| multipoint_2L (= 2 L bottle) | 5.6693 | 5.6279 | 0.0182 | 0.0131 | 5.6512 | 5.6148 | M2.energy_margin, M2.rpm_margin, M2.SM, M2.laps_frac | M2.energy_margin, M2.rpm_margin, M2.SM, M2.laps_frac |
| rdo (rdor_ms0) | 5.1283 | 5.1122 | 0.0094 | 0.0071 | 5.1188 | 5.1051 | none | M2.energy_margin, M2.laps_frac |
| 0.5 L bottle | 4.7624 | 4.7561 | 0.0051 | 0.0043 | 4.7573 | 4.7518 | M2.energy_margin, M2.rpm_margin, M2.SM, M2.wall_ratio, M3.wall_ratio, M2.laps_frac | M2.energy_margin, M2.rpm_margin, M2.SM, M2.wall_ratio, M3.wall_ratio, M2.laps_frac |

**This changes two conclusions of `results_2025.md`** (findings 7–9). Both are stated plainly here; the published
results are not edited.

1. **The robust objective's order flips.** With take-off off, the RDO design is ahead of the deterministic optimum
   by 1e-4 in mu − kσ (5.1188 against 5.1187). With take-off on, the deterministic optimum is ahead by 5e-4
   (5.1056 against 5.1051). Both gaps are small, and the order was a tie in practice.
2. **The RDO design loses its robust feasibility.** With take-off off it met every robust constraint; with take-off
   on it violates two:
   - M2's energy margin: mu − 2σ = −2.1 kJ;
   - M2's laps: mu − 2σ = 2.94 laps, against 3.

   At the published designs, "the RDO design keeps the laps margin for almost no score" (finding 8) holds only in a
   model without take-off. **Section 9 re-runs the RDO with take-off on and restores both:** the re-optimised RDO
   design meets every robust constraint and is again ahead in mu − kσ.

**Wind matters less with take-off on.** σ falls by about 25 % for every design (multipoint 0.0097 → 0.0074). A
headwind now also shortens the ground roll: the roll starts at airspeed w and lifts off at the same airspeed V_LOF,
covering less ground. That partly offsets the time the wind costs on the head-wind straights.

## 4. Woombah against its flown M2 time

Table 39 gives two readings of Woombah's M2 time [W p. 59]:
- **138.1 s** in total;
- **112.2 s** for 3 × the 37.4 s average lap.

The 25.9 s gap is one of the inconsistencies `docs/development/RESULTS_PLAN.md` §6 lists. Reading (a) is that 138.1 s is the scored
time with take-off included.

Here are the three cases of `validation_2025.md` §3, with take-off off and on:

| case | M2 mass (kg) | throttle M2 | t_TO (s) | roll (ft) | 3-lap time off (s) | on (s) | Δ (s) | M3 laps off | on |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| full throttle | 9.02 | 1.000 | 1.58 | 42 | 75.8 | 76.9 | +1.1 | 2.40 | 2.39 |
| logged current | 8.97 | 0.678 | 2.91 | 79 | 81.3 | 83.3 | +2.0 | 9.99 | 9.97 |
| logged current and mass | 12.20 | 0.678 | 4.83 | 155 | 88.3 | 91.8 | +3.5 | 9.58 | 9.52 |

The throttles and ballast are read from `validation_2025.json`, not re-solved.

- **The ground roll explains 1–3.5 s of the 25.9 s gap,** in the closest case (logged current and mass, 91.8 s). That
  is under 15 %.
- **The model's ground roll is not too short for Woombah.**
  - Woombah takes off at full throttle with flaps (§4.7.1), and its own take-off budget is "a 5-second take-off at
    99 % throttle" (§4.7.3).
  - The model's roll at the logged throttle and mass is 4.8 s, at full throttle 1.6 s.
- **So take-off cannot reconcile 112.2 s with 138.1 s.** If 138.1 s includes take-off, the other ~22 s is something
  the lap model leaves out:
  - the climb and the turn back onto the course;
  - the pilot's line;
  - or the landing, if the logged flight time runs to touchdown.

  The report does not say which. RESULTS_PLAN §6 reading (a) is therefore not supported by take-off alone.
- **The model is still fast.** With take-off on, its best case (91.8 s) is 20 s below the 112.2 s reading.
  `validation_2025.md` §3 lists the other missing terms (as-built drag, part-throttle propulsion, the pilot's line).

## 5. Wind limit

The wind limit is the largest steady wind (the robustness input `lap.wind`: headwind on the head straights) for which
the lap is still valid, with no `AnalysisError`. It was found by stepping the wind in 0.01 m/s on the lap component
at each design's converged nominal point (wind enters only the lap model). Units are m/s.

| design | M2 V_stall | M2 limit off | M2 V_LOF | M2 limit on | M3 V_stall | M3 limit off | M3 V_LOF | M3 limit on |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| woombah | 14.38 | 14.19 | 15.82 | 15.62 | 8.39 | 7.86 | 9.23 | 9.19 |
| ac1_direct | 15.49 | 15.48 | 17.04 | 17.03 | 7.78 | 7.73 | 8.56 | 7.92 |
| ac1_completed | 14.98 | 14.97 | 16.47 | 16.46 | 7.56 | 6.98 | 8.32 | 8.21 |
| multipoint (= 1 L bottle) | 14.44 | 14.43 | 15.88 | 15.66 | 8.30 | 8.26 | 9.13 | 8.52 |
| multipoint_2L (= 2 L bottle) | 15.34 | 15.34 | 16.88 | 16.76 | 7.07 | 7.04 | 7.78 | 7.74 |
| rdo (rdor_ms0) | 14.76 | 14.55 | 16.24 | 16.02 | 8.47 | 7.87 | 9.32 | 9.27 |
| 0.5 L bottle | 15.81 | 15.47 | 17.39 | 17.05 | 11.21 | 11.15 | 12.33 | 12.27 |

- **The limit moves from about V_stall up to just under V_LOF.**
  - With take-off off, lap 1 starts in the air at V_stall, so the ground speed after the start, V_stall − w, goes to
    zero as w → V_stall.
  - With take-off on, the in-air part starts at V_LOF = 1.1 V_stall, and a headwind at or above V_LOF raises
    `AnalysisError` before the roll.
  - M3 sets the design's limit everywhere. It rises by 0.3–1.4 m/s.
  - Every design's limit is now above the Monte Carlo's 7 m/s cap; with take-off off, ac1_completed's was not.
- **Likely effect on the Monte Carlo:** the "wind limit (no flight)" counts in `results_2025.md` §3 (ac1_completed 14,
  multipoint_2L 37, ac1_direct 4 of 1024) would probably fall with take-off on. That is not established without a
  rerun: the Monte Carlo also varies the mass, which moves V_stall.
- **The limit is a validity edge, not a physical one, and the lap time is inaccurate before it.**
  - The first straight is integrated in ground distance with a fixed 40-step RK4 (dt/ds = 1/(v − w)). As the ground
    speed just after the start becomes small, the step error grows.
  - Section 6 measures it. For ac1_direct and multipoint at M3 the edge sits 0.6 m/s below V_LOF because of this step
    error (the first straight's exit speed jumps from 36 to 54 m/s between w = 8.4 and 8.5 m/s for multipoint).
- **The edge is not even monotonic.** ac1_completed with take-off off is invalid at 7.0 m/s but valid at 7.5
  (section 6), so the scan reports the first edge from calm air upward.

## 6. Step resolution near the wind limit (found here; affects the published Monte Carlo)

M3's lap-1 time at the default `n_steps_straight` = 40, against 640 steps (`resolution`; relative error of the 40-step
value):

| design | take-off | w = 3.5 | w = 5 | w = 6 | w = 7 | w = 7.5 | w = 8 | w = 8.4 |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| multipoint | off | +0.01 % | +0.04 % | +0.12 % | +0.52 % | +2.42 % | -0.25 % | invalid |
| multipoint | on | +0.01 % | +0.02 % | +0.04 % | +0.13 % | +0.27 % | +0.72 % | +3.88 % |
| multipoint_2L | off | +0.03 % | +0.15 % | +0.80 % | +16.49 % | invalid | invalid | invalid |
| multipoint_2L | on | +0.01 % | +0.06 % | +0.21 % | +2.04 % | -0.13 % | invalid | invalid |
| ac1_completed | off | +0.02 % | +0.09 % | +0.32 % | invalid | +18.78 % | invalid | invalid |
| ac1_completed | on | +0.01 % | +0.04 % | +0.11 % | +0.46 % | +1.96 % | -0.27 % | invalid |

'invalid': a negative or non-finite lap time at 40 or 640 steps (past the wind limit).

- **Take-off off, which is how every published result was computed:**
  - the error stays under 0.15 % up to 5 m/s for all three designs;
  - it grows quickly near the wind limit; for the heavy designs (multipoint_2L, ac1_completed) it is already large
    inside the Monte Carlo's 0–7 m/s wind range.
  - The published Monte Carlo statistics for those two designs therefore carry a discretisation error in their
    highest-wind samples.
  - Its size is bounded by the share of samples near 7 m/s and by M3's weak dependence on lap 1 (0.04 of a lap per
    second of lap 1). The mean and p5 are probably affected in the third decimal at most, but this was not rerun.
- **Take-off on:** the error is smaller at the same wind, because the in-air part starts 10 % faster.
- **Fix, recorded in PLACEHOLDERS and not made:** more steps, or a distance grid clustered at the start of lap 1's
  first straight. That would change the take-off-off numbers, so it would mean new reference files; it was left out of
  this study.

## 7. Eq. 3.3 against the integrated ground roll

See `docs/development/TAKEOFF_COURSE_PLAN.md` for the table.
- **Eq. 3.3's roll** (V_R = 1.2 V_stall, drag at C_L,max and T(0) throughout) is 10 % to 1.9× longer than the
  integrated roll.
- **The constraint stays inactive:** every roll is far inside the 100 m take-off distance. `thrust_margin_static`
  stays the take-off constraint, unchanged.

## 8. Sensitivities and assumptions

- **`CL_ground`** (PLACEHOLDER, 0.15): going from 0 to 1 changes the M2 roll by ≤ 3 ft, or 19 ft for multipoint_2L.
  That is ≤ 0.0006 in F.
- **`k_lof`** = 1.1, from the comment in `performance_modeller.m`. It was not varied: it moves V_LOF² and the roll in
  proportion.
- **Take-off throttle:** re-analysed at throttle 1.0 as an estimate. The M2 roll would be 30–45 % shorter, so ΔF would
  shrink by roughly that share. The energy finding of section 2 would get worse: at full throttle E_takeoff
  rises by 4–38 %. A separate take-off throttle is not built (`docs/development/TAKEOFF_COURSE_PLAN.md`).
- **Not modelled:** rotation, transition, climb, and a start offset behind the line (default 0).
- **Placeholders:** PLACEHOLDERS Q40–Q44 cover `CL_ground`, `k_lof`, the take-off throttle, the start offset and
  climb. Q45 covers the step resolution.

## 9. Re-optimisation with take-off on

`studies/takeoff_study.py reopt --which best_warm|best_cold|rdo`, take-off on (`CL_ground` 0.15, `k_lof` 1.1, as in
sections 1–3). Only the best deterministic case (`mp_6S_11x8E_10x7E`) and its reduced-set RDO were re-optimised.
The other propulsion cases and the bottle runs were not.

- **Deterministic:** `configs/dbf2025_multipoint_takeoff.yaml`, SLSQP from the published optimum (warm, 14
  iterations, 61 s) and from the config's initial point (cold, 53 iterations, 266 s).
- **RDO:** `configs/dbf2025_multipoint_rdo_takeoff.yaml`, which is `dbf2025_multipoint_rdo.yaml` with the same take-off
  block. It uses the same four inputs and 9 UDR nodes, and k = 1, β = 2. One start, from the published RDO design
  `rdor_ms0` (13 iterations, 500 s).

### Deterministic best case

| | F | M2 | M3 | M2 energy constraint | M2 pack (Ah) | chord (m) | M2 mass (kg) |
|---|---:|---:|---:|---|---:|---:|---:|
| published, take-off off | 5.1394 | 1.7981 | 2.3413 | active | 1.026 | 0.289 | 8.159 |
| published design, take-off on (§1) | 5.1194 | | | violated | 1.026 | 0.289 | 8.159 |
| re-optimised, take-off on | 5.1200 | 1.7798 | 2.3401 | active | 1.068 | 0.306 | 8.189 |

- **The warm and cold starts reach the same F** (5.1200), the same airframe and the same active set. Kv (728 against
  752 rpm/V) and the throttles (0.825 against 0.798 in M2) differ, so the optimum is flat in those directions.
- **The active set is the published one:**
  - M2 energy, M2 rpm, M2 laps and the M3 rules energy;
  - both points' tail volumes;
  - span, tail arm and fuel at their upper bounds.

  The cruise speeds barely move (M2 39.40 → 39.31 m/s, M3 34.97 → 34.75 m/s).
- **The cost of take-off at the optimum is ΔF = −0.0194.** The re-analysis estimate (−0.0201) was within 0.0007.

### Robust design

Both re-optimised designs are evaluated with take-off on, at the same 9 nodes:

| design | mu F | sigma | mu − kσ | M2 energy, mu − 2σ (J) | M2 laps, mu − 2σ | robust constraints met |
|---|---:|---:|---:|---:|---:|---|
| re-optimised deterministic | 5.1127 | 0.0077 | 5.1050 | −1453 | 2.96 | no (M2 energy, M2 laps) |
| re-optimised RDO | 5.1127 | 0.0074 | 5.1053 | +8 | 3.00 | yes |

- **The published conclusion holds again.** The RDO design is ahead by 3e-4 in mu − kσ (1e-4 with take-off off), and
  it meets the two robust constraints that the deterministic design fails. Finding 8 of `results_2025.md` ("keeps
  the laps margin for almost no score") holds with take-off on.
- **The flip in section 3 was a stale-optimum effect.** It came from evaluating designs optimised without take-off.
- **How the RDO design gets its margin:** the same higher throttle and lower Kv as the published RDO (M2 throttle
  0.949, Kv 632). Its M2 pack also grows (1.04 → 1.07 Ah).

### What this does not settle

- **The ranking of propulsion cases.** The top four published cases are within 0.0009 of each other in F, and take-off
  costs about 0.02. Re-optimising only the best case cannot show that it is still the best. That needs the full
  enumeration with take-off on, which was not run.
- **The RDO's global optimum.** The RDO had one start; the published RDO used a multistart.
- **High-wind accuracy.** Q45, the step resolution, still applies to the high-wind nodes.

## Findings

1. **Take-off costs every design score, unevenly.** ΔF runs from −0.008 to −0.053 and grows with M2 mass, so leaving
   take-off out favours heavy designs. Nearly all of it is in M2; ΔM3 is −0.0007 everywhere.
2. **No ranking changes at nominal inputs.**
3. **Every published optimum violates M2's energy constraint with take-off on.** Its energy-limited M2 laps are
   2.87–2.97. Re-optimising the best case restores it with a bigger M2 pack, at F 5.1200 (ΔF −0.0194 against the
   published 5.1394, §9), so the re-analysis ΔF is a good estimate of the cost.
4. **The RDO conclusion survives re-optimisation.** At the published designs, the RDO design violates two robust
   constraints with take-off on and falls 5e-4 behind (§3). Re-optimised with take-off on (§9), it meets every robust
   constraint and is ahead by 3e-4 in mu − kσ, as without take-off. The ranking of propulsion cases with take-off on
   is not established: only the best case was re-optimised.
5. **Woombah:** take-off explains 1–3.5 s of the 25.9 s gap between its two M2 time readings. It does not reconcile
   them.
6. **Wind limit:** each design's limit rises from about V_stall to just under V_LOF (M3: +0.3 to +1.4 m/s), and all are
   now above the 7 m/s cap. Separately, the fixed-step lap-1 integration is inaccurate within about 1–2 m/s of the
   limit, and that already affects the published Monte Carlo's highest-wind samples for the heavy designs (Q45).
