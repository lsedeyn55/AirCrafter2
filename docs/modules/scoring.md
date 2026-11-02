# Scoring (`aircrafter2/scoring/`)

The objective lives in a scoring plug-in, not in the disciplines (`docs/how_to_add_a_scoring_plugin.md`). The score
component always exists (also in analysis runs), and is the objective only when optimising.

A plug-in's `constraints()` (like any new constraint) needs an entry in the constraints register,
`aircrafter2/constraints_view.REGISTER`: the UI's Constraints tab refuses a driver constraint it cannot describe.

| Plug-in | Use | Objective |
|---|---|---|
| `three_lap_time` | placeholder, single-point problems (`full_mdo`) | minimise the 3-lap time |
| `laps_in_window` | placeholder | maximise laps in the window |
| `dbf2025` | the 2024-25 ruleset, multipoint problem (`problems/multipoint.py`) | maximise M1 + M2 + M3 |

## `dbf2025`: AIAA DBF 2024-25, X-1 Supersonic Flight Program

### Verbatim (Rules_2025.pdf)

| Page | Rule |
|---|---|
| p. 11 | **M1** = 1.0 for successful mission. (3 laps in the 5-minute window, no payload; pylons not installed, p. 10) |
| p. 11 | **M2** = 1 + [N_(fuel weight / time) / Max_(fuel weight / time)], where Max_(fuel weight / time) is the highest (fuel weight / time) score of all teams. Time: to fly 3 laps, starting "when the airplane throttle is advanced for the first take-off". Payload: the X-1 and the fuel tanks. |
| p. 12 | **M3** = 2 + [N_(# laps flown + (bonus box score / X-1 test vehicle weight)) / Max_(# laps flown + (bonus box score / X-1 test vehicle weight))]. Laps flown prior to the X-1 launch. The launch lap must be completed in the window; the X-1 must be launched for a score. |
| p. 12 | **GM** = [Min_(mission time) / N_(mission time)] |
| p. 9 (Fig. 1) | Bonus box scores: 2.5 (the 200 × 200 ft box at the start line), 1.0 (the boxes either side), 0 outside |
| p. 26 | Weights in lb, times in s, both rounded to 2 decimals |
| p. 27 | Total Mission Score = M1 + M2 + M3 + GM; Competition Score = Total Report Score × Total Mission Score + P |

### Model form

    F  = M1 + M2 + M3                                     (the flight score; maximised)
    M1 = 1                                                (the design must pass the M1 check, below)
    M2 = 1 + (W_fuel / t_3) / R2                          W_fuel in lb (the model's kg / 0.45359237), t_3 in s
    M3 = 2 + (n_pre + B / W_X1) / R3                      n_pre = softplus(laps_frac − 1), width 0.05 lap

- **GM is left out** of the objective: it measures the crew's assembly speed, not the aircraft.
- **t_3** is the lap model's `ThreeLapTime` (lap 1 from the stall speed plus two laps). **Take-off is optional** (`course.takeoff.in_mission_time`, docs/modules/mission.md) and **off in every published result**, while the rule counts it, so the published t_3 is biased short. The bias is not uniform: it favours heavy M2 designs (ΔF −0.008 to −0.053 across the published designs; docs/studies/takeoff_study.md). Stated in every result table.
- **n_pre** is the laps completed in the window minus the launch lap, from the continuous `laps_frac` surrogate. The softplus floor makes it smooth at laps_frac = 1; it equals laps_frac − 1 to within 1e-9 for laps_frac > 1.5 (tested against the verbatim form). The X-1's mass is carried for the whole M3 flight (it is dropped for the last lap): ignored, a conservative 0.37 lb of ≈ 10 lb.
- **B / W_X1** is a scenario constant, because the X-1's own flight is not modelled. B = 0 in the main results; B = 2.5 at W_X1 = 0.37 lb (Woombah's X-1 as flown, Design Report Table 39) is a second scenario. B shifts M3 by a constant, so the optimum does not move.
- **R2, R3** (the best teams' raw values) are PLACEHOLDERS in the config (OPEN_QUESTIONS Q35): R2 = 0.20 lb/s, R3 = 27. Evidence: `docs/RESULTS_PLAN.md` §4 (the official 2025 results give only normalised scores: best M2 2.00, University at Buffalo; best M3 3.00, RPI; USYD 1.58 and 2.28).
- **Bookends:** options `w2`, `w3` weight the M2 and M3 terms of `score`; `flight_score` is always M1 + M2 + M3.
- **Outputs:** `score` (the objective), `M1`, `M2`, `M3`, `flight_score`, `n_pre`. Partials by complex step.

### M1 check

`problems.multipoint.m1_check` analyses the final design without payload or pylons, with M3's prop, battery and throttle, on the airframe sized at M2. It passes when laps_frac ≥ 3 in the 300 s window and the flight-safety constraints hold (stall margin, static margin, take-off thrust, spar). Model-validity limits (`v_fit_margin`: the light M1 aircraft can outrun the T(v) fit at M3's throttle) are reported, not failed on.

### Hand-worked Woombah case (`tests/test_results_2025.py`)

Woombah flew 13.2 lb in M2 and 7 laps (6 before the launch) with a 0.37 lb X-1 (Table 39; pre-competition testing). The official USYD scores (M2 1.58, M3 2.28) are reproduced by:
- R2 = (13.2 / t) / 0.58: 0.203 lb/s for t = 112.2 s (3 × 37.4 s), or 0.165 lb/s for t = 138.1 s (the flight time);
- R3 = (6 + B / 0.37) / 0.28: 21.4 for B = 0, or 45.6 for B = 2.5.

### In the 2024-25 results (`docs/studies/results/results_2025.md`)

- **Normalisation (§5):** the R2/R3 grid (×0.5, ×1, ×2) changes the totals, but the design barely moves. Equal
  scalings cannot move it at all: they scale the whole variable part of F. M2's lever is the fuel weight, which is
  capped by the tanks, and M3's laps are capped by the 100 Wh pack rule. So the normalisation re-weights the score
  without changing the aircraft. R2 and R3 stay PLACEHOLDERS (Q35).
- **Bonus (§6):** B = 2.5 adds the same constant to every design, as expected.
- **M1 check:** it fails at the best propulsion case, on the static margin (just above its band without payload).
  `v_fit_margin` is also listed, but it does not fail the check. The page flags this; M1 is still scored 1.
- **Bookends (§4):** the `w2`/`w3` options give the M2-only and M3-only optima. The multipoint optimum is close to
  the M2-only one.

## `dbf2026`: AIAA DBF 2025-26, Banner Towing Bush Plane

F = M1 + M2 + M3 with M2 = 1 + cap(NI / NI_max) and M3 = 2 + cap(T3 / B_max), T3 = laps × banner length / RAC
([R26] pp. 19–20; `aircrafter2/scoring/dbf2026.py`, docs/RESULTS_2026_PLAN.md §4.1). NI_max and B_max are the best
teams' values (PLACEHOLDERS, Q52).

- **The cap (2026-11-02).** The rules' Max is the best team's, so N / Max is at most 1: F ≤ 6. The model caps each ratio
  smoothly. cap(r) = r exactly for r ≤ 1 − δ (every score below the cap is unchanged bit for bit), 1 exactly for
  r ≥ 1 + δ, and the C1 quadratic blend r − (r − 1 + δ)²/(4δ) between. Its bias at r = 1 is δ/4; δ = `cap_delta` =
  0.005. It branches on the real part, so the complex-step partials hold. A KS max isn't used: it would shift every
  score near the cap, including Humpback's.
- **`floored()`** (report only) records the score as the judges would: completed laps, the banner rounded down to
  1/4 in, the span to the inch, the counts rounded, and the hard cap min(r, 1).
- **Outputs:** the plug-in also gives the uncapped ratios `r_M2`, `r_M3`. The run JSON (`studies/results_2026.py`)
  records `cap` = {δ, r_M2, r_M3, M2_capped, M3_capped}.
- **At the final-fixes optimum** (`fx_smoke_int_72p24c`), r_M2 = 1.0028, inside the blend (M2 1.9998), and
  r_M3 = 0.953. Uncapped, the payload credit had driven the design to the fuselage-length cap (97 passengers,
  F 6.17).

### Verbatim (`2026-DBF-Rules.pdf`, printed page "N of 34" = PDF page)

    p. 10  RAC = 0.05 * WS + 0.75 (WS = wingspan in ft, measured to the nearest inch); minimum RAC = 0.90 (3 ft span)
    p. 19  M1 = 1.0 for successful mission.
    p. 19  Net Income = Income - Cost
           Income = (#passengers * (Ip1 + (Ip2 * #laps))) + (#cargo * (Ic1 + (Ic2 * #laps)))
           Ip1 6, Ip2 2, Ic1 10, Ic2 8
    p. 20  Cost = #laps * (Ce + (#passengers * Cp) + (#cargo * Cc)) * EF
           EF = Efficiency Factor = Total Propulsion Battery Capacity (W-hrs) / 100;  Ce 10, Cp 0.5, Cc 2
    p. 20  M2 = 1 + [N_Net_Income / Max_Net_Income]
    p. 20  M3 = 2 + [N_(#laps X banner length / RAC) / Max_(#laps X banner length / RAC)]
    p. 21  GM = [Min_(mission time) / N_(mission time)]
    p. 32  Total Mission Score = M1 + M2 + M3 + GM; times to 0.01 s, lengths to 0.01 in, weights to 0.01 lb/oz
           (the banner length is recorded rounded down to 1/4 in, p. 12)

### Model

    objective  F = M1 + M2 + M3 (the flight score; GM isn't modelled: it measures the crew, and its time grows with
               the payload count)
    M1 = 1     the design is required to pass the M1 check (analysis only)
    N2, N3     laps in the 5-min window of the M2 and M3 points: the lap model's continuous laps_frac
    EF         E_M2 / 100, E_M2 = the M2 pack's rated energy (capacity x rated voltage, Wh): the pack is a per-mission
               choice (p. 24, p. 33), so the M2 pack size trades laps against EF
    n_p, n_c   passengers and cargo flown (continuous design variables; integers in the final design)
    L_banner   banner length in inches (design variable); RAC from the span design variable b
    NI_max, B_max  the best teams' Net_Income and laps x length / RAC: PLACEHOLDERS (OPEN_QUESTIONS Q52)
    weights    (w2, w3) multiply the M2 and M3 terms, for the single-mission bookends (default 1, 1)

`floored()` gives the same parts with the laps floored to completed laps and the banner and span recorded as the rules record them,
reported next to the continuous score. It isn't differentiable.
