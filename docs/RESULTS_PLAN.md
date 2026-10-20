# Thesis results on the 2024-25 ruleset: plan (checkpoint 1)

Brief: `THESIS/4_PROMPTS/aircrafter2_results_prompt.md`. Branch `results` from `kv-dv` at `b2adbb8`.

**Sources and citation keys:**
- **[R]** `Rules_2025.pdf` (27 pp.). PDF page = printed page.
- **[W]** `2025 USYD DBF Design Report.pdf` (60 pp., "Page x of 60" = PDF page).
- **[T]** `2025 top reports.pdf`:
  - pp. 1–3: the official 2025 results;
  - UNSW pp. 4–64;
  - UCLA pp. 65–125;
  - ERAU Daytona Beach pp. 126–183.
- **[J]** Josh Kallarackel's thesis (`THESIS/AirCrafter/`).
- **[TM]** `25 competition/T-Motor V3115/` (spec table and test report transcribed from T-Motor's product page; README there).

## 1. Prerequisites

| Check | Result |
|---|---|
| `kv-dv` finished | `b2adbb8`, 254 tests pass. Branch `results` created from it. |
| MATLAB | `/usr/local/bin/matlab`, **R2026a Update 4 (26.1.0.3312084)**, headless `matlab -batch` works. Aerospace Toolbox (`atmosisa`) present. |
| Sources | All three PDFs are present, plus the V3115 data folder (not named in the brief; it supplies Rm and I0, §5). |

## 2. The rules as you read them: verified

**Verdicts:** ✓ = confirmed. ✗ / + = correction or addition.

- **Scoring:**
  - **M1** = 1.0 for a successful mission (3 laps in the 5-min window, no payload) [R p. 11] ✓. **Pylons not installed** for M1 [R p. 10] ✓.
  - **M2** = 1 + [N_(fuel weight / time) / Max_(fuel weight / time)] [R p. 11] ✓. "Time" is the time to fly 3 laps, starting "when the airplane throttle is advanced for the first take-off" [R p. 11]: **take-off is included**.
  - **M3** = 2 + [N_(# laps flown + bonus box score / X-1 weight) / Max_(same)] [R p. 12] ✓. "Laps flown prior to launch" [R p. 12]. The release comes after crossing the start line, after lap 1 or later, before the upwind turn; the release lap must then be completed in the window; the X-1 must be launched for the mission to score [R p. 11–12] ✓.
  - **GM** = Min_(time) / N_(time) [R p. 12] ✓.
  - **Totals:** Total Mission Score = M1 + M2 + M3 + GM [R p. 27] ✓. Competition score = Total Report Score × Total Mission Score + P [R p. 27] ✓. + Total Report Score = 0.15 Proposal + 0.85 Design Report [R p. 27].
- **Fuel tanks:**
  - Commercial non-glass beverage bottle with a screw cap, unmodified, ≥ 16 fl oz [R p. 8] ✓.
  - All external tanks the same size, shape and brand **for all missions** [R p. 8] ✓.
  - At most one internal tank; optional; any bottle, no minimum capacity [R p. 8] ✓.
  - Filled with inert material (water, sand, etc.) to any weight [R p. 8] ✓. + Weights are rounded to 0.01 lb for scoring [R p. 26, Table 1].
- **External tanks and pylons:** at least two external tanks in M2 and M3; M3 tanks may be empty [R p. 9, p. 11] ✓. Pylons removable [R p. 8] ✓. The maximum number of pylons declared at tech inspection must be fitted for every M2/M3 attempt [R p. 10] ✓.
  - **+** At tech inspection the team declares the maximum number of tanks and the maximum total fuel weight; no attempt may exceed them [R p. 16].
- **M2 payload:** **✗ The X-1 is carried in M2 too:** "The payload for Mission 2 is the X-1 test vehicle and fuel tanks" [R p. 8, p. 11]. Your reading lists only the tanks.
- **X-1:**
  - ≤ 0.55 lb [R p. 9] ✓. Under the fuselage between the left and right external tanks [R p. 9] ✓. Released at 200–400 ft AGL [R p. 8, p. 11] ✓.
  - **+** Its weight may differ per attempt, and several X-1s may be brought [R p. 9].
  - **+** It must have a 0.25 in gap to the airframe [R p. 9].
  - Bonus boxes: 0 (outside), 1.0 or 2.5 points (rules Figure 1, p. 9; the 2.5 box is the 200 ft × 200 ft box at the start line) ✓, as Woombah says [W p. 7].
- **Missions:** flown in order [R p. 9–10] ✓. 5-min windows; the landing is not part of the window [R p. 11] ✓. Same configuration for all missions unless stated [R p. 10] ✓. No take-off distance limit [R p. 10] ✓. Prop may change per flight attempt [R p. 14] ✓.
- **Batteries:**
  - May not be changed or charged during an attempt [R p. 15] ✓.
  - Different (inspected) packs between attempts: **✓ but the page is p. 16**, the note on additional battery inspections "due to … real time power change requirements".
  - Within one attempt the packs must be identical (same part number, etc.) and **of one battery type for the whole competition** [R p. 14].
- **Airframe and power:**
  - Span ≤ 6 ft [R p. 8] ✓. TOGW < 55 lb [R p. 14] ✓.
  - One pack per propulsion system; identical packs; ≤ 100 Wh per pack; total propulsion energy ≤ 100 Wh [R p. 14] ✓.
  - Fuse ≤ 100 A; for lithium, ≤ capacity × C, up to 100 A [R p. 15, repeated p. 18] ✓.
- **Structure:** wing-tip lift at the maximum payload and battery combination, "roughly" 2.5 g [R p. 17] ✓. + The tip test is also stated on p. 9 (maximum designed take-off weight), and the maximum load demonstrated cannot be increased later [R p. 9].
- **Course:** markers 500 ft each side of the start line [R p. 18] ✓. Figure 1 [R p. 3]: two 180° turns and one 360° turn on the back straight. `smooth_lap.py` flies 500 ft, 180°, 360°, 1000 ft, 180°, 500 ft: the same distances and turns, but the 360° comes straight after the upwind turn rather than mid back-straight. That only changes the speed history (noted, not changed; MATLAB has the same order).

## 3. Scoring equations (`dbf2025` plug-in, `docs/modules/scoring.md`)

Verbatim [R p. 11–12, 27]:

    M1 = 1.0 for successful mission.
    M2 = 1 + [N_(fuel weight / time) / Max_(fuel weight / time)]
    M3 = 2 + [N_(# laps flown + (bonus box score / X-1 test vehicle weight)) / Max_(# laps flown + (bonus box score / X-1 test vehicle weight))]
    Total Mission Score = M1 + M2 + M3 + GM

**Model form:**
- **Objective:** F = M1 + M2 + M3. M1 = 1 if the M1 check passes (§9, analysis only). GM is left out: it measures the crew, not the aircraft.
- **M2:** M2 = 1 + (W_fuel[lb] / t_3laps[s]) / R2.
  - t_3laps = the lap model's `ThreeLapTime`. This has no take-off (§9): it is biased fast, and the bias is stated in every table.
  - W_fuel is a design variable.
- **M3:** M3 = 2 + (n_pre + B / W_X1[lb]) / R3.
  - n_pre = laps_frac − 1: laps completed in the window minus the release lap, using the continuous `laps_frac` surrogate.
  - B = 0 is the main scenario; B = 2.5 is a second scenario at W_X1 = 0.37 lb [W p. 59, Table 39].
  - The X-1's mass is dropped for the last lap, which **is ignored** (0.37 lb of ≈ 10 lb; conservative). Stated.
- **Smoothness:** both are smooth already: a ratio and a linear term. The only kink is n_pre at laps_frac < 1, below where any feasible design operates. A smooth floor, softplus(laps_frac − 1), is applied and tested against the verbatim form away from it.
- **Tests:**
  - hand-worked Woombah case: Table 39 actual, 13.2 lb in each of the readings of the M2 time (§6), and 7 laps → 6 pre-launch;
  - plus the official USYD normalised scores (M2 1.58, M3 2.28 [T p. 2]) back-solved against R2 and R3.

## 4. Normalisation placeholders (R2, R3): evidence

| Source | R2 = max fuel / time (lb/s) | R3 = max laps + B/W | Note |
|---|---:|---:|---|
| Official results [T p. 2] | (University at Buffalo, M2 = 2.00) | (RPI, M3 = 3.00) | normalised only; raw values not published |
| USYD [T p. 2] + Woombah test numbers [W p. 59] | 0.165–0.203 | 21 (B = 0), 31 (B = 1), 46 (B = 2.5) | USYD M2 = 1.58 ⇒ ratio 0.58, with 13.2 lb in 138.1 s or 112.2 s; M3 = 2.28 ⇒ 0.28 with 6 pre-launch laps. Table 39 is pre-competition testing [W p. 59; report due 21 Feb, fly-off 10–13 Apr, R p. 5–6], so this is approximate |
| Josh, back-calculated from the winner [J p. 80, Table 5.16] | ≈ 0.145 | ≈ 26 | winner raw 0.1 and 19 at ≈ 0.7 normalised; method not fully stated |
| UCLA's assumption [T p. 89] | 0.37 | 20 | their sizing script's maximum |
| ERAU's assumption [T p. 182, Table 8-4] | 0.30 (18 lb/min) | 37 (12 laps + 2.5/0.1) | |
| Shipped AirCrafter 1.0 (`AirCrafter.m` l. 241–243) | 0.2205 (10 kg / 100 s) | 20.5 | |

**Proposed PLACEHOLDERS** (config entries, OPEN_QUESTIONS Q35):
- **R2 = 0.20 lb/s:** the USYD back-calculation brackets 0.165–0.203, and it is consistent with the shipped 0.22. UCLA and ERAU guessed high; Josh is low.
- **R3 = 27:** consistent with the winner-level back-calculation [J] and with a top team flying ~8–10 laps plus a 2.5-point box at ~0.15 lb (ERAU's X-1 was 0.16 lb, [T p. 182]).

The ×0.5 / ×2 grid (0.10–0.40 lb/s, 13.5–54) covers every row above. If you have the real raw values for Buffalo and RPI, they replace these.

## 5. Woombah as built (`configs/woombah_2025.yaml`)

| Item | Value | Source |
|---|---|---|
| Span, area, AR, taper, incidence | 70.9 in (1.801 m), 914.5 in² (0.590 m²), 5.5, λ = 0.8 (root 14.33 in, tip 11.8 in), 2° | [W p. 21 Table 19; p. 31 Table 30] |
| Rectangular equivalent | c = S/b = 12.90 in (0.3276 m). **Taper flagged** | derived |
| Airfoil | Clark Y (`clarky` seed loads) | [W p. 20; Table 18 p. 19] |
| Spar | front 0.87/0.79 in, rear 0.79/0.71 in UD CF tubes. Equal-EI single tube: OD 0.87 in, ID 0.71 in (I = 0.0157 in⁴, the two tubes' sum) | [W p. 20–21], derived |
| Tails | HT: NACA0012, 236.4 in² (Table 30) or 1.66 ft² (Table 21), AR 4, arm 3.61 ft (1.100 m). VT: 103.85 in² (0.72 ft²), AR 1.38 (Table 30) **vs 1.1 (Table 21)**. V_H 0.722, V_V 0.059 | [W p. 21 Tables 20–21; p. 31 Table 30] |
| Fuselage | 82 in long, 4.72 × 4.72 in. Woombah has a full fuselage, the model a cargo box + tail boom: **modelling difference** | [W p. 31 Table 30] |
| Component masses | wing 3.41 lb, nose 0.15, fuselage 1.23, gear 0.66, HT 0.41, VT 0.19, ESC 0.56 (×2), Rx 0.19, Rx battery 0.30, motors 0.50 (×2), props 0.08 (×2), payload structure 0.44, bottles (M2) 13.5, glider structure 0.22, glider 0.33, batteries 1.64 (×2) | [W p. 45 Table 34] |
| Mission masses | M1 9.32, M2 26.02, M3 10.09, M3-empty 9.76 lb | [W p. 45 Table 34] |
| Powertrain | 2 × T-Motor V3115 1050 KV; Hargrave microDrive LP ESC (60 A continuous, 120 A burst); Dualsky 2250 mAh 6S 70C (49.95 Wh) per motor; APC 10x7E | [W p. 22–23, Table 23; p. 56] |
| Motor Rm, I0, mass | 33.84 mΩ; 1.79 A at 10 V; 112.7 g | [TM spec table]. **Not in the report.** You supplied them in the T-Motor folder, so no PLACEHOLDER is needed |
| Motor station | 9.9 in from centreline, 5.5 in ahead of the LE | [W p. 20], already in `full_mdo_twin.yaml` |
| Fill | steel powder, measured packed density 6.5 lb/L; 1 L Pepsi bottles, 6.5 lb each | [W p. 14, p. 18] |
| Pylon | 8.1 × 1.0 × 1.82 in | [W p. 18 Table 16] |
| X-1 | 0.37 lb actual (0.33 design); 15.5 in span, 82 in² wing, 10.62 × 2.36 × 2.75 in fuselage | [W p. 59 Tables 38–39; p. 24 Table 25] |
| M2 / M3 cases | M2: 13.2 lb fuel (actual) + pylons, tanks, payload structure, X-1. M3: empty tanks + pylons + X-1 | [W p. 59, p. 45] |

**Missing, so PLACEHOLDERs** (OPEN_QUESTIONS):
- cell resistance (Dualsky 70C; keep 5 mΩ, Q8);
- empty bottle mass and diameter (1 L PET; Q36);
- pylon spanwise station (Q36);
- whether "payload structure" (0.44 lb) is pylons + adapters only (assumed);
- nacelle mass (0 in Table 34; Q25).

**Internal inconsistencies in the report, flagged:**
- **Table 34 M2 mass:** 26.02 lb against a component sum of 23.81 lb (2.21 lb unexplained).
- **Table 34 M3 mass:** 10.09 lb = M1 + payload structure + glider. It **omits the two empty bottles the rules require**.
- **Table 29 glider:** 0.2 lb in M2 vs 0.33 lb in M3; Table 25's all-up weight is 6.16 oz (0.385 lb).
- **Bottle drag:** CFD C_D = 0.523 per bottle (frontal area; Hoerner's long cylinder 0.82) [W p. 43] implies about 0.009 on the wing area for two 1 L bottles. That is **20× Table 26's 0.00045** [W p. 28]. The model uses the CFD value.
- **Wing-loading limits:** Table 4's "0.19–0.46 oz/ft²" [W p. 10] cannot be right next to "M2 65 oz/ft²" [W p. 11]. Units or values are off.

## 6. Table 39 [W p. 59]: inconsistencies

| Quantity | Reported | Check | Possible readings |
|---|---|---|---|
| M2 avg lap 37.4 s, total 138.1 s, 51.1 mph | 3 × 37.4 = 112.2 ≠ 138.1 | 9000 ft / 138.1 s = 44.4 mph; / 112.2 s = 54.7 mph; a 3000 ft lap at 51.1 mph = 40.0 s | (a) 138.1 s is the scored time (take-off included); (b) 112.2 s is 3 laps flown; (c) 51.1 mph is a GPS/ground speed over a longer path |
| M2 design: 87 mph, 33 s lap, 100 s | 87 mph on a 3000 ft lap = 23.5 s, not 33 s | — | the design speed is a cruise speed, the lap time includes turns |
| M3 7 laps, 260.5 s, avg lap **19.1 s** | 7 × 19.1 = 133.7 ≠ 260.5; 260.5 / 7 = 37.2 s; 52.3 mph ⇒ 39.1 s per 3000 ft | | 19.1 s is a typo (37.2 s?) |
| M3 Test 1 (Table 38): 6 laps, 285.3 s, 36 s avg | 6 × 36 = 216 ≠ 285.3 | | as above |
| Units | Table 38's "Avg. speed [s]" means mph | | |
| M2 battery reserve 33 % | the text says 40 % and 32 % at 35 A and 40 A; Table 38 has no 40 A test | | |
| "Pylon weight 13.2 lb" | the fuel-tank weight (Table 38: 13.23) | | |
| M3 laps 7 | scored pre-launch laps = 6 [R p. 12] | | |
| Take-off weights | M2 26.9 lb, M3 13.7 lb (Table 38) vs Table 34's 26.02 / 10.09 | M3 +3.6 lb | ballast added after P1 [W p. 60 §8.2.3], plus the bottles omitted from Table 34 |

**What the report says about the flights** [W p. 59–60]:
- average currents of **30–35 A in M2 and 17–17.5 A in M3**, against a design point of 60 A / 82 % throttle in M2 and 50 A / 54 % in M3 [W p. 26];
- "During P2 testing, a lower current setting than P1 was used, this improved the endurance while compromising speed";
- ballast was needed for stability.

**So the headline 87 → 51 mph gap is largely an operating point, not only a model error.** The validation compares at full throttle (the design case) **and** at the recorded current. That second case is an input condition taken from the flight record, not a fit: the throttle that reproduces the logged average battery current. Each Table 39 quantity is compared against every reading above.

## 7. AirCrafter 1.0

**Baseline (done).** The shipped files run **unmodified, in place** (`reference/aircrafter1/run_shipped.m`):
- A shim on the MATLAB path redirects only the hard-coded APC folder (`AirCrafter.m` l. 18, `/Users/mine/...`) to `THESIS/PERFILES2`, through `.txt` links (the parser reads `*.txt`, and PERFILES2 has `.dat`). It calls a verbatim copy of the parser; sha256 of every shipped file is in `shipped_copy/SHA256SUMS`.
- **Full APC set (443 props): 105 s,** 14,553 grid points (29,106 performance calls), then **a crash in `propSelector.m` l. 178** (bug M8 below).
- **The three props the team tested: 109 s**, complete, including the design point and the sensitivity study.
- Both pick the same design point: m = 7 kg, b = 1.5 m, AR 4, M2 at 30 m/s (the top of every range). **The M3 lap count is 1 at every grid point** (bug M2), so M3 never influences the choice.
- Outputs in `reference/aircrafter1/baseline/`.

**The shipped code is not the version used for Woombah.** Woombah's AirCrafter [W pp. 9–11]:
- grid: mass 11–55 lb, span 1.5–6 ft, **power 600–2700 W**, AR 3–12;
- Table 4 assumptions: C_D0 0.05, payload fraction 0.5, runway 350 ft, ≥ 4 laps, AR ≥ 5, M3 window 4 min;
- pairing rules: fixed geometry, M3 mass ≥ empty + glider, power free per mission.

The shipped code instead grids **cruise speed** (10–30 m/s), uses C_D0 0.012 and payload fraction 0.4, and flies M3 empty.

**Inventory of rule-dependent lines, and what `2025` mode does:**

| File: line | Content | Ruleset | `2025` mode |
|---|---|---|---|
| `inputs_file.m` 31 | take-off distance 100 m | — (2025 has no limit) | 350 ft runway, Woombah's own limit [W p. 10 Table 4], labelled engineering |
| `inputs_file.m` 36–37 | span 3–5 ft | not 2025 | grid range per §8, clipped to 6 ft [R p. 8] |
| `inputs_file.m` 42–43 | mass 3–7 kg | not 2025 | Woombah's 11–55 lb [W p. 9], TOGW < 55 lb [R p. 14] |
| `inputs_file.m` 46–51 | AR 4–9, V 10–30 m/s | team | AR 3–12 [W p. 9]; V range §8 |
| `inputs_file.m` 56, 64 | 5-min window, altitude 400 m | ✓ / site | 300 s ✓; Tucson ~750 m (the file's own comment) |
| `assumptions_file.m` 31–74 | payload fraction 0.4, C_D0 0.012, … | team | Woombah Table 4 values [W p. 10] |
| `assumptions_file.m` 79 | E = 0.8 × 100 Wh | ✓ 2025 [R p. 14] | ✓ |
| `AirCrafter.m` 49–53 | `bannerL`, `batteryFraction` | 2026 banner leftovers | banner 0; battery fraction 1 (M2 fix) |
| `AirCrafter.m` 172 | M3 mass = m (1 − PF): empty | not 2025 (M3 carries tanks + X-1) | empty + empty bottles + pylons + X-1 |
| `AirCrafter.m` 241–243 | M2_max = 10 kg / 100 s, M3_max = 8 + 2.5/0.2 | 2025-ish | R2, R3 (§4), shared with AirCrafter 2.0 |
| `AirCrafter.m` 251–256 | M2 = (m PF / t3) / M2_max (no "1 +"); M3 = (laps − 1 + **1/0.33**) / M3_max | 2025, bonus inconsistent | rule form: 1 + …, 2 + (laps − 1 + B/W)/R3, B as in §3 |
| `AirCrafter.m` 279–284 | wing loading 50–500 N/m² | team | kept |
| `AirCrafter.m` 304–328 | geometry fixed, speed free per mission, M3 mass linked | ✓ 2025 structure | kept |
| `performance_modeller.m` 55 | E = batteryFraction × E | bug M2 | fixed behind a switch |
| `performance_modeller.m` 73–84 | banner mass and m cap | 2026 | banner 0 |
| `propSelector.m` 54–78 | cargo/passenger groups (0.17 / 0.02 kg), fuselage-bay drag | 2026 | removed in `2025` mode (no 2025 payload counts) |
| `propSelector.m` 80–83 | banner drag C_D 0.03 | 2026 | banner 0 |
| `static_thrust_calculator.m` 32 | `sg = inputs.takeoff_distance` | — | 350 ft engineering limit |

**Store drag:** AirCrafter 1.0 has none. Its C_D0 assumption "includes fuel tanks" [W p. 10]. **Recorded as a limitation, not added.** It is also single-motor in `propSelector` (thrust per prop vs twin): a limitation.

**Bugs** (PLAN §8 M1–M7, plus new ones). "Fix" = in `2025` mode only, behind a switch, recorded in the docs:

| ID | Bug | Invalidates results? | `2025` mode |
|---|---|---|---|
| M1 | V_stall rounded to an integer (`round`) | no (bias ≤ 0.5 m/s) | keep |
| **M2** | `batteryFraction = 0` ⇒ E = 0 ⇒ lap count 1 everywhere (confirmed in the baseline) | **yes** | **fix** (fraction 1) |
| M3 | take-off time missing from lap 1 | no (method) | keep, document |
| M4 | decel mask reuse | no | keep |
| M5 | accel table errors at an unreachable V | no | keep |
| M6 | T_static computed but not used in the lap time | no (method) | keep |
| **M7** | NaN lap time when the turn-entry speed is below the decel table (m = 7, b = 1.5, AR 9, V = 25) | **yes** (silently drops points) | **fix** (keep the speed) |
| **M8 (new)** | `propSelector` reads past the last RPM block of a file whose data stop below 0.85 × 150000/D: crash with the full APC set | **yes** (crash) | **fix** (stop at the file end) |
| M9 (new) | M3 score uses bonus 1 / 0.33 but normalises with 2.5 / 0.2 | yes (scoring) | replaced by the rule form |
| M10 (new) | `ThreeLapTime` leaves out the 3 s per-lap reaction time that the lap count includes | no (method) | keep, document |
| M11 (new) | `propSelector` computes the minimum-power prop but returns the whole table | no | read the min-power row as "selected" |
| M12–M13 | `ind2sub` 5-output on 4-D arrays (M3_k used as an index); duplicate rows for tied scores | no | keep |
| M14 | hard-coded APC path; parser reads `*.txt` only | portability | path argument; `.txt` links |

**Copies.** `reference/aircrafter1/{original,2025}/` with a `mode` switch:
- `original` must reproduce `baseline/shipped_tested.*` to machine precision;
- it must also reproduce `shipped_full.*` up to the M8 crash.

## 8. AirCrafter 1.0 grid (`2025` mode)

- **Ranges:** Woombah's ranges [W p. 9], clipped to the rules: mass 5.0–24.9 kg (11–55 lb, TOGW < 55 lb), span 0.5–1.8 m (1.5–6 ft), AR 3–12.
- **Fourth variable:** cruise speed, as in the shipped method. The report's power axis isn't in this code. Range **10–50 m/s**, chosen so that Woombah's 87 mph (39 m/s) design speed is inside it.
- **Steps:** the shipped ones (0.5 kg, 0.1 m, 0.5, 1 m/s): 41 × 14 × 19 × 41 = 447,146 points, about **55 min** in MATLAB (≈ 3.6 ms per call, from the baseline).
- **Refinement study:** 2× and 4× the points per dimension, or stop when the best score moves < 0.5 %. 4× is about 4.5 days, so it is **only run if 2× still moves**, else skipped and stated.

## 9. Multipoint problem (`problems/multipoint.py`, `configs/dbf2025_multipoint*.yaml`)

**Structure:** two mission points, M2 and M3, in an `om.ParallelGroup`. Each is a full coupled analysis plus the store modules. RDO wraps it as N nodes × 2 points with the same RetryNewton warm starts. **M1** is an analysis-only check on the final design: M3's configuration without payload and pylons, 3 laps in 300 s.

**Design-variable split:**

| Variable | Shared / per point | Rule or reason |
|---|---|---|
| b, c, tc_scale, camber_scale, l_ht, S_ht, S_vt, x_cb_frac | shared | same configuration for all missions [R p. 10] |
| Kv (motor), n_motors, cells | shared (Kv a DV; the rest enumerated) | same configuration [R p. 10]; one battery type [R p. 14] |
| pylon count, external bottle | shared (enumerated) | max pylons in every M2/M3 attempt [R p. 10]; same external bottle for all missions [R p. 8] |
| **prop** | per point (enumerated pairs) | may change per attempt [R p. 14] |
| **fuel weight** | M2 only, continuous, ≤ n_ext × V_bottle × 6.5 lb/L (+ internal tank) | inert fill to any weight [R p. 8]; steel powder density [W p. 18] |
| **capacity** (per pack) | per point | other inspected packs allowed between attempts [R p. 16]; identical within an attempt; ≤ 100 Wh per pack and in total [R p. 14] |
| **x_batt** | per point | Woombah moves the battery per mission (nose "enabling CG adjustments for each mission" [W p. 19]; Table 34 battery at −0.83 in for M2 vs −29.37 in for M3 [W p. 45]) |
| **throttle** | per point (recommended, see below) | a pilot setting per flight |

**Throttle:** recommended as a per-point design variable.
- **M3:** laps are energy-limited, so full throttle runs the battery flat early. Woombah flew M3 at 54 % / 17–20 A [W p. 26, p. 59].
- **M2:** likely rides throttle = 1. I'll report that.
- **Prerequisite (found in this checkpoint):** the powertrain's T(v) fit is wrong at part throttle. At throttle 0.5 it gives 138 N at 40 m/s, because the advance ratio passes the APC data and the C_T polynomial extrapolates past windmilling.
- **Fix in checkpoint 2, before any part-throttle use:** limit the fit nodes to below each throttle's windmill speed, scaling the node range with throttle (smooth), plus a test. Existing configs keep throttle = 1 and stay bit-identical.

**Battery per mission:** recommended (the rule allows it, above). Both capacities bounded by 50 Wh per pack ÷ (n_s × 3.7 V) for two packs.

**Discrete choices:**
- external tanks n_ext = 2, the rule minimum (more tanks add drag and mass; Woombah's study agrees [W p. 9]);
- bottle 1 L (Woombah) in the main enumeration; 0.5 L (16.9 oz) and 2 L run only on the best case;
- internal tank: none in the main runs, 1 L once on the best case. It only adds capacity, and capacity is unlikely to bind.

**Store modules** (each gives mass, station and drag):
- **External tank:**
  - mass = empty bottle (PLACEHOLDER) + fill; fill is the M2 DV, 0 in M3.
  - drag D = q · C_D · A_front per bottle, with C_D = 0.523 from Woombah's CFD [W p. 43]; Hoerner's long cylinder 0.82, as cited there, is the high case in RDO.
  - interference factor Q = 1.0 PLACEHOLDER.
- **Pylon:** 0.44 lb for the set [W p. 45]; drag by skin friction × Raymer wing form factor on 8.1 × 1.0 × 1.82 in [W p. 18], in the existing `ParasiteDrag` style. In M2 and M3, removed in M1.
- **X-1:** 0.37 lb in **both** M2 and M3 (the rules carry it in M2 [R p. 11]; a lighter M2 X-1 is allowed [R p. 9], but Woombah's M2 X-1 weight isn't reported); release structure 0.22 lb [W p. 45]; skin-friction build-up on Table 25's wetted areas [W p. 24].
- **Internal tank:** mass and CG only.
- **Integration:** all of these add to `CD_extra` and to the CG and mass at each point, and stability is checked at each point.
- A store drag factor (multiplier) is an uncertain parameter (§11).

**Structural load case:**
- The wing is sized **once, at the M2 point** (the heaviest case: maximum fuel plus the heavier battery, which is also the rules' tip-test case [R p. 9, 17]). The **M3 point uses M2's airframe masses** and does not re-size.
- Load factor n = 5 with SF 1.5 (Woombah's design case [W p. 3, p. 20]; already `full_mdo.yaml`'s `n: 5.0`), the same for all designs.
- The tip test (≈ 2.5 g [R p. 17]) is a post-check: root bending moment with the airplane supported at the tips at maximum take-off mass, against the sized capacity. It is weaker than 5 g × 1.5, so it shouldn't bind.
- Structural design speed = the larger of the two points' level speeds (smooth max).

**Rule constraints:**
- span ≤ 1.829 m (upper bound; widened from 1.52);
- M2 take-off mass < 24.95 kg (55 lb);
- each pack ≤ 100 Wh and the total ≤ 100 Wh (capacity bounds);
- current per motor ≤ min(100 A fuse, C × capacity) [R p. 15] and ≤ 60 A ESC continuous [W p. 22]; the ESC is an engineering limit, and the binding one;
- M2 3 laps in 300 s: `laps_frac` ≥ 3 at the M2 point.

**Engineering constraints:**
- static thrust for a 350 ft ground roll: Woombah's own limit [W p. 10]. **PLACEHOLDER engineering, not a rule.**
- the existing stall, turn, SM, V_h/V_v, spar, rpm and v_fit constraints at each point.
- energy reserve: Q24, 20 % PLACEHOLDER.

**Kv bounds and v_top:**
- **Kv ∈ [400, 1050] rpm/V**, the V3115 family's catalogue windings [TM]. That is data, not a placeholder, and Woombah's 1050 is the top.
- **Reference motor** = the 1050 KV winding (Kv_ref 1050, Rm_ref 33.84 mΩ, I0_ref 1.79 A, 112.7 g).
- **Rewind vs catalogue:** the rewind model gives R ∝ Kv⁻², I0 ∝ Kv. The catalogue follows R ∝ Kv⁻¹·⁵⁵ and I0 ∝ Kv¹·¹⁶ [TM README]. I'll report the rewind model's error at the 400 / 640 / 900 windings and keep the rewind model (settled).
- **v_top:** at full throttle, 6S and 1050 KV, the Woombah-like M2 case converges at 52.6 m/s level speed (checked here, with v_top = 100). With v_top = 50/60 the MDA fails.
  - **Proposal: v_top = 80 m/s, confirmed in checkpoint 2** by a corner sweep (Kv 1050, throttle 1, the lightest M3 case, each prop) showing every level speed < 0.95 v_top, plus the fit residual over the wider range.
  - If the fit degrades, lower the Kv upper bound instead, and report it.

**Propulsion candidates** (the same for all three methods):
- n_motors = 2 (Woombah; the X-1 under the fuselage needs clean air for its pitot [W p. 14]);
- props **9x6E, 10x7E, 11x8E** (tested by the team [W p. 22]; in APC PERFILES2 and parsed by `propSelector`);
- motor V3115, Kv a DV;
- cells 4S, 5S, 6S, with capacity bounded by 50 Wh per pack.

**Prop pairs:** 3 × 3 = 9 per cell count.

## 10. Expected binding constraints

- **M2 fuel:** fuel rides its capacity (2 × 1 L × 6.5 lb ≈ 13 lb), or the M2 lap-time / rpm trade.
- **Kv:** rides `rpm_margin` (as in the Kv study) or the 60 A current at full throttle.
- **Span:** at its 1.829 m upper bound (Woombah's sensitivity study [W p. 8–9] and the rules).
- **M3:** energy limits the laps.
- **Static margin:** SM bounds at one of the points (stores move the CG).
- **Not binding:** TOGW and the 100 Wh total (two 50 Wh packs) are probably inactive or exactly at their bounds.

## 11. RDO and screening

- **Formulation:** as in `rdo_study.md`: UDR, k = 1, β = 2, maximise μ − σ of F.
- **Robust constraints:** `rpm_margin` and `I_margin` are added at both points. Any rule constraint active at the deterministic optimum is made robust too.
- **Uncertain parameters:** the 15 parameters (Kv_factor, Rm_ref), plus a store drag factor (PLACEHOLDER ±30 %, Q37). Fuel weight is not uncertain (weighed at the field).
- **Screening:** re-screen on the multipoint problem with the 90 % rule; report d.
- **Node check:** across both points before any robust run.

## 12. Run list and cost estimate (checkpoint 4)

Basis:
- the single-point deterministic optimum (11 DVs) takes 1.3 min (3 threads);
- the single-point robust optimum (N = 15) takes 25 min;
- **multipoint has 2 points and 17 DVs**, estimated at **4 min** deterministic and **~90 min** robust (N ≈ 17 × 2 points);
- Monte Carlo of 1024 points × 2 points ≈ 12 min per design (5 workers).

| Item | Runs | Est. |
|---|---:|---:|
| Deterministic enumeration (3 cells × 9 prop pairs) | 27 × 4 min | 1.8 h |
| Bottle / internal-tank options on the best case | 3 | 0.2 h |
| Multistart (10 Sobol' starts, best case) | 10 | 0.7 h |
| Bookends (M2-only, M3-only) | 2 | 0.1 h |
| Normalisation grid 3 × 3 | 9 | 0.6 h |
| AirCrafter 1.0 `2025` sweep + mapping (b) | 1 + 1 | 1.1 h |
| Screening + node check | — | 0.7 h |
| RDO: best 3 cases + 2 extra starts (best case) | 5 × 90 min | 7.5 h |
| Monte Carlo: Woombah, AirCrafter 1.0 (a), (b), multipoint, RDO | 5 × 12 min | 1.0 h |
| **Total** | | **≈ 13.7 h** |

**This exceeds the 12 h limit, so I propose these cuts:**
1. RDO on the **best case only**, plus 2 extra starts (3 runs instead of 5): −3 h.
2. The 4S and 5S cells only for the best prop pair (11 deterministic runs instead of 27): −1.1 h.

That brings it to **≈ 9.6 h**. The robust propulsion-ranking question is then answered for the best case against the deterministic ranking, not across all cases; I'd state that. Checkpoint 5 (timing, repeats, scaling) is separate, about 6–8 h, and mostly RDO repeats (3 × 90 min) and the RDO-vs-d runs.

## 13. Things to decide or know

1. **Normalisation placeholders:** R2 = 0.20 lb/s, R3 = 27 (§4). If you have the real raw best values, they replace these.
2. **Checkpoint 4 cuts** (§12).
3. **Per-point throttle**, which needs the part-throttle powertrain fix (§9). Recommended.
4. **v_top = 80 m/s and Kv ∈ [400, 1050]** (§9), to be confirmed in checkpoint 2.
5. **AirCrafter 1.0 `2025` grid:** Woombah's ranges with the shipped steps and a speed axis of 10–50 m/s (§8). The report's power axis isn't in the shipped code.
6. **Take-off is missing from the M2 time** in both tools [R p. 11 counts it]. This biases both fast against the flight data; stated, not fixed.
7. **The X-1 in M2 is 0.37 lb**, the same vehicle, not a lighter one (§9).
