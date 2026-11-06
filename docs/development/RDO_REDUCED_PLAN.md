# RDO with a reduced uncertainty set (mass and wind)

Design notes for the reduced uncertain set of the 2024-25 robust optimisation (branch `rdo-reduced`): the build-mass
factor `f_build`, the evidence for each spread, the contributions at the deterministic optimum, the node check, the
cost estimate and the decisions taken.

## 1. `f_build`: what was built

- **Definition.** `f_build` is a promoted mass input (default 1.0, `aircrafter2/mass/garcia_gamboa/inputs.py`). It
  multiplies every G&G structural mass inside the mass closure:
  - `m_wing` in `WingMass` (the tails follow, since G&G give `m_ht` and `m_vt` as fractions of `m_wing`, Eqs. 34–35);
  - `m_tb` and `m_cb` in `FuselageMass`;
  - `m_gear` in `GearMass`.

  It does not touch m_sys (motor, ESC, prop, battery, avionics), `m_ballast` or the payload. The loads, the spar and
  boom sizing, the gear's f_gear m and the total mass all see it.
- **Implementation choice (flagged).** The specification says "multiplies the sum ... inside the mass closure". I applied the
  factor to each structural component rather than to the sum in `MassSum`.
  - The totals and the closure are identical either way.
  - The difference is the CG. `stability/static.py` builds the CG from the component masses, so only the
    per-component form moves the CG when the airframe is heavier.
  - With the factor only in `MassSum`, a heavier airframe would leave the CG and the static margin unchanged.
  - This matters in practice. `f_build` carries 70–78 % of the static-margin variance below, so the choice is
    visible in the results.
  - The literal `MassSum` form would be a three-line change.
- **Overlap with `f_tb`.** Both scale the boom. This is accepted as settled: f_tb carries the CG effect of mass at the
  tail, f_build the overall build. It is stated in docs/modules/mass.md.
- **Tests** (`tests/test_rdo_reduced.py`, all pass):
  - `check_partials` (complex step) of the three components at f_build = 1.3. The existing parametrised mass
    partials test also perturbs f_build now.
  - The closed mass loop with f_build = 1.2: the structure grows by at least the factor, and the payload is
    unchanged.
  - Bit for bit at f_build = 1, compared with `==` and no tolerance:
    - every config in `tests/data/configs_reference.json`;
    - the published multipoint nominal of `results_2025.json` (flight score, M2, M3).
    - None of the references were regenerated. Exactness holds because x · 1.0 is exact and the summation order is
      unchanged.
  - A small RDO with f_build active (d = 1, N = 3): the nodes differ in f_build and m_total, both points of a node
    share its value, and the design variables are shared.
- **Configs.**
  - `configs/uncertainty_2025_reduced.yaml` holds the four parameters.
  - `configs/dbf2025_multipoint_rdo.yaml` now points at it, with every parameter active (d = 4; UDR N = 2d + 1 = 9 nodes
    × 2 points).
  - `configs/dbf2025_multipoint_rdo_full.yaml` keeps the published 15-parameter setup reproducible.
  - `studies/results_2025.py` takes `--set reduced|full` (default reduced):
    - the full set's outputs stay where they were published (`outputs/results/`, runs `rdo_*`);
    - the reduced set's outputs go to `outputs/results/reduced/` (runs `rdor_*`).

## 2. Evidence for each spread

All four remain PLACEHOLDERS (independent uniforms). Proposals are marked and not adopted.

### `f_build` ±30 % about 1.0 (settled), PLACEHOLDERS Q39

- **G&G Table 14** (individual correction factors of UBI11, GRA13, UBI15 against the combined value)
  [GG p. 292]:

  | factor | UBI11 | GRA13 | UBI15 | combined | range against the combined value |
  |---|---:|---:|---:|---:|---|
  | f_wing | 1.054 | 1.151 | 1.097 | 1.120 | −6 % … +3 % |
  | f_tb | 11.64 | 19.49 | 11.61 | 12.81 | −9 % … +52 % |
  | f_ht | 0.6609 | 0.6390 | 0.7840 | 0.6411 | 0 % … +22 % |
  | f_vt | 1.172 | 0.4252 | 0.4091 | 0.3414 (general) | +20 % … +243 % |
  | f_cb (kg/m²) | 0.8667 | 0.4615 | 0.6739 | 0.7878 | −41 % … +10 % |
  | f_gear | 0.0100 | 0.0033 | 0.0109 | 0.0089 | −63 % … +22 % |

  The individual factors scatter far more than ±30 %, but they partly cancel. The **whole structure**, built against
  predicted with the combined factors [GG Table 15, p. 293], comes out at:
  - UBI11: 1.305 against 1.424 kg, built/predicted = 0.92;
  - GRA13: 1.870 against 1.736 kg (reported, not weighed), 1.08;
  - UBI15: 1.219 against 1.259 kg, 0.97.

  So the structure scatters about ±8 %, but in-sample: the combined factors were fitted to these three aircraft.
  Out of sample it will be wider.
- **Woombah** (validation_2025.md §2, Design Report Table 34 p. 45):
  - Table 34 gives the structure (wing, HT, VT, fuselage and nose, undercarriage) as 6.05 lb. The uncalibrated model
    gives 2.21 lb, so built/predicted = **2.74**.
  - Take-off masses are 24–41 % light.
  - Table 34's structural masses are CAD estimates "verified against the constructed structure" [W p. 44].
  - The flight-test take-off mass is 26.9 lb against Table 34's 26.02 lb (Table 38), i.e. +3.4 % overall.
- **Reading.**
  - ±30 % is generous for a G&G-calibrated family (about ±8 %).
  - Woombah's factor of 2.7 is a bias of the uncalibrated model for USYD's construction, not a scatter. Neither ±30 %
    nor the one-sided [1.0, 1.3] reaches it.
  - **Proposed, not adopted:** treat the Woombah ratio as a calibration, i.e. a nominal shift (the calibrated mass
    model, or f_build ≈ 2.7), with ±30 % about that. Otherwise the robust design is robust to a ±30 % error on a mass
    that is itself about 2.7× too light. This would add one RDO run.

### `f_tb` ±30 % (unchanged)

- G&G's individual f_tb ranges from −9 % to +52 % against the combined 12.81; GRAVITY's 19.49 is the outlier, and its
  boom mass is reported, not weighed [GG pp. 292–293].
- The Woombah calibration did not refit f_tb: its boom is inside "fuselage + nose", and the model is −26 % on that row
  (validation_2025.md §2).
- So ±30 % is inside the observed range, but the evidence is skewed towards heavier booms. Kept as is.

### `m_avionics` ±15 % (unchanged)

- Woombah Table 34: receiver 0.19 lb and receiver battery 0.30 lb, i.e. 0.222 kg, the config value [W p. 45].
- The masses are manufacturer values for COTS parts [W p. 44], so the parts themselves are known to a few per cent.
- Table 34 has no wiring or connector row. For the glider the team weighed wiring and adhesives because they were
  significant at that scale [W p. 44].
- ±15 % reads as a bound on the unlisted wiring rather than on the parts. Kept as is.
- It barely matters: at most 5.5 % of any quantity's variance below.

### `lap.wind` 0–3 m/s (unchanged)

- The rules orient the course into the prevailing wind [R p. 3], which supports the model's headwind on the
  straights.
- The fly-off was at TIMPA, Tucson, AZ, 10–13 April 2025 [R pp. 6–7].
- Tucson's April mean wind is 7.1 mph (2013) and 7.8 mph (2014), i.e. 3.2–3.5 m/s, at the airport anemometer, with
  peak gusts of 42–43 mph. Source: Arizona State Climate Office monthly summaries
  (azclimate.asu.edu/docs/azclimate/MonthlyClimateSummaries/AzClimSumApr2014.pdf, …May2013.pdf).
- So the 0–3 m/s range sits **below** the site's mean: its centre is 1.5 m/s and its top is near the mean.
- **Proposed, not adopted:** 0–6.6 m/s, centred on about 3.3 m/s (or a site value from NCEI for April 2025). This would
  change the reference node's wind, so it belongs in a decision, not a silent change.

## 3. Run 1: contributions at the deterministic optimum (`mp_6S_11x8E_10x7E`)

First-order variance V_qi = (∂q/∂p_i σ_i)² at the means, from one `compute_totals` call
(`results_2025.py screen --set reduced` → `outputs/results/reduced/screen.json`).
- The σ columns are √(Σ V) over the four, and over the old 15 (`outputs/results/screen.json`).
- "variance share of the old 15" is Σ V(4) / Σ V(15).
- The last four columns are each parameter's share of the four-parameter variance.
- f_tb, m_avionics and lap.wind reproduce their old contributions exactly (checked), so the change is f_build plus the
  11 that are now fixed.

| quantity | σ (4) | σ (old 15) | variance share of the old 15 | f_build | f_tb | m_avionics | lap.wind |
|---|---:|---:|---:|---:|---:|---:|---:|
| flight_score | 0.00335 | 0.0183 | 0.033 | 0.647 | 0.007 | 0.020 | 0.326 |
| M2.stall_margin | 0.13 | 1.31 | 0.010 | 0.960 | 0.011 | 0.029 | 0.000 |
| M2.thrust_margin_static | 0.501 | 4.34 | 0.013 | 0.960 | 0.011 | 0.029 | 0.000 |
| M2.energy_margin | 382 | 7.27e+03 | 0.003 | 0.788 | 0.009 | 0.024 | 0.179 |
| M2.rpm_margin | 0.615 | 323 | 0.000 | 0.960 | 0.011 | 0.029 | 0.000 |
| M2.I_margin | 0 | 2.42 | 0.000 | – | – | – | – |
| M2.SM | 0.00654 | 0.0101 | 0.418 | 0.699 | 0.246 | 0.055 | 0.000 |
| M2.wall_ratio | 0.000362 | 0.000519 | 0.486 | 0.960 | 0.011 | 0.029 | 0.000 |
| M3.stall_margin | 0.217 | 1.13 | 0.037 | 0.960 | 0.011 | 0.029 | 0.000 |
| M3.thrust_margin_static | 0.284 | 3.3 | 0.007 | 0.960 | 0.011 | 0.029 | 0.000 |
| M3.energy_margin | 240 | 5e+03 | 0.002 | 0.769 | 0.008 | 0.023 | 0.199 |
| M3.rpm_margin | 0.098 | 327 | 0.000 | 0.960 | 0.011 | 0.029 | 0.000 |
| M3.I_margin | 0 | 1.68 | 0.000 | – | – | – | – |
| M3.SM | 0.0235 | 0.016 | 2.179 | 0.778 | 0.193 | 0.029 | 0.000 |
| M3.wall_ratio | 0.000362 | 0.000519 | 0.486 | 0.960 | 0.011 | 0.029 | 0.000 |
| M2.laps_frac | 0.0114 | 0.217 | 0.003 | 0.788 | 0.009 | 0.024 | 0.180 |

(r_tube has no variance in either set: the tube radius is a geometric limit.)

**What this says, before any RDO:**
- The flight score's σ falls from 0.018 to 0.0034 (3 % of the variance).
- The binding M2 constraints (energy, rpm, laps) keep 0–0.3 % of their variance. Kv_factor, Rm_ref, the thrust factor
  and the store drag carried them, and those are now fixed.
- The constraints that keep or gain variance are the static margins and the spar wall ratio:
  - M3.SM rises to 2.2× its old variance: the light M3 aircraft's CG moves with the structure's share of the mass.
  - M2.SM keeps 42 % of its old variance.
  - The wall ratios keep 49 %.
- Several constraints carry the same three shares (0.960 / 0.011 / 0.029): stall, take-off thrust, rpm and the wall
  ratios. They depend on these parameters only through the total mass.
- **Expectation:** the robust design will be close to the deterministic one. Where it differs, it should do so
  through the M3 static margin (whose upper bound the M1 check already flags) and the stall and thrust margins, not
  through the M2 rpm and energy margins that drove the full-set RDO. This is to be reported plainly, with no widening.

## 4. Run 2: node check (all 9 nodes × 2 points)

`results_2025.py node_check --set reduced` → `outputs/results/reduced/node_check.json`. The designs are the
optimum, the start and the six Sobol' designs used before.

| design | failed nodes | wall (s) |
|---|---:|---:|
| optimum | 0/9 | 13 |
| start | 0/9 | 14 |
| sobol0 … sobol4 | 0/9 each | 13–15 |
| sobol5 | 9/9 | 11 |

sobol5 fails at every node because its own nominal point does not converge. This is the same design-level MDA
non-convergence as in the full-set check, not a node problem.

## 5. Cost estimate

The estimates scale from compute_performance.md: RDO cost per driver iteration is about proportional to N. N = 9 here
against 15 at d = 7.

| run | estimate |
|---|---|
| 3: RDO best case, 3 starts (in parallel, 3 threads each) | ~31 s per driver iteration, clean. The full set took 26–67 iterations, so about 0.5–1 h wall for the three |
| 4: propagation at the robust optimum: UDR (9), Smolyak level 2 (49 nodes at d = 4), Sobol' MC 1024 | ~5 min |
| 5: Monte Carlo over the 4, six designs × 1024, plus the 15-parameter MC on the new RDO | ~1.5 h (the full-set MC of six designs took 75 min with 5 workers) |
| 6: UDR statistics at the 11 enumerated optima and 5 common designs | ~5 min |
| 7: one-sided f_build RDO, 1 start from the robust optimum | ~15–30 min |
| docs: results_2025.md §3 and §7 regenerated, full set kept as a subsection | – |
| 8: timing at d = 4, 5 repeats of 3 SLSQP iterations, plus a clean main RDO run | ~20 min + 0.3–0.6 h |

Total about 3–4 h of compute for stage 2 and about 1 h for stage 3.

## Open decisions

1. Is the per-component `f_build` (the CG sees it) acceptable, rather than the literal `MassSum` form? (§1)
2. Should the Woombah ×2.7 be treated as a calibration, with one extra RDO run, or only noted? (§2)
3. Should the wind range follow the Tucson April mean (proposal: 0–6.6 m/s), or stay at 0–3 m/s? (§2)

None of the three is adopted. Stage 2 runs with the settled values.

## Decisions taken (2026-10-21), and what changed after the first review

1. **`f_build` per part** (the CG sees it): kept.
2. **Woombah ratio as a calibration:**
   - `results_2025.py calibrate` solves f_build through the closure so that Woombah as built (at its logged-current
     design) has Table 34's 6.05 lb of structure: **f_build = 2.738**.
   - The raw ratio is 2.86 (2.11 lb modelled at that design; validation_2025.md §2 quotes 2.21 lb for its own case).
     The spar and gear resize with the heavier structure, which is why the solved factor is lower.
   - The script writes `configs/uncertainty_2025_reduced_calibrated.yaml` (f_build centred there, ±30 %).
     `configs/dbf2025_multipoint_rdo_calibrated.yaml` is one extra RDO run from the main robust optimum
     (results_2025.md §7b).
3. **Wind widened**, then capped at the lap model's validity (`configs/uncertainty_2025_reduced.yaml`):
   - **Sites:** DBF alternates between Tucson (2025 [R pp. 6–7]) and Wichita. Their April means are 3.7 m/s (8.3 mph,
     NOAA Comparative Climatic Data via currentresults.com) and 5.6 m/s (12.6 mph, Weather Spark, airport station).
   - **First try:** a range centred on the mean of the two, 0–9.34 m/s. It broke the lap model. Lap 1 starts at
     V_stall on the into-wind straight, and the straights are integrated in ground distance, dt/ds = 1/(v − wind).
     At 8.29 m/s, the top UDR node, the light M3 aircraft (V_stall 8.3 m/s) has almost zero ground speed: M3 laps
     came out at 1e30, then NaN, and nothing flagged it. The first RDO attempt stopped on it. That output is kept
     in `outputs/results/reduced/invalid/` and is not used.
   - **Decision:** cap the range at **0–7 m/s, centre 3.5 m/s**. It covers Tucson's mean but cuts Wichita's windier
     days. At 7 m/s the lap model holds from a cold start, with about 1 m/s margin to the light f_build = 0.7
     node's V_stall (7.99 m/s). At 7.5 m/s that node gives a negative lap time.
   - **Safeguard:** `SmoothLapModel` now raises `AnalysisError` on a non-physical lap (non-finite, or a lap time ≤ 0),
     so such a point is an analysis failure, never a score. Valid points are unaffected; the lap tests pass.
   - **Calm-air nominal:** the deterministic designs were optimised in calm air, and the wind distribution's mean
     (3.5 m/s) is not calm. At the mean the deterministic optimum misses M2's 3 laps (laps_frac 2.98), so the
     reference node is not the deterministic nominal. This was already true of the 0–3 m/s range (centre 1.5 m/s).

The contributions table in §3 above is from stage 1, with the 0–3 m/s wind. The current tables (0–7 m/s, and the
calibrated and one-sided sets) are in results_2025.md §7 and §7b–c. With the wider wind, wind carries most of the
flight score's variance.

**Wind limit (decided 2026-10-21):** an aircraft may have a wind limit; if it is too windy it does not fly. A
headwind near the light M3 aircraft's V_stall is therefore a no-flight condition of the design, not a model error.
- It is reported as "wind limit (no flight)", with its probability per design, separately from MDA non-convergence
  and from constraint violations, and kept out of the score statistics.
- Within 0–7 m/s it occurs only for the bigger-wing designs (AirCrafter 1.0's mappings, multipoint 2 L), at 6.2–7 m/s.
- PLACEHOLDER (recommended default, not asked): the RDO does not constrain the wind limit. A design whose wind limit
  falls inside the range shows it in the Monte Carlo, not in the optimiser. Constraining it, e.g.
  V_stall(M3) − wind_max ≥ margin, is a possible extension.

## Status (2026-10-21)

All three stages are done:
- stage 1 at `e86c78f`;
- stage 2 at `3d6cccd`;
- stage 3 (timing at d = 4, the clean main run, compute_performance.md, INDEX.md) committed with this line.

Results: results_2025.md Findings 7–11 and sections 3 and 7. Timing: compute_performance.md §3 and §5.
