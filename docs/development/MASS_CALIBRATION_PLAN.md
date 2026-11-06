# Mass-model calibration on three USYD aircraft

Design notes and data for calibrating the Garcia & Gamboa mass model on Orange Hornet, Woombah and Humpback (branch
`mass-calibration`, from `placeholders-rulesets`). The numbers in these notes come from a standalone run of the mass
group made before the calibration was built.
`studies/mass_calibration_usyd.py` regenerates all of them, and its output
(`docs/studies/mass_calibration_usyd.md`) supersedes this plan's tables.

Sources (printed page numbers):
- [OH] Orange Hornet, USYD DBF 2023-24 (`3_LITERATURE/usyd reports/2024 USYD DBF Design Report (Orange Hornet).pdf`)
- [W] Project Woombah, USYD DBF 2024-25 (`3_LITERATURE/25 competition/2025 USYD DBF Design Report.pdf`)
- [HB] Project Humpback, USYD DBF 2025-26 (`3_LITERATURE/usyd reports/2026 USYD DBF Design Report (Project Humpback).pdf`;
  the PDF page index is one lower than the printed page number)
- [GG] Garcia & Gamboa (2022)

## Decisions (2026-10-23)

1. **Tail form: `areal`.** Eq. 128 objective 0.553 (areal) against 0.782 (gg). The LOO errors on the tail
   components have an RMS of 22 % (areal) against 34 % (gg). The LOO structure-total errors are mixed:
   - areal: −28.8 / +12.4 / +23.8 % ([OH] / [W] / [HB]);
   - gg: −30.0 / +15.4 / +18.2 %.

   Areal has the lower maximum and gg the lower RMS (22.7 against 22.1 %). Both forms give the same f_build spread
   (30 %). Areal was chosen on the components the tail form changes, and the mixed totals are reported in the
   calibration doc.
2. **k_fill stays 0.85.** The spar check gives +9 % [OH] and −16 % [W]; the specification's estimate was +14 / −6 %. The
   conclusion is the same: a front-spar fit makes [W]'s single tube 3.5 mm thick, t/r 0.41, over `wall_ratio_max` 0.2.
3. **One commit per stage.** The references are regenerated in their own commit after stage 2's list of changes.

## 1. Extraction, checked against the PDFs

Every value of the specification's table was read back from the PDFs. All of them match. What I read differently, or what
the specification leaves out:

| Item | Specification | Reports | Used |
|---|---|---|---|
| [OH] Table 35 | p. 42 | Table 35 is on p. 42 (CG column "y-Disp." is the longitudinal axis) | as specification |
| [OH] site | Wichita, 1300 ft (p. 25) | p. 25: "standard atmosphere density at an elevation of 400 m" (= 1312 ft) | 400 m |
| [OH] span | 1.524 m (Table 33) | Table 22 (p. 21) and Table 36 (p. 43) round it to 1.5 m | 1.524 m |
| [OH] wing endplates | not mentioned | 1 mm plywood endplates, h/b = 1/20 (p. 21) | `winglet_ratio` 0 (see note 1) |
| [OH] midsection longerons | not used | four 12/10 mm (p. 19) against four 10/8 mm (p. 39) | flagged only |
| [OH] spar step-down sizes | 20/18, 18/16 in the tube list | step-down at 0.5 m semi-span, 50 mm overlap (p. 34); outer tube sizes not printed | outer tubes 18/16 (front), 16/14 (rear): the next size inside, PLACEHOLDER |
| [OH] y_motor | find it in the drawings | not dimensioned. Scaled from the three-view (p. 44, 1524 mm span): 0.335 m | 0.335 m, PLACEHOLDER (sensitivity below) |
| [W] cargo box width | 4.7 in (Fig. 11, p. 19) | Table 30 (p. 31): 4.72 in maximum width (`woombah_2025.yaml` 0.120 m) | 0.1194 m (Fig. 11, the source of the 0.820 m) everywhere, including `woombah_2025.yaml` |
| [W] outer spar sizes | 0.87/0.79 and 0.79/0.71 in | outer panels "stepped down by 0.08 in", 2 in overhang into the mid-wing (p. 33): front 0.79/0.71, rear 0.71/0.63 in | as the report |
| [HB] spar sizes | 0.63/0.55 and 0.39/0.31 in | Table 24 (p. 26): 0.63/0.55 and 0.39/0.31. Text on p. 26: "0.71 in/0.63 in front, 0.55 in/0.47 in rear". p. 37: 5/8 and 3/8 in. All walls are 0.04 in | Table 24 (only the 1 mm wall is used) |
| [HB] spar layout | CF joiners | CF tubes only across the fuselage junction. Full-span 1/4 × 1/2 in Celery Top Pine caps and balsa webs form the I-beam outboard (pp. 26, 37). The tails use the same hybrid (p. 37) | one full-span G&G tube; `k_wing` absorbs the difference |
| [HB] area | 868 in² | p. 36: 868 in², but 59 × 14.57 = 859.6 in² | 868 in² (S); c from the specification, 14.57 in |

Notes:
1. **[OH] endplates.** The model has no endplates and the optimiser's designs have none, so `winglet_ratio` stays 0
   and [OH]'s `k_wing` absorbs them. Modelling them as winglets (ratio 0.10) would claim about 10 % of the wing. Two
   1 mm plywood plates of about 0.033 m² each weigh roughly 3 %.
2. **[OH] "Empenage" is the rear fuselage.** I read it the same way as the specification. Its CG is 898 mm aft of the LE
   datum. The 1050 mm midsection sits with its centroid at the wing quarter chord (p. 19), and the "Fuselage" row's
   CG is 108 mm. The 680 mm rear fuselage (p. 39) therefore spans about 630 to 1310 mm, and the HS and VS rows are at
   1169 and 1153 mm.
3. **[OH] W&B basis.** The values are CAD estimates (Onshape density × volume, p. 41). P2 flew at 8.6 kg with the
   2.8 kg M2 payload (Table 43, p. 59), i.e. 5.8 kg against Table 35's 5.84 kg M1. P1's "airframe weight" was 7.9 kg
   (p. 59). Table 44 (p. 59) gives the M2 take-off as 8.9 kg actual, while Table 43 gives 8.6 kg.
4. **[W]** CAD, "verified against the constructed structure" (p. 44). Table 34 has no servo row.
5. **[HB]** Table 36 has rows only for the fuselage (6.14 lb, which also holds the gear, servos, receivers, receiver
   batteries, prop and wiring), the tails, the wings, the motor and the ESC; they sum to the 9.93 lb empty weight. So
   [HB] is used for the wing and the tails only. Mass basis and table inconsistencies: §6.

### Component masses (model output ← report row)

| Output | [OH] Table 35, p. 42 (g) | [W] Table 34, p. 45 (lb) | [HB] Table 36, p. 44 (lb) |
|---|---|---|---|
| `m_wing` | Wing 1517.696 | Wing 3.41 | Port 0.79 + Starboard 0.79 |
| `m_ht` | Horizontal Stabaliser 203.763 | Horizontal Stabilizer 0.41 | Port HS 0.21 + Starboard HS 0.21 |
| `m_vt` | Vertical Stabliser 90.203 | Vertical Stabilizer 0.19 | Vertical Stab 0.28 |
| `m_cb` | Fuselage 903.552 + Nosecone (inc. crew restraint) 356.884 | – | – |
| `m_tb` | Empenage 231.598 (the rear fuselage) | – | – |
| `m_fus` | (m_cb + m_tb) | Fuselage 1.23 + Fuselage nose 0.15 (lumped target) | not used |
| `m_gear` | Undercarriage 366.567 (tricycle) | Undercarriage 0.66 (taildragger) | not used |

## 2. Case inputs

The geometry uses the optimiser's definitions:
- rectangular-equivalent chord: `c_root = c_mean = c`;
- `l_tb = l_ht − boom_offset`, with the placeholder `boom_offset` 0.15 m;
- `S_cb = 2(lw + lh + wh)`.

The calibration uses external sizing: the loads see the sizing mass, with no closure. Common to all three cases:
- n = 5 and SF = 1.5, cited for all three reports;
- the other inputs as `placeholders.yaml` and `mass_garcia_gamboa_2022.yaml`;
- `motor_inertia_relief` off and `nacelle_torsion` on, as the 2024-25 problem.

| Input | [OH] | [W] | [HB] |
|---|---|---|---|
| b (m) | 1.524 | 1.801 | 1.4986 |
| c (m) | 0.435 | 0.3276 (= S/b) | 0.3701 |
| S (m²) | 0.66294 | 0.5900 | 0.5600 |
| airfoil seed | clarky | clarky | s3016 (AeroSandbox UIUC database; p. 36 prints "SD3016") |
| t/c, t/c at the spar (0.25c) | 0.1173, 0.1165 | 0.1173, 0.1165 | 0.0951, 0.0945 |
| A/c², P/c | 0.0810, 2.045 | 0.0810, 2.045 | 0.0626, 2.028 |
| Cm, CL_α (1/rad) | −0.0833, 3.42 | −0.0834, 4.12 | −0.0327, 3.64 |
| n_panels | 1 (confirmed: one-piece swivel wing, step-downs epoxied, pp. 21, 34, 49) | 3 (p. 20) | 2 (p. 26, p. 37) |
| wing units | twin: m_u = (710 + 258 + 90)/2 g, x −0.130 m (p. 20), y 0.335 m (PLACEHOLDER) | twin, as `full_mdo_twin.yaml` (0.2515 m, −0.1397 m) | none |
| S_ht, S_vt (m²) | 0.1909, 0.0729 | 0.1525, 0.0670 | 0.1484, 0.0586 |
| tail arm (m) | 1.00 | 1.100 | 1.1 |
| cargo box l × w × h (m) | 1.050 × 0.167 × 0.156 | 0.820 × 0.1194 × 0.1194 | – |
| sizing mass | 8.0 kg | 26.02 lb | 14.3 lb |
| V_design | 140 km/h | 87 mph | 100 ft/s |
| site (ρ) | 400 m | 800 m | 1300 ft |

**Cm and CL_α** come from the aero module, as in the optimiser: the wing (`aero/wing.py`, VLM + NeuralFoil strips)
at V_design, the site density, and the α at which L equals the sizing weight. The values were checked for
sensitivity:
- [OH] and [HB] sit on the 1 mm minimum gauge, so Cm and CL_α do not change their wings.
- [W]'s tube is strength- and torsion-sized (1.17 mm). With Cm −0.15 and CL_α 3.5 its wing is 5.6 % heavier.

## 3. Uncalibrated real/predicted, G&G factors

Before Step 1, `t_tube_min` is 0.5 mm and `sigma_spar` 720 MPa. After Step 1 they are 1.0 mm and 313 MPa.

| Component | [OH] before | [OH] after | [W] before | [W] after | [HB] before | [HB] after |
|---|---:|---:|---:|---:|---:|---:|
| wing | 3.62 | **2.66** | 3.25 | **2.44** | 2.40 | **1.75** |
| HT | 2.64 | 1.94 | 2.36 | 1.77 | 3.76 | 2.74 |
| VT | 5.74 | 4.21 | 4.66 | 3.51 | 11.92 | 8.69 |
| cargo box | 2.19 | 2.19 | – | – | – | – |
| tail boom | 4.27 | 4.27 | – | – | – | – |
| fuselage (lumped) | – | – | 1.60 | 1.60 | – | – |
| gear | 5.15 | 5.15 | 2.85 | 2.85 | – | – |
| spar wall | 0.50 mm | 1.00 mm | 0.68 mm | 1.17 mm | 0.50 mm | 1.00 mm |

These are within ~4 % of the specification's estimate (wing 2.69 / 2.35 / 1.75). E_spar changes no wing mass (42.9 and
220 GPa give the same masses), because the deflection limit never binds.

**Quick fit** (a linear least-squares probe; the real fits run through `calibration.py` at stage 2):

| Fit | k_wing | a_ht | a_vt | f_cb | f_tb | f_gear | Eq. 128 |
|---|---|---|---|---|---|---|---|
| areal | 2.14 | 1.18 kg/m² | 1.39 kg/m² | 1.27 kg/m² | 3.27 | 0.030 | 0.553 |
| gg | 2.14 | f_ht 0.60 | f_vt 0.67 | 1.27 kg/m² | 3.27 | 0.030 | 0.782 |

## 4. Spar-mass check (k_fill)

Real CF spar mass from the tube sizes and spans, with ρ = `rho_spar` = 1600 kg/m³:

- **[OH]** 0.281 kg: front 0.149, rear 0.133. The centre tubes are 1.0 m long. The outer tubes are 0.262 m plus a
  50 mm overlap, and their sizes are inferred (see §1).
- **[W]** 0.369 kg: front 0.194, rear 0.175. The mid-wing is 2.95 ft. The outer panels are stepped down 0.08 in,
  with a 2 in overhang.

| k_fill | [OH] r, t | [OH] m_tube vs real | [W] r, t | [W] m_tube vs real |
|---|---|---|---|---|
| 0.85 (placeholder) | 20.5 mm, 1.00 mm | 0.307 kg, **+9 %** | 15.2 mm, 1.17 mm | 0.309 kg, **−16 %** |
| front-spar OD (0.434 / 0.632) | 10.0 mm, 2.00 mm (t/r 0.20) | −2 % | 11.1 mm, 2.14 mm (t/r 0.19) | +5 % |
| 0.5 | 11.7 mm, 1.48 mm | −12 % | 8.5 mm, 3.52 mm (**t/r 0.41**) | +17 % |

k_fill 0.85 stays (decision 2). It is now a checked equivalence: USYD carry the bending in two tubes, and the model
uses one tube at the full allowed depth. The resulting spar masses are within +9 / −16 %. Fitting k_fill to the front
spar makes the single tube thick-walled (t/r up to 0.41), outside the thin-wall equations. Q5 is updated.

## 5. Where each value lives

| Value | File | Status |
|---|---|---|
| `t_tube_min` 1.0 mm, `sigma_spar` 313 MPa | `placeholders.yaml` (lines changed in place) | cited ([OH] p. 36; [OH] p. 35, §8.1.1 p. 55) |
| `k_wing`, `a_ht`, `a_vt`, `f_cb`, `f_tb`, `f_gear` (combined, Eq. 128) | `placeholders.yaml`, new block "mass correction factors (USYD calibration)" | cited; Q..; `docs/studies/mass_calibration_usyd.md` |
| `f_wing` 1.12 | the same block | cited to G&G Table 14 |
| `tail_mass_model: areal` | the same block, as `mass.options.tail_mass_model` | `mass.options.*` dotted paths load like `mass.inputs.*` (`mass_config` < placeholders < config) and are rows in the Placeholders tab |
| `n_panels` 3, cargo box 0.820 × 0.1194 × 0.1194 m | `rulesets/dbf_2024_25.yaml` (`mass.options.n_panels`, `geometry.l_cb/w_cb/h_cb`) | cited [W] |
| `woombah_2025.yaml` `l_cb` 0.820, `w_cb`/`h_cb` 0.1194 | `woombah_2025.yaml` (already sets them; conflict flagged and resolved to the ruleset values) | cited |
| LOO factors ([OH] + [HB]) | `configs/mass_usyd_loo_woombah.yaml` | in-sample for [OH] and [HB] only |
| `f_build` spread | `placeholders.yaml` uncertainty line, nominal 1.0 | cited, Q39 |
| one-sided [1.0, 1.3] | `uncertainty_2025_reduced_onesided.yaml` | cited to [HB] p. 43 (≈20 % CAD → built) |
| superseded headers | `mass_usyd_woombah.yaml`, `uncertainty_2025_reduced_calibrated.yaml`, `dbf2025_multipoint_rdo_calibrated.yaml` | header line only |

**Conflicts with the code found during the inventory:**
- **Layer order.** `placeholders.yaml mass.inputs` sits *above* `mass_config`. A factor file used as `mass_config`,
  such as `mass_usyd_loo_woombah.yaml` or `mass_usyd_woombah.yaml`, would therefore be overridden by the calibrated
  placeholders. The validation applies these files at the problem level (`validation_2025.py` merges their
  inputs and options over the loaded config). Its "uncalibrated" column applies the G&G factors of
  `mass_garcia_gamboa_2022.yaml` the same way, with `tail_mass_model: gg`, `k_wing` 1 and the Step 1 physical
  values.
- **One place per value** (`tests/test_placeholders.py`). `mass_garcia_gamboa_2022.yaml` keeps the published G&G
  factors (Table 14), which the verification and the "uncalibrated" comparison read. It joins the test's `ALLOWED`
  list, as do `mass_usyd_loo_woombah.yaml` and the 2024-25 Woombah box in `woombah_2025.yaml`.
- **The migration snapshot test** compares every config with its pre-migration dump. It gets this specification's changes as
  a second list of expected differences: mass inputs and options for every config, and the cargo box and n_panels
  for the 2024-25 configs.
- **`f_build`'s placement.** It already multiplies the m_wing that feeds the gg tails. In `areal` mode it must
  multiply m_ht and m_vt directly. `BUILD_NOTE` is updated.

## 6. Report inconsistencies (to PLACEHOLDERS)

- [HB] mass basis: CAD values "validated against the measured weights … with a ≈20 % increase from CAD to
  manufacturing" (p. 43). The report does not say whether Table 36 includes the 20 %.
- [HB] Table 36: the "Mission 1" row reads 1.22 lb, the battery alone (empty 9.93 + 1.22 = 11.15 lb). M2 is printed
  as 18.54 lb against 18.24 lb from its own rows. The M1 take-off weight is 8.6 lb (Table 33, p. 35) and the MTOW
  14.3 lb (pp. 3, 36), both below the 11.15 lb empty weight plus battery.
- [HB] spar sizes: p. 26 text, Table 24 and p. 37 disagree (§1).
- [HB] airfoil: "SD3016" (p. 36) against S3016 (Table 24, p. 26; p. 37). The UIUC S3016 is used.
- [HB] area: 868 in² against 59 × 14.57 = 859.6 in².
- [W] Table 34: no servo row.
- [W] cargo box width: 4.7 in (Fig. 11) against 4.72 in (Table 30).
- [OH] longerons: 12/10 mm (p. 19) against 10/8 mm (p. 39).
- [OH] M2 take-off: 8.6 kg (Table 43) against 8.9 kg "actual" (Table 44).
- [OH] σ: "3.13 MPa" on p. 55 against 313 MPa on p. 35 (a typo).
- [OH] y_motor: not dimensioned.

## 7. Regression references (stage 2)

Every full config now loads the calibrated mass model, so every regenerable reference entry changes. Start point,
one coupled analysis (`tests/configs_snapshot.py`, `tests/rdo_off_snapshot.py`). The largest relative change is
m_wing (×3.1: k_wing 2.14 on a 1 mm spar, against 0.5 mm before), or wall_ratio (the spar wall doubles):

| Reference entry | m_total (kg) | ThreeLapTime (s) | largest change |
|---|---|---|---|
| full_mdo.yaml | 2.766 → 3.552 (+28 %) | 77.53 → 79.07 (+2.0 %) | m_wing 206 % |
| full_mdo_twin.yaml | 3.360 → 4.161 (+24 %) | 74.54 → 75.45 (+1.2 %) | m_wing 206 % |
| full_mdo_fixed_kv.yaml | 2.741 → 3.527 (+29 %) | 81.44 → 83.52 (+2.6 %) | m_wing 206 % |
| full_mdo_twin_fixed_kv.yaml | 3.283 → 4.082 (+24 %) | 77.16 → 78.18 (+1.3 %) | m_wing 206 % |
| full_mdo_rdo.yaml | 2.766 → 3.530 (+28 %) | 78.82 → 80.42 (+2.0 %) | m_wing 206 % |
| full_mdo_twin_rdo.yaml | 3.360 → 4.139 (+23 %) | 75.17 → 76.14 (+1.3 %) | m_wing 206 % |
| full_mdo_rdo_fixed_kv.yaml | 2.741 → 3.505 (+28 %) | 83.23 → 85.99 (+3.3 %) | m_wing 206 % |
| full_mdo_twin_rdo_fixed_kv.yaml | 3.283 → 4.060 (+24 %) | 78.18 → 79.27 (+1.4 %) | m_wing 206 % |
| kv_dv:full_mdo.yaml | 2.766 → 3.552 (+28 %) | 77.53 → 79.06 (+2.0 %) | wall_ratio 100 % |
| kv_dv:full_mdo_twin.yaml | 3.360 → 4.161 (+24 %) | 74.55 → 75.46 (+1.2 %) | wall_ratio 100 % |

The `configs_reference.json` and `kv_dv_reference.json` entries are marked strict xfail (`tests/migration.py`) until
they are regenerated in their own commit. **Not regenerated, ever:** `rdo_off_reference.json` (pre-RDO) and
`multipoint_nominal_reference.json` (the published 2024-25 nominal). Their tests now apply the pre-calibration mass
model (`tests/migration.py pre_calibration`), and both reproduce bit for bit. This also shows that nothing else in the
model moved.

## 8. Stages

1. This plan (commit).
2. `k_wing`, `tail_mass_model`, `f_build` on the areal tails, the calibration hook (external sizing, lumped
   targets, LOO, bounds), tests; `studies/mass_calibration_usyd.py fit/report`; configs; full suite; the list of
   reference changes (commit), then the regenerated references (their own commit).
3. Woombah validation with the LOO config; one deterministic 2024-25 multipoint optimum; the RDO cost estimate;
   `mass.md`, PLACEHOLDERS, PLAN G5, INDEX (commit).

## 9. Outcome (stages 2 and 3)

- Stage 2 `725cb51`; references regenerated `da1158f`; stage 3 the commit after. Full suite on stage 2:
  337 passed, 18 strict-xfail (the reference entries), then 0 xfail after the regeneration.
- Fits, LOO, gg vs areal, sensitivities, closure check and the calibrated optimum: `docs/studies/mass_calibration_usyd.md`.
- Woombah validation, out-of-sample: `docs/studies/results/validation_2025.md` §2–3 (`studies/validation_2025.py woombah
  --mass gg|loo`). The pre-calibration run (`outputs/results/validation_woombah.json`) is kept; the superseded
  Woombah-only refit stays as a column.
- RDO reruns on the calibrated model (2026-10-23, after the specification): the reduced set (calibration doc §10, `--set
  usyd`, runs `rdoru_*`, `outputs/results/reduced_usyd/`), the full 15-parameter set (§11, `--set full_usyd`, runs
  `rdofu_*`, `outputs/results/full_usyd/`), and the Monte Carlo at the calibrated deterministic and robust optima
  plus the propagation check (§12). The published `rdor_*` / `rdo_*` runs and outputs are untouched. Not rerun:
  the common evaluation of the other designs (Woombah, AirCrafter 1.0, the 2 L bottles). `results_2025.md`,
  `kv_study.md`, `rdo_study.md` and `twin_motor_study.md` still show the uncalibrated model; INDEX.md says so.
