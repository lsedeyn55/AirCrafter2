# Thesis results on the 2025-26 ruleset (Project Humpback)

Plan and model changes for the thesis's 2025-26 results (branch `results-2026`, from the tag
`results-2024-25-calibrated`): the rules as read, the normalisation, the Humpback data, the model changes, the
AirCrafter 1.0 mode, the uncertainty set and the run list, then the fixes made before the final run and a generated
summary of that run. The run log is `docs/development/RESULTS_LOG.md`; the result pages are in
`docs/studies/results_2026/`.

**Sources and citation keys** (printed page numbers throughout):
- **[R26]** `3_LITERATURE/26 competition/2026-DBF-Rules.pdf` (34 pp., downloaded 2026-10-26 from aiaa.org). Printed
  "N of 34" = PDF page.
- **[RES26]** `3_LITERATURE/26 competition/2026-DBF-Competition-Summary.pdf` (8 pp., aiaa.org, April 2026): the official
  final results (normalised scores only).
- **[HB]** `3_LITERATURE/usyd reports/2026 USYD DBF Design Report (Project Humpback).pdf` (59 PDF pages, printed
  "Page N of 60"): **printed = PDF + 1** (the contents page is unnumbered). Printed numbers below.
- **[W]** Woombah (2025), **[OH]** Orange Hornet (2024), **[GG]**, **[J]** as already cited in the repo.
- **[TM7]** `3_LITERATURE/26 competition/T-Motor AT7215/` (spec table and the 245 KV test report, transcribed from
  T-Motor's store page with its HTML saved; README there).
- **[APC]** `3_LITERATURE/26 competition/APC/APC-Propeller-RPM-Limits-rev5.pdf` (APC, Rev 5, 23 Mar 2022).

## 0. Phase A (done: tag `results-2024-25-calibrated`)

The 2024-25 pages are regenerated on the calibrated runs (`fc2c577`), take-off off as published; §3b/§7d point
to `mass_calibration_usyd.md` §11–12; `classify` skips without a Monte Carlo; the uncalibrated pages
(`takeoff_study.md`, `compute_performance.md`) carry a header note; housekeeping done. Suite: 355 passed.

| Headline (2024-25, take-off off) | published (uncalibrated) | calibrated |
|---|---:|---:|
| best deterministic F (6S 11x8E/10x7E, 1 L) | 5.1394 | 5.0993 |
| Woombah as built, nominal F | 5.1447 | 5.1020 |
| AirCrafter 1.0 direct / completed | 5.4295 / 5.3777 | 5.3735 / 5.2879 |
| multipoint 2 L (like-for-like) | 5.6868 | 5.4710 |
| like-for-like gain over AirCrafter 1.0 direct (completed) | +4.7 % (+5.7 %) | +1.8 % (+3.5 %) |
| reduced-set RDO μ − kσ (nominal F) | 5.1188 (5.1386) | 5.0770 (5.0986) |
| largest P(violation), multipoint → RDO | 0.866 → 0.028 | 0.621 → 0.016 |
| 2 L bottle fuel | 26.0 lb (fills) | 22.9 lb (M2 static-thrust margin binds) |
| M1 check at the best case | fails | **passes** (the specification's "fails" is not what the calibrated run records; Woombah and AirCrafter 1.0 direct fail it) |

**Found in Phase A (open):** rerunning `mass_calibration_usyd.py fit` on the committed tree reproduces the combined
factors exactly, but [OH]'s leave-one-out structure error comes out −34.1 % (published −26 %), which would make
f_build's spread ±35 % instead of ±30 %. The published fit was run from an uncommitted tree at `aff3998`. Nothing was
changed (the in-place write was reverted); see §9 Q9.

## 1. [R26] as read

| Item | [R26] | Page | Reading |
|---|---|---|---|
| Score | Total Mission Score = M1 + M2 + M3 + GM | 32 | ✓ (F = M1 + M2 + M3; GM not modelled) |
| M1 | 3 laps in the 5-min window, no payload; M1 = 1.0 | 19 | ✓ |
| M2 income | Income = n_p (Ip1 + Ip2 N) + n_c (Ic1 + Ic2 N); Ip1 6, Ip2 2, Ic1 10, Ic2 8 | 19 | ✓ |
| M2 cost | Cost = N (Ce + n_p Cp + n_c Cc) EF; Ce 10, Cp 0.5, Cc 2; EF = total propulsion battery capacity (Wh) / 100 | 20 | ✓ |
| M2 | M2 = 1 + N_Net_Income / Max_Net_Income | 20 | ✓ |
| M3 | M3 = 2 + N_(laps × banner length / RAC) / Max_(same) | 20 | ✓ |
| RAC | RAC = 0.05 WS + 0.75, WS in ft (span measured to the nearest inch, recorded in ft to 2 places); minimum RAC 0.90 (3 ft span) | 10 | ✓ |
| Banner | length ≥ 10 in, length/height ≤ 5; recorded in inches rounded down to ¼ in; stowed externally for take-off; remotely deployed and released; up to two banners at tech inspection | 12 | ✓ |
| Banner timing | deployed "after the first upwind turn … on the down wind leg"; released after the finish line on the last lap | 20 | ✓ |
| Span | maximum 5 ft | 10 | ✓ |
| TOGW | < 55 lb | 13 | ✓ |
| Energy | total propulsion energy ≤ 100 Wh; each pack ≤ 100 Wh; one pack per propulsion system | 16 | ✓ |
| Fuse | ≤ 100 A and ≤ capacity × C | 15–16, 24 | + (same as 2024-25) |
| Payload capacity | single passenger compartment and ≥ 1 cargo bay; **capable** of ≥ 3 passengers per piece of cargo **declared**; capable of ≥ 3 passengers and ≥ 1 cargo | 10–11 | **+ these are capacity rules.** The flown M2 load may be any number up to the declared maximum ("passengers and/or cargo", p. 19), even no cargo. The model ties the flown counts to the capacity (n_p ≥ 3 n_c, n_p ≥ 3, n_c ≥ 1 on the flown counts): conservative, stated |
| Duck | 2.3 in rubber duck, 2.0–2.3 × 2.0–2.5 × ≤ 2.5 in, **0.6–0.7 oz** | 10–11 | + [HB] Table 36's 1.84 lb / 42 = 19.9 g = 0.70 oz: the top of the rule range |
| Puck | standard hockey puck, 3 in × 1 in, **6 oz** (170.1 g) | 11 | + [HB] 5.25 lb / 14 = 170.1 g ✓ |
| Take-off | ground rolling, **no limit on take-off distance** | 18 | ✓ |
| Configuration | same configuration for all three missions unless cited; prop may change per attempt | 13, 18 | ✓ motor fixed, props may change |
| Packs between missions | **allowed**: FAQ "cannot change the hardware … unless specifically allowed … (i.e., batteries, payloads, payload support)" (p. 33); other packs may be inspected for "real time power change requirements" (p. 24) | 24, 33 | + so the M2 pack capacity is a per-mission DV, and it sets EF |
| Course | turn markers 500 ft each side of the start line; course figure: two 180° turns and a **360° turn halfway along the back (downwind) straight**; course oriented to the prevailing wind by the Flight Line Judge | 17, 18 | + the figure is the `dbf_2025_360_mid` preset, not the default `dbf_2025` (which flies the 360° right after the upwind turn). §4.4 |
| Fly-off | Textron Aviation Employees Flying Club, Wichita KS, 16–19 April 2026 | 8–9 | ✓ site Wichita, 1300 ft ([HB] Table 7) |

**[HB] differs from [R26] on the cost:** [HB] Eq. 6 (p. 7) omits EF. The rules (p. 20) are used.

## 2. Normalisation (new Q52): evidence and proposal

[RES26] publishes only normalised scores. Best M2: University of Michigan, M2 = 2.00 (M3 0); best M3: University of
Washington, M3 = 3.00. **USYD (rank 14): M2 1.54, M3 2.52, GM 0.01.** The raw values are not published.

| Source | Max_Net_Income | Max(N L / RAC) | How |
|---|---:|---:|---|
| USYD 1.54 / 2.52 with [HB] Table 43 "actual" (42 ducks / 14 pucks, 8 laps; 197 in, 4 laps), EF 0.9768 (97.68 Wh, §5.3.10), span 59 in → RAC 0.996 | **2776** (2744–2802 for M2 1.535–1.545) | **1521** (1507–1536) | back-calculation; Table 43 is pre-competition testing, so approximate |
| same, [HB] Table 1 (9 laps; 5 laps) | 3032 | 1902 | Table 1 vs Table 43 disagree (§3) |
| same, [HB] Table 42 test flights (36/12, 8 laps) | 2359 | – | |
| [HB] §4.11 self-normalised (best M2: 54/18, 8 laps, EF 1; best M3: 260 in, 6 laps, 55 in span) | 1936 | 1593 | AirCrafter's own maximum over its design space |
| Sanity check: a minimum M2 load (3 ducks, 1 puck, 3 laps, EF 1) | net 29.5 → M2 = 1.011 at 2776 | | matches the many 1.01 M2 scores in [RES26] |

**Proposed PLACEHOLDERS:** Max_Net_Income = **2800**, Max(N L/RAC) = **1500** (the USYD back-calculation, rounded). The
off-diagonal ×0.5 / ×2 runs (1400–5600, 750–3000) cover every row above.

## 3. [HB] extraction, checked against the PDF

| Input | Value | Source (printed page) | Check |
|---|---|---|---|
| span, chord, area, AR | 59 in; 14.57 in rectangular; 868 in²; AR 4 (§5.1) / 4.02 (Table 24) | §5.1 p. 36; Table 24 p. 26 | **flag:** 59 × 14.57 = 859.6 in² ≠ 868; Table 24 prints c̄ 1.21 ft (14.52 in) and b 4.92 ft, S 6.03 ft² (= 868 in²) |
| airfoil, incidence, panels | S3016 (Table 24; p. 24–25) vs "SD3016" (§5.1 p. 36) (Q51); i_w 1.0° (Table 24, §4.5.2 p. 25); end-plates (§8.2.1 p. 58); modular 2-piece wing (p. 26) | | ✓ |
| tails | S_ht 230 in² (HS 30.7 × 7.5 in, AR 4, SD8020); S_vt 90.9 in² (8.6/7.1 in, 11.9 in span, AR 1.5); l_t 1.1 m; V_H 0.80, V_V 0.08 | §5.1 p. 36; §4.5.3 pp. 26–27 | ✓ |
| fuselage | length 70.5 in, max width 7.8 in, max height 5.5 in (§5.1); passenger bay 39 × 7.7 in (§5.1) = "3 ft 3 in long, max width 7.7 in" (§4.2.2 p. 22); cargo bay 23.5 × 6.1 in, 2 bays of 7 pucks below the passengers (§4.2.1 p. 21, §5.1); Fig. 9 (p. 24): payload box 2 ft 9.4 in, nose 1 ft 7.7 in (1.3 MAC), tail 3 ft 7.3 in (3 MAC), 5 ft 1 in overall | | **flag:** passenger bay 39 in > payload box 33.4 in (specification); **flag:** 5 ft 1 in (Fig. 9) vs 70.5 in (§5.1); 1.3 MAC = 18.9 in vs 19.7 in |
| propulsion | AT7215-245KV, Hargrave microDRIVE LP (60 A cont.), 8S LiPo; 19×12 flown in §5.3.10; Table 42 flights 19×10 and 19×12 | Table 27 p. 29; §4.6.1 p. 28; Table 42 p. 58 | **flag: 3380 mAh** (§4.8.3 p. 30; your table) gives 100.05 Wh at 29.6 V, over the 100 Wh rule; **3300 mAh / 97.68 Wh** (GNB 8S, §5.3.10, §5.3.12 p. 42) is used |
| masses | Table 36: fuselage 6.14, HS 2 × 0.21, VS 0.28, wings 2 × 0.79, motor 1.27, ESC 0.24, empty 9.93; battery 1.22; passengers (42) 1.84; cargo (14) 5.25; banner with deployment 1.21 lb | p. 44 | ✓ (Q47 issues stand: "Mission 1" row 1.22; M2 18.54 vs 18.24 from rows) |
| drag build-up | C_D0 M1 0.0413, M2 0.0430, M3 stowed 0.0480, deployed 0.3600 (banner 0.3025 on wing area) | Table 29 p. 33 | ✓ |
| static margin | W&B: CG 3.18–3.54 in, SM 13.0–10.0 % (§5.4 p. 43); XFLR5 M1 17.6 %, M2 15.3 %, M3 15.2 % (§4.10.1 p. 34); P1 20 % "nose-heavy" (§8.2.1 p. 57); P2 12.5 % (§8.2.1 p. 58); design band 7.5–15 % MAC (Table 5 p. 9) | | ✓ |
| rpm limit | N_limit = 145,000 / D × 0.85 (Eq. 10, p. 28) | | **[APC] prints 150,000 / D for Thin Electric props** (§6) |
| motor mass | 1.27 lb (Table 36) | | [TM7] 555 g = 1.22 lb incl. cable (mount?) |
| site | Wichita 1300 ft (Table 7 p. 12); average wind ≈ 16.9 ft/s (5.15 m/s, NWS normals, p. 32) | | |

**Flight data** ([HB] Table 42 p. 58), as in the specification ✓ (M2 17.9 lb, 36/12, 19×10: 28 A, 38.8 s; 19×12: 31 A, 33.2 s;
M3 19×10: 118/157/197 in at 12.2/12.4/12.8 lb, 32/32/35 A, 62/56/55 s).

**Inconsistencies listed, not resolved (new Q57):** Table 1 (p. 4) vs Table 43 (p. 59): M2 laps 9 vs 8; M3 laps 5 vs
4; M3 speed 49.9 vs 73.3 ft/s (Table 1's 49.9 is Table 43's *design* speed); Table 1 M2 MTOW 17.5 lb vs Table 42
17.9 lb. Table 33 (42/14, 9 laps; MTOW 14.3 lb) vs Table 43 design (30/10, 9 laps, 275 in). §4.8.4 assumes a 13.2 lb
M2 MTOW, Table 33 14.3 lb, Tables 1/42 17.5–17.9 lb; §4.6 prop selection is at 55 lb MTOW. Battery 3380 vs 3300 mAh.
Spar sizes (p. 26 text, Table 24, p. 37) and the Table 36 issues (Q47). Table 41 says "all tests used 118 in banners"
but lists C_D for six lengths. Cost Eq. 6 without EF.

## 4. Model changes (B2–B5a)

### 4.1 Scoring plug-in `dbf2026` (`aircrafter2/scoring/dbf2026.py`)

    F = M1 + M2 + M3,  M1 = 1 (M1 check, analysis only)
    NetIncome = n_p (6 + 2 N2) + n_c (10 + 8 N2) − N2 (10 + 0.5 n_p + 2 n_c) EF,  EF = E_pack_M2[Wh] / 100
    M2 = 1 + NetIncome / NI_max
    M3 = 2 + (N3 L_banner[in] / RAC) / B_max,  RAC = 0.05 b[ft] + 0.75
    N2 = laps_frac of the M2 point, N3 = laps_frac of the M3 point (the continuous surrogate)

- E_pack_M2 = rated capacity × rated voltage (n_s × 3.7 V) of the M2 pack: the capacity DV sets EF (rules p. 16, 20).
- RAC ≥ 0.90 is the span lower bound b ≥ 3 ft (constraint, inactive unless the span collapses).
- **Reported at every result:** F continuous, and F with N2, N3 floored to completed laps (and L_banner rounded down
  to ¼ in, the span to the nearest inch, as the rules record them).
- Weights (w2, w3) for the bookends; NI_max, B_max PLACEHOLDERS (§2).
- GM not modelled (new Q53): it measures the crew, and its time grows with the payload count ([HB] §3.3, p. 10;
  USYD's GM was 0.01 in [RES26]). Stated in every table.

### 4.2 Payload (M2) and fuselage sizing (`stores.py`: a `payload2026` store set)

- **DVs:** n_p, n_c continuous, M2 only. Constraints n_p ≥ 3 n_c, n_p ≥ 3, n_c ≥ 1 (rules p. 10–11). Search bounds
  n_p ≤ 90, n_c ≤ 30 (about 2× Humpback; PLACEHOLDER, reported if active).
- **Masses:** duck 19.9 g ([HB] Table 36; rules 0.6–0.7 oz), puck 170.1 g (rules 6 oz = [HB]). Restraint and foam
  PLACEHOLDER (new Q54): 5 g per passenger seat and 10 g per puck slot ([HB] §5.3.8–5.3.9 describe EPE floor
  recesses and peeled foam; no masses given). Fixed in M1/M3 (the restraints stay in the aircraft, rules p. 10–11):
  only the ducks and pucks are removed.
- **Final design:** round to integers (n_c down and up, n_p = max(3 n_c, round(n_p)), plus both counts rounded
  down), re-optimise the continuous variables with the counts fixed, report the better. **2026-11-02:** only the
  candidates that fit the payload fuselage and the passenger ratio at the continuous optimum (`fuselage_margin` ≥ 0,
  `pax_margin` ≥ 0, `rounding_check` in `studies/results_2026.py`) are re-optimised: with the span at its bound the
  fuselage length is set by the counts alone, and rounding 96.92 passengers up made both of the old rule's cases
  infeasible for every design (docs/development/MDA_ROBUSTNESS_PLAN.md, status).
- **Fuselage sized by the payload** (the cargo box = the constant-section payload fuselage, calibration convention;
  PLACEHOLDER derived from [HB], new Q55):

      l_pax = 39 in × n_p / 42      (rows of 3 abreast; [HB] passenger bay 39 in for 42 ducks, §5.1)
      l_car = 23.5 in × n_c / 14    (2 bays of 7, side by side; 23.5 in for 14 pucks incl. the extraction slack)
      l_cb  = KSmax(l_pax, l_car)   (pucks below the passengers, [HB] §4.2.1, Fig. 9)
      w_cb  = 7.8 in, h_cb = 5.5 in ([HB] §5.1 maximum width and height: the cross-section is held)

  At Humpback's counts l_cb = 39 in (the passenger bay), against Fig. 9's 33.4 in payload box: flagged. The nose
  (1.3 MAC) and tail arm (3 MAC) stay as they are in the model (l_ht is a DV).
- The calibrated cargo-box factor f_cb was fitted on [OH] and [W]'s fuselages; Humpback's fuselage was never in the
  fit (Q46), so the payload fuselage's mass is an extrapolation in size. Stated.

### 4.3 Banner (M3)

- **DV** L_banner ∈ [10 in (rule), 275 in] (upper: Humpback's longest team-built banner, [HB] Table 34 p. 41;
  PLACEHOLDER, new Q56; AirCrafter 1.0's grid goes to 500 in, Table 7). Height H = L/5 (aspect ratio 5, the rule
  limit).
- **Drag** D = q C_D,b L H on the banner's own area. **C_D,b nominal 0.024, spread uniform 0.019–0.028**: the mean and
  range of [HB] Table 41's length column (ripstop, LE-only reinforcement: 0.028, 0.021, 0.020, 0.026, 0.028, 0.019 at
  118–275 in; non-monotonic). Disagreeing values (new Q56): flat plate C_D 1.2 (§4.8.5 p. 30); Table 29's deployed
  ΔC_D 0.3025 on the wing area (= C_D,b 0.034 on a 197 in banner's 53.9 ft², or 1.2 on 1.5 ft²); AirCrafter 1.0's
  0.03 on L²/5 (Table 6). At 197 in, C_D,b 0.024 is ΔC_D = 0.214 on the wing area.
- **Stowed:** ΔC_D0 +0.004 ([HB] Table 29 undercarriage 0.0050 → 0.0090) from take-off to deployment.
- **Lap model:** lap 1 is stowed until the start of the first downwind leg (after the first upwind turn), deployed
  from there (the 360° turn on the downwind leg is flown with it), and every later lap is deployed. `leg_breakdown()`
  gets a `banner` column (stowed / deployed / none).
- **Mass** m_b = ρ_A L H + m_ballast + m_mech: ripstop ρ_A = 0.22 oz/ft² ([HB] p. 19, cited); ballast 0.3 lb
  ([HB] §5.3.7 p. 41); m_mech by difference from Table 36's 1.21 lb at 197 in: fabric 0.74 lb + ballast 0.30 lb →
  **m_mech = 0.17 lb** (deployment and release plate, hammock, bridle, towline, LE webbing; the split is stated, Q56).
  Carried for the whole M3 flight (stowed, then towed: the tow line's share of the banner's weight is ignored,
  conservative).

### 4.4 Aircraft, propulsion, site, course

- Single nose tractor, one pack. **Modelling differences, stated:** rectangular wing with end-plates (end-plates not
  modelled); the flaps (C_L,TO) not modelled beyond `CL_ground`; taildragger as before.
- **Motor:** AT7215 245 KV reference (Rm 20 mΩ, I0 2.4 A at 10 V, 555 g incl. cable; [TM7]), rewound to the design Kv.
  **Kv ∈ [200, 270]**: the family's four catalogue windings (200, 220, 245, 270; [TM7]): cited, not PLACEHOLDER. The
  catalogue gives R ∝ Kv^−1.5 and constant I0 (rewind model: Kv^−2, Kv); the rewind error at 200/220/270 is reported
  as for the V3115. ESC Hargrave microDRIVE LP, 60 A continuous ([HB] §4.6.1). Fuse ≤ min(100 A, C × capacity).
- **Cells** {6S, 8S}: 8S as flown (3300 mAh, 97.68 Wh); the AT7215 245 KV is rated 8–10S [TM7], so 6S needs Kv ≈ 330
  for the same rpm, above the family's 270: **6S will be power-limited**, reported as such.
- **Props:** M2 {18x10E, 19x10E, 19x12E} × M3 {18x12E, 19x12E, 22x12E} at 8S (9 pairs), the best pair at 6S. All five
  are in PERFILES2; their data reach 38–50 m/s at the rpm block below APC's limit (18x10E 42.8, 19x10E 38.1, 19x12E
  44.1, 18x12E 49.9, 22x12E 38.8 m/s), against Humpback's 31 m/s M2 and 15–22 m/s M3: `v_fit_margin` is checked at
  every optimum.
- **Site and course:** Wichita, 1300 ft. The rules' course figure (p. 17) is exactly the shipped `dbf_2025_360_mid`
  preset (500 ft head, 180°, 500 ft tail, 360°, 500 ft tail, 180°, 500 ft head); it is used for every 2025-26 run, so
  no `dbf_2026.yaml` is needed. (The 2024-25 study keeps `dbf_2025`.)
- **Placement:** rule-dependent values in `configs/rulesets/dbf_2025_26.yaml`; universal values (AT7215, ESC, duck
  and puck masses, ripstop density, banner C_D,b) in `configs/placeholders.yaml` under the layering rules; problem
  configs `configs/dbf2026_multipoint.yaml`, `configs/dbf2026_multipoint_rdo.yaml`, `configs/humpback_2026.yaml`,
  `configs/mass_usyd_loo_humpback.yaml`.
- **Generalising the multipoint problem:** today `problems/multipoint.py` hard-codes the 2024-25 stores (tanks,
  pylons, X-1, `fuel_mass`) and `dbf2025`. The ruleset will name its store set and scoring plug-in, with the
  2024-25 path unchanged (its references bit for bit).

### 4.5 Constraints (B4)

| Constraint | Old set (sensitivity) | Relaxed set (main) | Source |
|---|---|---|---|
| rpm | 0.85 × 150,000/D | **1.0 × 150,000/D** | [APC] Rev 5: Thin Electric props 150,000/D. [HB] Eq. 10 uses 145,000/D × 0.85; APC's sheet does not print 145,000 |
| SM lower | 0.05 | 0.05 (unchanged) | Q9 |
| SM upper | 0.20 | **decision (§9 Q4)** | reports: [HB] 10–13 % W&B, 15.2–17.6 % XFLR5, P1 20 % "nose-heavy"; [W] 9.55–11.09 % (AVL, p. 29, p. 44); [OH] 10–15 % allowable, 12.5 % design (pp. 29–30). **0.20 covers every reported value** |
| V_h upper | 0.65 | **0.80** | the largest flown: [HB] 0.80; [W] 0.722; [OH] 0.66 (Table 24 p. 22) |
| V_v upper | 0.06 | **0.08** | [HB] 0.08; [OH] 0.072; [W] 0.059 |
| V_h, V_v lower | 0.35, 0.02 | unchanged | Q9 |

**Woombah's modelled SM (finding).** The calibrated common evaluation gives M2 0.23 and M3 0.54.
- **M3 0.54 is an artefact of the battery station.** `woombah_2025.yaml` takes x_batt_M3 = −29.37 in from [W] Table 34,
  but that table's own M3 CG (4.82 in) is inconsistent with it: Table 34's rows put the M3 CG at −0.1 in with the
  pack at −29.37 in, and at 4.3–4.4 in with it at −0.59/−0.83 in. With the M2 station (−0.83 in) the modelled M3 SM is
  **0.19** (0.188 at M1's −0.59 in). The 2024-25 pages keep the published station (Phase A is frozen); a new Q58 records
  it.
- **M2 0.23 is mostly the model's neutral point.** The model puts Woombah's NP at 0.208 m aft of the LE (63 % of the
  0.328 m equivalent chord); [W]'s eCalc NP is at 45.8 % MAC (p. 28), ≈ 0.151 m. The CGs agree within 0.01 m (model
  0.133 m vs Table 34's 4.83 in = 0.123 m). So the model's SM reads about +0.13–0.17 high against the teams' tools
  (rectangular equivalent wing, tail efficiency and downwash model, fuselage terms). Humpback is checked the same way
  at stage 2.
- So a bound taken from the reports (0.20) and a bound on the model's own SM are not the same thing (§9 Q4).

### 4.6 Take-off on, after the Q45 fix (B5)

- **Q45 fix:** lap 1's first straight (the in-air remainder after the ground roll; from V_stall with take-off off) is
  integrated on a grid clustered at its start, s_i = L (i/N)² (N = 40), instead of a uniform one; every other leg
  keeps the uniform grid. Error table against a uniform 640-step reference at wind 0–8 m/s (0.5 m/s steps) for a
  light (the 2024-25 M3 optimum) and a heavy design (multipoint_2L, M2), take-off off and on. If the quadratic grid
  is not enough, the fallback is a cubic one or 80 steps on that leg.
- **References:** regenerated in their own commit, listing which change and by how much. Only lap-1-derived numbers
  (Lap1time and what depends on it: ThreeLapTime, laps_frac, the scores) can move.
- **Settings:** take-off on for every 2025-26 run, with `CL_ground` 0.15 (Q40), `k_lof` 1.1 (Q41) and the point
  throttle (Q42; Humpback takes off at full throttle, [HB] §4.8.1 p. 29: stated).
- **Wind range (proposal):** uniform **0–10 m/s** (was 0–7). Evidence: Wichita April mean 5.6 m/s (Q39) and [HB]'s
  5.15 m/s (NWS normals, p. 32); a 0–10 uniform has mean 5.0. The slow M3 banner aircraft: Humpback's M3 cruise ≈ 15
  m/s (55 ft/s, §4.8.5) and V_stall ≈ 11.8 m/s at 12.4 lb (C_L,max 1.2): at 10 m/s the M3 downwind leg is still
  flown, the upwind ground speed is ≈ 5 m/s. Lap 1 starts at V_LOF = 1.1 V_stall with take-off on, so the validity
  edge is w < V_LOF (wind limit, counted separately). The cap is confirmed (or lowered) after the Q45 fix shows where
  the 40-step integration stays within 0.5 % of the 640-step one.

### 4.7 Wind direction (B5a)

- **What the lap model does now:** a steady wind along the course axis; straights get ±w by their `wind` sign; **turns
  ignore wind entirely** (no drift; the leg table shows no wind for turns), and turn time is set by the airspeed.
- **Straights:** along-track component a = w cos θ, crosswind c = w sin θ; ground speed
  v_g = v √(1 − (c/v)²) ∓ a (crab to hold the track); energy and power on airspeed. Written so that at θ = 0 it is
  `v·1.0 − sign·w·1.0`, bit for bit the current expression.
- **Turns:** flown at constant airspeed in the air-mass frame (turn time unchanged). The drift w·t_turn (vector) is
  carried to the next straight: its along-course part shortens it if the drift is in the next straight's direction
  of flight and lengthens it otherwise, and its cross-course part d is recovered on that straight, which is flown as
  the hypotenuse √((L − δ)² + d²). Smooth in w and θ.
- **Conflict with the specification (§9 Q1):** at θ = 0 the turn drift is not zero: the along-course part is w·t_turn. So "the
  current model at θ = 0" and "turn drift" cannot both hold when w > 0. Every reference at w = 0 is unchanged either
  way; the 2024-25 Monte Carlo and RDO runs (w > 0) would move. Proposal: a switch `course.turn_drift` (off by
  default, so every 2024-25 config and reference is bit for bit; on in the 2025-26 configs).
- **Take-off:** the ground roll uses the along-runway component w cos θ only; crosswind take-off limits not modelled
  (new Q60).
- **Wind limit vs θ:** for an airborne aircraft the upwind ground speed is positive iff v > w, for every θ (v² − c²
  > a² ⟺ v² > w²), so the lap model's limit barely depends on θ; the take-off limit becomes w cos θ < V_LOF. The
  envelope (limit against θ, light and heavy design) is reported as computed.
- **Distribution:** θ ~ uniform on [−45°, 45°], **PLACEHOLDER (new Q59)**, independent of w. Evidence searched: the
  Flight Line Judge lays the course out to the prevailing wind (rules p. 17); hourly direction climatology (e.g.
  "predominantly from the south" in April) describes the prevailing direction, not its shift within a 5-min flight
  once the course is set; no DBF source says how often the course is reset. So no cited value.
- **Smoothness and checks:** θ enters through cos θ, sin² θ (even in θ), so F and the lap time are symmetric in ±θ;
  `check_partials` in w and θ including θ = 0; a pure-crosswind straight against √(V² − w²); a turn's drift against a
  hand calculation.
- **UDR check:** with θ symmetric and F even in θ, UDR's 3 Gauss–Legendre points per parameter (0, ±0.775 θ_max)
  integrate F(θ) exactly up to degree 5; compared with Smolyak level 2 and the Sobol' Monte Carlo in the propagation
  check.

## 5. Humpback as built (B6)

`configs/humpback_2026.yaml` (§3 values; 8S 3300 mAh; 19×10E / 19×12E as flown; Table 42 masses), mass config
`configs/mass_usyd_loo_humpback.yaml` fitted on [OH] and [W] only (calibration doc §5), so the wing and tails stay
out-of-sample; its fuselage and gear were never in the fit. Pages as `validation_2025.md`: analysis envelope; mass
against Table 36 (out-of-sample calibrated column, uncalibrated G&G column); flight performance against Tables 42,
43 and 1 at the logged current (throttle solved to the average current: an input, not a fit) and logged mass;
banner-drag check (model M3 lap time vs the three M3 tests at their currents, nominal C_D,b; reported, not tuned);
AirCrafter 1.0 at Humpback's design point. Q47's Table 36 issues are carried as they are.

## 6. AirCrafter 1.0 `2026` mode (B7): inventory

| Shipped file: line | Content | `2026` mode |
|---|---|---|
| `AirCrafter.m` 49–53 | `inputs.bannerL = 0`, `inputs.batteryFraction = 0` | grid axes: battery fraction 0.4–1 (M2), banner 25–500 in (M3) ([HB] Table 7) |
| `AirCrafter.m` 170–180 | M3 mass = m (1 − PF) (empty), banner matrix | M3: empty + banner (the shipped `performance_modeller` adds the banner mass) |
| `AirCrafter.m` 241–256 | 2025 maxima and scoring | [R26] M2 net income and M3 (N L/RAC), NI_max and B_max as AirCrafter 2.0 (§2); payload as bundles of 3 ducks + 1 puck from the payload fraction (floor; [HB] §4.11); EF = battery fraction (capacity f × 100 Wh, usable f × 80 Wh) |
| `performance_modeller.m` 55 | E = batteryFraction × E | M2 fix on (fraction = the grid value) |
| `performance_modeller.m` 73–84 | banner mass L²/5 × 0.17 kg/m², NaN if m > m/PF | kept (shipped) |
| `propSelector.m` 56–62 | banner mass | kept |
| `propSelector.m` 97–108 (the specification's "54–83") | cargo 0.17 kg, passenger 0.02 kg, groups of 3 + 1, fuselage-bay drag C_D 0.05 | kept |
| `propSelector.m` 110–114 | banner drag C_D 0.03 × q × L²/5 | kept |
| `inputs_file.m` | grid, take-off distance, window, altitude | [HB] Table 7: mass 7–55 lb, span 3–5 ft, AR 4–9, V 33–148 ft/s, battery fraction 0.4–1, banner 25–500 in; TO 350 ft, 4.5-min window, 80 Wh, 1300 ft |
| `assumptions_file.m` | C_L,max, C_D0, … | [HB] Table 6: C_L,max 1.2, C_L,TO 1.3, C_D0 0.05, η 0.70, n 5, PF 0.4, e 0.85, μ 0.02 |

**New finding (M15):** in the shipped code the banner and fuselage-bay drag enter **only** `propSelector` (prop
choice), not `performance_modeller`'s lap time, so the M3 lap count does not see the banner's drag and AirCrafter
1.0 would pick the longest banner on its grid. §9 Q2.

Bug fixes M2/M5/M7/M8/M13 on, as in `2025` mode; `original` must still reproduce the shipped baseline exactly
(`check_original.m`).

**Sweep time:** at the shipped steps (0.5 kg, 0.1 m, 0.5, 1 m/s; battery fraction 0.1, banner 25 in) the grid is
44 × 7 × 11 × 36 = 122k points with 7 M2 and 20 M3 calls each ≈ 3.3 M calls × 3.6 ms ≈ **3.3 h**. Over the 1 h budget,
so **proposed:** battery fraction step 0.2 (0.4, 0.6, 0.8, 1.0), banner step 50 in (25–475), mass step 1 kg:
22 × 7 × 11 × 36 × 14 ≈ 0.85 M calls ≈ **51 min**, with the local 2× refinement check around the selected point as
in compute_performance §3.

Mappings: (a) direct, (b) completed (span, chord, payload counts and banner fixed, the rest optimised).

## 7. Uncertainty and RDO (B8)

- **Set (d = 5):** f_build ±30 % (cited, USYD LOO), f_tb ±30 % (PLACEHOLDER), m_avionics ±15 %, wind speed 0–10 m/s
  (§4.6), wind direction ±45° (§4.7). UDR N = 2d + 1 = 11 nodes × 2 points; k = 1, β = 2; robust constraints as
  published.
- **Banner-drag factor (not added; open decision, §9 Q5).** Hand estimate (the model value comes at stage 2, from
  the screening at the smoke-test optimum): the banner is ≈ 80 % of M3's drag at C_D,b 0.024 (ΔC_D 0.21 on C_D0 ≈
  0.05); M3's laps are energy-limited, so σ(N3)/N3 ≈ 0.8 × σ(C_D,b)/C_D,b = 0.8 × 0.11 = 0.09 (uniform 0.019–0.028:
  σ = 0.0026), and with M3's variable part ≈ 0.5 (USYD 2.52), **σ_F ≈ 0.04**. For comparison, the 2024-25 first-order
  σ_F were wind 0.009, f_build 0.0026, m_avionics 0.0005, f_tb 0.0003. **Recommendation:** add it as the fifth
  mass/drag parameter **in place of f_tb** (the smallest 2024-25 contribution, and a PLACEHOLDER spread), unless the
  stage-2 screening shows f_tb larger on the payload-sized fuselage.
- **Out-of-set check:** Monte Carlo of the RDO design over the extended list (motor Kv and Rm, thrust factor, banner
  drag (if not in the set), cell resistance), 1024 Sobol' points.

## 8. Run list against the 30 Oct freeze

Basis (calibrated 2024-25 runs, 3 threads): deterministic multipoint 4–7 min (60 iterations); RDO d = 4 30–46 min per
start; Monte Carlo 1024 × 2 points 13–20 min per design (5 workers). For 2025-26: +3 DVs (n_p, n_c, L_banner) and
take-off on → ≈ ×1.5 deterministic; d = 5 → 11 nodes ≈ ×1.3 RDO. Parallelism: 3 workers × 3 threads (memory, ≈ 1.3 GB
per worker, limits it).

| Item | Runs | Est. run time | Est. wall |
|---|---:|---:|---:|
| Enumeration: 9 pairs at 8S + best pair at 6S | 10 | 10 × 10 min | 0.6 h |
| Integer payload rounding + re-optimisation | 2 | 20 min | 0.2 h |
| Multistart (10) | 10 | 100 min | 0.6 h |
| Bookends | 2 | 30 min | 0.3 h |
| Normalisation off-diagonal | 4 | 20 min | 0.2 h |
| Sensitivities (old constraints, take-off off: re-opt; C_D,b low/high/0.03, rpm 0.85: analyses) | 2 + analyses | 30 min | 0.3 h |
| AirCrafter 1.0 `2026` sweep + mapping (b) | 1 + 1 | 51 + 10 min | 1.0 h |
| Screening + node check | – | 20 min | 0.3 h |
| RDO, 3 starts | 3 | 3 × 60 min | 1.0 h |
| Common evaluation: nominal (5) + Monte Carlo (5 designs + out-of-set) + propagation check | 5 + 6 MC | 6 × 20 min + 20 | 2.3 h |
| Wind envelope (2 designs × 9 θ, bisection) + F vs θ (5 designs × 9 θ) | analyses | 30 min | 0.5 h |
| Computational performance (single-threaded, clean tree; AC1 sweep, AC2 multipoint, RDO to convergence), 3 repeats | 9 | ≈ 3 × 2.5 h | 7.5 h |
| **Total** | | | **≈ 7 h + 7.5 h timing** |

**Schedule:** stage 2 (build, references, validation, AC1 mode, smoke test) Tue 27 Oct; runs overnight Tue–Wed
(stage 3 Wed 28); timing repeats overnight Wed–Thu; pages, exhibits and report Thu 29; freeze Fri 30. **If the
stage-2 build runs over,** the first cuts are the specification's: normalisation runs → re-analysis; timing repeats → one
clean run each (saves ≈ 5 h).

## 9. Open decisions

1. **Turn drift vs "exactly the current model at θ = 0"** (§4.7): a `course.turn_drift` switch, off for 2024-25 and on
   for 2025-26 (recommended); or always on (2024-25 Monte Carlo/RDO numbers move); or no turn drift (θ only on the
   straights).
2. **AirCrafter 1.0 M15** (§6): fix it in `2026` mode (banner and bay drag in the lap model's drag polar, behind a
   switch; recommended, otherwise its M3 ignores the banner's drag) or keep the shipped behaviour and report it.
3. **Normalisation PLACEHOLDERS** (§2): NI_max 2800, B_max 1500.
4. **SM upper bound** (§4.5): keep 0.20 (covers every reported value; Woombah and possibly Humpback stay outside it
   in the model's own SM), or set it from the modelled as-flown USYD aircraft (Woombah M2 0.23 with the M3 station
   corrected; Humpback from stage 2), rounded up.
5. **Banner-drag factor** (§7): add it in place of f_tb (recommended), or keep the five as decided.
6. **Wind range** (§4.6): 0–10 m/s (recommended) or keep 0–7.
7. **Course** (§4.4): the rules' figure = `dbf_2025_360_mid` (recommended), banner deployed at the start of the
   downwind leg (so the 360° is flown with it).
8. **Payload capacity reading** (§1): flown counts = capacity, n_p ≥ 3 n_c on the flown counts (recommended,
   conservative), or let the flown counts drop below the capacity rule (more DVs).
9. **Calibration LOO** (§0): investigate why [OH]'s LOO moved (−26 → −34 %) before the 2025-26 runs and keep
   ±30 % if the published value is reproduced, or adopt ±35 % (the committed code's value) for the 2025-26 study.

### Decided at stage 1 (2026-10-26)

1. Turn drift behind a switch `course.turn_drift`: off by default (every 2024-25 config and reference bit for bit),
   on in the 2025-26 configs.
2. **Uncertain set (d = 5):** f_build, **banner-drag factor C_D,b (uniform 0.019–0.028, [HB] Table 41) in place of
   f_tb**, m_avionics, wind speed, wind direction; confirmed against the stage-2 screening (f_tb's contribution
   on the payload-sized fuselage is reported). f_tb moves to the out-of-set Monte Carlo.
3. **SM upper bound from the modelled as-flown USYD aircraft:** the largest modelled SM of Woombah (with its M3
   battery station corrected to Table 34's M2 station) and Humpback (stage 2), rounded up to 0.01.
4. **Calibration LOO:** investigate first (about an hour at the start of stage 2); keep ±30 % if the published
   −26 % reproduces, else adopt the committed code's value and regenerate the calibration page.
5. Recommendation: AirCrafter 1.0 M15 fixed in `2026` mode behind a
   switch; NI_max 2800, B_max 1500; wind 0–10 m/s; course `dbf_2025_360_mid` with the banner deployed at the start
   of the downwind leg; flown payload counts = capacity.

## Progress review (2026-10-27)

**Built** (commits `f6f5432`, references `9d3e02d`): `dbf2026`, the payload / banner / payload-fuselage stores, the
multipoint problem by ruleset kind, the wind vector with turn drift behind `course.turn_drift`, the banner's
deployment in the lap model, the Q45 fix, AirCrafter 1.0 `2026` mode (M15 fixed), `studies/results_2026.py`,
`validation_2026.py`, `q45_resolution.py`; tests (`tests/test_results_2026.py`). Decided during the build:

- **Calibration LOO** (Q9 of §9): the fold fit did not hold the unidentified factors (f_tb) at their G&G values, so a
  rerun leaked the calibrated f_tb (fitted with [OH]) into [OH]'s own fold. Fixed: [OH] LOO −25.8 % (published
  −26.3 %), f_build stays ±30 %, the combined factors are unchanged.
- **SM upper bound 0.47** (decision 2026-10-27): Humpback's modelled SM is 0.39–0.46 (Q62).
- **`propulsion.f_fit` 0.95** in the 2025-26 configs (0.8 default kept): Humpback's props cruise at J ≈ 0.9 J_max;
  at 0.8 the M2 cruise lay past the fit nodes (T 5 % low there) and every start point violated v_fit_margin; at 0.95
  the top node stays inside the APC data (J 0.762 < 0.783, 19x12E) and T at cruise is within 1.4 %.
- **Start throttle 1.0 in M3**: a cold M3 MDA at part throttle can fail; warm starts converge down to 0.5.
- **Q45**: clustered lap-1 grid, ≤ 0.03 % against 640 steps up to the validity edge (old grid: up to 3 %).
- **AirCrafter 1.0 cost**: 0.43 ms per call (not 3.6 ms), so the coarse grid of §6 took 6 min; the shipped steps
  (0.5 kg, battery fraction 0.1, banner 25 in) cost about 25 min.
- **Woombah's M3 station** (Q58) recorded, not changed (Phase A frozen).

## Placeholder fixes before the final run (2026-11-01/02)

Branch `placeholder-fixes-2026` (configs `0f27643`, validation page `a2db0f4`), from the review "AirCrafter 2.0
Placeholder Ranges" (30 Oct 2026). All values are in `configs/rulesets/dbf_2025_26.yaml`, so the 2024-25 and
single-point configs are unchanged. This supersedes the SM lower bound of §4.5 (0.05) and the ±45° wind direction of §4.7 and §7.

1. **Energy reserve counted once:** `energy_reserve` 0.2 → 0.0. `usable_fraction` 0.8 already holds Humpback's
   single 20 % margin (80 Wh of 100 Wh, [HB p. 12 Table 7]); 0.2 on top flew 64 % of the pack, now 80 % (Q24).
2. **SM lower bound in the model's frame:** `stability.SM_min` 0.05 → 0.25. A real 5–10 % floor plus the model's NP
   offset of ~0.2 c aft of eCalc (Q9, Q62); `SM_max` 0.47 was already in the model's frame.
3. **Sourced values:** `multipoint.rules.c_rating` 60 → 100 (GNB 8S 3300 mAh 100C, Q8); `propulsion.m_prop`
   0.086 → 0.098 kg (APC LP19012E, Q61); `stability.tc_tail` 0.10 → 0.101 (SD8020, UIUC coordinates); `takeoff_distance`
   106.68 m unchanged, now cited ([HB p. 12 Table 7]).
4. **Wind direction ±45° → ±30°** (σ 17°, inside the daytime Pasquill classes B–D; Woodward 1998; Q59). The UDR
   direction nodes move from ±34.9° to ±23.2°.

Effect on Humpback as built (calibrated, validation_2026.md): F 5.086 → 5.323, M2 laps 6.96 → 8.76 (8 flown), M3 laps
4.57 → 5.69, SM 0.389/0.421 → 0.391/0.425. Found while checking (open):

- **The cold-start smoke run failed. Resolved 2026-11-02** (docs/development/MDA_ROBUSTNESS_PLAN.md). Cause: at the low-speed
  M3 banner designs the T = D balance has a second, sub-stall root, and Newton from a start far from the real root
  diverges (SLSQP's first step, Kv 245 → 200 and M3 throttle 1.0 → 0.795; `energy_reserve` 0.0 only changed the path).
  Fix, under the problem-config switch `mda_robust` (on in `dbf2026_multipoint.yaml` only): a continuation fallback
  in the MDA's Newton solver, and a warm path for `--start` / `integer()`. The smoke now converges (`mr_smoke`: 58
  iterations, F 6.1723); `--start` and `integer()` (96p32c, F 6.1675) converge too. Every 2024-25 and single-point
  result is unchanged (the switch is off there; configs snapshot bit for bit).
- **Humpback's M1 post-check (USYD factors without Humpback) failed on SM:** 0.4676 → 0.4718 > SM_max 0.47 (the
  heavier prop at the nose). SM_max was set from the M2/M3 values only. **Decided 2026-11-02, (b):** `SM_max` stays
  0.47, and for 2025-26 (`kind: dbf2026`) the M1 check fails on SM only below SM_min. Above SM_max it is reported
  (`SM_high` in the result), because a too-large margin is a trim-drag and handling cost, not a safety failure (Q9,
  Q62). The 2024-25 check keeps both bounds, as published. The M2/M3 SM constraints in the optimiser keep both bounds.
  Humpback's M1 SM: calibrated 0.4637 (pass), USYD-LOO 0.4718 (pass, SM_high), G&G 0.822 (pass, SM_high; uncalibrated
  masses). The smoke optimum's M1 SM is 0.369, inside both bounds, so the decision doesn't touch it.
- **The X-1's t/c** (`multipoint.stores.x1_wing_tc` 0.10, 2024-25 ruleset) is the same SD8020: 0.101 would apply, not
  changed (2024-25 references frozen).

## Final fixes before the final run (2026-11-02)

Branch `placeholder-fixes-2026` from `a41ad07`. Commits: `65c45a0`
(D5 and the small items), `a580942` (D1), `0cfb746` (D2), `5a27391` (D3), `7310a4f` (the 2025-26 references), `a33cb7d`
(the verification runs and the validation page), then the docs. Every 2024-25 and single-point reference is unchanged:
the configs snapshot is identical bit for bit (152 values), and the full suite passes (534 tests, 19 min).

### The fixes

- **D1, the score cap** (`scoring/dbf2026.py`). M2 = 1 + cap(NI / NI_max) and M3 = 2 + cap(T3 / B_max).
  - *Why:* the rules' Max is the best team's, so each ratio is at most 1 ([R26] p. 20). Uncapped, the payload credit
    drove the design to the fuselage-length cap: 97 passengers, l_cb 2.29 m, F 6.17 > 6.
  - *How:* cap is exactly r for r ≤ 1 − δ and exactly 1 for r ≥ 1 + δ, with a C1 quadratic blend between (bias δ/4 at
    r = 1); δ = `cap_delta` = 0.005. `floored()` applies the hard min(r, 1). The run JSON records `cap` (δ, the
    uncapped ratios, `M2_capped`, `M3_capped`).
  - *Effect:* the new optimum sits at the cap, r_M2 = 1.0028, inside the blend (M2 1.9998; the hard cap gives 2.0).
    Humpback (r 0.55) is unchanged bit for bit.
  - *AirCrafter 1.0 (not edited):* its selection is unchanged. The selected design (4.2819: M2 1.531, M3 2.750) is below
    both caps and is the maximum of the uncapped score over the 757 feasible points, so no point can outscore it once
    capped. None of the 20 stored top designs exceeds a cap (max M2 1.73, max M3 2.92).
- **D2, the rigid payload fuselage** (`stores2026.py`, 2025-26 only). `x_cb_frac` is now where the wing sits along the
  fuselage. The payload stations move with the box, and the motor group sits at the box front at Humpback's nose
  length (0.192 m): x_motor = x_motor_ref + (x_cb − x_cb_ref) − (l_cb − l_cb_ref)/2. Three new constraints:
  `batt_fwd_margin` (pack behind the motor: d_batt 0.1005 m, cited, Q64), `batt_aft_margin` (pack inside the box) and
  `tail_margin` (HT leading edge behind the box, no clearance, PLACEHOLDER, Q64). The start design is Humpback's
  layout (`initial.x_cb_frac` 0.416), and the `x_batt` lower bound is −1.5 m, so the margin governs.
  - *Why:* at `mr_smoke` the shell (about 2.0 kg) sat 0.19 m ahead of its payload (`x_cb_frac` 0), the pack was
    0.067 m ahead of the motor, and the 2.29 m box ran 0.61 m past the motor and past the HT.
  - *Effect:* Humpback as built (42/14) is identical bit for bit, M1 check included. The Table 42 flights now fly in the
    as-built 42/14 fuselage (`fuselage_counts`): the M2 tests (36/12) change only through it (l_cb 0.849 → 0.991 m;
    lap 28.59 → 28.55 s and 27.89 → 27.88 s, calibrated masses); the M3 tests are identical.
- **D3, undercarriage and leakage drag** (2025-26 only): `aero.cd_area_gear` = 0.0050 × S_ref = 0.0028 m² and
  `aero.cd_area_lp` = 0.0040 × S_ref = 0.00224 m² ([HB] Table 29, M2 column, p. 33; S_ref 868 in² = 0.5600 m², p. 36;
  Q57), constant for every design (PLACEHOLDER scaling, Q63). They are added at M1, M2 and M3; the stowed banner's 0.004
  stays (stowed M3 matches Table 29's undercarriage 0.0090), and Table 29's deployed leakage 0.005 is taken as 0.004.
  - *Why:* the parasite build-up had no undercarriage and no leakage term, and the model flew the test laps 16–48 % too
    fast. This is a missing component of the team's own build-up, not tuning: nothing is fitted to the flights.
  - *Effect (Humpback, USYD-LOO):* C_D0 M2 0.0175 → 0.0266 (Table 29: 0.0430), M3 deployed 0.2329 → 0.2421 (0.3600);
    F 5.3222 → 5.2839, M2 1.5728 → 1.5478, M3 2.7494 → 2.7361; SM 0.396/0.433 unchanged. Lap-time errors (model too
    fast), before → after: M2 −26.3 → −22.6 %, −16.0 → −14.6 %; M3 −47.9 → −46.4 %, −32.3 → −30.8 %, −21.5 → −20.2 %.
    About two thirds of the C_D0 gap at M2 remains.
- **D4, the old constraint set:** `OLD_CONSTRAINTS["stability.SM_max"]` = 0.40, the old 0.20 shifted into the model
  frame by the same +0.20 c as SM_min (Q9, Q62). *Why:* 0.20 sits below the 2025-26 SM_min 0.25, so the sensitivity
  could never be feasible.
- **D5, the robust analysis paths.**
  - `problems.multipoint.initial_values` (moved from `results_2026`, re-exported) and `run_warm(prob, mcfg_start, dv)`.
    `optimise`, `rounding_check`, the sampler and the RDO use it.
  - The sampler builds the 2025-26 problem with RetryNewton's robust mode and starts through `run_warm`.
  - `m1_check` uses RetryNewton's robust mode and counts a `RuntimeError` as a failure. Setup errors are now raised,
    after one was found hidden by that catch during D2.
  - `results_2026.py rdo --tag --case [--maxiter]`: the warm path, the robust summary, the node report and the
    fallbacks per node.
  - *Why:* each of these paths did a cold `run_model` at a low-speed banner design (docs/development/MDA_ROBUSTNESS_PLAN.md).
- **Small items:**
  - `kv_set_by` no longer matches `rules.fuselage_margin`;
  - `integer --maxiter`;
  - `cruise.nf_confidence_min` is in the run JSON;
  - the `results_2026.py` docstring lists only the parts that exist.

### Measured (verification 5–10)

| | `mr_smoke` | `mr_smoke_int_96p32c` | `fx_smoke` | `fx_smoke_int_72p24c` |
|---|---|---|---|---|
| F (capped) | 6.1723 (uncapped; 5.9012 capped) | 6.1675 (uncapped) | **5.9532** | **5.9532** |
| F uncapped | 6.1723 | 6.1675 | 5.9562 | 5.9563 |
| M2 / M3 | 2.2711 / 2.9012 | 2.2642 / 2.9033 | 1.9997 / 2.9535 | 1.9998 / 2.9535 |
| floored F | 6.0887 | 6.0839 | 5.8474 | 5.8474 |
| n_p / n_c, banner | 96.9 / 32.3, 249 in | 96 / 32, 250 in | 72.0 / 24.0, 272 in | 72 / 24, 272 in |
| l_cb (m) | 2.286 (at 1.5 b) | 2.264 | 1.698 | 1.698 |
| iterations, wall | 58, 292 s | 15, 64 s | 59, 336 s | 6, 54 s |

- **`fx_smoke`** (cold, from Humpback): converged, nothing violated, M1 passes (SM 0.462, laps 9.05).
  - Active constraints: V_h and V_v lower bounds at both points, the M3 energy, the passenger ratio.
  - Active bounds: b, **c** (0.45: still active), **camber_scale** (2.0, its upper bound; NeuralFoil confidence 0.953
    / 0.963).
  - The 1.5 × span fuselage cap is **no longer active** (fuselage margin 0.59 m).
  - Kv 235, interior. `x_cb_frac` 0.195, x_motor −0.953 m, x_batt −0.430 / −0.347 m.
  - Margins: battery forward 0.42 / 0.51 m, battery aft 1.37 / 1.28 m, tail 0.078 m. None is active.
  - SM 0.402 / 0.396. One continuation per point (the first big step).
- **Warm `--start fx_smoke`:** 2 iterations, ΔF 2e-14.
- **`integer`:** candidates 72p23c, 71p23c, 72p24c; all three fit. Results: F 5.9460, 5.9432, **5.9532 (72p24c, the
  result)**.
- **RDO readiness at 72p24c** (`fx_rdo_setup_check.json`).
  - The robust model (11 UDR nodes × 2 points) evaluates through `run_warm`: every solve converged at its first
    attempt, with no continuation, in 284 s. μ 5.849, σ 0.131.
  - Node by node on the plain problem: 11/11 converge at the first attempt, with no wind-limit case. F 5.69 (wind
    8.9 m/s) to 5.97.
- **RDO smoke** (`rdo --maxiter 3`): runs end to end; it stops at the iteration limit, with no error.
  - 549 s for 3 SLSQP iterations, about 180 s per iteration.
  - μ 5.847, σ 0.115, robust 5.731. Every robust bound is finite (M3 static thrust μ − 2σ is −0.25 at iteration 3).
  - Each M3 node needed one continuation (recovered).
- **Monte Carlo smoke** (32 scrambled Sobol' points over the five active parameters, 3 workers): 32/32 converge.
  - 0 retried, 0 MDA failures, 0 wind limits.
  - 59.5 s wall: 5.6 s per sample per worker, about 0.59 GB peak per worker.
  - μ 5.848, σ 0.127.
- **Implied campaign cost:**
  - A deterministic optimisation: about 5–6 min cold, about 1 min warm.
  - An RDO: about 3 min per SLSQP iteration, so 1.5–3 h for 30–60 iterations.
  - A 1024-sample Monte Carlo: about 1024 × 5.6 s / workers, so about 20 min on 5 workers (memory allows about 8).

**Run provenance.**
- `fx_smoke`, `fx_smoke_warm`, the integer runs and the Monte Carlo smoke record `5a27391-dirty`. During them only the
  suite's MATLAB timing file and the reference tests (committed in `7310a4f`) differed; no model code did.
- The RDO smoke records `7310a4f`, and the validation page records `7310a4f` too.

### Biases to state in the results pages (for the final-run session)

- **Tail volumes:** they sit at their PLACEHOLDER minima (V_h 0.35 and V_v 0.02 at `mr_smoke` and at `fx_smoke`,
  against 0.66–0.80 and 0.059–0.08 flown). There is no directional-stability or crosswind check.
- **The M3 validation gap and C_D,b:** the model still flies the M3 test laps 20–46 % too fast, and its deployed C_D0
  0.24 is against Table 29's 0.36. Add Table 29's equivalent, C_D,b ≈ 0.034 on L × H (0.3025 on the wing area), to
  B9's C_D,b sensitivities.
- **Calm air:** the deterministic optimum is designed in calm air, against the RDO's 5 m/s mean wind (as in 2024-25).
  Add one deterministic run at the mean wind (5 m/s, θ = 0) to the B9 list, so the RDO gain separates robustness
  from designing at the mean.
- **The box–boom overlap** (Q31): `l_tb = l_ht − boom_offset` with the boom starting inside the payload fuselage. With
  the 1.70 m box, the overlap is larger than on Humpback, and the box and boom drag and mass double-count it.
- **The chord bound:** `c` ≤ 0.45 m (`full_mdo.yaml`) is still active at `fx_smoke`, and so is `camber_scale` at
  2.0. NeuralFoil's confidence stays at 0.95 there, but the airfoil sits at the edge of its parametrisation.

<!-- final results run -->
## Final results run
Done 2026-11-04 on `thesis-results-2026`; freeze tag `results-2026-final` (`0a6527d`). What ran: the enumeration, rounding, `mdo_wind`, multistart, bookends, normalisation, bound and other sensitivities, AirCrafter 1.0 at its shipped steps with mappings (a) and (b), RDO from three starts with robust rounding, common evaluation, 11 Monte Carlos of 1024 points, propagation, screening, node check, wind, placeholders, validation, and the timing. 91 run files back the page, 0 of them off the freeze commit. Cuts: none. Tests: 547 at the freeze, 550 at the final commit. Results: `docs/studies/results_2026/INDEX.md`. Log, with the decisions: `docs/development/RESULTS_LOG.md`.
<!-- /final results run -->
