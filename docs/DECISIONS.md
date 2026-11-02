# Decisions and source notes

Why the code is the way it is. Code comments link here (`docs/DECISIONS.md#plan-d2`) instead of using bare labels such as
"D2", which clash between plan files. Each entry says what was decided, why, where it applies in the code and which plan
file it comes from. The plan files themselves are the history and aren't edited.

Open questions and placeholders are a separate, live list: `docs/OPEN_QUESTIONS.md` (the `Q` numbers in the code).

## Sources

The citation keys used in the code. Page numbers are printed page numbers unless stated. Paths starting with
`3_LITERATURE/` are in the thesis folder, next to the repository.

| Key | Source |
|---|---|
| **Kallarackel 2025**, **[J]** | J. Kallarackel's thesis on AirCrafter 1.0, `THESIS/AirCrafter/`. "Ch. 3", "Eq. 3.x" and "Table 3.5" refer to it. The MATLAB files (`performance_modeller.m`, `inputs_file.m`, `assumptions_file.m`, `propSelector.m`, ...) are its code. |
| **G&G**, **[GG]** | Garcia & Gamboa (2022), the structure-based mass model. "Eq. 26", "Table 15" and so on in `mass/` refer to this paper. |
| **Rules_2025**, **[R]**, "2025 rule C-F2" | AIAA DBF 2024-25 rules, `Rules_2025.pdf`. "Rules p. N" in the 2024-25 code means this file. |
| **[R26]**, "2026 rules" | AIAA DBF 2025-26 rules, `3_LITERATURE/26 competition/2026-DBF-Rules.pdf` (34 pp., printed "N of 34" = PDF page). |
| **[RES26]** | Official 2025-26 results, `3_LITERATURE/26 competition/2026-DBF-Competition-Summary.pdf` (8 pp., normalised scores only). |
| **[HB]**, Humpback | USYD 2026 design report (Project Humpback), `3_LITERATURE/usyd reports/2026 USYD DBF Design Report (Project Humpback).pdf`. Printed page = PDF page + 1. |
| **[W]**, Woombah | USYD 2025 design report (Project Woombah), `2025 USYD DBF Design Report.pdf` (60 pp., "Page x of 60" = PDF page). |
| **[OH]**, Orange Hornet | USYD 2024 design report. |
| **[TM7]** | T-Motor AT7215 spec table and the 245 KV test report, `3_LITERATURE/26 competition/T-Motor AT7215/`. |
| **[APC]** | APC Propeller RPM Limits, Rev 5, 23 Mar 2022, `3_LITERATURE/26 competition/APC/APC-Propeller-RPM-Limits-rev5.pdf`. |
| **APC PER3** | APC's performance data files, `PER3_*.dat`, in `THESIS/PERFILES2`. |
| **Raymer** | D. P. Raymer, *Aircraft Design: A Conceptual Approach*. Component drag build-up, form factors and tail volume coefficients (Table 6.4). |
| **Drela** | M. Drela's first-order DC motor model, used in `propulsion/powertrain.py`. |
| **ICAO Doc 7488/3** | Manual of the ICAO Standard Atmosphere, 1993. Used in `atmosphere.py`. |
| **Spar Sizer 3.1** | The team's earlier spar spreadsheet. Notes in `docs/PLAN.md` §10. |

## Architecture decisions (`docs/PLAN.md`)

<a id="plan-d1"></a>
### plan-d1: the spar tube radius is derived

- **Decided:** the tube radius comes from the airfoil, r = k_fill · t_spar · c / 2, instead of being a design variable with a tube-fit constraint. The design-variable mode stays available.
- **Why:** the tube always fills the section at the spar, and the optimiser has one variable and one constraint fewer.
- **Applies in:** `mass/garcia_gamboa/section.py` (`TubeRadius`, `TubeFitMargin`), `problems/aero_mass.py`.
- **Source:** `docs/PLAN.md` §11, approved 2026-10-09.

<a id="plan-d2"></a>
### plan-d2: a smooth twin of the lap modeller

- **Decided:** add a differentiable lap model for the optimiser. Keep the table-based port for reporting and for the check against the MATLAB.
- **Why:** gradients through lookup tables and while loops aren't exact. The smooth model gets exact complex-step partials.
- **Applies in:** `mission/smooth_lap.py` (`SmoothLapModel`), with `mission/lap_modeller.py` as the reference.
- **Source:** `docs/PLAN.md` §11, approved 2026-10-09.

<a id="plan-d3"></a>
### plan-d3: reduced airfoil shape modes

- **Decided:** the airfoil design variables are a few shape modes on a seed Kulfan airfoil (2 + 4 to 8). Full 18-weight CST stays optional.
- **Why:** fewer variables and fewer odd shapes than free CST weights.
- **Applies in:** `airfoil/shape.py`, `config/aero.py` (`n_modes`).
- **Source:** `docs/PLAN.md` §11, approved 2026-10-09.

<a id="plan-d4"></a>
### plan-d4: prop, motor and cells are enumerated outside the optimiser

- **Decided:** propeller, motor and cell count are discrete choices enumerated in an outer loop. Battery capacity stays continuous.
- **Why:** they are catalogue items, not continuous quantities, and gradients don't exist across them.
- **Applies in:** `propulsion/powertrain.py` (option `prop`), `problems/full_mdo.py` (`PropulsionInputs`), the studies that enumerate them. The motor Kv later became a continuous design variable with a rewound reference motor (`propulsion/motor_scaling.py`, `docs/KV_PLAN.md`).
- **Source:** `docs/PLAN.md` §11, approved 2026-10-09.

<a id="plan-d5"></a>
### plan-d5: Table 15 tolerance

- **Decided:** working value ±3 % per component for "a few percent" in G&G Table 15. The plan lists this one as still open.
- **Why:** the brief didn't give a number. The verification reproduces Table 15 within 2.3 % (`docs/verification/mass.md`).
- **Applies in:** the mass verification, `verification/` and `tests/`. No code in `aircrafter2/` depends on it.
- **Source:** `docs/PLAN.md` §11.
- **Unconfirmed:** the plan records it as open, and no later entry closes it.

<a id="plan-d6"></a>
### plan-d6: a single spar tube

- **Decided:** model one equivalent tube. An N-tube extension is left for later.
- **Why:** enough for the conceptual stage, and the paper's tube concept is one tube.
- **Applies in:** `mass/garcia_gamboa/sizing.py` (`TubeSparSizing`).
- **Source:** `docs/PLAN.md` §11, 2026-10-09.

<a id="plan-d7"></a>
### plan-d7: config validation with pydantic

- **Decided:** every config is a pydantic v2 model that forbids unknown keys.
- **Why:** clear error messages for students, and misspelt keys are errors instead of silently ignored values.
- **Applies in:** `config/`, `problems/full_mdo.py`, `problems/multipoint.py`.
- **Source:** `docs/PLAN.md` §11, approved 2026-10-09.

<a id="plan-d8"></a>
### plan-d8: twin wing-mounted tractors

- **Decided:** `n_motors` ∈ {1, 2} is a discrete config choice, enumerated with the prop and cells, not a design variable. Units are identical. Packs are `per_motor` by default, with `shared` as an option, and `capacity` is per pack. The wing-load motor terms are switchable, default off. The slipstream isn't modelled. The one-engine-inoperative check is for information only.
- **Why:** the twin layout is a configuration choice, and Woombah flew it. The team has no data yet to size the rudder against the check.
- **Applies in:** `geometry/planform.py` (`MotorPlacement`), `problems/full_mdo.py` (`build_full_model`, `full_constraints`), `propulsion/powertrain.py`, `mass/garcia_gamboa/loads.py`, `stability/static.py` (`oei_check`).
- **Source:** `docs/PLAN.md` §6h and §11, 2026-10-13 brief.

<a id="plan-b"></a>
### plan-b: the powertrain sets the power

- **Decided:** in the full model, thrust and battery power come from the powertrain at a fixed throttle (`power_mode="propulsion"`), and the cruise speed is an output of the MDA (where T = D). AirCrafter's rule P = D(V) V / η stays available as `power_mode="aircrafter"`.
- **Why:** under AirCrafter's rule, lap time improves with cruise drag, since more drag means more power and more thrust out of the turns. The optimiser exploited that (Finding L1 in the plan).
- **Applies in:** `mission/smooth_lap.py`, `propulsion/powertrain.py`, `problems/full_mdo.py`.
- **Source:** `docs/PLAN.md` §6e (Finding L1) and §6f, 2026-10-09.

<a id="plan-t6"></a>
### plan-t6: nacelle form factor

- **Decided:** the default is Raymer's body form factor, as the brief asked. Raymer's nacelle and smooth-store form, FF = 1 + 0.35/f, is the option `nacelle_ff: nacelle`.
- **Why:** at nacelle fineness (f ≈ 3), the body form gives FF ≈ 3.2 and the nacelle form ≈ 1.12, so the choice is left visible as a switch.
- **Applies in:** `aero/parasite.py`, `problems/full_mdo.py` (`PropulsionInputs.nacelle_ff`).
- **Source:** `docs/PLAN.md` §6h, T6.

<a id="plan-a2"></a>
### plan-a2: the VLM is a flat-plate model

- **Finding:** the vortex-lattice α is measured from zero lift, and camber enters only through the NeuralFoil strips, which are matched on section c_l.
- **Consequence:** `alpha_sec` reports the true section angle, and the bound on α_turn is set wide.
- **Applies in:** `aero/performance.py`, `aero/wing.py`, `aero/polar.py`.
- **Source:** `docs/PLAN.md` §6b.

## Source notes on Garcia & Gamboa 2022 (`docs/PLAN.md` §7)

The default is the paper as printed unless an entry says otherwise. Each deviation is a config switch.

<a id="plan-g1"></a>
### plan-g1: Eqs. 5-8 disagree with Table 3

Eq. 5 prints K_ht = (l_ht/c_wing) A_wing, so with Eq. 7 (l_ht = K_ht c) you'd get l_ht = l_ht A. Table 3 gives a tail arm-to-span ratio instead. **Decided:** use `l_ht` (a design variable, in metres) and `l_tb` directly; tests convert the Table 3 ratios as l = ratio · b. **Applies in:** `geometry/planform.py` (`TailGeometry`). See also [plan-g19](#plan-g19).

<a id="plan-g2"></a>
### plan-g2: Tables 4, 8 and 12 list A/c², not K_A

The tables' "aerofoil area-to-chord squared ratio" is A/c², not the K_A of Eq. 9 (A/((t/c) c²)). UBI15's 0.0868 at t/c 0.141 is a realistic A/c², and read as K_A it would be impossible. **Decided:** compute K_A from coordinates by Eq. 9; tests use K_A = table / (t/c). **Applies in:** `mass/garcia_gamboa/section.py` (`SectionCoeffs`).

<a id="plan-g3"></a>
### plan-g3: Eq. 16 typo

Eq. 16 prints A_tube/A + A_rib/A + A_rib/A + A_te/A = 1; the second A_rib should be A_le. It's also unclear whether the π r²/A term uses the mean or the root chord. **Decided:** corrected, with the mean chord to match Eqs. 26-31. **Applies in:** `mass/garcia_gamboa/section.py` (`SectionCoeffs`).

<a id="plan-g4"></a>
### plan-g4: cross-reference slips

Slips with no effect on the maths. In the code, the one that matters is §2.2.5: the Eq. 32 text says ribs "Eq. 23", which should be Eq. 26. The plan lists the others. **Applies in:** `mass/garcia_gamboa/masses.py` (`WingMass`).

<a id="plan-g5"></a>
### plan-g5: Eq. 32's f_wing^(n_panels-1) is an exponent

Back-solving Table 15 with the Table 14 factors gives exponents 2.08, 2.05 and 4.18, matching n_panels − 1 = 2, 2 and 4. A single-panel wing therefore gets no f_wing correction, so the USYD calibration added a wing construction factor `k_wing` (not in G&G, default 1), which reproduces Table 15 unchanged. **Applies in:** `mass/garcia_gamboa/masses.py` (`WingMass`), `mass/calibration.py`. **Source:** `docs/PLAN.md` §7, with the note of 2026-10-23.

<a id="plan-g6"></a>
### plan-g6: Eq. 33 multi-panel

The paper doesn't say whether the factors apply per panel or once to the sum. **Decided:** once to the sum, so a one-panel Eq. 33 reduces exactly to Eq. 32. The wing is rectangular and single-section in this code, so this isn't exercised. **Applies in:** `mass/garcia_gamboa/masses.py`.

<a id="plan-g7"></a>
### plan-g7: sizing chord and loads chord

§3.2 loads use the mean chord, and §3.3 sizing uses the root chord. **Decided:** loads at the mean chord, sizing with K_tube = r/c_root. For a rectangular wing the two are the same. **Applies in:** `mass/garcia_gamboa/loads.py`, `mass/garcia_gamboa/sizing.py`.

<a id="plan-g8"></a>
### plan-g8: Eq. 46 wording

The text says the torsion moment distribution is uniform, but the formula is linear in (b/2 − y): a uniform torque per unit span. Wording only. **Applies in:** `mass/garcia_gamboa/loads.py` (`WingLoads`).

<a id="plan-g9"></a>
### plan-g9: Eqs. 47 and 117 sign

With the spar forward of c/4, x_spar − 1/4 < 0 and t_div comes out negative. **Decided:** floor at 0 smoothly (softplus). Table 3 gives an aerofoil lift-curve slope of 6 /rad where Eq. 47 asks for the wing's; the framework uses the wing slope from the VLM, and tests use 6 /rad. **Applies in:** `mass/garcia_gamboa/sizing.py` (`TubeSparSizing`), `mass/garcia_gamboa/loads.py`.

<a id="plan-g10"></a>
### plan-g10: the tip-deflection polynomial can't be reproduced

The expressions agree at y1 = 0 (3 b⁴), but assembling the paper's Eqs. 63-69 doesn't give its Eq. 70, and an exact double integration gives a third result. At y1 = b/6, Eq. 70 is 23 % above exact. **Decided:** default to the printed Eq. 110, with `deflection_mode: exact` available. **Applies in:** `mass/garcia_gamboa/sizing.py` (`deflection_poly`).

<a id="plan-g11"></a>
### plan-g11: Eq. 112 shear factor

The 3Q/(2A) peak factor is for a solid rectangle. A thin-walled circular tube peaks at 2Q/A, so t^τ is 25 % low. It's a small term. **Decided:** implement the paper as printed. **Applies in:** `mass/garcia_gamboa/sizing.py` (`TubeSparSizing`).

<a id="plan-g12"></a>
### plan-g12: Eq. 115 torsional stiffness is twice Bredt-Batho

For a single-wall tube GJ = 2π r³ G t, but the paper gives 4π K³ c³ G t, probably carried over from the two-faced sandwich skin of Eq. 91. So t^θ (Eq. 116) and t^div (Eq. 117) are half the Bredt value. **Decided:** default to the paper, with `torsion_stiffness_mode: bredt` available. **Applies in:** `mass/garcia_gamboa/sizing.py` (`TubeSparSizing`).

<a id="plan-g13"></a>
### plan-g13: Eq. 37

The text mentions t_cb and ρ_cb, but the equation is just f_cb S_cb. **Decided:** implement the equation. **Applies in:** `mass/garcia_gamboa/masses.py` (`FuselageMass`).

<a id="plan-g14"></a>
### plan-g14: Eq. 39 uses the total mass

m_gear = f_gear · m uses the total mass, which adds to the closure loop. That's fine in the MDA. **Applies in:** `mass/garcia_gamboa/masses.py` (`GearMass`).

<a id="plan-g15"></a>
### plan-g15: Eq. 50 needs x_gd

Eq. 50 needs the ground-test hold point x_gd, which Table 3 doesn't give (`docs/OPEN_QUESTIONS.md` Q4). Eqs. 48-49 use the full weight, payload included. **Applies in:** `mass/garcia_gamboa/loads.py` (`WingLoads`).

<a id="plan-g16"></a>
### plan-g16: g in Table 3

Table 3 gives g = 9.80655, and the standard value is 9.80665. Tests use the table value and the framework uses 9.80665. The difference is negligible. **Applies in:** `constants.py`, `problems/full_mdo.py`.

<a id="plan-g17"></a>
### plan-g17: values missing from the paper's tables

For the tube concept c_rib/c isn't tabulated: 0.7 (§3.3.3, Table 8). The tail-boom material isn't in Table 13: unidirectional CFRP assumed. The maximum tail rotation in Table 3 is in degrees. **Applies in:** `mass/garcia_gamboa/inputs.py`, `mass/garcia_gamboa/materials.py`.

<a id="plan-g18"></a>
### plan-g18: flight and ground bending are equal at n = 2

With n = 2, Eqs. 45 and 49 at y = 0 both give W b/4, so Eq. 52 sits exactly on the kink of the smooth maximum. Hence the hard-max verification mode. **Applies in:** `mass/garcia_gamboa/loads.py` (`CriticalLoads`).

<a id="plan-g19"></a>
### plan-g19: tail-boom shear

The paper's numbers use Eq. 55 with K_ht from Eq. 5 as printed, which is Eq. 54's moment-balance shear divided by the aspect ratio. This reading reproduces all three Table 15 booms to 0.2 %; the literal Eq. 54 reading is 9-14 times high. **Decided:** `tail_boom_load_mode: paper` by default (f_tb = 12.81 was calibrated with it), `eq54` available but needs f_tb recalibrated. **Applies in:** `mass/garcia_gamboa/loads.py` (`TailBoomLoads`).

<a id="plan-g20"></a>
### plan-g20: the 12-19 % wing over-prediction

Resolved. It had two causes, plan-g19 and plan-g23, and no correction factor was tuned.

<a id="plan-g21"></a>
### plan-g21: statistical Eq. 124's fit range

The fit covers ACC aircraft of 8.9-16 kg. DBF aircraft (about 3-7 kg) are an extrapolation, with empty-mass errors up to 55 % in the paper. **Decided:** use it only as the MDA's initial guess and a sanity column. **Applies in:** `mass/statistical.py`.

<a id="plan-g22"></a>
### plan-g22: Table 15's UBI11 column

It uses UBI11's individual general factors, not the combined ones (f_wing and f_tb excepted). `configs/verification/ubi11.yaml` follows this. **Applies in:** the verification configs only.

<a id="plan-g23"></a>
### plan-g23: spar sizing radius

The paper's numbers use K_tube = r/c_mean with the sizing chord c_root, which sizes the tube as if its radius were r c_root/c_mean. **Decided:** `sizing_radius_mode: paper` by default (consistent with f_wing = 1.12), `physical` available. The two are identical for a rectangular wing. **Applies in:** `mass/garcia_gamboa/sizing.py` (`TubeSparSizing`).

## Port notes on the AirCrafter 1.0 MATLAB (`docs/PLAN.md` §8)

<a id="plan-m1"></a>
### plan-m1: MATLAB rounds V_stall

MATLAB rounds `Vstall` to an integer, and the rounded value starts the acceleration, deceleration and turn-rate tables. The port uses the exact value; `matlab_compat=True` reproduces the rounding. **Applies in:** `mission/lap_modeller.py`.

<a id="plan-m7"></a>
### plan-m7: unflagged NaN in `leg_calc_turn.m`

If the turn-entry speed is below the slowest speed in the deceleration table, `interp1` returns NaN, nothing flags it, the lap times become NaN and the lap count is 1. The port keeps the speed unchanged in that case; `matlab_compat=True` reproduces the NaN. **Applies in:** `mission/lap_modeller.py`. **Source:** found 2026-10-09 with the MATLAB run.

## Final fixes before the final run (`docs/RESULTS_2026_PLAN.md`)

All from the section "Final fixes before the final run", 2026-11-02. Each is limited to the 2025-26 ruleset.

<a id="ff-d1"></a>
### ff-d1: the score cap

- **Decided:** M2 = 1 + cap(NI / NI_max) and M3 = 2 + cap(T3 / B_max). The cap is exactly r for r ≤ 1 − δ and exactly 1 for r ≥ 1 + δ, with a C1 quadratic blend between (bias δ/4 at r = 1), δ = `cap_delta` = 0.005. `floored()` applies the hard min(r, 1).
- **Why:** the rules' Max is the best team's, so each ratio is at most 1 ([R26] p. 20). Uncapped, the payload credit drove the design to the fuselage-length cap (97 passengers, F 6.17 > 6).
- **Applies in:** `scoring/dbf2026.py`.

<a id="ff-d2"></a>
### ff-d2: the rigid payload fuselage

- **Decided:** `x_cb_frac` is where the wing sits along the fuselage. The payload stations move with the box, and the motor group sits at the box front at Humpback's nose length: x_motor = x_motor_ref + (x_cb − x_cb_ref) − (l_cb − l_cb_ref)/2. Three constraints: `batt_fwd_margin`, `batt_aft_margin` and `tail_margin`.
- **Why:** without them the optimum had the shell 0.19 m ahead of its payload, the pack ahead of the motor, and a 2.29 m box running past the motor and the tail.
- **Applies in:** `stores2026.py` (`Stores2026`, `Packaging2026`), `problems/full_mdo.py` (`build_full_model`), `problems/multipoint.py` (`multipoint_constraints`).

<a id="ff-d3"></a>
### ff-d3: undercarriage and leakage drag areas

- **Decided:** constant drag areas `aero.cd_area_gear` = 0.0028 m² and `aero.cd_area_lp` = 0.00224 m² ([HB] Table 29, M2 column, p. 33, on S_ref 0.5600 m², p. 36; Q57), added at M1, M2 and M3. The scaling is a placeholder (Q63).
- **Why:** the parasite build-up had no undercarriage and no leakage term, and the model flew the test laps 16-48 % too fast. This is a missing component of the team's own build-up, and nothing is fitted to the flights.
- **Applies in:** `problems/full_mdo.py` (`FullMDA`, `cd_extra_name`), `config/aero.py`.

<a id="ff-d4"></a>
### ff-d4: the old constraint set

- **Decided:** `OLD_CONSTRAINTS["stability.SM_max"]` = 0.40, the old 0.20 shifted into the model frame by the same +0.20 c as SM_min (Q9, Q62).
- **Why:** 0.20 sits below the 2025-26 SM_min of 0.25, so the sensitivity could never be feasible.
- **Applies in:** `studies/results_2026.py` only.

<a id="ff-d5"></a>
### ff-d5: the robust analysis paths

- **Decided:** `problems.multipoint.initial_values` and `run_warm` are the way to evaluate a 2025-26 design. The sampler, `optimise`, `rounding_check` and the RDO go through them. `m1_check` uses RetryNewton's robust mode and counts a `RuntimeError` as a failed check, with setup errors raised first.
- **Why:** each of these paths did a cold `run_model` at a low-speed banner design, which can diverge. A setup error was found hidden by the `RuntimeError` catch.
- **Applies in:** `problems/multipoint.py`, `robustness/sampling.py`, `studies/results_2026.py`.

## MDA robustness (`docs/MDA_ROBUSTNESS_PLAN.md`)

<a id="mr-f1"></a>
### mr-f1: continuation fallback in RetryNewton

- **Decided:** under `mda_robust`, when a solve fails, retry in this order: current state, own last converged state (skipped if equal to the start), continuation in the MDA's inputs (4 steps, then 8, the last step exactly the target), the reference node's state, cold.
- **Why:** at low-speed M3 banner designs, T = D has a second, sub-stall root, and the previous state lies outside the real root's basin. Bounds on v or a line search alone don't fix it (tested, §1.5).
- **Applies in:** `robustness/rdo_problem.py` (`RetryNewton`), `problems/full_mdo.py` (`FullMDA`, `mda_robust`).
- **Source:** §2.1 and "Status", 2026-11-02.

<a id="mr-f2"></a>
### mr-f2: the warm path

- **Decided:** a run with `start` or `x0` under `mda_robust` first converges at the config's unfixed start design, then moves to the target.
- **Why:** the target's own cold start can diverge, and the config's start design converges cold. This path can't be worse.
- **Applies in:** `studies/results_2026.py` (`optimise`), and as `run_warm` in `problems/multipoint.py` ([ff-d5](#ff-d5)).
- **Source:** §2.2 and "Status", 2026-11-02.

<a id="mr-quiet"></a>
### mr-quiet: QuietDirectSolver

- **Decided:** under `mda_robust` the MDA's linear solver raises `AnalysisError` at once on a singular or NaN Jacobian.
- **Why:** OpenMDAO's `DirectSolver` first builds a diagnosis (a dense SVD of the Jacobian), which took 575 s and over 3 GB and got a test run OOM-killed, and then raises `RuntimeError`, which `RetryNewton` doesn't catch, so no fallback ran. With this solver, the failing design converges in 24 s at 0.54 GB. A converged state is never singular, so results don't change.
- **Applies in:** `robustness/rdo_problem.py` (`QuietDirectSolver`), `problems/full_mdo.py` (`FullMDA`).
- **Source:** "Status", deviation 1, 2026-11-02.

<a id="mr-m1-sm"></a>
### mr-m1-sm: the M1 check's static margin is one-sided for 2025-26

- **Decided:** for `kind: dbf2026`, `m1_check` fails on SM only below SM_min. Above SM_max it reports `SM_high`. `SM_max` stays 0.47. The 2024-25 check keeps both bounds.
- **Why:** a too-large margin costs trim drag and elevator authority, not flight safety. The published 2024-25 results have designs failing M1 on SM above SM_max, so a global change would contradict a frozen page if it were rerun.
- **Applies in:** `problems/multipoint.py` (`M1_SM_ONE_SIDED`, `m1_check`).
- **Source:** "Status", Problem 2, decision (b), 2026-11-02. See also Q9, Q62.

## Other plans

<a id="tc-takeoff"></a>
### tc-takeoff: take-off in the mission time

- **Decided:** a switch, `course.takeoff.in_mission_time`, off by default so that published results don't change. When on, the ground roll is integrated and its time and energy go into lap 1.
- **Why:** the rules start the clock when the throttle is advanced for take-off, and `performance_modeller.m` has the ground roll commented out (l. 435), so model times were too short by an amount that depends on the design.
- **Applies in:** `mission/course.py` (`TakeoffConfig`), `mission/smooth_lap.py`, `mission/lap_modeller.py`.
- **Source:** `docs/TAKEOFF_COURSE_PLAN.md`.

<a id="r25-multipoint"></a>
### r25-multipoint: the 2024-25 multipoint formulation

- **Decided:** one airframe flying M2 and M3. Airframe and Kv are shared, capacity, x_batt and throttle are per point, fuel is M2 only, and the prop per point is enumerated. M1 is an analysis-only check.
- **Why:** the rules require the same configuration for all missions but allow other packs and props between attempts (Rules_2025 p. 10, 14, 16).
- **Applies in:** `problems/multipoint.py`, `scoring/dbf2025.py`, `stores.py`.
- **Source:** `docs/RESULTS_PLAN.md` §9.

<a id="r26-scoring"></a>
### r26-scoring: the 2025-26 score model

- **Decided:** F = M1 + M2 + M3 with M1 = 1, M2 from the net income, and M3 from the banner term, with the laps taken from the continuous `laps_frac`. See [ff-d1](#ff-d1) for the cap.
- **Why:** [R26] p. 20 defines the mission scores. [HB] Eq. 6 omits EF, and the rules are used.
- **Applies in:** `scoring/dbf2026.py`.
- **Source:** `docs/RESULTS_2026_PLAN.md` §4.1.

<a id="r26-payload"></a>
### r26-payload: payload and fuselage sizing

- **Decided:** n_p and n_c are continuous design variables with n_p ≥ 3 n_c, and the payload fuselage is sized from the counts. Final designs round the counts and re-optimise, running only the candidates that fit (`rounding_check`). The restraints stay on board in M1 and M3.
- **Why:** rules p. 10-11. Rounding candidates that can't fit the fuselage cap are infeasible for every design and stalled the optimiser.
- **Applies in:** `stores2026.py`, `problems/multipoint.py`, `studies/results_2026.py`.
- **Source:** `docs/RESULTS_2026_PLAN.md` §4.2, and `docs/MDA_ROBUSTNESS_PLAN.md` "Status", deviation 3.

<a id="r26-banner"></a>
### r26-banner: the banner

- **Decided:** L_banner is a design variable in [10 in, 275 in], height L/5, drag on the banner's own area with C_D,b 0.024 (the mean of [HB] Table 41's length column). The stowed banner flies lap 1 until the first downwind leg.
- **Why:** [R26] rule limits, and [HB]'s values disagree with each other, so the choice is recorded in Q56.
- **Applies in:** `stores2026.py`, `mission/smooth_lap.py` (`banner_stowed_legs`), `mission/course.py`.
- **Source:** `docs/RESULTS_2026_PLAN.md` §4.3.

<a id="r26-wind"></a>
### r26-wind: wind direction and turn drift

- **Decided:** a steady wind with a direction. On straights, the along-track component adds to the ground speed and the crosswind is crabbed out. Turns are flown at constant airspeed in the air-mass frame, and the drift can be carried to the next straight (`course.turn_drift`, off by default).
- **Why:** the earlier model had only an along-course wind, and turns ignored it. The expression reduces exactly to the earlier one at wind_dir = 0.
- **Applies in:** `mission/smooth_lap.py`, `mission/course.py`.
- **Source:** `docs/RESULTS_2026_PLAN.md` §4.7.

<a id="rr-f-build"></a>
### rr-f-build: the overall build factor

- **Decided:** `f_build` (default 1.0) multiplies every G&G structural mass inside the mass closure, and is the mass parameter of the reduced RDO.
- **Why:** it stands for how heavy the aircraft comes out when built. It carries 70-78 % of the static-margin variance, so it belongs in the uncertain set.
- **Applies in:** `mass/garcia_gamboa/inputs.py`, `mass/garcia_gamboa/masses.py`.
- **Source:** `docs/RDO_REDUCED_PLAN.md` §1.

<a id="pr-layers"></a>
### pr-layers: placeholders and ruleset files

- **Decided:** every placeholder lives in `configs/placeholders.yaml` (independent of the rules) or `configs/rulesets/<ruleset>.yaml` (depends on them), one value per line with a comment giving units, status, Q number and source. Load order: code defaults, placeholders, ruleset, problem config and its `extends:` chain, study variant.
- **Why:** one place to edit a value, with every config picking it up.
- **Applies in:** `config/loader.py`, `config/layer_files.py`.
- **Source:** `docs/PLACEHOLDERS_RULESETS_PLAN.md`, 2026-10-22.
