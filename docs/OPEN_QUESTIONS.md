# Open questions and placeholders

Each item is either a value I won't invent (ground rule) or a decision outside the brief. Placeholders appear in the
configs as `PLACEHOLDER`, with a pointer back to this file.

| # | Question | Needed by | Current placeholder |
|---|---|---|---|
| Q1 | ~~Git repo~~: **resolved 2026-10-09**, repo initialised in `AirCrafter2/`. A repo-local identity was set (`Lucas`, account email); change it with `git config user.name/user.email`. | n/a | n/a |
| Q2 | Table 15 tolerance: is ±3% per component acceptable? The cause was found (PLAN G19, G22, G23); worst case is now 2.3%. | step 2 | ±3% |
| Q3 | Commercial CF tube catalogue (OD/ID list) for post-processing and minimum gauge | step 2 | 22/20 and 20/18 mm from Spar Sizer 3.1; `t_min` = 0.5 mm placeholder |
| Q4 | G&G inputs not given in the paper: panel-break position y₁ (Eq. 110) and ground-test hold point x_gd (Eq. 50) for UBI11, GRA13 and UBI15 | step 2 | y₁ = b/(2·n_panels) and x_gd = spar position. Sensitivity is small: ≤1.3% and 0% on the wing (docs/mass_verification.md §3). |
| Q5 | Tube fill fraction `k_fill` (r_tube vs local half-thickness) and clearance | step 2 | k_fill = 0.85, clearance = 1 mm (placeholders) |
| Q6 | Spar material: G&G Table 13 (720 MPa / 84 GPa) or the team's tube data (Spar Sizer: 340.7 MPa UTS, 220 GPa)? (Single tube decided, D6.) | step 5 | Table 13 |
| Q7 | Motor catalogue (Kv, Rm, I0, mass, max current), ESC and servo masses, avionics mass | step 7 | none: needs team data |
| Q8 | Battery cell data (nominal voltage, internal resistance, specific energy, mass per Ah, usable fraction) | step 7 | none: needs team data |
| Q9 | Static-margin band and V_h, V_v ranges for RC aircraft | step 7 | SM 0.05–0.20; V_h, V_v from Raymer Table 6.4 (homebuilt/sailplane rows). Edition to verify. |
| Q10 | NeuralFoil `analysis_confidence` threshold for buildable/in-distribution shapes | step 3 | 0.8 |
| Q11 | **Resolved:** the MATLAB check covers both conventions (batteryFraction 1 and 0). Original: lap modeller convention for the MATLAB check: reproduce `batteryFraction = 0` (lapCount = 1) or set 1.0? (PLAN §8 M2) | step 6 | report both |
| Q12 | Payload mass `m_pay`, load factor n, design speed for loads, deflection ratio, tail-boom rotation limit for USYD designs | step 5 | G&G Table 3 values (n = 2, δ/b = 0.1, θ_tb = 1°); m_pay is a placeholder |
| Q13 | Spar Sizer 3.1 speed: 138.65 km/h is used, but the script's comment says V_a = 80.6 km/h | reference only | n/a |
| Q14 | Tail-boom load mode for USYD designs: the paper's Eq. 55/Eq. 5 reading (consistent with f_tb = 12.81) or the physical Eq. 54 (needs a USYD-calibrated f_tb)? Same question for `sizing_radius_mode` with tapered wings (PLAN G19, G23). | calibration | paper readings |
| Q15 | **Resolved 2026-10-09:** boom minimum wall of 1 mm (USYD tubes), separate from the spar gauge, with f_tb refitted to 0.847 on that basis (docs/tail_boom_calibration.md). Original question: tail-boom minimum gauge vs f_tb: f_tb = 12.81 scales the *computed* boom tube (walls of ~0.06 mm in the paper's cases) up to a whole fuselage. A 0.5 mm boom minimum gauge multiplied by 12.81 gives a heavy fuselage (0.35 kg boom in the README example). Should the minimum gauge apply to the spar only until f_tb is recalibrated for USYD aircraft? | step 5 | min gauge on both (as briefed) |
| Q16 | NeuralFoil transition setting n_crit for DBF conditions (XFoil default 9 = "average wind tunnel"; sailplanes use 12-14) and forced transition, if any | step 3 | n_crit = 9, free transition |
| Q17 | Step 3 placeholders: cruise V = 20 m/s, fixed lift 49 N, S = 0.375 m², t/c ≥ 0.10 (replaced by the coupled mass and tube-fit constraint in step 5) | step 5 | as listed |
| Q18 | Step 4 placeholders: turn C_L margin C_L,max/C_L ≥ 1.21 (10 % speed margin in turns), optional turn power limit (400 W used in the report), fixed V = 20 m/s | step 6-7 | as listed |
| Q19 | Step 5 placeholders: m_sys 1.2 kg, m_pay 1.5 kg, tail arm 0.8 m, S_ht 0.06 m², S_vt 0.03 m², boom offset 0.15 m, cargo bay 0.40 × 0.12 × 0.12 m, boom radius 8 mm, minimum spar tube radius 5 mm | step 7 | as listed |
| Q20 | **Resolved 2026-10-09:** MATLAB signed in, reference run done, port matches to ~1e-8. Original: MATLAB reference for the step 6 code check: please run `matlab -batch "run('verification/matlab/run_reference.m')"` from `AirCrafter2/` (needs your MathWorks sign-in; headless start from this session hangs) | step 6 | pending |
| Q21 | **Decided 2026-10-09: option B.** Propulsion sets the power (motor + prop + battery at a chosen throttle); cruise speed is the output where thrust = drag; legs use the propulsion thrust curve T(v). Original: Finding L1: replace AirCrafter's power setting P = D(V)V/η (which rewards cruise drag) with a propulsion-defined power in step 7? | step 7 | proposed: yes |
| Q22 | Step 6 placeholder: electrical power ≤ 300 W until propulsion exists | step 7 | 300 W |
| Q23 | Step 7 placeholders: motor Kv 900 rpm/V, Rm 0.05 Ω, I0 1 A, 0.15 kg, I_max 60 A; 4S LiPo 3.7 V/cell, 5 mΩ/cell, 150 Wh/kg; ESC 0.06 kg; prop 0.03 kg; avionics 0.25 kg; tail AR 4/2, t/c 0.09, η_h 0.9; stations (motor −0.30 m, avionics −0.10 m, gear 0.05 m, wing CG 0.40c); SM band 0.05–0.20; V_h 0.35–0.65; V_v 0.02–0.06; Monte Carlo spreads | step 7 | as listed |
| Q24 | **Placeholder adopted 2026-10-09:** `energy_reserve: 0.2` (20% of usable energy kept in reserve), changeable in the config. Original: objective: the placeholder 3-lap time sizes the battery to exactly three laps. Add an energy reserve now, or wait for the real scoring plug-in? | step 8 | none |
| Q25 | Nacelle mass per unit for the twin layout. The twin brief cites a "Motor nacelles" row of Woombah Table 34, but Table 34 has no such row: the nacelles look to be inside the 3.41 lb "Wing" item. "Motor nacelles" is a row of Table 26 (drag). Need a measured or CAD nacelle mass (PLAN §6h T1). | twin (checkpoint 2) | `m_nacelle` = 0.0 (twin config) |
| Q26 | Twin decisions from PLAN §6h: T2 motor x-station (firewall −5.5" vs Table 34 motor CG −6.5"), T3 one relief switch or a separate nacelle-torsion switch, T4 `y_motor` as a semi-span fraction or an absolute station, T5 shear relief | twin (checkpoint 2) | as briefed: −5.5", one switch, fraction, shear included |
| Q27 | Wing-mounted batteries (in the nacelles): bending relief from the pack masses. Not modelled; packs stay at the fuselage `x_batt`. Option for later. | later | fuselage packs |
| Q28 | `pack_I_max` for the shared topology (pack C-rating × capacity, or the wiring/fuse limit) | twin, shared | off |
