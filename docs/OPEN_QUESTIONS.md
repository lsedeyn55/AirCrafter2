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
| Q11 | Lap modeller convention for the MATLAB check: reproduce `batteryFraction = 0` (lapCount = 1) or set 1.0? (PLAN §8 M2) | step 6 | report both |
| Q12 | Payload mass `m_pay`, load factor n, design speed for loads, deflection ratio, tail-boom rotation limit for USYD designs | step 5 | G&G Table 3 values (n = 2, δ/b = 0.1, θ_tb = 1°); m_pay is a placeholder |
| Q13 | Spar Sizer 3.1 speed: 138.65 km/h is used, but the script's comment says V_a = 80.6 km/h | reference only | n/a |
| Q14 | Tail-boom load mode for USYD designs: the paper's Eq. 55/Eq. 5 reading (consistent with f_tb = 12.81) or the physical Eq. 54 (needs a USYD-calibrated f_tb)? Same question for `sizing_radius_mode` with tapered wings (PLAN G19, G23). | calibration | paper readings |
| Q15 | Tail-boom minimum gauge vs f_tb: f_tb = 12.81 scales the *computed* boom tube (walls of ~0.06 mm in the paper's cases) up to a whole fuselage. A 0.5 mm boom minimum gauge multiplied by 12.81 gives a heavy fuselage (0.35 kg boom in the README example). Should the minimum gauge apply to the spar only until f_tb is recalibrated for USYD aircraft? | step 5 | min gauge on both (as briefed) |
| Q16 | NeuralFoil transition setting n_crit for DBF conditions (XFoil default 9 = "average wind tunnel"; sailplanes use 12-14) and forced transition, if any | step 3 | n_crit = 9, free transition |
| Q17 | Step 3 placeholders: cruise V = 20 m/s, fixed lift 49 N, S = 0.375 m², t/c ≥ 0.10 (replaced by the coupled mass and tube-fit constraint in step 5) | step 5 | as listed |
| Q18 | Step 4 placeholders: turn C_L margin C_L,max/C_L ≥ 1.21 (10 % speed margin in turns), optional turn power limit (400 W used in the report), fixed V = 20 m/s | step 6-7 | as listed |
