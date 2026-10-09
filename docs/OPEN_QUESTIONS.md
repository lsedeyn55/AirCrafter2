# Open questions and placeholders

Each item is either a value I won't invent (ground rule) or a decision outside the brief. Placeholders appear in the
configs as `PLACEHOLDER`, with a pointer back to this file.

| # | Question | Needed by | Current placeholder |
|---|---|---|---|
| Q1 | `THESIS/` is not a git repo. Should I `git init` in `AirCrafter2/` (with a .gitignore for `.venv`, `*_out/`, `__pycache__`) so each milestone gets one commit? | step 2 | none (no commits) |
| Q2 | Table 15 tolerance: is ±3% per component acceptable once the cause of the 12–19% wing offset is found? (PLAN §7 G20) | step 2 | ±3% |
| Q3 | Commercial CF tube catalogue (OD/ID list) for post-processing and minimum gauge | step 2 | 22/20 and 20/18 mm from Spar Sizer 3.1; `t_min` = 0.5 mm placeholder |
| Q4 | G&G inputs not given in the paper: panel-break position y₁ (Eq. 110) and ground-test hold point x_gd (Eq. 50) for UBI11, GRA13 and UBI15 | step 2 | y₁ = b/(2·n_panels) and x_gd = spar position, both labelled as assumptions |
| Q5 | Tube fill fraction `k_fill` (r_tube vs local half-thickness) and clearance | step 2 | k_fill = 0.85, clearance = 1 mm (placeholders) |
| Q6 | Spar material: G&G Table 13 (720 MPa / 84 GPa) or the team's tube data (Spar Sizer: 340.7 MPa UTS, 220 GPa)? Single equivalent tube or two tubes with load split? | step 5 | Table 13, single tube |
| Q7 | Motor catalogue (Kv, Rm, I0, mass, max current), ESC and servo masses, avionics mass | step 7 | none: needs team data |
| Q8 | Battery cell data (nominal voltage, internal resistance, specific energy, mass per Ah, usable fraction) | step 7 | none: needs team data |
| Q9 | Static-margin band and V_h, V_v ranges for RC aircraft | step 7 | SM 0.05–0.20; V_h, V_v from Raymer Table 6.4 (homebuilt/sailplane rows). Edition to verify. |
| Q10 | NeuralFoil `analysis_confidence` threshold for buildable/in-distribution shapes | step 3 | 0.8 |
| Q11 | Lap modeller convention for the MATLAB check: reproduce `batteryFraction = 0` (lapCount = 1) or set 1.0? (PLAN §8 M2) | step 6 | report both |
| Q12 | Payload mass `m_pay`, load factor n, design speed for loads, deflection ratio, tail-boom rotation limit for USYD designs | step 5 | G&G Table 3 values (n = 2, δ/b = 0.1, θ_tb = 1°); m_pay is a placeholder |
| Q13 | Spar Sizer 3.1 speed: 138.65 km/h is used, but the script's comment says V_a = 80.6 km/h | reference only | n/a |
