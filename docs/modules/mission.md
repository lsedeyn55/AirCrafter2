# Mission / performance

Code: `aircrafter2/mission/`. Source: Kallarackel (2025), *AirCrafter*, Ch. 3, and the MATLAB in
`THESIS/AirCrafter/`.

## Reference port (`lap_modeller.py`)

A faithful port of `performance_modeller.m`:
- take-off static thrust (Eqs. 3.1–3.3);
- constant-power cruise (Eqs. 3.4–3.7);
- Euler acceleration and deceleration lookup tables;
- stall-limited turn rate capped at n_struct;
- lap assembly from Table 3.5;
- lap count within the time and energy budget.

NaN flags are replaced by continuous margins (`stall_margin`, `turn_speed_margin`).

`matlab_compat=True` reproduces the MATLAB exactly. That includes its rounded V_stall (PLAN M1) and its unflagged
NaN when a turn is entered below the deceleration table (M7). The port matches `performance_modeller.m`, run in
MATLAB R2026a, to ~1e-8 (`docs/problems/lap_mdo.md`, `verification/matlab/run_reference.m`).

## Smooth twin (`smooth_lap.py`, decision D2)

The same physics without tables or loops, with exact complex-step partials:
- **Straights:** RK4 in distance.
- **Turns:** entry turn rate ω = g√(n² − 1)/v, with n = KSmin(q S C_L,max / W, n_struct). The C_L,max-drag
  deceleration is integrated in time, keeping only its negative part.
- **Lap count:** continuous `laps_frac`; the integer `lapCount` is for reporting only.

There are two power modes:

| `power_mode` | Thrust | Speed | Energy |
|---|---|---|---|
| `aircrafter` | P = D(V)V/η, T = P/v | held at V (smooth min) | P · t |
| `propulsion` (decision B) | T(v) from the powertrain | no cap; settles where T = D | ∫ P_batt(v) dt per leg |

**Finding L1:** `aircrafter` mode rewards cruise drag. More drag means more power on every leg, which the
optimiser exploited. That's why the full model uses `propulsion` mode.

Robustness hooks: `wind` (headwind on the 500 ft legs, tailwind on the 1000 ft leg) and `thrust_factor`. **Wind
limit:** lap 1 starts at V_stall on the into-wind leg, and the straights are integrated in ground distance
(dt/ds = 1/(v − wind)). As the headwind nears V_stall the ground speed after lift-off goes to zero: the aircraft cannot
make progress into the wind and would not fly. That is the aircraft's wind limit, a property of the design, not a
model error. It is about 7.5 m/s for the light M3 aircraft of the multipoint optimum (V_stall 8.0–8.3 m/s) and lower
for bigger wings. A lap past it (non-finite, or a lap time ≤ 0) raises `AnalysisError`. Sampling records it as
`lap_invalid`, reported as "wind limit (no flight)": separate from MDA non-convergence and from constraint violation,
and never scored.

Agreement with the reference port: ≤ 0.65% on the 3-lap time across 8 points.

## Oval (`oval.py`, used by `oval_lap`)

Constant-speed oval: R = V²/(g√(n² − 1)), lap = (2L_s + 2πR)/V. Kept as a teaching and verification case.
