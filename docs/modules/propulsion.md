# Propulsion

Code: `aircrafter2/propulsion/`. Decision B (2026-10-09): the powertrain sets the power, and the cruise speed is
the speed at which thrust equals drag.

## Propeller data (`apc.py`)

- APC's published PER3 performance files (`../PERFILES2/PER3_*.dat`), the same data AirCrafter 1.0 used.
- Each prop gets least-squares polynomials C_T(J, RPM) and C_P(J, RPM): degree 4 in J, degree 2 in RPM/10⁴. The fit
  is about 0.0002 RMS in C_T and 0.001 in C_P. The windmill branch (C_T < 0) is dropped.
- RPM limit: 150000 / D[in] (APC, E props), used at 85% as in `propSelector.m`.

## Powertrain (`powertrain.py`)

| Element | Model |
|---|---|
| Battery | V_oc = n_s V_cell, R_b = n_s R_cell, E_usable = f_use n_s V_cell Q, m_batt = E_nom / e_spec |
| ESC | ideal PWM at throttle δ: V_motor = δ V_term, I_batt = δ I_motor |
| Motor | first-order DC model (Drela, *First-Order DC Electric Motor Model*, MIT 16.50 notes, 2007): I = (δ V_oc − ω/K_v) / (R_m + δ² R_b), Q = (I − I₀)/K_v |
| Prop | T = C_T ρ n² D⁴, Q = C_P ρ n² D⁵ / 2π, J = v / (nD) |

The rotor speed solves motor torque = prop torque (Newton) at 8 speed nodes from 0 to `v_top`. The outputs are
cubic fits T(v), P_batt(v) and rpm(v), plus T_static, the static current and m_sys. All partials are exact
(complex step through the fixed-iteration Newton).

## Constraints in the full model

| Constraint | How it's checked |
|---|---|
| Static thrust ≥ take-off requirement | ground roll of `takeoff_distance`; Josh's Eq. 3.3 |
| RPM at the top speed ≤ 0.85 × APC limit | rotor speed rises with flight speed, so it's checked at the top speed |
| Static current ≤ I_max | post-check: no design variable changes it at a fixed prop, cell count and throttle |
| Three laps' energy ≤ (1 − reserve) × usable energy | energy reserve is a placeholder (20%) |

**Discrete choices** (prop, cell count, motor) are enumerated outside the optimiser (D4): `examples/full_mdo.py`
or `aircrafter2 run --prop`.

## Placeholders (OPEN_QUESTIONS Q7, Q8, Q23)

Motor Kv/Rm/I0/mass/I_max, cell voltage/resistance/specific energy, ESC/prop/avionics masses. Lap time is most
sensitive to Kv (normalised sensitivity −0.25), so replace that first.
