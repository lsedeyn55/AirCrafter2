# Propulsion

Code: `aircrafter2/propulsion/`. Decision B (2026-10-09): the powertrain sets the power, and the cruise speed is
the speed at which thrust equals drag.

## Propeller data (`apc.py`)

- APC's published PER3 performance files (`../PERFILES2/PER3_*.dat`), the same data AirCrafter 1.0 used. Override the folder with `propulsion.apc_dir` or `AIRCRAFTER2_APC_DIR` (README, Install).
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

## Several motors (`n_motors`, D8)

`n_motors` is a discrete layout choice, set in the config or with `--n-motors`. It is enumerated with the prop and
the cell count (D4); it is not a DV.

| `n_motors` | Layout | Stations (`motor_stations`) |
|---|---|---|
| 1 | centreline nose tractor (default) | [0] |
| 2 | symmetric pair of wing-mounted tractors | [−y_motor, +y_motor] × b/2 |

Any other value is rejected. The units are identical, and the operating point is solved once, for one unit.

Outputs come at three levels:
- **total:** thrust, battery power, energy, battery mass, `m_motor_group`, `m_sys`;
- **per unit:** rpm, `I_static`, `T_static_unit`, `m_motor_unit` (motor + ESC + prop + nacelle), `D_prop`;
- **per pack:** `I_pack_static`.

m_sys = n (m_motor + m_esc + m_prop + m_nacelle) + m_batt,total + m_avionics. `m_nacelle` is 0 for the nose
layout, where the nose structure is in G&G's fuselage.

### Battery topology (`battery_topology`)

**`capacity` is the capacity of ONE pack** in both topologies.

**`per_motor` (default):** one pack per unit, as on Woombah (2025 rule C-F2). Each unit sees its own pack, so the
per-unit operating point is exactly the single-unit one. Thrust, power, energy and battery mass are n× per unit.

**`shared`:** one pack feeds every ESC. With an ideal ESC:
- the pack current is n δ I, so the terminal voltage is V_term = V_oc − n δ I R_b;
- each motor obeys δ V_term = ω/K_v + I R_m.

Eliminating V_term:

    I = (δ V_oc − ω/K_v) / (R_m + n δ² R_b),     I_pack = n δ I,     P_batt = n V_oc δ I

Each motor sees n times the pack resistance. So, at the same cells, the per-unit thrust is lower than with
`per_motor`. Energy and mass are those of one pack. (`tests/test_twin_motor.py` checks this against an
independent root solve.)

**Current limits:**
- `I_max` applies to each ESC's input current, δ I (`I_static`).
- `pack_I_max` (PLACEHOLDER, off by default) adds a pack-current post-check, `I_pack_margin`.

**Energy:** `E_nom_total` is the nominal energy of all packs, in Wh. It is there for the step 9 rule C-F1 cap
(≤ 100 Wh) and is not constrained unless a config asks.

## Constraints in the full model

| Constraint | How it's checked |
|---|---|
| Static thrust ≥ take-off requirement | ground roll of `takeoff_distance`; Josh's Eq. 3.3 |
| RPM at the top speed ≤ 0.85 × APC limit | rotor speed rises with flight speed, so it's checked at the top speed |
| Static current ≤ I_max | per motor/ESC; post-check: no design variable changes it at a fixed prop, cell count and throttle |
| Pack current ≤ pack_I_max | optional (shared topology); post-check |
| Three laps' energy ≤ (1 − reserve) × usable energy | energy reserve is a placeholder (20%) |

**Discrete choices** (prop, cell count, motor, `n_motors`) are enumerated outside the optimiser (D4): `examples/full_mdo.py`
or `aircrafter2 run --prop`.

## Placeholders (OPEN_QUESTIONS Q7, Q8, Q23)

Motor Kv/Rm/I0/mass/I_max, cell voltage/resistance/specific energy, ESC/prop/avionics masses. Lap time is most
sensitive to Kv (normalised sensitivity −0.25), so replace that first.
