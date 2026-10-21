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

**Energy:** `E_nom_total` is the nominal energy of all packs, in Wh. It is there for the competition rule C-F1 cap
(≤ 100 Wh) and is not constrained unless a config asks.

## Motor Kv as a design variable (`motor_scaling.py`, default)

By default `Kv` is a continuous design variable of the full problem (`default_full_dvs()`, and the
`design_variables` block of `configs/full_mdo.yaml` / `full_mdo_twin.yaml`; the reference motor's Kv, Rm and I0 are in `configs/placeholders.yaml`): the optimiser picks the winding for the
given frame, prop and cell count. The config's motor (`propulsion.Kv`, `Rm`, `I0`) is then a **reference motor**
(Kv_ref, Rm_ref, I0_ref), and `MotorRewind` gives the powertrain the motor that the same frame becomes when it is
rewound to the design Kv.

**Derivation.** A brushless motor with N turns per phase on a fixed stator and magnets:
- back-EMF and torque constants are proportional to N, so **Kv ∝ 1/N**;
- at the same copper volume (same slots, same fill factor), each turn's wire is 1/N times the cross-section and
  the total wire length is ∝ N, so the winding resistance is **Rm ∝ N · N = N² ∝ 1/Kv²**;
- the no-load current I0 comes from losses that do not depend on the winding (iron, bearings, windage), i.e. a
  no-load torque Q0 that stays the same; Q0 = I0 / Kv (Kv in rad/s/V), so **I0 = Q0 Kv ∝ Kv**.

So, with a winding tolerance factor `Kv_factor` (1 except at robust-design nodes, applied first so a Kv error moves
Rm and I0 consistently):

    Kv_eff = Kv · Kv_factor,     Rm = Rm_ref (Kv_ref / Kv_eff)²,     I0 = I0_ref Kv_eff / Kv_ref,     m_motor unchanged

The motor constant Km = Kt/√Rm = 1/(Kv √Rm) is unchanged: a rewind trades current for voltage, not efficiency.
Kv = Kv_ref reproduces the reference motor exactly (`tests/test_kv_dv.py`).

**Assumptions:** the same frame (stator, magnets, mass); the same copper fill and wire type (no change of slot fill
with wire gauge, no change of end-turn fraction); no change in iron losses with the winding (I0 is quoted at one
test voltage, and its speed dependence is not modelled, as in the first-order model itself); ideal scaling of turns
(real windings come in whole turns and standard gauges, so catalogue Kv values are discrete). Source for the scaling
laws: [CITATION NEEDED] (standard brushless-motor rewinding relations; not verified against a reference here).

**Real motors.** Kv stays continuous in the optimiser. `api.reanalyse_with_motor(cfg, dv, Kv, Rm, I0, m_motor)` (also
on the UI results page) re-analyses a design, without optimising, with a real motor, and reports the score and
constraint changes. No motor catalogue is built in (OPEN_QUESTIONS Q7).

**Fixed motor.** Leave `Kv` out of `design_variables`: `configs/full_mdo_fixed_kv.yaml` and
`full_mdo_twin_fixed_kv.yaml` are the defaults with `fixed_design_variables: [Kv]`, which removes the inherited DV
(the loader cannot delete keys otherwise). Then no `MotorRewind` is built, `Kv`, `Rm` and `I0` are plain inputs, and
results are bit-for-bit those of the 10-DV problem (`tests/test_rdo.py` regression). The pinned studies (twin
motor, RDO, the VSP/AIAC figures) use these configs.

**Robust design.** With Kv a DV the design Kv cannot also be an uncertain parameter (the robust nodes share the design
variables). Its uncertainty is `Kv_factor` and the resistance uncertainty moves to `Rm_ref` (configs/uncertainty.yaml,
`kv_mode`; docs/modules/robustness.md).

## Constraints in the full model

| Constraint | How it's checked |
|---|---|
| Static thrust ≥ take-off requirement | ground roll of `takeoff_distance`; Josh's Eq. 3.3 |
| RPM at the top speed ≤ 0.85 × APC limit | rotor speed rises with flight speed, so it's checked at the top speed |
| Static current ≤ I_max | per motor/ESC. Kv a DV (default): an optimiser constraint `I_margin ≥ 0` (ref 10 A). Fixed motor: a post-check (no design variable changes it at a fixed prop, cell count, throttle and Kv) |
| Pack current ≤ pack_I_max | optional (shared topology); a constraint with Kv a DV, else a post-check |
| Three laps' energy ≤ (1 − reserve) × usable energy | energy reserve is a placeholder (20%) |

**Discrete choices** (prop, cell count, `n_motors`) are enumerated outside the optimiser (D4): `examples/full_mdo.py`
or `aircrafter2 run --prop`. The motor's Kv is continuous (above); the motor frame is the reference motor.

## Placeholders (OPEN_QUESTIONS Q7, Q8, Q23)

Motor Kv/Rm/I0/mass/I_max (with Kv a DV: the reference motor and the Kv bounds, 500-1500 rpm/V, Q34), cell
voltage/resistance/specific energy, ESC/prop/avionics masses. With a fixed motor the lap time is most sensitive to Kv
(normalised sensitivity −0.25), so replace that first.
