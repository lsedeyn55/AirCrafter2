# Problems (`aircrafter2/problems/`)

The builders, from aero-only up. Each takes a config and returns a set-up OpenMDAO problem. Generated reports for the first
four are in `docs/problems/`.

## `wing_aero`

    minimise   CD                       (= drag, since q and S are fixed)
    w.r.t.     b, c, alpha, tc_scale, camber_scale (and bump modes, if any)
    subject to L = W                    (fixed lift)
               b c = S                  (fixed wing area, i.e. fixed wing loading)
               tc_max >= tc_min         (room for the spar; PLACEHOLDER, aero_mass couples mass)
               CL_max / CL >= 1.2^2     (cruise at >= 1.2 V_stall, as in the lap modeller)
               cl_margin_sec >= 0       (no stalled strip)

## `oval_lap`

One wing and airfoil, two aero points at the same speed:

    cruise: alpha_cruise such that L = W                (equality constraint)
    turn:   alpha_turn; load factor n = L_turn / W (KS-capped at n_struct)

    minimise   lap_time = (2 L_s + 2 pi R) / V,   R = V^2 / (g sqrt(n^2 - 1))
    w.r.t.     b, c, alpha_cruise, alpha_turn, tc_scale, camber_scale
    subject to L_cruise = W
               CL_max / CL_turn   >= turn_clmax_ratio_min   (the C_L margin that sets the turn radius)
               CL_max / CL_cruise >= 1.44                    (cruise at >= 1.2 V_stall)
               tc_max >= tc_min                              (spar placeholder)
               cl_margin_sec >= 0 at both points
               P_turn = D_turn V / eta <= P_available        (optional)

Mass is fixed (W) here. Without a power limit the problem favours a large wing, because at fixed speed only the turn radius
matters, and a larger wing has more lift to spare. The bounds on b and c, plus the optional power constraint, keep it finite.

## `aero_mass`

    airfoil (shape DVs -> Kulfan, t/c, t at spar, A/c^2, P/c)
    geometry (b, c, l_ht, tails, cargo bay -> S, chords, l_tb, S_cb, V_h, V_v)
    MDA  [Newton, solve_subsystems, DirectSolver]
        cruise aero point  (alpha_cruise)  -> L, D, C_m,wing, C_L,alpha, C_L,max
        balance            alpha_cruise such that L = W
        G&G mass group     (its own NLBGS closure on m)  -> W, masses, tube sizing

The optimisation minimises cruise drag at fixed speed. With mass in the loop, a thicker section costs profile drag but buys a
lighter spar (larger tube radius), which lowers W and with it the induced drag. That's the trade this problem exists to show.

## `lap_mdo`

    [aero_mass model: airfoil, geometry, MDA{cruise aero, L = W balance, G&G mass}]
    mid and near-C_L,max aero points -> three-point drag polar CD = CD0 + CD1 CL + k CL^2
    SmoothLapModel(m = m_total, S, V, polar, CL_max)  -> Lap1time, ThreeLapTime, laps_frac, margins

    minimise   ThreeLapTime  (placeholder objective)
    w.r.t.     b, c, tc_scale, camber_scale, V
    subject to stall_margin >= 0 (V >= 1.2 V_stall), turn_speed_margin >= 0,
               Preq <= power_available (fixed PLACEHOLDER: no propulsion model here),
               energy for 3 laps <= usable energy, spar wall ratio and minimum tube radius,
               no stalled strip at cruise.

## `full_mdo`

The full problem. The powertrain sets the power ([plan-b](../DECISIONS.md#plan-b)).

    airfoil, geometry (planform, tails, cargo bay)
    powertrain (APC prop + motor + battery at fixed throttle) -> T(v), P(v), T_static, masses
    MDA  [Newton, solve_subsystems]:
        parasite drag (tails, optional fuselage)  at v
        cruise aero point                         at (alpha_cruise, v)
        balance alpha_cruise:  L = W
        balance v:             T(v) = D(v)        (v: the full-throttle level speed, also the G&G design speed)
        G&G mass (NLBGS closure), m_sys from the powertrain
    mid / near-C_L,max aero points -> three-point polar
    lap model (propulsion mode): T(v) and P(v) on every leg, battery energy, laps
    static stability: neutral point, CG, static margin

**Design variables:** b, c, tc_scale, camber_scale, battery capacity, tail arm, S_ht, S_vt, battery and cargo-bay stations, and the
motor Kv (by default; leave `Kv` out of `design_variables`, as in `configs/full_mdo_fixed_kv.yaml`, for a fixed motor). With Kv a
design variable, the config's motor is a reference motor and a `MotorRewind` component scales Rm and I0 with Kv (the same frame
rewound; `propulsion/motor_scaling.py`), and the static current limit becomes a constraint.

**Objective:** a scoring plug-in (`aircrafter2.scoring`; placeholder default `three_lap_time`).

## `multipoint`

Why: [r25-multipoint](../DECISIONS.md#r25-multipoint). Rules and sources: `docs/development/RESULTS_PLAN.md`, `docs/modules/scoring.md`. The notes
below are the 2024-25 ruleset; 2025-26 has the same structure with its own stores, extra variables and score
(`docs/modules/stores.md`).

    model
      points  (om.ParallelGroup; serial without MPI)
        M2   full coupled analysis (full_mdo.build_full_model) with the M2 stores: filled tanks, pylons, X-1.
             The structure is sized here (the heaviest case: maximum fuel and battery, the rules' wing-tip test case,
             Rules_2025 p. 9 and p. 17).
        M3   full coupled analysis with the M3 stores: empty tanks, pylons, X-1. Flies M2's airframe: its structural
             sizing inputs are M2's mass and speed (sizing="external"), so the structure is the same.
      scoring (dbf2025): F = M1 + M2 + M3 from M2's fuel weight and 3-lap time and M3's laps
      rules   TOGW < 55 lb (M2), total propulsion energy <= 100 Wh and fuse current <= min(100 A, C x capacity) at each
              point, 3 laps in the M2 window

**Design variables:**
- **Shared:** the airframe of the base config (b, c, tc_scale, camber_scale, l_ht, S_ht, S_vt, x_cb_frac) and the motor Kv: "the
  airplane must be flown in the same configuration for all three missions" (p. 10).
- **Per point:** capacity, x_batt, throttle (names suffixed `_M2`, `_M3`). Other inspected battery packs may be flown between
  attempts (p. 16), the battery station sets the CG per mission (Woombah: p. 19, Table 34), and the throttle is the pilot's setting
  for each flight.
- **M2 only:** fuel_mass, up to the declared tanks' capacity (`stores.StoreConfig.fuel_capacity`).
- **Enumerated:** the prop of each point (the prop may change per attempt, p. 14), cells, n_motors, the store set.

**M1** (3 laps in 5 min, no payload, no pylons; p. 10–11) is an analysis-only check of a final design (`m1_check`).
