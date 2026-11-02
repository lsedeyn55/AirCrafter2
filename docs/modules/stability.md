# Stability (`aircrafter2/stability/static.py`)

- **Neutral point:** x_np = x_ac,w + η_h V_h (a_h/a_w)(1 − dε/dα) c, from Nelson, *Flight Stability and Automatic
  Control*, Ch. 2, and Raymer Ch. 16. a_h is Helmbold's low-AR slope, dε/dα = 2a_w/(πA), and a_w is the coupled
  VLM C_L,α.
- **CG:** mass-weighted component stations. The battery and cargo-bay stations are design variables, so the
  optimiser can trim the CG.
- **Motor group:** n × m_motor_unit at `x_motor`, which is measured per unit. Both layouts are symmetric, so
  there is no lateral CG. For the twin config, `x_motor` defaults to −5.5" (Woombah's firewall, 5.5" ahead of
  the LE; PLAN §6h T2).
- **2025-26: the rigid payload fuselage** (`stores2026.Packaging2026`, docs/RESULTS_2026_PLAN.md "Final fixes" D2).
  - The payload fuselage moves as one body: `x_cb_frac` (a design variable) is where the wing sits along it. The box
    structure (x_cb = x_cb_frac c), the payload (x_pax, x_car = (frac + Δ) c, Δ = x_cb_frac − 0.416) and the motor
    group move together; the banner station stays fixed.
  - x_motor = x_motor_ref + (x_cb − x_cb_ref) − (l_cb − l_cb_ref)/2: the motor sits at the box front at Humpback's
    nose length (0.192 m) and moves forward as the box grows forward.
  - Constraints (2025-26 only): `batt_fwd_margin` = x_batt − (x_motor + d_batt) ≥ 0 (d_batt 0.1005 m, Q64),
    `batt_aft_margin` = (x_cb + l_cb/2) − x_batt ≥ 0, and `tail_margin` = (x_tail − 0.25 c_ht) − (x_cb + l_cb/2) −
    tail_clear ≥ 0 (c_ht = √(S_ht/AR_ht), tail_clear 0, Q64).
  - Every shift is a difference from Humpback's layout, so it is exactly 0.0 there and Humpback as built reproduces
    bit for bit. The 2024-25 and single-point stations are unchanged.
- **Static margin:** SM = (x_np − x_cg)/c.
- **Constraints:** SM, V_h and V_v bands. All are placeholders (Q9).
- **AVL later:** an AVL-based component only needs to output x_np (and could add lateral derivatives); the
  interface is in the module docstring.

## One-engine-inoperative yaw (twin, report only)

`oei_check(prob, cfg)` is a post-check; the optimiser does not see it. As far as we know DBF has no OEI
requirement, and the Woombah report doesn't mention one. The model:
- yawing moment from the live unit: N = T_static,unit × y_m, using static thrust as the worst case and
  neglecting the windmilling drag of the dead prop;
- required yaw coefficient: C_n,req = N/(q S b), at V = k V_stall;
- rudder power: C_n,δr = η_v V_v a_v τ_r, with a_v from Helmbold on AR_vt;
- required rudder: δ_req = C_n,req / C_n,δr.

η_v, τ_r, δ_r,max and k are PLACEHOLDERS (`StabilityInputs`: `eta_v`, `rudder_tau`, `rudder_max_deg`,
`oei_speed_factor`). The result is reported in `results.json` under `oei`.
