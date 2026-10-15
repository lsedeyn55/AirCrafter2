# Stability (`aircrafter2/stability/static.py`)

- **Neutral point:** x_np = x_ac,w + η_h V_h (a_h/a_w)(1 − dε/dα) c, from Nelson, *Flight Stability and Automatic
  Control*, Ch. 2, and Raymer Ch. 16. a_h is Helmbold's low-AR slope, dε/dα = 2a_w/(πA), and a_w is the coupled
  VLM C_L,α.
- **CG:** mass-weighted component stations. The battery and cargo-bay stations are design variables, so the
  optimiser can trim the CG.
- **Motor group:** n × m_motor_unit at `x_motor`, which is measured per unit. Both layouts are symmetric, so
  there is no lateral CG. For the twin config, `x_motor` defaults to −5.5" (Woombah's firewall, 5.5" ahead of
  the LE; PLAN §6h T2).
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
