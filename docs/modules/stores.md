# Stores (`aircrafter2/stores.py`, `aircrafter2/stores2026.py`)

What a mission point carries outside the base aircraft: its mass, CG station and drag. `build_full_model` adds one of these
when it's given a store set. `problems/multipoint.py` chooses the flags per mission.

## 2024-25: tanks, pylons and the X-1 (`stores.py`)

Why this formulation: [r25-multipoint](../DECISIONS.md#r25-multipoint).

**Rules (`Rules_2025.pdf`):**
- Tanks are commercial beverage bottles (≥ 16 fl oz, all external tanks the same bottle for all missions).
- At least two external tanks in M2 and M3 (empty allowed in M3), on removable pylons. The declared pylon set is fitted for
  every M2 and M3 attempt (p. 8–10).
- At most one internal tank, any bottle (p. 8).
- The X-1 (≤ 0.55 lb) flies under the fuselage between the external tanks in M2 and M3 (p. 8–9, p. 11).
- M1 flies with no payload and no pylons (p. 10–11).

**Model, per mission point** (flags say which stores are on board):

    mass   m_pay = n_ext m_bottle + m_internal_bottle + m_fuel + m_pylons + m_X1 + m_X1_structure
    CG     x_pay = sum m_i x_i / m_pay, stations as chord fractions from the wing LE (Woombah Table 34 / c)
    drag   CD_stores = f_drag [ n_ext Q_tank C_D,bottle A_bottle                   (pressure drag, frontal area)
                              + n_pyl Q_pyl C_f(Re_l) FF_wing S_wet,pyl            (Raymer component build-up)
                              + Q_X1 (C_f FF S_wet)_X1 wing + body ] / S

- **C_D,bottle** = 0.523 per bottle, on the frontal area, bottom first: Woombah CFD, validated there against Hoerner's 0.82
  for a long cylinder (2025 USYD DBF Design Report, p. 43). Woombah's FlightStream build-up gives a far smaller bottle term
  (Table 26, p. 28: CD0 0.00045 on the wing area for both bottles, about 1/20 of the CFD value). The CFD value is used.
- **Pylons and the X-1:** Raymer, *Aircraft Design: A Conceptual Approach*, Ch. 12, as in `aero/parasite.py`.
- **f_drag** (the store drag factor) is an uncertain parameter in the robust problem. Interference factors are PLACEHOLDERS
  (`docs/OPEN_QUESTIONS.md`).

All arithmetic is complex-step safe, with `method="cs"` partials.

## 2025-26: payload, banner and payload fuselage (`stores2026.py`)

Why this formulation: [r26-payload](../DECISIONS.md#r26-payload), [r26-banner](../DECISIONS.md#r26-banner),
[ff-d2](../DECISIONS.md#ff-d2). Plan: `docs/RESULTS_2026_PLAN.md` §4.2–4.3.

**Rules (`2026-DBF-Rules.pdf`, printed page "N of 34"):**
- M2 carries passengers (2.3 in rubber ducks, 0.6–0.7 oz) and cargo (hockey pucks, 6 oz) in a single passenger compartment
  and separate cargo bay(s), capable of ≥ 3 passengers per declared cargo, ≥ 3 passengers and ≥ 1 cargo (p. 10–11). The
  restraints stay in the aircraft (p. 10–11).
- M3 tows a banner (length ≥ 10 in, length / height ≤ 5), stowed externally for take-off and deployed on the downwind leg
  after the first upwind turn (p. 12, p. 20).
- M1 flies with no payload (p. 19).

**Model, per mission point** (flags: `payload` = ducks and pucks on board, M2; `banner` = the banner on board, M3):
- **Fuselage** (`PayloadFuselage`, ahead of the planform and the MDA). The cargo box is the constant-section payload fuselage
  (the calibration convention), sized by the payload at Humpback's packing ([HB] §4.2, §5.1): l_pax = `pax_pitch` n_p (rows of
  3 abreast), l_car = `car_pitch` n_c (two bays of 7 side by side below the passengers), l_cb = KSmax(l_pax, l_car), with w_cb
  and h_cb held at Humpback's maximum width and height. n_p and n_c are the design's capacity, shared by every point.
- **Mass:** restraints n_p m_seat + n_c m_slot at every point. M2 adds n_p m_duck + n_c m_puck. M3 adds the banner,
  m_b = ρ_A L H + m_ballast + m_mech, H = L / AR_banner.
- **CG:** x_pay = Σ m_i x_i / m_pay, stations as chord fractions from the wing LE ([HB] Table 36 / chord).
- **Drag:** the deployed banner on its own area, CD_banner = f_b C_D,b L H / S (M3; f_b is the banner-drag factor, an uncertain
  parameter), and the stowed banner's ΔC_D0 (`CD_stowed`). The M3 point's steady (cruise) state is the tow, so
  `CD_stores` = `CD_banner`. The lap model flies lap 1's legs before deployment with `dCD_stowed` = CD_stowed − CD_banner added
  (`mission/smooth_lap.py`, `banner_stowed_legs`).
- **Packaging** (`Packaging2026`, after the MDA): see [ff-d2](../DECISIONS.md#ff-d2) and the class docstring.

All arithmetic is complex-step safe, with `method="cs"` partials.
