# Mass: Garcia & Gamboa (2022)

Code: `aircrafter2/mass/`. Source: Garcia, G.L. & Gamboa, P.V. (2022), *Mass prediction models for air cargo
challenge aircraft*, Aeronaut. J. 126, 249–296, doi:10.1017/aer.2021.68. "Eq. N" below refers to that paper.

## What it does

A structure-based (class II½) mass model for a **circular carbon-tube spar** wing:
1. Size the structure from the loads: tube wall, rib thickness, tail-boom wall.
2. Turn the sized structure into component masses.
3. Add the tails, fuselage (boom + cargo bay), landing gear and systems.
4. Close the loop on total mass: the loads depend on the weight, and the weight on the masses.

## Equations by component

| Component (code) | Equations | Notes |
|---|---|---|
| `Allowables` | 42 | Allowable = ultimate / SF; moduli not reduced. |
| `SectionCoeffs` | 9, 10, 11, 15, 16 | Uses A/c² and P/c from the airfoil coordinates (PLAN G2). Eq. 16 typo (A_le) corrected (G3). |
| `TubeRadius` | (D1) | r = k_fill · (t/c at spar) · c_root / 2 − clearance. |
| `WingLoads` | 43–50 | Root values. Uniform lift. q at the design speed; q_div at 1.2 × design speed. |
| `CriticalLoads` | 51–53 | KS max (smooth) or hard max. |
| `TubeSparSizing` | 105–118 | Bending, tip deflection, shear, torsion, optional twist, divergence; minimum gauge. |
| `RibSizing` | 97–104 | Total n_rib · t_rib, so no integer rib count. |
| `WingMass` | 26, 27, 28, 30, 31, 32 | f_wing^(n_panels−1), confirmed as an exponent (G5). |
| `TailMass` | 34, 35 | |
| `TailBoomLoads` / `TailBoomSizing` | 54–56, 119–123 | Minimum boom wall (`tb_min_gauge`). |
| `FuselageMass` | 36–38 | |
| `GearMass` | 39 | Uses the coupled total mass. |
| `MassSum` | 40–41 | Optional `m_ballast` robustness hook (not G&G). |
| `StatisticalMass` | 124–126 | Initial guess for the closure and a sanity check. |
| `calibration.fit_correction_factors` | 127–128 | Offline refit of the correction factors. |

**Closure.** The group uses block Gauss–Seidel with Aitken acceleration when standalone. Inside the full model it
is a subsystem of the MDF Newton solver.

## Interpretation switches

Defaults reproduce the paper. Each switch is a `mass.options` key.

| Option | Default | Meaning |
|---|---|---|
| `tail_boom_load_mode` | `paper` | Eq. 55 with K_ht from Eq. 5 as printed (reproduces Table 15). `eq54` uses the consistent moment balance (needs a recalibrated f_tb). |
| `sizing_radius_mode` | `paper` | K_tube = r / c̄ times c_root (Table 15). `physical` uses r. The two are identical for a rectangular wing. |
| `deflection_mode` | `paper` | Printed Eq. 110 (G10). `exact` uses Euler–Bernoulli. |
| `torsion_stiffness_mode` | `paper` | Eq. 115. `bredt` uses GJ = 2π r³ G t (G12). |
| `include_twist` | `false` | Eq. 116 (the paper used divergence only). |
| `use_min_gauge` / `tb_min_gauge` | `true` / `true` | Spar and boom minimum walls (t_tube_min, t_tb_min). |
| `max_mode`, `rho_ks` | `ks`, 100 | Smooth max (log-space KS, over-estimate ≤ 0.7% for two terms). |

## Wing-mounted propulsion units (twin layout, not in G&G)

G&G size the root for uniform lift with the whole weight on the centreline. With `n_motors: 2`, `WingLoads` gets
one unit per semi-span. Each unit (motor + ESC + prop + nacelle, mass m_u = `m_motor_unit`) sits at
y_m = `y_m` and x_m = `x_motor`.

**Free body.** Take the semi-span outboard of the root.
- x is measured from the LE, positive aft.
- Torsion is about the quarter-chord line, as G&G do (Eqs. 46 and 50).
- Signs: shear Q is positive up, bending M positive tip-up, torque T positive nose-up.
- In flight the semi-span carries:
  - uniform lift nW/2;
  - the unit's inertial load n g m_u, downward at (y_m, x_m).
- In the ground wingtip-suspension test it carries:
  - the tip reaction W/2, upward at (b/2, x_gd c);
  - the unit's weight g m_u, downward at (y_m, x_m), with no load factor.

| Switch | Equation | Published (root) | Added term |
|---|---|---|---|
| `motor_inertia_relief` (default **off**) | 44 | Q_fl = nW/2 | − n g m_u |
| | 45 | M_fl = nWb/8 | − n g m_u y_m |
| | 48 | Q_gd = W/2 | − g m_u |
| | 49 | M_gd = Wb/4 | − g m_u y_m |
| `nacelle_torsion` (default **on**, does nothing without wing units) | 46 | T_fl = q\|C_m\|c²b/2 | + n g m_u (c/4 − x_m) |
| | 50 | T_gd = (W/2) c \|1/4 − x_gd\| | the signed sum \|(W/2) c (1/4 − x_gd) − g m_u (c/4 − x_m)\| |

Notes on the table:
- **Eq. 46:** a unit ahead of the axis under positive n pitches the section nose-down, the same sense as
  C_m < 0, so the two torques add. This assumes C_m ≤ 0, the same assumption G&G make when they take |C_m|.
- **Eq. 50:** the tip-reaction torque changes sign with x_gd. The terms add for x_gd > 1/4 and partly cancel for
  x_gd < 1/4.
- **Relief in the ground case:** relative to the root moment, it is twice the flight relief (g m_u y_m on Wb/4,
  against n g m_u y_m on nWb/8).

Why two switches: at the step 7 scale, the nacelle torque is about 3.5× G&G's aerodynamic torsion. A single
switch defaulting to off would under-size t_T (PLAN §6h T3). With both defaults, every load is conservative.

**Not modified:**
- **Eq. 110 deflection:** uses nW and ignores the point relief, which is conservative.
- **Eq. 117 divergence:** unchanged.
- **Thrust and motor-torque loads** on the wing are not modelled (limitation).
- **Batteries** stay at the fuselage `x_batt`. Wing-mounted packs are an option for later (OPEN_QUESTIONS Q27).

The partials of the new terms are analytic (`loads.py`) and checked with complex step
(`tests/test_twin_motor.py`). With no wing units (`n_motor_pairs = 0`) the component has its original inputs, and
all the G&G verification tests are unchanged.

## Defaults and calibration

`configs/mass_garcia_gamboa_2022.yaml` holds the defaults:
- materials: Table 13;
- correction factors: Table 14;
- f_tb = 0.847: refitted for a 1 mm boom wall from the USYD tubes (`docs/tail_boom_calibration.md`). The paper's
  12.81 applies only to load-sized walls.

All of these are placeholders until the model is calibrated on USYD aircraft. Use
`calibration.fit_correction_factors` with your real component masses (Eq. 127 for one aircraft, Eq. 128 for
several).

## Verification

- **Table 15** (UBI11, GRA13, UBI15): every component within 2.3% (`docs/mass_verification.md`). This needed three
  documented readings of the paper (PLAN G19, G22, G23).
- **Independent reference implementation:** agrees to 1e-8 across all switch combinations.
- **Calibration:** recovers the paper's Table 14 individual factors from Table 15's real masses.
- **Derivatives:** `check_partials` (complex step) on every component, and `check_totals` through the closure.
