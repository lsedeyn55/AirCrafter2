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
