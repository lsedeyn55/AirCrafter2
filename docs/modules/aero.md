# Aerodynamics and airfoil

Code: `aircrafter2/airfoil/`, `aircrafter2/aero/`.

## Airfoil (`airfoil/`)

- **Shape.** Kulfan/CST "with LEM" exactly as in AeroSandbox, which NeuralFoil was trained on: 8 + 8 weights, a
  leading-edge weight and the TE thickness (`cst.py`; matches AeroSandbox to 1e-16).
- **Design variables (D3).** On a seed airfoil (`airfoil.seed`), `tc_scale` scales the thickness distribution and
  `camber_scale` the camber distribution. Optionally, `n_modes` local bump modes are added per surface. The TE
  thickness is fixed (buildability).
- **Section geometry.** Max t/c, its position, the thickness at the spar, A/c² and P/c, which feed the mass
  model. Partials are exact (complex step).
- **Section aerodynamics.** NeuralFoil (v0.3.3, `model_size` xlarge by default) at the VLM strip's own c_l and Re:
  1. α is solved so that c_l,NeuralFoil(α) = c_l (grid bracket, then Newton);
  2. c_d and c_m are read there;
  3. c_l,max is the C_l(α) peak.
- **Derivatives.** Implicit-function theorem: only the network is finite-differenced. Step sizes are α 1e-4°,
  ln Re 1e-5 and Kulfan weights 1e-6, from `docs/verification/neuralfoil_fd_steps.md` (the TE-thickness derivative needs
  ≤ 1e-6).

## Wing (`aero/wing.py`)

1. **VLM.** OpenAeroStruct `AeroPoint`, aero only, `with_viscous=False`, on a rectangular half-wing mesh
   (`num_y` = 41 nodes by default; 21 in the full optimisation config).
2. **Profile drag.** Strip integral C_Dp = 2 Σ c_d,i c_i w_i / S of the NeuralFoil section drag. OAS's own
   viscous drag is off, so profile drag isn't counted twice.
3. **C_L,max** by the critical-section method: C_L,max = C_L · KSmin(c_l,max,i / c_l,i). This is exact for the
   linear VLM.
4. **Moment and lift slope.** C_m,wing (chord-weighted strip c_m) and C_L,α = C_L/α, both used by the mass loads.
5. **Parasite drag** (`aero/parasite.py`): Raymer's component build-up (Ch. 12) for the tails and a switchable
   fuselage (cargo box + boom). Fully turbulent C_f with thin-surface and body form factors.
6. **Drag polar for the mission** (`aero/polar.py`): CD = CD0 + CD1 C_L + k C_L², fitted exactly through cruise, mid
   and 0.95 C_L,max points, so the lap model interpolates and never extrapolates.

## Nacelle drag (twin layout)

For `n_motors: 2`, `ParasiteDrag` adds C_D,nac = 2 C_f(Re_l) FF Q_nac π d l / S. This is Raymer's Ch. 12
build-up for a cylinder of length l and diameter d. The output is `CD_nac`, which is included in `CD_extra` and
drawn in the drag breakdown. For the nose layout, C_D,nac = 0: the nose is in the fuselage term.

| Input | Default | Source |
|---|---|---|
| `Q_nac` | 1.3 | Woombah Sec. 4.3.2: Q ≈ 1.3 with motors 5.5" ahead of the LE (≈ 1.5 at the LE), after Raymer |
| `l_nac`, `d_nac` | 0.15 m, 0.05 m | PLACEHOLDERS: not in the Woombah report |
| `nacelle_ff` | `body` | as briefed: the body form 1 + 60/f³ + f/400; `nacelle` uses Raymer's nacelle/store form 1 + 0.35/f |

**Cross-check against Woombah Table 26** (M1, two nacelles, C_D0 = 0.00069 on S = 914.5 in², 87 mph), using the
placeholder size:
- the `nacelle` form gives 0.00062;
- the `body` form gives 0.0018, 2.6× more. The body formula is meant for slender bodies; at f = 3 it gives
  FF = 3.2.

Woombah's nacelle dimensions aren't published, so this only shows the placeholders are plausible (PLAN §6h T6).

**Slipstream is not modelled.** See PLAN §6h for the proposal, and for why leaving it out is only partly neutral
in a single-versus-twin comparison.

## Assumptions to know

- **The VLM is a flat plate.** α is measured from zero lift; camber enters only through NeuralFoil. `alpha_sec`
  gives the true section angle.
- **C_L,max uses the analysis point's Reynolds number**, which is optimistic at stall.
- **Rectangular wing only** (multi-panel tapered wings were dropped).

## Verification (`docs/verification/aero.md`)

| Check | Result |
|---|---|
| Trefftz-plane drag vs OAS near-field drag | −0.3% |
| Span efficiency e, VLM vs lifting line (AR 6) | +4.7% (a genuine difference between the theories) |
| Strip C_Dp vs 2D c_d at the wing C_L | +4.6% (tip strips run off-design) |
| Total C_D vs the hand calculation | −0.7% |
| Mesh convergence | tabulated in the report |
| `check_partials` / `check_totals` | pass |

One OAS gotcha: OAS reads the *values* of the template mesh passed in the surface dict, not just its shape. A zero
placeholder gives garbage (PLAN A1).
