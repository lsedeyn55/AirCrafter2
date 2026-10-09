# AirCrafter 2.0: Build Plan (Step 1: Explore and Plan)

Status: **draft for review**. No package code has been written yet. Everything below comes
from reading the sources listed in §1. Two small scratch scripts were also run (outside
the repo) to check equations from the Garcia & Gamboa (2022) paper; their results are
reported in §7.

Notation: "G&G" means Garcia, G.L. & Gamboa, P.V. (2022), *Mass prediction models for air cargo
challenge aircraft*, Aeronaut. J. 126, 249–296, doi:10.1017/aer.2021.68. "Eq. N" always refers
to a G&G equation unless stated otherwise.

---

## 1. Sources read

| Source | Status | Notes |
|---|---|---|
| `AirCrafter/*.m` (AirCrafter, performance_modeller, static_thrust_calculator, leg_calc_straight/turn, inputs_file, assumptions_file) | read | See §8 for MATLAB quirks that matter for the step 6 reproduction check. |
| `AirCrafter/propSelector.m`, `parse_apc_txt_folder.m` | read | The APC data **is on disk**: `THESIS/PERFILES2/PER3_*.dat` (443 files, `PROP RPM =` blocks, 15 columns). The MATLAB parser globs `*.txt` and hard-codes a macOS path, so neither works here as-is. |
| `AirCrafter/Josh Kal's Thesis.pdf` Ch. 3 | read | `dbf_performance_modeller.py` matches Eqs. 3.1–3.12 and Table 3.5. |
| `AirCrafter2/dbf_performance_modeller.py` | read | Reused as the mission core (§5.7). |
| `AirCrafter2/{rect_wing,quick_example_oas,basic_mdo,basic_perf,draft}.py` | read | Patterns reused (§2). They will be moved to `examples/legacy/` unchanged. They import only external libraries, so the move doesn't break them. |
| G&G 2022 paper (all of it, including Tables 1–15) | read; key pages rendered to check garbled equations | Mass model spec (§6). Errata and ambiguities are in §7. |
| `0_DELIVERABLES/Preliminary Report/Thesis_Preliminary_Report.pdf` Ch. 3 | read | Formal problem and XDSM (Fig. 3.1). The XDSM is updated in §4. Two points where the brief supersedes the report are in §9 (B1). |
| `2_DBF_CODE/Spar Sizer 3.1/…/WingSparSizing_V3_0_1.m` (+ `getDeflection.m`, `runavl_C2.m`, `wing.avl`) | read (reference only) | Notes in §10. One of them conflicts with the single-tube G&G concept (§9, B2). |

Environment checks:
- `AirCrafter2/.venv` runs Python 3.14.7, with openmdao 3.45.0, openaerostruct 2.12.0, numpy 2.5.2, scipy 1.18.0,
  matplotlib 3.11.1 and pyyaml 6.0.3.
- NeuralFoil 0.3.3 and AeroSandbox 4.2.10 install and run on 3.14 (tested in a throwaway env). Not yet in the venv.
- Missing from the venv: pytest, streamlit, pyxdsm, pydantic, neuralfoil, aerosandbox. pyOptSparse is not available
  and I don't plan to require it (SLSQP is the default).
- MATLAB is installed (`/usr/local/bin/matlab`), so step 6 can generate reference outputs directly from the original
  code. That needs the Aerospace Toolbox for `atmosisa`.
- `THESIS/` is **not a git repository**. See OPEN_QUESTIONS Q1.

---

## 2. What gets reused

| Existing code | Where it goes | How |
|---|---|---|
| `dbf_performance_modeller.py` (whole file) | `aircrafter2/mission/lap_modeller.py` | Moved verbatim; it stays the *reference* implementation. Its static `Cd0/Clmax/e/m` assumptions become component inputs fed by aero, mass and propulsion. NaN-flag replacement (`stall_margin`, `turn_speed_margin`) is kept. |
| `isa_density()` from the same file | `aircrafter2/atmosphere.py` | Shared by all modules. |
| `basic_perf.py` `RequiredCL` and `LapTime` | `aircrafter2/mission/oval.py` | The step 4 "simple oval" model. Fixes: `np.where(n_max>1.01, …)` becomes a smooth lower bound, and the turn CL is set by a CL-margin parameter. |
| `basic_mdo.py` pattern (coupled group, `NewtonSolver` + `DirectSolver`) | `aircrafter2/problem.py` | The MDA solver setup. |
| `rect_wing.py` OAS wiring (Geometry → AeroPoint connections) | `aircrafter2/aero/oas_wing.py` | Same connections, but `with_viscous=False` and no `fem_model_type`. |
| `parse_apc_txt_folder.m` logic | `aircrafter2/propulsion/apc.py` | Python port that reads `PER3_*.dat`. The parsed table is cached to `data/apc/apc_per3.npz`. |
| `propSelector.m` RPM limit `RPM ≤ 0.85·150000/D[in]` | `aircrafter2/propulsion/` | Kept as a prop RPM constraint, with the source cited as the MATLAB tool (APC's published E-prop limit is 150000/D). |

---

## 3. Architecture and package layout

**MDF + gradient-based** (settled). One `om.Problem`. A coupled MDA group holds the feedback loop
(aero trim ↔ mass ↔ propulsion), and all other disciplines are feed-forward from it.

```
AirCrafter2/
├── pyproject.toml                # pinned deps (§12), console script `aircrafter2`
├── aircrafter2/
│   ├── __init__.py
│   ├── atmosphere.py             # ISA (from the port)
│   ├── smooth.py                 # KS / smooth max / softplus helpers + hard-max "report" mode
│   ├── config/                   # YAML loading + pydantic schema + defaults merging
│   ├── geometry/                 # planform, panels, OAS mesh, tails, boom (Eqs. 5–8 as corrected, §7 G1), wetted areas
│   ├── airfoil/                  # CST/Kulfan param → coords → K_A, K_P, t(x_spar) ; NeuralFoil polars; database mode
│   ├── aero/                     # OAS VLM (aero-only) + NeuralFoil strip profile drag + parasite drag + CLmax
│   ├── mass/
│   │   ├── garcia_gamboa/        # one component per block (§6)
│   │   ├── statistical.py        # Eqs. 124–126
│   │   └── calibration.py        # Eqs. 127–128 hook (scipy), not in the optimisation loop
│   ├── propulsion/               # motor (Kv/Rm/I0), APC prop tables, battery; plug-in base class
│   ├── stability/                # tail-volume neutral point, CG, SM; AVL-ready interface
│   ├── mission/                  # lap_modeller.py (reference port), smooth lap model, oval.py
│   ├── scoring/                  # base.py (plug-in ABC + registry), placeholder.py
│   ├── problem.py                # builds the om.Problem from a config (MDA, DVs, cons, objective, recorder)
│   ├── robustness/               # sensitivities, worst-case, Monte Carlo
│   ├── plotting/                 # thesis-style figures (vector PDF) + mass/drag breakdowns
│   ├── cli.py                    # `aircrafter2 run|check|sweep|figures configs/<name>.yaml`
│   └── ui/app.py                 # Streamlit app, thin over problem.py
├── configs/
│   ├── default.yaml
│   ├── mass_garcia_gamboa_2022.yaml   # Tables 12–14 defaults, each value cites its table
│   └── verification/{ubi11,gra13,ubi15}.yaml  # paper inputs (Tables 3, 12, 13, 14)
├── data/                         # apc cache, airfoil seeds (.dat), motor/battery catalogues (placeholders)
├── tests/                        # per-module unit tests + check_partials + check_totals
├── examples/legacy/              # the five earlier scripts, unchanged
└── docs/                         # README quickstart lives at repo root; one page per module here
```

**Units:** SI internally, angles in rad, and every OpenMDAO variable declares `units=`. The UI shows degrees
and km/h where that helps.

**Derivatives policy:**
- G&G mass equations, geometry, CST→coefficients, stability and the smooth lap model: analytic partials,
  checked with `check_partials(method="cs")`.
- NeuralFoil: FD with relative step 1e-6 on the Kulfan weights and 1e-4 on log(Re). The step size will be
  confirmed by a step-size sweep in step 3 and documented.
- OAS: its own analytic partials.
- Reference lap modeller: FD (it uses table lookups with while loops). That's why §5.7 proposes a smooth version.

---

## 4. Updated XDSM

This extends Fig. 3.1 of the preliminary report by adding the Airfoil, Mass, Stability and Scoring blocks. The
diagonal holds the blocks. Above the diagonal is feed-forward data; **bold** entries below the diagonal are the
feedback couplings that the MDA converges. A pyXDSM script will render this for the thesis in step 8.

| | Optimiser | MDA (Newton/NLBGS) | Geometry | Airfoil | Aero (cruise, turn, CLmax) | Propulsion | Mass (G&G) | Stability | Mission | Scoring |
|---|---|---|---|---|---|---|---|---|---|---|
| **Optimiser** | 0 → 10 | | b, AR, λ, l_ht, S_ht, S_vt | airfoil params (CST modes) | V_cruise | battery capacity; *(prop/motor: outer enumeration)* | r_tube *(optional DV)* | | V_cruise | |
| **MDA** | | 1 → 1 | | | **m (guess)** | **m (guess)** | **m (guess)** | | | |
| **Geometry** | | | 2 | | mesh, S, c̄, c_root, tail geom, S_wet | | b, c̄, c_root, S, panels, y₁, l_ht, l_tb, S_ht, S_vt, S_cb | x-stations, l_ht, S_ht, S_vt, c_mac | S, b | |
| **Airfoil** | | | | 3 | polar tables C_l, C_d, C_m(α, Re); C_l,max(Re) | | K_A, K_P, K_A,le/te/rib, t/c, t(x_spar) | | | |
| **Aero** | | | | | 4 | D_cruise, CD0/k for take-off | dC_L/dα, C_m, q, *(opt.)* L(y) | a_w | polar (CD0, k, CL_mind), CL_max, V_stall | |
| **Propulsion** | | | | | | 5 | m_sys | x_cg of battery/motor | η(V), P_max, T_static,avail, E_usable | |
| **Mass** | | | | | **W** | **m (T_static,req)** | 6 | component masses | m | |
| **Stability** | | | | | | | | 7 | | |
| **Mission** | | | | | | | | | 8 | ThreeLapTime, laps_frac, energy |
| **Scoring** | f | | | | | | | | | 9 |
| constraints → Optimiser | | | wing-loading, span bound | t/c ≥ t_min, TE, NF confidence | CL_cruise ≤ CL_max margin | thrust, power, current, energy margins | tube-fit, min-gauge margins | SM band, V_h, V_v bands | stall_margin, turn_speed_margin | |

The coupled set inside the MDA is {Aero trim, Propulsion, Mass}:
- **Aero ↔ Mass:** W sets the trimmed CL and α. Aero returns dCL/dα and Cm to the loads.
- **Propulsion ↔ Mass:** m sets the required static thrust and cruise power. Propulsion returns m_sys.

Geometry and Airfoil are upstream and explicit. Stability, Mission and Scoring are downstream with no feedback.
Cruise lift = weight is a `BalanceComp` on α inside the MDA, so the optimiser never sees an unconverged trim.

---

## 5. Module interfaces

All inputs and outputs are scalars unless a shape is given. "Config" means a fixed YAML value, wired in as an
`IndepVarComp` output so `compute_totals` can give sensitivities to it.

### 5.1 geometry
| | Name | Units | Source |
|---|---|---|---|
| in | `b`, `AR`, `taper` | m, –, – | DV / config |
| in | `n_panels`, `panel_breaks` (fractions of semispan) | –, – | config (discrete, not differentiated) |
| in | `l_ht`, `S_ht`, `S_vt`, `AR_ht`, `AR_vt` | m, m², m², –, – | DV / config |
| in | `x_spar_frac` | – | config (default 0.25) |
| in | fuselage: `l_nose`, cargo-bay box `l_cb, w_cb, h_cb`, `boom_start_x` | m | config |
| out | `S`, `c_mean`, `c_root`, `c_tip`, `c_mac`, `x_ac_w` | m², m | |
| out | `mesh` (OAS, nx×ny×3), `panel_span[j]`, `panel_chord[j]` | m | |
| out | `y1` (first panel-break position, for Eq. 110) | m | |
| out | `l_tb` (boom length; see §7 G1) | m | |
| out | `S_cb` (cargo-bay surface area), `S_wet_{wing,ht,vt,fus}` | m² | |
| out | `V_h = S_ht l_ht/(S c_mac)`, `V_v = S_vt l_vt/(S b)` | – | |

### 5.2 airfoil
Two modes: **continuous** (default) and **database** (outer loop over named airfoils; seed shapes in `data/airfoils/`).

Proposed parameterisation (decision D3): seed airfoil → `asb.KulfanAirfoil` (8 upper + 8 lower weights, LE
weight, TE thickness). Design variables are a small set of shape modes:
- `tc_scale`: scales the thickness distribution.
- `camber_scale`: scales the camber distribution.
- Optionally 2–4 Bernstein-mode perturbations each on the upper and lower surfaces.

The TE thickness is fixed from config. The full 18-weight CST stays available as an advanced option.

| | Name | Units | Partials |
|---|---|---|---|
| in | shape params (above) or `kulfan_upper[8]`, `kulfan_lower[8]`, `le_weight`, `te_thickness` | – | |
| out | `kulfan[18]`, `coords[N,2]` (unit chord) | – | analytic (CST is closed-form) |
| out | `tc_max`, `x_tc_max`, `t_at_spar` (= local t/c at `x_spar_frac`) | – | analytic / cs |
| out | `A_over_c2`, `P_over_c` → `K_A` (Eq. 9), `K_P` (Eq. 10) | – | analytic (shoelace area, polyline perimeter) |
| out | `K_A_le`, `K_A_te` (Eq. 11 with config area ratios), `K_A_rib` (Eq. 16) | – | analytic |
| out | polar table `Cl, Cd, Cm [nα × nRe]`, `Cl_max[nRe]`, `Cm0`, `nf_confidence_min` | –, rad | FD (NeuralFoil) |
| constraints | `t_at_spar ≥ (2 r_tube/c_root)/k_fill`, `tc_max ∈ [tc_min, tc_max]`, `nf_confidence_min ≥ 0.8` (placeholder) | | |

### 5.3 aerodynamics
OAS `Geometry` + `AeroPoint` (aero-only, `with_viscous=False`, `with_wave=False`). There are three points:
- cruise, with α set by a BalanceComp so that L = W;
- turn, at a config `CL_turn`;
- an α-perturbed point for `dCL/dα` (exact for linear VLM).

NeuralFoil profile drag is a strip integral:
`CD_p = (2/S) Σ_i Cd(Cl_i, Re_i) c_i Δy_i`.

`Cd(Cl, Re)` and `Cm(Cl, Re)` come from the airfoil polar table. Each table is inverted to a function of Cl by a
monotone spline on the pre-stall branch, which avoids solving for α inside NeuralFoil.

CLmax uses the critical-section method on the linear VLM: `CL_max = CL_ref · KSmin_i( Cl_max,i(Re_i) / cl_i )`, where cl_i is the spanwise
section Cl at `CL_ref`.

| | Name | Units |
|---|---|---|
| in | mesh, `S`, `c_mean`, `V`, `rho`, `mu_air`, `W` (from mass), polar tables | |
| in | tail/fuselage `S_wet`, form factors `FF_*`, `Cf` model (config) | |
| out | `CL`, `CDi`, `CDp_wing`, `CD_tail`, `CD_fus` (switchable), `CD`, `L_over_D`, `alpha_trim` | –, rad |
| out | `CL_alpha` (wing, per rad), `CL_max`, `V_stall` | 1/rad, –, m/s |
| out | `Cm_airfoil` (at cruise Cl, for Eq. 46), `q`, `q_div = ½ρ(1.2V)²` | –, Pa |
| out | `cl_span[ny-1]`, `L_span[ny-1]` (for the optional non-uniform loads mode) | –, N/m |
| out | polar fit `CD0, k, CL_mind` from the cruise and turn points (for the mission module) | – |

Parasite drag uses `CD_0,comp = Cf·FF·S_wet/S` with Raymer's form factors (Raymer, *Aircraft Design*, Ch. 12). The fuselage term is
switched by `aero.fuselage_drag: true|false`.

### 5.4 structures + mass (G&G)
See §6 for the block-by-block map. Group interface:

| | Name | Units |
|---|---|---|
| in | `b, c_mean, c_root, S, n_panels, y1, l_ht, l_tb, S_ht, S_vt, S_cb, winglet_ratio` | m, m², – |
| in | `tc, K_A, K_P, K_A_le, K_A_te, K_A_rib, x_spar_frac, c_rib_frac` | – |
| in | `r_tube` (derived or DV, see D1), `r_tb`, `theta_tb_max`, `delta_tip_ratio`, `theta_tip_max` (optional) | m, rad |
| in | `n` (load factor), `V_design`, `rho`, `Cm`, `CL_alpha`, `x_gd` | –, m/s, kg/m³, –, 1/rad, m |
| in | materials (σ, τ, E, G, ρ; balsa σ, τ, ρ; γ_film), `SF` | Pa, kg/m³, kg/m² |
| in | correction factors `f_wing, f_tb, f_ht, f_vt, f_cb, f_gear` | – (f_cb in kg/m²) |
| in | `m_sys` (propulsion + avionics), `m_pay` | kg |
| in (coupled) | `m` (total) | kg |
| out | `t_tube` and its parts (`t_sigma, t_delta, t_tau, t_T, t_theta, t_div`), `nt_rib`, `t_tb` | m |
| out | `m_tube, m_rib, m_te, m_le, m_film, m_wing, m_ht, m_vt, m_tb, m_cb, m_fus, m_gear, m_empty, m_total` | kg |
| out | `m_stat` (Eq. 124, report + initial guess) | kg |
| out (constraints) | `tube_fit_margin`, `min_gauge_margin = t_tube,req − t_min` (report), `rib_ratio_margin = A_rib/A > 0` | m, – |

### 5.5 propulsion
Plug-in base class `PropulsionModel(om.Group)` with a fixed I/O contract, so a better model can be dropped in later.

The default model `KvRmI0Model` combines:
- a motor model (Kv, Rm, I0);
- an APC PER3 prop table (Ct, Cp vs J for each RPM block, interpolated in J and RPM);
- a battery model: `n_s` cells, capacity Q, internal resistance R_int, usable fraction.

At each operating point it solves throttle so that T = D (cruise) and returns electrical power.

| | Name | Units |
|---|---|---|
| in | `V_cruise`, `D_cruise`, `T_static_req` (take-off, Raymer/Gudmundsson Eq. 3.3 in Josh's thesis), `rho` | m/s, N |
| in | `capacity_Ah` (DV), selections `prop_id`, `motor_id`, `n_s` (config / outer loop) | Ah |
| out | `P_elec_cruise`, `eta_total`, `I_cruise`, `T_static_avail`, `P_max`, `E_usable` | W, –, A, N, W, J |
| out | `m_motor, m_esc, m_prop, m_batt, m_sys = Σ + m_avionics(config)` | kg |
| out (constraints) | `thrust_margin_static`, `current_margin`, `rpm_margin`, `energy_margin` | |

Discrete choices (prop, motor, cell count) are **not** gradient variables. They're handled like airfoil
database mode, by an outer enumeration over a short catalogue (D4).

### 5.6 stability (low-fidelity, AVL-ready)
- Neutral point: `x_np = x_ac_w + η_h V_h (a_h/a_w)(1 − dε/dα) c_mac`.
  - `a_w` comes from aero.
  - `a_h` uses the Helmbold low-AR relation.
  - `dε/dα ≈ 2a_w/(π AR)`.
  - Sources: Raymer Ch. 16 and Nelson Ch. 2. The exact edition and equation numbers will be cited in code.
- CG: mass-weighted average of component masses at component stations (from geometry plus config offsets).
- Outputs: `x_cg, x_np, SM, V_h, V_v`.
- Constraints: `SM ∈ [SM_min, SM_max]`, `V_h ∈ […]`, `V_v ∈ […]`. All bands are **placeholders** in config (OPEN_QUESTIONS).
- Interface: `StabilityModel` ABC with inputs (geometry, masses, a_w) and outputs (x_np, SM). An `AVLStability`
  implementation can later wrap `runavl_C2.m`-style file generation (§10).

### 5.7 mission / performance
- **Reference model.** The ported `performance_modeller()` runs unchanged, with `Cd0, e, Clmax, m, eta`
  replaced by inputs from aero, mass and propulsion. A `matlab_compat` flag reproduces the MATLAB `round(Vstall)`
  (§8, M1) for the step 6 code check.
- **Proposal D2 (needs your approval).** The reference model's gradients come from FD through while loops and
  table lookups whose length changes with the inputs. That gives noisy or stepwise derivatives, which SLSQP handles
  badly. I propose a *smooth twin* with the same physics:
  - same constant-power assumption, accel/decel at C_L,max, stall-limited turn rate capped by n, and leg sequence
    from Table 3.5;
  - straights integrated as an ODE in distance with fixed-step RK4 (fixed N steps per leg);
  - turns integrated in turn angle;
  - every `max/min/clip` replaced by a smooth version.

  It's complex-step safe, so its partials can be exact. A test would require it to match the reference to <1% over
  a grid. The reference stays the source of truth for reporting and for the MATLAB check. If you'd rather keep a
  single FD-differentiated model, step 4 and step 6 still work; optimisation will just be slower and less robust.
- Outputs: `Lap1time, otherLapTime, ThreeLapTime, laps_frac` (continuous surrogate:
  `smoothmin(t_mission/t_lap, E_usable/(P t_lap))`), `lapCount` (report only), `Tstatic_req, Preq, Vstall,
  stall_margin, turn_speed_margin, energy_used`.
- Optional headwind on straights for the worst-case check: `V_ground = V_air − V_wind` per leg direction.

### 5.8 scoring (plug-in)
```python
class ScoringPlugin(ABC):
    name: str
    sense: Literal["min", "max"]
    def inputs(self) -> dict[str, dict]: ...        # names + units it needs (wired by problem.py)
    def build(self) -> om.ExplicitComponent: ...    # computes `score`
    def constraints(self) -> list[dict]: ...        # optional extra constraints (e.g. min laps)
```
Plug-ins register via the `aircrafter2.scoring` entry point or the config path `scoring.plugin: module:Class`.
There are two placeholders:
- `three_lap_time` (default, minimise);
- `laps_in_window` (maximise `laps_frac` within the mission time and energy).

No ruleset content.

---

## 6. Mass model: equation → component map (step 2)

All components are `ExplicitComponent`s with **analytic partials**, checked by `check_partials(method="cs")`.
Each `max` in Eqs. 51–53, 104, 118 and 123 uses `smooth.ks_max(x…, rho_ks, ref)`:
- formula: `ref/ρ · ln Σ exp(ρ x_i/ref)` (stable form), with a dimensional reference scale per quantity so that ρ is
  dimensionless;
- error ≤ `ref·ln(n)/ρ`.

A global `mass.max_mode: ks|hard` switches between the two. **Verification tests run in `hard` mode**, because
for n = 2 the flight and ground bending moments are *exactly equal* (§7 G18), so KS sits on its kink.

| Component | Equations | Notes |
|---|---|---|
| `SectionCoeffs` | 9, 10, 11, 15, 16, 17, 18 | Fed from airfoil coords; Table 12 values in verification mode (see G2). |
| `FlightLoads` | 43, 44, 45, 46, 47 (y = 0) | Optional `loads_mode: oas` integrates L(y) from aero instead of Eq. 43. |
| `GroundLoads` | 48, 49, 50 | Needs `x_gd` (G15). |
| `CriticalLoads` | 51, 52, 53 | KS. |
| `TubeSparSizing` | 105–118 | `t_div` passed through a softplus so that x̄_spar < 0.25 gives 0 (G9). Optional `deflection_mode: paper|exact` (G10), `torsion_stiffness_mode: paper|bredt` (G12). Min gauge via KS(t, t_min). |
| `RibSizing` | 97–104 (D-box derivation, applied with c_rib = 0.7c per §3.3.3) | Total n_rib·t_rib, no integer count. |
| `WingMass` | 26, 27, 28 (γ form), 30, 31, 32 | Multi-panel via Eq. 33 (G7). |
| `TailMass` | 34, 35 | |
| `TailBoomLoads` + `TailBoomSizing` | 55 with Eq. 5 (default) or 54; 56; 119–123 | See G1, G19. |
| `FuselageMass` | 36, 37, 38 | |
| `GearMass` | 39 | Uses coupled m. |
| `MassSum` | 40, 41 | Closure residual `m − (m_empty + m_pay)` converged by the MDA, not inside a component. |
| `StatisticalMass` | 124 (all), 125 (balsa), 126 (composite) | Initial guess for m + sanity output. |
| `calibration.fit()` | 127, 128 | scipy least-squares over chosen factors; offline tool, not in the loop. |

**Tube radius (decision D1).** I propose a **derived** radius by default:
`r_tube = k_fill · t_at_spar · c_root / 2 − clearance`, with `k_fill` and `clearance` in config.

Reasoning from the closed forms:
- `m_tube ≈ 2π r t b ρ`.
- `t^σ ∝ 1/r²` and `t^δ ∝ 1/r³`, so tube mass falls monotonically with r (as 1/r or 1/r²) until minimum gauge binds.
- So a free `r_tube` DV would sit on the tube-fit constraint at almost every optimum.

Making it derived removes a DV and an always-active constraint. It also couples t/c to spar mass directly, which is
the trade step 5 must show. A `r_tube: design_variable` mode with the `tube_fit_margin ≥ 0` constraint stays
available for cases where minimum gauge creates an interior optimum. Post-processing reports the next commercial
tube size up (catalogue in `data/`, placeholder).

**Verification (step 2, done).** `tests/mass/test_table15.py` reproduces every Table 15 component of UBI11, GRA13
and UBI15 within ±3% (worst case 2.3%). `docs/mass_verification.md` has the full table. That needed the readings in
G19, G22 and G23.

`tests/mass/test_reference.py` checks the group against an independent plain-Python implementation to 1e-8,
across all 16 combinations of the interpretation switches. `tests/mass/test_calibration.py` checks that Eq. 127,
fitted to Table 15's real masses, recovers the Table 14 individual factors (within 0.4%).

**Closure solver.** Standalone, the mass group uses NonlinearBlockGS with Aitken acceleration. The closure is weakly
coupled (dm_total/dm ≈ 0.05). Newton from the cold-start outputs diverged.

**Multi-panel Eq. 33 is dropped** (your decision, 2026-10-09: tapered multi-panel wings aren't needed for the
optimiser). The model uses Eq. 32 with one equivalent panel; `n_panels` only sets the f_wing exponent (the
number of wing joints).

---

## 6b. Step 3 status and findings (aero)

What's implemented:
- `aircrafter2/airfoil/`: CST evaluator matching AeroSandbox to 1e-16, reduced shape modes (D3), section
  geometry (t/c, thickness at the spar, A/c², P/c) with cs partials, and a NeuralFoil section solver.
- `aircrafter2/aero/`: rectangular half-wing mesh, OAS AeroPoint (aero-only, `with_viscous=False`),
  NeuralFoil strip profile drag, critical-section C_L,max, and wing totals.
- `aircrafter2/problems/wing_aero.py`, configured by `configs/step3_wing_aero.yaml`.
- Report: `docs/aero_verification.md`. FD step study: `docs/neuralfoil_fd_steps.md`.

Changes from §5.3:
- **No polar tables.** Each strip solves C_l,NF(α) = C_l directly (grid bracket, then Newton). Partials use the
  implicit-function theorem; only the network is finite-differenced. This avoids table-interpolation error and
  big table Jacobians. It costs about 0.1 s per run plus 0.1 s per derivative evaluation at 40 strips.
- **Parasite drag of tails and fuselage is deferred to step 7,** where the geometry module provides the wetted
  areas. `WingPerformance` already has a `CD_extra` input for it.

Findings:
- **A1. OAS reads the template mesh values.** OAS reads `surface["mesh"]` *values* at setup (e.g.
  `eval_mtx.py` checks which half-wing it is). An all-zero placeholder gives garbage (C_L ~ 1e15). We pass a
  unit-size template with the same node pattern; its size doesn't affect results (tested).
- **A2. The VLM is a flat-plate model.** Its α is measured from zero lift, and camber enters only through the
  NeuralFoil strips (matched on c_l). Report `alpha_sec` for the true section angle.
- **A3. Span efficiency, mesh and theory.** At AR 6 the VLM gives e ≈ 0.99 (converged) against lifting-line
  0.954, and C_L,α is 7% lower. OAS's near-field drag equals an independent Trefftz-plane calculation, so the
  difference is between the theories, not an OAS bug. Coarse meshes overstate e (1.04 at 10 strips); the
  default is 40 strips per half.
- **A4. FD steps for NeuralFoil.** NeuralFoil's derivative w.r.t. TE thickness is strongly nonlinear: a 1e-4
  step was off by up to tens of percent. Defaults are now α 1e-4°, ln Re 1e-5, Kulfan 1e-6, all on the plateau.
- **A5. C_L,max uses the cruise-point Reynolds number.** That's slightly optimistic for the stall check, because
  Re is lower at stall. It's documented, and a dedicated low-speed point can be added in step 7 if needed.

## 6c. Step 4 status (oval lap)

- `aircrafter2/mission/oval.py`: `LoadFactor` (n = L_turn/W, KS-capped at n_struct) and `OvalLapTime`
  (R = V²/(g√(n²−1)), lap = (2L_s + 2πR)/V), with analytic partials. Developed from
  `examples/legacy/basic_perf.py`; its `np.where` floor became a softplus floor.
- `aircrafter2/problems/oval_lap.py` (config `configs/step4_oval_lap.yaml`): one wing, two aero points
  (cruise with L = W; turn with its own α). The turn C_L margin C_L,max/C_L ≥ 1.21 sets n and R.
- Results: `docs/step4_oval.md`.
  - Without a power limit the turn margin is active (lap 33.6 s, R = 9.9 m, n = 4.2). With nothing penalising
    drag, the optimiser maximises the wing and its camber.
  - With a 400 W placeholder on turn power, the power limit becomes active (lap 35.0 s, R = 14.4 m).
- Note: VLM α is measured from zero lift (A2), so the α_turn bound is set wide (30°). The C_L-margin and
  per-strip stall constraints are what limit the turn.

## 6d. Step 5 status (aero + mass MDF)

- `aircrafter2/geometry/`: rectangular planform, tail arm/boom and tail volume coefficients, cargo-bay area.
- The aero point now also outputs `Cm_wing` (chord-weighted strip C_m) and `CL_alpha` (= C_L/α, exact for
  the flat-plate VLM).
- `aircrafter2/problems/aero_mass.py` (config `configs/step5_aero_mass.yaml`; `mass_config` points at the base
  mass YAML): MDA = Newton (solve_subsystems) + DirectSolver around {cruise aero, `BalanceComp` α for
  L = W, G&G mass with its own NLBGS closure}.
- Results: `docs/step5_aero_mass.md`.
  - The MDA converges at all 16 corners of the DV box plus the centre in ≤ 3 Newton iterations.
  - `check_totals` passes through the coupled loop.
  - The cruise-drag optimum is set by the spar: the thinnest section (t/c ≈ 0.08) is where the thin-wall
    limit (gauge on) or the minimum tube radius (gauge off) binds.
- New constraints: `wall_ratio ≤ 0.2` (thin-wall validity) and `r_tube ≥ r_tube_min` (placeholder 5 mm).
- **Finding M1 (resolved 2026-10-09):** with load-sized walls (micrometres at DBF scale) the G&G
  tail-boom × f_tb fuselage model gave 0.006 kg or 0.32 kg depending only on the minimum gauge.
  - Fix: a separate tail-boom minimum wall (`tb_min_gauge`, t_tb_min = 1 mm from the USYD tubes in Spar
    Sizer 3.1), with an output `t_tb_gauge_margin` (< 0 means the minimum wall governs).
  - f_tb is refitted on that basis to Table 15's real boom masses with the Eq. 128 hook: f_tb = 0.847
    (`docs/tail_boom_calibration.md`).
  - The step 5 placeholder aircraft now gets m_tb = 0.042 kg.
  - The Table 15 verification keeps the paper's basis (no minimum wall, f_tb = 12.81).
- **Finding M2:** the G&G LE/TE stringer and rib masses scale with section area, so at DBF scale they outweigh
  the strength-governed spar saving from a thicker section.

## 6e. Step 6 status (lap modeller)

- `aircrafter2/mission/lap_modeller.py` is the reference port (moved from `dbf_performance_modeller.py`).
  It gains `matlab_compat=True`, which reproduces MATLAB's rounded V_stall (M1) and its unrounded
  turn-table start.
- **MATLAB check: done (2026-10-09, R2026a).**
  - `verification/matlab/run_reference.m` ran the original `performance_modeller.m` unchanged at 6 points
    (2 deliberately infeasible), with `batteryFraction` 1 and 0 → `tests/data/matlab_reference.csv`.
  - The port (`matlab_compat=True`) matches to round-off (~1e-8): lap times, Preq, Tstatic, rounded V_stall
    and lap counts. NaN-flagged points have negative stall margins in the port.
  - The run exposed MATLAB bug M7 (§8). The test now uses rel 1e-6.
- `aircrafter2/mission/smooth_lap.py`: the smooth twin (D2).
  - Straights are RK4 in distance with a smooth speed hold at V; turns are RK4 in time with a smooth
    min(a, 0) at C_L,max drag.
  - Turn rate is stall-limited and KS-capped at n_struct. Lap count is a fractional surrogate; the integer
    `lapCount` is for reporting only. Partials are complex step (exact).
  - Matches the port within 0.65% (3-lap time) on 8 points, including near-stall, with identical lap counts.
  - A first multiplicative speed cap leaked (thrust > drag at V because P includes 1/η) and was replaced
    by a smooth min with relaxation.
- `aircrafter2/problems/lap_mdo.py` (config `configs/step6_lap_mdo.yaml`) couples the lap model:
  - the step 5 MDA (aero + mass);
  - two more aero points (mid, 0.95 C_L,max) feeding a three-point polar CD = CD0 + CD1 CL + k CL²;
  - inputs: m_total, S, V (now a DV), C_L,max; take-off C_L = C_L,max (no flaps).
  - Results: `docs/step6_lap_mdo.md`.
- **Finding L1:** AirCrafter's P = D(V)V/η makes lap time *improve* with cruise drag (higher drag means a
  higher throttle setting and more thrust out of the turns). The optimiser exploits it up to the 300 W
  placeholder (cruise L/D ≈ 4). Proposed for step 7: the power setting comes from propulsion, not from
  cruise drag.

## 6f. Step 7 status (propulsion, stability, parasite drag, full optimisation)

Decision B (2026-10-09): the powertrain sets the power; the cruise speed is an output.

- `aircrafter2/propulsion/apc.py`: APC PER3 parser (the AirCrafter 1.0 data in `THESIS/PERFILES2`) and
  smooth C_T(J, RPM), C_P(J, RPM) surrogates (RMS ≈ 0.0002 and 0.001).
- `aircrafter2/propulsion/powertrain.py`: battery + ideal ESC + first-order DC motor (Drela) + prop.
  - Torque balance solved at speed nodes, giving cubic T(v) and P_batt(v).
  - Also outputs static thrust, peak current, max RPM, usable energy and m_sys. Exact cs partials.
- Lap twin, `power_mode="propulsion"`:
  - thrust from T(v), no speed hold;
  - battery energy integrated on every leg;
  - energy-limited fractional laps and a static-thrust margin;
  - headwind and thrust-factor hooks.
- `aircrafter2/aero/parasite.py`: Raymer component build-up for the tails and a switchable fuselage
  (cargo box + boom).
- `aircrafter2/stability/static.py`: tail-volume neutral point, CG from component stations, static margin.
- `aircrafter2/problems/full_mdo.py` (config `configs/step7_full.yaml`):
  - the MDA now also solves v where T(v) = D(v) (that speed is the G&G design speed too), and includes
    parasite drag;
  - DVs: b, c, tc_scale, camber_scale, battery capacity, tail arm, S_ht, S_vt, battery station and
    cargo-bay station;
  - current/RPM limits are post-checks (they don't depend on the DVs at fixed prop, cells and throttle);
  - the prop is enumerated outside the optimiser (D4).
- `aircrafter2/robustness/suite.py`: normalised sensitivities (compute_totals), one-at-a-time worst cases
  (mass, thrust, capacity, headwind), and Monte Carlo over the uncertain inputs.
- Results: `docs/step7_full_mdo.md`.
- Notes:
  - The first full run was infeasible: with the placeholder stations the CG was too far forward for
    SM ≤ 0.20 together with V_h ≥ 0.35. Making the cargo-bay station a DV fixed it.
  - With the placeholder 3-lap objective, the battery is sized to exactly three laps (no reserve).
  - Rotor speed rises with flight speed. A first max-over-nodes RPM post-check therefore judged props
    above their top speed; it was replaced by a constraint on rpm at the full-throttle level speed.
  - Result (placeholder powertrain): 10x7E best at 79.7 s for 3 laps; 11x7E 80.7 s (RPM-limited); 12x8E
    did not converge (tight RPM limit).
  - Sensitivities: the motor Kv placeholder dominates (−0.25), then payload, Rm, air density and R_cell.

## 7. G&G 2022: errata, ambiguities and findings

"Paper wins, but flag it" is followed throughout. The default implementation is the paper as printed, unless an
item says otherwise. Each deviation is a config switch.

**G1. Eqs. 5–8 are inconsistent with Table 3.** Eq. 5 prints `K_ht = (l_ht/c_wing)·A_wing`, so with Eq. 7
(`l_ht = K_ht c`) you'd get `l_ht = l_ht·A`. Table 3 gives a "tail arm-to-span ratio" (l_ht/b), which suggests the
intent was `K = l/b`. Eqs. 55–56 are written with K_ht and K_tb.
→ Use `l_ht` (a DV, in metres) and `l_tb` directly. Tests convert Table 3 ratios as `l = ratio·b`. For the boom
loads, see G19: the paper's own numbers use Eq. 5 *as printed*.

**G2. Tables 4, 8 and 12 list "Aerofoil area-to-chord squared ratio" (A/c²), not K_A from Eq. 9 (A/((t/c)c²)).**
UBI15's 0.0868 at t/c = 0.141 is a realistic A/c². Read as K_A, it would give A/c² = 0.012, which is impossible for
a 14% section.
→ Compute K_A by Eq. 9 from coordinates. Tests use `K_A = table/(t/c)`.

**G3. Eq. 16 typo.** It prints `A_tube/A + A_rib/A + A_rib/A + A_te/A = 1`; the second A_rib should be A_le. The
`πr²/A` term uses "A of the aerofoil under consideration". It's unclear whether that A is at the mean chord (used
by the mass equations) or the root chord (used for sizing).
→ Mean chord, to match Eqs. 26–31. Flagged.

**G4. Cross-reference slips** (no effect on the maths):
- p. 260: "Eq. (7) … substituting i with le or te" should be Eq. (11).
- §2.2.5: ribs "Eq. (23)" should be Eq. (26). Confirmed; this matches the brief.
- After Eq. 107: "M_x in Eq. (59)" should be Eq. (107).
- Before Eq. 111: "given by Eq. (82)" should be Eq. (85).
- After Eq. 93: "x̄_e from Eq. (21)" should be Eq. (20).
- Before Eq. 94: twist "Eq. (91)" should be Eq. (92).
- §3.4: "as described in Section 3.4" should be §3.3.3 (matches the brief).
- Eqs. 102–103: "n_tib" should be n_rib.
- Eq. 33: "(c/t)_j" should be (t/c)_j.

**G5. Eq. 32: `f_wing^(n_panels−1)` is an exponent. Confirmed numerically.** Back-solving Table 15 with the
per-aircraft and combined f_wing values from Table 14 gives exponents of 2.08 (UBI15), 2.05 (UBI11) and 4.18 (GRA13).
Those match n_panels − 1 = 2, 2 and 4. Consequence: a single-panel wing gets no f_wing correction.

**G6. Eq. 33 (multi-panel).** It doesn't say whether `f_wing^(n−1)` and `(1 + S_winglet/S_wing)` apply per panel
or once to the sum.
→ Apply once to the sum, so a 1-panel Eq. 33 reduces exactly to Eq. 32. Flagged.

**G7. Sizing chord vs loads chord.** §3.2 loads use the mean chord; §3.3 sizing uses c = c_root. Eq. 46 (torsion)
uses c², and it feeds Eqs. 114 and 116.
→ Loads at c̄, sizing K_tube = r/c_root (for a fixed r the two cancel in Eqs. 107–114 anyway). Flagged.

**G8. Eq. 46 wording.** The text says "torsion moment distribution is uniform", but the formula is linear in
(b/2 − y), i.e. a uniform torque *per unit span*. The Eq. 116 integration (which gives the 16π) is consistent with
that linear form. Wording only.

**G9. Eqs. 47 and 117 sign.** With the spar forward of c/4, `x̄_spar − 1/4 < 0` and t_div comes out negative.
→ softplus(·) ≥ 0. Table 3 spar positions are 0.26–0.30, so divergence *is* active in all three test aircraft.
Table 3 also gives an *aerofoil* (2D) lift-curve slope of 6 /rad, while Eq. 47 asks for the *wing* dC_L/dα. In the
framework we use the OAS wing slope; tests use 6 /rad.

**G10. Tip-deflection polynomial (Eqs. 70, 109, 110) can't be reproduced.** Symbolic check
(`sympy`, cantilever with I₂ = (1−2y₁/b)² I₁ outboard):
- All three expressions agree at y₁ = 0 (= nWb³/(128 E I)), so the 384 in Eq. 109 is right for the tube.
- Assembling the paper's own Eqs. 63–69 gives `3b⁴ + 12b³y₁ − 108b²y₁² + 224by₁³ − 112y₁⁴`, not Eq. 70's
  `3b⁴ + 24b³y₁ − 120b²y₁² + 224by₁³ − 112y₁⁴`.
- Eqs. 64–65 also leave out the tip shear force acting on the inboard segment.
- Exact double integration gives `3(b⁴ + 4b³y₁ − 20b²y₁² + 32by₁³ − 16y₁⁴)` over the same denominator.
- At y₁ = b/6, Eq. 70 is **+23% above the exact result**.
→ Default to the printed Eq. 110 (to reproduce the paper), with `deflection_mode: exact` available. **y₁ isn't
given in Table 3** (OPEN_QUESTIONS Q4).

**G11. Eq. 112 shear factor.** The 3Q/(2A) peak factor is for a solid rectangle. A thin-walled circular tube peaks
at 2Q/A, so t^τ is 25% low. It's a small term; I implement the paper as printed. Flagged.

**G12. Eq. 115 torsional stiffness is 2× Bredt–Batho.** For a single-wall tube, `GJ = 4A²G t/∮ds = 2π r³ G t`. The
paper gives `4π K³c³ G t`. It looks carried over from Eq. 91, where the sandwich skin has two faces (2 t_skin_face).
Effect: t^θ (Eq. 116) and t^div (Eq. 117) are half the Bredt value.
→ Default to the paper, with `torsion_stiffness_mode: bredt` available. In the prototype, Bredt raises UBI15's wing
mass by about 3.5%.

**G13.** Eq. 37: the text mentions t_cb and ρ_cb, but the equation is just `f_cb S_cb`. Implement the equation.

**G14.** Eq. 39: `m_gear = f_gear·m` uses the *total* mass. This adds to the closure loop; that's fine in the MDA.

**G15.** Eq. 50 needs the ground-test hold point x_gd, which isn't in Table 3 (OPEN_QUESTIONS Q4). Eqs. 48–49 use
the *full* W (payload included).

**G16.** Table 3 gives g = 9.80655 (standard is 9.80665). Tests use the table value; the framework uses 9.80665.
The difference is negligible.

**G17.** For the tube concept, c_rib/c isn't tabulated. §3.3.3 says "around 70%", and Table 8 (D-box) gives
0.67/0.70. → 0.7. The tail-boom material isn't in Table 13 → assume UD CFRP. Max tail rotation in Table 3 is in
degrees.

**G18.** With n = 2 (all three test aircraft), flight and ground bending (Eqs. 45, 49 at y = 0) are both `Wb/4`.
Eq. 52 is evaluated exactly at the KS kink, hence the hard-max verification mode.

**G19. Tail-boom shear: resolved. The paper's numbers use Eq. 55 with K_ht from Eq. 5 as printed.**
With K_ht = (l_ht/c)·A (A = b/c), Eq. 55 gives Q_tb = q|Cm| b c/K_ht = q|Cm| c³/l_ht. That's Eq. 54's
moment-balance shear divided by the aspect ratio. Eq. 56's moment is unaffected (A cancels), so M_tb ≠ Q_tb·l_tb.
This reading reproduces all three Table 15 tail booms to within 0.2%. The literal Eq. 54 reading is 9–14× high,
because the rotation-stiffness term (Eq. 121) then dominates.
- `tail_boom_load_mode: paper` (default): f_tb = 12.81 was calibrated with it.
- `eq54`: self-consistent physics, but needs f_tb recalibrated.

Note the two modes differ by a factor of A in the boom *shear*, so the choice matters for any recalibration.

**G20. Resolved in step 2.** The 12–19% wing over-prediction from step 1 had two causes:
- G19: the boom error inflated total mass, and with it the wing loads.
- G23: the spar sizing radius.

No correction factor was tuned.

**G22. Table 15's UBI11 column uses UBI11's *individual* general factors.** For UBI11 the ratios m_ht/m_wing and
m_vt/m_wing in Table 15 imply f_ht = 0.659 and f_vt = 1.177. Its m_cb and m_gear imply f_cb = 0.867 and
f_gear = 0.0100. Those are the individual Table 14 values (0.6609, 1.172, 0.8667, 0.0100), not the combined ones.
Table 14's "combined" column even lists f_vt = 0.7753 for UBI11. f_wing and f_tb are the combined values: the wing
and boom errors in Table 15 equal the combined/individual factor ratios. GRA13 and UBI15 use combined factors
throughout. `configs/verification/ubi11.yaml` follows this.

**G23. Spar sizing radius: the paper's numbers use K_tube = r/c̄ with the sizing chord c_root.** Eq. 15 defines
K_tube = r/c, and §2.2.2 says c is the mean chord in the mass models. Eqs. 107–117 write π K² c² (etc.), and §3.3
says c = c_root there. Read literally, the tube is sized as if its radius were r·c_root/c̄. That makes it thinner
by (c_root/c̄)² in bending and (c_root/c̄)³ in deflection and divergence. With G19, this reproduces the Table 15
wings to −0.6…−1.9%.
- `sizing_radius_mode: paper` (default, consistent with f_wing = 1.12).
- `physical`: sizes with r.

The two are identical for a rectangular wing (c_root = c̄). For taper, `paper` is unconservative.

**G21. Statistical Eq. 124's fit range.** It was fitted to ACC aircraft with m = 8.9–16 kg and m_pay = 5–12 kg.
DBF aircraft (~3–7 kg) are an extrapolation, and the paper reports empty-mass errors up to 55%.
→ Use it only as the MDA initial guess and a sanity column.

---

## 8. MATLAB ↔ Python port discrepancies (relevant to step 6)

**M1.** MATLAB rounds `Vstall = round(sqrt(...))` to an integer, and the stall check is
`1.2*round(Vstall) >= V`. The rounded value also starts the accel table, the decel table and the turn-rate table.
The port uses the exact Vstall. → Add a `matlab_compat` switch.

**M2.** `AirCrafter.m` sets `inputs.batteryFraction = 0` and never changes it, so `E = 0`, the energy check trips on
the first lap, and **`lapCount` is always 1** in the MATLAB runner. The port defaults `battery_fraction = 1.0`.
Reproduction runs must state which convention they use.

**M3.** Take-off time is not added to Lap 1. This is known and already documented in the port.

**M4.** In the MATLAB decel-table build, `a_vector` is filtered and then reused as a mask for `v_vector`. The port
does this correctly. The results match as long as the a < 0 region is a contiguous prefix of the descending
velocity vector (true for C_L,max-drag turns), so it doesn't matter in practice. I'll still verify on the
reproduction points.

**M5.** MATLAB raises an error if accel-table building exceeds 100000 steps (for example when the target speed is
unreachable). The port breaks out with `a ≤ 0`. Both behave the same on feasible points.

**M7 (found 2026-10-09 with the MATLAB run).** In `leg_calc_turn.m`, if the turn-entry speed is below the
slowest speed in the deceleration table (where C_L,max-drag deceleration ends), `interp1` returns NaN. It is
not flagged: the lap times become NaN and the lap count is 1 (`NaN < missionTime` is false). An example is
m = 7, b = 1.5, AR = 9, V = 25. The port now keeps the speed unchanged in that case; `matlab_compat`
reproduces the NaN.

**M6.** `static_thrust_calculator` computes `Tstatic`, but it doesn't feed the lap time in either version. In 2.0 it
becomes the propulsion requirement `T_static_req`.

---

## 9. Conflicts with the brief or existing work (flagged, not decided)

**B1. Preliminary report vs brief.** Report §3.2 lists *m* in the design vector *x* and treats the airfoil as a
*discrete* database choice (§3.3). The brief's settled decisions make *m* a coupling variable and the airfoil
continuous, with database mode kept. I'm following the brief; the thesis text will need to match.

**B2. One tube vs two tubes.**
- The USYD 2025 spar sizer models **two CF tubes**: front 22/20 mm and rear 20/18 mm, with a 60/40 load split.
- It has a stepped-down outer wing at 0.45 semispan and a wing-mounted payload pylon (bending relief).
- Its spar properties are UTS = 340.7 MPa and E = 220 GPa, against G&G Table 13's 720 MPa and 84 GPa.

The G&G tube concept is single-tube. Options:
- (a) one equivalent tube (default, as briefed);
- (b) extend `TubeSparSizing` to N tubes with a load split.

I'll keep the Table 13 placeholders as briefed, but the team's own tube data differs a lot (OPEN_QUESTIONS Q6).

**B3. x̄_spar default.** The brief's 0.25–0.30 range: at anything above 0.25, Eq. 117 is active. I'll default to
0.25 and expose it in config.

**B4. "Reuse this code rather than rewriting it" vs a smooth lap model (D2).** The reference port is reused
verbatim. The smooth twin is an *addition* for gradients and needs your go-ahead.

**B5. Two weight bases.** The lap modeller's `payload_fraction` assumption is unused once mass comes from the G&G
closure with a fixed `m_pay`. It will be dropped from the 2.0 inputs, but kept in the reference port's signature.

---

## 10. Spar Sizer 3.1: notes for structures and stability

**Useful for structures:**
- Real USYD tube sizes (22/20 and 20/18 mm OD/ID). These seed the commercial-tube catalogue and the minimum gauge
  (1 mm wall).
- Load factor 5 "Maneuver" case with FS 1.5 at 138.65 km/h. The script's own comment notes V_a = 80.6 km/h, so
  which speed is intended needs checking.
- Combined bending and shear principal stress with margin of safety `MoS = UTS/(FS σ₁) − 1`.
- The outer-wing step location (0.45 semispan) is a concrete y₁ for Eq. 110 in USYD designs.

**Validation hook (later, out of scope now):** the AVL lift distribution (`runavl_C2.m`), integrated with a
double-integration cantilever deflection (`getDeflection.m`), is an independent check for the `loads_mode: oas`
option and the `deflection_mode: exact` switch.

**For stability:** `runavl_C2.m` already generates `.avl` geometry (taper type, sweep, washout, dihedral) and parses
CL, CDi, e, CLa and Cma. That's the natural seed for the future `AVLStability` implementation (§5.6). `avl.exe` is
a Windows binary, so on Linux we'd need a native AVL build.

---

## 11. Decisions

**Resolved 2026-10-09:**
- D1: derived radius (approved).
- D2: smooth lap-model twin (approved).
- D3: reduced airfoil shape modes (approved).
- D4: discrete outer enumeration for prop, motor and cells (approved).
- D6: **single tube**.
- D7: pydantic (approved).
- Git: approved; repo initialised in `AirCrafter2/`.

D5 (Table 15 tolerance) is still open. Working value: ±3%.

| ID | Decision | Recommendation (as proposed) |
|---|---|---|
| D1 | Spar tube radius: derived (r = k_fill·t_spar·c_root/2) or DV + tube-fit constraint | **Derived** by default; DV mode available |
| D2 | Add a smooth twin of the lap modeller for gradients (reference kept for reporting and the MATLAB check) | **Yes** |
| D3 | Airfoil DVs: reduced shape modes on a seed Kulfan airfoil, or full 18-weight CST | **Reduced modes** (2 + 4–8), full CST optional |
| D4 | Prop, motor and cell count as discrete outer enumeration (not gradient DVs); battery capacity continuous | **Yes** |
| D5 | Table 15 tolerance for "a few percent" | ±3% per component *if* the cause in G20 is found; otherwise I report |
| D6 | B2: single equivalent tube or N-tube extension | Single tube now, N-tube later |
| D7 | Config validation with pydantic v2 (clear error messages for students) | **Yes** |

---

## 12. Dependencies (to pin in `pyproject.toml`)

Python `==3.14.*` (the venv's interpreter).

Pins:
- `openmdao==3.45.0`, `openaerostruct==2.12.0`
- `numpy==2.5.2`, `scipy==1.18.0`
- `neuralfoil==0.3.3`, `aerosandbox==4.2.10`
- `pyyaml==6.0.3`, `pydantic` (2.x, pinned to the resolved version)
- `matplotlib==3.11.1`

Extras:
- `[ui]`: `streamlit`
- `[docs]`: `pyxdsm`
- `[test]`: `pytest`, `sympy`
- pyOptSparse optional (IPOPT/SNOPT used only if importable)

Exact versions for unpinned items will be the ones `uv` resolves at install time, recorded in the pyproject. Nothing
gets installed until you approve.

---

## 13. Milestone acceptance criteria

| Step | Done when |
|---|---|
| 2 Mass | All §6 components exist with analytic partials. `check_partials` passes (cs, rtol 1e-6). Table 15 reproduced for UBI11, GRA13 and UBI15 within D5, *or* a written report pinning the non-reproducible equation(s). Statistical model and calibration hook unit-tested. |
| 3 Aero | OAS + NeuralFoil rectangular wing; min CD at fixed CL converges. NeuralFoil + VLM CD within ~5% of a hand calculation for one airfoil: `CD = CDp(Cl≈CL) + CL²/(π e AR)` with e from VLM. FD step sweep documented. |
| 4 Oval | basic_perf-style lap time minimised at fixed V. Turn radius from the C_L margin. Smooth, with `check_totals` passing. |
| 5 Aero + mass MDF | MDA converges on a corner + centre grid over the DV bounds (Newton or NLBGS with Aitken, ≤ 30 iterations). Sweep showing the t/c drag vs spar-mass trade. `check_totals` passes. |
| 6 Lap modeller | Reference port vs MATLAB for 3–5 grid points from `inputs_file.m` ranges (with `matlab_compat`, M1/M2 stated) to ≤ 0.5% on Lap1time, otherLapTime, ThreeLapTime, Preq and Tstatic. Then the coupled version. |
| 7 Full | Propulsion, then stability, then fuselage drag. The full optimisation finds a feasible optimum. Sensitivity bars, worst-case table and Monte Carlo on the finalists. |
| 8 UI/docs/figs | Streamlit app, README quickstart, one doc page per module, "how to add a scoring plug-in", vector-PDF figure script. |
