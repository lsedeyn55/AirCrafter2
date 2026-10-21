# Universal placeholders file and ruleset files: plan and inventory

Brief: `4_PROMPTS/aircrafter2_placeholders_rulesets_prompt.md`. Branch `placeholders-rulesets`, from `26b0e30` (the
take-off re-optimisation commit, which settled the brief's uncommitted take-off files) rather than `6341316`.

## Decisions (user, 2026-10-22, before checkpoint 1 closed)

1. **Single-point configs get a generic ruleset.** `configs/rulesets/generic.yaml` holds the `inputs_file.m` values:
   altitude 400 m, mission time 300 s, the course preset, the pilot reaction time, the start offset and the 0–3 m/s
   headwind. `full_mdo*.yaml`, `aero_mass.yaml`, `lap_mdo.yaml`, `wing_aero.yaml` and `oval_lap.yaml` name it. The
   2024-25 configs name `rulesets/dbf_2024_25.yaml`.
2. **The shared RDO spreads are only the reduced set's four.** f_build, f_tb and m_avionics go in `placeholders.yaml`;
   lap.wind goes in each ruleset. Every other spread (f_cb, f_gear, f_ht, f_vt, e_spec, R_cell, Kv/Kv_factor,
   Rm/Rm_ref, m_motor, eta_h, lap.thrust_factor, m_pay, store_drag_factor) stays only in the files of the published
   variants that use it. `configs/uncertainty.yaml` is the 15-parameter set of `rdo_study`, `rdo_full`,
   `full_mdo_rdo*` and `examples/full_mdo.py`, and it is marked `standalone: true`: its list is the whole pool, with no
   shared spreads merged in. So the published full-set problems keep their membership and order. `active: null` in
   the reduced configs stays d = 4.
3. **The G&G values move** (materials, k_fill, clearance, t_tube_min, delta_tip_ratio, theta_tb_max). The
   verification configs, which extend `mass_garcia_gamboa_2022.yaml` directly, then fall back to the `inputs.py`
   code defaults. These are identical except theta_tb_max, so that default changes from 0.01745 to 0.017453 rad (1 deg,
   G&G Table 3). The verification files are unchanged.
4. **Code defaults: only the stores and rules move.** The `StoreConfig`, `Bottle`, `BOTTLES` and `RulesConfig` values
   move to the ruleset; their fields become required. Other pydantic defaults stay as layer 1, always overridden by
   `placeholders.yaml`.

## Load order (lowest to highest)

1. code defaults
2. `placeholders.yaml`: the `placeholders:` key, default `configs/placeholders.yaml`
3. the ruleset: the `ruleset:` key, opt-in
4. the problem config and its `extends:` chain
5. the study variant (a config that extends the problem config)

- `mass:` layers become: `mass_config`, then `placeholders.yaml mass`, then the config's `mass:`.
- A single-point load keeps only the ruleset's top-level keys that its schema has, and ignores `multipoint:`.
- Layer keys the schema does not have are dropped silently, but only for layers 2–3. A typo in the problem config
  still fails, as now.
- `uncertainty:` lists in layers 2–3 are merged by name into a pool, in order: placeholders, ruleset, the
  `uncertainty_config` file, then `robust.parameters`. A `standalone: true` uncertainty file replaces the shared pool.

## Inventory: `configs/placeholders.yaml`

Current value and where it is set today (file:line). "→" marks a conflict that the most-recent rule resolves (next
table). Status is the tag the value carries. Untagged values that get tagged now: n_crit, w_cb, h_cb, V_cell, V_h_max,
V_v_min, V_v_max, k_lof.

| Area | Path | Value (winner) | Set today | Status | Q |
|---|---|---|---|---|---|
| Energy | `energy_reserve` | 0.2 | full_mdo:41, full_mdo_twin:47 | PLACEHOLDER | Q24 |
| Energy | `propulsion.R_cell` | 0.005 Ω | full_mdo:51, full_mdo_twin:65, dbf2025_multipoint:22 | PLACEHOLDER | Q8 |
| Energy | `propulsion.V_cell` | 3.7 V | full_mdo:50, full_mdo_twin:64, dbf2025_multipoint:21 (untagged) | PLACEHOLDER | Q8 |
| Energy | `propulsion.e_spec` | 134.3 Wh/kg → | full_mdo:53 150, full_mdo_twin:67 150, dbf2025_multipoint:23 134.3 | cited [W p. 22, Table 34] | Q8 |
| Motor | `propulsion.Kv` | 1050 rpm/V → | full_mdo:54 900, full_mdo_twin:68 900, dbf2025_multipoint:25 1050 | cited [W p. 22] | Q7 |
| Motor | `propulsion.Rm` | 0.03384 Ω → | full_mdo:55 0.05, full_mdo_twin:69, dbf2025_multipoint:26 | cited [TM] | Q7 |
| Motor | `propulsion.I0` | 1.79 A → | full_mdo:56 1.0, full_mdo_twin:70, dbf2025_multipoint:27 | cited [TM] | Q7 |
| Motor | `propulsion.m_motor` | 0.1127 kg → | full_mdo:57 0.15, full_mdo_twin:71, dbf2025_multipoint:28 | cited [TM] | Q7 |
| Motor | `propulsion.m_esc` | 0.127 kg → | full_mdo:58 0.06, full_mdo_twin:72, dbf2025_multipoint:29 | cited [W Table 34] | Q7 |
| Motor | `propulsion.m_prop` | 0.018 kg → | full_mdo:59 0.03, full_mdo_twin:73, dbf2025_multipoint:30 | cited [W Table 34] | Q23 |
| Motor | `propulsion.m_avionics` | 0.222 kg → | full_mdo:60 0.25, full_mdo_twin:74, dbf2025_multipoint:31 | cited [W Table 34] | Q23 |
| Motor | `propulsion.I_max` | 60 A (same value; newer citation) | full_mdo:61, full_mdo_twin:75, dbf2025_multipoint:33 | cited [W p. 22] | Q7 |
| Nacelles | `propulsion.m_nacelle` | 0.0 kg (same value; newer comment) | full_mdo_twin:55, dbf2025_multipoint:32 | PLACEHOLDER | Q25 |
| Nacelles | `propulsion.l_nac` | 0.15 m | full_mdo_twin:57 | PLACEHOLDER | Q25 |
| Nacelles | `propulsion.d_nac` | 0.05 m | full_mdo_twin:58 | PLACEHOLDER | Q25 |
| Nacelles | `propulsion.c_clear` | 0.025 m | full_mdo_twin:56 | PLACEHOLDER | Q29 |
| Structure | `mass.inputs.r_tb` | 0.008 m | full_mdo:13, full_mdo_twin:19, aero_mass:14, lap_mdo:14 | PLACEHOLDER | Q19 |
| Structure | `mass.inputs.x_gd` | 0.25 | full_mdo:14, full_mdo_twin:20, aero_mass:15, lap_mdo:15 | PLACEHOLDER | Q4 |
| Structure | `r_tube_min` | 0.005 m | full_mdo:30, full_mdo_twin:36, aero_mass:32, lap_mdo:32 | PLACEHOLDER | Q3 |
| Structure | `mass.inputs.k_fill`, `clearance` | 0.85, 0.001 m | mass_garcia_gamboa_2022:67–68 | PLACEHOLDER | Q5 |
| Structure | `mass.inputs.t_tube_min` | 5.0e-4 m | mass_garcia_gamboa_2022:70 | PLACEHOLDER | Q3 |
| Materials | `mass.inputs.sigma_spar`, `tau_spar`, `E_spar`, `G_spar`, `rho_spar` | 7.2e8, 5.4e7, 8.4e10, 5.0e9 Pa, 1600 kg/m³ | mass_garcia_gamboa_2022:37–41 | cited G&G Table 13 (PLACEHOLDER for USYD) | Q6 |
| Materials | `mass.inputs.sigma_tb`, `tau_tb`, `E_tb`, `rho_tb` | as the spar | mass_garcia_gamboa_2022:43–46 | assumed UD CFRP (PLAN G17) | Q6 |
| Materials | `mass.inputs.delta_tip_ratio`, `theta_tb_max` | 0.1, 0.017453 rad | mass_garcia_gamboa_2022:33–34 | cited G&G Table 3 (PLACEHOLDER for USYD) | Q12 |
| Payload | `mass.inputs.m_pay` | 1.5 kg | full_mdo:12, full_mdo_twin:18, aero_mass:13, lap_mdo:13 | PLACEHOLDER | Q12, Q19 |
| Fuselage | `geometry.l_cb`, `w_cb`, `h_cb`, `boom_offset` | 0.40, 0.12, 0.12, 0.15 m | full_mdo:22–25, full_mdo_twin:28–31, aero_mass:24–27, lap_mdo:24–27 (woombah_2025 overrides l_cb, w_cb, h_cb) | PLACEHOLDER | Q19 |
| Tail and CG | `stability.AR_ht`, `AR_vt`, `tc_tail`, `eta_h`, `wing_cg_frac` | 4.0, 2.0, 0.09, 0.9, 0.40 | full_mdo:66–70, full_mdo_twin:80–84 (woombah_2025 overrides AR, tc) | PLACEHOLDER | Q23 |
| Tail and CG | `stability.x_motor` | −0.1397 m → | full_mdo:71 −0.30, full_mdo_twin:85 −0.1397, dbf2025_multipoint:40 −0.1397 | cited [W p. 20] | Q23 |
| Tail and CG | `stability.x_avionics`, `x_gear` | −0.10, 0.05 m | full_mdo:72–73, full_mdo_twin:86–87 | PLACEHOLDER | Q23 |
| Stability limits | `stability.SM_min`, `SM_max`, `V_h_min`, `V_h_max`, `V_v_min`, `V_v_max` | 0.05, 0.20, 0.35, 0.65, 0.02, 0.06 | full_mdo:75–80, full_mdo_twin:89–94 | PLACEHOLDER | Q9 |
| OEI | `stability.eta_v`, `rudder_tau`, `rudder_max_deg`, `oei_speed_factor` | 0.9, 0.5, 25 deg, 1.2 | code defaults only (full_mdo.py:125–128) | PLACEHOLDER | Q29 |
| Aero | `aero.n_crit` | 9.0 | full_mdo:6, full_mdo_twin:9, aero_mass:6, lap_mdo:6, wing_aero:13, oval_lap:5 | PLACEHOLDER (XFoil default) | Q16 |
| Take-off | `takeoff_distance` | 106.68 m → | full_mdo:35 100, full_mdo_twin:41 100, lap_mdo:37 100, dbf2025_multipoint:10 106.68 | PLACEHOLDER constraint [W p. 10 Table 4] | Q40 |
| Take-off | `course.takeoff.k_lof`, `CL_ground` | 1.1, 0.15 | code defaults (course.py:49–53); the `_takeoff` variants | k_lof: MATLAB comment (now tagged); CL_ground: PLACEHOLDER | Q40, Q41 |
| RDO spreads | `uncertainty: f_build, f_tb, m_avionics` | f_build 1.0 ± 0.30; f_tb ± 30 % rel.; m_avionics ± 15 % rel. | uncertainty_2025_reduced:6–8 (f_tb, m_avionics also uncertainty.yaml) | PLACEHOLDER | Q32, Q39 |

## Inventory: rulesets

`rulesets/dbf_2024_25.yaml`. Citations move with the values.

| Area | Path | Value | Set today |
|---|---|---|---|
| Scoring | `multipoint.scoring.R2`, `R3`, `bonus`, `x1_weight_lb` | 0.20 lb/s, 27.0, 0.0, 0.37 lb | dbf2025_multipoint:64–67 (R2, R3 PLACEHOLDER Q35) |
| Stores | `multipoint.stores.*`: bottle, n_ext, fill_density, cd_bottle, Q_tank, pylon dimensions and pylon_xt, Q_pylon, m_pylons, X-1 data and t/c, Q_x1, stations, x_internal_frac | stores.py defaults | stores.py:37–77 (`stores: {}` at dbf2025_multipoint:62) |
| Stores | `bottles` (the bottle-study catalogue) | 0.5, 1, 2 L | stores.py:44 `BOTTLES` (Q36) |
| Rules | `multipoint.rules.*` (TOGW 55 lb, 100 Wh, fuse 100 A, 70 C, 3 laps, 6 ft span) | | multipoint.py:56–62 code defaults |
| Mission | `mission_time`; `course.preset`, `course.pilot_reaction_time_s`, `course.takeoff.start_offset_ft` | 300 s; dbf_2025, 3.0 s, 0 ft | dbf2025_multipoint:12; course.py code defaults |
| Site | `altitude` | 800 m | dbf2025_multipoint:9 |
| Uncertainty | `uncertainty: lap.wind` | 3.5 ± 3.5 m/s | uncertainty_2025_reduced:16 |

`store_drag_factor` (rdo_full only) stays in `dbf2025_multipoint_rdo_full.yaml` (decision 2).

`rulesets/generic.yaml`: altitude 400 m, mission_time 300 s (each at full_mdo:16 and 36, full_mdo_twin:22 and 42,
aero_mass:17, lap_mdo:17 and 38, wing_aero:15, oval_lap:7); course preset `dbf_2025`, pilot reaction 3.0 s and start
offset 0 ft (code defaults); lap.wind 1.5 ± 1.5 m/s (uncertainty.yaml:38, which keeps it since it is standalone).

## Conflicts (most recent line wins, `git blame`)

| Path | Values (file:line, commit, date) | Winner |
|---|---|---|
| `propulsion.e_spec` | 150 (full_mdo:53 ccf2f13 2026-10-09; full_mdo_twin:67 1aa05c8 2026-10-13); 134.3 (dbf2025_multipoint:23 20ef669 2026-10-20 11:41) | 134.3 |
| `propulsion.Kv`, `Rm`, `I0`, `I_max` | 900, 0.05, 1.0, 60 (full_mdo:54–56 and 61, full_mdo_twin:68–70 and 75: 3bd961f 2026-10-20 10:04); 1050, 0.03384, 1.79, 60 (dbf2025_multipoint:25–27 and 33: 20ef669 2026-10-20 11:41) | dbf2025 |
| `propulsion.m_motor`, `m_esc`, `m_prop`, `m_avionics` | 0.15, 0.06, 0.03, 0.25 (ccf2f13 2026-10-09; 1aa05c8 2026-10-13); 0.1127, 0.127, 0.018, 0.222 (20ef669 2026-10-20) | dbf2025 |
| `propulsion.m_nacelle` | 0.0 (full_mdo_twin:55 1aa05c8 2026-10-13); 0.0 (dbf2025_multipoint:32 20ef669 2026-10-20) | dbf2025's comment |
| `stability.x_motor` | −0.30 (full_mdo:71 ccf2f13 2026-10-09); −0.1397 (full_mdo_twin:85 1aa05c8 2026-10-13; dbf2025_multipoint:40 20ef669 2026-10-20) | −0.1397 |
| `takeoff_distance` | 100 (full_mdo:35 ccf2f13 2026-10-09; full_mdo_twin:41 1aa05c8 2026-10-13; lap_mdo:37 cfa742d 2026-10-09); 106.68 (dbf2025_multipoint:10 20ef669 2026-10-20) | 106.68 |
| `mass.inputs.theta_tb_max` (code against config) | 0.01745 (inputs.py:49); 0.017453 (mass_garcia_gamboa_2022:34) | 0.017453 (decision 3) |
| `n_cells` | not moved (a design choice) | — |
| headwind spread | 0–3 m/s (uncertainty.yaml:38 c0e9b1d 2026-10-19) against 0–7 m/s (uncertainty_2025_reduced:16 3d6cccd 2026-10-21) | no conflict now: one per ruleset (decision 1) |

No conflict touches a value that the 2024-25 configs use: dbf2025's line is the newest everywhere.

## Variants that keep explicit overrides (current effective values)

- `full_mdo_fixed_kv.yaml`, `full_mdo_twin_fixed_kv.yaml`, `full_mdo_rdo_fixed_kv.yaml`,
  `full_mdo_twin_rdo_fixed_kv.yaml`: the old single-motor values. Kv 900, Rm 0.05, I0 1.0, m_motor 0.15, m_esc 0.06,
  m_prop 0.03, m_avionics 0.25, e_spec 150, takeoff_distance 100, and x_motor −0.30 for the single layout. They stay
  bit-for-bit, and with them the AIAC twin study and the pinned RDO study.
- `dbf2025_multipoint_rdo_full.yaml`: `uncertainty.yaml` (standalone) plus store_drag_factor, as now.
- `dbf2025_multipoint_rdo_calibrated.yaml`, `_rdo_onesided`: their uncertainty files keep only the f_build override.
- `dbf2025_multipoint_takeoff.yaml`, `_rdo_takeoff`: their take-off block, as now.
- `woombah_2025.yaml`, `mass_usyd_woombah.yaml`: aircraft values, unchanged.
- `configs/verification/`: unchanged.

## What changes (single-point configs, under the most-recent rule)

| Config | Paths that change |
|---|---|
| `full_mdo.yaml`, `full_mdo_rdo.yaml` | e_spec 150→134.3, Kv 900→1050, Rm 0.05→0.03384, I0 1.0→1.79, m_motor 0.15→0.1127, m_esc 0.06→0.127, m_prop 0.03→0.018, m_avionics 0.25→0.222, x_motor −0.30→−0.1397, takeoff_distance 100→106.68 |
| `full_mdo_twin.yaml`, `full_mdo_twin_rdo.yaml` | the same, except x_motor (already −0.1397) |
| `lap_mdo.yaml` | takeoff_distance 100→106.68 |
| `aero_mass.yaml`, `wing_aero.yaml`, `oval_lap.yaml` | nothing |

The full table with commits is printed by the checkpoint 2 test (`tests/test_placeholders.py`).

**References and pages affected (not regenerated; the user decides):**
- `tests/data/configs_reference.json`: entries `full_mdo`, `full_mdo_twin`, `full_mdo_rdo` and `full_mdo_twin_rdo`.
  The four `*_fixed_kv` entries are unchanged.
- `tests/data/kv_dv_reference.json` and `rdo_off_reference.json`: both entries (`full_mdo`, `full_mdo_twin`).
- **Study pages:** `docs/studies/kv_study.md` (it runs `full_mdo.yaml` and `full_mdo_rdo.yaml`), the Kv-as-DV parts
  of `rdo_study.md`, the `examples/full_mdo.py` output, and the UI's default run.
- **Unchanged:** `twin_motor_study.md` (`*_fixed_kv`), `tail_boom_calibration.md`, every 2024-25 page and
  `multipoint_nominal_reference.json`.

## Didn't fit, and how it was settled

- The four questions above (decisions 1–4).
- **`aero_mass.yaml`, `lap_mdo.yaml`, `wing_aero.yaml`, `oval_lap.yaml`** have their own schemas. Each takes only the
  layer keys its schema has; for example, `wing_aero` takes `aero.n_crit` and the ruleset's altitude.
- **`results_2025.py calibrate`** rewrites `uncertainty_2025_reduced_calibrated.yaml` from the text of the reduced
  file. It now writes only the f_build override.

## Status

- **Checkpoint 1** (`cee6a3c`): this inventory.
- **Checkpoint 2:** the files, the loader, the migration and the non-UI verification.
  - **Files:** `configs/placeholders.yaml` (60 values and the 3 shared spreads), `configs/rulesets/dbf_2024_25.yaml` (50
    values, the bottle catalogue and lap.wind) and `configs/rulesets/generic.yaml`.
  - **Code:**
    - `aircrafter2/config/loader.py` (`load_layered`, dotted keys);
    - `aircrafter2/config/layer_files.py` (rows, save in place, "overridden by");
    - the problem loaders; the spread pool in `full_config_from_dict` (`standalone: true`);
    - `StoreConfig`, `Bottle` and `RulesConfig` without code defaults (`StoreConfig.from_ruleset`), with `BOTTLES` read
      from the ruleset;
    - the `inputs.py` theta_tb_max default set to 0.017453.
  - **Verification** (`tests/test_placeholders.py`):
    - each layer overrides the one below;
    - the default placeholders file applies;
    - a single-point load ignores `multipoint:`;
    - the mass layering;
    - every config's `model_dump()` matches the pre-migration snapshot (`tests/data/configs_premigration.json`), except
      the listed single-point changes. Every 2024-25 config and variant is identical;
    - one place per value;
    - every row is tagged;
    - save in place leaves every other line byte-identical;
    - an edit reaches `full_mdo.yaml` and `dbf2025_multipoint.yaml` unless overridden;
    - the reduced RDO set is the shared spreads.

    The mass verification and `multipoint_nominal_reference.json` pass unchanged.
  - **References:** the regression entries the migration changes are strict xfail (`tests/migration.py`), not
    regenerated: `configs_reference.json` (`full_mdo`, `full_mdo_twin`, `full_mdo_rdo`, `full_mdo_twin_rdo`, in
    `test_rdo_reduced` and `test_results_2025`) and `kv_dv_reference.json` (`full_mdo`, `full_mdo_twin`).
  - **`test_kv_dv.py`:** four tests hard-coded the old 900 rpm/V reference motor. They now read the reference motor from
    the config, and the fixed-motor comparison uses `full_mdo.yaml` with its own motor fixed.
- **Checkpoint 3:** the UI.
  - **Tabs:** "Design and mission", "Placeholders" and "Ruleset". The layer tabs run first and the config is loaded over
    their (possibly unsaved) edits. Each layer table has an "overridden by" column, and Save / Discard sit on each tab.
  - **Multipoint configs** show the two layer tabs and an info box, with no Run button.
  - **`AIRCRAFTER2_CONFIGS`** points the app at another configs folder (used by the tests).
  - **Moved widgets** (deleted where they were):
    - to the Placeholders tab: nacelle mass, length and diameter and the energy reserve (sidebar); payload mass and the
      current limit (Key assumptions); `k_lof`, `CL_ground` and the take-off distance (Mission);
    - to the Ruleset tab: the start offset and the mission time.

    The take-off switch, `mu_roll`, the reaction time and the course editor stay in Mission, with a caption pointing to
    the tabs.
  - **UI test lines updated** in `tests/test_ui.py`:
    - `test_streamlit_kv_toggle_bounds_and_validation`: the Kv caption's start is now 1050 (the placeholders file's
      reference motor; a value change, not a moved widget);
    - `test_streamlit_takeoff_and_course_headless`: the `k_lof` and take-off-distance widget checks are replaced by the
      Mission caption check (moved widgets).
  - **New UI tests:**
    - the three tabs, and the multipoint view;
    - an unsaved energy-reserve edit changes an Analyse run (num_y 11);
    - Save writes a temporary copy (one line changes) and Discard restores.
