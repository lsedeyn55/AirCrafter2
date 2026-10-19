# Motor Kv as a design variable: plan (checkpoint 1)

Brief: `THESIS/Claude outputs/aircrafter2_kv_dv_prompt.md`. Branch `kv-dv` from `rdo` HEAD (f462408).

## 1. "What I already know", checked against the code

| # | Claim | Verdict |
|---|---|---|
| 1 | `Kv` is a promoted top-level input; `Powertrain` has cs partials of every electrical output w.r.t. `Kv` | **Correct** (`powertrain.py:117-120`, `full_mdo.py:263-264`). No new derivative code in the powertrain; the new rewind component needs its own (analytic) partials. |
| 2 | `full_constraints` leaves out `I_margin` on the grounds that the static current doesn't depend on the DVs; with Kv a DV it does, and Kv will run to its upper bound past `I_max` | **Comment and check: correct** (`full_mdo.py:372-373`, `check_fixed_limits`). **The prediction is probably wrong for the default props:** `rpm_margin` is already a constraint and, on the bare powertrain with the rewind scaling, binds first (10x7E 4S at v = 38 m/s: rpm limit 0.85 × 15000 = 12750 reached near Kv ≈ 975, where I_static ≈ 40 A; I_max = 60 A is reached only near Kv ≈ 1150). So I expect Kv to ride `rpm_margin`, not `I_max`, at I_max ≥ ~45 A. That's an estimate at a fixed speed: the full model's top speed moves with Kv. `I_margin` is still added as the brief says. |
| 3 | `Rm` and `I0` are fixed when Kv changes | **Correct** (independent inputs). |
| 4 | In RDO, `set_val("nodes.n{j}.Kv")` hits the one shared input when Kv is a DV, so the nodes overwrite each other and the DV | **Correct, and silent.** Checked on a toy ParallelGroup: all three writes succeed, the last one wins, and every node sees it. `full_mdo_rdo.yaml` and `full_mdo_twin_rdo.yaml` have `Kv` and `Rm` in `active`. |
| 5 | `suite.py` (`monte_carlo`, `worst_cases`, `sensitivities`) and `api.py` (~154) set or perturb `Kv`/`Rm` by name; `api.py` would list `Kv` twice | **Partly wrong.** `suite.py` never names Kv or Rm; its callers do: `api.py:154` (duplicate `Kv`, **correct**), `examples/full_mdo.py:89` (same duplicate) and its Monte Carlo over `configs/uncertainty.yaml`, `studies/twin_motor_study.py:50`. **New hazard:** with Kv a DV, `Rm` and `I0` become computed outputs, so `set_val("Rm", …)` is silently overwritten on the next run and `compute_totals(wrt="Rm")` fails. |
| 6 | `default_full_dvs()` is used only when a YAML has no `design_variables` block; `full_mdo.yaml` and `full_mdo_twin.yaml` have their own | **Correct** (`full_mdo.py:437`). Also: the RDO configs `extends:` those two, so they would silently gain Kv. And a `FullMDOConfig` built directly (not through the loader) gets the 4-DV default of `AeroMassProblemConfig`, not `default_full_dvs()`. Nothing does that today; noted only. |
| 7 | `ui/app.py` ~104-109 has a DV bounds table that writes `Bounds`; Kv appears there once it's a DV; ~117 is the Kv number input | **Correct** (table 104-111, input 117). Kv's row would show a blank `initial`, because nothing is in `cfg.initial`; I'd prefill it with Kv_ref. |
| 8 | `tests/rdo_off_snapshot.py` / `test_rdo.py` compare RDO-off runs to `tests/data/rdo_off_reference.json` (fixed Kv) | **Correct** (configs `full_mdo.yaml`, `full_mdo_twin.yaml`). The test isn't bit-for-bit: rtol 1e-9 on outputs, 1e-7 on totals, 1e-6 after 3 SLSQP iterations, because of BLAS thread-count noise (RDO checkpoint 3). |

Other things found:
- **No git tag exists** (`git tag` is empty). The AIAC twin study is commit `58344f2` ("Twin motors, checkpoint 3: single vs twin study for the AIAC paper"). Nothing in this work rewrites history.
- **`verification/hand_calcs.py` doesn't exist.** The hand check goes in `tests/test_propulsion.py`.
- **The loader can't delete a key** (`config/loader.py`: dicts deep-merge). So a fixed-Kv config can't `extends: full_mdo.yaml` and drop Kv. A null value can't mean "remove" either, because `y_motor_m: null` already means "unset" (§3, decision A).
- The robust problem starts each DV at `cfg.initial.get(name, midpoint)` (`rdo_problem.py:172`): for Kv that's 1000, not Kv_ref = 900. Needs a fix.
- `studies/vsp_mockup.py` builds the **AIAC paper figure** from the stored twin-study designs with `full_mdo.yaml` / `full_mdo_twin.yaml`, and checks the lap time to 1e-4.
- The UI's config list keeps only YAMLs whose text contains `propulsion:`, so a short `extends:` config without that key wouldn't be listed.
- APC data range: at Kv = 500 and 1500, rotor speeds stay inside the tabulated rpm range for 10x7E, 11x7E and 8x6E, so there is no polynomial extrapolation. Low Kv gives little static thrust (10x7E: 10.5 N at Kv 600), so the level-speed balance may fail near the lower bound. That needs a corner check (§4).
- Kv = Kv_ref reproduces the fixed motor **exactly** (Rm_ref · 1.0², I0_ref · 1.0 are exact in floating point).

## 2. Every place that sets Kv, Rm or I0 by name

| Where | What | Change |
|---|---|---|
| `propulsion/powertrain.py:90,117,136-141` | inputs and model | none |
| `problems/full_mdo.py:79-81` | `PropulsionInputs` fields | the reference motor (description only) |
| `problems/full_mdo.py:263-264` | promotes `Kv`, `Rm`, `I0` | DV mode: the powertrain's inputs come from the rewind component |
| `problems/full_mdo.py:400` | `apply_full_values` sets them | DV mode: set `Kv_ref`, `Rm_ref`, `I0_ref` (not `Rm`, `I0`) |
| `api.py:154` | sensitivity "assumptions" | drop DVs; DV mode: `Kv_factor`, `Rm_ref` |
| `ui/app.py:117` | Kv number input | relabel and show Rm, I0 (§3) |
| `configs/full_mdo*.yaml` | reference motor values | none (now the reference motor) |
| `configs/full_mdo_rdo.yaml:14`, `full_mdo_twin_rdo.yaml:10` | `active: [Kv, Rm, …]` | `Kv_factor`, `Rm_ref` |
| `configs/uncertainty.yaml:23-24` | Kv, Rm spreads | add `Kv_factor`, `Rm_ref` (§3, decision C) |
| `examples/full_mdo.py:89,112` | assumptions; Monte Carlo over the shared list | as `api.py` |
| `studies/twin_motor_study.py:50` | `ASSUMPTIONS` | pinned to fixed Kv: unchanged |
| `tests/test_propulsion.py:28-31`, `tests/test_twin_motor.py:100-123` | powertrain unit tests | none (component level) |
| `tests/test_rdo.py:108,117`, `tests/test_ui.py:28` | RDO with `active = [Kv]` | pin / update (§5) |
| `verification/neuralfoil_fd_steps.py` | `Rm` is a Reynolds number | not related |

## 3. Design

**Motor rewind** (`propulsion/motor_scaling.py`, `MotorRewind`, analytic partials). It is added only when `"Kv"` is in
`cfg.design_variables`:

    Kv_eff = Kv · Kv_factor                    (Kv_factor = 1 except at RDO nodes)
    Rm     = Rm_ref · (Kv_ref / Kv_eff)²       (turns ∝ 1/Kv; fixed copper volume: R ∝ turns²)
    I0     = I0_ref · Kv_eff / Kv_ref          (no-load torque Q0 = I0/Kv held constant)
    m_motor unchanged (same frame)

So Km = 1/(Kv √Rm) is constant. `Kv_ref = cfg.propulsion.Kv`. The powertrain gets `Kv_eff`, `Rm` and `I0` from the
component. The promoted name `Kv` stays the DV, and `Rm_ref`, `I0_ref` and `Kv_factor` are new promoted inputs. The
fixed-Kv model is built exactly as now: no new component, no new constraint, no new DV ref.

**Constraints (DV mode only).** `I_margin ≥ 0` (ref 10 A), plus `I_pack_margin ≥ 0` (ref 10 A) when `pack_I_max` is
set. `rpm_margin` is unchanged. I'll update the `full_constraints` comment and the `check_fixed_limits` docstring.
**Scaling.** `DV_UNITS["Kv"] = "rpm/V"`, `DV_REF = {"Kv": 1000.0}`, used by `add_full_driver` and the robust driver.
**Start.** `cfg.initial["Kv"]` if given, else Kv_ref, in both drivers. The robust midpoint default gets fixed for Kv.

**Decisions I need from you** (A–C are implementation choices the brief leaves open; D–E concern results):

- **A. How the fixed-Kv configs drop Kv.** I recommend a loader directive, `fixed_design_variables: [Kv]`. `load_full_config` removes those names from `design_variables` before validation, with an error if a name isn't a DV. Then `full_mdo_fixed_kv.yaml` is `extends: full_mdo.yaml` plus that line, and it resolves to exactly today's `full_mdo.yaml`. Alternatives:
  - standalone copies (about 100 lines each, can drift);
  - make the fixed-Kv files the base and have `full_mdo.yaml` extend them to add Kv (inverts which file is primary).
- **B. RDO clash checks, both ways.** With Kv a DV, `Kv`/`Rm` in `robust.active` fails validation, as briefed. I'd also make a fixed-Kv config with `Kv_factor`/`Rm_ref` in `active` fail: in that mode the rewind component doesn't exist, so those parameters would have nothing to act on. The UI takes the robust block of `full_mdo_rdo.yaml` or `full_mdo_rdo_fixed_kv.yaml` according to the Kv toggle, so you never see the error there.
- **C. One shared `uncertainty.yaml` for both modes.** Add `Kv_factor` (nominal 1.0, ±0.05) right after `Kv`, and `Rm_ref` (relative ±0.20) right after `Rm`. Each of the four gets a new field, `kv_mode: fixed | dv`; the default `both` applies to everything else. The resolved list then has the same 15 parameters in the same order in either mode. In fixed mode it is exactly today's list, so the RDO study's all-parameter Monte Carlo (its Sobol' dimensions follow list order) reproduces. Explicit `active` names are still checked strictly (B), so nothing is silently remapped.
- **D. Which results follow the new default and which are pinned to fixed Kv** (§5). Pinned: the twin study, the VSP/AIAC figure, the RDO study, the RDO plan's screening and node-check scripts, and the twin verification harnesses. **Your call:**
  - `examples/full_mdo.py`, which writes `docs/problems/full_mdo.md`. That doc is already stale (it predates the twin layout and the 15-parameter Monte Carlo). I'd move it to the default and regenerate it in checkpoint 4.
  - `studies/make_thesis_figures.py`, which writes `outputs/thesis_figures/` (not in git). Default or pinned?
- **E. The current-limit sweep.** I'll run it as briefed (I_max 40/50/60/70 A). Per §1 item 2, I expect `rpm_margin`, not I_max, to set Kv above about 45 A. If so, the plot will show Kv flat there and I'll say so plainly.

**UI.** Checkbox "Optimise motor Kv", on by default; off removes `Kv` from `cfg.design_variables`.
- Bounds and start: in **the existing DV bounds table**, where Kv's row appears by itself, with `initial` prefilled with Kv_ref. If the row turns out not to behave cleanly when toggled (the `data_editor` keeps edits by row), I'll switch to dedicated inputs and say so.
- Relabel the Kv input to "Reference motor Kv (rpm/V) - PLACEHOLDER" with Rm_ref and I0_ref beside it.
- Check before running: lower < upper, start within the bounds, bounds within 100–5000.
- After a run: optimal Kv, the scaled Rm and I0, I_static vs I_max, and what set Kv. For the last, I look for a bound on Kv (within tolerance) or the active current/rpm constraint.

**Real-motor check** (`api.reanalyse_with_motor(result_or_cfg, dv, Kv, Rm, I0, m_motor)`): builds the fixed-Kv problem with the given motor, sets the design, runs the analysis only, and returns the score and constraint changes against the continuous-Kv design. It's also offered on the results page. No catalogue.

**`suite.py`:** a guard so that setting or perturbing a computed output (e.g. `Rm` with Kv a DV) raises a clear error instead of being silently ignored.

## 4. Verification

- **Exact fixed-Kv check.** The existing reference test runs on the fixed-Kv configs, plus a one-off exact-equality comparison at `OMP_NUM_THREADS=3` (the reference's thread count). The reference file stays as it is. A new reference, `tests/data/kv_dv_reference.json`, covers the Kv-DV defaults.
- **Tests:**
  - `check_partials` on `MotorRewind`;
  - Km constant, and Kv = Kv_ref reproduces the fixed motor exactly;
  - `check_totals` of score, `I_margin` and `rpm_margin` w.r.t. Kv on the full model;
  - the default config has Kv, and the fixed-Kv configs don't;
  - the RDO clash, both ways;
  - an RDO d = 1 `Kv_factor` test: nodes differ, the DV is shared;
  - the UI toggle, bounds and validation;
  - the hand check at two Kv values (static current, thrust and rpm from an independent solve with the scaled Rm and I0).
- **Corner check before the comparison:** the MDA at Kv = 500 and 1500 with the other DVs at the start point, and at the deterministic optimum.

## 5. Files

**New:**
- `propulsion/motor_scaling.py`;
- `configs/full_mdo_fixed_kv.yaml`, `full_mdo_twin_fixed_kv.yaml`, `full_mdo_rdo_fixed_kv.yaml`, `full_mdo_twin_rdo_fixed_kv.yaml`;
- `tests/test_kv_dv.py`, `tests/data/kv_dv_reference.json`;
- `studies/kv_study.py`, `docs/studies/kv_study.md`, `docs/studies/kv_study_results.json`;
- this plan.

**Changed (code):**
- `problems/full_mdo.py`: docstring, defaults, `DV_UNITS`/`DV_REF`, model, constraints, driver, values, loader directive, RDO validator;
- `robustness/rdo_problem.py`, `uncertainty.py`, `suite.py`;
- `api.py`, `ui/app.py`, `cli.py` (help text only).

**Changed (configs, studies, docs):**
- `configs/full_mdo.yaml`, `full_mdo_twin.yaml` (+Kv), `full_mdo_rdo.yaml`, `full_mdo_twin_rdo.yaml`, `uncertainty.yaml`;
- `examples/full_mdo.py`;
- pinned to fixed Kv: `studies/twin_motor_study.py`, `vsp_mockup.py`, `rdo_study.py` (with a map from the config names stored in its JSONs to the fixed-Kv files), `rdo_screening.py`, `rdo_node_check.py`, `verification/twin/mda_sweep.py`, `opt_twin.py`;
- docs: `modules/propulsion.md` (derivation), `modules/robustness.md`, `problems/full_mdo.md` (D), `OPEN_QUESTIONS.md` (new Q34: Kv bounds and the reference motor, PLACEHOLDER; Q7 cross-reference), `README.md`.

**Published results.** None of these is regenerated:
- `docs/studies/twin_motor_study.md` and its JSON;
- `docs/studies/rdo_study.md` and its JSON;
- `docs/RDO_PLAN.md`;
- `docs/studies/tail_boom_calibration.md`, the verification docs, and the lap/aero-mass/oval problem docs (other problems: not affected).

Pinning a study script changes no numbers. The docs keep naming `full_mdo.yaml`, which was true when they were generated.

**Tests (fixed-Kv check vs default):**
- `test_rdo.py`:
  - regression: pinned to the fixed-Kv configs;
  - `robust_small`: pinned to `full_mdo_rdo_fixed_kv.yaml` (checks old behaviour);
  - new `Kv_factor` tests on the default.
- `test_full_mdo.py`: stays on the default, so its `check_totals` now covers Kv and `I_margin` (updated).
- `test_ui.py` robust toggle: updated to `Kv_factor` (default).
- `rdo_off_snapshot.py`: config list becomes a parameter.
- Run on the default and expected to pass unchanged (analysis or plumbing only, not the DV list): `test_twin_motor.py`, `test_geometry_export.py`, `test_scoring.py`, `test_cli.py`, `test_vsp.py`. Each one I have to change will be listed in checkpoint 2.
