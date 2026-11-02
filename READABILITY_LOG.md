# Readability pass log

## Morning summary
**Outcome: finished, and no number changed.** Branch `readability` (25 commits on `e958559`), worktree `../AirCrafter2-readability`, not merged. Every module in `aircrafter2/` has been through the pass.

**Final verification, on the final code:**
- **Both full optimisations are bit for bit identical.** `rd_final`: 59 iterations, F 5.953236787867297, same `mda_fallbacks` (M2 and M3: 59 solves, 1 continuation, 1 failed attempt). `full_mdo`: 43 iterations, score 80.26302847, all 445 outputs equal. The run JSON was compared with `tag`, `time` and `git` left out.
- **All analysis points, derivatives and MDA attempt paths are bit for bit identical:** 28 points, 254,427 variables. Two runs on the base commit were also identical, so no tolerance was needed anywhere.
- **Full suite: 533 passed, 1 skipped,** same as the base, at every checkpoint. No test fails on the base or on the branch.
- **Static checks:** `ruff check` and `ruff format --check` pass. All 52 anchor patterns match. All 163 doc links resolve.
- **Docstrings missing:** 421/694 to 140/711. Public objects: 0 of 445. The 140 left are framework hooks (`setup`, `compute`, `io`, `evaluate`, ...), which your docstring rule leaves out unless they do something non-obvious. Module docstrings over 10 lines: 39 to 0.
- **Nothing was reverted or skipped** for changing a number.

**Most worth your attention:**
1. **The MATLAB test didn't run.** It's the 1 skip: the APC `.txt` links and shipped baseline it needs aren't in a fresh worktree. Run `tests/test_aircrafter1.py` once in the main checkout after merging. This pass didn't touch that MATLAB code.
2. **The framework-hook exclusion is my call** (Decisions 8). If you want a one-liner on all 140, it's mechanical and safe.
3. **`docs/DECISIONS.md` is new and carries the thesis-facing explanations.** It's worth a read, especially the G&G source notes. One entry is marked Unconfirmed: `plan-d5` (the Table 15 tolerance, which PLAN leaves open).
4. **Frozen strings still carry history labels** ("(D4)", "PLAN 6h T6", "before 2026-10-27", ...): listed below. They'd need a rewording pass, and tests match some strings.
5. **Possible issues, not fixed:** the `apc.parse_apc_file` pitch guess, and the dead code and duplicates listed below. `ruff` 0.17.3 is now installed in the main `.venv`, as the prompt asked; `pyproject.toml` pins it only on this branch.

## Status
- **Current checkpoint:** 4, complete.
- **Last completed module:** all.
- **Next step:** your review; the branch is unmerged.

## Baseline
- **Base:** branch `placeholder-fixes-2026` at `e958559`, clean. New branch `readability` at `../AirCrafter2-readability` (neither name existed, so no `-2`).
- **Starting state of the main checkout (`git status`):** on `placeholder-fixes-2026`, nothing to commit, no run in progress (no heavy python processes at the start). Other worktrees left alone: `AirCrafter2-constraints-ui`, `outputs/wt_b2adbb8`.
- **Python environment check:** `PYTHONPATH=<worktree>` with `<main>/.venv/bin/python` (absolute path) gives `aircrafter2.__file__ = .../AirCrafter2-readability/aircrafter2/__init__.py`, the same under the pytest environment. Every check script asserts this. `OMP_NUM_THREADS=3` throughout. The interpreter must be called by its absolute path: a relative `../AirCrafter2/.venv/bin/python` prints a `sys.prefix` warning.
- **Baseline files (outside the repository):** `/tmp/claude-1000/-home-lucas-University-THESIS/0ebc5d39-e686-4673-ba87-97ddb3c4360f/scratchpad/rd/` (`base1/`, `base2/`, `base_opt/`, `anchors_base.json`, `tests/base.txt`, and the throwaway scripts `points.py`, `compare.py`, `astcheck.py`, `anchors.py`, `coverage.py`, `links.py`).

- **Run times:** full suite 13 min 55 s (534 collected: 533 passed, 1 skipped); analysis points and derivatives 5 min at 3 parallel workers; `rd_base` 323 s; `full_mdo` optimisation about 60 s. Quick check including `test_constraints_view` 7.5 min.
- **Determinism:** two runs of all 28 analysis points and derivatives on the base commit: **bit for bit identical** (254,427 variables, including every input vector and the `RetryNewton` logs). So every later comparison is exact, with no spread needed.
- **Golden results:**
  - `rd_base`: 59 iterations, F 5.953236787867297, r_M2 1.00268, r_M3 0.95351, nothing violated, M1 passes, `mda_fallbacks` M2 and M3 each 59 solves, 1 continuation, 1 failed attempt.
  - `full_mdo`: 43 iterations, success, score 80.26302847.
  - `integer_optimum` F = 5.953236035567057 against the stored `integer_optimum_F` 5.953236035567042 (1.5e-14 apart, path-dependent last bits; the stored value came from the optimiser's own path).
- **Tests:** none fail on the base commit. The skip is `tests/test_aircrafter1.py::test_original_mode_reproduces_the_shipped_code` ("APC .txt links or the shipped baseline are missing"): those links aren't in a fresh worktree, so MATLAB doesn't run here and `check_original.json` isn't rewritten. It's restored after each run anyway.
- **Coverage at the start:** 421/694 missing, matching the survey when nested functions count (public 347/586 by my count, where dunders count as public; the survey's 335/574 differs by that).

### Anchor list at the base commit
The full list is in `anchors_base.json` next to the baselines (52 patterns, file, line and text), checked by `anchors.py` at every quick check.

## Style reference (from the pilot, `mission/smooth_lap.py`)
Before:
```python
def normalise_legs(legs):
    """(kind, size, wind_sign) legs; legacy (kind, size) straights alternate head, tail, head, ... (the old rule)."""
...
        o = self.options
        cd = lambda cl: CD0 + CD1 * cl + k * cl**2  # noqa: E731
```
After:
```python
def normalise_legs(legs):
    """Put a leg list into the (kind, size, wind_sign) form the model flies.

    Parameters
    ----------
    legs : list of tuple
        ("s", length_ft[, wind_sign]) for a straight, ("t", angle_deg[, ...]) for a turn. wind_sign is +1 head, -1 tail,
        0 none. Straights without a sign alternate head, tail, head, ...

    Returns
    -------
    list of tuple
        (kind, size, wind_sign), with wind_sign None for a turn.
    """
...
        opts = self.options

        def cd(cl):
            """Drag coefficient at lift coefficient cl (the towing state, if there is a banner)."""
            return CD0 + CD1 * cl + k * cl**2
```
A component class has Options, Inputs, Outputs and Symbols sections, and one line on partials. The module docstring says what it does, where it sits, its main source and where the details are.

## Checkpoints

### Checkpoint 0
- **Commits:** ruff config and decisions page; frozen statements (`# fmt: off`); format; lint; pilot docstrings; pilot renames and lambdas; pilot nested docstrings.
- **Format:** line length 120 touches 1,493 of 12,444 lines (12 %), below the 30 % limit, so 125 isn't needed (it would save only 200 added lines).
- **Quick check:** identical after lint, and identical after the pilot's renames and lambdas.
- **Full suite:** 533 passed, 1 skipped, same as the base (12 min 49 s).
- **Self-review:** AST checks pass; scan and history words only on anchored lines; the Symbols block and moved-notes table are in place.

### Checkpoint 1
- **Commits:** one per group: mission, aero and airfoil, mass, propulsion/geometry/stability, stores and core helpers, comment fixes, then physics renames and lambdas.
- **Proof by type:** every docstring commit: AST with docstrings stripped identical to the commit before. Rename and lambda commit: analysis points and derivatives bit for bit identical (`qc_cp1`), `test_constraints_view` 103 passed, anchors all match, links all resolve.
- **Full suite:** 533 passed, 1 skipped, same as the base.
- **Self-review:** the helper flags scan and history words in new text. It flagged only anchored lines and the path `archive/legacy`. A wrap tool mangled one equation line in `masses.py` (Eq. 27 joined to the note above); found in review, fixed, and the tool now leaves aligned equation blocks alone. I scanned the rest for the same fault and found none.

### Checkpoint 2
- **Commits:** problems; config, API and CLI; robustness and scoring; renames and lambdas (`full_mdo`, `multipoint`, `api`, `sampling`, score plug-ins).
- **Proof by type:** docstring commits: AST with docstrings stripped identical. Rename and lambda commit: analysis points and derivatives bit for bit identical (`qc_cp2`), `test_constraints_view` 103 passed, anchors all match, links all resolve.
- **Full suite:** 533 passed, 1 skipped, same as the base.
- **Self-review:** coverage of public objects outside checkpoint 3: 0. History words: only `docs/PLAN.md` as the architecture pointer and two false hits ("used to", "no longer" in their plain meanings). No string literal changed (AST check).

### Checkpoint 3
- **Commits:** docstrings for `constraints_view`, `ui`, `plotting`, `vsp` and `verification`, with section comments in long functions. README Layout and links. Abbreviations.
- **Proof by type:** no AST change outside docstrings, so no rename or lambda commit here. `qc_cp3`: bit for bit identical. `test_constraints_view` 103 passed, anchors all match, links all resolve.
- **Full suite:** 533 passed, 1 skipped, same as the base.
- **Self-review:** every Options, Inputs and Outputs name in a class docstring was checked by script to appear in that class's code: no mismatch. A second script listed abbreviations a module used without spelling them out (34 in 26 modules); each of those modules now has an `Abbreviations:` line, and `docs/DECISIONS.md` has the full list. Section comment inserts landed inside two calls in `build_full_model`; found in review and moved above them.

### Checkpoint 4
- **Commits:** README Layout; abbreviations; this log.
- **Full check (code at `bbbf939`; only this log changed after it):** both optimisations, every analysis point, derivatives, attempt paths, full suite, anchors, links, coverage, ruff: see the morning summary. `rd_final`'s `git` field reads `bbbf939-dirty` because this log was being edited while it ran.
- **AST since the format commit:** 16 files differ, all explained: `constraints_view` and `api` (lint), and the rename and lambda files: `smooth_lap`, `course`, `parasite`, `static`, `powertrain`, `apc`, `export`, `stores`, `stores2026`, `dbf2025`, `dbf2026`, `full_mdo`, `multipoint`, `sampling`.
- **`git diff e958559 HEAD` touches only** `aircrafter2/`, `docs/modules/`, `docs/DECISIONS.md`, `README.md`, `READABILITY_LOG.md` and `pyproject.toml`.

## Renames
| File | Function | Old -> new |
|---|---|---|
| `mission/smooth_lap.py` | `SmoothLapModel._model` | `o`->`opts`, `x`->`inputs`, `tc`->`thrust_coeffs`, `pc`->`power_coeffs`, `th`->`wind_dir_rad`, `st`->`is_stowed`, `ban`->`banner_state`, `gs`->`ground_speed`, `cdg`->`cd_ground`, `cdf`->`polar`, `f`->`rates`, `r`->`to_float`, `to`->`takeoff_out`, `tr`->`t_react` |
| `mission/smooth_lap.py` | `SmoothLapModel.setup` | `o`->`opts` |
| `mission/course.py` | `legs_from_rows` | `o`->`order_value` |
| `aero/parasite.py` | `ParasiteDrag.compute` | `x`->`vals` |
| `stability/static.py` | `StaticStability.compute` | `x`->`vals` |
| `propulsion/powertrain.py` | `Powertrain.compute` | `x`->`vals`, `r`->`node_results`, `nu`->`n_units`, `tc`->`t_coeffs` |
| `propulsion/powertrain.py` | `Powertrain.solve_nodes` | `p`->`prop` |
| `stores.py` | `Stores.compute` | `o`->`opts`, `x`->`vals`, `c`->`cfg` |
| `stores2026.py` | `Stores2026.compute` | `o`->`opts`, `x`->`vals`, `c`->`cfg` |
| `stores2026.py` | `PayloadFuselage.compute`, `Packaging2026.compute` | `c`->`cfg` |
| `problems/full_mdo.py` | `build_full_model` | `a`->`aero_cfg`, `st`->`stab_cfg`, `pr`->`prop_cfg`, `co`->`course_cfg`, `geo`->`geo_cfg` |
| `problems/full_mdo.py` | `apply_full_values` | `pr`->`prop_cfg`, `st`->`stab_cfg` |
| `problems/full_mdo.py` | `full_constraints` | `st`->`stab_cfg` |
| `api.py` | `motor_report` | `pr`->`prop_cfg` |
| `scoring/dbf2025.py`, `scoring/dbf2026.py` | `parts` | `o`->`opts` |

## Moved notes: old docstring paragraph -> new location
### `mission/smooth_lap.py` (module docstring, 52 lines)
| Old paragraph | New location |
|---|---|
| Same physics, reformulated | module docstring; `mission.md` "How each part of the lap is modelled" |
| Constant power, drag polar | `mission.md` "Power, aircrafter mode", "Drag polar"; class Symbols |
| Straights, speed hold | `mission.md` "Straights" |
| Wind, crab, legs without a direction, turn drift, ground-roll wind | `mission.md` "Wind" |
| Banner | `mission.md` "Banner" |
| Turns | `mission.md` "Turns"; `omega` and `a_turn` docstrings |
| Lap 1, take-off | `mission.md` "Lap 1" and "Take-off" (the equation of motion, integration, floors and assumptions were already in "Take-off") |
| Lap count; "lapCount (integer) is reported only" | `mission.md` "Lap count"; the anchored phrase stays in the class docstring, Outputs |
| `power_mode` | `mission.md` "Power, propulsion mode"; class Options |
| Complex-step safe | class docstring; `mission.md` opening of the section |

### `mission/lap_modeller.py` (61 lines)
| Old paragraph | New location |
|---|---|
| Port of Kallarackel Ch. 3, MATLAB files, steps 1-6 | `mission.md` "Reference port" |
| Margins instead of a NaN flag | module docstring; `mission.md` "Deliberate deviations" |
| Banner block left out | `mission.md` "Deliberate deviations" |
| Lap 1 Leg 1 not added, `Tstatic` unused, the switch | `mission.md` "Reproduced as they are" |
| `lapCount` is a step function | `mission.md`; the `AirCrafterPerformance` docstring |

### Others in checkpoint 1
| Module | Old notes | New location |
|---|---|---|
| `mission/course.py` | leg forms, presets, validation, take-off switch | module docstring; `mission.md` "Course"; `check_course` docstring |
| `mission/oval.py` | load factor, floor, turn radius | `mission.md` "Oval"; component docstrings |
| `aero/parasite.py` | Raymer build-up, tails, fuselage, nacelles | `aero.md` "Parasite drag"; `ParasiteDrag` docstring |
| `aero/polar.py` | three points, why not two | `aero.md` "Polar" |
| `aero/profile_drag.py` | strip equations | `NeuralFoilStripDrag` docstring |
| `airfoil/neuralfoil_section.py` | solve, implicit function, clamp, source | `aero.md` "NeuralFoil sections"; `solve_sections`, `section_sensitivities` docstrings |
| `airfoil/shape.py` | shape equations, TE input, no full CST | `AirfoilShape` docstring; `aero.md` |
| `propulsion/powertrain.py` (41 lines) | battery, ESC, motor, prop, topologies | `propulsion.md` (already there: "Powertrain", "Battery topology") |
| `propulsion/powertrain.py` | fit range, outputs | `propulsion.md` "Fit range", "Outputs for the lap model"; the anchored `rpm_max` phrase in the class docstring |
| `propulsion/apc.py` | file format, surrogate, folder | `propulsion.md` "Propeller data"; `parse_apc_file`, `PropSurrogate` docstrings |
| `propulsion/motor_scaling.py` | rewind laws | `MotorRewind` docstring; `propulsion.md` |
| `geometry/planform.py` | boom, volumes | `geometry.md`; `TailGeometry` docstring |
| `stability/static.py` | neutral point | `StaticStability` docstring; `stability.md` |
| `stores.py` (20 lines) | rules, model, bottle drag | new `stores.md` |
| `stores2026.py` (23 lines) | rules, model | new `stores.md`; `Stores2026` docstring |
| `smooth.py` (25 lines) | each function | that function's docstring |
| `analytic.py` | how to subclass | module docstring (shortened) |
| `geometry/export.py`, `atmosphere.py` | kept | shortened in place |

### Checkpoint 2
| Module | Old notes | New location |
|---|---|---|
| `problems/full_mdo.py` | model structure, design variables, objective | `problems.md` "full_mdo"; `FullMDA`, `build_full_model` docstrings |
| `problems/multipoint.py` | model tree, design variables, M1 | `problems.md` "multipoint"; `m1_check` docstring (also the deleted "pass =" comment) |
| `problems/wing_aero.py`, `oval_lap.py`, `aero_mass.py`, `lap_mdo.py` | formulations, trades | `problems.md`; a short form in `wing_aero`'s module docstring |
| `scoring/dbf2026.py` (30 lines) | rules verbatim, model, `floored` | `scoring.md` "dbf2026": "Verbatim", "Model" |
| `scoring/dbf2025.py` | rules verbatim, model | `scoring.md` "dbf2025" (already there); `DBF2025.parts`, `__init__` docstrings |
| `scoring/base.py` | how to write a plug-in, with its example | `docs/how_to_add_a_scoring_plugin.md` (already there); `ScoringPlugin` docstring |
| `robustness/rdo_problem.py` | model tree, problem statement | `build_robust_problem` docstring; `robustness.md` |
| `robustness/rdo_problem.py` `RetryNewton` | fallback orders, continuation | short form in the class; `_continuation` docstring; `robustness.md` "MDA failures at the nodes" |
| `robustness/rdo_multipoint.py` | tree, statement, parameters | `build_robust_multipoint` docstring; module docstring |
| `robustness/stats.py`, `quadrature.py`, `uncertainty.py` | formulas, rules | `RobustStats`, `Rule`, `udr`, `smolyak`, `sobol`, `Dist`, `screen` docstrings |
| `robustness/suite.py` | three functions | their docstrings; `robustness.md` section 1 |
| `robustness/sampling.py` | retry, failures | `_run` docstring; module docstring |
| `config/layer_files.py` | file format | new `config.md` "Layer files" |
| `api.py` | what a run writes | module docstring (shortened) |

### Checkpoint 3
| Module | Old notes | New location |
|---|---|---|
| `constraints_view.py` (21 lines) | sections, join to `REGISTER`, how to add a constraint, one-node rule | new `constraints.md`; `setup_problem` docstring |
| `vsp/build.py`, `render.py`, `check.py` | frame, what's illustrative, rendering | shortened in place; `vsp.md` (already there); the list of what comes from the result stays in `mockup_assumptions` and `vsp.md` |
| `verification/hand_calcs.py` | three methods | those functions' docstrings |
| `plotting/*`, `ui/app.py` | kept | shortened in place |

## Decisions
1. **Coverage counts nested functions.** That's the only count that matches the survey's 694. Options: nested excluded (658). Undo: read the figures as such. Nested helpers get one-line docstrings so the public target of 0 holds under this count.
2. **`evaluate(self, x)` keeps `x`.** It's a method signature defined by `AnalyticComponent` and overridden everywhere, and the base class documents it. `SmoothLapModel._model(self, x, ...)` is private and called positionally, so it became `inputs`, as the prompt's example says.
3. **E741 is ignored in the ruff config** (`I`, `l` are equation symbols, which the prompt keeps). Options: per-line `noqa`. Undo: remove `ignore` and add the `noqa`s.
4. **Only assigned lambdas (E731) become named functions.** Lambdas passed inline (sort keys, `applies=`, `progress=`) stay, since they aren't what the `noqa` silences and turning them into defs would mean moving code.
5. **`F541` accepted** on `api.py`'s `f"# AirCrafter 2.0 result\n"`: the string's value is the same, and it's ruff's safe fix. It's the one literal whose source text changed outside docstrings.
6. **Ruff format also touched two docstrings' indentation** (`cli.py`, `problems/wing_aero.py`), so the format commit's AST is identical with docstrings stripped, and differs in those two docstrings' whitespace only. Neither is read at run time.
8. **Framework hooks don't need a docstring** (`initialize`, `setup`, `compute`, `compute_partials`, `io`, `evaluate`, ...), as the prompt's rule says, unless they do something non-obvious, so the public target of 0 excludes them. The coverage line reports them separately. Options: a one-liner on each, which would only say "declare the options". Undo: remove the exclusion in the coverage script and write them.
9. **Public count** follows the survey's: no leading underscore, so dunders are private. That gives its total.
10. **`x` -> `vals` in `compute` methods** where `inputs` is already taken, since `compute`'s `inputs` is the OpenMDAO vector and `x` the plain dict of it. `c` -> `cfg` in the stores, where `c` otherwise reads as the chord. Physics symbols in `evaluate` methods (`na`, `gR`, `ne`, `dne`, `s`) stayed and got a Symbols block, under the prompt's "if unsure, keep and explain".
11. **`docs/modules/problems.md`, `config.md`, `stores.md`, `constraints.md` are new pages,** as no existing page fitted. Linked from the README's Documentation list at wrap-up.
12. **Decision entries beyond the prompt's list:** `plan-g22`, `plan-g23` (PLAN has them, the prompt's range stopped at G21), `plan-m1`, `plan-m7`, `plan-t6` (labels the code used), `mr-quiet`, `tc-takeoff`, `r25-multipoint`, `r26-scoring`, `r26-payload`, `r26-banner`, `r26-wind`, `rr-f-build`, `pr-layers` (other plan files, the same pattern).
13. **Plan-file references in code became `DECISIONS.md` links,** except inside frozen strings. `docs/PLAN.md` stays as a pointer to the architecture in `aircrafter2/__init__.py` and `plotting/xdsm.py`, since that's a live document for it, not a label.
14. **Section comments in the longest functions only:** `build_full_model`, `FullMDA.setup`, `build_multipoint_model`, `build_robust_problem`, `build_robust_multipoint`, `_finish`, `main`, `constraints_tab`, `vsp_check`. The others over 70 lines already had comments that show their structure (`evaluate` by equation, `vsp.build` by part, `solve_sections` by numbered step), or are one data structure (`export_geometry`).
15. **`Abbreviations:` line in the module docstring,** instead of rewriting each first use. It's one line per module, and easy to remove. Undo: delete the line.
7. **`# fmt: off` on six statements** (`smooth_lap` x2, `full_mdo` x3, `garcia_gamboa/group.py`), found by a trial format against the anchor list. Other anchored lines kept their matched text; a few changed around it (for example a closing bracket).

## Skipped or reverted changes
None reverted: no check ever differed. Left undone on purpose: inline lambdas (Decisions 4); terse physics names in `evaluate` methods (Decisions 10); `st` in `export_geometry` (For me to look at).

## For me to look at
- **Dead code:** `mission/smooth_lap._sigmoid` isn't called anywhere in the repository.
- **`geometry/export.py`, `export_geometry`:** `st` is assigned and never used. Deleting it is ruff's unsafe fix and outside "may change", so it has `# noqa: F841`.
- **`mass/garcia_gamboa/masses.BUILD_NOTE`:** a module-level string used only as a note that docstrings point to. It's a public name, so it stays, and it still cites `docs/RDO_REDUCED_PLAN.md`. Worth turning into a comment.
- **Tests and the README:** the MATLAB test is skipped in this worktree (see Baseline), so it should be run once in the main checkout after a merge.
- **Strings left alone that would read better reworded** (frozen literals with history labels): `config/aero.py` `n_modes` "(D3)"; `problems/full_mdo.py` `n_motors` and `prop` "(discrete, D4; ...)" and `nacelle_ff` "(PLAN 6h T6)"; `propulsion/powertrain.py` `prop` "(discrete choice, D4)"; `mission/course.py` `lap1_cluster` "before 2026-10-27", `CL_ground` "docs/TAKEOFF_COURSE_PLAN.md", `turn_drift` "§4.7"; `constraints_view.py` notes "(D2)", "(report only, D8)".
- **Unconfirmed in `docs/DECISIONS.md`:** one entry, `plan-d5` (Table 15 tolerance). PLAN says it's open, and nothing later closes it.
- **Docstrings that disagreed with the code** (code kept, docstring fixed):
  - `sampling.evaluate` said it returns `cpu_workers`; the key is `workers`.
  - `neuralfoil_section.section_sensitivities` named `dClmax_dlnRe` and `dClmax_dk`; the keys are `Clmax_dlnRe` and `Clmax_dk`.
  - `api._run_robust` said it returns four values; it returns five.
  - `RetryNewton` said "for a robust node"; `multipoint` points use it too.
  - `DBF2026Score` left out `r_M2` and `r_M3`.
- **Still stale in a doc I didn't rewrite:** `docs/modules/mission.md`, "Course": "`{kind: turn, angle_deg}`: turns ignore drift". True only with `turn_drift` off. Left as is, since `Wind` above it says it properly; one word to fix.
- **Possible problems (none fixed):**
  - `propulsion/apc.parse_apc_file` guesses decimals from the file name (`p_in > d_in * 1.6` means one decimal in the pitch). A real high-pitch prop would be misread.
  - `rdo_problem.build_robust_problem` imports `add_full_driver` and then `del`s it, which does nothing useful.
  - `constraints_view.effective_limit` has a parameter `kw` that nothing reads.
  - `problems/full_mdo.py` sets g = 9.80665, while the `g` input default of `SmoothLapModel` is 9.81 and `lap_modeller.G` is 9.81. The full model overrides the first; the reference port keeps 9.81 to match the MATLAB, which is deliberate. A stand-alone `SmoothLapModel` gets 9.81.
- **Dead code:** `stores2026.LB` and `stores2026.OZ_FT2`, `multipoint.kv_settings_report`, `vsp/build.EXPORT_SET`, and `mission/lap_modeller.AirCrafterPerformance` outside its own demo (nothing else imports it). All public names, so all kept.
- **Duplicated code:**
  - `stores.py` and `stores2026.py` share only the pattern (mass, station, drag), not code, and serve separate rulesets, so I'd keep both. Their unit constants `LB`, `INCH` repeat, and `LB` also in `scoring/dbf2025.py`.
  - The `_value`/`_val` one-liner is written out in `api`, `robustness/suite`, `sampling`, `uncertainty`, `rdo_problem` and `rdo_multipoint`.
  - `multipoint.initial_values` and `set_dv_defaults` repeat one start-value rule; `build_robust_multipoint` repeats `_node_table`.
  - `lap_modeller.isa_density` and `atmosphere.isa_density` agree, and the reference port keeps its own on purpose.
  - The polar block in `problems/lap_mdo.add_lap_model` and `full_mdo.build_full_model` is the same.
- **Split suggestions for long functions (not split):**
  - `build_full_model`: one helper per section comment; the input-defaults table first.
  - `FullMDA.setup`: stores and drag areas apart from the balances and solvers.
  - `SmoothLapModel._model`: `straight`, `turn`, `ground_roll` and `lap` could become methods with the polar and thrust passed in.
  - `ui/app.constraints_tab`: its section comments mark the cuts; "editors" and "saving" first.
  - `vsp/build.build`: geometry, read-back and export.
  - `api._finish`: values, then each file.
  - `cli.main`: one function per command.
- **`aero/performance.CriticalSectionCLmax`:** its docstring says the textbook reference is "to be added in docs/modules/aero.md". It still isn't there.

## Skipped or reverted changes
None reverted: no check ever differed. Left undone on purpose: inline lambdas (Decisions 4); terse physics names in `evaluate` methods (Decisions 10); `st` in `export_geometry` (For me to look at).

