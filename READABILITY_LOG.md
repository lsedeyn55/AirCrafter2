# Readability pass log

## Status
- **Current checkpoint:** 0, full suite running.
- **Last completed module:** `mission/smooth_lap.py`.
- **Next step:** checkpoint 1, physics disciplines.

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

## Renames
| File | Function | Old -> new |
|---|---|---|
| `mission/smooth_lap.py` | `SmoothLapModel._model` | `o`->`opts`, `x`->`inputs`, `tc`->`thrust_coeffs`, `pc`->`power_coeffs`, `th`->`wind_dir_rad`, `st`->`is_stowed`, `ban`->`banner_state`, `gs`->`ground_speed`, `cdg`->`cd_ground`, `cdf`->`polar`, `f`->`rates`, `r`->`to_float`, `to`->`takeoff_out`, `tr`->`t_react` |
| `mission/smooth_lap.py` | `SmoothLapModel.setup` | `o`->`opts` |

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

## Decisions
1. **Coverage counts nested functions.** That's the only count that matches the survey's 694. Options: nested excluded (658). Undo: read the figures as such. Nested helpers get one-line docstrings so the public target of 0 holds under this count.
2. **`evaluate(self, x)` keeps `x`.** It's a method signature defined by `AnalyticComponent` and overridden everywhere, and the base class documents it. `SmoothLapModel._model(self, x, ...)` is private and called positionally, so it became `inputs`, as the prompt's example says.
3. **E741 is ignored in the ruff config** (`I`, `l` are equation symbols, which the prompt keeps). Options: per-line `noqa`. Undo: remove `ignore` and add the `noqa`s.
4. **Only assigned lambdas (E731) become named functions.** Lambdas passed inline (sort keys, `applies=`, `progress=`) stay, since they aren't what the `noqa` silences and turning them into defs would mean moving code.
5. **`F541` accepted** on `api.py`'s `f"# AirCrafter 2.0 result\n"`: the string's value is the same, and it's ruff's safe fix. It's the one literal whose source text changed outside docstrings.
6. **Ruff format also touched two docstrings' indentation** (`cli.py`, `problems/wing_aero.py`), so the format commit's AST is identical with docstrings stripped, and differs in those two docstrings' whitespace only. Neither is read at run time.
7. **`# fmt: off` on six statements** (`smooth_lap` x2, `full_mdo` x3, `garcia_gamboa/group.py`), found by a trial format against the anchor list. Other anchored lines kept their matched text; a few changed around it (for example a closing bracket).

## Skipped or reverted changes
(none yet)

## For me to look at
- **Dead code:** `mission/smooth_lap._sigmoid` isn't called anywhere in the repository.

## Skipped or reverted changes
(none yet)

