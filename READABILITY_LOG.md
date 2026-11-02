# Readability pass log

## Status
- **Current checkpoint:** 0 (setup, baseline, ruff, pilot).
- **Last completed module:** none yet.
- **Next step:** finish the golden baseline, then ruff, then the `smooth_lap.py` pilot.

## Baseline
- **Base:** branch `placeholder-fixes-2026` at `e958559`, clean. New branch `readability` at `../AirCrafter2-readability` (neither name existed, so no `-2`).
- **Starting state of the main checkout (`git status`):** on `placeholder-fixes-2026`, nothing to commit, no run in progress (no heavy python processes at the start). Other worktrees left alone: `AirCrafter2-constraints-ui`, `outputs/wt_b2adbb8`.
- **Python environment check:** `PYTHONPATH=<worktree>` with `<main>/.venv/bin/python` (absolute path) gives `aircrafter2.__file__ = .../AirCrafter2-readability/aircrafter2/__init__.py`, the same under the pytest environment. Every check script asserts this. `OMP_NUM_THREADS=3` throughout. The interpreter must be called by its absolute path: a relative `../AirCrafter2/.venv/bin/python` prints a `sys.prefix` warning.
- **Baseline files (outside the repository):** `/tmp/claude-1000/-home-lucas-University-THESIS/0ebc5d39-e686-4673-ba87-97ddb3c4360f/scratchpad/rd/` (`base1/`, `base2/`, `base_opt/`, `anchors_base.json`, `tests/base.txt`, and the throwaway scripts `points.py`, `compare.py`, `astcheck.py`, `anchors.py`, `coverage.py`, `links.py`).

## Decisions
(filled in as work goes on)

## Skipped or reverted changes
(none yet)

## For me to look at
(filled in as work goes on)
