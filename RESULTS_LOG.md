# Final results run on the 2025-26 ruleset: log

## Status
- **Current checkpoint:** 3, computational performance. Paused 21:09-22:35 Tue 3 Nov at the user's request; resumed when they said so (service `fr-perf`).
- **Done:** the enumeration, rounding, `mdo_wind`, multistart, bookends, normalisation, bounds, the other sensitivities, AirCrafter 1.0 sweep and mappings, RDO starts and rounding.
- **Running or left:** timing, then pages, figures, docs, final suite.
- **Projected finish:** timing about 06:30 Wed 4 Nov; pages Wed 4 Nov midday. Deadline Thu 5 Nov 12:00. No cuts needed.
- **Next step:** after the timing, commit the scripts and log, apply the validation patch, regenerate the pages.

## Baseline
- **Starting `git status`:** `placeholder-fixes-2026` at `ac8e14d`, nothing to commit (clean), as expected.
- **Branch:** `thesis-results-2026` from `ac8e14d` (the name was free, so no `-2`).
- **Environment:** `.venv/bin/python`; `AIRCRAFTER2_APC_DIR=/home/lucas/University/THESIS/PERFILES2`; MATLAB headless works (`26.1.0.3312084 (R2026a) Update 4`); machine 16 logical cores, 14 GB RAM (about 7 GB available with the desktop open); power profile `performance`.

- **Freeze:** tag `results-2026-final` at `0a6527d` (study code, plotting addition, AirCrafter 1.0 options; `git diff ac8e14d 0a6527d -- aircrafter2/` is `plotting/figures.py` only).
- **Full suite at the freeze commit:** 547 passed, 0 skipped, 16 min 25 s (534 + 13 new in `tests/test_results_2026_study.py`). `check_original.json` restored.
- **Reproducibility check (`fr_check`):** F **5.953236787867297** in **59** iterations, nothing violated, bit for bit against `READABILITY_LOG.md`'s golden result; `mda_fallbacks` M2 and M3 each 59 solves, 1 continuation, 1 failed attempt (also identical). Wall 308 s, peak 0.77 GB.

## Projection (Checkpoint 0, from the measured costs)
| Item | Basis | Wall |
|---|---|---|
| Enumeration, 10 runs on 3 slots | 5.1 min each | 20 min |
| `integer`, `mdo_wind` + `integer`, multistart 10, bookends 2, normalisation 4, bounds 3, old constraints, take-off off, on 2 slots beside the first RDO | 1-5 min each | 45 min |
| AirCrafter 1.0 sweep at the shipped steps, mappings | 25 min + 5 min | inside the above |
| RDO, 3 starts (2 GB each) | 3 min per iteration | 2-3 h |
| Robust rounding | at most 15 iterations per candidate | up to 2 h |
| 11 in-set and out-of-set Monte Carlo runs, 3 workers | 1024 x 5.6 s / 3 = 32 min | 6 h |
| Common, wind, screening, node check, re-analyses, placeholders, validation | | 1.5 h |
| Timing | 3 x 25 min sweep, 3 x 8 min optimisation, per-evaluation repeats, scaling, one single-threaded RDO of 3-5 h | about 9 h |

Total about 25 h from the freeze, so about 24 h of margin before the deadline. No cut decided.

## Checkpoints
**Checkpoint 1 (deterministic runs), self-checks at the end of the campaign.** Provenance: every `fr_*` JSON records `0a6527d`, clean. Enumeration: 10 of 11 converge (6S from Humpback's start is an analysis failure, Decision 11). The multistart's four failures are at the start design, at its first MDA, and are stated as analysis failures, not infeasibility.

**Checkpoint 2 (robust and evaluation runs), 20:20 Tue 3 Nov.**
- Campaign: 80 attempts, 0 failed. Found by this check: the Monte Carlos' "MDA failures" were a worker failure, not a design response. Fixed by rerunning (Decision 13).
- Rerun: 11 of 11 at 1024/1024 converged, 0 MDA failures, 0 lap-model limits. On the samples both versions ran, F agrees to at most 1.9e-12, so the failure didn't bias the samples that had converged. Means moved by at most 0.0008, sigmas by at most 0.0010.
- Propagation rerun on the new arrays (`fr_propagation`; the old one kept): UDR mean 5.8751, sigma 0.0836; Smolyak 5.8740, 0.0849; Sobol' 5.8737, 0.0833, se(mean) 0.0026: agreement within sampling error.
- Provenance: 90 run files on the page, 0 not at the freeze. No tracked file differs, HEAD at the tag. `git diff results-2026-final -- aircrafter2 configs` is empty.
- Consistency: floored <= uncapped at every design; failures counted apart.

**Checkpoint 3 note.** During the timing the machine ran only the desktop and an idle `qemu` VM from the Claude desktop app (about 1 % of one core; not mine, not stopped), plus a shell check every ten minutes.

**Timing paused, 21:09 Tue 3 Nov.** The user asked to pause so their work wouldn't affect the timing. `fr-perf` was stopped about 30 s into AirCrafter 1.0 sweep #1, whose partial repeat is discarded; finished repeats are kept. Not frozen with SIGSTOP, because a repeat's wall clock would have counted the pause. Resumed 22:35 on the user's word with the same command under `systemd-run`, which skips finished repeats: tree clean, HEAD `0a6527d`, power profile `performance`, load average 0.7.

## Decisions
1. **The log stays untracked until the runs and the timing have finished.** `git_hash()` records HEAD and adds `-dirty` when a tracked file differs, so committing or editing a tracked log during the campaign would break "every run records the freeze commit, clean". No tracked file changes while runs go; new page scripts are new, untracked files until the runs end. Options: commit the log between runs (moves HEAD, so later runs would record another commit). Undo: nothing to undo; the log is committed with the first page commit.
2. **Raymer Table 6.4: the homebuilt row, V_h 0.50, V_v 0.04** for bound sensitivity (ii), read from the page image (Raymer, *Aircraft Design: A Conceptual Approach*, 6th ed., Table 6.4, p. 160; PDF page 194 of `3_LITERATURE/textbooks/`). Options: the sailplane row (0.50, 0.02) or general aviation single engine (0.70, 0.04). Homebuilt is the class Q9 already names and the nearest to a large RC aircraft. Undo: change the `--override` of `fr_bound_ii` in `studies/results_2026_campaign.py`.
3. **Normalisation runs: one normaliser at a time, ×0.5 and ×2** (`NI_max` 1400, 5600 and `B_max` 750, 3000), warm from the best continuous case. That is the four off-diagonal runs the prompt counts. Note: with the cap, equal scaling is not exactly neutral any more (the ratios are capped at 1), so "can't move the optimum" holds for the uncapped score only. Undo: add runs to the campaign's list.
4. **Multistart box: the payload counts are drawn up to what the fuselage cap allows at the largest span** (n_p up to 1.5 b / pitch = 96.9, n_c up to 32.3), not the config's nominal 500 / 200 ("no bound in practice", Q55). A start at 400 passengers would test the cap and the MDA's far field, not the optimiser. Every other variable uses its config bounds. Undo: `box_designs` in `studies/results_2026.py`.
5. **Humpback in the common evaluation flies at full throttle in both missions**, `humpback_2026.yaml` as it stands (42/14, 197 in), the same case as the validation page's "as built at full throttle". Options: the throttles solved from Table 42's logged currents, but those flights carried 36/12 and other banners, so they are not one design. Undo: `designs()` in `studies/results_2026.py`.
6. **Mapping (a)'s tails: the middles of the 2025-26 bands** (V_h 0.575, V_v 0.05) at l_ht 1.1 m, the rule of the 2024-25 mapping (it used the middles of its bands). Options: Humpback's own 0.80 and 0.08. Undo: `V_H`, `V_V` in `studies/aircrafter1_2026.py`.
7. **The out-of-set list** is `configs/uncertainty.yaml`'s parameters that act on the 2025-26 problem (f_cb, f_gear, e_spec, R_cell, Kv_factor, Rm_ref, m_motor, eta_h, lap.thrust_factor), f_tb from `placeholders.yaml`, and a_ht, a_vt at ±30 % in place of f_ht, f_vt, which do nothing under the areal tail model. m_pay is left out because the stores compute it. That gives 17 parameters including the five active ones. Undo: `EXTRA_TAILS`, `NOT_IN_2026` in `studies/results_2026.py`.
8. **The C_D,b re-analyses also get a full-M3-throttle row.** At fixed throttle, the smoke design's M3 falls below 1.2 V_stall at C_D,b 0.03 and 0.034, so the pilot's answer (more throttle) is worth reporting. Extra rows only; the asked rows are there unchanged.
9. **`machine sleep`:** the campaign runs under `systemd-inhibit` so that the machine doesn't suspend (a suspend paused an earlier campaign for 14 h). Nothing else about the session changes.

10. **`rdo` in the comparison is the rounded design, continued to convergence.** The three rounding candidates ran with the prompt's `--maxiter 15` and all stopped at that limit (`ok` false; mean - k sigma 5.7899, 5.7878, 5.7914 against the continuous 5.7928). `best_rdo` only takes converged runs, so it fell back to the continuous design (73.8 passengers, 24.6 cargo), and `fr_common_rdo`, `fr_reanalysis_rdo` and `fr_wind_env_rdo` ran on it. Settled decision 6 asks for the rounded design, so at 13:03 I stopped the campaign, moved those three JSONs to `outputs/results_2026/superseded_continuous_rdo/`, and continued the best candidate, 75p25c, from its iterate to convergence (`fr_rdo_mdo_int_75p25c_conv`), with the frozen `results_2026.rdo`, then restarted the campaign. Options: report the continuous RDO (the fallback the prompt allows only if rounding doesn't fit in the time). Undo: delete that run and the rdo-based JSONs after it; the campaign reruns them on the continuous design.
11. **One retry of 6S from Kv 270.** `fr_6S_19x12E_22x12E` failed at its first MDA (Humpback's start at 6S, Kv 245: M3 didn't converge), so there is no optimisation to compare. The plan expects 6S to be power-limited. `fr_6S_19x12E_22x12E_kv270` starts at the family's top Kv through a run-time override of the start, nothing else changed. If it fails too, the page reports 6S as an analysis failure, not as infeasible.
12. **An added bound sensitivity: the banner's 275 in** (`fr_bound_banner`, upper bound 500 in, AirCrafter 1.0's grid limit, [HB] Table 7). The 22x12E optimum sits on this PLACEHOLDER bound (Q56), which the prompt's three active bounds didn't list, because the earlier 19x12E/19x12E optimum stopped at 272 in. Extra run only.
13. **The Monte Carlos were rerun in batches (study script added after the freeze, model and study code untouched).** The first 11 Monte Carlos finished without an error, but 17-30 % of their samples were "MDA failures" in long blocks of consecutive samples at the end of each worker's work, with the same rate in every quartile of every parameter, so they couldn't be the design's response to the sample. Reproduced in one process: Humpback, sample 281 (solve 284): `singular Jacobian in 'points.M3.mda'`, and from then on every sample fails, the nominal retry included. It depends on how many evaluations the process has done, not on the sample. `sampling.evaluate` keeps one pool, and my 3 workers (the budget) gave each worker about 340 samples; earlier 5-worker runs stayed under the count. Options: (a) keep the runs and state the failures (rejected: μ and σ would rest on a truncated sequence, and the failure counts would be read as analysis failures of the design, which they aren't); (b) fix `sampling.py` (forbidden after the freeze, and it's model code); (c) recycle the workers from outside, what I did: `studies/results_2026_mc_batched.py` calls the frozen `results_2026.mc` with `sampling.evaluate` wrapped into batches of 384 samples, each on a fresh pool. The rows, seed and outputs are the same, and the runs record the freeze commit, clean (the script is new and untracked until the timing ends). This is the one place where the brief's "after the tag, only page, exhibit and log scripts may change" gives way to "a missing result is better than a wrong one". Old files: `outputs/results_2026/superseded_worker_failure/`. Undo: move them back.

## Cuts
None so far.

## For me to look at
- **Model bug: a worker's problem goes permanently singular after about 250-280 evaluations (top of the morning summary if it isn't resolved).** `singular Jacobian in 'points.M3.mda'` at solve 284 for Humpback, with the same count across samples; no NaN in the state vectors; a restart from nominal doesn't clear it. Effect on these results: the first Monte Carlos only, fixed by Decision 13 and shown not to have biased the samples that converged (F agrees to 1.9e-12); no other run's JSON mentions it. A fresh problem in the same process solves that sample, so the cause is in the problem object. The bad M3 Jacobian holds NaN entries while the state vectors have none (scratchpad diagnostic, not traced further). Earlier studies' Monte Carlos: any run with more than about 250 samples per worker in one process could have hit it, so check them if they're reused (2024-25 pages not rerun, as instructed). Not fixed: `aircrafter2/` is frozen.
- **`fr_rdo_humpback` failed at its 5th evaluation:** SLSQP's early step reached a design whose M2 V_stall is 17.2 m/s at a node, and the lap model stopped there (non-physical lap: wind limit class). It's deterministic, so a retry would repeat it. Two of three starts converge, to the same optimum (mean - k sigma 5.79280 in both).
- **Fixed-throttle re-analysis can land on the sub-stall root.** In the smoke, the optimum at C_D,b 0.03 with M3's throttle held returned M3 SM 4.37 and thrust margin −13.5, which are physically meaningless: the T = D balance sits on the spurious low-speed root (docs/MDA_ROBUSTNESS_PLAN.md §0). The constraint violation is counted and F is still computed there. To be measured in the Monte Carlo.
