# 2025-26 results campaign: run log

Record of the final 2025-26 results campaign (tag `results-2026-final`): a summary of the outcome, the baseline and
projection, the checks at each stage, the decisions taken during the campaign, open issues and the run table. The
result pages are in `docs/studies/results_2026/`; the plan is `docs/development/RESULTS_2026_PLAN.md`.

<!-- summary -->
## Summary
**Outcome.** Every planned run finished; no cuts. The four designs, the sensitivities, the Monte Carlos and the timing are done, and the pages are in `docs/studies/results_2026/` (start at `INDEX.md`). In AirCrafter 2.0's own model, MDO reaches F 5.9775, against 5.2845 for Humpback and 5.6579 for AirCrafter 1.0 mapped (b). RDO gives up 0.0413 of nominal F, keeps the mean, cuts sigma from 0.1175 to 0.0833 and takes P(any violation) from 0.071 to 0.000 in set. Out of set it is 0.108 to 0.021, so the robustness is not tuned only to the five parameters. These are comparisons inside one model that favours its own optima, not predictions: the model misses most of Humpback's flight criteria. Tests: 547 passed at the freeze, 550 at the final commit. Branch `thesis-results-2026`, tag `results-2026-final` (`0a6527d`).

| design | F nominal | floored | MC mean | MC sigma | mean - k sigma | P(any) in set | P(any) out of set | MC failed |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| Humpback | 5.2845 | 5.1948 | 5.2672 | 0.0486 | 5.2186 | 0.293 | 0.305 | 0 + 0 of 1024 |
| AC1 mapped (b) | 5.6579 | 5.5839 | 5.5889 | 0.0843 | 5.5046 | 0.354 | – | 0 + 0 of 1024 |
| MDO | 5.9775 | 5.8588 | 5.8683 | 0.1175 | 5.7507 | 0.071 | 0.108 | 0 + 0 of 1024 |
| MDO, mean wind | 5.9718 | 5.8385 | 5.8743 | 0.1094 | 5.7650 | 0.137 | 0.187 | 0 + 0 of 1024 |
| MDO + RDO | 5.9362 | 5.9120 | 5.8737 | 0.0833 | 5.7904 | 0.000 | 0.021 | 0 + 0 of 1024 |

**Wall times (single-threaded):** AirCrafter 1.0 sweep 16.6 min; MDO from Humpback 6.0 min; RDO to convergence 0.79 h.

**Main findings and caveats:**
1. **A solver-state defect affected the first Monte Carlos; the reported results come from the rerun.** A worker's problem goes permanently singular after about 250-280 evaluations, which cost 17-30 % of the samples. All 11 Monte Carlos were rerun in batches on fresh workers, with a study script added after the freeze (Decision 13). All samples now converge, and F agrees to about 2e-12 where both runs converged. The defect itself is unfixed, and Monte Carlos of earlier studies may have been affected ("Open issues").
2. **Validation is weak on flight.** §3.7's criteria: structure 3 of 5 rows pass, flight 1 of 7. M3 laps run 20-46 % too fast. F is a comparison between designs, not a predicted score (`validation_2026.md`).
3. **Much of the optimum is set by PLACEHOLDERS.** The tail volumes, chord, camber and banner sit on PLACEHOLDER bounds, and M2 sits at the cap, so the normalisation (Q52) sets the payload: F moves -0.489 at B_max x2 and -0.448 at NI_max x2. Bound changes are smaller: results_2026.md §5.3.
4. **Read RDO's gain as variance and feasibility, not mean.** 73 of 1024 MDO samples sit on the sub-stall balance at fixed throttle. Without them MDO's mean is 5.8785, above RDO's 5.8737. Those samples are violations with an F that means little.
5. **Judgement calls.** The rounded RDO needed more than the planned 15 iterations (Decision 10). The multistart converges from only 6 of 10 starts, all to one optimum, and the others are analysis failures at the start design; one of three RDO starts fails too. The 13-variable scaling run did not converge in 100 iterations, so that scaling curve isn't clean.
<!-- /summary -->

## Status
- **State:** complete (2026-11-04). The timing was paused 21:09-22:35 on 3 Nov, between measurements.
- **Done:** every run on the list, the pages, exhibits, docs and final suite.

## Baseline
- **Starting `git status`:** `placeholder-fixes-2026` at `ac8e14d`, nothing to commit (clean), as expected.
- **Branch:** `thesis-results-2026` from `ac8e14d` (the name was free, so no `-2`).
- **Environment:** `.venv/bin/python`; `AIRCRAFTER2_APC_DIR=/home/lucas/University/THESIS/PERFILES2`; MATLAB headless works (`26.1.0.3312084 (R2026a) Update 4`); machine 16 logical cores, 14 GB RAM (about 7 GB available with the desktop open); power profile `performance`.

- **Freeze:** tag `results-2026-final` at `0a6527d` (study code, plotting addition, AirCrafter 1.0 options; `git diff ac8e14d 0a6527d -- aircrafter2/` is `plotting/figures.py` only).
- **Full suite at the freeze commit:** 547 passed, 0 skipped, 16 min 25 s (534 + 13 new in `tests/test_results_2026_study.py`). `check_original.json` restored.
- **Full suite at the page commit `b1b3159`:** 550 passed, 0 failed, 16 min 30 s (547 + 3 new in `tests/test_results_2026_pages.py`). `check_original.json` restored. The only commit after it changes this log.
- **Reproducibility check (`fr_check`):** F **5.953236787867297** in **59** iterations, nothing violated, bit for bit against the golden result of the earlier readability pass; `mda_fallbacks` M2 and M3 each 59 solves, 1 continuation, 1 failed attempt (also identical). Wall 308 s, peak 0.77 GB.

## Projection (before the campaign, from the measured costs)
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

## Stages
**Stage 1 (deterministic runs), self-checks at the end of the campaign.** Provenance: every `fr_*` JSON records `0a6527d`, clean. Enumeration: 10 of 11 converge (6S from Humpback's start is an analysis failure, Decision 11). The multistart's four failures are at the start design, at its first MDA, and are stated as analysis failures, not infeasibility.

**Stage 2 (robust and evaluation runs), 20:20 Tue 3 Nov.**
- Campaign: 80 attempts, 0 failed. Found by this check: the Monte Carlos' "MDA failures" were a worker failure, not a design response. Fixed by rerunning (Decision 13).
- Rerun: 11 of 11 at 1024/1024 converged, 0 MDA failures, 0 lap-model limits. On the samples both versions ran, F agrees to at most 1.9e-12, so the failure didn't bias the samples that had converged. Means moved by at most 0.0008, sigmas by at most 0.0010.
- Propagation rerun on the new arrays (`fr_propagation`; the old one kept): UDR mean 5.8751, sigma 0.0836; Smolyak 5.8740, 0.0849; Sobol' 5.8737, 0.0833, se(mean) 0.0026: agreement within sampling error.
- Provenance: 90 run files on the page, 0 not at the freeze. No tracked file differs, HEAD at the tag. `git diff results-2026-final -- aircrafter2 configs` is empty.
- Consistency: floored <= uncapped at every design; failures counted apart.

**Stage 3 note.** During the timing the machine ran only the desktop and an idle `qemu` virtual machine (about 1 % of one core), plus a shell check every ten minutes.

**Stage 3 (computational performance), 00:18 Wed 4 Nov.** 18 measurements, all at `0a6527d`, clean, one thread. Sweep repeats 989-1005 s; `fr_perf_rdo` reproduces `fr_rdo_mdo`'s optimum (mean - k sigma 5.7928) in the same 45 iterations. Fixed on the page, after looking at it: the main row had pulled in the scaling runs (a glob), the core count read the thread setting, the reliability row had picked up smoke runs, and the numbers printed as exponents. The 13-variable scaling run didn't converge in 100 iterations, stated on the page, not rerun: it's a measurement.

**Stage 4 (pages and exhibits), Wed 4 Nov.** Figures: all 24 opened and viewed. Fixed: the enumeration's labels and its two 6S cases, the scaling legends over the data, a clipped ranking label, the P(violation) axis and a note that RDO has none, RDO's line style on the CDFs (it read like MDO in greyscale), and the failed C_D,b 0.03 case, which the sensitivity figure had left out silently. Spot-checks: five or more numbers per page against the JSON and, for F and sigma, against the run JSON and arrays: all agree. Pages: 91 run files, 0 off the freeze; page commit and run commit recorded on every page. `git diff results-2026-final -- aircrafter2 configs` empty. Consistency: the same F on every page; floored <= uncapped at all six designs; UDR and Sobol' within sampling error; failures counted apart. 2024-25 pages not regenerated, and `docs/studies/results/` has no diff. `planform()` was rebuilt on a shared helper at the freeze (plotting only); the 2024-25 page generation wasn't rerun, as instructed.

**Timing paused, 21:09 Tue 3 Nov.** Paused so that other work on the machine would not affect the timing. `fr-perf` was stopped about 30 s into AirCrafter 1.0 sweep #1, whose partial repeat is discarded; finished repeats are kept. Not frozen with SIGSTOP, because a repeat's wall clock would have counted the pause. Resumed 22:35 with the same command under `systemd-run`, which skips finished repeats: tree clean, HEAD `0a6527d`, power profile `performance`, load average 0.7.

## Decisions
1. **The log stays untracked until the runs and the timing have finished.** `git_hash()` records HEAD and adds `-dirty` when a tracked file differs, so committing or editing a tracked log during the campaign would break "every run records the freeze commit, clean". No tracked file changes while runs go; new page scripts are new, untracked files until the runs end. Options: commit the log between runs (moves HEAD, so later runs would record another commit). Undo: nothing to undo; the log is committed with the first page commit.
2. **Raymer Table 6.4: the homebuilt row, V_h 0.50, V_v 0.04** for bound sensitivity (ii), read from the page image (Raymer, *Aircraft Design: A Conceptual Approach*, 6th ed., Table 6.4, p. 160; PDF page 194 of `3_LITERATURE/textbooks/`). Options: the sailplane row (0.50, 0.02) or general aviation single engine (0.70, 0.04). Homebuilt is the class Q9 already names and the nearest to a large RC aircraft. Undo: change the `--override` of `fr_bound_ii` in `studies/results_2026_campaign.py`.
3. **Normalisation runs: one normaliser at a time, ×0.5 and ×2** (`NI_max` 1400, 5600 and `B_max` 750, 3000), warm from the best continuous case. That is the four off-diagonal runs the specification counts. Note: with the cap, equal scaling is not exactly neutral any more (the ratios are capped at 1), so "can't move the optimum" holds for the uncapped score only. Undo: add runs to the campaign's list.
4. **Multistart box: the payload counts are drawn up to what the fuselage cap allows at the largest span** (n_p up to 1.5 b / pitch = 96.9, n_c up to 32.3), not the config's nominal 500 / 200 ("no bound in practice", Q55). A start at 400 passengers would test the cap and the MDA's far field, not the optimiser. Every other variable uses its config bounds. Undo: `box_designs` in `studies/results_2026.py`.
5. **Humpback in the common evaluation flies at full throttle in both missions**, `humpback_2026.yaml` as it stands (42/14, 197 in), the same case as the validation page's "as built at full throttle". Options: the throttles solved from Table 42's logged currents, but those flights carried 36/12 and other banners, so they are not one design. Undo: `designs()` in `studies/results_2026.py`.
6. **Mapping (a)'s tails: the middles of the 2025-26 bands** (V_h 0.575, V_v 0.05) at l_ht 1.1 m, the rule of the 2024-25 mapping (it used the middles of its bands). Options: Humpback's own 0.80 and 0.08. Undo: `V_H`, `V_V` in `studies/aircrafter1_2026.py`.
7. **The out-of-set list** is `configs/uncertainty.yaml`'s parameters that act on the 2025-26 problem (f_cb, f_gear, e_spec, R_cell, Kv_factor, Rm_ref, m_motor, eta_h, lap.thrust_factor), f_tb from `placeholders.yaml`, and a_ht, a_vt at ±30 % in place of f_ht, f_vt, which do nothing under the areal tail model. m_pay is left out because the stores compute it. That gives 17 parameters including the five active ones. Undo: `EXTRA_TAILS`, `NOT_IN_2026` in `studies/results_2026.py`.
8. **The C_D,b re-analyses also get a full-M3-throttle row.** At fixed throttle, the smoke design's M3 falls below 1.2 V_stall at C_D,b 0.03 and 0.034, so the pilot's answer (more throttle) is worth reporting. Extra rows only; the asked rows are there unchanged.
9. **`machine sleep`:** the campaign runs under `systemd-inhibit` so that the machine doesn't suspend (a suspend paused an earlier campaign for 14 h). Nothing else about the session changes.

10. **`rdo` in the comparison is the rounded design, continued to convergence.** The three rounding candidates ran with the specified `--maxiter 15` and all stopped at that limit (`ok` false; mean - k sigma 5.7899, 5.7878, 5.7914 against the continuous 5.7928). `best_rdo` only takes converged runs, so it fell back to the continuous design (73.8 passengers, 24.6 cargo), and `fr_common_rdo`, `fr_reanalysis_rdo` and `fr_wind_env_rdo` ran on it. Settled decision 6 asks for the rounded design, so at 13:03 I stopped the campaign, moved those three JSONs to `outputs/results_2026/superseded_continuous_rdo/`, and continued the best candidate, 75p25c, from its iterate to convergence (`fr_rdo_mdo_int_75p25c_conv`), with the frozen `results_2026.rdo`, then restarted the campaign. Options: report the continuous RDO (the fallback the specification allows only if rounding doesn't fit in the time). Undo: delete that run and the rdo-based JSONs after it; the campaign reruns them on the continuous design.
11. **One retry of 6S from Kv 270.** `fr_6S_19x12E_22x12E` failed at its first MDA (Humpback's start at 6S, Kv 245: M3 didn't converge), so there is no optimisation to compare. The plan expects 6S to be power-limited. `fr_6S_19x12E_22x12E_kv270` starts at the family's top Kv through a run-time override of the start, nothing else changed. If it fails too, the page reports 6S as an analysis failure, not as infeasible.
12. **An added bound sensitivity: the banner's 275 in** (`fr_bound_banner`, upper bound 500 in, AirCrafter 1.0's grid limit, [HB] Table 7). The 22x12E optimum sits on this PLACEHOLDER bound (Q56), which the specified three active bounds didn't list, because the earlier 19x12E/19x12E optimum stopped at 272 in. Extra run only.
13. **The Monte Carlos were rerun in batches (study script added after the freeze, model and study code untouched).** The first 11 Monte Carlos finished without an error, but 17-30 % of their samples were "MDA failures" in long blocks of consecutive samples at the end of each worker's work, with the same rate in every quartile of every parameter, so they couldn't be the design's response to the sample. Reproduced in one process: Humpback, sample 281 (solve 284): `singular Jacobian in 'points.M3.mda'`, and from then on every sample fails, the nominal retry included. It depends on how many evaluations the process has done, not on the sample. `sampling.evaluate` keeps one pool, and the 3 workers (the budget) gave each worker about 340 samples; earlier 5-worker runs stayed under the count. Options: (a) keep the runs and state the failures (rejected: μ and σ would rest on a truncated sequence, and the failure counts would be read as analysis failures of the design, which they aren't); (b) fix `sampling.py` (forbidden after the freeze, and it's model code); (c) recycle the workers from outside, which was done: `studies/results_2026_mc_batched.py` calls the frozen `results_2026.mc` with `sampling.evaluate` wrapped into batches of 384 samples, each on a fresh pool. The rows, seed and outputs are the same, and the runs record the freeze commit, clean (the script is new and untracked until the timing ends). This is the one place where the specification's "after the tag, only page, exhibit and log scripts may change" gives way to "a missing result is better than a wrong one". Old files: `outputs/results_2026/superseded_worker_failure/`. Undo: move them back.

## Cuts
None.

## Open issues
- **Model bug: a worker's problem goes permanently singular after about 250-280 evaluations.** `singular Jacobian in 'points.M3.mda'` at solve 284 for Humpback, with the same count across samples; no NaN in the state vectors; a restart from nominal doesn't clear it. Effect on these results: the first Monte Carlos only, fixed by Decision 13 and shown not to have biased the samples that converged (F agrees to 1.9e-12); no other run's JSON mentions it. A fresh problem in the same process solves that sample, so the cause is in the problem object. The bad M3 Jacobian holds NaN entries while the state vectors have none (scratchpad diagnostic, not traced further). Earlier studies' Monte Carlos: any run with more than about 250 samples per worker in one process could have hit it, so check them if they're reused (2024-25 pages not rerun, as instructed). Not fixed: `aircrafter2/` is frozen.
- **`fr_rdo_humpback` failed at its 5th evaluation:** SLSQP's early step reached a design whose M2 V_stall is 17.2 m/s at a node, and the lap model stopped there (non-physical lap: wind limit class). It's deterministic, so a retry would repeat it. Two of three starts converge, to the same optimum (mean - k sigma 5.79280 in both).
- **Fixed-throttle re-analysis can land on the sub-stall root.** In the smoke, the optimum at C_D,b 0.03 with M3's throttle held returned M3 SM 4.37 and thrust margin −13.5, which are physically meaningless: the T = D balance sits on the spurious low-speed root (docs/development/MDA_ROBUSTNESS_PLAN.md §0). The constraint violation is counted and F is still computed there. To be measured in the Monte Carlo.

<!-- run table -->
## Run table
From `outputs/results_2026/campaign_status.md`; every `fr_*` run. The 11 Monte Carlo rows are the batched reruns, at the same commit (Decision 13). `fr_propagation` was rerun on the new arrays.

Updated 2026-11-03 17:01:51 (freeze 0a6527d). Running: none. Failed: none.

| tag | what | start → wall | its (nit) | converged | F capped | F floored | MDA fallbacks | commit |
|---|---|---|---:|---|---:|---:|---|---|
| fr_check |  | ends 03 Nov 10:03, 308 s | 59 (51) | True | 5.9532 | 5.8474 | M2 1c; M3 1c | 0a6527d |
| fr_8S_18x10E_22x12E |  | ends 03 Nov 10:08, 325 s | 63 (53) | True | 5.9756 | 5.8615 | M2 0c; M3 0c | 0a6527d |
| fr_8S_18x10E_18x12E |  | ends 03 Nov 10:08, 326 s | 61 (54) | True | 5.9427 | 5.8359 | M2 0c; M3 0c | 0a6527d |
| fr_8S_18x10E_19x12E |  | ends 03 Nov 10:09, 366 s | 67 (58) | True | 5.9513 | 5.8476 | M2 0c; M3 1c | 0a6527d |
| fr_8S_19x10E_18x12E |  | ends 03 Nov 10:14, 334 s | 63 (56) | True | 5.9419 | 5.8348 | M2 0c; M3 0c | 0a6527d |
| fr_8S_19x10E_19x12E |  | ends 03 Nov 10:14, 343 s | 64 (56) | True | 5.9505 | 5.8464 | M2 0c; M3 1c | 0a6527d |
| fr_8S_19x10E_22x12E |  | ends 03 Nov 10:15, 314 s | 61 (53) | True | 5.9748 | 5.8612 | M2 0c; M3 0c | 0a6527d |
| fr_8S_19x12E_18x12E |  | ends 03 Nov 10:20, 344 s | 62 (55) | True | 5.9446 | 5.8358 | M2 1c; M3 0c | 0a6527d |
| fr_8S_19x12E_19x12E |  | ends 03 Nov 10:20, 334 s | 59 (51) | True | 5.9532 | 5.8474 | M2 1c; M3 1c | 0a6527d |
| fr_8S_19x12E_22x12E |  | ends 03 Nov 10:21, 367 s | 69 (60) | True | 5.9775 | 5.8588 | M2 1c; M3 0c | 0a6527d |
| fr_6S_19x12E_22x12E |  | ends 03 Nov 10:21, 17 s | 0 (None) | False | – | – | M2 0c; M3 0c | 0a6527d |
| fr_8S_19x12E_22x12E_int_check |  | ends 03 Nov 10:22, 0 s | – (–) | – | – | – | – | 0a6527d |
| fr_8S_19x12E_22x12E_int_72p23c |  | ends 03 Nov 10:24, 147 s | 31 (23) | True | 5.9697 | 5.8413 | M2 0c; M3 1c | 0a6527d |
| fr_8S_19x12E_22x12E_int_71p23c |  | ends 03 Nov 10:27, 162 s | 34 (26) | True | 5.9668 | 5.8359 | M2 0c; M3 1c | 0a6527d |
| fr_8S_19x12E_22x12E_int_72p24c |  | ends 03 Nov 10:27, 19 s | 2 (1) | True | 5.9775 | 5.8588 | M2 0c; M3 1c | 0a6527d |
| fr_wind |  | ends 03 Nov 10:33, 333 s | 63 (51) | True | 5.9226 | 5.8320 | M2 1c; M3 0c | 0a6527d |
| fr_wind_int_check |  | ends 03 Nov 10:33, 0 s | – (–) | – | – | – | – | 0a6527d |
| fr_wind_int_71p23c |  | ends 03 Nov 10:36, 184 s | 37 (30) | True | 5.9160 | 5.8081 | M2 0c; M3 1c | 0a6527d |
| fr_wind_int_72p24c |  | ends 03 Nov 10:37, 30 s | 6 (1) | True | 5.9213 | 5.8385 | M2 0c; M3 1c | 0a6527d |
| fr_M2only |  | ends 03 Nov 10:38, 44 s | 8 (4) | True | 5.9740 | 5.8588 | M2 0c; M3 1c | 0a6527d |
| fr_M3only |  | ends 03 Nov 10:39, 65 s | 12 (8) | True | 5.3083 | 5.2189 | M2 0c; M3 1c | 0a6527d |
| fr_norm_NI0.5 |  | ends 03 Nov 10:40, 68 s | 13 (8) | True | 6.0000 | 5.9167 | M2 0c; M3 1c | 0a6527d |
| fr_norm_NI2 |  | ends 03 Nov 10:43, 152 s | 27 (26) | True | 5.5291 | 5.4454 | M2 0c; M3 1c | 0a6527d |
| fr_norm_B0.5 |  | ends 03 Nov 10:44, 44 s | 8 (4) | True | 6.0000 | 5.9421 | M2 0c; M3 1c | 0a6527d |
| fr_norm_B2 |  | ends 03 Nov 10:45, 55 s | 11 (6) | True | 5.4887 | 5.4004 | M2 0c; M3 1c | 0a6527d |
| fr_ac1_direct |  | ends 03 Nov 10:47, 0 s | 0 (–) | True | 5.4883 | 5.3000 | M2 0c; M3 0c | 0a6527d |
| fr_bound_i |  | ends 03 Nov 10:49, 223 s | 45 (37) | True | 5.9973 | 5.8715 | M2 0c; M3 1c | 0a6527d |
| fr_ac1_completed |  | ends 03 Nov 10:51, 220 s | 45 (37) | True | 5.6579 | 5.5839 | M2 0c; M3 0c | 0a6527d |
| fr_bound_ii |  | ends 03 Nov 10:52, 181 s | 36 (31) | True | 5.9667 | 5.8556 | M2 0c; M3 1c | 0a6527d |
| fr_old_constraints |  | ends 03 Nov 10:53, 18 s | 2 (1) | True | 5.9775 | 5.8588 | M2 0c; M3 1c | 0a6527d |
| fr_bound_iii |  | ends 03 Nov 10:55, 226 s | 45 (38) | True | 5.9564 | 5.8549 | M2 0c; M3 1c | 0a6527d |
| fr_takeoff_off |  | ends 03 Nov 10:57, 230 s | 57 (39) | True | 5.9818 | 5.8924 | M2 0c; M3 1c | 0a6527d |
| fr_ms1 |  | ends 03 Nov 10:58, 44 s | 0 (None) | False | – | – | M2 1c; M3 0c | 0a6527d |
| fr_ms2 |  | ends 03 Nov 10:59, 38 s | 0 (None) | False | – | – | M2 1c; M3 0c | 0a6527d |
| fr_ms0 |  | ends 03 Nov 11:01, 384 s | 76 (62) | True | 5.9775 | 5.8588 | M2 1c; M3 0c | 0a6527d |
| fr_ms3 |  | ends 03 Nov 11:05, 330 s | 66 (56) | True | 5.9775 | 5.8588 | M2 0c; M3 0c | 0a6527d |
| fr_ms5 |  | ends 03 Nov 11:06, 37 s | 0 (None) | False | – | – | M2 1c; M3 0c | 0a6527d |
| fr_ms6 |  | ends 03 Nov 11:07, 42 s | 0 (None) | False | – | – | M2 1c; M3 0c | 0a6527d |
| fr_ms4 |  | ends 03 Nov 11:08, 371 s | 72 (59) | True | 5.9775 | 5.8588 | M2 1c; M3 1c | 0a6527d |
| fr_ms7 |  | ends 03 Nov 11:14, 390 s | 77 (66) | True | 5.9775 | 5.8588 | M2 0c; M3 0c | 0a6527d |
| fr_ms8 |  | ends 03 Nov 11:15, 440 s | 86 (70) | True | 5.9775 | 5.8588 | M2 1c; M3 1c | 0a6527d |
| fr_rdo_mdo | RDO μ | ends 03 Nov 11:16, 2900 s | 51 (45) | True | 5.8764 | – | 11c | 0a6527d |
| fr_ms9 |  | ends 03 Nov 11:19, 297 s | 54 (47) | True | 5.9775 | 5.8588 | M2 2c; M3 0c | 0a6527d |
| fr_common_humpback |  | ends 03 Nov 11:19, 13 s | – (–) | – | – | – | – | 0a6527d |
| fr_common_humpback_loo |  | ends 03 Nov 11:20, 14 s | – (–) | – | – | – | – | 0a6527d |
| fr_rdo_humpback | RDO μ | ends 03 Nov 11:20, 252 s | 5 (None) | False | 5.1470 | – | 9c | 0a6527d |
| fr_common_ac1_direct |  | ends 03 Nov 11:20, 41 s | – (–) | – | – | – | – | 0a6527d |
| fr_common_ac1_completed |  | ends 03 Nov 11:21, 18 s | – (–) | – | – | – | – | 0a6527d |
| fr_common_mdo |  | ends 03 Nov 11:21, 32 s | – (–) | – | – | – | – | 0a6527d |
| fr_common_mdo_wind |  | ends 03 Nov 11:22, 38 s | – (–) | – | – | – | – | 0a6527d |
| fr_wind_env_mdo |  | ends 03 Nov 11:23, 64 s | – (–) | – | – | – | – | 0a6527d |
| fr_reanalysis_mdo |  | ends 03 Nov 11:23, 0 s | – (–) | – | – | – | – | 0a6527d |
| fr_rdo_second | RDO μ | ends 03 Nov 11:56, 2388 s | 43 (38) | True | 5.8764 | – | 11c | 0a6527d |
| fr_rdo_mdo_int_check |  | ends 03 Nov 11:56, 0 s | – (–) | – | – | – | – | 0a6527d |
| fr_rdo_mdo_int_74p24c | RDO μ | ends 03 Nov 12:13, 993 s | 20 (15) | False | 5.8739 | – | 1c | 0a6527d |
| fr_rdo_mdo_int_73p24c | RDO μ | ends 03 Nov 12:29, 1009 s | 20 (15) | False | 5.8726 | – | 2c | 0a6527d |
| fr_rdo_mdo_int_75p25c | RDO μ | ends 03 Nov 12:45, 947 s | 20 (15) | False | 5.8750 | – | 0c | 0a6527d |
| fr_bound_banner |  | ends 03 Nov 13:05, 109 s | 22 (16) | True | 5.9783 | 5.8839 | M2 0c; M3 1c | 0a6527d |
| fr_6S_19x12E_22x12E_kv270 |  | ends 03 Nov 13:08, 293 s | 57 (48) | True | 5.9168 | 5.7931 | M2 0c; M3 1c | 0a6527d |
| fr_rdo_mdo_int_75p25c_conv | RDO μ | ends 03 Nov 13:48, 2743 s | 59 (47) | True | 5.8751 | – | 1c | 0a6527d |
| fr_common_rdo |  | ends 03 Nov 13:49, 20 s | – (–) | – | – | – | – | 0a6527d |
| fr_wind_env_rdo |  | ends 03 Nov 13:50, 70 s | – (–) | – | – | – | – | 0a6527d |
| fr_reanalysis_rdo |  | ends 03 Nov 13:51, 0 s | – (–) | – | – | – | – | 0a6527d |
| fr_screen_mdo |  | ends 03 Nov 13:58, 0 s | – (–) | – | – | – | – | 0a6527d |
| fr_placeholders_mdo |  | ends 03 Nov 14:04, 0 s | – (–) | – | – | – | – | 0a6527d |
| fr_screen_rdo |  | ends 03 Nov 14:06, 0 s | – (–) | – | – | – | – | 0a6527d |
| fr_node_check |  | ends 03 Nov 14:09, 0 s | – (–) | – | – | – | – | 0a6527d |
| fr_wind_dir_humpback |  | ends 03 Nov 14:09, 8 s | – (–) | – | – | – | – | 0a6527d |
| fr_wind_dir_humpback_loo |  | ends 03 Nov 14:10, 8 s | – (–) | – | – | – | – | 0a6527d |
| fr_wind_dir_ac1_direct |  | ends 03 Nov 14:10, 22 s | – (–) | – | – | – | – | 0a6527d |
| fr_wind_dir_ac1_completed |  | ends 03 Nov 14:11, 10 s | – (–) | – | – | – | – | 0a6527d |
| fr_wind_dir_mdo |  | ends 03 Nov 14:11, 19 s | – (–) | – | – | – | – | 0a6527d |
| fr_wind_dir_mdo_wind |  | ends 03 Nov 14:12, 20 s | – (–) | – | – | – | – | 0a6527d |
| fr_wind_dir_rdo |  | ends 03 Nov 14:13, 12 s | – (–) | – | – | – | – | 0a6527d |
| fr_mc_in_mdo |  | ends 03 Nov 14:28, 915 s | – (–) | – | – | – | – | 0a6527d |
| fr_mc_in_rdo |  | ends 03 Nov 14:43, 905 s | – (–) | – | – | – | – | 0a6527d |
| fr_mc_in_mdo_wind |  | ends 03 Nov 14:59, 938 s | – (–) | – | – | – | – | 0a6527d |
| fr_mc_in_ac1_completed |  | ends 03 Nov 15:14, 893 s | – (–) | – | – | – | – | 0a6527d |
| fr_mc_in_ac1_direct |  | ends 03 Nov 15:29, 896 s | – (–) | – | – | – | – | 0a6527d |
| fr_mc_in_humpback |  | ends 03 Nov 15:44, 880 s | – (–) | – | – | – | – | 0a6527d |
| fr_mc_in_humpback_loo |  | ends 03 Nov 15:59, 873 s | – (–) | – | – | – | – | 0a6527d |
| fr_propagation |  | ends 03 Nov 16:00, 0 s | – (–) | – | – | – | – | 0a6527d |
| fr_mc_out_mdo |  | ends 03 Nov 16:16, 917 s | – (–) | – | – | – | – | 0a6527d |
| fr_mc_out_rdo |  | ends 03 Nov 16:31, 897 s | – (–) | – | – | – | – | 0a6527d |
| fr_mc_out_mdo_wind |  | ends 03 Nov 16:46, 944 s | – (–) | – | – | – | – | 0a6527d |
| fr_mc_out_humpback |  | ends 03 Nov 17:01, 874 s | – (–) | – | – | – | – | 0a6527d |

**Timing (single-threaded, `outputs/results_2026/perf/`):**

| measurement | mean | repeats |
|---|---:|---:|
| AirCrafter 1.0 sweep, shipped steps | 999 s | 3 |
| AirCrafter 2.0 multipoint, cold | 361 s | 3 |
| one analysis, warm from Humpback (M2 + M3) | 11.5 s | 3 |
| one analysis at the converged state | 0.465 s | 3 |
| one total-derivative evaluation | 3.44 s | 3 |
| one robust evaluation, 11 nodes x 2 points | 138 s | 3 |
| its total derivatives | 38 s | 3 |
| RDO to convergence, one run | 2.83e+03 s | 1 |
<!-- /run table -->

<!-- findings -->
## Findings
1. **Best propulsion case: 8S_19x12E_22x12E.** After payload rounding (MDO): F = 5.9775 capped, 5.9806 uncapped, 5.8588 floored, with M2 1.9998, M3 2.9777, 72 passengers, 24 cargo, a 275 in banner.
2. The enumeration's spread is 0.0607 in F over 10 converged cases (8S_19x12E_22x12E first, 6S_19x12E_22x12E_kv270 last); 1 did not converge.
3. **Against Humpback as built (5.2845 in the same model):** MDO gains +0.6930, through payload (42/14 to 72/24) and banner (197 to 275 in). The common evaluation is AirCrafter 2.0's own model, so it favours this; and the Ground Mission, which grows with the counts, isn't in F (Q53).
4. **Against AirCrafter 1.0's selection in the same model:** direct (a) 5.4883, completed (b) 5.6579, MDO 5.9775: +0.3196 over (b), the like-for-like comparison (the same model, optimised around AirCrafter 1.0's span, chord, counts and banner).
5. **What robustness costs.** Nominal F: -0.0057 from designing at the mean wind, -0.0357 from robustness itself. Monte Carlo mean: +0.0061 and -0.0007. Sigma: 0.1175 (MDO) to 0.0833 (RDO). P(any violation): 0.071 to 0.137 to 0.000. RDO's gain is in variance and feasibility, not the mean: 73 of 1024 MDO samples sit on the sub-stall balance, and without them MDO's mean is 5.8785 against RDO's 5.8737. On the floored score (completed laps, hard cap) the order reverses: RDO 5.9120, MDO 5.8588.
6. **Multistart (10 Sobol' starts):** 6 converge feasibly, to 1 distinct optimum/optima (F within 0.001): F 5.9775–5.9775 (6). The default start's F is 5.9775. The others: ms1: MDA non-convergence: Solver 'NL: Newton' on system 'points.M2.mda' fai; ms2: MDA non-convergence: Solver 'NL: Newton' on system 'points.M2.mda': re; ms5: MDA non-convergence: Solver 'NL: Newton' on system 'points.M2.mda': re; ms6: MDA non-convergence: Solver 'NL: Newton' on system 'points.M2.mda': re.
7. **The bound-driven part.** (i) c <= 0.60 m and camber_scale <= 2.5: ΔF +0.0198. (ii) V_h >= 0.50, V_v >= 0.04 (Raymer 6th ed., Table 6.4, p. 160, homebuilt): ΔF -0.0108. (iii) V_h >= 0.66 ([OH]), V_v >= 0.059 ([W]), the lowest flown: ΔF -0.0211. (added) banner <= 500 in, AirCrafter 1.0's grid limit ([HB] Table 7), against the 275 in the team built (Q56): ΔF +0.0008.
8. **Propagation at the robust optimum:** mean of F 5.8751 (UDR), 5.8740 (Smolyak), 5.8737 (Sobol'); sigma 0.0836, 0.0849, 0.0833.
9. **Analysis failures are counted apart from violations:** in-set Monte Carlo, MDA failures / lap-model limits out of N: Humpback 0 / 0 of 1024; MDO 0 / 0 of 1024; MDO, mean wind 0 / 0 of 1024; MDO + RDO 0 / 0 of 1024. A first run lost 17-30 % of its samples to a worker failure, not to the design (docs/development/RESULTS_LOG.md, Decision 13), and was replaced.
10. **Banner drag at fixed throttle:** across the 4 C_D,b values re-analysed, MDO loses level flight or a constraint at 3 and RDO at 0 (§5.4). MDO's limit on the placeholder banner bound leaves it no margin on M3's low-speed balance.
11. **Where this differs from 2024-25 (Appendix A).** There the multipoint gain over the as-built aircraft was small (Woombah 5.1020, MDO 5.0993) and the optimum was tank-limited on M2; here MDO is +0.6930 over Humpback, and M2 sits at the score's cap, so the cap and the fuselage's payload law set the payload. There robustness cost 0.0007 in nominal F and took the largest P(violation) from 0.621 to 0.016; here it costs 0.0413 and takes P(any) from 0.071 to 0.000.

## Recommendations (as on the page)
- **Choose the M3 prop first; it matters more than the M2 prop.** Across the 8S enumeration the mean F runs from 5.9430 to 5.9760 with the M3 prop (22x12E best), and only from 5.9557 to 5.9584 with the M2 prop. *Rests on §5.1. Confidence: medium; the ordering comes from the cited APC data, but the model flies M3 laps too fast (§7), so the size of the gain isn't a prediction.*
- **Carry more passengers than Humpback's 42, and test the fuselage it takes.** Every normalisation tried wants at least 53 passengers, so the direction doesn't rest on the PLACEHOLDER normalisers (Q52), though the MDO's own count does. *Rests on §5.1 and §5.2. Confidence: medium for the direction, low for the count; the Ground Mission, which grows with the counts, isn't modelled (Q53), and a fuselage this long is an extrapolation of the mass calibration (§7).*
- **Don't size the banner to its limit before its drag is known.** RDO gives up banner length (275 to 228 in) for margin: sigma of F 0.1175 to 0.0833, P(any violation) 0.071 to 0.000, at 0.0413 of nominal F. The re-analyses at [HB] Table 41's drag coefficients show why (§5.4). *Rests on §4 and §5.4. Confidence: medium; the Monte Carlo favours RDO by construction, so read it with §2.5's out-of-set result.*
- **Spend ground and flight tests on banner drag and built mass.** They are the largest contributions to the sigma of F with measured spreads: banner drag factor 0.0375 ([HB] Table 41), build factor 0.0292 (USYD leave-one-out). Wind contributes more, but its spread is a PLACEHOLDER and it can't be designed out by a test. *Rests on §4's screening. Confidence: medium.*
- **Don't commission a motor rewind for this score.** At the best prop pair, every converged start reaches the same F with Kv anywhere from 220 to 245 rpm/V: a flat direction, inside which the throttle takes up the difference. *Rests on §5.1's multistart. Confidence: medium; the motor's limits are checked, but its thermal behaviour isn't modelled.*
- **Use the rule limits on span and M3 energy.** The optima sit on the 5 ft span (b 1.524 m, Humpback 1.499 m) and the 100 Wh limit in M3, both from the rules [R26 p. 10, p. 16], not from a PLACEHOLDER. *Rests on §3's active-constraint table. Confidence: high that they're active in the model, medium on the gain, which isn't separated from the others.*
- **Log M3 lap times and throttle in test flights, with the banner's size.** The largest gap between this model and Humpback is M3's lap time, and these logs are what closes it. *Rests on validation_2026.md and §7's first row. Confidence: high that the gap exists, its cause isn't separated (Q56, Q57).*
<!-- /findings -->
