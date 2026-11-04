# Re-run of the 2024-25 study at the final code: log

Brief: `4_PROMPTS/aircrafter2_rerun_2025_overnight_prompt.md`. All times Sydney (AEDT). The user was away from 00:25; every decision below was taken without them and is for review.

## Setup

- 00:16 Inventory. Machine: 16 logical cores, 14 GB RAM, about 7-10 GB available, swap 14 GB, on AC power (battery 76 %), profile `performance`.
- 00:20 Worktree `AirCrafter2/outputs/wt_rerun_2025`, branch `rerun-2025-final`, from tag `results-2026-final` (`0a6527d`). `thesis-results-2026`, its outputs and the thesis repo were not modified.
- 00:25 All five configs of the brief load and set up a problem at the final code, unchanged: `dbf2025_multipoint.yaml`, `dbf2025_multipoint_takeoff.yaml`, `dbf2025_multipoint_rdo_calibrated.yaml`, `woombah_2025.yaml`, and `mass_usyd_loo_woombah.yaml` (applied by `validation_2025.py --mass loo`). **No config needed a change to validate.**
- 00:35 `pytest -k "2025 or stores or dbf2025"`: **76 passed, 481 deselected** (173.9 s), run on the patched study scripts before the freeze.
- 00:36 Smoke test of each changed path (warm start, Sobol' start, bottle, screen, RDO, batched Monte Carlo) in `outputs/results_2025_smoke/` (not results).
- 00:36 Run commit `576eaa8`, frozen in `outputs/results_2025_final/RUN_COMMIT`. `git diff results-2026-final 576eaa8 -- aircrafter2` is empty.
- 00:36 Campaign launched: `systemd-run --user --unit=rerun2025 systemd-inhibit --what=sleep:idle:handle-lid-switch ... studies/results_2025_campaign.py run`. Each job under `nice -n 10`, `OMP_NUM_THREADS=3`, `AIRCRAFTER2_APC_DIR=/home/lucas/University/THESIS/PERFILES2`.

## Decisions for review

1. **The uncertain set is `uncertainty_2025_reduced.yaml`, not `dbf2025_multipoint_rdo_calibrated.yaml`.** The brief names the calibrated file in setup step 1 and asks for "the calibrated reduced uncertain set". That file centres f_build on 2.74 (the Woombah-only factor, superseded 2026-10-23). The USYD factors in `placeholders.yaml` already carry the build bias, so using it would apply the correction twice. The brief's spreads (±30, ±30, ±15 %, wind 0-7 m/s) and the published page (f_build mean 1, f_tb 3.2737, m_avionics 0.222, wind 3.5) both match the reduced file, resolved at run time to exactly those means. `dbf2025_multipoint_rdo_final.yaml` uses it.
2. **The MDA continuation fallback is on** (`mda_robust: true`), and optimisations from a given start take the warm path, as `results_2026.py`. The brief's table lists the fallback as a difference between the two code versions but its config list does not mention it; turning it on removes the difference. It can change which starts converge, so the multistart counts are not purely a take-off effect.
3. **Wind along the course axis** (brief's choice, recorded): no `lap.wind_dir`, `course.turn_drift` false. Course preset `dbf_2025`.
4. **Woombah needs its own final config.** `woombah_2025.yaml` extends the take-off-off problem, so running the validation on it would silently leave take-off out. `configs/woombah_2025_final.yaml` adds the same take-off block and fallback.
5. **Woombah in the common evaluation uses the calibrated factors (in sample)**, as the published page; the out-of-sample LOO factors are for the validation only.
6. **Batch size 240** (brief: at most 250), so about 80 samples per worker with 3 workers. Each batch is checkpointed. `results_2026_mc_batched.py` is not at the tag (added after the freeze), so `results_2025.evaluate_batched` reimplements it.
7. **The propagation check reuses the robust optimum's in-set Monte Carlo** where rows are identical (all four parameters active), as `results_2026.propagation` does, instead of repeating 1024 analyses.
8. **Extra run, not in the brief: `continuity`.** The six published designs analysed unchanged at the final code, take-off off and on. One process, a few minutes. It separates the code change from the take-off in the diff.
9. **The venv's editable install points at the main checkout**, so every job sets `PYTHONPATH` to the worktree, and every run JSON records `model_path`.
10. **Screening runs right after the enumeration** instead of in stage 9, because the RDO reads its output. Its stage label stays 9.
11. **Campaign deadlines:** no new job after 05:55, running jobs stopped at 06:10, leaving time for the pages by 06:30. Timing (stage 10) starts only before 05:00 and runs alone.
12. **The resource limits cannot all hold at their stated sizes:** 3 jobs at 2.5 GB with 2 GB free needs 9.5 GB. The scheduler starts a job only if MemAvailable covers its share plus 2 GB (measured deterministic jobs ~1 GB).

## Run events

See `outputs/results_2025_final/campaign.log` for each job's start and end; failures and skips are copied below.

- 00:57 Enumeration complete, no failures. 02:10 stages 1-6 complete, no failures; first two Monte Carlos clean (0 analysis failures).
- 03:02 All six Monte Carlos complete: 0 analysis failures of 1024 each, none retried (`failure_check` in each `reduced_final/mc_*.json`: no failures to cluster).
- 03:57 Campaign finished: all 10 stages, 0 failures, 0 retries, 0 skips. Every run JSON records `576eaa8`, clean, and the worktree's model path.
- 03:58 Two extra runs at the run commit, not in the brief: `robust_eval` (the page's RDO finding reads it) and multistarts 0 and 7 with the fallback off (`configs/dbf2025_multipoint_final_nofallback.yaml`, `outputs/results_2025_final_nofallback/`), to attribute the change from 6 to 8 converged.
- 04:02 Extra runs finished. Multistart 0, fallback off: converges (55 evaluations, F 5.0737). Multistart 7, fallback off: fails at evaluation 5, MDA non-convergence, as published. So of the two starts newly converged, one is the take-off and start path and one is the fallback.
- 04:06 Pages generated: `docs/studies/results_2025_final/`. Same test selection rerun on the tree before the final commit: **79 passed, 481 deselected** (the 3 extra tests are parametrised over config files and include the configs added here; see below).

Earlier incidents, no effect on any result: the first smoke script lacked a `__main__` guard, so forkserver workers re-imported it and it crashed (00:32, before the freeze, script fixed). A `pkill -f` in my clean-up matched its own shell and ended that command; nothing else was running then.

## Further decisions for review

13. **The two multistart optima for RDO are multistarts 0 and 2**, the converged pair farthest apart in K_V among those that had finished (633 and 909 rpm/V). Both are in the lower group (F 5.0737). All three RDO starts nevertheless end at the same mu − sigma (spread 2e-11) on the upper group's airframe.
14. **"Deterministic optimum" stays the default start's design (F 5.0737)**, the method of the brief and the published page, even though four multistarts reach 5.0740. Stages 4-9 are built on it. See Open decisions.
15. **"Still true?" uses three grades** (yes, number changes, NO), because almost every number moves a little while most statements survive. The tolerance for "yes" is the quoted precision. "NO" is reserved for statements whose meaning fails.
16. **Timing used single-thread `optimise` and `rdo` runs of the study script with the method of `compute_performance.py`** (own process, OMP/OPENBLAS/MKL = 1, nothing else queued; the desktop stayed open), without its solver counters. Results: 260, 261, 262 s, and identical F to all printed digits; RDO 1613 s.
17. **The sleep inhibitor ended with each service.** Nothing was left running, and the power profile was not changed. The machine was not suspended.

## Open decisions for the user

A. **Which deterministic optimum to quote:** 5.0737 (default start, as computed here) or 5.0740 (best known). If 5.0740, then the common evaluation, Monte Carlo and bookends need rerunning from it; about 1.5 h. The difference is 0.0003.
B. **How to reword the robustness finding,** since wind no longer drives it (`thesis_number_diff.md`).
C. **Which Woombah lap error to quote:** lap 2+ (−27 %) or all three laps with the take-off (−22 %).
D. **Keep the fallback on in the 2024-25 study?** It makes the two rulesets alike but changes one multistart outcome. F values do not depend on it, as far as the two reruns here show.
E. **Whether to adopt the re-run at all,** given that it reverses a stated finding. Nothing was merged and the thesis was not edited.

## Stages

All complete: 1 validation (00:39), 2 enumeration (00:57), 3 multistart, 4 bottles, 5 mappings and common evaluation (01:44), 6 RDO (01:44), 7 Monte Carlo (03:02), 8 propagation (03:03), 9 bookends, normalisation, screening (03:16), 10 timing (03:57). Skipped: none. Failed: none. Retries: none. Config changes needed to validate: none.
