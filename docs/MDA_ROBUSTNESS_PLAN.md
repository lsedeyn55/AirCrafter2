# MDA robustness at low-speed banner designs: plan

Brief: `4_PROMPTS/aircrafter2_mda_robustness_prompt.md`. Branch `placeholder-fixes-2026` (`e3d40e8`, clean at the
start). **Nothing in the repo is changed:** no code, config or test. This file is the only new file under `docs/`, and
it is uncommitted. Diagnostics ran on a copy of the tree (`e3d40e8`, the same venv packages) in a scratch environment.
Prototype fixes were tried only in that copy. Scripts, logs and the prototype diffs are in
`outputs/mda_robustness/` (gitignored), listed in §6.

## Status (2026-11-02): F1 and F2 built and verified

Approved 2026-11-02 and built on `placeholder-fixes-2026` (brief `Claude outputs/aircrafter2_mda_robustness_impl_prompt.md`).
The text below this section is the plan as approved; this section records what was built and measured.

**Commits.** `94ed195` this plan; `5b868c1` F1 + F2 + tests; `4061c67` `integer()` rounding check; `15f1dcc` the
envelope's stall-margin column; `fe681dc` verification runs and the regenerated validation page; then the docs and the
M1-check commit (Problem 2, decision (b)).

**What was built.**
- `FullMDOConfig.mda_robust` (bool, default false), `true` in `configs/dbf2026_multipoint.yaml`. It is inherited by
  `dbf2026_multipoint_rdo.yaml` and by `humpback_2026.yaml` (the as-built validation config; kept on, decided with the
  user).
- F1, `aircrafter2/robustness/rdo_problem.py`: `RetryNewton(robust=...)`. Robust order: current → own last converged
  (skipped when equal to the start state) → continuation (4 steps, then 8; the last step is the target exactly) →
  reference → cold. `good_in` is stored in both modes. Switch off: the original loop, unchanged.
- F2, `studies/results_2026.py` `optimise`: with `start` or `x0` under the switch, a pre-run at the unfixed config
  start, then the target. The run JSON gets `warm_path` and `mda_fallbacks` (solves needing the continuation, per point).
- Tests: `tests/test_mda_robust.py` (12 tests) with `tests/data/mda_low_speed_designs.json` (iter2, iter3, pf_before).

**Deviations from the plan (each decided with the user on 2026-11-02).**
1. **`QuietDirectSolver`** (new, under the switch only). On this machine a diverged Newton attempt at the iter3 step
   reaches a singular Jacobian, at M2 as well as M3. OpenMDAO's `DirectSolver` then builds its diagnosis, a dense SVD of
   the MDA Jacobian, which took 575 s and > 3 GB and got a test run OOM-killed. It also raises `RuntimeError`, which
   `RetryNewton` doesn't catch, so no fallback ran. Under `mda_robust` the MDA's linear solver now raises
   `AnalysisError` at once. With it, iter3 converges in 24 s at 0.54 GB.
2. **Test 1's switch-off assertion is dropped.** Here that run ends in the slow `RuntimeError`, not `AnalysisError`
   (on the diagnosis machine it ended in NaN after 25 iterations).
3. **`integer()` rounding (§4.2 rule).** Verification 5 failed with the old rule. Diagnosis (per-evaluation logs,
   `outputs/mda_robustness/impl/`): both rounded cases (97p32c, 99p33c) were infeasible for every design, because
   `fuselage_margin` was violated from the first evaluation (−1.9 mm, −49 mm) with the span at its bound. SLSQP
   stalled, then failed: a wild step into the lap model's "non-physical lap", and a history-dependent singular M3
   Jacobian at evaluation 845, which doesn't reproduce in isolation. Now the candidates include both counts rounded
   down, and only those that fit at the continuous optimum (`rounding_check`) are re-optimised. `integer()` gained a
   `maxiter` passthrough.
4. **Validation envelope.** Two cells that failed before now converge (below). The table gained each point's stall
   margin, and marks cells inside 1.2 V_stall.

**Verification (this machine, 3 threads unless stated).**

| # | Check | Result |
|---|---|---|
| 1 | Full suite | Pass: 508 = 496 + 12 new. Two existing tests needed the new field: `NEW_FIELDS` in `test_placeholders.py`, and a `constraints_view` anchor for the new `raise`. The final count is from the rerun after commit 5. |
| 2 | `configs_snapshot.py` vs `tests/data/configs_reference.json` | Pass: 152 values, 0 differences (bit for bit). `rdo_off_snapshot.py` has no compare mode; `test_rdo` covers it in the suite. |
| 3 | Cold smoke `mr_smoke` | Pass: 58 iterations, 292 s, F **6.1723** (M2 2.2711, M3 2.9012), the prototype exactly. Nothing violated. M1 passes (SM 0.369, laps 9.13). `mda_fallbacks`: one continuation per point (the iter3 step). |
| 4 | Warm `--start mr_smoke` | Pass: 2 iterations, 11 s, ΔF 7e-15, no fallbacks |
| 5 | `integer --case mr_smoke` | Pass after deviation 3. Candidates 97p32c, 96p32c, 99p33c; only **96p32c** fits (fuselage margin +21.6 mm). It converged in 15 iterations (64 s): F **6.1675** (M2 2.2642, M3 2.9033), floored 6.0839, nothing violated, M1 passes (SM 0.369), no fallbacks. That is the result. |
| 6 | RDO setup check (`pf_rdo_setup_check_mr.json`) | Pass: 11/11 nodes at the first attempt (both points), 0 differences from the stored file apart from timing |
| 7 | Validation page | Humpback loo/gg/cal: identical apart from the git field, now clean (`fe681dc`'s page; this closes the dirty-provenance item). Envelope: 10 of 12 cells identical; 19x12E/22x12E Kv 270 throttle 0.5 converges (stall margin +3.0 / +0.8); 19x10E/19x12E Kv 270 throttle 0.5 converges inside 1.2 V_stall (stall margin −1.6 / −1.7, marked). Both through `continuation 4`. |

**Smoke optimum (`mr_smoke`).**
- Active constraints: M2 SM (lower), M2 and M3 V_v (lower), M2 and M3 energy (lower), pax ratio, fuselage length.
- Active bounds: b and c (upper), x_cb_frac and x_batt_M2 (lower).
- M3 cruise 16.07 m/s at throttle 0.768.

**Costs.**
- A failing optimiser step costs about 25 s (one failed attempt + 4 short solves).
- A `--start` run costs one extra `run_model` (about 6 s).
- `test_mda_robust.py` adds about 2.5 min to the suite.

**Problem 2: decided (b), 2026-11-02.** `SM_max` stays 0.47. For 2025-26 (`kind: dbf2026`), `m1_check` fails on SM
only below SM_min; above SM_max it reports `SM_high`.
- **Deviation:** limited to 2025-26 (`M1_SM_ONE_SIDED` in `aircrafter2/problems/multipoint.py`). The published
  2024-25 results have Woombah and the AirCrafter 1.0 direct design failing M1 on SM above its 2024-25 SM_max, so a
  global change would contradict a frozen page if it were rerun.
- **Humpback's M1 SM:** calibrated 0.4637, USYD-LOO 0.4718, G&G 0.822. The LOO and G&G checks now pass with
  `SM_high`; G&G's only other violation was SM.
- **Guard:** the smoke optimum's (and 96p32c's) M1 SM is 0.369, inside [0.25, 0.47].

**Open (not fixed).**
- The singular M3 Jacobian at evaluation 845 of the old 99p33c run depends on the run's history: it came at an
  infeasible stall, 845 evaluations in. A full replay would pin it down (about 20 min).
- The X-1 t/c (left, as planned).

## 0. Summary

- **Root cause.** At the low-speed M3 banner designs, the MDA's thrust balance T(v) = D(v) (with L = W) has **two
  roots**:
  - the real one, at 15.5 m/s;
  - a spurious back-side one, at 5.0–5.7 m/s. It is below the stall speed (CL ≈ 6), and exists only because the
    linear VLM lift never stalls.

  The reduced residual T − D peaks at 8–9 m/s, below stall. The Newton start is either the config's cold start
  (v 30 m/s, α 2°) or the previous iterate's state after SLSQP's first big step. Both lie far from the real root, so
  Newton's first step lands near the peak or past it. From there, α (which has no bounds) runs off to ±10³–10¹⁰°, v
  bounces between its 5 and 80 m/s clamps, and the residuals grow to 1e48. **The first non-finite values are the
  NeuralFoil strip outputs** (`cruise.strips.alpha_sec`, `Cd_sec`, …), at Newton iteration 24. By then α_cruise is
  about −3e10° and `m_total` about −3e19 kg. So the NaN is the overflow of a diverged iteration, not a domain error at
  a physical point.
- **The `--start` / cold `run_model` failure is cold-start only.** The stored optimum converges at the first attempt
  (5.7 s) when warm-started from Humpback's converged state. Fails: cold `run_model` at it.
- **The SLSQP failure (iteration 3) is a warm-start failure after a large design step.** Kv 245 → 200 and
  throttle_M3 1.0 → 0.795 cut v_fit_max from 43.4 to 28.2 m/s. The previous state (v 23.4 m/s, α 4.25°) is then
  outside the Newton basin of the new root. All three existing `RetryNewton` starts fail there: current, own last
  converged (the same vector), and cold.
- **Proposed fix (ranked 1 + 2).**
  - **F1:** a continuation fallback in `RetryNewton`. When every start fails, step the MDA's input vector from the last
    converged inputs to the target in N = 4 steps, each solve warm-started from the last.
  - **F2:** `optimise(start=…)` and `integer()` converge at the config's start first, then move to the target design.

  Both are behind a new switch that is off by default, so every 2024-25 and single-point run follows the old code path
  bit for bit. Prototyped: F1 converges the failing step. The full cold-start smoke run with F1 converged: **58 iterations, F 6.1723 (M2 2.2711, M3
  2.9012), no violated constraints, M1 check passed** (420 s on 1 thread here). It's a prototype run, not a result.
- **Problem 2** (M1 SM 0.4718 > 0.47) is your decision, §4.

## 1. Root cause (diagnostic runs, 2026-11-02)

### 1.1 The failing design point

The cold-start smoke run, re-run with every model evaluation's design variables logged
(`outputs/mda_robustness/diag/cold_smoke_evals.jsonl`), fails at SLSQP's third evaluation, the same point as in the
brief:

| | Kv | throttle_M2 | throttle_M3 | S_ht (m²) | S_vt (m²) | c (m) | b (m) | L_banner (in) | capacity_M3 (Ah) |
|---|---|---|---|---|---|---|---|---|---|
| iterations 0–2 (Humpback start) | 245 | 1.0 | 1.0 | 0.1484 | 0.0586 | 0.3701 | 1.4986 | 197.0 | 3.30 |
| **iteration 3 (fails)** | **200.0** | **0.7725** | **0.795** | **0.0961** | **0.0428** | **0.3398** | 1.4945 | 198.17 | 3.337 |

On this machine (2 cores, 1 thread) it stops with "failed to converge in 30 iterations" instead of your "inf or NaN
after 21 iterations". It is the same divergence: the warm-started reproduction below gives the NaN message.

### 1.2 What Newton does there

- **Warm, from the iteration-2 state** (v 23.4 m/s, α 4.25°), as SLSQP runs it. This gives the brief's message
  exactly ("residuals contain 'inf' or 'NaN' after 25 iterations").
  - Iteration 1: v → 8.79 m/s, α → 9.6°.
  - Then α swings −35°, −84°, +552°, …, −1981°, while v sits on its 5 m/s bound and jumps to 80 m/s.
  - The residual norm grows from 1 to 1e48.
  - At iteration 24 the NeuralFoil strip outputs overflow, and iteration 25 is NaN.
  - `m_total` stays at 4.783 kg until iteration 21, then diverges with the rest.
- **Cold, from the config's initial state** (v 30 m/s, α 2°): iteration 1 sends v to its 5 m/s bound (α 8.4°). It
  diverges the same way, and stops at 30 iterations at a residual of 4e24.
- **Your stored low-speed optimum** (`pf_before_smoke_…` dv, cold): the same pattern (v 5 m/s, α 128° at iteration 2).
  It fails on the current tree and, per the brief, on the pre-fix tree.

### 1.3 Why: two roots, and the starts are outside the real root's basin

The reduced residual f(v) = T(v) − D(v), with CL = W / (q S) at each v, from the converged polar (CD0, CD1, k) and the
T(v) fit of each state (`outputs/mda_robustness/diag/curve_*.csv`):

| State | Point | roots of f (m/s) | f max at (m/s) | V_stall (m/s) | v_fit_max (m/s) | α at the root | CL at the root |
|---|---|---|---|---|---|---|---|
| Humpback start (iterations 0–2) | M3 | **23.36** only | 8.25 | 11.62 | 43.41 | 4.25° | 0.27 |
| Iteration 3 (fails) | M3 | 5.03, **15.45** | 8.79 | 12.16 | 28.17 | 9.98° | 0.65 |
| Stored optimum (`pf_before_smoke`) | M3 | 5.71, **15.74** | 9.48 | 12.10 | 32.40 | 11.71° | 0.68 |
| Iteration 3 | M2 | 7.40, **24.79** | 13.2 | 14.91 | 27.37 | 6.03° | 0.40 |
| Stored optimum | M2 | 10.15, **31.30** | 17.41 | 16.65 | 35.85 | 5.72° | 0.34 |

(Model roots, from the converged MDA: M3 15.479 and 15.760 m/s; the curve's 2.5 cm grid agrees.)

- **At part throttle the thrust curve sinks onto the drag bucket.**
  - The banner makes CD0 about 0.25 (CD_banner 0.23 of it), so the real root sits close to the drag peak: at 15.5 m/s,
    df/dv = −3.4 N/(m/s), against −5.3 at Humpback's start.
  - A second root appears below stall. At full throttle (Humpback's start) there is only one.
- **The starts are far from the real root.**
  - Cold: 30 m/s at α 2°, against 15.5 m/s at α 10°.
  - Warm after SLSQP's first step: the previous root (23.4 m/s at α 4.25°) was found with v_fit_max 43.4. The new
    v_fit_max is 28.2, so 23.4 m/s is near the top of the new fit.
- **The coupled (α, v) Newton step overshoots past the drag peak.** The warm step lands at 8.79 m/s, where f peaks.
  There the reduced Jacobian is near singular, and every later step is large. Below stall, L = W needs CL ≈ 3–6. The
  linear VLM gives it at α of tens of degrees, with nothing to stop it: `alpha_cruise` has no bounds. The iteration
  never comes back.
- **What `energy_reserve` 0.0 does.** It doesn't change the MDA's equations: `E_mission` is downstream of the MDA. It
  changes SLSQP's path. With 25 % more mission energy (0.8 of the pack, against 0.64), the first step goes to Kv's lower
  bound and a 0.795 throttle in one move (my reading of the brief's step; not separately tested). With the reserve,
  the first step was smaller and stayed in the basin. So it is the trigger, not the cause, and it stays at 0.0 as
  settled.

### 1.4 The brief's suspects

| Suspect | Verdict |
|---|---|
| Cruise v driven below the stall speed | **Yes: the mechanism.** The first step goes below stall (8.8 m/s against V_stall 12.2), toward the spurious back-side root. |
| v past the T(v) fit range | **Secondary.** Later iterations hit v = 80 m/s (v_top), far past v_fit_max 28 m/s, where the polynomial is extrapolated. That keeps the iteration from recovering, but it doesn't start the divergence. |
| The banner drag at low v | **Contributes.** CD0 0.25 puts the real root near the drag peak (df/dv −3.4), which shrinks the basin. Not a numerical fault in the banner model. |
| A negative argument in the lap or power balance | **No.** The lap model is downstream of the MDA and is never evaluated in a failed solve. The only domain hazard found: parasite drag gives NaN at v ≤ 0 (seen once, in a broken prototype, §2.5). v's 5 m/s bound prevents it in the real model. |

### 1.5 Why "bounds on v" or a line search alone don't fix it (tested)

| Variant (cold `run_model` at the iteration-3 design, unless stated) | Result |
|---|---|
| As is | fails (30 iterations) |
| v start 0.6 v_fit_max (16.9 m/s), α start 2°, 3°, 4° or 6° | **fails**: M2 diverges (NaN at 16–17 iterations), its real root is at 0.91 v_fit_max |
| v start 0.6 v_fit_max, α start 10° (both points) | converges (M2 + M3, 10 s) |
| v start 0.6 v_fit_max with `RetryNewton`'s fallbacks (M2 then falls back to cold and converges) | converges (M3 in 4 iterations) |
| α bounded [−10°, 20°] (`BoundsEnforceLS`, scalar) | **fails.** Pinned at the (v 5 m/s, α 20°) corner for 10 iterations: the scalar step is cut to zero. It reaches the real root at iteration 28 (residual 2e-8), then runs out at 30 iterations. |
| `ArmijoGoldsteinLS` (both points) | **fails.** M2 now diverges. |
| Speed balance in s = v / v_fit_max ∈ [0.2, 1], start 0.6 | **fails.** The first step still goes to the lower bound; the prototype then hit NaN at v < 0 (my prototype's error, §2.5) |
| Warm: iteration-2 state → iteration-3 design (the optimiser's step) | **fails** (as SLSQP) |
| **Warm + continuation, 4 steps** (prototype F1) | **converges**: v 15.479 m/s, α 9.978°, F 5.17798 |
| Humpback's converged state → the stored optimum (one warm jump) | **converges at the first attempt** (5.7 s): F 5.981649, M3 v 15.760 m/s |
| `m1_check` at the stored optimum (after that jump) | passes, laps 8.873 (stored 8.874), 3.0 s |

- v is already bounded ([5 m/s, v_top]) and enforced by `BoundsEnforceLS`, and the spurious root sits at 5.0–5.7 m/s.
- A stall-speed bound would be the right physical cut, but V_stall depends on W and CL_max, which are MDA states, so
  `BoundsEnforceLS` can't hold it as a static bound.
- A good start for v helps only together with a good start for α, and a different one per point (M3 at about 0.5
  v_fit_max, M2 at about 0.9). That is a heuristic with its own failure region (§2.3).

## 2. Proposed fixes, ranked

A new problem-config switch, `mda_robust` (bool, default **false**), turns on F1 and F2 together. It goes on in
`configs/dbf2026_multipoint.yaml`, and the RDO config inherits it. It is a solver setting, not a ruleset value, so it
goes in the problem config. It is not a placeholder.

### 2.1 F1 (recommended): continuation fallback in `RetryNewton`

**What.** After the existing starts fail, interpolate the MDA's input vector from the last converged inputs
(`good_in`, stored next to `good`) to the target: t = 1/4, 1/2, 3/4, 1. Solve at each step, warm-started from the
previous step, and set the target inputs exactly at the last step. If a step fails, halve it once (8 steps) before
raising. The prototype has the 4 steps only; the halving and the exact last step are not prototyped yet. This is the brief's candidate 2 done inside the solver, so it covers every caller. That includes SLSQP's
path, the robust nodes, and the M2 point.

With the switch on, two ordering changes cut the cost of a failing solve (§2.1, cost):
- skip "own last converged" when it equals the current vector, which is the usual case in an optimiser step;
- try the continuation before "cold".

**Files.**
- `aircrafter2/robustness/rdo_problem.py`: `RetryNewton` gets `good_in`, the continuation and a `continuation` option
  (number of steps, 0 = off). The prototype is 25 lines (`outputs/mda_robustness/diag/prototype_continuation.diff`).
- `aircrafter2/problems/full_mdo.py`: the `mda_robust` field in `FullMDOConfig`, passed to `RetryNewton`. The Newton
  line stays as it is, so `constraints_view.py`'s hard-coded regex still matches.
- `aircrafter2/constraints_view.py`: the `balance.v` / `balance.alpha_cruise` entry notes name the fallback.
- `docs/modules/` (the MDA page) and `docs/RESULTS_2026_PLAN.md` (a line under "Placeholder fixes").

**Existing results.**
- **Switch off: nothing can move.** `RetryNewton`'s code path is the current one. The continuation is never reached
  and `good_in` is only stored. This covers every 2024-25 config (including `dbf2025_multipoint*.yaml`, which also use
  `fit_range: throttle`), the single-point configs, the 2024-25 RDO and IPOPT runs (whose back-off reacted to MDA
  failures), the AIAC commit and the published 2024-25 results.
- **Switch on (2025-26 only):** a solve that converged before follows the same attempts in the same order, so it gives
  the same result bit for bit. Only solves that raised before can change.

  One exception: the ordering change (skip the duplicate start, continuation before cold) can change which fallback
  succeeds, at a solve that failed its first attempt. In that case only 2025-26 results move, and none are final yet.
  If you prefer strictly fallback-only, keep the old order and append the continuation (cost: about 25 s more per
  failing step).

**How I'll show it.**
- Full suite, which includes `tests/test_results_2025.py` (the 2024-25 regression), `tests/configs_snapshot.py`
  against `tests/data/configs_reference.json`, and `tests/rdo_off_snapshot.py`.
- `validation_2026` regenerated and diffed: Humpback converges at the first attempt, so it should be identical.
- `pf_rdo_setup_check` re-run and diffed: all 11 nodes converged at the first attempt before.

**Tests** (`tests/test_results_2026.py`, or a new `tests/test_mda_robust.py`):
1. *The SLSQP step:* converge at Humpback's start, set the iteration-3 design (stored in
   `tests/data/mda_low_speed_designs.json`, since `outputs/` is gitignored), then `run_model`.
   - Fails with `mda_robust` off. This documents the need.
   - Converges with it on: M3 v 15.479 m/s within 1e-6.
   - The log shows one continuation entry.
2. *Fallback-only:* at Humpback's start, with the switch on, the log has only first-attempt entries, and the outputs
   equal the switch-off run's bit for bit.
3. *Default:* every config except the 2025-26 ones loads with `mda_robust` false. A loop over `configs/*.yaml`.

**Cost.** None for a solve that converges at the first attempt. A failing optimiser step costs one failed attempt (30
Newton iterations, 10–25 s here) plus 4 short solves (4–6 iterations each, about 5 s). Measured at the iteration-3
step: 57 s with the current order (three failed attempts first), against 52 s for the old failure. With the ordering
change, about 15–20 s. Smoke run with the prototype (current order): 58 iterations in 420 s on 1 thread. This isn't comparable with
your 300 s for `pf_before_smoke` (54 iterations, 3 threads); verification step 3 measures it on your machine.

### 2.2 F2 (recommended, with F1): warm path for `--start` and `integer()`

**What.** In `studies/results_2026.py` `optimise`, when `start` (or `x0`) is given:
1. `run_model` at the config's initial design (Humpback), which converges cold;
2. then set the target design variables;
3. the driver's first evaluation is then warm.

If the jump fails, F1's continuation takes it from Humpback's state. `integer()` goes through `optimise(start=case)`,
so it is covered. Measured: Humpback's state → the stored optimum converges at the first attempt (5.7 s), and so does
`m1_check` after it.

**Files.** `studies/results_2026.py` (`optimise`: about 6 lines, under `mda_robust`). There's no change to
`build_multipoint_problem`, so the robust problem and the UI are unaffected.

**Existing results.** None move: no stored 2025-26 run used `--start` successfully (the warm-start smoke failed at
iteration 0), and the 2024-25 study (`results_2025.py`) is a different script.

**Tests.**
4. `optimise(start=<the stored optimum>, maxiter=1)` reaches iteration 1 without an MDA error (about 1 min).
5. `integer()` on a stored continuous result with `maxiter=1`: both rounded cases evaluate (about 2 min). Uses the
   test-data design, not `outputs/`.

**Cost.** One extra `run_model` per `--start` run (about 6 s).

### 2.3 F3 (not recommended as the fix): a design-aware start for v and α

**What.** The cold start becomes v = s0 · v_fit_max per point (M2 about 0.9, M3 about 0.55), with α0 from CL = W/(qS).
**Why not first:**
- It changes the cold path itself. Every reference made from a cold start (the snapshot configs, Humpback's
  validation) could move in the last digits unless it is gated too.
- It needs per-point tuning: at 0.6 v_fit_max with α 2–6°, M2 fails.
- It does nothing for the warm-start failure along SLSQP's path, which is the one that stopped the smoke run.

Keep as a fallback if F1 turns out too slow in the RDO.

### 2.4 F4 (fallback only): the IPOPT path with fail flags for the deterministic problem

As in the brief: it changes the optimiser between the nominal and robust results, and the evidence above doesn't need
it. Not proposed.

### 2.5 Tried and rejected

- **α bounds with `BoundsEnforceLS`:**
  - Scalar enforcement pins the iteration at a corner (§1.5).
  - Vector enforcement was not tried.
  - Bounds would also change the path of solves that converge today, wherever α passes through the bound region.
- **`ArmijoGoldsteinLS`:** breaks M2.
- **Speed balance in s = v / v_fit_max:** the first step still goes to the bound. My prototype also routed v through an
  ExecComp that `BoundsEnforceLS` doesn't see (v reached −21 m/s → NaN in the parasite drag). Not pursued.

## 3. Verification list (after approval, in this order)

1. Full suite (about 14 min): 496 pass, plus the new tests. Restore
   `reference/aircrafter1/baseline/check_original.json` (`wall_s`) before committing.
2. `tests/configs_snapshot.py` → compare against `tests/data/configs_reference.json`: bit for bit (switch off).
3. The cold-start smoke: `studies/results_2026.py optimise --tag mr_smoke` (about 5 min). Converges; F, iterations and
   wall compared with the prototype's (58 iterations, F 6.1723, M3 v 16.07 m/s, SM_min active at M2), and with `pf_before_smoke` (F 5.9636, on the old values).
4. `--start mr_smoke --tag mr_smoke_warm` (about 5 min): converges at the same optimum (|ΔF| < 1e-4).
5. `integer --case mr_smoke` (2 × about 5 min): both rounded cases converge.
6. The RDO setup check (`pf_rdo_setup_check`): 11/11 nodes converge, and the values are identical to the stored
   file.
7. `validation_2026` regenerated: identical to `a2db0f4`'s page except the git field.
8. `git diff --stat`: only the files in §2.1–2.2, the new tests and test data, and the docs.

Long jobs run under `systemd-run --user`. With about 1.3 GB per worker, run 3 and 4 in parallel, then 5's two cases.

## 4. For you

### Problem 2: Humpback's M1 static margin above SM_max (decision needed)

The M1 post-check with the out-of-sample USYD factors: SM 0.4676 → **0.4718 > SM_max 0.47**. The cause is the sourced
prop: +12 g at x_motor −0.53 m. Calibrated: 0.4637, which passes. G&G: 0.82, which fails as before (uncalibrated).

- **(a) Recommended:** `stability.SM_max` 0.47 → **0.48** in `configs/rulesets/dbf_2025_26.yaml`. This applies the
  existing rule ("the largest modelled SM of the as-flown USYD aircraft, rounded up", Q9/Q62) with M1 included. The
  comment then names M1 (0.472, out of sample) alongside M2/M3. Optimisation can move only where SM_max is active, and
  it isn't active at the stored optimum (SM 0.184 / 0.246).
- **(b)** Check only SM's lower bound in M1: a code change in `m1_check`, since a too-large SM is a handling issue,
  not a safety one. It splits `M1_SAFETY`'s SM into a one-sided check.

Which one? Neither is applied until you say.

### Minor items (not fixed)

- **X-1 t/c:** `multipoint.stores.x1_wing_tc` 0.10 (2024-25 ruleset) is the same SD8020, so 0.101 would apply.
  Suggestion: leave it, because the 2024-25 references are frozen.
- **Dirty provenance:** `validation_2026` records `0f27643-dirty` for the gg/cal/envelope runs (only docs were
  uncommitted). A clean rerun takes about 20 min. Optional; verification step 7 regenerates it anyway, which makes it
  clean.
- **Stale worktrees.** `git worktree list` shows four entries. Run `git worktree prune` **on your machine, not from a
  sandboxed session**: in a session where the paths are mounted elsewhere, every worktree looks "prunable", so a prune
  there would drop the live `AirCrafter2-constraints-ui` worktree too. On your machine:
  - the `/tmp/claude-1000/…/wt_before` scratch worktree of `73a1db4` (last session) should be the only prunable one;
  - `outputs/wt_b2adbb8` is the `configs_snapshot` reference worktree of `b2adbb8`. It still exists; keep it or
    `git worktree remove` it.
- **Inefficiency in `RetryNewton` (any config):** "own last converged" repeats "current" in an optimiser step: the
  same vector, one more 30-iteration failure (10–25 s) per failing step. F1 removes it only under the switch.
- **Cold M2 is fragile too:** from 0.6 v_fit_max with α ≤ 6°, M2 diverges. No run hits it today, because the config's
  cold start (v 30 m/s) is near M2's root. F1 covers it.

## 5. Time estimates

| Step | Time |
|---|---|
| F1 + switch + docs | about 1.5 h |
| F2 | about 0.5 h |
| Tests 1–5 + test data | about 1 h (the tests themselves add about 4 min to the suite) |
| Verification 1–2 (suite, snapshot) | about 20 min |
| Verification 3–5 (smoke, `--start`, integer: 3 workers) | about 15 min |
| Verification 6–7 (RDO setup check, validation page) | about 25 min |
| **Total** | **about 4 h**, about 1 h of it runs |

## 6. Diagnostics (outputs/mda_robustness/, gitignored)

- `diag/cold_smoke_evals.jsonl`: every design evaluated by the failing cold smoke (iteration and design variables, the
  M3 v and α start and result).
- `diag/variants.py`: a cold `run_model` at a stored or logged design, with the start, bound and line-search variants
  of §1.5, and `trace` for the Newton history and the first non-finite output or residual.
- `diag/two_step.py`: converge at design A, then `run_model` at B (the optimiser's step, or `--start`).
- `diag/curve.py` + `curve_*.csv`: the reduced residual T(v) − D(v) of §1.3.
- `diag/m1.py`: `m1_check` after the warm jump.
- `diag/capture.py`: the smoke `optimise` with every evaluation logged. `proto_cont/` holds the prototype F1 smoke run
  (`diag_cold_smoke.json`, all 59 evaluations in `evals.jsonl`; its `git` field is empty because the copy had no `.git`).
- `diag/prototype_continuation.diff`, `diag/prototype_sv_alpha_bounds.diff`: the prototype patches. They are not
  applied to the repo, and the s / α one is the rejected variant. The scripts read the prototype switches as
  environment variables (`AC2_PROTO_CONT`, `AC2_PROTO_SV`, `AC2_PROTO_ALPHA`); they only take effect with a patch
  applied.
- `diag_src.tgz`, `diag_sitepkgs.tgz`: the copies used to set up the scratch environment (about 300 MB). Safe to
  delete.

To reproduce on your machine: `PYTHONPATH=. AIRCRAFTER2_APC_DIR=/home/lucas/University/THESIS/PERFILES2
.venv/bin/python outputs/mda_robustness/diag/variants.py outputs/mda_robustness/diag/cold_smoke_evals.jsonl:2 warm trace
noretry`. The JSONL paths inside the scripts are given on the command line.
