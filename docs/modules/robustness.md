# Robustness (`aircrafter2/robustness/`)

Two levels:

1. **Post-optimality checks** (`suite.py`), run on a converged design: sensitivities, one-at-a-time worst cases
   and a Monte Carlo. They report how robust a design is; they don't change it.
2. **Robust design optimisation (RDO)** (`uncertainty.py`, `quadrature.py`, `stats.py`, `rdo_problem.py`):
   statistics of the objective and probabilistic constraints inside the optimiser, so robustness shapes the design.
   It's optional: a config without an enabled `robust:` block is the deterministic problem, unchanged (regression
   test `tests/test_rdo.py::test_rdo_off_matches_pre_rdo_reference`).

Plan, screening and node-convergence check: `docs/development/RDO_PLAN.md`. Study results: `docs/studies/rdo_study.md`.

## 1. Post-optimality checks (`suite.py`)

- **Sensitivities:** normalised (x/f) df/dx from `compute_totals`, for design variables and assumptions.
- **Worst cases:** empty mass +X % (ballast), lap thrust −Y %, capacity −Z %, headwind. Each one is a full MDA
  re-analysis.
  Thrust is scaled in the lap model only: the MDA's level-speed balance keeps the nominal thrust, because that speed sets
  the stall margin and the structural design speed.
- **Monte Carlo:** independent inputs, one full MDA per sample. The inputs are either relative uniform spreads or
  the distributions of `configs/uncertainty.yaml`, the same source RDO uses. It reports the objective statistics
  and the probability of each constraint being violated. `sampling.evaluate` is the parallel version used by the
  study, with MDA failures recorded separately.

## 2. Robust design optimisation

### Uncertain parameters

An uncertain parameter is a model input p_i (a promoted name, or `lap.<name>` on the lap model) with an independent
distribution, x = mean + scale · ξ:

| Distribution | ξ | scale | σ |
|---|---|---|---|
| uniform | U(−1, 1) | half-width | scale/√3 |
| normal | N(0, 1) | σ | scale |
| triangular (symmetric) | density 1 − \|ξ\| on [−1, 1] | half-width | scale/√6 |

Design variables are never uncertain parameters. `robust.dv_tolerances` is a hook for manufacturing tolerances
(x + δ at the nodes), not implemented yet. **Every spread in `configs/uncertainty.yaml` is a PLACEHOLDER
(PLACEHOLDERS Q32).**

**The parameter pool** (docs/development/PLACEHOLDERS_RULESETS_PLAN.md) is merged by name in this order:
1. the shared spreads in `configs/placeholders.yaml` and the ruleset;
2. the config's `uncertainty_config` file;
3. `robust.parameters`.

An entry replaces the one of the same name in place, or is appended. A file marked `standalone: true` is the whole
pool by itself: `configs/uncertainty.yaml`, the published 15-parameter set of `rdo_study`,
`dbf2025_multipoint_rdo_full` and `examples/full_mdo.py`, which takes none of the shared spreads.

**Uncertain sets of the 2024-25 multipoint RDO.**
- `configs/dbf2025_multipoint_rdo.yaml` (main) uses the shared spreads: f_build, f_tb and m_avionics from
  `configs/placeholders.yaml` and lap.wind from `configs/rulesets/dbf_2024_25.yaml`
  (`uncertainty_2025_reduced.yaml` now holds only what differs, which is nothing). That makes 4 mass-and-wind
  parameters, all active (d = 4, UDR N = 9 nodes × 2 points, no screening selection):
  - `f_build` ±30 % (the build-mass factor, docs/modules/mass.md);
  - `f_tb` ±30 %;
  - `m_avionics` ±15 %;
  - `lap.wind` 0–7 m/s (Tucson and Wichita April means 3.7 and 5.6 m/s; capped at the lap model's validity).
  A calibrated variant (`dbf2025_multipoint_rdo_calibrated.yaml`, f_build centred on Woombah's 2.74) and a one-sided
  one (`..._onesided.yaml`, f_build on [1.0, 1.3]) are sensitivity runs.
  Every other parameter is held at its config value. Evidence for each spread: docs/development/RDO_REDUCED_PLAN.md §2 and
  PLACEHOLDERS Q39.
- `configs/dbf2025_multipoint_rdo_full.yaml` keeps the published 15-parameter set, screened to d = 7.

**Motor Kv a design variable (the default).** The design variables are promoted once and shared by every node, so a
node cannot set its own `Kv` (the writes would all land on the one shared input). The motor's uncertainty is then:
- `Kv_factor` (nominal 1, the same ±5 % as `Kv`): the effective Kv is `Kv · Kv_factor`, applied before the rewind
  scaling, so a winding tolerance moves Kv, Rm and I0 consistently (docs/modules/propulsion.md);
- `Rm_ref` (the same ±20 % as `Rm`): the reference motor's resistance.

`configs/uncertainty.yaml` lists both pairs with a `kv_mode` tag (`fixed`: Kv, Rm; `dv`: Kv_factor, Rm_ref) in the
same list positions, and `for_kv_mode` keeps the pair of the config's mode, so either mode has the same 15
parameters in the same order (the fixed-Kv list is exactly the old one). Explicit names are checked, not remapped:
a config with Kv a DV and `Kv` or `Rm` in `robust.active` fails validation (and `Kv_factor` / `Rm_ref` with a fixed
motor). `configs/full_mdo_rdo.yaml` uses Kv_factor / Rm_ref; `configs/full_mdo_rdo_fixed_kv.yaml` keeps Kv / Rm and is
the problem of docs/studies/rdo_study.md.

**Screening** (`uncertainty.screen`): at the reference point p̄ (all means), the first-order variance contributions
are V_qi = (∂q/∂p_i · σ_i)², from one `compute_totals` call. The active set is the smallest set (≤ `max_dim`) covering
`screening_threshold` of Σ_i V_qi for the objective and every active constraint with nonzero variance. The other
parameters are held at their means.

### Propagation: multi-point model at fixed nodes

N copies of the full analysis (`build_full_model`) sit in one `om.ParallelGroup` (serial without MPI) and share the
design variables. Node j has the parameters at p_j, and node 0 is always the reference point p̄. Fixed nodes (common
random numbers) make every statistic a deterministic, smooth function of x. The totals come through each node's
coupled derivatives (a DirectSolver on every MDA), exactly as in the deterministic problem.

Every rule is written as

    μ_q = Σ_j a_j q_j,      Var_q = qᵀ Q q,      Q = diag(D) − U diag(c) Uᵀ,

with q_j the value at node j. That makes the statistics partials analytic for every rule:
∂μ/∂q = a, ∂Var/∂q = 2 Q q.

- **Univariate dimension reduction** (`udr`, default; Rahman & Xu 2004):

      q(p) ≈ q̂(p) = Σ_{i=1}^{d} q(p̄₁, …, p_i, …, p̄_d) − (d − 1) q(p̄),

  with an n-point Gauss rule in each dimension. Gauss–Legendre is used for uniform inputs, Gauss–Hermite
  (probabilists') for normal ones, and the Gauss rule of the triangular density, computed by Stieltjes and
  Golub–Welsch, for triangular ones. With n = 3, the middle point of every symmetric rule is the mean, so
  **N = 2d + 1**. μ = E[q̂] and Var = Σ_i Var_i[q(p̄₁ … p_i … p̄_d)], the exact variance of the additive
  approximation, which is never negative. Rahman & Xu apply the decomposition to the raw moments E[qⁿ] instead;
  the two differ by cross products of the univariate deviations, and the raw-moment form can come out negative.
  Neither represents interaction (cross) terms between parameters.
- **Smolyak sparse grid** (`smolyak`, level L): the combination technique on Gauss rules with 1, 3, 5, … points.
  It is exact to total degree 2L + 1 and represents cross terms. Level 2 has 71 nodes for d = 5 and 127 for d = 7.
  Its weights can be negative, so a smooth positive part is taken (below).
- **Sobol'** (`sobol`): a fixed scrambled sample with equal weights and the unbiased variance. It's the Monte
  Carlo reference.

### Statistics component (`stats.RobustStats`)

    Var⁺ = Var                                  (positive semi-definite Q: udr, sobol)
         = ½ (Var + sqrt(Var² + ε⁴))            (otherwise)
    σ    = sqrt(Var⁺ + ε²)                      (finite derivative at zero variance)
    lo   = μ − β σ,      hi = μ + β σ,      robust objective = μ + s k σ   (s = +1 minimised, −1 maximised)

ε = `sigma_eps` × the constraint's `ref` (or the plug-in's `ref` for the score). Partials are analytic, and are
checked with complex step in `tests/test_rdo.py`.

### Robust problem (`rdo_problem.build_robust_problem`)

In the notation of the preliminary report §3.2, with nodes p_j and coupled states y_j:

    minimise    μ_f(x) + k σ_f(x)                  [or: minimise μ_f(x) subject to σ_f(x) ≤ ε  (Pareto sweep)]
    w.r.t.      x_L ≤ x ≤ x_U   (the deterministic design variables)
    subject to  R(x, y_j; p_j) = 0,                j = 0 … N−1   (one coupled MDA per node)
                μ_g(x) − β σ_g(x) ≥ g_lower,  μ_g(x) + β σ_g(x) ≤ g_upper,   g ∈ robust constraints
                g(x, y_0; p̄) within its bounds,    every other constraint (reference node)

- **f** is the scoring plug-in output. The default robust constraints are the energy margin, take-off thrust
  margin, stall margin, static margin (both sides), spar wall ratio and r_tube.
- **The (n_motors, prop, cells) choice** stays in the outer enumeration, as in the deterministic problem.
- **The normality assumption.** For a normally distributed g, μ − βσ ≥ lower means P(g < lower) ≤ Φ(−β): about
  2.3 % for β = 2. Nothing guarantees g is normal. The inputs here are independent uniforms, so a near-linear g is
  close to a sum of uniforms, with lighter tails than a normal. A strongly nonlinear g can have heavier ones. The
  study checks the actual P(violation) of each robust design with a Monte Carlo over all parameters.

### MDA failures at the nodes

Each node's Newton solver is a `RetryNewton`:
- It warm-starts from the node's last converged state.
- A node that has never converged starts from the reference node's state.
- A failed solve is retried from the node's last converged state, then from the reference node's state, then from
  the cold start. Every attempt is logged (`node_report`).

With the problem-config switch `mda_robust: true` (default false; on in `configs/dbf2026_multipoint.yaml`, so in its
RDO config and `humpback_2026.yaml`; docs/development/MDA_ROBUSTNESS_PLAN.md), `RetryNewton` runs in its robust mode:
- The order is current → own last converged (skipped when it equals the state the solve started from, the usual case
  in an optimiser step) → **continuation** → reference → cold.
- The continuation steps the MDA's input vector linearly from the last converged inputs (the reference node's, for a
  node that has never converged) to the target in 4 steps, each solve warm-started from the previous one, the last
  step at the target exactly; if a step fails, once more in 8 steps. The inputs include upstream outputs such as the
  T(v) fit coefficients, so the intermediate states are smooth but not physical: only the last step is the answer.
  Log labels: `continuation 4`, `continuation 8` (the log keeps its (solve, start, converged) tuples).
  It takes Newton across a design step that leaves the previous state outside the new root's basin.
- The MDA's linear solver is a `QuietDirectSolver`: a singular Jacobian (a diverged attempt) raises `AnalysisError` at
  once, instead of OpenMDAO's `RuntimeError` after a dense-SVD diagnosis (about 10 min and several GB on this MDA).
- Why: at the low-speed 2025-26 banner designs the T = D balance has a second root below stall; a Newton start far
  from the real root steps past the drag peak and diverges.

With the switch off (every 2024-25 and single-point config), the order and the code path are the original ones. A solve
that converges at its first attempt gives the same result bit for bit in both modes.

**The other analysis paths under `mda_robust` (2026-11-02, Final fixes D5).** Each of them used to do a cold
`run_model` at the design, which can diverge at a low-speed 2025-26 banner design.
- **`problems.multipoint.run_warm(prob, mcfg_start, dv)`** converges at `mcfg_start`'s unfixed start design (which
  converges cold), then sets `dv` and runs from that state. It works on the multipoint problem and on the robust one
  (both promote the design variables at the top). Use it to evaluate a 2025-26 design:
  `p = build_multipoint_problem(mcfg, driver=False, mda_retry=True); run_warm(p, mcfg, dv)`.
- **Its callers:** `results_2026.optimise` (the warm path for `--start` / `integer()`), `rounding_check`, the
  `rdo` subcommand and the sampler.
- **The sampler** (`sampling.evaluate`): for a `MultipointConfig` with `mda_robust` it builds with
  `mda_retry=True` and starts each worker through `run_warm`. With the switch off, the code path is the original
  one.
- **`m1_check`**: under `mda_robust` the M1 model uses RetryNewton's robust mode, and a `RuntimeError` counts as an
  analysis failure. Setup errors are raised first, not reported as non-convergence.

If all of those fail, the node raises `AnalysisError`:
- **SLSQP** (the default, `ScipyOptimizeDriver`) then stops, and the run is reported as MDA non-convergence, not
  as an infeasible design.
- **IPOPT** through pyOptSparse (`robust.optimizer: ipopt`) instead receives a fail flag and backs off in its line
  search. pyOptSparse's own SLSQP ignores the flag, so it isn't offered.
- No penalty values are used: they aren't differentiable and would mislead SLSQP's quasi-Newton model.

### Configuration

`configs/full_mdo_rdo.yaml` (single 10x7E 4S, Kv a DV), `configs/full_mdo_twin_rdo.yaml`, and their fixed-motor
versions `*_rdo_fixed_kv.yaml` (the RDO study) set these `robust:` keys:

| Key | Meaning |
|---|---|
| `enabled` | turns RDO on |
| `uncertainty_config` | the shared parameter list |
| `active` | the screened parameters |
| `quadrature` | `udr`, `smolyak` or `sobol` |
| `points` | Gauss points per dimension (udr) |
| `smolyak_level`, `sobol_n` | settings of the other two rules |
| `objective` | `mean_plus_k_sigma` or `mean_sigma_cap` |
| `k`, `sigma_cap` | the objective's weight, or its σ cap |
| `beta` | the constraint margin |
| `robust_constraints` | which constraints use the robust form |
| `optimizer` | `slsqp` or `ipopt` |
| `screening_threshold`, `max_dim` | the screening settings |

Run with `aircrafter2 run configs/full_mdo_rdo.yaml`. `--robust` and `--no-robust` override the config.

**Installing pyOptSparse and IPOPT** (optional, for `optimizer: ipopt`). pyOptSparse isn't on PyPI:

    uv pip install --python .venv/bin/python "pyoptsparse @ git+https://github.com/mdolab/pyoptsparse@v2.16.1"

IPOPT needs the Ipopt library, which isn't on PyPI either. `scripts/install_ipopt.sh` builds Ipopt 3.14.20 with
MUMPS 3.0.14 from coin-or into the virtual environment itself (`.venv/lib`, `.venv/include/coin-or`; no root, system
BLAS/LAPACK and gfortran, build tree in the git-ignored `third_party/ipopt_build`), then builds cyipopt 1.7.0 against
it with an rpath to `.venv/lib`. Deleting `.venv` removes it all. The rpath is absolute, so rerun the script after
moving the project folder.

    scripts/install_ipopt.sh

## References

- Rahman, S. & Xu, H. (2004). A univariate dimension-reduction method for multi-dimensional integration in
  stochastic mechanics. *Probabilistic Engineering Mechanics* 19(4), 393–408. doi:10.1016/j.probengmech.2004.04.003.
- Beyer, H.-G. & Sendhoff, B. (2007). Robust optimization – a comprehensive survey. *Computer Methods in Applied
  Mechanics and Engineering* 196(33–34), 3190–3218.
- Yao, W., Chen, X., Luo, W., van Tooren, M. & Guo, J. (2011). Review of uncertainty-based multidisciplinary design
  optimization methods for aerospace vehicles. *Progress in Aerospace Sciences* 47(6), 450–479.
- Smolyak sparse grids: Smolyak, S. A. (1963), *Soviet Mathematics Doklady* 4 [CITATION NEEDED: full title and
  pages not verified]. Golub–Welsch: Golub, G. H. & Welsch, J. H. (1969), Calculation of Gauss quadrature rules,
  *Mathematics of Computation* 23(106) [CITATION NEEDED: pages not verified].
