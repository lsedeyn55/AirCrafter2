# Robust design optimisation (RDO)

Design notes for the robust optimisation (branch `rdo`): the uncertain parameters, the screening, the quadrature rule,
node convergence, failure handling, the cost and the robust problem statement. Scripts: `studies/rdo_screening.py`, `studies/rdo_node_check.py`; raw results in
`outputs/rdo/` (not in git).

**Note.** Every spread below is a PLACEHOLDER (PLACEHOLDERS Q32), as are the motor, battery and
payload values they perturb. The study measures how this model behaves under RDO and what RDO costs; the numbers
are not a design recommendation.

## 0. Decisions

| Item | Decision | Why |
|---|---|---|
| Screening dimension | **d = 7, N = 15**, not d ≤ 5 | No 5-parameter set reaches 90 % (best: 81 % of the lap time, 95 % of the energy margin, 77 % of the static margin; §2). 7 is the smallest set that does. Going back to the d = 5 set is one line in the config. |
| "Payload mass" spread | `m_pay` ± 10 % (uniform) | `worst_cases()` has no payload case: its 10 % mass step is ballast on the *empty* mass. I used the payload as specified with the same 10 %. |
| Capacity −10 % (a `worst_cases()` case) | Not an uncertain parameter | `capacity` is a design variable; the specification excludes DVs (the DV-tolerance hook covers this later). |
| Headwind | `lap.wind` ~ U(0, 3) m/s, mean 1.5 | 3 m/s is the worst-case default. The course is laid out into the wind, so the wind speed along the straights is ≥ 0. The reference node therefore flies in 1.5 m/s, not the deterministic 0. |
| AIAC twin-study tag | None exists in the repo (`git tag` is empty) | Nothing to avoid; no history is rewritten either way. |
| pyOptSparse and IPOPT | pyOptSparse v2.16.1 (from source), Ipopt 3.14.20 with MUMPS 3.0.14 built into the virtual environment by `scripts/install_ipopt.sh` (no sudo; the AUR package needs a password), cyipopt 1.7.0 linked to it by rpath. Checked on a constrained paraboloid. `robust.optimizer: ipopt` selects it; SLSQP stays the default | SNOPT needs a licence. pyOptSparse's SLSQP ignores the fail flag, so only IPOPT can back off from a failed analysis (§5). |
| Promoted inputs | **None changed** | Every uncertain quantity is already an OpenMDAO input (the `f_*` mass factors, the powertrain values, `eta_h`, `m_pay`, and `lap.wind` / `lap.thrust_factor` on the lap model). |
| `rho` | Not uncertain | It is in the sensitivity table of `examples/full_mdo.py` but not in its Monte Carlo spreads, which the specification says to start from. |

## 1. Uncertain parameters

One source: `configs/uncertainty.yaml`, used by the RDO configs (`robust.uncertainty_config`) and, from stage 2,
by the post-optimality Monte Carlo in `examples/full_mdo.py`. All independent.

| Parameter | OpenMDAO input? | Distribution | Nominal | Spread | Std | Source |
|---|---|---|---:|---|---:|---|
| `f_tb` (G&G tail-boom factor) | yes (mass) | uniform | 0.847 (config) | ± 30 % | 0.147 | PLACEHOLDER, examples/full_mdo.py |
| `f_cb` (cargo-bay areal mass) | yes | uniform | 0.7878 | ± 30 % | 0.136 | PLACEHOLDER, examples/full_mdo.py |
| `f_gear` | yes | uniform | 0.0089 | ± 30 % | 0.00154 | PLACEHOLDER, examples/full_mdo.py |
| `f_ht` | yes | uniform | 0.6411 | ± 30 % | 0.111 | PLACEHOLDER, examples/full_mdo.py |
| `f_vt` | yes | uniform | 0.3414 | ± 30 % | 0.0591 | PLACEHOLDER, examples/full_mdo.py |
| `e_spec` (Wh/kg) | yes (powertrain) | uniform | 150 | ± 10 % | 8.66 | PLACEHOLDER, examples/full_mdo.py |
| `R_cell` (Ω) | yes | uniform | 0.005 | ± 30 % | 0.000866 | PLACEHOLDER, examples/full_mdo.py |
| `Kv` (rpm/V) | yes | uniform | 900 | ± 5 % | 26.0 | PLACEHOLDER, examples/full_mdo.py |
| `Rm` (Ω) | yes | uniform | 0.05 | ± 20 % | 0.00577 | PLACEHOLDER, examples/full_mdo.py |
| `m_avionics` (kg) | yes | uniform | 0.25 | ± 15 % | 0.0217 | PLACEHOLDER, examples/full_mdo.py |
| `m_motor` (kg) | yes | uniform | 0.15 | ± 10 % | 0.00866 | PLACEHOLDER, examples/full_mdo.py |
| `eta_h` | yes (stability) | uniform | 0.9 | ± 10 % | 0.052 | PLACEHOLDER, examples/full_mdo.py |
| `lap.thrust_factor` | yes (lap model) | uniform | 1.0 | ± 0.10 | 0.0577 | PLACEHOLDER, `worst_cases()` thrust −10 % |
| `lap.wind` (m/s) | yes (lap model) | uniform | 1.5 | ± 1.5 (0 to 3) | 0.866 | PLACEHOLDER, `worst_cases()` headwind 3 m/s |
| `m_pay` (kg) | yes (mass, CG) | uniform | 1.5 | ± 10 % | 0.0866 | PLACEHOLDER, `worst_cases()` mass +10 % |

The thrust factor scales thrust in the lap model only; the MDA's level-speed balance keeps the nominal thrust,
exactly as in `worst_cases()`.

## 2. Screening at the best deterministic design

Design: deterministic optimum of `configs/full_mdo.yaml` with prop 10x7E, 4S. It reproduces the twin-motor study:
3-lap time 79.73 s, 39 SLSQP iterations, 86 s. Active constraints: energy margin (lower), SM (upper), V_h (lower),
r_tube (lower).

Method: every parameter at its mean, then (∂q/∂p_i · σ_i)² from one `compute_totals` call (1.7 s for 15 parameters,
13 quantities). V_h and r_tube are pure geometry (zero variance), so the screened quantities are the 3-lap time,
the energy margin and SM.

Share of first-order variance (%), sorted by the lap time:

| Parameter | 3-lap time | energy margin | SM | thrust margin | stall margin | wall ratio |
|---|---:|---:|---:|---:|---:|---:|
| Kv | **54.9** | **85.0** | 0.0 | 23.4 | 77.9 | 0.5 |
| lap.thrust_factor | **22.6** | **9.4** | 0.0 | 57.5 | 0.0 | 0.0 |
| Rm | **10.8** | **3.8** | 0.0 | 9.2 | 7.7 | 0.0 |
| lap.wind | 4.2 | 0.1 | 0.0 | 0.0 | 0.0 | 0.0 |
| R_cell | 3.9 | 1.4 | 0.0 | 3.3 | 2.8 | 0.0 |
| m_pay | **3.1** | 0.2 | **41.2** | 5.5 | 9.6 | 82.8 |
| f_cb | 0.4 | 0.0 | 5.0 | 0.7 | 1.2 | 10.0 |
| m_avionics | 0.2 | 0.0 | **14.3** | 0.3 | 0.6 | 5.2 |
| m_motor | 0.0 | 0.0 | **14.5** | 0.1 | 0.1 | 0.8 |
| eta_h | 0.0 | 0.0 | **21.2** | 0.0 | 0.0 | 0.0 |
| e_spec, f_tb, f_gear, f_ht, f_vt | < 0.1 | < 0.1 | 3.7 together | < 0.1 | < 0.1 | 0.6 together |
| **σ, first order** | 0.760 s | 2033 J | 0.0212 | 1.61 N | 1.07 m/s | 0.00475 |
| **σ, sum of 1-D 3-point Gauss variances** | 0.813 s | 2023 J | 0.0212 | 1.61 N | 1.07 m/s | 0.00475 |

Bold: the selected set. Coverage of the smallest covering set, by size:

| d | Best set | 3-lap time | energy | SM |
|---:|---|---:|---:|---:|
| 3 | Kv, eta_h, m_pay | 58 % | 85 % | 62 % |
| 4 | + Rm | 69 % | 89 % | 62 % |
| 5 | Kv, m_motor, eta_h, thrust_factor, m_pay | 81 % | 95 % | 77 % |
| 6 | f_cb, Kv, m_avionics, eta_h, thrust_factor, m_pay | 81 % | 95 % | 82 % |
| **7** | **Kv, Rm, m_avionics, m_motor, eta_h, thrust_factor, m_pay** | **92 %** | **98 %** | **91 %** |

**Why d ≤ 5 can't reach 90 %:** the lap time and the energy margin are set by the powertrain (Kv, thrust factor,
Rm), and the static margin by masses and stations (payload, tail efficiency, motor, avionics). The two groups
barely overlap, so covering both needs about four parameters each.

**Check of the first-order screen.** For each parameter alone, I also integrated the variance with 3 Gauss points
(the UDR's own per-dimension term, 2 analyses per parameter, 55 s for all 15). It sees curvature. The sums agree
with the first-order σ within 1 % for every constraint, and within 7 % for the lap time, where the gap is the wind.
The lap time is nearly even in the wind: 1000 ft of straights fly into it and 1000 ft with it. At a wind mean of 0
a first-order screen would score the wind at zero; at the 1.5 m/s mean its 1-D share is 3.7 % (4.2 % first order),
so leaving it out is consistent.

## 3. Rule: d, N and quadrature

- **Univariate dimension reduction** (Rahman & Xu 2004) over the 7 active parameters, 3-point Gauss–Legendre per
  dimension (all uniform): **N = 2d + 1 = 15** nodes, node 0 being the reference point (all means). The other
  8 parameters are held at their means.
- **μ = Σ_j a_j f_j, var = fᵀ Q f.** Q gives the exact variance of the additive approximation,
  Σ_i Var_i[f(p̄₁ … p_i … p̄_d)], which is never negative. Rahman & Xu apply the decomposition to the raw moments
  E[fⁿ] instead; the two differ by cross products of the univariate deviations, and theirs can come out negative.
  This difference goes in the docs.
- **Swappable rules** (`robust.quadrature`): `smolyak` (level 2: exact to total degree 5, represents cross terms;
  71 nodes for d = 5, 127 for d = 7) to check the cross terms, and `sobol` (fixed scrambled Sobol' set) for the
  Monte Carlo references.
- **Tests:** the Gauss rules reproduce the uniform, normal and triangular moments to degree 4; UDR is exact for
  additive polynomials; Smolyak recovers an a·b cross term that UDR misses.

## 4. Node convergence

`studies/rdo_node_check.py`: all 15 nodes at 27 points, single 10x7E 4S. The points are the config start point,
the deterministic optimum, and the 25-point box sweep of `verification/twin/mda_sweep.py` (centre, 16 random
corners, 8 random interior points, seed 0). Each node is run twice: **cold** (the config's initial guesses, as a
fresh analysis) and **warm** (from the nominal node's converged state, which is how the robust model starts
off-nominal nodes). 21 min with 5 workers.

| Points | Cold | Warm from nominal |
|---|---|---|
| start, det. optimum, centre, 8 random interior points, 9 corners | 15/15 everywhere | 15/15 |
| corner 7 (b 0.9, c 0.15, capacity 6) | 14/15 (m_pay+ fails) | **15/15** |
| corners 9, 10 (b 0.9, c 0.45, capacity 6) | 13/15 (Kv+ and m_pay− fail) | **15/15** |
| corners 0, 14 (b 1.52, c 0.45, capacity 0.5) | 1/15 | none: the nominal node fails |
| corners 6, 8, 13 (b 1.52, c 0.45, capacity 6) | 0/15 | none: the nominal node fails |

- **Hard failures are only at the largest-wing corner** (b = 1.52 m, c = 0.45 m). The deterministic MDA fails there
  too: it is the existing cold-start limit (PLAN §6h, stage 2), not something RDO adds. The original sweep,
  rerun with 11x7E, still gives 21/25.
- **Warm-starting from the nominal node rescues every off-nominal failure.**
- The deterministic optimum (c at its 0.15 m lower bound) is far from the failing corner, and the deterministic
  optimiser has never gone there.
- A bug in the first version of this check is worth recording. Restoring the solver states also restored the
  auto-IVC outputs, i.e. the inputs, so every "cold" run was actually at the start point and reported 27/27. Rerunning
  the known 11x7E failures exposed it; the fixed check reproduces them.

MDA non-convergence and constraint violation are kept separate in every report: a node either fails to converge
(an `AnalysisError`), or converges and then has constraint values.

## 5. What happens when a node fails mid-optimisation

**Prevention (implemented, `RetryNewton` in `rdo_problem.py`):**
1. Each node warm-starts from its own last converged state; OpenMDAO keeps the states between iterations.
2. A node that has never converged starts from the reference node's converged state.
3. On a failure, retry from the node's last converged state, then from the reference node's current state, then
   from the cold initial state. Every attempt is logged per node (`node_report`).

**If every attempt fails:** the node raises `AnalysisError`.
- With **SLSQP** (ScipyOptimizeDriver) the run stops. The driver catches the error, hands SLSQP 0 and re-raises
  when SLSQP returns. The study records it as **MDA non-convergence** with the node and design point, never as an
  infeasible design.
- **Proposed upgrade: IPOPT through pyOptSparse.** OpenMDAO's pyOptSparseDriver turns `AnalysisError` into
  `fail = 1`, and IPOPT backs off in its line search. pyOptSparse and IPOPT are installed (`robust.optimizer: ipopt`). pyOptSparse's
  SLSQP is **not** an option here: it ignores the fail flag and would carry on with the values the failed analysis
  left behind.
- **Not done:** no penalty value (it isn't differentiable and would distort SLSQP's quasi-Newton model), and no
  silent tightening of the bounds. If failures show up near the large-wing corner, tightening the c or b bound is
  an open decision; the recommendation is to report it rather than do it.

## 6. Cost estimate

Measured on this machine (16 cores, 14 GB):

| Item | Deterministic | Robust, N = 15 | Ratio |
|---|---:|---:|---:|
| analysis, warm start (s)¹ | 3.39 | 50.7 | 15.0 |
| total derivatives, all responses (s)¹ | 1.93 | 30.9 | 16.0 |
| memory, one problem (GB) | ~0.4 | 1.42 | |
| optimisation, single 10x7E 4S | 86 s, 39 iterations | **~22 min if the iteration count is similar** (N × deterministic) | ~15 |

¹ Measured with the node check's 5 workers loading the CPU; the ratio is what carries over. Clean timings follow in
stage 3.

- **Monte Carlo reference:** 1000 analyses per design at ~1–3 s each, so 5 worker processes take ~5–10 min. 14 workers
  ran the machine out of memory, so studies run at most ~5 model copies at once.
- **Whole study** (stages 2–4): about 15 robust optimisations (energy reserve ×2, Pareto ×7, enumeration ×3,
  repeats), at ~22 min each, run 3–4 at a time. Plus about 10 Monte Carlo sets of 1000. About 3–4 h of wall time.

## 7. Robust problem statement

In the notation of the preliminary report §3.2 (minimise −S_DBF(x, y) s.t. R(x, y) = 0, g(x, y) ≤ 0,
h(x, y) = 0, x_L ≤ x ≤ x_U), with the uncertain parameters p (active part p_a ∈ R^d, the rest at their means) and
the rule's nodes p_j, weights a and matrix Q (j = 0 … N−1, p_0 = p̄):

```
minimise    F(x) = μ_f(x) + k σ_f(x)                 [or: minimise μ_f(x) subject to σ_f(x) ≤ ε]
w.r.t.      x = [b, c, tc_scale, camber_scale, capacity, l_ht, S_ht, S_vt, x_batt, x_cb_frac]ᵀ,   x_L ≤ x ≤ x_U
subject to  R(x, y_j; p_j) = 0,                      j = 0 … N−1      (one coupled MDA per node)
            μ_{g_i}(x) + β σ_{g_i}(x) ≤ 0,           i ∈ I_R          (robust constraints)
            g_i(x, y_0; p̄) ≤ 0,                      i ∉ I_R          (deterministic, at the reference node)

with        f_j = f(x, y_j; p_j),   μ_f = Σ_j a_j f_j,   σ_f = sqrt(fᵀ Q f + ε_f²)       (same for each g_i)
```

- **f:** the scoring plug-in output (placeholder: 3-lap time, minimised; for a maximised score, −S). k = 1 and
  β = 2 by default, both from the config.
- **The equality constraints h** of §3.2 (L = W, T = D) are balance residuals inside each node's R.
- **The repo writes constraints as g ≥ lower** (and ≤ upper), so the code imposes μ − βσ ≥ lower and μ + βσ ≤ upper.
  The two-sided bands (SM, V_h, V_v) get both sides.
- **I_R** = {energy margin, take-off thrust margin, stall margin, SM, wall ratio, r_tube} by default
  (`robust.robust_constraints`). r_tube is geometry-only, so σ = ε and its robust form equals the deterministic one.
- **β = 2:** about 97.7 % one-sided reliability *if* g is normally distributed. That is an assumption: with uniform
  inputs and a near-linear g, g is closer to a sum of uniforms. The Monte Carlo P(violation) in stages 2–3
  checks it.
- **Hook, not implemented:** DV manufacturing tolerances (`robust.dv_tolerances`), with x_j = x + δ_j at the nodes.
- **The discrete choices** (n_motors, prop, cells) stay in the outer enumeration; the robust problem runs inside
  each case.

Relative to §3.2's x: there, m and V_cruise were design variables. In the built framework they are coupling
outputs (the G&G mass closure and T = D), and the airfoil is parametrised by tc_scale and camber_scale.

## 8. Implementation status

| File | Content | Status |
|---|---|---|
| `aircrafter2/robustness/uncertainty.py` | `UncertainParam`, `RobustConfig` (the `robust:` block, off by default), `Dist`, screening | done |
| `aircrafter2/robustness/quadrature.py` | Gauss rules, `udr`, `smolyak`, `sobol`; `Rule` with μ = a·f, var = fᵀQf (Q factored) | done |
| `aircrafter2/robustness/stats.py` | `RobustStats`: μ, σ, μ + kσ, μ ∓ βσ with analytic partials | done |
| `aircrafter2/robustness/rdo_problem.py` | `build_robust_problem`, `RetryNewton`, node and summary reports | done; optimiser runs in stage 3 |
| `aircrafter2/problems/full_mdo.py` | `robust:` field; `full_constraints(cfg)` (same list and order as before); `mda_retry` option (off) | done |
| `configs/uncertainty.yaml`, `full_mdo_rdo.yaml`, `full_mdo_twin_rdo.yaml` | spreads and RDO configs | done |
| `tests/test_rdo.py` | quadrature vs analytic moments, stats `check_partials`, robust `check_totals` (d = 1), RDO-off regression vs a snapshot from the pre-RDO commit (both configs, rtol 1e-9, incl. 3 SLSQP iterations) | 13 tests pass |
| CLI / API / UI, `examples/full_mdo.py` reading the shared spreads, docs | | stages 2–4 |

## References

- Rahman, S. & Xu, H. (2004). A univariate dimension-reduction method for multi-dimensional integration in
  stochastic mechanics. *Probabilistic Engineering Mechanics* 19(4), 393–408. doi:10.1016/j.probengmech.2004.04.003.
- Beyer, H.-G. & Sendhoff, B. (2007). Robust optimization – a comprehensive survey. *Computer Methods in Applied
  Mechanics and Engineering* 196(33–34), 3190–3218.
- Yao, W., Chen, X., Luo, W., van Tooren, M. & Guo, J. (2011). Review of uncertainty-based multidisciplinary design
  optimization methods for aerospace vehicles. *Progress in Aerospace Sciences* 47(6), 450–479.
