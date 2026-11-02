# The constraints inventory (`aircrafter2/constraints_view.py`)

Everything but the design-variable bounds, with the limit, its units and the file and line that set it. It's what the UI's Constraints tab shows
(README, "The Constraints tab").

`constraint_table(cfg, ...)` gives rows in five sections:

1. **Optimiser constraints:** the driver's list, read from the set-up problem (deterministic or RDO, single-point or multipoint), in the driver's
   order, with the raw lower, upper, equals and ref.
2. **Checks after a run:** `check_fixed_limits`, `m1_check`, `oei_check`, never constraints.
3. **Analysis limits:** `AnalysisError` raise sites (the wind limit), outputs for information only, and the G&G sizing criteria. Not enforced by the optimiser.
4. **MDA balances:** L = W, T = D and the mass closure, solved by the MDA's own solvers.
5. **Unused limit fields:** limit fields of the schema that no row of sections 1–4 reads for this problem.

**The optimiser section is never a hand-kept copy.** The problem is built and set up (setup + final_setup, no run_model), and its driver's constraints are
joined to `REGISTER`. A driver constraint with no register entry raises `UnregisteredConstraint`, and so does a margin output of the model that is neither
constrained nor in the register. So a new constraint or check can't drop out of the tab silently.

**To add a constraint:** add it to the problem (`full_constraints`, `multipoint_constraints` or a scoring plug-in) and add its `Entry` to `REGISTER`.

**RDO problems** are set up with a one-node rule (udr, no active parameter). The driver's constraints don't depend on the rule (they are the `stats.*` moments and
the reference node n0), and the full rule takes 4–21 s to set up. `tests/test_constraints_view.py` checks that both give the same list.

**Source lines are part of the contract.** `REGISTER`'s `anchors=` and `hard_coded=` patterns find lines by regex, including some comments and docstrings, and
`tests/test_constraints_view.py` treats every line that matches its scan words as a site that must be registered. So keep anchored text on one line, and don't
reuse those words elsewhere.
