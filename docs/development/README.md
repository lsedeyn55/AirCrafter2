# Development notes

These notes were written while AirCrafter 2.0 was being built, one file per piece of work. They record what was
planned, what was checked against the sources and the code, the decisions taken and why, and what was measured. They
are kept for provenance: the code, configs and tests link to them for the reasoning behind a choice, and
`docs/DECISIONS.md` summarises the decisions that still apply.

For how the framework works and how to use it, start with the main `README.md`, `docs/modules/` and `docs/problems/`.
The placeholder inputs and open questions are in `docs/PLACEHOLDERS.md`.

| File | What it covers |
|---|---|
| `PLAN.md` | The build plan: sources, architecture, XDSM, module interfaces, the mass-model equation map, and the status and findings of each build step (aero, lap model, MDF, propulsion, stability, UI, twin motors, OpenVSP). |
| `RDO_PLAN.md` | Robust design optimisation: uncertain parameters, screening, quadrature rule, node convergence, failure handling, cost. |
| `KV_PLAN.md` | The motor speed constant K_v as a design variable. |
| `RDO_REDUCED_PLAN.md` | The reduced mass-and-wind uncertain set of the 2024-25 robust optimisation, and the build-mass factor `f_build`. |
| `TAKEOFF_COURSE_PLAN.md` | The take-off ground roll in the mission time, and the editable course. |
| `PLACEHOLDERS_RULESETS_PLAN.md` | The split of the inputs into `configs/placeholders.yaml` and one file per ruleset. |
| `MASS_CALIBRATION_PLAN.md` | Calibration of the mass model on three USYD aircraft. |
| `RESULTS_PLAN.md` | The 2024-25 results study. |
| `RESULTS_2026_PLAN.md` | The 2025-26 results study (the thesis's main results), and the fixes made before its final run. |
| `MDA_ROBUSTNESS_PLAN.md` | Failures of the multidisciplinary analysis at low-speed banner designs, and the fallback that fixes them. |
| `RESULTS_LOG.md` | Run log of the final 2025-26 results campaign. |

Commit hashes in these notes refer to the original development history. Before the repository was published, that
history was rewritten to remove AirCrafter 1.0 (the USYD DBF team's code), which changed every hash, so the hashes
quoted here do not resolve in this repository. Branch and tag names are unchanged.
