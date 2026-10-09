# AirCrafter 2.0

A DBF-specific MDAO framework (Python + OpenMDAO) for the conceptual design of small fixed-wing electric RC aircraft.
It is the successor to AirCrafter 1.0 (MATLAB, J. Kallarackel, 2025). See `docs/PLAN.md` for the architecture.

**Status:** step 7 of 8. Mass model (step 2), aero-only wing with NeuralFoil profile drag (step 3), simple oval lap time (step 4), the coupled aero + mass MDF loop (step 5) the coupled AirCrafter lap modeller (step 6) and the full model with propulsion, stability, parasite drag and a robustness suite (step 7) work.

## Install

```bash
uv venv --python 3.14 .venv            # or use the existing .venv
uv pip install --python .venv/bin/python -e ".[test]"
```

## Quickstart: mass model

```python
from aircrafter2.config import MassConfig, load_yaml
from aircrafter2.mass.garcia_gamboa import build_mass_problem, mass_breakdown

cfg = MassConfig(**load_yaml("configs/mass_garcia_gamboa_2022.yaml"))
cfg.inputs.update(b=1.5, c_mean=0.25, c_root=0.25, S_wing=0.375, tc=0.12, t_at_spar=0.12,
                  l_ht=0.8, l_tb=0.7, S_ht=0.06, S_vt=0.03, S_cb=0.12, r_tb=0.008,
                  V_design=22.0, Cm=-0.08, CL_alpha=4.8, n=5.0, m_sys=1.2, m_pay=2.0)
prob = build_mass_problem(cfg)
prob.run_model()
print(mass_breakdown(prob))
```

All inputs, their units and descriptions are in `aircrafter2/mass/garcia_gamboa/inputs.py`.

## Tests and verification

```bash
.venv/bin/python -m pytest                     # unit tests, check_partials, check_totals, Table 15
.venv/bin/python examples/mass_verification.py  # regenerates docs/mass_verification.md
.venv/bin/python examples/aero_verification.py  # hand calcs, mesh study, step 3 optimisation -> docs/aero_verification.md
.venv/bin/python examples/neuralfoil_fd_steps.py  # FD step study -> docs/neuralfoil_fd_steps.md
.venv/bin/python examples/oval_lap.py            # step 4 oval lap optimisation -> docs/step4_oval.md
.venv/bin/python examples/aero_mass.py           # step 5 MDA grid, t/c trade, optimisation -> docs/step5_aero_mass.md
.venv/bin/python examples/tail_boom_calibration.py  # f_tb refit for the minimum boom wall -> docs/tail_boom_calibration.md
.venv/bin/python examples/lap_mdo.py             # step 6 twin check + coupled lap optimisation -> docs/step6_lap_mdo.md
matlab -batch "run('verification/matlab/run_reference.m')"   # MATLAB reference for the step 6 code check
.venv/bin/python examples/full_mdo.py [props]    # step 7: prop enumeration + robustness -> docs/step7_full_mdo.md
```

## Layout

| Path | What |
|---|---|
| `aircrafter2/` | the package: `geometry/`, `airfoil/`, `aero/`, `mass/`, `mission/`, `problems/` so far |
| `configs/` | YAML configs; `verification/` holds the paper's test aircraft |
| `tests/` | pytest suite |
| `examples/legacy/` | earlier OpenMDAO/OAS experiments, kept unchanged |
| `docs/` | plan, open questions, verification report |
