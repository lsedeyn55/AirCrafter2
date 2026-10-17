# Robustness (`aircrafter2/robustness/suite.py`)

- **Sensitivities:** normalised (x/f) df/dx from `compute_totals`, for design variables and assumptions.
- **Worst cases:** empty mass +X% (ballast), lap thrust −Y%, capacity −Z%, headwind. Each is a full MDA
  re-analysis.
- **Monte Carlo:** independent uniforms on the uncertain inputs, one full MDA per sample. Reports objective
  statistics and the probability of each constraint being violated.

Results for the full_mdo design are in `docs/problems/full_mdo.md`.
