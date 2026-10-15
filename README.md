# AirCrafter 2.0

A lightweight MDAO framework (Python + OpenMDAO) for the conceptual design of the USYD AIAA Design/Build/Fly
aircraft: a small fixed-wing electric RC aircraft, optimised for competition mission performance. It is the
successor to AirCrafter 1.0 (MATLAB grid search, J. Kallarackel, 2025), and adds coupled aerodynamics, a
structure-based mass model, airfoil optimisation, propulsion, stability and gradient-based optimisation.

**Status:** all 8 build steps are implemented. Inputs marked `PLACEHOLDER` in the configs are not USYD data yet
(see `docs/OPEN_QUESTIONS.md`); replace them before trusting the numbers.

## Install

```bash
cd AirCrafter2
uv venv --python 3.14 .venv                                   # or use the existing .venv
uv pip install --python .venv/bin/python -e ".[test,ui,docs]"
```

The APC propeller data is read from `../PERFILES2/` (the same files AirCrafter 1.0 used).

## Quickstart

```bash
# optimise the full design (aero + mass + propulsion + stability + lap model)
.venv/bin/aircrafter2 run configs/step7_full.yaml --out outputs/my_design

# analyse only (no optimisation), or try another propeller
.venv/bin/aircrafter2 run configs/step7_full.yaml --analyse --prop 11x7E

# twin wing-mounted tractors, one pack per motor (capacity is per pack)
.venv/bin/aircrafter2 run configs/step7_full_twin.yaml
# or switch the layout / battery topology of any config from the command line
.venv/bin/aircrafter2 run configs/step7_full.yaml --n-motors 2 --battery-topology shared

# the graphical interface
.venv/bin/aircrafter2 ui

# available objectives (scoring plug-ins)
.venv/bin/aircrafter2 plugins
```

A run writes into its output folder:

| File | What |
|---|---|
| `summary.md`, `results.json` | optimum, constraints and margins, sensitivities |
| `figures/*.pdf` (+ `.png`) | planform, airfoil, spanwise lift, drag breakdown, mass breakdown, convergence, sensitivities |
| `cases.sql` | optimiser history (OpenMDAO `CaseReader`) |
| `n2.html` | the model's N2 diagram |
| `xdsm.tex` + `xdsm.tikz`, `xdsm.pdf` | XDSM (compile the `.tex` with LaTeX, e.g. Overleaf; the `.pdf` is a quick look) |

For thesis figures including the design-space surface: `.venv/bin/python examples/make_thesis_figures.py`.

From Python:

```python
from aircrafter2.api import run_design
res = run_design("configs/step7_full.yaml", optimise=True, out_dir="outputs/demo")
print(res.values["ThreeLapTime"], res.values["m_total"])
```

## Configs

Everything a run needs is in one YAML file (`configs/step7_full.yaml` is the full model). Configs can point at
shared files (`mass_config:`) and override them. Misspelt keys are errors, not silently ignored values.

| Section | Contents |
|---|---|
| `airfoil` | seed airfoil (AeroSandbox name or `.dat`) and shape modes |
| `aero` | VLM mesh, NeuralFoil settings |
| `mass` | Garcia & Gamboa options and inputs (materials, correction factors, payload, load factor) |
| `geometry`, `stability` | tail and fuselage placeholders, CG stations, stability bands |
| `propulsion` | APC prop, motor, battery, throttle |
| `scoring` | the objective plug-in (`three_lap_time`, `laps_in_window`, or your own) |
| `design_variables`, `initial` | bounds and start point |

## Documentation

- `docs/PLAN.md`: architecture, XDSM, module interfaces, every finding and decision.
- `docs/modules/`: one page per module (physics, equations, assumptions, sources).
- `docs/how_to_add_a_scoring_plugin.md`: adding a year-specific score.
- `docs/OPEN_QUESTIONS.md`: placeholders and open decisions.
- Verification reports:
  - `docs/mass_verification.md`: Garcia & Gamboa Table 15 reproduced within 2.3%.
  - `docs/aero_verification.md`: hand calculations and mesh study.
  - `docs/step6_lap_mdo.md`: port vs the original MATLAB, matching to ~1e-8.
  - `docs/step7_full_mdo.md`: full optimisation and robustness.

## Tests

```bash
.venv/bin/python -m pytest          # ~190 tests: check_partials, check_totals, verification, CLI, UI
```

## Layout

| Path | What |
|---|---|
| `aircrafter2/geometry` | rectangular planform, tails, cargo bay |
| `aircrafter2/airfoil` | Kulfan/CST shapes, section geometry, NeuralFoil section solver |
| `aircrafter2/aero` | OpenAeroStruct VLM + NeuralFoil strip drag, C_L,max, parasite drag, drag polar |
| `aircrafter2/mass` | Garcia & Gamboa (2022) structure-based and statistical mass, calibration |
| `aircrafter2/propulsion` | APC data, motor + ESC + battery powertrain |
| `aircrafter2/stability` | static margin from tail volume |
| `aircrafter2/mission` | AirCrafter lap model: MATLAB-faithful reference and smooth twin; oval |
| `aircrafter2/scoring` | objective plug-ins |
| `aircrafter2/problems` | the problem builders for each build step (`full_mdo.py` is the full model) |
| `aircrafter2/robustness` | sensitivities, worst cases, Monte Carlo |
| `aircrafter2/plotting`, `api.py`, `cli.py`, `ui/` | figures, API, CLI, Streamlit |
| `configs/`, `tests/`, `examples/`, `verification/matlab/` | configs, tests, report scripts, MATLAB reference |
| `examples/legacy/` | earlier experiments, unchanged |
