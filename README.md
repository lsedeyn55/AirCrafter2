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

The APC propeller data (`PER3_*.dat`, the same files AirCrafter 1.0 used) is not in this repo. By default it is
read from `../PERFILES2/`, i.e. `THESIS/PERFILES2` next to the repo. In a clone somewhere else, point at the folder
in one of two ways (the first that is set wins):

1. the config key `propulsion.apc_dir: /path/to/PERFILES2` (absolute, or relative to the working directory);
2. the environment variable `export AIRCRAFTER2_APC_DIR=/path/to/PERFILES2`.

## Quickstart

```bash
# optimise the full design (aero + mass + propulsion + stability + lap model)
.venv/bin/aircrafter2 run configs/full_mdo.yaml --out outputs/my_design

# analyse only (no optimisation), or try another propeller
.venv/bin/aircrafter2 run configs/full_mdo.yaml --analyse --prop 11x7E

# twin wing-mounted tractors, one pack per motor (capacity is per pack)
.venv/bin/aircrafter2 run configs/full_mdo_twin.yaml
# or switch the layout / battery topology of any config from the command line
.venv/bin/aircrafter2 run configs/full_mdo.yaml --n-motors 2 --battery-topology shared

# 3D OpenVSP mock-up of a run (optional OpenVSP, see below): .vsp3, STEP, STL, figures, consistency check
.venv/bin/aircrafter2 vsp outputs/my_design/
.venv/bin/aircrafter2 run configs/full_mdo_twin.yaml --vsp

# the graphical interface
.venv/bin/aircrafter2 ui

# available objectives (scoring plug-ins)
.venv/bin/aircrafter2 plugins
```

A run writes into its output folder:

| File | What |
|---|---|
| `summary.md`, `results.json` | optimum, constraints and margins, sensitivities |
| `geometry.json` | full geometry of the result, for the OpenVSP mock-up (docs/modules/vsp.md) |
| `vsp/` (with `--vsp`) | `.vsp3`, `.stp`, `.stl`, three-view and isometric figures, `vsp_check.md` |
| `figures/*.pdf` (+ `.png`) | planform, airfoil, spanwise lift, drag breakdown, mass breakdown, convergence, sensitivities |
| `cases.sql` | optimiser history (OpenMDAO `CaseReader`) |
| `n2.html` | the model's N2 diagram |
| `xdsm.tex` + `xdsm.tikz`, `xdsm.pdf` | XDSM (compile the `.tex` with LaTeX, e.g. Overleaf; the `.pdf` is a quick look) |

For thesis figures including the design-space surface: `.venv/bin/python studies/make_thesis_figures.py`.

### Where outputs go

Everything generated lands in `outputs/`, which is git-ignored:

| Path | What |
|---|---|
| `outputs/<config name>/` | `aircrafter2 run` without `--out` (with `--out DIR`, that folder instead) |
| `outputs/ui_runs/<YYYYMMDD_HHMMSS>/` | one folder per run of the graphical interface |
| `outputs/<study>/` | named studies written by `studies/` scripts (`thesis_figures`, `twin_study`, `twin_analyse`, `vsp`, ...) |
| `outputs/_openmdao/` | OpenMDAO's own per-problem scratch folders (`<problem name>_out/`, coloring files) |

OpenMDAO normally creates `<problem name>_out/` in the current directory. Importing `aircrafter2` sets
`OPENMDAO_WORKDIR` to `outputs/_openmdao/` if it is unset (or to a temp folder when the package is not a source
checkout), so set `OPENMDAO_WORKDIR` yourself to put them elsewhere.

From Python:

```python
from aircrafter2.api import run_design
res = run_design("configs/full_mdo.yaml", optimise=True, out_dir="outputs/demo")
print(res.values["ThreeLapTime"], res.values["m_total"])
```

## Configs

Everything a run needs is in one YAML file (`configs/full_mdo.yaml` is the full model). Configs can point at
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
- `docs/steps/`: one report per build step (`step4_oval.md` ... `step7_full_mdo.md`).
- `docs/verification/`: verification reports.
  - `docs/verification/mass.md`: Garcia & Gamboa Table 15 reproduced within 2.3%.
  - `docs/verification/aero.md`: hand calculations and mesh study.
  - `docs/verification/neuralfoil_fd_steps.md`: NeuralFoil finite-difference step study.
  - `docs/steps/step6_lap_mdo.md`: port vs the original MATLAB, matching to ~1e-8.
  - `docs/steps/step7_full_mdo.md`: full optimisation and robustness.
- `docs/studies/`: studies (`twin_motor_study.md`, `tail_boom_calibration.md`).

Most pages in `docs/steps/`, `docs/verification/` and `docs/studies/` are generated by the script of the same
topic in `examples/`, `verification/` or `studies/`; rerun the script rather than editing the page.

## Tests

```bash
.venv/bin/python -m pytest          # ~220 tests: check_partials, check_totals, verification, CLI, UI
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
| `aircrafter2/vsp` | OpenVSP mock-up: build, render, consistency check |
| `aircrafter2/plotting`, `api.py`, `cli.py`, `ui/` | figures, API, CLI, Streamlit |
| `configs/` | run configs (`full_mdo.yaml` is the full model), shared mass inputs, `verification/` paper cases |
| `examples/` | user-facing demos, one per build step (`oval_lap`, `aero_mass`, `lap_mdo`, `full_mdo`) |
| `studies/` | thesis and paper scripts (thesis figures, twin-motor study, OpenVSP mock-ups, tail-boom calibration) |
| `verification/` | verification harnesses (aero, mass, NeuralFoil steps), `matlab/` reference, `twin/` checks |
| `tests/` | pytest suite |
| `docs/` | see Documentation above |
| `outputs/` | generated results, git-ignored (see "Where outputs go") |
| `archive/legacy/` | the five pre-AirCrafter-2 prototype scripts, unchanged |

## OpenVSP (optional, installed locally)

OpenVSP 3.55.0 is not on PyPI, and openvsp.org publishes Linux builds only as Ubuntu `.deb` packages. It is
installed into the project without root:

- **Install location:** `third_party/openvsp/` (git-ignored, ~190 MB). This is the contents of
  `OpenVSP-3.55.0-Ubuntu-26.04_amd64.deb` from openvsp.org, which is built for Python 3.14 like this venv.
- **Bundled libraries:** `third_party/openvsp/lib/` holds the two libraries Arch lacks, taken from Ubuntu 26.04
  (resolute) universe and checked against its package index SHA-256:
  - `libcminpack1` 1.3.11-1 (`libcminpack.so.1`);
  - `libglew2.2` 2.2.0-4build2 (`libGLEW.so.2.2`; GUI only).
- **Library path:** the executables and the Python extension have a relative `RUNPATH` (set with
  `uvx patchelf`), so they find `lib/` without `LD_LIBRARY_PATH`.
- **Python API:** editable installs of `openvsp`, `openvsp_config`, `degen_geom` and `utilities` from
  `third_party/openvsp/python/`, with `import openvsp as vsp`. The optional AvlPy, CHARM, pyPMARC and PyVSP
  packages are not installed.
- **Executables:** `third_party/openvsp/{vsp,vspscript,vspaero}`.

To recreate it on another machine, repeat these steps. `pyproject.toml` does not list OpenVSP, because it can't
be installed from an index.

