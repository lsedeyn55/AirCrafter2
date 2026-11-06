# AirCrafter 2.0

A lightweight MDAO framework (Python + OpenMDAO) for the conceptual design of the USYD AIAA Design/Build/Fly
aircraft: a small fixed-wing electric RC aircraft, optimised for competition mission performance. It is the
successor to AirCrafter 1.0 (MATLAB grid search, J. Kallarackel, 2025), and adds coupled aerodynamics, a
structure-based mass model, airfoil optimisation, propulsion, stability and gradient-based optimisation.

**Thesis:** this repository accompanies the undergraduate honours thesis *AirCrafter 2.0: A Gradient-Based
Multidisciplinary Design Optimisation Framework for the AIAA Design/Build/Fly Competition* (Lucas Sedeyn, School of
Aerospace, Mechanical and Mechatronic Engineering, The University of Sydney, 2026; supervisor Dr Bingran Wang). Its
results were run at the tag `results-2026-final`; the branch `thesis-results-2026` holds the 2025-26 result pages
and `rerun-2025-final` the 2024-25 re-run. The thesis's Reproducibility appendix lists every command.

**Not included:** AirCrafter 1.0, the USYD DBF team's MATLAB tool, is the team's code and is not in this public
repository. The steps that run it (the AirCrafter 1.0 sweeps, timings and `tests/test_aircrafter1.py`) need a copy
in `reference/aircrafter1/`, available from the team; everything else runs without it. Its outputs used by the
thesis are kept in `docs/studies/`.

**Status:** the full model and all its sub-problems are implemented. Inputs marked `PLACEHOLDER` in the configs are not USYD data yet
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
# optimise the full design (aero + mass + propulsion + stability + lap model); the motor Kv is a design variable
# (the config's motor is rewound to it: docs/modules/propulsion.md)
.venv/bin/aircrafter2 run configs/full_mdo.yaml --out outputs/my_design
# the same with a fixed motor (Kv = propulsion.Kv): the 10-DV problem of the pinned studies
.venv/bin/aircrafter2 run configs/full_mdo_fixed_kv.yaml

# analyse only (no optimisation), or try another propeller
.venv/bin/aircrafter2 run configs/full_mdo.yaml --analyse --prop 11x7E

# twin wing-mounted tractors, one pack per motor (capacity is per pack)
.venv/bin/aircrafter2 run configs/full_mdo_twin.yaml
# or switch the layout / battery topology of any config from the command line
.venv/bin/aircrafter2 run configs/full_mdo.yaml --n-motors 2 --battery-topology shared

# robust design optimisation: minimise mu + k sigma of the score with mu -/+ beta sigma constraints over
# fixed uncertainty nodes (~15x the deterministic cost; spreads in configs/uncertainty.yaml are PLACEHOLDERS)
.venv/bin/aircrafter2 run configs/full_mdo_rdo.yaml
.venv/bin/aircrafter2 run configs/full_mdo_rdo.yaml --no-robust   # same config, deterministic

# 3D OpenVSP mock-up of a run (optional OpenVSP, see below): .vsp3, STEP, STL, figures, consistency check
.venv/bin/aircrafter2 vsp outputs/my_design/
.venv/bin/aircrafter2 run configs/full_mdo_twin.yaml --vsp

# the graphical interface (Mission panel: take-off switch, course presets, leg table and sketch)
.venv/bin/aircrafter2 ui

# available objectives (scoring plug-ins)
.venv/bin/aircrafter2 plugins
```

A run writes into its output folder:

| File | What |
|---|---|
| `summary.md`, `results.json` | optimum, constraints and margins, sensitivities |
| `constraints.json` | the driver's constraints: bounds, refs and values (the UI's Constraints tab), and any switched off |
| `geometry.json` | full geometry of the result, for the OpenVSP mock-up (docs/modules/vsp.md) |
| `vsp/` (with `--vsp`) | `.vsp3`, `.stp`, `.stl`, three-view and isometric figures, `vsp_check.md` |
| `figures/*.pdf` (+ `.png`) | planform, airfoil, spanwise lift, drag breakdown, mass breakdown, convergence, sensitivities |
| `cases.sql` | optimiser history (OpenMDAO `CaseReader`) |
| `n2.html` | the model's N2 diagram |
| `xdsm.tex` + `xdsm.tikz`, `xdsm.pdf` | XDSM (compile the `.tex` with LaTeX, e.g. Overleaf; the `.pdf` is a quick look) |

For thesis figures including the design-space surface: `.venv/bin/python studies/make_thesis_figures.py`.

### 2024-25 results (AirCrafter 1.0 vs multipoint vs RDO)

The 2024-25 ruleset (`dbf2025` scoring, M2 and M3 flown by one airframe: `problems/multipoint.py`) with Woombah as
the validation case. Each script writes its page and JSON in `docs/studies/results/` (index: `INDEX.md`):

```bash
.venv/bin/python studies/validation_2025.py report         # Woombah: envelope, mass, flight performance
.venv/bin/python studies/results_2025.py <part>            # optimise, screen, rdo, mc, common, robust_eval, report, ...
.venv/bin/python studies/compute_performance.py run        # timing (clean committed tree, nothing else running)
.venv/bin/python studies/compute_performance.py refine     # AirCrafter 1.0 local grid refinement
.venv/bin/python studies/compute_performance.py report
.venv/bin/python studies/results_index.py                  # docs/studies/results/INDEX.md
```

AirCrafter 1.0 (MATLAB) runs from `reference/aircrafter1/` (`run_2025.m`, `time_2025.m`), which is not in the public repository (see "Not included" above).

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
shared files (`mass_config:`) and override them, or extend another config (`extends:`); `fixed_design_variables:
[Kv]` removes inherited design variables (the `*_fixed_kv.yaml` configs). Misspelt keys are errors, not silently
ignored values.

Design variables of the full model: span `b`, chord `c`, `tc_scale`, `camber_scale`, battery `capacity` (per pack),
tail arm `l_ht`, `S_ht`, `S_vt`, battery station `x_batt`, cargo-bay station `x_cb_frac`, and motor `Kv` (default;
PLACEHOLDER bounds 500-1500 rpm/V, OPEN_QUESTIONS Q34). Prop, cell count and number of motors are enumerated outside
the optimiser.

| Section | Contents |
|---|---|
| `airfoil` | seed airfoil (AeroSandbox name or `.dat`) and shape modes |
| `aero` | VLM mesh, NeuralFoil settings |
| `mass` | Garcia & Gamboa options and inputs (materials, correction factors, payload, load factor) |
| `geometry`, `stability` | tail and fuselage placeholders, CG stations, stability bands |
| `propulsion` | APC prop, motor, battery, throttle |
| `scoring` | the objective plug-in (`three_lap_time`, `laps_in_window`, `dbf2025`, or your own) |
| `design_variables`, `initial` | bounds and start point |

### Placeholders and rulesets

Every placeholder and assumption lives in one of two kinds of file, which every run loads underneath its config
(plan and conflict table: `docs/PLACEHOLDERS_RULESETS_PLAN.md`):

- `configs/placeholders.yaml`: one universal file with every value that does **not** depend on the competition
  rules: battery and motor data, nacelles, structure and materials, payload, fuselage, tail and CG, stability limits,
  n_crit, take-off (`k_lof`, `CL_ground`, the Eq. 3.3 distance) and the shared RDO spreads (f_build, f_tb, m_avionics).
- `configs/rulesets/<ruleset>.yaml`: one file per competition with everything that does: site altitude, mission
  time, course preset, pilot reaction time, start offset, the headwind spread, and for the multipoint problem the
  scoring (R2, R3, X-1), the stores (bottles, pylons, X-1 data) and the rule limits. `dbf_2024_25.yaml` is the
  2024-25 ruleset; `generic.yaml` holds the original AirCrafter inputs (`inputs_file.m`) for the single-point problems.

Both files are written one value per line, as a dotted path into the config with a comment
`# <units>; <PLACEHOLDER | cited>; Q..; <source>`. Load order, lowest to highest:

1. code defaults
2. `placeholders.yaml` (the `placeholders:` key; the default file when the key is missing)
3. the ruleset (the `ruleset:` key; opt-in; a single-point load ignores its `multipoint:` block)
4. the problem config and its `extends:` chain
5. the study variant (a config that extends the problem config)

`mass_config` sits below `placeholders.yaml`'s `mass.inputs`. A value set in a config overrides both layers: the
`*_fixed_kv.yaml` variants pin the AIAC-era motor values that way, and `woombah_2025.yaml` its as-built geometry.
Edit a value in either file and every config that uses it picks it up, `studies/results_2025.py` included.

In the UI (`aircrafter2 ui`), the main area has four tabs:
- **Design and mission**: design-variable bounds, key assumptions, the Mission column (take-off switch, `mu_roll`,
  reaction time) and the course editor.
- **Constraints**: every constraint, check and limit of the selected problem (below).
- **Placeholders**: one editable table for `placeholders.yaml` and a second one for the RDO spreads.
- **Ruleset**: a selector over `configs/rulesets/` and the same kind of table for the chosen file.

Each table has an **overridden by** column. It shows the file and line of the selected config that sets the value
instead, so an edit there won't reach that config's runs. **Save** writes the edits back into the file in place,
keeping every comment and the order. **Discard changes** drops them. Unsaved edits still apply to the current run, and
a note says so.

For a multipoint config, the UI shows the Constraints tab and the two layer tabs, with no Run button, because it runs
single-point problems. The multipoint runs are `studies/results_2025.py` and `studies/results_2026.py`.
To evaluate a 2025-26 design, use the warm path, because a cold run at a low-speed banner design can diverge
(docs/MDA_ROBUSTNESS_PLAN.md): `p = build_multipoint_problem(mcfg, driver=False, mda_retry=True)`, then
`run_warm(p, mcfg, dv)` (both in `aircrafter2.problems.multipoint`).

#### The Constraints tab

The tab lists everything but the design-variable bounds (those stay in Design and mission, and Kv in the sidebar; a
caption lists the rules that are enforced as bounds, such as span <= 5 ft). It reads the set-up problem the run
would build, not a copy (`aircrafter2/constraints_view.py`), in five sections:

1. **Optimiser constraints**: the driver's list (deterministic or RDO, single-point or multipoint), with the raw
   bounds, an effective limit in physical units (e.g. `rpm(v) <= 1 x 150000/19 in = 7,895 rpm`) and **set in**: the
   file and line that sets each limit's effective value (problem config, ruleset, placeholders, mass config, code
   default, or hard-coded file:line).
2. **Post-checks**: `check_fixed_limits`, `m1_check`, the OEI check.
3. **Analysis limits and report-only checks**: the wind limit (`AnalysisError`), report-only outputs, the G&G sizing
   criteria. Read-only, labelled not enforced.
4. **MDA balances**: L = W, T = D and the mass closure, with their solvers. Read-only.
5. **Limit fields this problem doesn't use**, greyed out.

Edit **lower** / **upper** where a config value is the bound (SM_min, V_h_max, wall_ratio_max, ...), and **limit**
where a margin's 0 bound stands for a physical limit (rpm_fraction, I_max, takeoff_distance, togw_max_lb, ...). These
values are edited only here; the layer editors leave their rows out. A value that is also a model input (`mu_roll`,
`energy_reserve`, `v_top`, `k_lof`, the G&G sizing inputs) is shown read-only, with the tab it is edited in. An
invalid limit (lower > upper, a fraction outside (0, 1], a non-positive physical limit) blocks Run.

For single-point runs, **on** switches a constraint off for this run (`energy_margin` is `require_3lap_energy`, the
others `disabled_constraints`); a safety constraint switched off gets a warning, and the run's summary lists it at the
top. With RDO on, **robust** chooses `robust.robust_constraints`. After a run the tab shows each constraint's value,
margin and state (active, inactive or violated), from the run's `constraints.json`.

**Save (constraints)** writes each changed value back to the file and line that sets it, keeping the comment (the
status, Q and source of a layer-file value are editable in the tab's comments table). Before writing a file that a
study script reads (found by parsing `studies/` and `verification/`), the tab lists those scripts and asks for a
confirmation: saving changes the inputs of their next runs. On/off and robust choices are not saved, except
`require_3lap_energy` and `robust_constraints`, which are config fields. **Discard changes** drops the edits; **Reset to
config values** also resets on/off and robust.

To add a constraint, add it to the problem (`full_constraints`, `multipoint_constraints` or a scoring plug-in's
`constraints()`) and give it an `Entry` in `constraints_view.REGISTER` (and its editable cells in `CELLS`). A driver
constraint with no entry, or a margin output that is neither constrained nor registered, is an error, so it cannot
drop out of the tab silently (`tests/test_constraints_view.py`).

## Documentation

- `docs/PLAN.md`: architecture, XDSM, module interfaces, every finding and decision.
- `docs/modules/`: one page per module (physics, equations, assumptions, sources).
  They include `docs/modules/problems.md`, `config.md`, `stores.md` and `constraints.md`.
- `docs/DECISIONS.md`: why the code is the way it is, one entry per decision, and the citation keys used in the code (Sources).
- `docs/how_to_add_a_scoring_plugin.md`: adding a year-specific score.
- `docs/OPEN_QUESTIONS.md`: placeholders and open decisions.
- `docs/problems/`: one report per problem in `aircrafter2/problems/` (`oval_lap.md`, `aero_mass.md`, `lap_mdo.md`,
  `full_mdo.md`), generated by the matching `examples/` script.
- `docs/verification/`: verification reports.
  - `docs/verification/mass.md`: Garcia & Gamboa Table 15 reproduced within 2.3%.
  - `docs/verification/aero.md`: hand calculations and mesh study.
  - `docs/verification/neuralfoil_fd_steps.md`: NeuralFoil finite-difference step study.
  - `docs/problems/lap_mdo.md`: port vs the original MATLAB, matching to ~1e-8.
  - `docs/problems/full_mdo.md`: full optimisation and robustness.
- `docs/studies/`: studies (`twin_motor_study.md`, `tail_boom_calibration.md`, `rdo_study.md`).
- `docs/studies/results/`: the 2024-25 results. `INDEX.md` lists every table, figure and JSON with its commit and
  config, and ranks the PLACEHOLDERS by their effect on the score; `validation_2025.md` (Woombah),
  `results_2025.md` (AirCrafter 1.0 vs multipoint vs RDO), `compute_performance.md` (timing and scaling).
- `docs/studies/results_2026/`: the 2025-26 results, the thesis's main results. `INDEX.md` lists every table, figure and JSON; `validation_2026.md` (Humpback), `results_2026.md` (Humpback, AirCrafter 1.0, MDO, MDO + RDO), `compute_performance_2026.md`.
- `docs/RESULTS_PLAN.md`: the 2024-25 plan (rules as read, scoring, stores, Woombah values, run list).
- `docs/RDO_REDUCED_PLAN.md`: the 2024-25 RDO with the reduced mass-and-wind uncertain set (`f_build`, evidence for the spreads).
- `docs/RDO_PLAN.md`: robust design optimisation plan (uncertain parameters, screening, node convergence, cost).

Most pages in `docs/problems/`, `docs/verification/` and `docs/studies/` are generated by the script of the same
topic in `examples/`, `verification/` or `studies/`; rerun the script rather than editing the page.

## Tests

```bash
.venv/bin/python -m pytest          # 550 tests (2026-11-04, about 17 min): check_partials, check_totals, verification, CLI, UI
```

## Layout

**Where to start reading.** Start with `aircrafter2/problems/full_mdo.py`: `build_full_model` is the whole model of one mission point in
order, with a section comment for each discipline, and each component it adds lives in the folders below. Then read
`aircrafter2/mission/smooth_lap.py`, the lap model, which is the example the rest of the code follows: each physics class says what it
computes, lists its options, inputs and outputs, and has a Symbols block linking its names to the equations. `docs/modules/` has the longer
explanations, one page per folder, and `docs/DECISIONS.md` says why the code is the way it is and what the citation keys mean. To run a
design rather than read one, start at `aircrafter2/api.py` (one mission point) or `aircrafter2/problems/multipoint.py` (`run_warm`, a whole
ruleset).

The `aircrafter2/` rows run in data-flow order, from the design variables to the score:

| Path | What |
|---|---|
| `aircrafter2/config` | YAML loading, placeholder and ruleset layers, schemas of the airfoil, aero and mass sections |
| `aircrafter2/airfoil` | Kulfan/CST shapes, section geometry, NeuralFoil section solver |
| `aircrafter2/geometry` | rectangular planform, tails, cargo bay, motor placement; `geometry.json` export |
| `aircrafter2/propulsion` | APC data, motor + ESC + battery powertrain, rewound motor |
| `aircrafter2/stores.py`, `aircrafter2/stores2026.py` | what a mission point carries: 2024-25 tanks, pylons and X-1; 2025-26 payload, banner and payload fuselage |
| `aircrafter2/aero` | OpenAeroStruct VLM + NeuralFoil strip drag, C_L,max, parasite drag, drag polar |
| `aircrafter2/mass` | Garcia & Gamboa (2022) structure-based and statistical mass, calibration |
| `aircrafter2/mission` | AirCrafter lap model: MATLAB-faithful reference and smooth twin; course and take-off; oval |
| `aircrafter2/stability` | static margin from tail volume, one-engine-inoperative check |
| `aircrafter2/scoring` | objective plug-ins (`three_lap_time`, `laps_in_window`, `dbf2025`, `dbf2026`) |
| `aircrafter2/problems` | problem builders, from aero-only up: `wing_aero`, `oval_lap`, `aero_mass`, `lap_mdo`, `full_mdo` (the full model), `multipoint` (2024-25 and 2025-26: M2 and M3 on one airframe) |
| `aircrafter2/robustness` | sensitivities, worst cases, Monte Carlo; robust design optimisation (`rdo_problem.py`, `rdo_multipoint.py`), `RetryNewton` |
| `aircrafter2/smooth.py`, `analytic.py`, `atmosphere.py`, `constants.py` | shared helpers: smooth min and max, analytic components, ISA |
| `aircrafter2/constraints_view.py` | inventory of a problem's constraints, checks and limits (the UI's Constraints tab) |
| `aircrafter2/api.py`, `cli.py`, `ui/`, `plotting/` | API, CLI, Streamlit, figures |
| `aircrafter2/vsp` | OpenVSP mock-up: build, render, consistency check |
| `aircrafter2/verification` | hand calculations for the aero checks |
| `configs/` | run configs (`full_mdo.yaml` is the full model), `placeholders.yaml`, `rulesets/`, shared mass inputs, `verification/` paper cases |
| `examples/` | user-facing demos, one per problem (`oval_lap`, `aero_mass`, `lap_mdo`, `full_mdo`) |
| `studies/` | thesis and paper scripts (thesis figures, twin-motor study, OpenVSP mock-ups, tail-boom calibration, 2024-25 validation, results, computational performance) |
| `reference/aircrafter1/` | AirCrafter 1.0 (not in the public repository): shipped copy, the switchable copy with its 2024-25 mode, baseline checks |
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

## Licence

MIT (see `LICENSE`). AirCrafter 1.0 and the APC propeller data are not covered and not included.
