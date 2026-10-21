"""docs/studies/results/INDEX.md: every table, figure and JSON of the 2024-25 results, with the commit and config that
produced it, and the PLACEHOLDERS affecting the results ranked by sensitivity (from the results JSON).

    .venv/bin/python studies/results_index.py
"""

import json
import math
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DOCS = ROOT / "docs" / "studies" / "results"
OUT = ROOT / "outputs" / "results"

PAGES = [  # page, JSON, script, configs
    ("validation_2025.md", "validation_2025.json", "studies/validation_2025.py report",
     "configs/woombah_2025.yaml (extends dbf2025_multipoint.yaml), configs/mass_usyd_woombah.yaml"),
    (None, "aircrafter1_2025.json", "reference/aircrafter1/run_2025.m + studies/aircrafter1_2025.py",
     "reference/aircrafter1/ac1/inputs_2025.m, assumptions_2025.m; configs/dbf2025_multipoint.yaml"),
    ("results_2025.md", "results_2025.json", "studies/results_2025.py report",
     "configs/dbf2025_multipoint.yaml, configs/dbf2025_multipoint_rdo.yaml (+ _full, _calibrated, _onesided), "
     "configs/woombah_2025.yaml, configs/uncertainty_2025_reduced.yaml, "
     "configs/uncertainty.yaml"),
    ("compute_performance.md", "compute_performance.json", "studies/compute_performance.py report",
     "configs/dbf2025_multipoint.yaml, configs/dbf2025_multipoint_rdo.yaml; reference/aircrafter1/time_2025.m"),
]
FIGURES = {"woombah_": "validation_2025.md", "results_": "results_2025.md", "compute_": "compute_performance.md"}
UNC_SPREAD = {  # configs/uncertainty_2025_reduced.yaml (main) and uncertainty.yaml (the spreads are PLACEHOLDERS)
    "f_build": "build-mass factor ±30 % (Q39)",
    "Kv_factor": "Kv tolerance ±5 %", "lap.thrust_factor": "lap thrust factor ±10 %", "Rm_ref": "motor Rm ±20 %",
    "store_drag_factor": "store drag factor ±30 % (Q37)", "R_cell": "cell resistance ±30 %",
    "lap.wind": "wind 0–7 m/s (Q32)", "f_cb": "cargo-bay mass factor ±30 %", "e_spec": "pack specific energy ±10 %",
    "m_avionics": "avionics mass ±15 %", "m_motor": "motor mass ±10 %", "f_gear": "gear mass factor ±30 %",
    "f_tb": "tail-boom mass factor ±30 %", "f_ht": "HT mass factor ±30 %", "f_vt": "VT mass factor ±30 %",
    "eta_h": "tail dynamic-pressure ratio ±10 %"}


def git_hash():
    return subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=ROOT, capture_output=True,
                          text=True).stdout.strip()


def _tables(md):
    """Section headings of a markdown page that contain a table."""
    out, head, seen = [], None, set()
    for line in md.splitlines():
        if line.startswith("#"):
            head = line.lstrip("#").strip()
        elif line.startswith("|") and head and head not in seen:
            seen.add(head)
            out.append(head)
    return out


def _git_of(js):
    if "git" in js:
        return js["git"]
    if "machine" in js:
        return js["machine"].get("git")
    return "?"


def ranking():
    """PLACEHOLDERS ranked by their effect on the flight score F (each row states its measure)."""
    R = json.loads((DOCS / "results_2025.json").read_text())
    rows = []
    g = {(r["f2"], r["f3"]): r for r in R["normalisation"]["grid"] if "F" in r}
    c = g.get((1.0, 1.0))
    if c:
        for (f2, f3), lab in (((2.0, 1.0), "R2 x2 (0.40 lb/s)"), ((0.5, 1.0), "R2 x0.5 (0.10 lb/s)"),
                              ((1.0, 2.0), "R3 x2 (54)"), ((1.0, 0.5), "R3 x0.5 (13.5)")):
            if (f2, f3) in g:
                r = g[(f2, f3)]
                rows.append(("Q35 normalisation " + lab, r["F"] - c["F"],
                             f"F re-optimised; design change: fuel {r['fuel_lb'] - c['fuel_lb']:+.2f} lb, laps "
                             f"{r['laps'] - c['laps']:+.2f}"))
    b = {x["bottle"]: x for x in R.get("bottles", []) if "F" in x}
    if "1L" in b:
        for k in ("2L", "0.5L"):
            if k in b:
                rows.append((f"Q36/Q38 bottle {k} instead of 1 L (and its PLACEHOLDER dimensions)",
                             b[k]["F"] - b["1L"]["F"], f"F re-optimised; fuel {b[k]['fuel_lb']:.1f} lb"))
    sc = R["uq"]["reduced"]["screen"]["contrib"]["flight_score"]
    for k, v in sc.items():
        rows.append((f"uncertainty spread (reduced set, active): {UNC_SPREAD.get(k, k)}", math.sqrt(v),
                     "first-order sigma of F at the deterministic optimum (abs(dF/dp) x sigma_p)"))
    rr = (R["uq"]["reduced"].get("robust_eval") or {}).get("results", {})
    rc = (R["uq"]["calibrated"].get("robust_eval") or {}).get("results", {})
    base = R["best_case"]
    if base in rr and base in rc and "mu" in rr[base] and "mu" in rc[base]:
        cal = R["uq"]["calibrated"]["calibration"]["f_build"]
        rows.append((f"Q39 build mass calibrated on Woombah (f_build {cal:.2f} instead of 1)",
                     rc[base]["mu"] - rr[base]["mu"],
                     "change of the UDR mean of F at the deterministic optimum (calibrated set against reduced set)"))
    full = R["uq"]["full"]["screen"]["contrib"]["flight_score"]
    for k, v in full.items():
        if k not in sc:
            rows.append((f"uncertainty spread (full set only, now fixed): {UNC_SPREAD.get(k, k)}", math.sqrt(v),
                         "first-order sigma of F at the deterministic optimum, published 15-parameter screening"))
    rows.sort(key=lambda r: -abs(r[1]))
    return rows


def main():
    L = []
    w = L.append
    w("# 2024-25 results: index\n\n")
    w(f"Generated by `studies/results_index.py` (git `{git_hash()}`). Every page below is generated by its script "
      "from its JSON; rerun the script rather than editing the page. Run outputs (one JSON per optimisation, the "
      "Monte Carlo arrays, the timing measurements) are in `outputs/results/` (git-ignored).\n\n")
    w("## Pages, tables and JSON\n\n")
    w("| page | JSON | commit | script | configs | tables (section) |\n|---|---|---|---|---|---|\n")
    for page, js, script, cfg in PAGES:
        jf = DOCS / js
        if not jf.exists():
            continue
        gitc = _git_of(json.loads(jf.read_text()))
        tabs = _tables((DOCS / page).read_text()) if page and (DOCS / page).exists() else []
        w(f"| {f'`{page}`' if page else '–'} | `{js}` | `{gitc}` | `{script}` | {cfg} | "
          f"{'; '.join(tabs) or '–'} |\n")
    w("\n## Figures (vector PDF + PNG, `figures/`)\n\n")
    w("| figure | page | commit |\n|---|---|---|\n")
    for f in sorted((DOCS / "figures").glob("*.pdf")):
        page = next((p for k, p in FIGURES.items() if f.name.startswith(k)), "?")
        js = DOCS / page.replace(".md", ".json")
        gitc = _git_of(json.loads(js.read_text())) if js.exists() else "?"
        w(f"| `figures/{f.name}` (+ .png) | `{page}` | `{gitc}` |\n")
    w("\n## Caveats: PLACEHOLDERS affecting the results, ranked by their effect on F\n\n")
    w("Measures differ by row (stated): a normalisation or bottle row is the change in the re-optimised flight "
      "score; an uncertainty row is the first-order standard deviation of F that the PLACEHOLDER spread produces "
      "at the deterministic optimum (results_2025.md §7 contributions; parameters the reduced set fixes are listed from the published 15-parameter screening, §7d). The best deterministic F is "
      f"{max(r.get('F', 0.0) for r in json.loads((DOCS / 'results_2025.json').read_text())['enumeration']):.4f}, "
      "so 0.01 in F is about 0.2 %. The normalisation rows are mostly a rescaling of the score: the design barely "
      "moves (their measure column), so they change the totals and the M2/M3 balance of the score, not the aircraft "
      "or the ranking of the methods.\n\n")
    w("| rank | PLACEHOLDER | effect on F | measure |\n|---:|---|---:|---|\n")
    for i, (lab, d, how) in enumerate(ranking(), start=1):
        w(f"| {i} | {lab} | {d:+.4f} | {how} |\n")
    w("\nNot quantified (no study moves them; they enter through the rows above or not at all): the store "
      "interference factors and the X-1's t/c (Q37: covered by the store drag factor's ±30 %); the bottle empty "
      "masses (Q36: grams against kilograms of fuel); the pylons' spanwise station (Q36: carried as fuselage "
      "payload, conservative for the spar); the uncalibrated mass model (validation_2025.md: 24–41 % light on "
      "Woombah; a calibrated model was not used for the results, so absolute masses are low for every design "
      "alike); the missing take-off in the lap model (biases every M2 high alike); the motor catalogue and Kv bounds "
      "(Q7, Q34; Kv is set by the rpm limit, not its bounds).\n")
    w("\nThe tank size (Q38) is the largest lever overall: see results_2025.md §1b.\n")
    (DOCS / "INDEX.md").write_text("".join(L))
    print("wrote docs/studies/results/INDEX.md")


if __name__ == "__main__":
    main()
