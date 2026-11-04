"""The page of the 2024-25 re-run at the final code: docs/studies/results_2025_final/results_2025_final.{md,json} and
validation_2025_final.{md,json}, in the structure of results_2025.md, from outputs/results_2025_final/. Reuses
results_2025.collect() and render() with the 'reduced' set pointed at the final set; the sentences that describe the
published study's model (no take-off) are replaced. Run after the campaign, at any later commit: the run JSONs keep the
run commit (outputs/results_2025_final/RUN_COMMIT), and this script writes only the pages.
"""

import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "studies"))
import results_2025 as R  # noqa: E402
import validation_2025 as V  # noqa: E402

OUT_REL = "outputs/results_2025_final"
CFG, RDO_CFG, WB_CFG = ("configs/dbf2025_multipoint_final.yaml", "configs/dbf2025_multipoint_rdo_final.yaml",
                        "configs/woombah_2025_final.yaml")
DOCS = ROOT / "docs/studies/results_2025_final"


def stages():
    """Which stages completed, from the campaign's own log and state."""
    sys.path.insert(0, str(ROOT / "studies"))
    import results_2025_campaign as C

    state = C.jread(C.STATE) or dict(failed=[], skipped=[])
    js = C.jobs(set(state["failed"]))
    names = {1: "Woombah validation", 2: "enumeration", 3: "multistart", 4: "bottles", 5: "AirCrafter 1.0 mappings and common evaluation",
             6: "RDO", 7: "Monte Carlo", 8: "propagation", 9: "bookends, normalisation, screening", 10: "timing"}
    rows = []
    for s, nm in names.items():
        sj = [j for j in js if j.stage == s]
        d = [j for j in sj if j.done()]
        rows.append((s, nm, len(d), len(sj), [j.name for j in sj if not j.done()]))
    return rows, state


def wind_finding(data):
    """The published sentence says the wind costs laps. With the take-off in the mission time it hardly does: read it from the
    Monte Carlo array."""
    import numpy as np

    f = R.UQ_SETS["reduced"]["out"] / "mc_multipoint.npz"
    if not f.exists():
        return "**Wind and laps:** the deterministic optimum's Monte Carlo did not finish."
    z = np.load(f)
    names = [str(x) for x in z["outputs"]]
    laps, u = z["values"][:, names.index("points.M2.laps_frac")], z["u"]

    def pq(col):
        q = np.minimum((u[:, col] * 4).astype(int), 3)
        return [100 * float((laps[q == i] < 3 - 1e-6).mean()) for i in (0, 3)]

    w, b = pq(3), pq(0)
    lap0 = data["uq"]["reduced"]["screen"]["values"]["points.M2.laps_frac"]
    return (f"**Build mass, not wind, is what robustness buys here (reversal of the published finding).** The deterministic optimum flies "
            f"M2's 3 laps exactly in calm air, but at the mean headwind (3.5 m/s) it flies {lap0:.3f}: more, not fewer, because with the take-off "
            "in the mission time the ground roll starts at an airspeed equal to the headwind, which saves take-off energy (mission/smooth_lap.py). It misses "
            f"the 3 laps in {100 * float((laps < 3 - 1e-6).mean()):.0f} % of samples, {w[0]:.0f} % in the calmest quartile of wind and {w[1]:.0f} % in the windiest, "
            f"against {b[0]:.0f} % in the lightest quartile of build mass and {b[1]:.0f} % in the heaviest. The RDO design keeps the margin (mu - 2 sigma >= 3) for almost no score.")


def optima_finding(data):
    """The multistart's clusters and the default start's place in them."""
    runs = [json.loads((R.RUNS / f"{data['best_case']}_ms{r['index']}.json").read_text()) for r in data["multistart"]["runs"] if r["ok"]]
    if not runs:
        return ""
    groups = {}
    for d in runs:
        groups.setdefault(round(d["dv"]["c"], 2), []).append(d)  # by chord: the score gap is below any sensible rounding
    txt = "; ".join(f"F {v[0]['score']['flight_score']:.4f}: {len(v)} starts, chord {v[0]['dv']['c']:.3f}, t/c scale {v[0]['dv']['tc_scale']:.2f}, camber scale {v[0]['dv']['camber_scale']:.2f}"
                    for v in sorted(groups.values(), key=lambda v: v[0]["score"]["flight_score"]))
    return (f"**Not one optimum.** The converged multistarts reach {len(groups)} distinct airframes: {txt}. The default start (F {data['multistart']['default']:.4f}, the "
            "design every section below calls 'multipoint') is in the lower group, so it is not the best known design; the gap is smaller than any other effect on this page. "
            "Every robust start ends near the upper group's airframe.")


def main():
    R.use_paths(OUT_REL, CFG, RDO_CFG, WB_CFG, "docs/studies/results_2025_final")
    final = R.UQ_SETS["final"]
    R.UQ_SETS["reduced"] = dict(final, name="reduced", select=False)
    empty = R.OUT / "_none"
    for name in ("full", "onesided"):
        R.UQ_SETS[name] = dict(R.UQ_SETS[name], out=empty, prefix="_none_")
    R.use_set("reduced")
    data = R.collect()
    data["run_commit"] = (R.OUT / "RUN_COMMIT").read_text().strip()
    data["git"] = data["run_commit"]
    data["figures"] = R._figures(data)
    data["continuity"] = json.loads((R.RUNS / "continuity.json").read_text())["designs"]
    timing = sorted(R.RUNS.glob("timing_*.json"))
    data["timing"] = {f.stem: json.loads(f.read_text()).get("time") for f in timing}
    md = R.render(data)
    rows, state = stages()
    data["stages"] = rows
    data["campaign_failed"] = state["failed"]
    DOCS.mkdir(parents=True, exist_ok=True)
    (DOCS / "results_2025_final.json").write_text(json.dumps(data, indent=1, default=float))
    head, _, rest = md.partition("## Findings")
    new_head = ["# 2024-25 results at the final code: AirCrafter 1.0 vs multipoint vs RDO\n",
                f"- **Commit:** `{data['run_commit']}` on `rerun-2025-final`: tag `results-2026-final` (`0a6527d`) plus configs and "
                "study scripts; nothing under `aircrafter2/` differs from the tag.",
                "- **Take-off in mission time: on** (k_lof 1.1, CL_ground 0.15, start offset 0 ft; `configs/dbf2025_multipoint_takeoff.yaml`).",
                "- **Mass model: USYD-calibrated** (`configs/placeholders.yaml`; Woombah's validation column: factors fitted without it).",
                "- MDA continuation fallback on. Wind along the course axis (no wind direction, no turn drift). Course preset `dbf_2025`.",
                "- RDO: k = 1, beta = 2, reduced set (f_build ±30 %, f_tb ±30 %, m_avionics ±15 %, wind 0–7 m/s). Monte Carlos in batches of "
                f"{R.BATCH} samples on fresh workers.",
                "- **Stages completed:** " + "; ".join(f"{s} {nm} {d}/{n}" for s, nm, d, n, _ in rows) + ".",
                "\nGenerated by `studies/results_2025_final_report.py` from `outputs/results_2025_final/`; every number is read from "
                "`results_2025_final.json`. The published study at tag `results-2024-25-calibrated`: `docs/studies/results/results_2025.md`. "
                "Decisions and failures: `RERUN_2025_LOG.md`. Old against new: `thesis_number_diff.md`.\n",
                "**Biases stated up front.** (i) Take-off is in the mission time here, but the validation still shows the model flying "
                "too fast (`validation_2025_final.md`). (ii) The common evaluation in the AirCrafter 2.0 analysis favours the AirCrafter "
                "2.0 designs (they were optimised in it); the Monte Carlo favours RDO by construction. (iii) Analysis failures are "
                "counted separately from constraint violations everywhere.\n"]
    md = "\n".join(new_head) + "\n## Findings" + rest
    md = re.sub(r"- No take-off in the lap model.*?\n", "- Take-off is in the mission time at both points (the published study's was not); its "
                "ground-roll inputs CL_ground and start offset are PLACEHOLDERS.\n", md)
    md = re.sub(r"\*\*Wind is what robustness buys here\.\*\*.*?\n", wind_finding(data) + "\n", md, count=1)
    md = md.replace("## 1. Propulsion enumeration", optima_finding(data) + "\n\n## 1. Propulsion enumeration", 1)
    c = data["continuity"]
    rows_c = ["\n## 8. The published designs at the final code\n",
              "Each design of the published page, unchanged, analysed at the final code with take-off off, then on. The first step is "
              "what the code change moved; the second is what the take-off moved. A design optimised without take-off may violate "
              "constraints with it.\n",
              "| design | published F | final code, take-off off | take-off on | violated with take-off on |", "|---|---:|---:|---:|---|"]
    for k, x in c.items():
        off, on = x.get("takeoff_off", {}), x.get("takeoff_on", {})
        rows_c.append(f"| {k} | {x['old_F']:.4f} | {off.get('F', float('nan')):.4f} | {on.get('F', float('nan')):.4f} | "
                      f"{', '.join(v.replace('points.', '') for v in on.get('violated', [])) or 'none'} |")
    md = md.replace("\n## Caveats\n", "\n".join(rows_c) + "\n\n## Caveats\n")
    if data["timing"]:
        md = md.replace("\n## Caveats\n", "\n## 9. Timing\n\nSingle-threaded, each in its own process, nothing else running: "
                        + ", ".join(f"{k} {x / 60:.1f} min" for k, x in data["timing"].items() if x) + ".\n\n## Caveats\n")
    (DOCS / "results_2025_final.md").write_text(md)

    # the validation page, with the published script's report
    V.OUT, V.DOCS, V.CONFIG = R.OUT, DOCS, ROOT / WB_CFG
    V.report()
    txt = (DOCS / "validation_2025.md").read_text()
    txt = txt.replace("the lap model has no take-off, while the rules time M2 from the first throttle advance [R p. 11] "
                      "(switch `course.takeoff.in_mission_time`, off here; Woombah with take-off on: docs/studies/takeoff_study.md §4); ",
                      "take-off in the mission time (`course.takeoff.in_mission_time`, on here); ")
    txt = txt.replace("(no take-off)", "(take-off included)").replace("take-off (in the flown time, not in the model), ", "")
    txt = "**Re-run at the final code: take-off in mission time on, commit " + data["run_commit"] + ".**\n\n" + txt
    (DOCS / "validation_2025_final.md").write_text(txt)
    (DOCS / "validation_2025.md").unlink()
    (DOCS / "validation_2025_final.json").write_text((DOCS / "validation_2025.json").read_text())
    (DOCS / "validation_2025.json").unlink()
    print("wrote", DOCS)


if __name__ == "__main__":
    main()
