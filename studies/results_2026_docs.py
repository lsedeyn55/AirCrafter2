"""The generated parts of RESULTS_LOG.md, docs/RESULTS_2026_PLAN.md, docs/OPEN_QUESTIONS.md and README.md: every number comes
from the pages' JSON, the run table from the campaign's status file. Rerun after the pages; each part replaces itself.

    .venv/bin/python studies/results_2026_docs.py
"""

import json
import re
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DOCS = ROOT / "docs" / "studies" / "results_2026"
RUNS = ROOT / "outputs" / "results_2026"
LOG = ROOT / "RESULTS_LOG.md"
LABEL = dict(humpback="Humpback", ac1_completed="AC1 mapped (b)", mdo="MDO", mdo_wind="MDO, mean wind", rdo="MDO + RDO")


def load():
    R = json.loads((DOCS / "results_2026.json").read_text())
    C = json.loads((DOCS / "compute_performance_2026.json").read_text())
    V = json.loads((DOCS / "validation_2026.json").read_text())
    S = json.loads((DOCS / "suite_counts.json").read_text())
    return R, C, V, S


def nominal(R, lab):
    return R["designs"][lab]["common"]["nominal"]


def wall(C, name):
    return next(r["mean"] for r in C["main_rows"] if r["name"] == name)


def block(text, start, end, body):
    """Replace what lies between the two markers, or add it at the end."""
    part = f"{start}\n{body.rstrip()}\n{end}"
    if start in text:
        return re.sub(re.escape(start) + r".*?" + re.escape(end), lambda _: part, text, flags=re.S)
    return text.rstrip() + "\n\n" + part + "\n"


def headline(R, C):
    rows = ["| design | F nominal | floored | MC mean | MC sigma | mean - k sigma | P(any) in set | P(any) out of set | MC failed |", "|---|---:|---:|---:|---:|---:|---:|---:|---:|"]
    for lab, name in LABEL.items():
        n, m, o = nominal(R, lab), R["designs"][lab]["mc_in"], R["designs"][lab].get("mc_out")
        f = m["stats"]["flight_score"]
        rows.append(f"| {name} | {n['score']['flight_score']:.4f} | {_floored(n):.4f} | {f['mu']:.4f} | {f['sigma']:.4f} | {f['robust']:.4f} | {m['p_any']:.3f} | "
                    + (f"{o['p_any']:.3f}" if o else "–") + f" | {m['mda_failed']} + {m['wind_limit']} of {m['N']} |")
    return "\n".join(rows)


def _floored(n):
    x = n["floored"]
    return x["flight_score"] if isinstance(x, dict) else x


def morning(R, C, V, S):
    mdo, rdo, hb, a2 = (nominal(R, x)["score"]["flight_score"] for x in ("mdo", "rdo", "humpback", "ac1_completed"))
    d = R["decomposition"]
    cr = V["criteria"]
    ss = R["sub_stall"]["mdo"]
    out = R["designs"]
    w = lambda n: wall(C, n)  # noqa: E731
    norm = {k: v["F"] - R["designs"]["mdo"]["common"]["nominal"]["score"]["flight_score"] for k, v in R["normalisation"].items()}
    ms = R["multistart_summary"]
    bad = next(r for r in C["scaling"]["grid_vs_gradient"]["gradient"] if not r["ok"]) if any(not r["ok"] for r in C["scaling"]["grid_vs_gradient"]["gradient"]) else None
    return f"""## Morning summary
**Outcome.** Every planned run finished; no cuts. The four designs, the sensitivities, the Monte Carlos and the timing are done, and the pages are in `docs/studies/results_2026/` (start at `INDEX.md`). In AirCrafter 2.0's own model, MDO reaches F {mdo:.4f}, against {hb:.4f} for Humpback and {a2:.4f} for AirCrafter 1.0 mapped (b). RDO gives up {mdo - rdo:.4f} of nominal F, keeps the mean, cuts sigma from {d['sigma']['mdo']:.4f} to {d['sigma']['rdo']:.4f} and takes P(any violation) from {d['p_any']['mdo']:.3f} to {d['p_any']['rdo']:.3f} in set. Out of set it's {out['mdo']['mc_out']['p_any']:.3f} to {out['rdo']['mc_out']['p_any']:.3f}, so the robustness isn't tuned only to the five parameters. These are comparisons inside one model that favours its own optima, not predictions: the model misses most of Humpback's flight criteria. Tests: {S['freeze_passed']} passed at the freeze, {S['final_passed']} at the final commit. Branch `thesis-results-2026`, tag `results-2026-final` (`{R['freeze']}`), unmerged.

{headline(R, C)}

**Wall times (single-threaded):** AirCrafter 1.0 sweep {w('AirCrafter 1.0 sweep, shipped steps') / 60:.1f} min; MDO from Humpback {w('AirCrafter 2.0 multipoint, cold') / 60:.1f} min; RDO to convergence {w('RDO to convergence, one run') / 3600:.2f} h.

**The five things most worth your attention:**
1. **A model bug invalidated my first Monte Carlos; the headline results are clean now.** A worker's problem goes permanently singular after about 250-280 evaluations, which cost 17-30 % of the samples. I reran all 11 with fresh workers, using a new script added after the freeze: an exception to the brief, Decision 13. All samples now converge, and F agrees to about 2e-12 where both runs converged. The bug itself is unfixed, and older studies' Monte Carlos may have hit it ("For me to look at").
2. **Validation is weak on flight.** §3.7's criteria: structure {cr['mass_pass']} of {cr['mass_n']} rows pass, flight {cr['flight_pass']} of {cr['flight_n']}. M3 laps run 20-46 % too fast. Keep F as a comparison in Chapter 4, not as a predicted score (`validation_2026.md`).
3. **Much of the optimum is set by PLACEHOLDERS.** The tail volumes, chord, camber and banner sit on PLACEHOLDER bounds, and M2 sits at the cap, so the normalisation (Q52) sets the payload: F moves {norm['B2']:+.3f} at B_max x2 and {norm['NI2']:+.3f} at NI_max x2. Bound changes are smaller: results_2026.md §5.3.
4. **Read RDO's gain as variance and feasibility, not mean.** {ss['n']} of {ss['of']} MDO samples sit on the sub-stall balance at fixed throttle. Without them MDO's mean is {ss['mu_without']:.4f}, above RDO's {d['mu']['rdo']:.4f}. Those samples are violations with an F that means little; it's your call how to phrase this in Chapter 4.
5. **Three judgement calls need your check.** The rounded RDO needed more than the brief's 15 iterations (Decision 10). The multistart converges from only {ms['converged']} of {ms['n']} starts, all to one optimum, and the others are analysis failures at the start design; one of three RDO starts fails too.{f" The {bad['n']}-variable scaling run did not converge in {bad['nit']} iterations, so that scaling curve isn't clean." if bad else ""} The thesis text still describes commit `73a1db4`, not the tag (`INDEX.md` hand-off).
"""


def plan(R, C, S):
    return f"""## Final results run
Done 2026-11-04 on `thesis-results-2026`; freeze tag `results-2026-final` (`{R['freeze']}`). What ran: the enumeration, rounding, `mdo_wind`, multistart, bookends, normalisation, bound and other sensitivities, AirCrafter 1.0 at its shipped steps with mappings (a) and (b), RDO from three starts with robust rounding, common evaluation, 11 Monte Carlos of 1024 points, propagation, screening, node check, wind, placeholders, validation, and the timing. {R['provenance']['n']} run files back the page, {len(R['provenance']['not_at_freeze'])} of them off the freeze commit. Cuts: none. Tests: {S['freeze_passed']} at the freeze, {S['final_passed']} at the final commit. Results: `docs/studies/results_2026/INDEX.md`. Log, with the decisions: `RESULTS_LOG.md`."""


def questions(R):
    F0 = nominal(R, "mdo")["score"]["flight_score"]
    b = {x["key"]: x for x in R["bound_lines"] if "missing" not in x}
    rows = {(r["name"], r["side"]): r for r in R["placeholders"]["rows"]}

    def span(s):
        lo, hi = rows[(s, "lo")], rows[(s, "hi")]
        return f"F {lo['F']:.4f} to {hi['F']:.4f} over {lo['value']:g} to {hi['value']:g}"

    norm = R["normalisation"]
    ov = R["overlap"]
    sc = R["designs"]["mdo"]["screen"]["oned"]
    re_ = R["designs"]["mdo"]["reanalysis"]["cases"]
    cd = "; ".join(f"{k.split(' ')[0]}: " + (f"F {c['F']:.4f}" if c.get("F") else "lap-model limit") for k, c in re_.items() if "rpm" not in k and "full" not in k)
    return f"""## Measured by the final results run (2026-11-04)
Measurements only; each question stays open unless it says otherwise. MDO F {F0:.4f}. Source: `docs/studies/results_2026/results_2026.md`, at the tag `results-2026-final`.

- **Q9** (§5.3): tail-volume floors at Raymer's homebuilt row cost ΔF {b['ii']['dF']:+.4f}, at the lowest flown {b['iii']['dF']:+.4f}. The optimum sits on the 0.35 and 0.02 PLACEHOLDERS.
- **Q10** (§5.3): c <= 0.60 m and camber_scale <= 2.5 gain ΔF {b['i']['dF']:+.4f}, NeuralFoil confidence {b['i'].get('nf', float('nan')):.3f}.
- **Q31** (§7): the box-boom overlap at MDO is {ov['mdo']['length']:.3f} m; at RDO {ov['rdo']['length']:.3f} m. Not fixed.
- **Q52** (§5.2), re-optimised: NI_max x0.5 F {norm['NI0.5']['F']:.4f}, x2 F {norm['NI2']['F']:.4f}; B_max x0.5 F {norm['B0.5']['F']:.4f}, x2 F {norm['B2']['F']:.4f}.
- **Q55** (§5.5): passenger pitch: {span('Q55 passenger pitch (m)')}; width: {span('Q55 fuselage width (m)')}; height: {span('Q55 fuselage height (m)')}.
- **Q56** (§5.3, §5.4): banner bound to 500 in gains ΔF {b['banner']['dF']:+.4f}. C_D,b re-analysis at fixed design and throttle: {cd}.
- **Q59** (§4): wind direction's sigma of F at MDO, 1-D Gauss: {sc['lap.wind_dir'] ** 0.5:.4f}.
- **Q63** (§5.5): gear area: {span('Q63 gear drag area (m²)')}; leakage: {span('Q63 leakage drag area (m²)')}."""


def main():
    R, C, V, S = load()
    subprocess.run([str(ROOT / ".venv/bin/python"), "studies/results_2026_campaign.py", "status"], cwd=ROOT, check=False, capture_output=True)
    text = LOG.read_text()
    table = (RUNS / "campaign_status.md").read_text()
    findings = "\n".join(f"{i}. {x}" for i, x in enumerate(R["findings"], 1))
    recs = "\n".join(f"- {x}" for x in R["recommendations"])
    perf = "\n".join(f"| {r['name']} | {r['mean']:.3g} s | {r['stat']['n']} |" for r in C["main_rows"])
    text = block(text, "<!-- run table -->", "<!-- /run table -->", f"## Run table\nFrom `outputs/results_2026/campaign_status.md`; every `fr_*` run. The 11 Monte Carlo rows are the batched reruns, at the same commit (Decision 13). `fr_propagation` was rerun on the new arrays.\n\n{table}\n**Timing (single-threaded, `outputs/results_2026/perf/`):**\n\n| measurement | mean | repeats |\n|---|---:|---:|\n{perf}")
    text = block(text, "<!-- findings -->", "<!-- /findings -->", f"## Findings\n{findings}\n\n## Recommendations (as on the page)\n{recs}")
    summary = morning(R, C, V, S)
    start, end = "<!-- morning -->", "<!-- /morning -->"
    if start in text:
        text = block(text, start, end, summary)
    else:
        head, rest = text.split("\n", 1)
        text = f"{head}\n\n{start}\n{summary.rstrip()}\n{end}\n{rest}"
    LOG.write_text(text)
    p = ROOT / "docs" / "RESULTS_2026_PLAN.md"
    p.write_text(block(p.read_text(), "<!-- final results run -->", "<!-- /final results run -->", plan(R, C, S)))
    p = ROOT / "docs" / "OPEN_QUESTIONS.md"
    p.write_text(block(p.read_text(), "<!-- final results run -->", "<!-- /final results run -->", questions(R)))
    p = ROOT / "README.md"
    t = p.read_text()
    t = re.sub(r"# \d+ tests \([0-9-]+, about \d+ min\)", f"# {S['final_passed']} tests ({S['date']}, about {S['minutes']} min)", t)
    pointer = "- `docs/studies/results_2026/`: the 2025-26 results, the thesis's main results. `INDEX.md` lists every table, figure and JSON; `validation_2026.md` (Humpback), `results_2026.md` (Humpback, AirCrafter 1.0, MDO, MDO + RDO), `compute_performance_2026.md`.\n"
    if "docs/studies/results_2026/`: " not in t:
        t = t.replace("- `docs/RESULTS_PLAN.md`:", pointer + "- `docs/RESULTS_PLAN.md`:", 1)
    p.write_text(t)
    print("wrote the log's generated parts, the plan's section, the open questions' section, the README")


if __name__ == "__main__":
    main()
