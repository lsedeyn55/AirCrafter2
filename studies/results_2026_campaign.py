"""The final 2025-26 campaign's queue: every run of the planned run list, in order, within the memory limit, resuming.

    .venv/bin/python studies/results_2026_campaign.py run     # under systemd-run --user (docs: docs/development/RESULTS_LOG.md)
    .venv/bin/python studies/results_2026_campaign.py status  # what is done, ready, running and left

Each job is one process with a log in outputs/results_2026/logs/. A job is done when its JSON exists and records the
freeze commit, clean, so a restart skips it. At most WEIGHT_MAX heavy jobs at once (an optimisation or an RDO counts 1, a
Monte Carlo counts its workers), and a job starts only if MemAvailable leaves its share. A failed process is retried
once; then it is logged as missing and the jobs that need it never start. The campaign refuses to start unless HEAD is
the freeze tag and the tracked tree is clean.
"""

import argparse
import json
import os
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "studies"))
import results_2026 as R  # noqa: E402

PY = str(ROOT / ".venv" / "bin" / "python")
LOGS = R.OUT / "logs"
STATE = R.OUT / "campaign_state.json"
CAMPAIGN_LOG = R.OUT / "campaign.log"
WEIGHT_MAX = 3
MEM_MARGIN_GB = 1.0
DV_STUDY = "studies/results_2026.py"
LABELS_IN = ["mdo", "rdo", "mdo_wind", "ac1_completed", "ac1_direct", "humpback", "humpback_loo"]
LABELS_OUT = ["mdo", "rdo", "mdo_wind", "humpback"]
LABELS_COMMON = ["humpback", "humpback_loo", "ac1_direct", "ac1_completed", "mdo", "mdo_wind", "rdo"]


@dataclass
class Job:
    name: str
    argv: list
    done_file: Path
    weight: int = 1
    mem_gb: float = 1.5
    env: dict = field(default_factory=dict)
    check_git: bool = True


def freeze_hash():
    return subprocess.run(["git", "rev-parse", "--short", R.FREEZE_TAG], cwd=ROOT, capture_output=True, text=True).stdout.strip()


def is_done(path, freeze, check_git=True):
    """A run is done when its file exists and records the freeze commit, clean (a run from anywhere else is not reused)."""
    p = Path(path)
    if not p.exists():
        return False
    if not check_git:
        return True
    try:
        return json.loads(p.read_text()).get("git") == freeze
    except (json.JSONDecodeError, OSError):
        return False


def run_file(tag):
    return R.RUNS / f"{tag}.json"


def opt(tag, *args, weight=1, mem=1.5):
    return Job(tag, [PY, DV_STUDY, "optimise", "--tag", tag, *args], run_file(tag), weight, mem)


def study(name, part, *args, done, weight=1, mem=1.5):
    return Job(name, [PY, DV_STUDY, part, *args], done, weight, mem)


def validation_jobs():
    out = [Job("validation_envelope", [PY, "studies/validation_2026.py", "envelope"], R.OUT / "validation_envelope.json", 3, 3.0)]
    for m in ("loo", "gg", "cal"):
        out.append(Job(f"validation_{m}", [PY, "studies/validation_2026.py", "humpback", "--mass", m],
                       R.OUT / f"validation_humpback_{m}.json", 3, 3.0))
    return out


def jobs(freeze, failed):
    """The job list from the current state: a job appears once what it needs exists; the order is the priority."""

    def ok(path):
        return is_done(path, freeze)

    out = []
    check = run_file("fr_check")
    out.append(opt("fr_check", weight=3, mem=2.0))
    if not ok(check) and "fr_check" not in failed:
        return out  # the reproducibility check first, alone
    props = [(m2, m3) for m2 in R.M2_PROPS for m3 in R.M3_PROPS]
    enum8 = [R.enumeration_tag(8, m2, m3) for m2, m3 in props]
    for (m2, m3), tag in zip(props, enum8):
        out.append(opt(tag, "--cells", "8", "--m2", m2, "--m3", m3))
    enum8_terminal = all(ok(run_file(t)) or t in failed for t in enum8)
    if not enum8_terminal:
        return out
    ranked8 = R.ranked_cases(8.0)
    if not ranked8:
        return out
    best8 = ranked8[0]
    tag6 = R.enumeration_tag(6, best8["props"]["M2"], best8["props"]["M3"])
    out.append(opt(tag6, "--cells", "6", "--m2", best8["props"]["M2"], "--m3", best8["props"]["M3"]))
    if not (ok(run_file(tag6)) or tag6 in failed):
        return out
    ranked = R.ranked_cases()
    best = ranked[0]
    second = ranked[1] if len(ranked) > 1 else None
    case = best["tag"]
    cells, m2, m3 = str(best["cells"]), best["props"]["M2"], best["props"]["M3"]
    sel = ["--cells", cells, "--m2", m2, "--m3", m3]
    out.append(study(f"integer_{case}", "integer", "--case", case, done=rounded_file(case, ok)))
    if not all_rounded(case, ok) and f"integer_{case}" not in failed:
        return out
    mdo = R.best_integer(case)
    if mdo is None:
        return out
    fix = ["--fix", f"n_p={mdo['fixed']['n_p']:g}", f"n_c={mdo['fixed']['n_c']:g}"]
    # the first robust start as soon as the best case is known
    out.append(study("fr_rdo_mdo", "rdo", "--tag", "fr_rdo_mdo", "--case", case, "--start", mdo["tag"], "--free",
                     done=run_file("fr_rdo_mdo"), mem=RDO_MEM))
    # deterministic
    out.append(opt("fr_wind", *sel, "--mean-wind"))
    if ok(run_file("fr_wind")):
        out.append(study("integer_fr_wind", "integer", "--case", "fr_wind", done=rounded_file("fr_wind", ok)))
    out.append(Job("ac1_sweep", ["matlab", "-batch", "run('reference/aircrafter1/run_2026.m')"],
                   R.OUT / "ac1" / "fr_ac1_2026_shipped.json", 1, 1.5,
                   env=dict(AC1_SHIPPED_STEPS="1", AC1_TAG="fr_ac1_2026_shipped", AC1_OUT=str(R.OUT / "ac1"), AC1_FRESH="1",
                            AC1_GIT=freeze)))
    if (R.OUT / "ac1" / "fr_ac1_2026_shipped.json").exists() or "ac1_sweep" in failed:
        out.append(Job("fr_ac1_direct", [PY, "studies/aircrafter1_2026.py", "direct"], run_file("fr_ac1_direct")))
        out.append(Job("fr_ac1_completed", [PY, "studies/aircrafter1_2026.py", "completed"], run_file("fr_ac1_completed")))
    start = ["--start", case]
    out.append(opt("fr_M2only", *sel, "--w2", "1", "--w3", "0", *start))
    out.append(opt("fr_M3only", *sel, "--w2", "0", "--w3", "1", *start, "--maxiter", "300"))
    for lab, a in (("fr_norm_NI0.5", ["--NI", "1400"]), ("fr_norm_NI2", ["--NI", "5600"]),
                   ("fr_norm_B0.5", ["--B", "750"]), ("fr_norm_B2", ["--B", "3000"])):
        out.append(opt(lab, *sel, *a, *start))
    frm = ["--start", mdo["tag"], *fix]
    out.append(opt("fr_bound_i", *sel, *frm, "--bound", "c.upper=0.6", "camber_scale.upper=2.5"))
    # (ii) Raymer 6th ed., Table 6.4, p. 160, homebuilt row: 0.50, 0.04; (iii) the lowest flown: [OH] 0.66, [W] 0.059
    out.append(opt("fr_bound_ii", *sel, *frm, "--override", "stability.V_h_min=0.50", "stability.V_v_min=0.04"))
    out.append(opt("fr_bound_iii", *sel, *frm, "--override", "stability.V_h_min=0.66", "stability.V_v_min=0.059"))
    out.append(opt("fr_old_constraints", *sel, *frm, "--old-constraints"))
    out.append(opt("fr_takeoff_off", *sel, *frm, "--takeoff-off"))
    for i in range(10):
        out.append(study(f"fr_ms{i}", "multistart", "--case", case, "--index", str(i), done=run_file(f"fr_ms{i}")))
    # the other robust starts
    if second is not None:
        out.append(study("fr_rdo_second", "rdo", "--tag", "fr_rdo_second", "--case", case, "--start", second["tag"], "--free",
                         done=run_file("fr_rdo_second"), mem=RDO_MEM))
    out.append(study("fr_rdo_humpback", "rdo", "--tag", "fr_rdo_humpback", "--case", case, "--start", "humpback", "--free",
                     done=run_file("fr_rdo_humpback"), mem=RDO_MEM))
    # analyses of the designs that exist
    have = {"humpback": True, "humpback_loo": True, "mdo": True,
            "ac1_direct": ok(run_file("fr_ac1_direct")), "ac1_completed": ok(run_file("fr_ac1_completed")),
            "mdo_wind": R.best_integer("fr_wind") is not None and all_rounded("fr_wind", ok), "rdo": rdo_final(freeze, failed)}
    starts = ["fr_rdo_mdo", "fr_rdo_second", "fr_rdo_humpback"]
    if all(ok(run_file(t)) or t in failed for t in starts):
        best = R.best_rdo(rounded=False)
        if best:
            out.append(study("rdo_integer", "rdo_integer", "--case", best["tag"], done=rounded_file(best["tag"], ok),
                             mem=RDO_MEM))
    for lab in LABELS_COMMON:
        if have[lab]:
            out.append(study(f"fr_common_{lab}", "common", "--label", lab, done=run_file(f"fr_common_{lab}")))
    for lab in ("mdo", "rdo"):
        if have[lab]:
            out.append(study(f"fr_reanalysis_{lab}", "reanalysis", "--label", lab, done=run_file(f"fr_reanalysis_{lab}")))
            out.append(study(f"fr_wind_env_{lab}", "wind_env", "--label", lab, done=run_file(f"fr_wind_env_{lab}")))
    out += validation_jobs()
    if have["mdo"]:
        out.append(study("fr_screen_mdo", "screen", "--label", "mdo", done=run_file("fr_screen_mdo"), weight=3, mem=3.0))
        out.append(study("fr_placeholders_mdo", "placeholders", "--label", "mdo", done=run_file("fr_placeholders_mdo"), weight=3,
                         mem=3.5))
    if have["rdo"]:
        out.append(study("fr_screen_rdo", "screen", "--label", "rdo", done=run_file("fr_screen_rdo"), weight=3, mem=3.0))
        out.append(study("fr_node_check", "node_check", done=run_file("fr_node_check"), weight=3, mem=3.0))
    for lab in LABELS_COMMON:
        if have[lab]:
            out.append(study(f"fr_wind_dir_{lab}", "wind_dir", "--label", lab, done=run_file(f"fr_wind_dir_{lab}"), weight=3,
                             mem=2.5))
    for lab in LABELS_IN:
        if have[lab]:
            out.append(study(f"fr_mc_in_{lab}", "mc", "--label", lab, "--set", "in", done=run_file(f"fr_mc_in_{lab}"), weight=3,
                             mem=2.5))
    if ok(run_file("fr_mc_in_rdo")):
        out.append(study("fr_propagation", "propagation", done=run_file("fr_propagation"), weight=3, mem=2.5))
    for lab in LABELS_OUT:
        if have[lab]:
            out.append(study(f"fr_mc_out_{lab}", "mc", "--label", lab, "--set", "out", done=run_file(f"fr_mc_out_{lab}"), weight=3,
                             mem=2.5))
    return out


RDO_MEM = 2.5  # GB, replaced by the measured peak once a robust run has finished (run())


def all_rounded(case, ok):
    """The rounding of `case` is finished: its check exists and every candidate it kept has a run."""
    f = run_file(f"{case}_int_check")
    if not ok(f):
        return False
    d = json.loads(f.read_text())
    return all(ok(run_file(f"{case}_int_{a}p{b}c")) for a, b in d["run"])


def rounded_file(case, ok):
    """The rounding job's done file: its check once every candidate has a run, else a name that doesn't exist (a restart
    re-enters the job, which skips the finished candidates)."""
    return run_file(f"{case}_int_check") if all_rounded(case, ok) else run_file(f"{case}_int_unfinished")


def rdo_final(freeze, failed):
    """The robust design is final: every start finished, and the rounding of the best one finished or failed."""
    ok = lambda p: is_done(p, freeze)  # noqa: E731
    if not all(ok(run_file(t)) or t in failed for t in ("fr_rdo_mdo", "fr_rdo_second", "fr_rdo_humpback")):
        return False
    best = R.best_rdo(rounded=False)
    if best is None:
        return False
    return all_rounded(best["tag"], ok) or "rdo_integer" in failed


def mem_available_gb():
    for line in Path("/proc/meminfo").read_text().splitlines():
        if line.startswith("MemAvailable"):
            return int(line.split()[1]) / 1e6
    return 0.0


def clog(msg):
    line = f"{time.strftime('%Y-%m-%d %H:%M:%S')} {msg}"
    print(line, flush=True)
    with CAMPAIGN_LOG.open("a") as f:
        f.write(line + "\n")


def run_table(freeze):
    """The log's run table, from the JSONs (the measured numbers, nothing typed)."""
    rows = ["| tag | what | start → wall | its (nit) | converged | F capped | F floored | MDA fallbacks | commit |",
            "|---|---|---|---:|---|---:|---:|---|---|"]
    for f in sorted(R.RUNS.glob("fr_*.json"), key=lambda p: p.stat().st_mtime):
        d = json.loads(f.read_text())
        wall = d.get("time") or d.get("wall")
        when = time.strftime("%d %b %H:%M", time.localtime(f.stat().st_mtime))
        sc = d.get("score", {})
        fs = (d.get("summary") or {}).get("flight_score", {})
        F = sc.get("flight_score") if sc else fs.get("mu")
        floored = (d.get("floored") or {}).get("flight_score")
        fb = d.get("mda_fallbacks")
        fb = "; ".join(f"{k} {x['continuation']}c" for k, x in fb.items()) if fb else ("–" if "fallbacks" not in d else
             f"{sum(x['continuation'] for x in d['fallbacks'])}c")
        rows.append(f"| {d.get('tag', f.stem)} | {'RDO μ' if fs else ''} | ends {when}, {wall or 0:.0f} s | "
                    f"{d.get('its', '–')} ({d.get('nit', '–')}) | {d.get('ok', '–')} | "
                    f"{'–' if F is None else f'{F:.4f}'} | {'–' if floored is None else f'{floored:.4f}'} | {fb} | {d.get('git')} |")
    return "\n".join(rows)


def run():
    global RDO_MEM
    freeze = freeze_hash()
    if not freeze or R.git_hash() != freeze:
        raise SystemExit(f"HEAD is {R.git_hash()}, the freeze tag is {freeze!r}: tag the clean tree first")
    LOGS.mkdir(parents=True, exist_ok=True)
    state = json.loads(STATE.read_text()) if STATE.exists() else dict(attempts={}, failed=[])
    running = {}
    idle_since, mem_logged = None, set()
    env0 = dict(os.environ, OMP_NUM_THREADS="3", AIRCRAFTER2_APC_DIR="/home/lucas/University/THESIS/PERFILES2")
    clog(f"campaign start, freeze {freeze}, {len(state['failed'])} failed so far")
    while True:
        for name, (proc, job, t0, logf) in list(running.items()):
            if proc.poll() is None:
                continue
            del running[name]
            logf.close()
            wall = time.time() - t0
            good = proc.returncode == 0 and is_done(job.done_file, freeze, job.check_git)
            clog(f"end   {name}: exit {proc.returncode}, {wall:.0f} s, done={good}")
            state["attempts"][name] = state["attempts"].get(name, 0) + 1
            if not good and state["attempts"][name] >= 2:
                state["failed"].append(name)
                clog(f"FAILED {name} after {state['attempts'][name]} attempts: logged as missing")
            STATE.write_text(json.dumps(state, indent=1))
            write_status(freeze, state, running)
        peaks = [json.loads(f.read_text()).get("peak_rss_gb") or 0 for f in R.RUNS.glob("fr_rdo_*.json")
                 if not f.stem.endswith("_check")]
        if peaks:
            RDO_MEM = max(peaks) + 0.3
        todo = [j for j in jobs(freeze, set(state["failed"]))
                if not is_done(j.done_file, freeze, j.check_git) and j.name not in state["failed"]]
        waiting = [j for j in todo if j.name not in running]
        used = sum(r[1].weight for r in running.values())
        started = False
        for j in waiting:
            # keep the order for the heavy jobs: a Monte Carlo waits for its slots instead of being overtaken for ever
            if used + j.weight > WEIGHT_MAX:
                if j.weight == WEIGHT_MAX:
                    break
                continue
            if mem_available_gb() < j.mem_gb + MEM_MARGIN_GB and not (not running and idle_since and
                                                                       time.time() - idle_since > 900):
                if j.name not in mem_logged:
                    clog(f"wait  {j.name}: {mem_available_gb():.1f} GB available, needs {j.mem_gb:.1f} + margin")
                    mem_logged.add(j.name)
                continue
            logf = (LOGS / f"{j.name}.log").open("a")
            logf.write(f"\n--- {time.strftime('%F %T')} {' '.join(j.argv)}\n")
            logf.flush()
            proc = subprocess.Popen(j.argv, cwd=ROOT, env={**env0, **j.env}, stdout=logf, stderr=subprocess.STDOUT)
            running[j.name] = (proc, j, time.time(), logf)
            used += j.weight
            started = True
            clog(f"start {j.name} (weight {j.weight}, {mem_available_gb():.1f} GB available)")
        if running or started:
            idle_since = None
        elif not waiting:
            clog("campaign finished: nothing left")
            write_status(freeze, state, running)
            return
        elif idle_since is None:  # waiting only on memory: after 15 min idle the head job starts anyway (swap protects it)
            idle_since = time.time()
        time.sleep(20)


def write_status(freeze, state, running):
    """The machine-written part of the log (outputs/results_2026/campaign_status.md, pasted into docs/development/RESULTS_LOG.md)."""
    lines = [f"Updated {time.strftime('%Y-%m-%d %H:%M:%S')} (freeze {freeze}). Running: {', '.join(running) or 'none'}. "
             f"Failed: {', '.join(state['failed']) or 'none'}.", "", run_table(freeze)]
    (R.OUT / "campaign_status.md").write_text("\n".join(lines) + "\n")


def status():
    freeze = freeze_hash()
    state = json.loads(STATE.read_text()) if STATE.exists() else dict(attempts={}, failed=[])
    js = jobs(freeze, set(state["failed"]))
    done = [j.name for j in js if is_done(j.done_file, freeze, j.check_git)]
    left = [j.name for j in js if j.name not in done]
    print(f"freeze {freeze}\ndone {len(done)}: {done}\nleft {len(left)}: {left}\nfailed: {state['failed']}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("part", choices=["run", "status"])
    {"run": run, "status": status}[ap.parse_args().part]()
