"""The 2024-25 study re-run at the final code: the queue of the brief's stages, in priority order, resuming.

    <venv python> studies/results_2025_campaign.py freeze   # record HEAD as the run commit (clean tree only)
    <venv python> studies/results_2025_campaign.py run      # under systemd-run --user (RERUN_2025_LOG.md)
    <venv python> studies/results_2025_campaign.py status

Modelled on results_2026_campaign.py. Each job is one process with a log in outputs/results_2025_final/logs/. A job is done
when its JSON records the run commit, clean, so a restart skips it. Three slots (an optimisation or an RDO counts 1, a Monte
Carlo counts its 3 workers), and a job starts only if MemAvailable leaves its share plus 2 GB. A failed process is retried
once, then logged and skipped; the rest continue. No job starts after STOP_START, and running jobs are stopped at
STOP_KILL, so that the pages are written by the deadline (a Monte Carlo keeps its finished batches).
"""

import argparse
import json
import os
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "studies"))
import results_2025 as R  # noqa: E402

OUT_REL = "outputs/results_2025_final"
CFG, RDO_CFG, WB_CFG = ("configs/dbf2025_multipoint_final.yaml", "configs/dbf2025_multipoint_rdo_final.yaml",
                        "configs/woombah_2025_final.yaml")
R.use_paths(OUT_REL, CFG, RDO_CFG, WB_CFG)
R.use_set("final")
OUT = R.OUT
LOGS, STATE, CLOG, COMMIT_FILE = OUT / "logs", OUT / "campaign_state.json", OUT / "campaign.log", OUT / "RUN_COMMIT"
PY = str(Path.home() / "University/THESIS/AirCrafter2/.venv/bin/python")
SLOTS, MEM_FREE_GB = 3, 2.0
RES = ["studies/results_2025.py"]
PATHS = ["--config", CFG, "--rdo-config", RDO_CFG, "--woombah-config", WB_CFG, "--out", OUT_REL, "--set", "final"]
TIMING_BEFORE = "05:00"
STOP_START, STOP_KILL = "05:55", "06:10"
ENUM = [(4, "11x8E", "10x7E"), (5, "11x8E", "10x7E")] + [(6, m2, m3) for m2 in ("11x8E", "10x7E", "9x6E")
                                                         for m3 in ("10x7E", "11x8E", "9x6E")]
ENUM.sort(key=lambda c: c != (6, "11x8E", "10x7E"))  # the previous best first
MC_ORDER = ["multipoint", "rdo", "woombah", "ac1_completed", "ac1_direct", "multipoint_2L"]
SINGLE = {"OMP_NUM_THREADS": "1", "OPENBLAS_NUM_THREADS": "1", "MKL_NUM_THREADS": "1"}


@dataclass
class Job:
    name: str
    stage: int
    argv: list
    done: Callable
    weight: int = 1
    mem_gb: float = 1.3
    env: dict = field(default_factory=dict)


def commit():
    return COMMIT_FILE.read_text().strip() if COMMIT_FILE.exists() else ""


def jread(path):
    try:
        return json.loads(Path(path).read_text())
    except (OSError, json.JSONDecodeError):
        return None


def done_json(path):
    def f():
        d = jread(path)
        return bool(d) and d.get("git") == commit()
    return f


def run_done(tag):
    return done_json(R.RUNS / f"{tag}.json")


def tag_of(cells, m2, m3):
    return f"mp_{cells:g}S_{m2}_{m3}"


def opt(tag, stage, *args, **kw):
    return Job(tag, stage, [*RES, "optimise", "--tag", tag, *args, *PATHS], run_done(tag), **kw)


def part(name, stage, p, *args, done, **kw):
    return Job(name, stage, [*RES, p, *args, *PATHS], done, **kw)


def common_done(*labels):
    def f():
        d = jread(R.UQ_SETS["final"]["out"] / "common_nominal.json")
        return bool(d) and all(k in d and d[k].get("git") == commit() for k in labels)
    return f


def ms_starts(best, failed):
    """The multistart optima for the two other robust starts: the converged pair farthest apart in Kv (the flat
    direction), None until every multistart has ended."""
    tags = [f"{best}_ms{i}" for i in range(10)]
    if not all(run_done(t)() or t in failed for t in tags):
        return None
    ok = [d for d in (jread(R.RUNS / f"{t}.json") for t in tags) if d and d.get("ok") and "score" in d]
    ok.sort(key=lambda d: d["dv"]["Kv"])
    return [d["tag"] for d in ([ok[0], ok[-1]] if len(ok) >= 2 else ok)]


def before(hhmm):
    return time.strftime("%H:%M") < hhmm or time.strftime("%H") >= "12"


def jobs(failed):
    """The job list from the current state; a job appears once what it needs exists; the order is the priority."""
    term = lambda name, f: name in failed or f()  # noqa: E731
    out = [Job("continuity", 1, [*RES, "continuity", *PATHS], run_done("continuity"), mem_gb=2.0)]
    for m in ("loo", "cal", "gg"):
        out.append(Job(f"validation_{m}", 1, ["studies/validation_2025.py", "woombah", "--mass", m, "--config", WB_CFG,
                                              "--out", OUT_REL], done_json(OUT / f"validation_woombah_{m}.json")))
    for cells, m2, m3 in ENUM:
        out.append(opt(tag_of(cells, m2, m3), 2, "--cells", str(cells), "--m2", m2, "--m3", m3))
    enum_end = all(term(tag_of(*c), run_done(tag_of(*c))) for c in ENUM)
    b = R.best_case() if enum_end else None
    if b is None:
        return out
    best = b["tag"]
    sel = ["--cells", f"{b['cells']:g}", "--m2", b["props"]["M2"], "--m3", b["props"]["M3"]]
    screen_f = R.UQ_SETS["final"]["out"] / "screen.json"
    out.append(part("screen", 9, "screen", "--case", best, done=done_json(screen_f)))
    for i in range(10):
        out.append(part(f"ms{i}", 3, "multistart", "--case", best, "--index", str(i), done=run_done(f"{best}_ms{i}")))
    bottles = [f"{best}_bottle{x}" for x in ("2L", "0.5L")]
    for x, t in zip(("2L", "0.5L"), bottles):
        out.append(opt(t, 4, *sel, "--start", best, "--bottle", x))
    for m in ("direct", "completed"):
        out.append(Job(f"ac1_{m}", 5, ["studies/aircrafter1_2025.py", m, "--config", CFG, "--out", OUT_REL],
                       run_done(f"ac1_{m}")))
    rdo_tags = []
    if done_json(screen_f)():
        out.append(part("rdof_best", 6, "rdo", "--tag", "rdof_best", "--case", best, done=run_done("rdof_best"), mem_gb=2.5))
        rdo_tags.append("rdof_best")
        starts = ms_starts(best, {t.replace("ms", f"{best}_ms") for t in failed})
        if starts is not None:
            for lab, st in zip(("ms_a", "ms_b"), starts):
                out.append(part(f"rdof_{lab}", 6, "rdo", "--tag", f"rdof_{lab}", "--case", best, "--start", st,
                                done=run_done(f"rdof_{lab}"), mem_gb=2.5))
                rdo_tags.append(f"rdof_{lab}")
    pre = ["woombah", "ac1_direct", "ac1_completed", "multipoint", "multipoint_2L"]
    ready = all(term(n, f) for n, f in [("validation_cal", done_json(OUT / "validation_woombah_cal.json")),
                                        ("ac1_direct", run_done("ac1_direct")), ("ac1_completed", run_done("ac1_completed")),
                                        (bottles[0], run_done(bottles[0]))])
    if not ready:
        return out
    if not (OUT / "validation_woombah_cal.json").exists():
        pre.remove("woombah")
    out.append(part("common_pre", 5, "common", "--labels", *pre, done=common_done(*pre)))
    if not ("rdof_best" in rdo_tags and len(rdo_tags) >= 2 and all(term(t, run_done(t)) for t in rdo_tags)):
        return out
    if R.best_rdo() is None:
        return out
    out.append(part("common_rdo", 5, "common", "--labels", "rdo", done=common_done("rdo")))
    q = R.UQ_SETS["final"]["out"]
    for lab in MC_ORDER:
        out.append(part(f"mc_{lab}", 7, "mc", "--labels", lab, "--workers", "3", done=done_json(q / f"mc_{lab}.json"), weight=3,
                        mem_gb=3.5))
    if done_json(q / "mc_rdo.json")():
        out.append(part("propagation", 8, "propagation", "--workers", "3", done=done_json(q / "propagation.json"), weight=3,
                        mem_gb=3.0))
    bk = [run_done(f"{best}_M2only"), run_done(f"{best}_M3only")]
    out.append(part("bookends", 9, "bookends", "--case", best, done=lambda: all(f() for f in bk)))
    for f2, f3 in ((2, 0.5), (0.5, 2), (1, 2), (2, 1), (1, 0.5), (0.5, 1)):
        out.append(part(f"norm_{f2:g}_{f3:g}", 9, "normalisation", "--case", best, "--f2", str(f2), "--f3", str(f3),
                        done=run_done(f"{best}_norm_{f2:g}_{f3:g}")))
    stage9 = [j for j in out if j.stage <= 9]
    if all(term(j.name, j.done) for j in stage9):
        for i in range(3):
            out.append(Job(f"timing_det_{i}", 10, [*RES, "optimise", "--tag", f"timing_det_{i}", *sel, *PATHS],
                           run_done(f"timing_det_{i}"), weight=3, env=SINGLE))
        out.append(part("timing_rdo", 10, "rdo", "--tag", "timing_rdo", "--case", best, done=run_done("timing_rdo"), weight=3,
                        mem_gb=2.5, env=SINGLE))
    return out


def mem_available_gb():
    for line in Path("/proc/meminfo").read_text().splitlines():
        if line.startswith("MemAvailable"):
            return int(line.split()[1]) / 1e6
    return 0.0


def clog(msg):
    line = f"{time.strftime('%Y-%m-%d %H:%M:%S')} {msg}"
    print(line, flush=True)
    with CLOG.open("a") as f:
        f.write(line + "\n")


def write_status(state, running):
    js = jobs(set(state["failed"]))
    rows = [f"Updated {time.strftime('%F %T')}, run commit {commit()}. Running: {', '.join(running) or 'none'}. "
            f"Failed: {', '.join(state['failed']) or 'none'}. Skipped: {', '.join(state.get('skipped', [])) or 'none'}.", ""]
    for s in range(1, 11):
        sj = [j for j in js if j.stage == s]
        rows.append(f"- stage {s}: {sum(j.done() for j in sj)}/{len(sj)} known jobs done")
    (OUT / "campaign_status.md").write_text("\n".join(rows) + "\n")


def run():
    freeze = commit()
    if not freeze or R.git_hash() != freeze:
        raise SystemExit(f"HEAD is {R.git_hash()}, the run commit is {freeze!r}: `freeze` on the clean tree first")
    LOGS.mkdir(parents=True, exist_ok=True)
    state = jread(STATE) or dict(attempts={}, failed=[], skipped=[])
    running, mem_logged, idle_since = {}, set(), None
    env0 = dict(os.environ, OMP_NUM_THREADS="3", AIRCRAFTER2_APC_DIR="/home/lucas/University/THESIS/PERFILES2",
                PYTHONPATH=str(ROOT))
    clog(f"campaign start, run commit {freeze}")
    stop_logged = False
    while True:
        for name, (proc, job, t0, logf) in list(running.items()):
            if proc.poll() is None:
                continue
            del running[name]
            logf.close()
            good = proc.returncode == 0 and job.done()
            clog(f"end   {name}: stage {job.stage}, exit {proc.returncode}, {time.time() - t0:.0f} s, done={good}")
            state["attempts"][name] = state["attempts"].get(name, 0) + 1
            if not good and state["attempts"][name] >= 2:
                state["failed"].append(name)
                clog(f"FAILED {name} after {state['attempts'][name]} attempts: logged, skipped")
            STATE.write_text(json.dumps(state, indent=1))
            write_status(state, running)
        kill = not before(STOP_KILL)
        if kill and running:
            for name, (proc, job, _, _) in running.items():
                clog(f"deadline: stopping {name}")
                proc.terminate()
                state["skipped"].append(name)
            time.sleep(20)
            continue
        failed = set(state["failed"])
        todo = [j for j in jobs(failed) if j.name not in failed and not j.done()]
        if not before(STOP_START):
            if not stop_logged:
                clog(f"past {STOP_START}: no new jobs. Not started: {', '.join(j.name for j in todo if j.name not in running)}")
                stop_logged = True
            todo = [j for j in todo if j.name in running]
        timing_late = not before(TIMING_BEFORE)
        if timing_late:
            todo = [j for j in todo if j.stage != 10 or j.name in running]
        waiting = [j for j in todo if j.name not in running]
        used = sum(r[1].weight for r in running.values())
        started = False
        for j in waiting:
            if used + j.weight > SLOTS:
                if j.weight == SLOTS:  # a Monte Carlo waits for its slots instead of being overtaken for ever
                    break
                continue
            if mem_available_gb() < j.mem_gb + MEM_FREE_GB and not (not running and idle_since and time.time() - idle_since > 600):
                if j.name not in mem_logged:
                    clog(f"wait  {j.name}: {mem_available_gb():.1f} GB available, needs {j.mem_gb:.1f} + {MEM_FREE_GB}")
                    mem_logged.add(j.name)
                continue
            logf = (LOGS / f"{j.name}.log").open("a")
            argv = ["nice", "-n", "10", PY, *j.argv]
            logf.write(f"\n--- {time.strftime('%F %T')} {' '.join(argv)}\n")
            logf.flush()
            proc = subprocess.Popen(argv, cwd=ROOT, env={**env0, **j.env}, stdout=logf, stderr=subprocess.STDOUT)
            running[j.name] = (proc, j, time.time(), logf)
            used += j.weight
            started = True
            clog(f"start {j.name} (stage {j.stage}, weight {j.weight}, {mem_available_gb():.1f} GB available)")
        if running or started:
            idle_since = None
        elif not waiting:
            clog("campaign finished: nothing left")
            write_status(state, running)
            return
        elif idle_since is None:
            idle_since = time.time()
        write_status(state, running)
        time.sleep(15)


def status():
    state = jread(STATE) or dict(attempts={}, failed=[], skipped=[])
    js = jobs(set(state["failed"]))
    done = [j.name for j in js if j.done()]
    print(f"run commit {commit()}\ndone {len(done)}: {done}\nleft: {[j.name for j in js if j.name not in done]}\nfailed: {state['failed']}")


def freeze():
    g = R.git_hash()
    if g.endswith("-dirty"):
        raise SystemExit("commit first: the tracked tree is dirty")
    OUT.mkdir(parents=True, exist_ok=True)
    COMMIT_FILE.write_text(g + "\n")
    print("run commit", g)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("part", choices=["freeze", "run", "status"])
    {"freeze": freeze, "run": run, "status": status}[ap.parse_args().part]()
