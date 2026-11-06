"""Rerun of the 2025-26 Monte Carlos in batches, each on fresh workers, with the frozen model and study code.

The first Monte Carlos (fr_mc_*, moved to outputs/results_2026/superseded_worker_failure/) lost 17-30 % of their samples to
an analysis failure that has nothing to do with the sample: after about 250-280 evaluations, a worker's problem answers
every later sample with "singular Jacobian in 'points.M3.mda'", and the retry from the nominal state fails too
(docs/development/RESULTS_LOG.md, Decision 13). `aircrafter2/robustness/sampling.evaluate` keeps one pool for all the samples, and model code
can't change after the freeze, so this wraps it: at most BATCH samples per pool, about 128 per worker, then results_2026.mc
runs as frozen. Same rows, same seed, same outputs.

Usage: results_2026_mc_batched.py run [--workers 3]
       results_2026_mc_batched.py check   (compares with the superseded runs, tests that failures depend on the sample)
"""

import argparse
import json
import shutil
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))

import results_2026 as R  # noqa: E402

from aircrafter2.robustness import sampling  # noqa: E402

BATCH = 384
OLD = R.OUT / "superseded_worker_failure"
IN = ["mdo", "rdo", "mdo_wind", "ac1_completed", "ac1_direct", "humpback", "humpback_loo"]
OUT_ = ["mdo", "rdo", "mdo_wind", "humpback"]
_frozen = sampling.evaluate


def evaluate_batched(cfg, dv, rows, outputs, workers=5, chunksize=None):
    """sampling.evaluate over consecutive batches of the rows, so that no worker reaches the failure's count."""
    parts = [_frozen(cfg, dv, rows[i:i + BATCH], outputs, workers, chunksize) for i in range(0, len(rows), BATCH)]
    shift = lambda key: sorted(i * BATCH + j for i, p in enumerate(parts) for j in p[key])  # noqa: E731
    return dict(values=np.vstack([p["values"] for p in parts]), failed=shift("failed"), lap_invalid=shift("lap_invalid"),
                retried=shift("retried"), wall=sum(p["wall"] for p in parts), workers=workers)


def run(workers):
    """Every Monte Carlo that hasn't been redone; the superseded files are kept."""
    sampling.evaluate = evaluate_batched
    runs = [(x, "in") for x in IN] + [(x, "out") for x in OUT_]
    OLD.mkdir(parents=True, exist_ok=True)
    for label, which in runs:
        tag = f"fr_mc_{which}_{label}"
        js, npz = R.RUNS / f"{tag}.json", R.MC_DIR / f"{tag}.npz"
        if (OLD / f"{tag}.json").exists() and js.exists():
            print(f"{tag}: done", flush=True)
            continue
        if js.exists():
            shutil.move(js, OLD / js.name)
            shutil.move(npz, OLD / npz.name)
        R.mc(label, which, workers)


def check():
    """The new failures against the old ones: blocks of consecutive samples, and agreement on the samples both ran."""
    for which, labels in (("in", IN), ("out", OUT_)):
        for label in labels:
            tag = f"fr_mc_{which}_{label}"
            new, old = np.load(R.MC_DIR / f"{tag}.npz"), np.load(OLD / f"{tag}.npz")
            Vn, Vo = new["values"], old["values"]
            fn, fo = np.isnan(Vn).any(axis=1), np.isnan(Vo).any(axis=1)
            both = ~fn & ~fo
            d = np.abs(Vn[both, 0] - Vo[both, 0])
            print(f"{tag}: failed {fo.sum()} -> {fn.sum()}, longest failed run {_longest(fn)}, both ran {both.sum()}, "
                  f"max |dF| {d.max():.2e}, new mean {Vn[~fn, 0].mean():.4f} sd {Vn[~fn, 0].std(ddof=1):.4f}, "
                  f"old mean {Vo[~fo, 0].mean():.4f} sd {Vo[~fo, 0].std(ddof=1):.4f}", flush=True)
            if fn.any():
                u = new["u"]
                print("   failed by quartile of each parameter:",
                      json.dumps([[round(float(fn[np.minimum((u[:, j] * 4).astype(int), 3) == q].mean()), 2) for q in range(4)]
                                  for j in range(u.shape[1])]), flush=True)


def _longest(f):
    """The longest run of consecutive True."""
    best = run_ = 0
    for x in f:
        run_ = run_ + 1 if x else 0
        best = max(best, run_)
    return best


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("part", choices=["run", "check"])
    ap.add_argument("--workers", type=int, default=3)
    a = ap.parse_args()
    run(a.workers) if a.part == "run" else check()
