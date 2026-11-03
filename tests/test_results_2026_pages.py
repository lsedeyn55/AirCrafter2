"""The page helpers added after the freeze: the batched Monte Carlo wrapper and the recommendations."""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "studies"))

import results_2026_mc_batched as B  # noqa: E402
import results_2026_recommendations as RR  # noqa: E402


def test_batches_keep_the_order_and_shift_the_failures(monkeypatch):
    calls = []

    def fake(cfg, dv, rows, outputs, workers, chunksize):
        calls.append(len(rows))
        first = rows[0]["i"]
        failed = [j for j, r in enumerate(rows) if r["i"] % 500 == 7]
        return dict(values=np.array([[r["i"]] for r in rows], float), failed=failed, lap_invalid=failed[:1], retried=[], wall=1.0, workers=workers, first=first)

    monkeypatch.setattr(B, "_frozen", fake)
    out = B.evaluate_batched(None, {}, [{"i": i} for i in range(1024)], ["x"], 3)
    assert calls == [384, 384, 256]
    assert (out["values"][:, 0] == np.arange(1024)).all()
    assert out["failed"] == [7, 507, 1007]
    assert out["lap_invalid"] == [7, 507, 1007]
    assert out["wall"] == 3.0


def test_longest_run():
    assert B._longest(np.array([0, 1, 1, 0, 1, 1, 1, 0], bool)) == 3
    assert B._longest(np.zeros(4, bool)) == 0


def test_recommendations_leave_out_what_has_no_evidence():
    class P:
        nominal = staticmethod(lambda R, lab: None)
        decomposition = staticmethod(lambda R: None)
        multistart_summary = staticmethod(lambda R: None)

    R = dict(enumeration=[], normalisation={}, designs=dict(rdo={}), validation=None)
    assert RR.build(R, P) == []
