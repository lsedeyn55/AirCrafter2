"""Take-off in the mission time: re-analysis of the published 2024-25 designs (docs/studies/takeoff_study.md,
docs/TAKEOFF_COURSE_PLAN.md). Analysis only: nothing is re-optimised, no published result is touched.

    .venv/bin/python studies/takeoff_study.py run        # every design, take-off off and on (+ CL_ground, throttle)
    .venv/bin/python studies/takeoff_study.py woombah    # Woombah as built (validation_2025.md section 3), off and on
    .venv/bin/python studies/takeoff_study.py wind       # each design's wind limit, take-off off and on
    .venv/bin/python studies/takeoff_study.py robust     # the RDO's UDR statistics (9 nodes) at each design, off and on
    .venv/bin/python studies/takeoff_study.py resolution # lap-1 time against 16x the straight steps, in wind
    .venv/bin/python studies/takeoff_study.py report     # prints the tables (the docs are written from the JSON)

Designs: the six of results_2025.md section 2 (woombah, ac1_direct, ac1_completed, multipoint, multipoint_2L, rdo =
the reduced-set RDO, rdor_ms0) and the bottle optima of section 1b (0.5 L, 1 L = multipoint, 2 L = multipoint_2L), at
their published design variables. Take-off on = configs/dbf2025_multipoint_takeoff.yaml's course block.

Per design and mission point: t_takeoff, s_ground_roll, V_LOF, E_takeoff, the take-off margins, the 3-lap time, laps,
and the scores, take-off off and on; the Eq. 3.3 ground roll implied by the design's own static thrust
(static_thrust_calculator.m solved for s_g: V_R = 1.2 V_stall, drag at C_L,to and T = T(0) through the roll); the
ground roll at CL_ground 0, 0.5 and 1 (sensitivity to the PLACEHOLDER); and, as an estimate for the take-off-throttle
question only, the ground roll if the throttle were 1.0 (the whole point re-analysed at full throttle: only its
roll is read).

Woombah as built: configs/woombah_2025.yaml in validation_2025.md's three cases (full throttle; the logged-current
throttles; those and the logged take-off masses through the m_ballast hook), with the throttles and ballast read from
outputs/results/validation_woombah_cal.json (validation_2025.py woombah --mass cal: the calibrated factors the config
resolves to, as results_2025's common evaluation; not re-solved), take-off off and on.

Wind limit: per design and point, the largest steady wind (lap.wind, the robustness input: headwind on the head
straights) for which the lap is still valid (no AnalysisError), by stepping the wind in 0.01 m/s on the lap component
alone at the converged nominal point (wind only enters the lap model). No Monte Carlo is rerun.

Robust statistics: results_2025.robust_eval's UDR moments (the reduced set's screened parameters and nodes, k and beta
of configs/dbf2025_multipoint_rdo.yaml) at each design, take-off off (reproduces results_2025.md section 7) and on.
Nodes only: the Monte Carlo is not rerun.

Step resolution: M3's lap-1 time at the default n_steps_straight (40) against 640, at winds 3.5-8.4 m/s, off and on,
for multipoint, multipoint_2L and ac1_completed (the designs whose M3 wind limit is near the Monte Carlo's 7 m/s cap).

Output: outputs/takeoff/reanalysis.json, woombah.json, wind_limit.json, robust_eval.json, resolution.json.
"""

import os

os.environ.setdefault("OMP_NUM_THREADS", "2")

import argparse  # noqa: E402
import json  # noqa: E402
import sys  # noqa: E402
import warnings  # noqa: E402
from multiprocessing import get_context  # noqa: E402
from pathlib import Path  # noqa: E402

import numpy as np  # noqa: E402

warnings.filterwarnings("ignore")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "studies"))
OUT = ROOT / "outputs" / "takeoff"
TAKEOFF_CFG = ROOT / "configs" / "dbf2025_multipoint_takeoff.yaml"
FT = 0.3048
PT_OUT = ["t_takeoff", "s_ground_roll", "V_lof", "E_takeoff", "takeoff_accel_margin", "takeoff_straight_margin",
          "Lap1time", "otherLapTime", "ThreeLapTime", "laps_frac", "energy_margin", "energy_3laps", "Vstall", "v",
          "m_total", "S", "T_static", "thrust_margin_static", "turn_speed_margin", "stall_margin"]
SCORES = ["score", "M1", "M2", "M3", "flight_score"]


def designs():
    """[(label, MultipointConfig, dv)]: the six of section 2, then the 0.5 L bottle optimum."""
    import results_2025 as R

    R.use_set("reduced")
    out = R.mc_designs()
    b = R.best_case()
    d = R.load(f"{b['tag']}_bottle0.5L")
    out.append(("bottle_0.5L", R._mcfg_for(d), d["dv"]))
    return out


def _takeoff_block():
    from aircrafter2.problems.multipoint import load_multipoint_config

    return load_multipoint_config(TAKEOFF_CFG).base.course


def _v(p, k):
    return float(np.ravel(p.get_val(k))[0])


def eq33_roll(p, pt, mcfg):
    """static_thrust_calculator.m solved for the ground roll with the design's own T(0):
    s_g = m V_R^2 / (2 (T0 - q(V_R) S C_D,to - mu W)), V_R = 1.2 sqrt(W / (q S C_L,to)) (Eq. 3.3)."""
    g = lambda k: _v(p, f"points.{pt}.lap.{k}")  # noqa: E731
    m, S, rho, grav, CLto = g("m"), g("S"), g("rho"), g("g"), g("CL_to")
    CD0, CD1, k, mu = g("CD0"), g("CD1"), g("k"), g("mu_roll")
    T0 = float(np.ravel(p.get_val(f"points.{pt}.lap.T_coeffs"))[0]) * g("thrust_factor")
    W = m * grav
    VR = 1.2 * np.sqrt(W / (0.5 * rho * S * CLto))
    D = 0.5 * rho * VR**2 * S * (CD0 + CD1 * CLto + k * CLto**2)
    F = T0 - D - mu * W
    return dict(s_eq33=m * VR**2 / (2.0 * F) if F > 0 else float("inf"), V_R=VR, T0=T0, F_net_eq33=F,
                t_eq33=m * VR / F if F > 0 else float("inf"))  # constant net force: t = m V_R / F


def evaluate(job):
    """One design in one setting: job = (label, mcfg, dv, setting, overrides)."""
    from openmdao.core.analysis_error import AnalysisError

    from aircrafter2.problems.multipoint import build_multipoint_problem, m1_check

    label, mcfg, dv, setting, over = job
    mcfg = mcfg.model_copy(deep=True)
    if setting != "off":
        mcfg.base.course = _takeoff_block()
        for k, x in over.get("takeoff", {}).items():
            setattr(mcfg.base.course.takeoff, k, x)
    p = build_multipoint_problem(mcfg, driver=False, mda_retry=True)
    for k, x in {**dv, **over.get("dv", {})}.items():
        p.set_val(k, x)
    try:
        p.run_model()
    except AnalysisError as exc:
        return label, setting, dict(ok=False, error=str(exc)[:300])
    res = dict(ok=True, score={k: _v(p, k) for k in SCORES})
    for pt in ("M2", "M3"):
        res[pt] = {k: _v(p, f"points.{pt}.{k}") for k in PT_OUT}
        res[pt].update(eq33_roll(p, pt, mcfg))
        res[pt]["throttle"] = _v(p, f"throttle_{pt}")
        res[pt]["legs"] = p.model._get_subsystem(f"points.{pt}.lap").leg_breakdown() if setting == "on" else None
    if setting in ("off", "on"):
        m1 = m1_check(mcfg, p)
        res["M1"] = {k: m1.get(k) for k in ("passed", "laps_frac", "ThreeLapTime", "error")}
    return label, setting, res


def run(workers=4):
    OUT.mkdir(parents=True, exist_ok=True)
    path = OUT / "reanalysis.json"
    done = json.loads(path.read_text()) if path.exists() else {}
    jobs = []
    for label, mcfg, dv in designs():
        settings = [("off", {}), ("on", {}), ("CLg0", {"takeoff": {"CL_ground": 0.0}}),
                    ("CLg0.5", {"takeoff": {"CL_ground": 0.5}}), ("CLg1", {"takeoff": {"CL_ground": 1.0}}),
                    ("full_throttle", {"dv": {"throttle_M2": 1.0, "throttle_M3": 1.0}})]
        jobs += [(label, mcfg, dv, s, o) for s, o in settings if s not in done.get(label, {})]
    print(f"{len(jobs)} jobs", flush=True)
    with get_context("forkserver").Pool(workers) as pool:
        for label, setting, res in pool.imap_unordered(evaluate, jobs):
            done.setdefault(label, {})[setting] = res
            path.write_text(json.dumps(done, indent=1, default=float))
            print(label, setting, res.get("score", {}).get("flight_score", res.get("error")), flush=True)


def _wind_job(job):
    from openmdao.core.analysis_error import AnalysisError

    from aircrafter2.problems.multipoint import build_multipoint_problem

    label, mcfg, dv, setting = job
    mcfg = mcfg.model_copy(deep=True)
    if setting == "on":
        mcfg.base.course = _takeoff_block()
    p = build_multipoint_problem(mcfg, driver=False, mda_retry=True)
    for k, x in dv.items():
        p.set_val(k, x)
    p.run_model()
    res = {}
    for pt in ("M2", "M3"):
        comp = p.model._get_subsystem(f"points.{pt}.lap")
        base = {k: np.array(comp._inputs[k], dtype=float) for k in comp._inputs}

        def valid(w):
            x = dict(base, wind=np.array([w]))
            try:
                comp.compute(x, {})
            except AnalysisError:
                return False
            return True

        w, step = 0.0, 0.5
        while step >= 0.01 - 1e-12:  # coarse to fine: largest valid wind on a 0.01 m/s grid
            while w + step < 40.0 and valid(w + step):
                w += step
            step /= 5.0 if step > 0.02 else 2.0
        res[pt] = dict(wind_limit=w, Vstall=_v(p, f"points.{pt}.lap.Vstall"),
                       V_lof=_v(p, f"points.{pt}.lap.V_lof") if setting == "on" else None)
    return label, setting, res


def wind(workers=4):
    jobs = [(label, mcfg, dv, s) for label, mcfg, dv in designs() for s in ("off", "on")]
    out = {}
    with get_context("forkserver").Pool(workers) as pool:
        for label, setting, res in pool.imap_unordered(_wind_job, jobs):
            out.setdefault(label, {})[setting] = res
            print(label, setting, res, flush=True)
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "wind_limit.json").write_text(json.dumps(out, indent=1, default=float))


def robust(workers=5):
    import results_2025 as R
    from aircrafter2.problems.multipoint import build_multipoint_problem, multipoint_constraints
    from aircrafter2.robustness.quadrature import make_rule
    from aircrafter2.robustness.rdo_multipoint import resolved_dists_mp, robust_names
    from aircrafter2.robustness.sampling import evaluate

    R.use_set("reduced")
    sc = json.loads((R.UQ["out"] / "screen.json").read_text())
    out = {}
    for label, mcfg0, dv in designs():
        for setting in ("off", "on"):
            mcfg = mcfg0.model_copy(deep=True)
            if setting == "on":
                mcfg.base.course = _takeoff_block()
            mcfg.base.robust.active = sc["selected"]
            rob = mcfg.base.robust
            p = build_multipoint_problem(mcfg, driver=False)
            dists, act = resolved_dists_mp(mcfg, lambda n: _v(p, f"points.M2.{n}"))
            rule = make_rule(rob, act)
            base = {d.name: d.mean for d in dists}
            rows = [dict(base, **{d.name: float(x) for d, x in zip(act, r)}) for r in rule.nodes]
            cons = dict(multipoint_constraints(mcfg))
            rn = robust_names(mcfg, extra=("points.M2.laps_frac",))
            ev = evaluate(mcfg, dv, rows, ["flight_score"] + rn, workers)
            res = dict(N=rule.n, failed=len(ev["failed"]), robust_violated=[], margins={})
            if not ev["failed"]:
                mu, var = rule.moments(ev["values"][:, 0])
                sig = float(np.sqrt(max(var, 0.0)))
                res.update(mu=mu, sigma=sig, robust=mu - rob.k * sig)
                for j, n in enumerate(rn, start=1):
                    m_, var_ = rule.moments(ev["values"][:, j])
                    s_ = float(np.sqrt(max(var_, 0.0)))
                    kw = cons[n]
                    tol = 1e-6 * max(abs(kw.get("lower") or kw.get("upper") or 0.0), kw.get("ref") or 0.0, 1.0)
                    res["margins"][n] = dict(mu=m_, sigma=s_)
                    if (kw.get("lower") is not None and m_ - rob.beta * s_ < kw["lower"] - tol) or \
                            (kw.get("upper") is not None and m_ + rob.beta * s_ > kw["upper"] + tol):
                        res["robust_violated"].append(n)
            out.setdefault(label, {})[setting] = res
            print(label, setting, {k: res.get(k) for k in ("mu", "sigma", "robust", "failed", "robust_violated")},
                  flush=True)
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "robust_eval.json").write_text(json.dumps(dict(selected=sc["selected"], results=out), indent=1,
                                                     default=float))


RES_WINDS = (3.5, 5.0, 6.0, 7.0, 7.5, 8.0, 8.4)


def _resolution_job(job):
    from aircrafter2.problems.multipoint import build_multipoint_problem

    label, mcfg, dv, setting = job
    mcfg = mcfg.model_copy(deep=True)
    if setting == "on":
        mcfg.base.course = _takeoff_block()
    p = build_multipoint_problem(mcfg, driver=False, mda_retry=True)
    for k, x in dv.items():
        p.set_val(k, x)
    p.run_model()
    comp = p.model._get_subsystem("points.M3.lap")
    base = {k: np.array(comp._inputs[k], dtype=float) for k in comp._inputs}
    rows = []
    for w in RES_WINDS:
        t = {}
        for n in (40, 640):
            comp.options["n_steps_straight"] = n
            t[n] = float(np.real(np.ravel(comp._model(dict(base, wind=np.array([w])))["Lap1time"])[0]))
        comp.options["n_steps_straight"] = 40
        rows.append(dict(wind=w, lap1_40=t[40], lap1_640=t[640], rel_err=t[40] / t[640] - 1.0))
    return label, setting, rows


def resolution(workers=4):
    jobs = [(label, mcfg, dv, s) for label, mcfg, dv in designs()
            if label in ("multipoint", "multipoint_2L", "ac1_completed") for s in ("off", "on")]
    out = {}
    with get_context("forkserver").Pool(workers) as pool:
        for label, setting, rows in pool.imap_unordered(_resolution_job, jobs):
            out.setdefault(label, {})[setting] = rows
            print(label, setting, [(r["wind"], round(100 * r["rel_err"], 2)) for r in rows], flush=True)
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "resolution.json").write_text(json.dumps(out, indent=1, default=float))


def _woombah_job(job):
    from openmdao.core.analysis_error import AnalysisError

    from aircrafter2.problems.multipoint import build_multipoint_problem, load_multipoint_config

    case, setting, th2, th3, ballast = job
    mcfg = load_multipoint_config(ROOT / "configs" / "woombah_2025.yaml")
    if setting == "on":
        mcfg.base.course = _takeoff_block()
    p = build_multipoint_problem(mcfg, driver=False)
    p.set_val("throttle_M2", th2)
    p.set_val("throttle_M3", th3)
    for pt, kg in ballast.items():
        p.set_val(f"points.{pt}.m_ballast", kg)
    try:
        p.run_model()
    except AnalysisError as exc:
        return case, setting, dict(ok=False, error=str(exc)[:300])
    res = dict(ok=True, score={k: _v(p, k) for k in SCORES})
    for pt in ("M2", "M3"):
        res[pt] = {k: _v(p, f"points.{pt}.{k}") for k in PT_OUT}
        res[pt]["legs"] = p.model._get_subsystem(f"points.{pt}.lap").leg_breakdown() if setting == "on" else None
    return case, setting, res


def woombah(workers=4):
    val = json.loads((ROOT / "outputs" / "results" / "validation_woombah_cal.json").read_text())
    lt = val["logged_throttle"]
    th = (lt["M2"]["throttle"], lt["M3"]["throttle"])
    ballast = val["cases"]["logged_current_and_mass"]["ballast_kg"]
    cases = dict(full_throttle=(1.0, 1.0, {}), logged_current=(*th, {}), logged_current_and_mass=(*th, ballast))
    jobs = [(c, s, *a) for c, a in cases.items() for s in ("off", "on")]
    out = {}
    with get_context("forkserver").Pool(workers) as pool:
        for case, setting, res in pool.imap_unordered(_woombah_job, jobs):
            out.setdefault(case, {})[setting] = res
            print(case, setting, res.get("M2", {}).get("ThreeLapTime", res.get("error")), flush=True)
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "woombah.json").write_text(json.dumps(out, indent=1, default=float))


def report():
    d = json.loads((OUT / "reanalysis.json").read_text())
    print("| design | pt | throttle | t_TO (s) | s_roll (ft) | V_LOF | s_Eq3.3 (ft) | V_R | dt3 (s) | dlaps | "
          "s_roll CLg 0 / 0.5 / 1 (ft) | t_TO full thr. (s) | s_roll full thr. (ft) |")
    for label, r in d.items():
        if not (r.get("off", {}).get("ok") and r.get("on", {}).get("ok")):
            print(label, {s: x.get("error") for s, x in r.items() if not x.get("ok")})
            continue
        for pt in ("M2", "M3"):
            on, off = r["on"][pt], r["off"][pt]
            c0, c5, c1, ft = (r.get(s, {}).get(pt, {}) for s in ("CLg0", "CLg0.5", "CLg1", "full_throttle"))
            print(f"| {label} | {pt} | {on['throttle']:.3f} | {on['t_takeoff']:.2f} | {on['s_ground_roll'] / FT:.0f} | "
                  f"{on['V_lof']:.2f} | {on['s_eq33'] / FT:.0f} | {on['V_R']:.2f} | "
                  f"{on['ThreeLapTime'] - off['ThreeLapTime']:+.2f} | {on['laps_frac'] - off['laps_frac']:+.3f} | "
                  f"{c0.get("s_ground_roll", np.nan) / FT:.0f} / {c5.get("s_ground_roll", np.nan) / FT:.0f} / {c1.get('s_ground_roll', np.nan) / FT:.0f} | "
                  f"{ft.get('t_takeoff', np.nan):.2f} | {ft.get('s_ground_roll', np.nan) / FT:.0f} |")
        so, sn = r["off"]["score"], r["on"]["score"]
        print(f"|   {label} scores | F {so['flight_score']:.4f} -> {sn['flight_score']:.4f} "
              f"(M2 {sn['M2'] - so['M2']:+.4f}, M3 {sn['M3'] - so['M3']:+.4f}) |")


TAKEOFF_RDO_CFG = ROOT / "configs" / "dbf2025_multipoint_rdo_takeoff.yaml"


def reopt(which):
    """Re-optimise with take-off on (outputs/results/runs/to_*.json; the published runs are untouched).
    best_warm / best_cold: the best deterministic case from its published optimum / the config's initial point;
    rdo: the reduced-set RDO of the same case from the published RDO design."""
    import results_2025 as R

    R.use_set("reduced")
    b = R.best_case()
    if which in ("best_warm", "best_cold"):
        R.optimise(f"to_{b['tag']}_{which.split('_')[1]}", b["cells"], b["props"]["M2"], b["props"]["M3"],
                   start=b["tag"] if which == "best_warm" else None, cfg=TAKEOFF_CFG, bottle=b.get("bottle"))
        return
    r = R.best_rdo()
    R.UQ["cfg"] = TAKEOFF_RDO_CFG  # same screen.json (all four inputs), take-off config
    R.rdo(f"to_rdor_{r['tag']}", case=r["case"], x0=r["dv"])


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["run", "woombah", "wind", "robust", "resolution", "report", "reopt"])
    ap.add_argument("--which", choices=["best_warm", "best_cold", "rdo"], default="best_warm")
    ap.add_argument("--workers", type=int, default=4)
    a = ap.parse_args()
    {"run": lambda: run(a.workers), "woombah": lambda: woombah(a.workers), "wind": lambda: wind(a.workers), "robust": lambda: robust(a.workers), "resolution": lambda: resolution(a.workers), "report": report, "reopt": lambda: reopt(a.which)}[a.cmd]()
