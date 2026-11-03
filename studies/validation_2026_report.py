"""docs/studies/results_2026/validation_2026.{md,json}: Project Humpback as built, from the runs of
studies/validation_2026.py (outputs/results_2026/validation_*.json) and AirCrafter 1.0's '2026' run
(reference/aircrafter1/runs/ac1_2026_r1.json). Every number is read from those files.

    .venv/bin/python studies/validation_2026.py report
"""

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "studies"))
from validation_2026 import (DOCS, IN, LB, MASS_VARIANTS, REPORT_VARIANTS, TABLE1, TABLE36, TABLE42,  # noqa: E402
                             TABLE43, git_hash, humpback_config, load)

FT = 0.3048
CRIT_MASS, CRIT_FLIGHT = 0.30, 0.10  # thesis §3.7: structure within the LOO spread; flight within the report's range or 10 %
AC1 = ROOT / "reference" / "aircrafter1" / "runs" / "ac1_2026_r1.json"


def _m1(m1):
    """The M1-check cell: pass / fail with the violated constraints; SM above SM_max is reported, not failed."""
    cell = "pass" if m1["passed"] else "fail: " + ", ".join(m1.get("violated") or [])
    return cell + (" (SM above SM_max: reported, not failed)" if m1["passed"] and m1.get("SM_high") else "")


def _f(x, nd=2):
    if x is None:
        return "–"
    if isinstance(x, str):
        return x
    return f"{x:.{nd}f}"


def _pct(a, b):
    return f"{100 * (a / b - 1):+.1f} %" if a is not None and b else "–"


def _table(header, rows):
    s = "| " + " | ".join(header) + " |\n|" + "|".join("---" if i == 0 else "---:" for i in range(len(header))) + "|\n"
    return s + "".join("| " + " | ".join(str(c) for c in r) + " |\n" for r in rows)


def _model_rows(snap, mcfg):
    """Model masses in [HB] Table 36's rows (lb). Table 36's 'Fuselage' (6.14 lb) also holds the gear, servos,
    receivers, receiver batteries, prop and wiring ([HB] p. 44; calibration doc §1): the model's cargo box, tail boom,
    gear, avionics and prop are summed against it."""
    pr = mcfg.base.propulsion
    m2 = snap["M2"]
    fus = m2["m_cb"] + m2["m_tb"] + m2["m_gear"] + pr.m_avionics + pr.m_prop
    return {"wing": m2["m_wing"], "HT": m2["m_ht"], "VT": m2["m_vt"], "fuselage": fus, "motor": pr.m_motor,
            "ESC": pr.m_esc, "empty": m2["m_empty"], "battery": m2["m_batt"], "M2": m2["m_total"],
            "M3": snap["M3"]["m_total"]}


def collect():
    R = dict(git=git_hash(), variants={}, ac1=None)
    for m in ("loo", "gg", "cal"):
        d = load(f"humpback_{m}")
        if d:
            R["variants"][m] = d
    R["envelope"] = load("envelope")
    R["run_git"] = ", ".join(sorted({d.get("git") for d in list(R["variants"].values()) + [R["envelope"] or {}]} - {None}))
    R["criteria"] = criteria(R)
    if AC1.exists():
        a = json.loads(AC1.read_text())
        R["ac1"] = dict(humpback_point=a.get("humpback_point"), DP=a.get("DP"))
    return R


def criteria(R):
    """Thesis §3.7's agreement criteria on the out-of-sample column: structure mass within +/-30 % (the LOO spread); flight
    performance (speed, laps, energy) within the report's own range or 10 %. Fails are reported, nothing is tuned."""
    if "loo" not in R["variants"]:
        return None
    mcfg = humpback_config()
    V = R["variants"]["loo"]["cases"]
    mod = _model_rows(V["as_built_full_throttle"], mcfg)
    mass = []
    for k, lab in (("wing", "wing (out of sample)"), ("HT", "HT (out of sample)"), ("VT", "VT (out of sample)"), ("fuselage", "'fuselage' row (never in a fit)")):
        err = mod[k] / LB / TABLE36[k] - 1
        mass.append(dict(item=lab, hb=TABLE36[k], model=mod[k] / LB, err=err, ok=abs(err) <= CRIT_MASS))
    struct_hb = sum(TABLE36[k] for k in ("wing", "HT", "VT", "fuselage"))
    struct = sum(mod[k] for k in ("wing", "HT", "VT", "fuselage")) / LB
    mass.append(dict(item="structure, the four rows", hb=struct_hb, model=struct, err=struct / struct_hb - 1, ok=abs(struct / struct_hb - 1) <= CRIT_MASS))
    flight = []
    for name, t in TABLE42.items():
        c = V.get(name, {})
        s = c.get("snapshot", {}).get(t["point"], {})
        if s:
            err = s["otherLapTime"] / t["lap_s"] - 1
            flight.append(dict(item=f"{name.replace('_', ' ')} lap time (s)", hb=t["lap_s"], model=s["otherLapTime"], err=err, ok=abs(err) <= CRIT_FLIGHT))
    b = V["as_built_full_throttle"]
    a2, a3 = TABLE43["M2"]["actual"], TABLE43["M3"]["actual"]
    d2, d3 = TABLE43["M2"]["design"], TABLE43["M3"]["design"]
    for item, x, lo, hi in (("M2 laps, as built", b["M2"]["laps_frac"], min(a2["laps"], d2["laps"]), max(a2["laps"], d2["laps"])),
                            ("M3 laps, as built", b["M3"]["laps_frac"], min(a3["laps"], d3["laps"]), max(a3["laps"], d3["laps"]))):
        flight.append(dict(item=f"{item} ([HB] Tables 1, 43: {lo}-{hi})", hb=f"{lo}-{hi}", model=x, err=None, ok=lo * (1 - CRIT_FLIGHT) <= x <= hi * (1 + CRIT_FLIGHT)))
    for name in ("M2_test2", "M3_test3"):
        c, t = V[name], TABLE42[name]
        s = c["snapshot"][t["point"]]
        flight.append(dict(item=f"{name.replace('_', ' ')} energy per lap at the logged current (Wh)", hb=None, model=s["energy_3laps"] / 3 / 3600.0,
                           err=None, ok=None))
    return dict(mass=mass, flight=flight, mass_pass=sum(1 for x in mass if x["ok"]), mass_n=len(mass), flight_pass=sum(1 for x in flight if x["ok"]),
                flight_n=sum(1 for x in flight if x["ok"] is not None))


def _crit_rows(rows):
    out = []
    for x in rows:
        hb = x["hb"] if isinstance(x["hb"], str) else _f(x["hb"])
        out.append([x["item"], hb, _f(x["model"]), f"{100 * x['err']:+.1f} %" if x["err"] is not None else "–", {True: "pass", False: "**fail**", None: "no [HB] value"}[x["ok"]]])
    return out


def render(R):
    L = []
    w = L.append
    mcfg = humpback_config()
    w("# 2025-26 validation: Project Humpback as built\n")
    w(f"Generated by `studies/validation_2026.py report` (page commit `{R['git']}`; runs at `{R.get('run_git')}`) from `outputs/results_2026/validation_*.json` "
      "and `reference/aircrafter1/runs/ac1_2026_r1.json`; every number below is read from "
      "`docs/studies/results_2026/validation_2026.json`. Config `configs/humpback_2026.yaml` (extends "
      "`dbf2026_multipoint.yaml`; the ruleset `configs/rulesets/dbf_2025_26.yaml`). [HB] = 2026 USYD DBF Design Report "
      "(Project Humpback), printed page.\n")
    w("**Stated up front.** (i) **Out-of-sample:** the 'USYD without Humpback' column uses the mass factors fitted on "
      "Orange Hornet and Woombah only (`configs/mass_usyd_loo_humpback.yaml`), so Humpback's wing and tails are "
      "predicted, not fitted. Humpback's fuselage and gear were never in any fit ([HB] Table 36 lumps them, Q47): "
      "they are an extrapolation in every column. (ii) **Nothing is tuned to the flight data.** The flight cases set "
      "the throttle from the logged average battery current and the take-off mass from the logged take-off weight "
      "(ballast through the mass model's hook): both are input conditions of the flights, as in validation_2025.md. "
      "(iii) Calm air; take-off in the mission time; course `dbf_2025_360_mid` (the 2026 rules' figure, p. 17). "
      "(iv) Modelling differences: rectangular wing with end-plates (end-plates not modelled), flaps not modelled, "
      "the payload fuselage as a box sized by the payload counts (PLACEHOLDER law, Q55).\n")

    # ---------------------------------------------------------------- criteria
    cr = R.get("criteria")
    if cr:
        V = R["variants"]["loo"]["cases"]
        m3 = [V[k]["snapshot"]["M3"] for k in ("M3_test1", "M3_test2", "M3_test3") if V.get(k, {}).get("snapshot")]
        errs = [100 * (s["otherLapTime"] / TABLE42[k]["lap_s"] - 1) for s, k in zip(m3, ("M3_test1", "M3_test2", "M3_test3"))]
        cd = m3[-1]["CD0"]
        w("**Stated plainly.** Out of sample for the wing and tails only; the fuselage and gear were never in a fit. "
          f"The M3 laps are flown {abs(max(errs)):.0f}-{abs(min(errs)):.0f} % too fast, and the deployed C_D0 at the 197 in test is {cd:.2f} against Table 29's 0.36. "
          "[HB] disagrees with itself (§7, Q57), so 'the report' is not one number.\n")
        w("## 0. The thesis §3.7 agreement criteria\n")
        w(f"Structure mass within ±{100 * CRIT_MASS:.0f} % (the LOO spread); flight performance (speed, laps, energy) within the report's own range or {100 * CRIT_FLIGHT:.0f} %. Out-of-sample column. **Structure: {cr['mass_pass']} of {cr['mass_n']} rows pass. Flight: {cr['flight_pass']} of {cr['flight_n']} rows pass.**\n")
        w(_table(["structure mass (lb)", "[HB]", "model", "error", "±30 %"], _crit_rows(cr["mass"])))
        w("\n" + _table(["flight performance", "[HB]", "model", "error", "range or 10 %"], _crit_rows(cr["flight"])))
        w("\nSpeed is compared through the lap time (the lap is a fixed course), since [HB] logs lap times per flight. Energy has no per-lap [HB] value: the model's is listed, unjudged.\n")
        w("![Mass](figures/humpback_validation_mass.png)\n\n![Flight](figures/humpback_validation_flight.png)\n\n"
          "The calibration's parity plot (predicted against real component masses, in sample and leave-one-out, from `docs/studies/mass_calibration_usyd.json`, not refitted):\n\n![Parity](figures/mass_parity.png)\n")
    # ---------------------------------------------------------------- mass
    w("## 1. Mass against [HB] Table 36\n")
    rows = []
    cols = [m for m in REPORT_VARIANTS if m in R["variants"]]
    mod = {m: _model_rows(R["variants"][m]["cases"]["as_built_full_throttle"], mcfg) for m in cols}
    t36 = dict(TABLE36, M2=TABLE36["M2"], M3=TABLE36["M3"])
    labels = {"wing": "wing (port + starboard)", "HT": "HT (port + starboard)", "VT": "VT",
              "fuselage": "'Fuselage' row (model: cargo box + tail boom + gear + avionics + prop)",
              "motor": "motor", "ESC": "ESC", "empty": "empty", "battery": "battery",
              "M2": "M2 take-off (42 passengers, 14 cargo)", "M3": "M3 take-off (197 in banner)"}
    for k, lab in labels.items():
        row = [lab, _f(t36[k])]
        for m in cols:
            x = mod[m][k] / LB
            row += [_f(x), _pct(x, t36[k])]
        rows.append(row)
    hdr = ["Table 36 row", "[HB] (lb)"]
    for m in cols:
        hdr += [f"{MASS_VARIANTS[m]} (lb)", "error"]
    w(_table(hdr, rows))
    w("\nThe motor, ESC and battery are inputs (the [TM7] spec and [HB] rows), not predictions. [HB] says its CAD "
      "masses were 'validated against the measured weights … with a ≈20 % increase from CAD to manufacturing' "
      "(p. 43), without saying whether Table 36 includes it (Q47); its M2 row (18.54 lb) differs from the sum of its "
      "rows (18.24 lb).\n")

    # ---------------------------------------------------------------- flight
    w("## 2. Flight performance against [HB] Tables 42, 43 and 1\n")
    for m in cols:
        V = R["variants"][m]
        w(f"### {MASS_VARIANTS[m]}\n")
        rows = []
        for name, t in TABLE42.items():
            c = V["cases"].get(name, {})
            pt = t["point"]
            s = c.get("snapshot", {}).get(pt, {})
            lap = s.get("otherLapTime")
            rows.append([name.replace("_", " "), t["prop"], f"{t['n_p']}/{t['n_c']}" if pt == "M2"
                         else f"{t['banner_in']} in", _f(t["tow_lb"], 1), _f(t["current_A"], 0),
                         _f(c.get("throttle"), 3) if c.get("throttle") else (c.get("note") or "–"),
                         _f(t["lap_s"], 1), _f(lap, 1), _pct(lap, t["lap_s"]), _f(s.get("laps_frac"), 2),
                         _f(s.get("v"), 1), _f(c.get("ballast_kg"), 2)])
        w(_table(["test", "prop", "payload / banner", "TOW (lb)", "avg current (A)", "throttle (solved)",
                  "avg lap [HB] (s)", "lap 2+ model (s)", "error", "laps in 5 min (model)", "cruise v (m/s)",
                  "ballast to the logged TOW (kg)"], rows))
        w("\n")
    w("[HB] Tables 43 and 1 (the competition-style counts, not the test flights above): " + "; ".join(
        f"{k} design {d['design']} / actual {d['actual']}" for k, d in TABLE43.items()) + ". Table 1: "
      + "; ".join(f"{k} {d}" for k, d in TABLE1.items()) + ". They disagree with each other (M2 laps 9 vs 8, M3 laps "
      "5 vs 4, M3 speed 49.9 vs 73.3 ft/s; Q57).\n")

    # ---------------------------------------------------------------- banner
    w("## 3. Banner-drag check (M3, nominal C_D,b)\n")
    cd = mcfg.mp.stores2026.cd_banner
    w(f"The model's M3 lap time against the three banner flights at their logged currents, with the nominal banner "
      f"drag coefficient C_D,b = {cd:g} on the banner's own area ([HB] Table 41, car-tow tests). **Reported, not "
      "tuned.**\n")
    for m in cols:
        rows = []
        for name in ("M3_test1", "M3_test2", "M3_test3"):
            c = R["variants"][m]["cases"].get(name, {})
            s = c.get("snapshot", {}).get("M3", {})
            t = TABLE42[name]
            rows.append([f"{t['banner_in']} in", _f(t["lap_s"], 1), _f(s.get("otherLapTime"), 1),
                         _pct(s.get("otherLapTime"), t["lap_s"]), _f(s.get("CD_banner"), 3), _f(s.get("CD0"), 3),
                         _f(s.get("v"), 1)])
        w(f"{MASS_VARIANTS[m]}:\n\n")
        w(_table(["banner", "avg lap [HB] (s)", "lap 2+ model (s)", "error", "banner ΔC_D (wing area)",
                  "C_D0 towing", "cruise v (m/s)"], rows))
        w("\n")

    # ---------------------------------------------------------------- as built, SM
    w("## 4. Humpback as built at full throttle, and the static margin\n")
    rows = []
    for m in cols + [x for x in ("cal",) if x in R["variants"]]:
        c = R["variants"][m]["cases"]["as_built_full_throttle"]
        rows.append([MASS_VARIANTS[m], _f(c["score"]["flight_score"], 4), _f(c["score"]["M2"], 4),
                     _f(c["score"]["M3"], 4), _f(c["M2"]["laps_frac"], 2), _f(c["M3"]["laps_frac"], 2),
                     _f(c["M2"]["SM"], 3), _f(c["M3"]["SM"], 3), _f(c["M2"]["x_cg"], 4), _f(c["M2"]["x_np"], 4),
                     _m1(c["M1"])])
    w(_table(["mass model", "F", "M2", "M3", "M2 laps", "M3 laps", "SM M2", "SM M3", "x_cg M2 (m)", "x_np (m)",
              "M1 check"], rows))
    ref = R["variants"].get("loo") or next(iter(R["variants"].values()))
    c = ref["cases"]["as_built_full_throttle"]
    np_ecalc = 0.3613 * 14.57 * IN          # [HB] §4.10.1 p. 34: neutral point 36.13 % MAC, MAC = 14.57 in
    cg_m2 = 3.52 * IN                       # [HB] Table 36: M2 CG 3.52 in aft of the LE
    chord = 14.57 * IN
    w(f"\n[HB] reports SM 10–13 % from its weight and balance (§5.4, p. 43; CG 3.18–3.54 in aft of the LE) and an "
      f"eCalc neutral point at 36.13 % MAC ({np_ecalc:.3f} m; §4.10.1, p. 34). **The model's neutral point "
      f"({c['M2']['x_np']:.3f} m, {MASS_VARIANTS[ref['mass']]}) sits {c['M2']['x_np'] - np_ecalc:.3f} m "
      f"({(c['M2']['x_np'] - np_ecalc) / chord:.2f} c) aft of eCalc's** (classic tail term, no fuselage term, with "
      f"Humpback's 70.5 in fuselage), and its M2 CG ({c['M2']['x_cg']:.3f} m) {cg_m2 - c['M2']['x_cg']:+.3f} m "
      f"forward of Table 36's ({cg_m2:.3f} m), so its SM reads {c['M2']['SM'] - 0.13:.2f}–{c['M2']['SM'] - 0.10:.2f} "
      f"higher. The relaxed upper SM bound ({mcfg.base.stability.SM_max:g}) is set from the modelled values "
      "(decision 2026-10-27; Q9, Q62).\n")

    # ---------------------------------------------------------------- AC1
    w("## 5. AirCrafter 1.0 ('2026' mode) at Humpback's design point\n")
    a = (R.get("ac1") or {}).get("humpback_point")
    if a:
        rows = []
        for mis in ("M2", "M3"):
            o = a.get(mis) or {}
            rows.append([mis, _f(o.get("V"), 0), _f(o.get("lapCount"), 0), _f(o.get("otherLapTime"), 1),
                         _f(o.get("Preq"), 0), _f(o.get("score"), 4)])
        w(f"Inputs: {a['inputs']} ([HB] Tables 24, 36, 42: M2 17.9 lb with 42 / 14, M3 12.8 lb with the 197 in banner, "
          "its 1.21 lb taken out because AirCrafter 1.0 adds the banner's mass; the 97.68 Wh pack). The speed with the "
          "best mission score on the shipped 1 m/s axis, as its selection does; M15 fixed (banner and bay drag in the "
          "lap model).\n\n")
        w(_table(["mission", "speed (m/s)", "laps (integer)", "lap 2+ (s)", "P_req (W)", "score"], rows))
        w("\n")
    else:
        w("Not run yet (reference/aircrafter1/run_2026.m).\n")

    # ---------------------------------------------------------------- envelope
    w("## 6. Analysis envelope\n")
    env = R.get("envelope")
    if env:
        rows = []
        for r in sorted(env["rows"], key=lambda r: (r["props"], r["Kv"], r["throttle"])):
            if r["converged"]:
                stall = [r[pt].get("stall_margin") for pt in ("M2", "M3")]
                low = any(x is not None and x < 0 for x in stall)
                conv = "yes, < 1.2 V_stall" if low else "yes"
                rows.append([r["props"], _f(r["Kv"], 0), _f(r["throttle"], 1), conv]
                            + [_f(r[pt].get(k), 1) for pt in ("M2", "M3")
                               for k in ("v", "v_fit_margin", "laps_frac", "stall_margin")])
            else:
                rows.append([r["props"], _f(r["Kv"], 0), _f(r["throttle"], 1), "no"] + [""] * 8)
        w(_table(["props M2/M3", "Kv", "throttle", "converged", "M2 v", "M2 v_fit margin", "M2 laps",
                  "M2 stall margin", "M3 v", "M3 v_fit margin", "M3 laps", "M3 stall margin"], rows))
        w("\nThe MDA runs with the continuation fallback (`mda_robust`, docs/MDA_ROBUSTNESS_PLAN.md): cells whose "
          "Newton start lies outside the T = D root's basin converge through it. Stall margin = V - 1.2 V_stall (m/s); "
          "'yes, < 1.2 V_stall': the balance converged with V below 1.2 V_stall (stall margin < 0), which the "
          "optimiser's stall_margin constraint rejects.\n")
        w("\nThe T(v) fit range: `f_fit` = 0.95 for the 2025-26 configs (0.8 in 2024-25): Humpback's props cruise at "
          "J ≈ 0.9 J_max, past the 0.8 fit's top node (thrust 5 % low at the M2 cruise speed), while at 0.95 the top "
          "node stays inside the APC data and the thrust is within 1.4 % (checkpoint-2 check, 19x12E).\n")
    else:
        w("Not run yet (`studies/validation_2026.py envelope`).\n")

    # ---------------------------------------------------------------- inconsistencies
    w("## 7. [HB] inconsistencies (listed, not resolved; Q57)\n")
    for s in ["Table 1 (p. 4) vs Table 43 (p. 59): M2 laps 9 vs 8; M3 laps 5 vs 4; M3 speed 49.9 vs 73.3 ft/s.",
              "Table 33 (p. 35, 42 passengers / 14 cargo, 9 laps) vs Table 43 'design' (30 / 10, 275 in banner).",
              "M2 take-off weight: Table 1 17.5 lb, Table 42 17.9 lb, Table 33 14.3 lb, §4.8.4 13.2 lb, Table 36 18.54 lb.",
              "Battery: 3380 mAh (§4.8.3 p. 30; 100.05 Wh at 29.6 V, over the 100 Wh rule), 3300 mAh / 97.68 Wh "
              "(§5.3.10 p. 42, §5.3.12), 3350 mAh (drawing p. 47). 3300 mAh is used.",
              "Wing: 59 × 14.57 = 859.6 in² against the printed 868 in² (p. 36); Table 24 c̄ 1.21 ft.",
              "Fuselage: passenger bay 39 in (p. 36; '3 ft 3 in', p. 22) longer than Fig. 9's 2 ft 9.4 in payload box; "
              "Fig. 9's 5 ft 1 in against p. 36's 70.5 in.",
              "Cost equation (Eq. 6, p. 7) without the rules' EF.",
              "Banner drag: car-tow C_D,b 0.019–0.028 (Table 41) against the flat-plate 1.2 (§4.8.5) and Table 29's "
              "ΔC_D 0.3025 on the wing area (Q56); Table 41 says all tests used 118 in banners but lists six lengths.",
              "Spar sizes: p. 26 text, Table 24 and p. 37 disagree; Table 36's 'Mission 1' row (Q47)."]:
        w(f"- {s}")
    w("")
    return "\n".join(L)


def report():
    R = collect()
    DOCS.mkdir(parents=True, exist_ok=True)
    (DOCS / "validation_2026.json").write_text(json.dumps(R, indent=1, default=float))
    (DOCS / "validation_2026.md").write_text(render(R))
    print("wrote docs/studies/results_2026/validation_2026.{md,json}")
