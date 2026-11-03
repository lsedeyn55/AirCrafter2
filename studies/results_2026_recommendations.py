"""The recommendations for the team (results_2026.md §8): generated from the run data, each pointing to its result.

Model-based. None rests only on a PLACEHOLDER-driven result: where a PLACEHOLDER sets a number (the payload at the score's
cap, Q52), the bullet states only what holds at every normalisation tried.
"""


def _m3_prop_means(R):
    """Mean F at 8S per M3 prop and per M2 prop, from the converged enumeration."""
    m2, m3 = {}, {}
    for d in R["enumeration"]:
        parts = d["tag"].replace("fr_", "").split("_")
        if d.get("ok") and d.get("F") is not None and parts[0] == "8S":
            m2.setdefault(parts[1], []).append(d["F"])
            m3.setdefault(parts[2], []).append(d["F"])
    return ({k: sum(v) / len(v) for k, v in m2.items()}, {k: sum(v) / len(v) for k, v in m3.items()})


def build(R, P):
    """Five to eight bullets; a bullet whose evidence is missing is left out, never written from memory."""
    recs = []
    m2, m3 = _m3_prop_means(R)
    if m2 and m3:
        best = max(m3, key=m3.get)
        recs.append(f"**Choose the M3 prop first; it matters more than the M2 prop.** Across the 8S enumeration the mean F runs from "
                    f"{min(m3.values()):.4f} to {max(m3.values()):.4f} with the M3 prop ({best} best), and only from {min(m2.values()):.4f} to "
                    f"{max(m2.values()):.4f} with the M2 prop. *Rests on §5.1. Confidence: medium; the ordering comes from the cited APC data, "
                    "but the model flies M3 laps too fast (§7), so the size of the gain isn't a prediction.*")
    hb = P.nominal(R, "humpback")
    norm = {k: v for k, v in R["normalisation"].items() if v.get("ok") and v.get("dv")}
    if hb and norm:
        low = min(v["dv"]["n_p"] for v in norm.values())
        recs.append(f"**Carry more passengers than Humpback's {hb['dv']['n_p']:.0f}, and test the fuselage it takes.** Every normalisation tried wants at least "
                    f"{low:.0f} passengers, so the direction doesn't rest on the PLACEHOLDER normalisers (Q52), though the MDO's own count does. "
                    "*Rests on §5.1 and §5.2. Confidence: medium for the direction, low for the count; the Ground Mission, which grows with the counts, isn't modelled (Q53), and "
                    f"a fuselage this long is an extrapolation of the mass calibration (§7).*")
    mdo, rob = P.nominal(R, "mdo"), P.nominal(R, "rdo")
    d = P.decomposition(R)
    if mdo and rob and d:
        recs.append(f"**Don't size the banner to its limit before its drag is known.** RDO gives up banner length ({mdo['dv']['L_banner']:.0f} to {rob['dv']['L_banner']:.0f} in) for margin: "
                    f"sigma of F {d['sigma']['mdo']:.4f} to {d['sigma']['rdo']:.4f}, P(any violation) {d['p_any']['mdo']:.3f} to {d['p_any']['rdo']:.3f}, at {-d['nominal']['total']:.4f} of nominal F. "
                    "The re-analyses at [HB] Table 41's drag coefficients show why (§5.4). *Rests on §4 and §5.4. Confidence: medium; the Monte Carlo favours RDO by construction, so read "
                    "it with §2.5's out-of-set result.*")
    sc = (R["designs"]["rdo"].get("screen") or {}).get("oned")
    if sc:
        cited = {k: sc[k] ** 0.5 for k in ("banner_drag_factor", "f_build") if sc.get(k) is not None}
        if len(cited) == 2:
            recs.append(f"**Spend ground and flight tests on banner drag and built mass.** They are the largest contributions to the sigma of F with measured spreads: banner drag factor "
                        f"{cited['banner_drag_factor']:.4f} ([HB] Table 41), build factor {cited['f_build']:.4f} (USYD leave-one-out). Wind contributes more, but its spread is a PLACEHOLDER and it can't be designed out by a test. "
                        "*Rests on §4's screening. Confidence: medium.*")
    ms = P.multistart_summary(R)
    if ms and ms["spread"].get("Kv"):
        kv = ms["spread"]["Kv"]
        recs.append(f"**Don't commission a motor rewind for this score.** At the best prop pair, every converged start reaches the same F with Kv anywhere from {kv['min']:.0f} to {kv['max']:.0f} rpm/V: "
                    "a flat direction, inside which the throttle takes up the difference. *Rests on §5.1's multistart. Confidence: medium; the motor's limits are checked, but its thermal behaviour isn't modelled.*")
    if mdo and hb:
        recs.append(f"**Use the rule limits on span and M3 energy.** The optima sit on the 5 ft span (b {mdo['dv']['b']:.3f} m, Humpback {hb['dv']['b']:.3f} m) and the 100 Wh limit in M3, both from the rules [R26 p. 10, p. 16], not from a PLACEHOLDER. "
                    "*Rests on §3's active-constraint table. Confidence: high that they're active in the model, medium on the gain, which isn't separated from the others.*")
    v = R.get("validation")
    if v:
        recs.append("**Log M3 lap times and throttle in test flights, with the banner's size.** The largest gap between this model and Humpback is M3's lap time, and these logs are what closes it. "
                    "*Rests on validation_2026.md and §7's first row. Confidence: high that the gap exists, its cause isn't separated (Q56, Q57).*")
    return recs
