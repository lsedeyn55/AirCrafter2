# Geometry (`aircrafter2/geometry/`)

| Quantity | Model |
|---|---|
| Wing | rectangular: S = bc, AR = b/c, c_root = c_mean = c |
| Tail boom | l_tb = l_ht − boom_offset |
| Tail volume coefficients | V_h = S_ht l_ht/(S c), V_v = S_vt l_ht/(S b) (Raymer Sec. 6.4) |
| Cargo bay | box surface area 2(lw + lh + wh) |

Partials are analytic. Multi-panel tapered wings were dropped (2026-10-09).

**Tail boom length** (G&G Fig. 1 layout). The boom runs from a point `boom_offset` aft of the wing quarter chord to the tail
quarter chord, so l_tb = l_ht − `boom_offset`. G&G Eqs. 5–8 relate l_ht and l_tb to the chord; here both come from the tail arm
directly ([plan-g1](../DECISIONS.md#plan-g1)).

**Tail volume coefficients** (Raymer, *Aircraft Design: A Conceptual Approach*, Sec. 6.4): V_h = S_ht l_ht / (S c_mac),
V_v = S_vt l_vt / (S b), with l_vt = l_ht.
