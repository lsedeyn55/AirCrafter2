"""Planform, tail and cargo-bay geometry, with analytic partials.

Rectangular wing only: c_root = c_mean = c_mac = c. The first step of the model after the airfoil: b, c and the tail
design variables become areas, volume coefficients and the boom length for aero, mass and stability.
Sources: G&G (Garcia & Gamboa 2022) Fig. 1 and Eqs. 5-8; Raymer, Aircraft Design: A Conceptual Approach, Sec. 6.4.
Details: docs/modules/geometry.md.
"""

from ..analytic import AnalyticComponent, D


class RectPlanform(AnalyticComponent):
    """S = b c, AR = b / c, c_mean = c_root = c. Partials are analytic.

    Inputs
    ------
    b [m], c [m]

    Outputs
    -------
    S [m**2], AR, c_mean [m], c_root [m]
    """

    def io(self):
        return (
            {"b": (1.5, "m"), "c": (0.25, "m")},
            {"S": (0.375, "m**2"), "AR": (6.0, None), "c_mean": (0.25, "m"), "c_root": (0.25, "m")},
        )

    def evaluate(self, x):
        b, c = x["b"], x["c"]
        return (
            {"S": b * c, "AR": b / c, "c_mean": c, "c_root": c},
            {
                "S": D({"b": c, "c": b}),
                "AR": D({"b": 1.0 / c, "c": -b / c**2}),
                "c_mean": D({"c": 1.0}),
                "c_root": D({"c": 1.0}),
            },
        )


class TailGeometry(AnalyticComponent):
    """Tail boom length and tail volume coefficients. Partials are analytic.

    The boom runs from a point boom_offset aft of the wing quarter chord to the tail quarter chord, so l_tb = l_ht -
    boom_offset. G&G Eqs. 5-8 disagree with its Table 3 on the tail arm, so both lengths come from the tail arm directly
    (docs/DECISIONS.md#plan-g1). V_h = S_ht l_ht / (S c_mac), V_v = S_vt l_vt / (S b), with l_vt = l_ht (Raymer Sec.
    6.4).

    Inputs
    ------
    l_ht [m], S_ht, S_vt, S [m**2], b, c, boom_offset [m]

    Outputs
    -------
    l_tb [m], V_h, V_v

    Symbols
    -------
    l : tail arm l_ht [m]
    Sh, Sv : tail areas [m**2]
    """

    def io(self):
        ins = {
            "l_ht": (0.8, "m"),
            "S_ht": (0.06, "m**2"),
            "S_vt": (0.03, "m**2"),
            "S": (0.375, "m**2"),
            "b": (1.5, "m"),
            "c": (0.25, "m"),
            "boom_offset": (0.1, "m"),
        }
        return ins, {"l_tb": (0.7, "m"), "V_h": (0.5, None), "V_v": (0.04, None)}

    def evaluate(self, x):
        l, Sh, Sv, S, b, c = x["l_ht"], x["S_ht"], x["S_vt"], x["S"], x["b"], x["c"]
        Vh = Sh * l / (S * c)
        Vv = Sv * l / (S * b)
        return (
            {"l_tb": l - x["boom_offset"], "V_h": Vh, "V_v": Vv},
            {
                "l_tb": D({"l_ht": 1.0, "boom_offset": -1.0}),
                "V_h": D({"S_ht": l / (S * c), "l_ht": Sh / (S * c), "S": -Vh / S, "c": -Vh / c}),
                "V_v": D({"S_vt": l / (S * b), "l_ht": Sv / (S * b), "S": -Vv / S, "b": -Vv / b}),
            },
        )


class CargoBay(AnalyticComponent):
    """Surface area of a box cargo bay: S_cb = 2 (l w + l h + w h) (G&G Eq. 37 input). Partials are analytic.

    Inputs
    ------
    l_cb, w_cb, h_cb [m]

    Outputs
    -------
    S_cb [m**2]
    """

    def io(self):
        return ({"l_cb": (0.4, "m"), "w_cb": (0.12, "m"), "h_cb": (0.12, "m")}, {"S_cb": (0.2, "m**2")})

    def evaluate(self, x):
        l, w, h = x["l_cb"], x["w_cb"], x["h_cb"]
        return (
            {"S_cb": 2.0 * (l * w + l * h + w * h)},
            {"S_cb": D({"l_cb": 2.0 * (w + h), "w_cb": 2.0 * (l + h), "h_cb": 2.0 * (l + w)})},
        )


class MotorPlacement(AnalyticComponent):
    """Wing-motor station and prop clearance margins (n_motors = 2, docs/DECISIONS.md#plan-d8). Continuous, never NaN.

    station_mode="absolute" (default): y_m is an input (m from the centreline).
    station_mode="fraction": y_m = y_motor b / 2, with y_motor a fraction of the semi-span.

        fus_clear_margin  = y_m - D/2 - (w_cb/2 + c_clear)   >= 0   prop tip clear of the fuselage
        tip_clear_margin  = b/2 - (y_m + D/2)                >= 0   prop disc inside the wing tip
        prop_clear_margin = 2 y_m - (D + c_clear)            >= 0   disc to disc (implied by the first)

    Partials are analytic.

    Options
    -------
    station_mode : "absolute" or "fraction"

    Inputs
    ------
    b [m], D_prop [m], w_cb [m], c_clear [m], and y_motor or y_m

    Outputs
    -------
    fus_clear_margin, tip_clear_margin, prop_clear_margin [m], and y_m with station_mode = fraction
    """

    def initialize(self):
        self.options.declare("station_mode", default="absolute", values=["absolute", "fraction"])

    def io(self):
        ins = {"b": (1.5, "m"), "D_prop": (0.254, "m"), "w_cb": (0.12, "m"), "c_clear": (0.025, "m")}
        outs = {"fus_clear_margin": (0.05, "m"), "tip_clear_margin": (0.3, "m"), "prop_clear_margin": (0.2, "m")}
        if self.options["station_mode"] == "fraction":
            ins["y_motor"] = (0.28, None)
            outs["y_m"] = (0.25, "m")
        else:
            ins["y_m"] = (0.25, "m")
        return ins, outs

    def evaluate(self, x):
        b, Dp, w, cc = x["b"], x["D_prop"], x["w_cb"], x["c_clear"]
        if self.options["station_mode"] == "fraction":
            ym, gym = x["y_motor"] * b / 2, D({"y_motor": b / 2, "b": x["y_motor"] / 2})
        else:
            ym, gym = x["y_m"], D({"y_m": 1.0})
        values = {
            "fus_clear_margin": ym - Dp / 2 - w / 2 - cc,
            "tip_clear_margin": b / 2 - ym - Dp / 2,
            "prop_clear_margin": 2 * ym - Dp - cc,
        }
        grads = {
            "fus_clear_margin": gym + D({"D_prop": -0.5, "w_cb": -0.5, "c_clear": -1.0}),
            "tip_clear_margin": D({"b": 0.5, "D_prop": -0.5}) - gym,
            "prop_clear_margin": 2.0 * gym + D({"D_prop": -1.0, "c_clear": -1.0}),
        }
        if self.options["station_mode"] == "fraction":
            values["y_m"], grads["y_m"] = ym, gym
        return values, grads
