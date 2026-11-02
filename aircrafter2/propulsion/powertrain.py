"""Electric powertrain: battery + ESC + brushless motor + APC propeller.

The rotor speed balances motor and prop torque at fixed flight-speed nodes, giving cubic fits T(v), P_batt(v) and rpm(v)
at a fixed throttle. In the full model it runs before the MDA (problems/full_mdo.py), and its fits and system masses
feed the MDA and the lap model, so the powertrain sets the power (docs/DECISIONS.md#plan-b).
Sources: M. Drela, "First-Order DC Electric Motor Model", MIT 16.50 notes, 2007; APC PER3 data (apc.py).
Details: docs/modules/propulsion.md, "Powertrain".
"""

import numpy as np
import openmdao.api as om

from .apc import load_prop

RPM2RADS = 2.0 * np.pi / 60.0
BATTERY_TOPOLOGIES = ("per_motor", "shared")


def motor_stations(n_motors, y_motor=None):
    """Spanwise stations of the propulsion units, as signed fractions of the semi-span.

    Built as a list of stations so further symmetric pairs can be added later.

    Parameters
    ----------
    n_motors : int
        1: one centreline nose tractor. 2: a symmetric pair of wing-mounted tractors.
    y_motor : float, optional
        Station as a fraction of the semi-span (n_motors = 2), in (0, 1).

    Returns
    -------
    list of float
        [0.0], or [-y_motor, +y_motor]. Raises ValueError for anything else.
    """
    if n_motors == 1:
        return [0.0]
    if n_motors == 2:
        if y_motor is None or not 0.0 < y_motor < 1.0:
            raise ValueError(f"n_motors = 2 needs y_motor (fraction of the semi-span) in (0, 1), got {y_motor!r}")
        return [-float(y_motor), float(y_motor)]
    raise ValueError(f"n_motors = {n_motors!r} is not supported: use 1 (nose tractor) or 2 (wing-mounted pair)")


class Powertrain(om.ExplicitComponent):
    """Cubic T(v), P_batt(v) and rpm(v) fits, static thrust and current, energy and masses.

    The operating point is solved once, for one unit, with a fixed iteration count; totals scale with the number of
    units. Partials are complex step through that fixed-iteration Newton, which makes them exact; everything is
    complex-step safe.

    Options
    -------
    prop : str
        APC PER3 name (enumerated outside the optimiser, docs/DECISIONS.md#plan-d4).
    apc_dir : str or None
        Folder with the APC data (apc.load_prop).
    n_nodes, deg, newton_iters : int
        Speed nodes, degree of the fits, and Newton iterations.
    v_top : float
        m/s, highest speed node, fit_range = fixed.
    n_motors : int
        Identical propulsion units, 1 or 2 (docs/DECISIONS.md#plan-d8).
    battery_topology : "per_motor" or "shared"
        One pack per unit, or one pack shared by every ESC. capacity is the capacity of one pack in both.
    fit_range : "fixed" or "throttle"
        Fit nodes in [0, v_top], or in [0, v_fit_max(throttle)] for part throttle.
    f_fit : float
        v_fit_max = f_fit J_max n0 D, fit_range = throttle.

    Inputs
    ------
    throttle, rho [kg/m**3], n_cells, V_cell [V], R_cell [ohm], capacity [A*h], usable_fraction, e_spec [W*h/kg]
    Kv [rpm/V], Rm [ohm], I0 [A], m_motor, m_esc, m_prop, m_avionics, m_nacelle [kg]

    Outputs
    -------
    T_coeffs, P_coeffs, RPM_coeffs : arrays (deg + 1,)
        Totals over all units, except rpm: T(v) = sum t_i v^i [N], P_batt(v) [W], rpm(v), with v in m/s.
    T_static, T_static_unit [N], I_static, I_pack_static [A]
        Static thrust, total and per unit; ESC input current of one unit at v = 0 (the maximum); current of one pack.
    rpm_max [rpm]
        Largest rotor speed over the nodes (report only: the top node is above the flight envelope). Use rpm(v) at the
        actual top speed instead.
    E_nom_total [W*h], E_usable [J], m_batt, m_motor_unit, m_motor_group, m_sys [kg], v_fit_max [m/s]
        Full list in `setup()`.

    Symbols
    -------
    dl : throttle delta
    Voc, Rb : pack open-circuit voltage [V] and resistance [ohm]
    kv : Kv in rad/s/V
    Rm : winding resistance [ohm]
    I : motor current [A]
    omega : rotor speed [rad/s]
    n, D, J : rotor speed [rev/s], prop diameter [m], advance ratio v / (n D)
    Qm, Qp : motor and prop torque [N*m]
    """

    def initialize(self):
        self.options.declare("prop", default="11x7E", desc="APC PER3 name (discrete choice, D4)")
        self.options.declare("apc_dir", default=None)
        self.options.declare("n_nodes", default=8, types=int)
        self.options.declare("v_top", default=40.0, desc="m/s, highest speed node (fit range [0, v_top])")
        self.options.declare("deg", default=3, types=int)
        self.options.declare("newton_iters", default=40, types=int)
        self.options.declare("n_motors", default=1, types=int, desc="identical propulsion units (1 or 2)")
        self.options.declare(
            "battery_topology",
            default="per_motor",
            values=BATTERY_TOPOLOGIES,
            desc="one pack per unit, or one pack shared by every ESC",
        )
        self.options.declare(
            "fit_range",
            default="fixed",
            values=["fixed", "throttle"],
            desc="T(v) fit nodes in [0, v_top], or in [0, v_fit_max(throttle)] (part throttle)",
        )
        self.options.declare("f_fit", default=0.8, desc="v_fit_max = f_fit J_max n0 D (fit_range = throttle)")

    def setup(self):
        motor_stations(self.options["n_motors"], 0.5)  # validates n_motors
        n_units = self.options["n_motors"]
        shared = self.options["battery_topology"] == "shared"
        self._n_units = float(n_units)
        self._units_per_pack = float(n_units) if shared else 1.0
        self._n_packs = 1.0 if shared else float(n_units)
        self._prop = load_prop(self.options["prop"], self.options["apc_dir"])
        n = self.options["n_nodes"]
        self._v = np.linspace(0.0, self.options["v_top"], n)
        self._s = np.linspace(0.0, 1.0, n)  # fit_range = throttle: nodes s = v / v_fit_max
        nodes = self._v if self.options["fit_range"] == "fixed" else self._s
        V = np.stack([nodes**i for i in range(self.options["deg"] + 1)], axis=1)
        self._pinv = np.linalg.pinv(V)  # real: keeps the fit complex-step analytic
        d = self.options["deg"] + 1

        for name, val, units in [
            ("throttle", 1.0, None),
            ("rho", 1.18, "kg/m**3"),
            ("n_cells", 4.0, None),
            ("V_cell", 3.7, "V"),
            ("R_cell", 0.005, "ohm"),
            ("capacity", 3.0, "A*h"),
            ("usable_fraction", 0.8, None),
            ("e_spec", 150.0, "W*h/kg"),
            ("Kv", 900.0, "rpm/V"),
            ("Rm", 0.05, "ohm"),
            ("I0", 1.0, "A"),
            ("m_motor", 0.15, "kg"),
            ("m_esc", 0.06, "kg"),
            ("m_prop", 0.03, "kg"),
            ("m_avionics", 0.25, "kg"),
            ("m_nacelle", 0.0, "kg"),
        ]:
            self.add_input(name, val, units=units)

        # thrust, battery power, energy and masses are totals over all units; rpm and current are per unit
        self.add_output("T_coeffs", np.zeros(d), desc="total T(v) = sum t_i v^i, N, v in m/s")
        self.add_output("P_coeffs", np.zeros(d), desc="total P_batt(v) = sum p_i v^i, W")
        self.add_output("RPM_coeffs", np.zeros(d), desc="rotor rpm(v) = sum r_i v^i (rises with v)")
        self.add_output("T_static", 10.0, units="N", desc="total static thrust")
        self.add_output("T_static_unit", 10.0, units="N", desc="static thrust of one unit")
        self.add_output("I_static", 30.0, units="A", desc="ESC input current of one unit at v = 0 (the maximum)")
        self.add_output("I_pack_static", 30.0, units="A", desc="current of one pack at v = 0")
        self.add_output("rpm_max", 10000.0, units="rpm")
        self.add_output("rpm_limit", self._prop.rpm_limit, units="rpm")
        self.add_output("D_prop", self._prop.diameter, units="m")
        self.add_output("E_nom_total", 100.0, units="W*h", desc="nominal energy of all packs (rule C-F1)")
        self.add_output("E_usable", 1e5, units="J", desc="usable energy of all packs")
        self.add_output("m_batt", 0.3, units="kg", desc="all packs")
        self.add_output("m_motor_unit", 0.24, units="kg", desc="motor + ESC + prop + nacelle, one unit")
        self.add_output("m_motor_group", 0.24, units="kg", desc="all units")
        self.add_output("m_sys", 0.8, units="kg")
        self.add_output("T_nodes", np.zeros(n), units="N", desc="total")
        self.add_output("P_nodes", np.zeros(n), units="W", desc="total")
        self.add_output("eta_prop_nodes", np.zeros(n))
        self.add_output("fit_residual_T", 0.0, units="N", desc="max |fit - node| (report)")
        self.add_output("v_fit_max", self.options["v_top"], units="m/s", desc="top of the T(v) fit range")
        elec = ["throttle", "rho", "n_cells", "V_cell", "R_cell", "Kv", "Rm", "I0"]
        for out in [
            "T_coeffs",
            "P_coeffs",
            "RPM_coeffs",
            "T_static",
            "T_static_unit",
            "I_static",
            "I_pack_static",
            "rpm_max",
            "T_nodes",
            "P_nodes",
            "eta_prop_nodes",
        ]:
            self.declare_partials(out, elec, method="cs")
        if self.options["fit_range"] == "throttle":
            self.declare_partials("v_fit_max", elec, method="cs")
        pack = ["n_cells", "V_cell", "capacity"]
        self.declare_partials("E_nom_total", pack, method="cs")
        self.declare_partials("E_usable", pack + ["usable_fraction"], method="cs")
        self.declare_partials("m_batt", pack + ["e_spec"], method="cs")
        unit = ["m_motor", "m_esc", "m_prop", "m_nacelle"]
        self.declare_partials("m_motor_unit", unit, val=1.0)
        self.declare_partials("m_motor_group", unit, val=self._n_units)
        self.declare_partials("m_sys", pack + ["e_spec", "m_avionics"] + unit, method="cs")

    def solve_nodes(self, x):
        """Solve one unit's operating point at every speed node.

        Parameters
        ----------
        x : dict
            Input name -> value (complex allowed).

        Returns
        -------
        dict
            Arrays (n_nodes,): omega [rad/s], I_motor [A], T [N], P_batt [W], eta_prop, J, v [m/s]; and Voc [V].
        """
        prop = self._prop
        D = prop.diameter
        rho, dl = x["rho"], x["throttle"]
        Voc = x["n_cells"] * x["V_cell"]
        Rb = x["n_cells"] * x["R_cell"]
        kv = x["Kv"] * RPM2RADS
        Rtot = x["Rm"] + self._units_per_pack * dl**2 * Rb  # shared pack: n units sag one pack

        def residual(om_, v):
            """Motor torque minus prop torque at rotor speed om_ and flight speed v, and I, n, J there."""
            I = (dl * Voc - om_ / kv) / Rtot
            Qm = (I - x["I0"]) / kv
            n = om_ / (2 * np.pi)
            J = v / (n * D)
            Qp = prop.cp(J, om_ / RPM2RADS) * rho * n**2 * D**5 / (2 * np.pi)
            return Qm - Qp, I, n, J

        v = self._v if self.options["fit_range"] == "fixed" else self._s * self.v_fit_max(x)
        om_ = 0.7 * kv * dl * Voc * np.ones_like(v)  # start below no-load speed
        h = 1e-3
        for _ in range(self.options["newton_iters"]):
            r, *_ = residual(om_, v)
            r2, *_ = residual(om_ + h, v)
            om_ = om_ - r / ((r2 - r) / h)
        _, I, n, J = residual(om_, v)
        T = prop.ct(J, om_ / RPM2RADS) * rho * n**2 * D**4
        P_batt = Voc * dl * I
        P_shaft = prop.cp(J, om_ / RPM2RADS) * rho * n**3 * D**5
        eta_prop = np.where(np.real(P_shaft) > 0, T * v / P_shaft, 0.0 * T)
        return dict(omega=om_, I_motor=I, T=T, P_batt=P_batt, eta_prop=eta_prop, J=J, Voc=Voc, v=v)

    def v_fit_max(self, x):
        """Top of the fit range: v_top, or (fit_range = throttle) KSmin(v_top, f_fit J_max n0 D).

        Parameters
        ----------
        x : dict
            Input name -> value.

        Returns
        -------
        float or array
            m/s.
        """
        if self.options["fit_range"] == "fixed":
            return self.options["v_top"]
        from ..smooth import ks_min

        n0 = x["Kv"] * RPM2RADS * x["throttle"] * x["n_cells"] * x["V_cell"] / (2 * np.pi)
        v_wm = self.options["f_fit"] * self._prop.J_max * n0 * self._prop.diameter
        return ks_min([self.options["v_top"] * np.ones_like(v_wm), v_wm], rho=50.0)[0]

    def compute(self, inputs, outputs):
        vals = {k: inputs[k] for k in inputs}
        node_results = self.solve_nodes(vals)  # one unit
        n_units = self._n_units
        T, P = n_units * node_results["T"], n_units * node_results["P_batt"]
        outputs["T_nodes"] = T
        outputs["P_nodes"] = P
        outputs["eta_prop_nodes"] = node_results["eta_prop"]
        if self.options["fit_range"] == "fixed":
            t_coeffs = self._pinv @ T
            outputs["T_coeffs"] = t_coeffs
            outputs["P_coeffs"] = self._pinv @ P
            outputs["RPM_coeffs"] = self._pinv @ (node_results["omega"] / RPM2RADS)
            V = np.stack([self._v**i for i in range(len(t_coeffs))], axis=1)
            outputs["fit_residual_T"] = np.max(np.abs(np.real(V @ t_coeffs - T)))
            outputs["v_fit_max"] = self.options["v_top"]
        else:  # fit in s = v / vmax on fixed nodes (analytic), then t_i = a_i / vmax^i
            vmax = self.v_fit_max(vals)
            scale = np.array([vmax**-i for i in range(self.options["deg"] + 1)]).ravel()
            t_coeffs = (self._pinv @ T) * scale
            outputs["T_coeffs"] = t_coeffs
            outputs["P_coeffs"] = (self._pinv @ P) * scale
            outputs["RPM_coeffs"] = (self._pinv @ (node_results["omega"] / RPM2RADS)) * scale
            S = np.stack([self._s**i for i in range(len(t_coeffs))], axis=1)
            outputs["fit_residual_T"] = np.max(np.abs(np.real(S @ (self._pinv @ T) - T)))
            outputs["v_fit_max"] = vmax
        outputs["T_static"] = T[0]
        outputs["T_static_unit"] = node_results["T"][0]
        outputs["I_static"] = vals["throttle"] * node_results["I_motor"][0]
        outputs["I_pack_static"] = self._units_per_pack * vals["throttle"] * node_results["I_motor"][0]
        outputs["D_prop"] = self._prop.diameter
        outputs["rpm_max"] = (
            np.max(np.real(node_results["omega"])) / RPM2RADS
            if not np.iscomplexobj(node_results["omega"])
            else node_results["omega"][np.argmax(np.real(node_results["omega"]))] / RPM2RADS
        )
        outputs["rpm_limit"] = self._prop.rpm_limit
        E_nom = self._n_packs * vals["n_cells"] * vals["V_cell"] * vals["capacity"] * 3600.0  # all packs, J
        outputs["E_nom_total"] = E_nom / 3600.0
        outputs["E_usable"] = vals["usable_fraction"] * E_nom
        m_batt = E_nom / 3600.0 / vals["e_spec"]
        outputs["m_batt"] = m_batt
        m_unit = vals["m_motor"] + vals["m_esc"] + vals["m_prop"] + vals["m_nacelle"]
        outputs["m_motor_unit"] = m_unit
        outputs["m_motor_group"] = n_units * m_unit
        outputs["m_sys"] = m_batt + n_units * m_unit + vals["m_avionics"]
