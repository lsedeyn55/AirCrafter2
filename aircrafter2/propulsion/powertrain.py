"""Electric powertrain: battery + ESC + brushless motor + APC propeller (decision B).

Models (all complex-step safe; the component uses exact cs partials):

Battery    V_oc = n_s V_cell,  R_b = n_s R_cell,  E_nom = n_s V_cell Q,  E_usable = f_use E_nom.
ESC        ideal PWM at throttle delta: V_motor = delta V_term, I_batt = delta I_motor.
Motor      first-order DC model (M. Drela, "First-Order DC Electric Motor Model", MIT 16.50
           notes, 2007):  I = (V_motor - omega / K_v) / R_m,  Q = (I - I_0) / K_v,
           with K_v in rad/s/V. Combined with the ESC and battery:
               I = (delta V_oc - omega / K_v) / (R_m + delta^2 R_b)
Propeller  APC surrogate (apc.py): T = C_T rho n^2 D^4,  Q_p = C_P rho n^2 D^5 / (2 pi),
           J = v / (n D),  n = omega / (2 pi).
Operating point: omega solves Q_motor(omega) = Q_prop(omega, v) (Newton, fixed iteration
count) at fixed flight-speed nodes v_k in [0, v_top].

Several identical units (motor + ESC + prop [+ nacelle]), option ``n_motors`` (1 or 2, see
``motor_stations``). The operating point is solved once, for one unit; totals scale with the
number of units. ``battery_topology``:
    per_motor  one pack of n_cells x capacity per unit (default; Woombah, 2025 rule C-F2). Each unit
               sees its own pack, so the per-unit operating point is the single-unit one.
    shared     one pack of n_cells x capacity feeds every ESC. The pack current is n delta I, so
               V_term = V_oc - n delta I R_b and
                   I = (delta V_oc - omega / K_v) / (R_m + n delta^2 R_b)
               (derivation: docs/modules/propulsion.md).
``capacity`` is the capacity of one pack in both topologies.

Outputs for the lap model: cubic least-squares fits through the nodes
    T(v)      = sum_i t_i v^i         thrust (N)
    P_batt(v) = sum_i p_i v^i         power drawn from the battery chemistry, V_oc I_batt (W)
plus the static thrust T(0), the static battery current (the maximum), the largest rotor
speed over the nodes (report only: the top node is above the flight envelope), the rotor-speed
fit rpm(v) (check it at the actual top speed), and the system masses.
"""

import numpy as np
import openmdao.api as om

from .apc import load_prop

RPM2RADS = 2.0 * np.pi / 60.0
BATTERY_TOPOLOGIES = ("per_motor", "shared")


def motor_stations(n_motors, y_motor=None):
    """Spanwise stations of the propulsion units, as signed fractions of the semi-span.

    n_motors = 1: one centreline nose tractor, [0.0].
    n_motors = 2: a symmetric pair of wing-mounted tractors, [-y_motor, +y_motor].
    Built as a list of stations so further symmetric pairs can be added later.
    """
    if n_motors == 1:
        return [0.0]
    if n_motors == 2:
        if y_motor is None or not 0.0 < y_motor < 1.0:
            raise ValueError(f"n_motors = 2 needs y_motor (fraction of the semi-span) in (0, 1), got {y_motor!r}")
        return [-float(y_motor), float(y_motor)]
    raise ValueError(f"n_motors = {n_motors!r} is not supported: use 1 (nose tractor) or 2 (wing-mounted pair)")


class Powertrain(om.ExplicitComponent):
    def initialize(self):
        self.options.declare("prop", default="11x7E", desc="APC PER3 name (discrete choice, D4)")
        self.options.declare("apc_dir", default=None)
        self.options.declare("n_nodes", default=8, types=int)
        self.options.declare("v_top", default=40.0, desc="m/s, highest speed node (fit range [0, v_top])")
        self.options.declare("deg", default=3, types=int)
        self.options.declare("newton_iters", default=40, types=int)
        self.options.declare("n_motors", default=1, types=int, desc="identical propulsion units (1 or 2)")
        self.options.declare("battery_topology", default="per_motor", values=BATTERY_TOPOLOGIES,
                             desc="one pack per unit, or one pack shared by every ESC")

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
        V = np.stack([self._v**i for i in range(self.options["deg"] + 1)], axis=1)
        self._pinv = np.linalg.pinv(V)  # real: keeps the fit complex-step analytic
        d = self.options["deg"] + 1

        for name, val, units in [
            ("throttle", 1.0, None), ("rho", 1.18, "kg/m**3"),
            ("n_cells", 4.0, None), ("V_cell", 3.7, "V"), ("R_cell", 0.005, "ohm"),
            ("capacity", 3.0, "A*h"), ("usable_fraction", 0.8, None), ("e_spec", 150.0, "W*h/kg"),
            ("Kv", 900.0, "rpm/V"), ("Rm", 0.05, "ohm"), ("I0", 1.0, "A"),
            ("m_motor", 0.15, "kg"), ("m_esc", 0.06, "kg"), ("m_prop", 0.03, "kg"), ("m_avionics", 0.25, "kg"),
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
        elec = ["throttle", "rho", "n_cells", "V_cell", "R_cell", "Kv", "Rm", "I0"]
        for out in ["T_coeffs", "P_coeffs", "RPM_coeffs", "T_static", "T_static_unit", "I_static", "I_pack_static",
                    "rpm_max", "T_nodes", "P_nodes", "eta_prop_nodes"]:
            self.declare_partials(out, elec, method="cs")
        pack = ["n_cells", "V_cell", "capacity"]
        self.declare_partials("E_nom_total", pack, method="cs")
        self.declare_partials("E_usable", pack + ["usable_fraction"], method="cs")
        self.declare_partials("m_batt", pack + ["e_spec"], method="cs")
        unit = ["m_motor", "m_esc", "m_prop", "m_nacelle"]
        self.declare_partials("m_motor_unit", unit, val=1.0)
        self.declare_partials("m_motor_group", unit, val=self._n_units)
        self.declare_partials("m_sys", pack + ["e_spec", "m_avionics"] + unit, method="cs")

    def solve_nodes(self, x):
        p = self._prop
        D = p.diameter
        rho, dl = x["rho"], x["throttle"]
        Voc = x["n_cells"] * x["V_cell"]
        Rb = x["n_cells"] * x["R_cell"]
        kv = x["Kv"] * RPM2RADS
        Rtot = x["Rm"] + self._units_per_pack * dl**2 * Rb  # shared pack: n units sag one pack

        def residual(om_, v):
            I = (dl * Voc - om_ / kv) / Rtot
            Qm = (I - x["I0"]) / kv
            n = om_ / (2 * np.pi)
            J = v / (n * D)
            Qp = p.cp(J, om_ / RPM2RADS) * rho * n**2 * D**5 / (2 * np.pi)
            return Qm - Qp, I, n, J

        v = self._v
        om_ = 0.7 * kv * dl * Voc * np.ones_like(v)  # start below no-load speed
        h = 1e-3
        for _ in range(self.options["newton_iters"]):
            r, *_ = residual(om_, v)
            r2, *_ = residual(om_ + h, v)
            om_ = om_ - r / ((r2 - r) / h)
        _, I, n, J = residual(om_, v)
        T = p.ct(J, om_ / RPM2RADS) * rho * n**2 * D**4
        P_batt = Voc * dl * I
        P_shaft = p.cp(J, om_ / RPM2RADS) * rho * n**3 * D**5
        eta_prop = np.where(np.real(P_shaft) > 0, T * v / P_shaft, 0.0 * T)
        return dict(omega=om_, I_motor=I, T=T, P_batt=P_batt, eta_prop=eta_prop, J=J, Voc=Voc)

    def compute(self, inputs, outputs):
        x = {k: inputs[k] for k in inputs}
        r = self.solve_nodes(x)  # one unit
        nu = self._n_units
        T, P = nu * r["T"], nu * r["P_batt"]
        outputs["T_nodes"] = T
        outputs["P_nodes"] = P
        outputs["eta_prop_nodes"] = r["eta_prop"]
        tc = self._pinv @ T
        outputs["T_coeffs"] = tc
        outputs["P_coeffs"] = self._pinv @ P
        outputs["RPM_coeffs"] = self._pinv @ (r["omega"] / RPM2RADS)
        V = np.stack([self._v**i for i in range(len(tc))], axis=1)
        outputs["fit_residual_T"] = np.max(np.abs(np.real(V @ tc - T)))
        outputs["T_static"] = T[0]
        outputs["T_static_unit"] = r["T"][0]
        outputs["I_static"] = x["throttle"] * r["I_motor"][0]
        outputs["I_pack_static"] = self._units_per_pack * x["throttle"] * r["I_motor"][0]
        outputs["D_prop"] = self._prop.diameter
        outputs["rpm_max"] = np.max(np.real(r["omega"])) / RPM2RADS if not np.iscomplexobj(r["omega"]) else \
            r["omega"][np.argmax(np.real(r["omega"]))] / RPM2RADS
        outputs["rpm_limit"] = self._prop.rpm_limit
        E_nom = self._n_packs * x["n_cells"] * x["V_cell"] * x["capacity"] * 3600.0  # all packs, J
        outputs["E_nom_total"] = E_nom / 3600.0
        outputs["E_usable"] = x["usable_fraction"] * E_nom
        m_batt = E_nom / 3600.0 / x["e_spec"]
        outputs["m_batt"] = m_batt
        m_unit = x["m_motor"] + x["m_esc"] + x["m_prop"] + x["m_nacelle"]
        outputs["m_motor_unit"] = m_unit
        outputs["m_motor_group"] = nu * m_unit
        outputs["m_sys"] = m_batt + nu * m_unit + x["m_avionics"]
