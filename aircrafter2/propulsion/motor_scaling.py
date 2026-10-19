"""Motor rewind: the same frame wound for a different Kv (used when Kv is a design variable).

The config's motor (Kv, Rm, I0) is a reference motor (Kv_ref, Rm_ref, I0_ref). Rewinding the same stator with
N turns per phase at the same copper volume (same slot fill):
    Kv ∝ 1/N                                        (back-EMF constant ∝ N)
    Rm ∝ N² ∝ 1/Kv²                                 (wire length ∝ N, cross-section ∝ 1/N)
    I0 = Q0 Kv, with the no-load torque Q0 held     (same iron, same bearings: same no-load loss torque)
so
    Kv_eff = Kv · Kv_factor
    Rm     = Rm_ref (Kv_ref / Kv_eff)²
    I0     = I0_ref Kv_eff / Kv_ref
and the motor constant Km = 1 / (Kv √Rm) is unchanged. The mass is unchanged (same frame). Kv_factor is a
multiplicative winding tolerance (1 except at robust-design nodes), applied before the scaling so that a Kv error
moves Rm and I0 consistently. Derivation and assumptions: docs/modules/propulsion.md.
"""

import openmdao.api as om


class MotorRewind(om.ExplicitComponent):
    def setup(self):
        self.add_input("Kv", 900.0, units="rpm/V", desc="design Kv (the design variable)")
        self.add_input("Kv_factor", 1.0, desc="winding tolerance factor (RDO uncertain parameter; 1 nominal)")
        self.add_input("Kv_ref", 900.0, units="rpm/V", desc="reference motor Kv (config propulsion.Kv)")
        self.add_input("Rm_ref", 0.05, units="ohm", desc="reference motor resistance (config propulsion.Rm)")
        self.add_input("I0_ref", 1.0, units="A", desc="reference motor no-load current (config propulsion.I0)")
        self.add_output("Kv_eff", 900.0, units="rpm/V", desc="Kv passed to the powertrain")
        self.add_output("Rm", 0.05, units="ohm", desc="winding resistance of the rewound motor")
        self.add_output("I0", 1.0, units="A", desc="no-load current of the rewound motor")
        self.declare_partials("Kv_eff", ["Kv", "Kv_factor"])
        self.declare_partials("Rm", ["Kv", "Kv_factor", "Kv_ref", "Rm_ref"])
        self.declare_partials("I0", ["Kv", "Kv_factor", "Kv_ref", "I0_ref"])

    def compute(self, inputs, outputs):
        kv = inputs["Kv"] * inputs["Kv_factor"]
        outputs["Kv_eff"] = kv
        outputs["Rm"] = inputs["Rm_ref"] * (inputs["Kv_ref"] / kv) ** 2
        outputs["I0"] = inputs["I0_ref"] * kv / inputs["Kv_ref"]

    def compute_partials(self, inputs, J):
        Kv, f, Kr = inputs["Kv"], inputs["Kv_factor"], inputs["Kv_ref"]
        Rr, Ir = inputs["Rm_ref"], inputs["I0_ref"]
        kv = Kv * f
        Rm = Rr * (Kr / kv) ** 2
        J["Kv_eff", "Kv"] = f
        J["Kv_eff", "Kv_factor"] = Kv
        dRm_dkv = -2.0 * Rm / kv
        J["Rm", "Kv"] = dRm_dkv * f
        J["Rm", "Kv_factor"] = dRm_dkv * Kv
        J["Rm", "Kv_ref"] = 2.0 * Rr * Kr / kv**2
        J["Rm", "Rm_ref"] = (Kr / kv) ** 2
        J["I0", "Kv"] = Ir * f / Kr
        J["I0", "Kv_factor"] = Ir * Kv / Kr
        J["I0", "Kv_ref"] = -Ir * kv / Kr**2
        J["I0", "I0_ref"] = kv / Kr
