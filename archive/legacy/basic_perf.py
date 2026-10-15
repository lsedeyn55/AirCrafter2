import numpy as np
import openmdao.api as om
from openaerostruct.aerodynamics.aero_groups import AeroPoint
from openaerostruct.geometry.geometry_group import Geometry
from openaerostruct.meshing.mesh_generator import generate_mesh


# ===========================================================
# Custom components: trim + lap time
# ===========================================================
class RequiredCL(om.ExplicitComponent):
    """CL needed for level flight at fixed speed, given current wing area."""

    def setup(self):
        self.add_input("weight", val=200.0, units="N")
        self.add_input("rho", val=1.225, units="kg/m**3")
        self.add_input("v", val=35.0, units="m/s")
        self.add_input("S_ref", val=0.45, units="m**2")
        self.add_output("CL_req", val=0.8)
        self.declare_partials("CL_req", ["weight", "rho", "v", "S_ref"], method="cs")

    def compute(self, inputs, outputs):
        W = inputs["weight"]
        rho = inputs["rho"]
        v = inputs["v"]
        S = inputs["S_ref"]
        outputs["CL_req"] = W / (0.5 * rho * v**2 * S)


class LapTime(om.ExplicitComponent):
    """Rudimentary oval-course lap time: 2 straights + 2 constant-radius turns,
    flown at fixed speed. Turn radius is set by the load factor available
    before hitting CL_max."""

    def setup(self):
        self.add_input("v", val=35.0, units="m/s")
        self.add_input("straight_length", val=200.0, units="m")
        self.add_input("CL", val=0.8)
        self.add_input("CL_max", val=1.3)

        self.add_output("load_factor_max", val=2)
        self.add_output("turn_radius", val=20.0, units="m")
        self.add_output("lap_time", val=10.0, units="s")

        self.declare_partials("*", "*", method="cs")

    def compute(self, inputs, outputs):
        g = 9.81
        v = inputs["v"]
        CL = inputs["CL"]
        CL_max = inputs["CL_max"]

        n_max = CL_max / CL
        outputs["load_factor_max"] = n_max

        n_eff = np.where(n_max > 1.01, n_max, 1.01)
        R = v**2 / (g * np.sqrt(n_eff**2 - 1))
        outputs["turn_radius"] = R

        lap_distance = 2 * inputs["straight_length"] + 2 * np.pi * R
        outputs["lap_time"] = lap_distance / v


# ===========================================================
# 1. Rectangular (taperable) wing mesh
# ===========================================================
mesh_dict = {
    "num_y": 15,
    "num_x": 5,
    "wing_type": "rect",
    "symmetry": True,
    "span": 1.8,
    "root_chord": 0.25,
}
mesh = generate_mesh(mesh_dict)

surface = {
    "name": "wing",
    "symmetry": True,
    "S_ref_type": "wetted",
    "mesh": mesh,
    "twist_cp": np.zeros(1),
    "chord_cp": np.ones(1),
    "taper": 1.0,
    "span": mesh_dict["span"],
    "CL0": 0.0,
    "CD0": 0.015,
    "k_lam": 0.05,
    "t_over_c_cp": np.array([0.12]),
    "c_max_t": 0.303,
    "with_viscous": True,
    "with_wave": False,
    "fem_model_type": "tube",
}

# ===========================================================
# 2. OpenMDAO problem
# ===========================================================
prob = om.Problem()
model = prob.model

indep = om.IndepVarComp()
indep.add_output("v", val=35.0, units="m/s")
indep.add_output("alpha", val=3.0, units="deg")
indep.add_output("Mach_number", val=0.044)
indep.add_output("re", val=3.0e5, units="1/m")
indep.add_output("rho", val=1.225, units="kg/m**3")
indep.add_output("cg", val=np.zeros(3), units="m")
model.add_subsystem("prob_vars", indep, promotes=["*"])

geom_group = Geometry(surface=surface)
model.add_subsystem(surface["name"], geom_group)

point_name = "aero_point_0"
aero_group = AeroPoint(surfaces=[surface])
model.add_subsystem(
    point_name,
    aero_group,
    promotes_inputs=["v", "alpha", "Mach_number", "re", "rho", "cg"],
)

name = surface["name"]
model.connect(f"{name}.mesh", f"{point_name}.{name}.def_mesh")
model.connect(f"{name}.mesh", f"{point_name}.aero_states.{name}_def_mesh")
model.connect(f"{name}.t_over_c", f"{point_name}.{name}_perf.t_over_c")

# ---- Trim + lap time subsystems ----
model.add_subsystem("trim", RequiredCL(), promotes_inputs=["v", "rho"])
model.add_subsystem("lap", LapTime(), promotes_inputs=["v"])
model.add_subsystem("cl_match", om.ExecComp("cl_error = CL - CL_req"))

model.connect(f"{point_name}.{name}.S_ref", "trim.S_ref")
model.connect(f"{point_name}.wing_perf.CL", "cl_match.CL")
model.connect("trim.CL_req", "cl_match.CL_req")
model.connect(f"{point_name}.wing_perf.CL", "lap.CL")

# ---- Placeholder mission values -- UPDATE these to your real numbers ----
model.set_input_defaults("trim.weight", val=20.0, units="N")  # aircraft weight
model.set_input_defaults("lap.straight_length", val=100.0, units="m")  # course geometry
model.set_input_defaults("lap.CL_max", val=1.3)  # airfoil CL_max w/ margin

# ===========================================================
# 3. Driver, DVs, objective, constraints
# ===========================================================
prob.driver = om.ScipyOptimizeDriver()
prob.driver.options["optimizer"] = "SLSQP"
prob.driver.options["tol"] = 1e-7
prob.driver.options["disp"] = True

model.add_design_var("wing.span", lower=1.0, upper=3.0)
model.add_design_var("wing.chord_cp", lower=0.5, upper=2.0)
model.add_design_var("wing.taper", lower=0.5, upper=1.0)
model.add_design_var("alpha", lower=-3.0, upper=12.0)

model.add_constraint("cl_match.cl_error", equals=0.0)
model.add_objective("lap.lap_time")

prob.setup()
prob.run_driver()

print(f"Span: {prob['wing.span'][0]:.3f} m")
print(f"Chord scale: {prob['wing.chord_cp'][0]:.3f}")
print(f"Taper: {prob['wing.taper'][0]:.3f}")
print(f"Alpha: {prob['alpha'][0]:.3f} deg")
print(f"CL: {prob[f'{point_name}.wing_perf.CL'][0]:.4f}")
print(f"CD: {prob[f'{point_name}.wing_perf.CD'][0]:.5f}")
print(f"S_ref: {prob[f'{point_name}.{name}.S_ref'][0]:.4f} m^2")
print(f"Turn radius: {prob['lap.turn_radius'][0]:.2f} m")
print(f"Load factor margin: {prob['lap.load_factor_max'][0]:.2f}")
print(f"Lap time: {prob['lap.lap_time'][0]:.2f} s")
