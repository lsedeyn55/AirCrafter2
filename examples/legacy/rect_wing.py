import numpy as np
import openmdao.api as om
from openaerostruct.aerodynamics.aero_groups import AeroPoint
from openaerostruct.geometry.geometry_group import Geometry
from openaerostruct.meshing.mesh_generator import generate_mesh

# ---------------------------------------------------------
# 1. Rectangular wing mesh + geometry DV setup
# ---------------------------------------------------------
mesh_dict = {
    "num_y": 15,
    "num_x": 5,
    "wing_type": "rect",
    "symmetry": True,
    "span": 1.8,  # m, initial guess
    "root_chord": 0.25,  # m, initial guess
}
mesh = generate_mesh(mesh_dict)  # "rect" type returns mesh only, no twist_cp

surface = {
    "name": "wing",
    "symmetry": True,
    "S_ref_type": "wetted",
    "mesh": mesh,
    # Geometric DVs OAS will build IndepVarComps for:
    "twist_cp": np.zeros(1),  # no twist -> keep untwisted
    "chord_cp": np.ones(1),  # single CP -> uniform scaling, stays rectangular
    "taper": 1.0,  # rectangular, not tapered
    "span": mesh_dict["span"],
    # Aero properties
    "CL0": 0.0,
    "CD0": 0.015,
    "k_lam": 0.05,
    "t_over_c_cp": np.array([0.12]),
    "c_max_t": 0.303,
    "with_viscous": True,
    "with_wave": False,
    "fem_model_type": "tube",
}

# ---------------------------------------------------------
# 2. OpenMDAO problem
# ---------------------------------------------------------
prob = om.Problem()
model = prob.model

indep = om.IndepVarComp()
indep.add_output("v", val=15.0, units="m/s")
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

# ---------------------------------------------------------
# 3. Driver, DVs, objective, constraints
# ---------------------------------------------------------
prob.driver = om.ScipyOptimizeDriver()
prob.driver.options["optimizer"] = "SLSQP"
prob.driver.options["tol"] = 1e-7
prob.driver.options["disp"] = True

target_CL = 0.5  # pick this to match your cruise condition

model.add_design_var("wing.span", lower=1.0, upper=3.0)
model.add_design_var("wing.chord_cp", lower=0.5, upper=2.0)
model.add_design_var("wing.taper", lower=0.8, upper=1.0)
model.add_design_var("alpha", lower=-3.0, upper=12.0)

model.add_objective(f"{point_name}.wing_perf.CD", scaler=1e2)
model.add_constraint(f"{point_name}.wing_perf.CL", equals=target_CL)
# model.add_constraint(
#     f"{point_name}.{name}.S_ref", equals=mesh_dict["span"] * mesh_dict["root_chord"] * (1+["taper"])
# )
baseline_area = (
    mesh_dict["span"] * mesh_dict["root_chord"]
)  # area of the initial (taper=1) rectangular wing
model.add_constraint(f"{point_name}.{name}.S_ref", equals=baseline_area)

prob.setup()
prob.run_driver()

print(f"Span: {prob['wing.span'][0]:.3f} m")
print(f"Chord scale: {prob['wing.chord_cp'][0]:.3f}")
print(f"Alpha: {prob['alpha'][0]:.3f} deg")
print(f"Taper: {prob['wing.taper'][0]:.3f}")
print(f"CL: {prob[f'{point_name}.wing_perf.CL'][0]:.4f}")
print(f"CD: {prob[f'{point_name}.wing_perf.CD'][0]:.5f}")
print(
    f"L/D: {prob[f'{point_name}.wing_perf.CL'][0] / prob[f'{point_name}.wing_perf.CD'][0]:.2f}"
)
