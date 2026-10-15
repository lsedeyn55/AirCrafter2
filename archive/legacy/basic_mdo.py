import openmdao.api as om


# --- Discipline 1 ---
class Discipline1(om.ExplicitComponent):
    def setup(self):
        self.add_input("x", val=1.0)
        self.add_input("y2", val=1.0)
        self.add_output("y1", val=1.0)
        self.declare_partials(of="*", wrt="*", method="fd")

    def compute(self, inputs, outputs):
        outputs["y1"] = 2.0 * inputs["x"] + 0.5 * inputs["y2"]


# --- Discipline 2 ---
class Discipline2(om.ExplicitComponent):
    def setup(self):
        self.add_input("x", val=1.0)
        self.add_input("y1", val=1.0)
        self.add_output("y2", val=1.0)
        self.declare_partials(of="*", wrt="*", method="fd")

    def compute(self, inputs, outputs):
        outputs["y2"] = 3.0 * inputs["x"] - 0.2 * inputs["y1"]


# --- Objective Function ---
class ObjectiveComp(om.ExplicitComponent):
    def setup(self):
        self.add_input("y1", val=1.0)
        self.add_input("y2", val=1.0)
        self.add_output("obj", val=1.0)
        self.declare_partials(of="*", wrt="*", method="fd")

    def compute(self, inputs, outputs):
        outputs["obj"] = (inputs["y1"] - 5.0) ** 2 + inputs["y2"] ** 2


# --- Problem Assembly ---
prob = om.Problem()

# Coupled Subsystem (MDF Group)
coupled_group = prob.model.add_subsystem("coupled", om.Group(), promotes=["*"])
coupled_group.add_subsystem("d1", Discipline1(), promotes=["*"])
coupled_group.add_subsystem("d2", Discipline2(), promotes=["*"])

coupled_group.nonlinear_solver = om.NewtonSolver(solve_subsystems=False)
coupled_group.nonlinear_solver.options["iprint"] = 0
coupled_group.nonlinear_solver.options["maxiter"] = 20
coupled_group.linear_solver = om.DirectSolver()

# Objective Subsystem
prob.model.add_subsystem("obj_comp", ObjectiveComp(), promotes=["*"])

# Optimization Setup
prob.driver = om.ScipyOptimizeDriver()
prob.driver.options["optimizer"] = "SLSQP"

prob.model.add_design_var("x", lower=-10.0, upper=10.0)
prob.model.add_objective("obj")
prob.model.add_constraint("y1", lower=0.0)

prob.setup()
prob.set_val("x", 2.0)
prob.run_driver()

print(f"Optimal x:   {prob.get_val('x')[0]:.4f}")
print(f"Optimal y1:  {prob.get_val('y1')[0]:.4f}")
print(f"Optimal y2:  {prob.get_val('y2')[0]:.4f}")
print(f"Min Obj:     {prob.get_val('obj')[0]:.4f}")
