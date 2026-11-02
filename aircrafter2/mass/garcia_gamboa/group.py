"""Garcia & Gamboa (G&G 2022) structure-based mass model, circular tube spar wing concept: the assembled group.

    Allowables -> SectionCoeffs -> WingLoads(m) -> CriticalLoads -> TubeSparSizing -> RibSizing -> WingMass -> TailMass
    TailBoomLoads -> TailBoomSizing -> FuselageMass
    MassSum(m) -> m_total  --(closure)-->  m

Inputs come from geometry, airfoil, aero and the config; m_total feeds the MDA's lift balance (problems/full_mdo.py).
Details: docs/modules/mass.md.
"""

import numpy as np
import openmdao.api as om

from ...config.mass import MassConfig, MassOptions
from ..statistical import StatisticalMass, statistical_mass
from .inputs import INPUTS, active_inputs
from .loads import CriticalLoads, TailBoomLoads, WingLoads
from .masses import FuselageMass, GearMass, MassSum, TailMass, WingMass
from .materials import Allowables
from .section import SectionCoeffs, TubeFitMargin, TubeRadius
from .sizing import RibSizing, TailBoomSizing, TubeSparSizing

MASS_OUTPUTS = [
    "m_tube",
    "m_rib",
    "m_le",
    "m_te",
    "m_film",
    "m_wing",
    "m_ht",
    "m_vt",
    "m_tb",
    "m_cb",
    "m_fus",
    "m_gear",
    "m_empty",
    "m_total",
    "m_stat",
]


class GarciaGamboaMass(om.Group):
    """The mass group. All promoted.

    The closure m = m_total (Eqs. 39-41, with the loads depending on W = m g) is a coupled cycle, converged by the
    enclosing nonlinear solver: NonlinearBlockGS + DirectSolver when ``standalone_solver``, the MDF (multidisciplinary
    feasible) solver in the full framework. No component iterates internally. The statistical model (Eq. 124) gives the
    initial guess for m.

    Options
    -------
    config : MassOptions or None
    external_sizing : bool
        The loads see an external sizing mass m, with no closure: a mission point flying an airframe sized at another
        point.

    Inputs
    ------
    Everything in inputs.INPUTS.

    Outputs
    -------
    MASS_OUTPUTS [kg]: m_wing, m_ht, m_vt, m_fus, m_gear, m_empty, m_total, ...
    """

    def initialize(self):
        self.options.declare("config", default=None, types=(MassOptions, type(None)))
        self.options.declare(
            "external_sizing",
            default=False,
            types=bool,
            desc="the loads see an external sizing mass m (no closure m = m_total): a mission point "
            "that flies an airframe sized at another point (problems/multipoint.py)",
        )

    def setup(self):
        opts = self.options["config"] or MassOptions()
        self._opts = opts
        self._guessed = False
        smooth = dict(max_mode=opts.max_mode, rho_ks=opts.rho_ks)

        self.add_subsystem("allowables", Allowables(), promotes=["*"])
        if opts.r_tube_mode == "derived":
            self.add_subsystem("tube_radius", TubeRadius(), promotes=["*"])
        self.add_subsystem("section", SectionCoeffs(), promotes=["*"])
        self.add_subsystem(
            "wing_loads",
            WingLoads(
                n_motor_pairs=opts.n_motor_pairs,
                motor_inertia_relief=opts.motor_inertia_relief,
                nacelle_torsion=opts.nacelle_torsion,
            ),
            promotes=["*"],
        )
        self.add_subsystem("critical_loads", CriticalLoads(**smooth), promotes=["*"])
        self.add_subsystem(
            "spar_sizing",
            TubeSparSizing(
                **smooth,
                softplus_scale=opts.softplus_scale,
                deflection_mode=opts.deflection_mode,
                torsion_stiffness_mode=opts.torsion_stiffness_mode,
                include_twist=opts.include_twist,
                use_min_gauge=opts.use_min_gauge,
                sizing_radius_mode=opts.sizing_radius_mode,
            ),
            promotes=["*"],
        )
        self.add_subsystem("rib_sizing", RibSizing(**smooth), promotes=["*"])
        self.add_subsystem("wing_mass", WingMass(n_panels=opts.n_panels), promotes=["*"])
        self.add_subsystem("tail_mass", TailMass(tail_mass_model=opts.tail_mass_model), promotes=["*"])
        self.add_subsystem("boom_loads", TailBoomLoads(load_mode=opts.tail_boom_load_mode), promotes=["*"])
        self.add_subsystem("boom_sizing", TailBoomSizing(**smooth, use_min_gauge=opts.tb_min_gauge), promotes=["*"])
        self.add_subsystem("fuselage_mass", FuselageMass(), promotes=["*"])
        self.add_subsystem("gear_mass", GearMass(), promotes=["*"])
        self.add_subsystem("mass_sum", MassSum(), promotes=["*"])
        self.add_subsystem("statistical", StatisticalMass(model=opts.statistical_model), promotes=["*"])
        if opts.r_tube_mode == "design_variable":
            self.add_subsystem("tube_fit", TubeFitMargin(), promotes=["*"])

        # mass closure: the loads and the gear mass see the converged total mass
        if not self.options["external_sizing"]:
            self.connect("m_total", "m")

        for name in active_inputs(opts):
            units, val, _ = INPUTS[name]
            self.set_input_defaults(name, val=val, units=units)

        if opts.standalone_solver:
            # The closure is weakly coupled (d m_total / d m ~ 0.05), so block Gauss-Seidel converges
            # in a few iterations from any start; Newton from the cold-start outputs diverges.
            # constraints_view.REGISTER matches this text line by line
            # fmt: off
            self.nonlinear_solver = om.NonlinearBlockGS(maxiter=50, atol=1e-12, rtol=1e-12, use_aitken=True,
                                                        iprint=-1, err_on_non_converge=True)
            # fmt: on
            self.linear_solver = om.DirectSolver()

    def guess_nonlinear(self, inputs, outputs, residuals):
        """Initial guess of the closure from the statistical model (Eqs. 124-126), first solve only."""
        if not self._guessed and not self.options["external_sizing"]:
            m_pay = inputs["statistical.m_pay"]  # computed payloads (stores) may not be transferred yet: floor it
            outputs["m_total"] = statistical_mass(
                inputs["statistical.b"],
                inputs["statistical.c_mean"],
                m_pay if np.real(m_pay).min() > 0.0 else 0.5 + 0.0 * m_pay,
                self._opts.statistical_model,
            )
            self._guessed = True


def set_mass_inputs(prob, values, prefix=""):
    """Set mass-group inputs on a set-up problem.

    Parameters
    ----------
    prob : om.Problem
    values : dict
        {name from INPUTS: SI value}.
    prefix : str
        Path to the mass group.
    """
    for name, val in values.items():
        units = INPUTS[name][0]
        prob.set_val(prefix + name, val, units=units)


def build_mass_problem(config: MassConfig, force_alloc_complex=False):
    """Standalone problem: just the mass group, with inputs set from the config.

    Parameters
    ----------
    config : MassConfig
    force_alloc_complex : bool
        For complex-step checks.

    Returns
    -------
    om.Problem
        Set up. Valid names that the chosen options don't use (e.g. k_fill with a fixed radius) are skipped.
    """
    prob = om.Problem(reports=False)
    prob.model.add_subsystem("mass", GarciaGamboaMass(config=config.options), promotes=["*"])
    prob.setup(force_alloc_complex=force_alloc_complex)
    # Valid names that the chosen options don't use (e.g. k_fill with a fixed radius) are skipped.
    active = set(active_inputs(config.options))
    set_mass_inputs(prob, {k: v for k, v in config.inputs.items() if k in active})
    return prob


def mass_breakdown(prob, prefix=""):
    """Component masses after a run.

    Parameters
    ----------
    prob : om.Problem
    prefix : str
        Path to the mass group.

    Returns
    -------
    dict
        {name in MASS_OUTPUTS: mass in kg}.
    """
    return {name: float(prob.get_val(prefix + name, units="kg")[0]) for name in MASS_OUTPUTS}
