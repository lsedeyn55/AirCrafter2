"""Schema for the mass model section of a config file."""

from typing import Literal

from pydantic import BaseModel, ConfigDict, Field, field_validator


class MassOptions(BaseModel):
    """Structural switches (not differentiated)."""

    model_config = ConfigDict(extra="forbid")

    max_mode: Literal["ks", "hard"] = Field("ks", description="smooth (KS) or exact max in the sizing rules")
    rho_ks: float = Field(100.0, gt=0, description="KS sharpness; over-estimate <= n**(1/rho)")
    softplus_scale: float = Field(1e-6, gt=0, description="m; width of the smooth floor on t_div")
    deflection_mode: Literal["paper", "exact"] = "paper"
    torsion_stiffness_mode: Literal["paper", "bredt"] = "paper"
    tail_boom_load_mode: Literal["paper", "eq54"] = Field(
        "paper", description="paper: Eq. 55 with Eq. 5 as printed (reproduces Table 15; f_tb calibrated with it)")
    sizing_radius_mode: Literal["paper", "physical"] = Field(
        "paper", description="paper: K_tube = r/c_mean times c_root (reproduces Table 15); physical: r")
    include_twist: bool = Field(False, description="add Eq. 116 (G&G Sec. 4.2 did not use it)")
    use_min_gauge: bool = Field(True, description="spar tube minimum wall thickness t_tube_min")
    tb_min_gauge: bool = Field(True, description="tail boom minimum wall thickness t_tb_min; f_tb must be "
                               "calibrated on the same basis (docs/studies/tail_boom_calibration.md)")
    r_tube_mode: Literal["fixed", "derived", "design_variable"] = "derived"
    n_panels: int = Field(1, ge=1)
    tail_mass_model: Literal["gg", "areal"] = Field(
        "gg", description="gg: m_ht, m_vt = f S/S_wing m_wing (Eqs. 34-35); areal: m = a S (kg/m^2, not G&G; "
        "docs/studies/mass_calibration_usyd.md)")
    statistical_model: Literal["all", "balsa", "composite"] = "all"
    motor_inertia_relief: bool = Field(False, description="wing-mounted units relieve root bending and shear "
                                       "(flight and ground test); off = the published G&G model")
    nacelle_torsion: bool = Field(True, description="wing-mounted units ahead of the quarter chord add root "
                                  "torsion (flight and ground test); no effect without wing units")
    n_motor_pairs: Literal[0, 1] = Field(0, description="set by the full model from propulsion.n_motors")
    standalone_solver: bool = Field(True, description="attach NonlinearBlockGS + DirectSolver for the mass closure")


class MassConfig(BaseModel):
    model_config = ConfigDict(extra="forbid")

    options: MassOptions = MassOptions()
    inputs: dict[str, float] = {}
    notes: dict[str, str] = {}

    @field_validator("inputs")
    @classmethod
    def _known_inputs(cls, v):
        from ..mass.garcia_gamboa.inputs import INPUTS

        unknown = sorted(set(v) - set(INPUTS))
        if unknown:
            raise ValueError(f"unknown mass input(s) {unknown}; valid names are listed in "
                             "aircrafter2/mass/garcia_gamboa/inputs.py")
        return v


class MassVerificationCase(MassConfig):
    """A MassConfig plus the reference masses to compare against."""

    name: str
    source: str
    expected: dict[str, float]
