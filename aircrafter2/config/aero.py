"""Schemas for the airfoil and aerodynamics sections of a config file."""

from typing import Literal

from pydantic import BaseModel, ConfigDict, Field

from ..airfoil.neuralfoil_section import NFSettings


class AirfoilConfig(BaseModel):
    model_config = ConfigDict(extra="forbid")

    seed: str = Field("naca4412", description="AeroSandbox airfoil name or path to a .dat file")
    n_modes: int = Field(0, ge=0, le=8, description="local Bernstein bump modes per surface (D3)")
    te_thickness: float | None = Field(None, ge=0.0, description="TE thickness / c; None keeps the seed's")


class AeroConfig(BaseModel):
    model_config = ConfigDict(extra="forbid")

    num_x: int = Field(2, ge=2, description="chordwise VLM nodes (2 = one panel)")
    num_y: int = Field(41, ge=5, description="spanwise VLM nodes on the half wing; 41 is within ~1 % "
                       "of the converged span efficiency (docs/aero_verification.md)")
    spacing: Literal["cosine", "uniform"] = "cosine"
    max_mode: Literal["ks", "hard"] = "ks"
    rho_ks: float = 100.0
    model_size: str = Field("xlarge", description="NeuralFoil network size")
    n_crit: float = Field(9.0, description="e^N transition amplification (XFoil convention)")
    alpha_min: float = -8.0
    alpha_max: float = 25.0
    alpha_step: float = 0.5
    h_alpha: float = 1e-4  # deg; step study: docs/neuralfoil_fd_steps.md
    h_logre: float = 1e-5
    h_k: float = 1e-6

    def nf_settings(self):
        return NFSettings(model_size=self.model_size, n_crit=self.n_crit, alpha_min=self.alpha_min,
                          alpha_max=self.alpha_max, alpha_step=self.alpha_step, h_alpha=self.h_alpha,
                          h_logre=self.h_logre, h_k=self.h_k)
