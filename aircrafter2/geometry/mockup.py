"""Optional `mockup:` config section: how the 3D mock-up places what the framework does not define.

The framework sizes a rectangular wing, tails, a tail boom, a cargo-bay box and the propulsion units, but not their
vertical placement, the nacelle shape or the blades. These settings fill that gap for the OpenVSP mock-up only
(aircrafter2.vsp); they never enter the OpenMDAO model. Every entry is exported to geometry.json under
`mockup_assumptions` with its source. Woombah = USYD 2025 DBF design report, "Project Woombah".
"""

from typing import Literal

from pydantic import BaseModel, ConfigDict, Field

IN = 0.0254


class MockupInputs(BaseModel):
    model_config = ConfigDict(extra="forbid")
    wing_position: Literal["high"] = Field(
        "high", description="wing chord line on top of the cargo bay (Woombah Sec. 3.5: high-wing configuration)"
    )
    wing_incidence: Literal["zero", "cruise"] = Field(
        "zero",
        description="zero: no incidence (the framework models none); cruise: tilt the whole model to the "
        "cruise AoA for a flight-attitude view",
    )
    nacelle_drop_m: float = Field(
        0.8 * IN,
        description="m, thrust axis of the wing nacelles below the wing chord line: Woombah Sec. 5.3.2, "
        "firewall 0.8 in below the wing (nacelles under the wing, Sec. 4.3.2)",
    )
    fairings: bool = Field(False, description="illustrative nose and tail fairings on the cargo bay")
    markers: bool = Field(True, description="CG / NP spheres and the spar tube (never exported to STEP/STL)")
    n_blades: int = Field(2, description="illustrative: only the prop diameter and station come from the result")
    spinner_diameter_frac: float = Field(0.18, description="illustrative spinner diameter / prop diameter")


def mockup_assumptions(m: MockupInputs, n_motors: int) -> dict:
    """The mock-up entries the framework does not define: {name: {value, source, note}}."""
    out = {
        "wing_position": dict(
            value=m.wing_position,
            source="Woombah Sec. 3.5 ('high-wing, conventional')",
            note="wing chord line at z = 0 on top of the cargo bay; the framework has no z",
        ),
        "cargo_bay_z_m": dict(
            value="-h_cb/2 (centre)", source="illustrative", note="cargo bay hangs below the wing chord line"
        ),
        "boom_placement": dict(
            value="cargo-bay centreline",
            source="illustrative",
            note="G&G single-boom layout; the framework gives only start, length and radius",
        ),
        "tail_placement": dict(
            value="HT on top of the boom, VT on the HT centreline",
            source="illustrative",
            note="the framework gives tail areas, aspect ratios and the c/4 station only",
        ),
        "wing_incidence": dict(
            value=m.wing_incidence, source="config (mockup.wing_incidence)", note="the framework models no incidence"
        ),
        "blades": dict(value=m.n_blades, source="illustrative", note="blade shape illustrative; D and x from result"),
        "spinner_diameter_frac": dict(value=m.spinner_diameter_frac, source="illustrative", note=""),
        "fairings": dict(value=m.fairings, source="illustrative", note="off by default"),
    }
    if n_motors == 1:
        out["nose_motor_z"] = dict(
            value="cargo-bay centreline", source="illustrative", note="nose tractor on the fuselage axis"
        )
    else:
        out["nacelle_drop_m"] = dict(
            value=m.nacelle_drop_m,
            source="Woombah Sec. 5.3.2 (firewall 0.8 in below the wing) and Sec. 4.3.2 (nacelles under the wing)",
            note="read as the thrust axis 0.8 in below the wing chord line",
        )
        out["nacelle_shape"] = dict(
            value="streamlined pod l_nac x d_nac",
            source="illustrative",
            note="l_nac, d_nac from the result (PLACEHOLDERS in the config, Q29)",
        )
    return out
