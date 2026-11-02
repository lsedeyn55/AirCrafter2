"""Every input of the Garcia & Gamboa mass group: units, a neutral default and a description.

All values are SI. The defaults here only make the group runnable; the cited values live in
configs/mass_garcia_gamboa_2022.yaml. Config files are checked against this table, so a misspelt key is an error, not a
silently ignored value. Used by config/mass.py and garcia_gamboa/group.py. See docs/modules/mass.md.
"""

INPUTS = {
    # --- geometry (from the geometry module in the full framework) ---
    "b": ("m", 3.0, "wingspan"),
    "c_mean": ("m", 0.3, "wing mean geometric chord"),
    "c_root": ("m", 0.35, "wing root chord (structural sizing chord, Sec. 3.3)"),
    "S_wing": ("m**2", 0.9, "wing planform reference area"),
    "tc": (None, 0.12, "aerofoil thickness-to-chord ratio"),
    "x_spar": (None, 0.25, "spar position as a chord fraction"),
    "winglet_ratio": (None, 0.0, "S_winglet / S_wing (Eq. 32)"),
    "y1": ("m", 0.0, "first panel-break position from the root (Eq. 110)"),
    "l_ht": ("m", 1.0, "tail arm"),
    "l_tb": ("m", 1.0, "tail boom length"),
    "S_ht": ("m**2", 0.1, "horizontal tail area"),
    "S_vt": ("m**2", 0.08, "vertical tail area"),
    "S_cb": ("m**2", 0.2, "cargo bay surface area (Eq. 37)"),
    "r_tb": ("m", 0.0125, "tail boom outer radius"),
    # --- cross-section (from the airfoil module) ---
    "A_c2": (None, 0.08, "aerofoil area / chord^2"),
    "P_c": (None, 2.07, "aerofoil perimeter / chord"),
    "le_ratio": (None, 0.021, "A_le / A_aerofoil"),
    "te_ratio": (None, 0.021, "A_te / A_aerofoil"),
    "c_rib_frac": (None, 0.7, "aft rib length / chord (Sec. 3.3.3)"),
    # --- spar tube radius: r_tube_mode = fixed | design_variable ---
    "r_tube": ("m", 0.012, "spar tube outer radius"),
    # --- spar tube radius: r_tube_mode = derived | design_variable ---
    "t_at_spar": (None, 0.12, "local aerofoil thickness / chord at the spar"),
    "k_fill": (None, 0.85, "allowed tube diameter / local section thickness"),
    "clearance": ("m", 0.001, "radial clearance between tube and skin"),
    # --- flight condition and loads ---
    "n": (None, 2.0, "limit load factor"),
    "V_design": ("m/s", 15.0, "maximum level design speed"),
    "rho": ("kg/m**3", 1.225, "air density"),
    "g": ("m/s**2", 9.80665, "gravitational acceleration"),
    "Cm": (None, -0.1, "aerofoil/wing pitching moment coefficient"),
    "CL_alpha": ("1/rad", 5.0, "lift-curve slope used in Eq. 47"),
    "div_speed_factor": (None, 1.2, "divergence speed / design speed (after Eq. 47)"),
    "x_gd": (None, 0.25, "ground-test hold point as a chord fraction (Eq. 50)"),
    # --- stiffness limits and minimum gauge ---
    "delta_tip_ratio": (None, 0.1, "max tip deflection / span (Eq. 110)"),
    "theta_tip_max": ("rad", 0.0349, "max tip twist (Eq. 116; only with include_twist)"),
    "theta_tb_max": ("rad", 0.017453, "max tail boom rotation at the tail (Eq. 121)"),
    "t_tube_min": ("m", 5e-4, "spar tube minimum gauge (only with use_min_gauge)"),
    "t_tb_min": ("m", 1e-3, "tail boom minimum wall thickness (only with tb_min_gauge)"),
    # --- materials (ultimate values; Eq. 42 divides strengths by SF) ---
    "SF": (None, 1.5, "safety factor (Eq. 42)"),
    "sigma_spar": ("Pa", 720e6, "spar direct strength"),
    "tau_spar": ("Pa", 54e6, "spar shear strength"),
    "E_spar": ("Pa", 84e9, "spar Young's modulus"),
    "G_spar": ("Pa", 5e9, "spar shear modulus"),
    "rho_spar": ("kg/m**3", 1600.0, "spar density"),
    "sigma_balsa": ("Pa", 10e6, "balsa direct strength (ribs)"),
    "tau_balsa": ("Pa", 1.4e6, "balsa shear strength (ribs)"),
    "rho_balsa": ("kg/m**3", 200.0, "balsa density (ribs, LE/TE stringers)"),
    "gamma_film": ("kg/m**2", 0.036, "covering film areal mass"),
    "sigma_tb": ("Pa", 720e6, "tail boom direct strength"),
    "tau_tb": ("Pa", 54e6, "tail boom shear strength"),
    "E_tb": ("Pa", 84e9, "tail boom Young's modulus"),
    "rho_tb": ("kg/m**3", 1600.0, "tail boom density"),
    # --- correction factors (Table 14) ---
    "f_wing": (None, 1.12, "wing correction factor per panel joint (Eq. 32)"),
    "f_tb": (None, 12.81, "tail boom to full fuselage length factor (Eq. 36)"),
    "f_ht": (None, 0.6411, "horizontal tail factor (Eq. 34)"),
    "f_vt": (None, 0.3414, "vertical tail factor (Eq. 35)"),
    "f_cb": ("kg/m**2", 0.7878, "cargo bay areal mass factor (Eq. 37)"),
    "f_gear": (None, 0.0089, "landing gear mass fraction (Eq. 39)"),
    # --- construction factors (not G&G; default 1 / unused: the published model) ---
    "k_wing": (None, 1.0, "wing construction factor on Eq. 32, whatever n_panels (not G&G; default 1)"),
    "a_ht": ("kg/m**2", 1.0, "horizontal tail areal mass (tail_mass_model: areal; not G&G)"),
    "a_vt": ("kg/m**2", 1.0, "vertical tail areal mass (tail_mass_model: areal; not G&G)"),
    # --- fixed masses ---
    "m_sys": ("kg", 0.65, "systems mass: propulsion, battery, servos, receiver"),
    "m_pay": ("kg", 1.0, "payload mass"),
    "m_ballast": ("kg", 0.0, "robustness margin added to the empty mass (not G&G; default 0)"),
    "f_build": (
        None,
        1.0,
        "build-mass factor on the whole structure: wing, tails, boom, cargo bay, gear (not G&G; default 1)",
    ),
}


def active_inputs(opts):
    """Names of the group inputs that exist for a given MassOptions.

    Parameters
    ----------
    opts : MassOptions

    Returns
    -------
    list of str
    """
    skip = set()
    if not opts.include_twist:
        skip.add("theta_tip_max")
    if not opts.use_min_gauge:
        skip.add("t_tube_min")
    if not opts.tb_min_gauge:
        skip.add("t_tb_min")
    if opts.r_tube_mode == "fixed":
        skip |= {"t_at_spar", "k_fill", "clearance"}
    elif opts.r_tube_mode == "derived":
        skip.add("r_tube")
    skip |= {"f_ht", "f_vt"} if opts.tail_mass_model == "areal" else {"a_ht", "a_vt"}
    return [name for name in INPUTS if name not in skip]
