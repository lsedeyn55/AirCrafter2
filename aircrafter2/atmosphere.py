"""International Standard Atmosphere (troposphere) and air viscosity.

ISA: ICAO Doc 7488/3 (1993), troposphere (h < 11 km): T = T0 - L h, p = p0 (T/T0)^(g0/(L R)).
Matches the ``isa_density`` used in dbf_performance_modeller.py (and MATLAB's ``atmosisa``).

Dynamic viscosity: Sutherland's law, mu = mu_ref (T/T_ref)^1.5 (T_ref + S)/(T + S), with
mu_ref = 1.716e-5 Pa s, T_ref = 273.15 K, S = 110.4 K (White, Viscous Fluid Flow, 3rd ed., Table 1-2).
"""

import numpy as np

T0 = 288.15  # K
P0 = 101325.0  # Pa
LAPSE = 0.0065  # K/m
R_AIR = 287.05287  # J/(kg K)
G0 = 9.80665  # m/s^2
GAMMA = 1.4


def isa(altitude):
    """Return dict(T, p, rho, mu, a) at geometric altitude (m), troposphere only."""
    h = np.asarray(altitude, dtype=float)
    T = T0 - LAPSE * h
    p = P0 * (T / T0) ** (G0 / (LAPSE * R_AIR))
    rho = p / (R_AIR * T)
    mu = 1.716e-5 * (T / 273.15) ** 1.5 * (273.15 + 110.4) / (T + 110.4)
    a = np.sqrt(GAMMA * R_AIR * T)
    return dict(T=T, p=p, rho=rho, mu=mu, a=a)


def isa_density(altitude):
    return isa(altitude)["rho"]
