"""Propulsion: APC propeller data parsing and fits, powertrain operating point (decision B)."""

import numpy as np
import pytest

from aircrafter2.propulsion.apc import DEFAULT_DIR, fit_prop, parse_apc_file
from aircrafter2.propulsion.powertrain import RPM2RADS, Powertrain

from .conftest import check_partials_fd, run_component


# --- APC data ---------------------------------------------------------------------------------
@pytest.mark.parametrize("name,d_in,p_in", [("11x7E", 11.0, 7.0), ("105x45", 10.5, 4.5), ("9x45E", 9.0, 4.5)])
def test_apc_parse_and_fit(name, d_in, p_in):
    data = parse_apc_file(DEFAULT_DIR / f"PER3_{name}.dat")
    assert data.diameter == pytest.approx(d_in * 0.0254) and data.pitch == pytest.approx(p_in * 0.0254)
    s = fit_prop(data)
    k = data.Ct >= 0
    assert s.rms_ct < 0.002 and s.rms_cp < 0.003  # vs Ct ~ 0.1, Cp ~ 0.05
    i = np.argmin(np.abs(data.rpm - 8000) + np.abs(data.J - 0.4) * 1e4)
    assert s.ct(data.J[i], data.rpm[i]) == pytest.approx(data.Ct[i], abs=0.003)


# --- powertrain ---------------------------------------------------------------------------------
def test_powertrain_operating_point_balances_torque():
    pt = Powertrain(prop="11x7E")
    p = run_component(pt)
    x = {k: p.get_val(f"c.{k}") for k in ["throttle", "rho", "n_cells", "V_cell", "R_cell", "Kv", "Rm", "I0"]}
    r = pt.solve_nodes(x)
    kv = x["Kv"] * RPM2RADS
    Qm = (r["I_motor"] - x["I0"]) / kv
    n = r["omega"] / (2 * np.pi)
    Qp = pt._prop.cp(r["J"], r["omega"] / RPM2RADS) * x["rho"] * n**2 * pt._prop.diameter**5 / (2 * np.pi)
    assert np.max(np.abs(Qm - Qp)) < 1e-10
    assert p.get_val("c.fit_residual_T")[0] < 0.1  # N, cubic fit of T(v) through the nodes
    assert np.all(np.diff(p.get_val("c.T_nodes")) < 0)  # thrust falls with speed


def test_powertrain_partials():
    check_partials_fd(run_component(Powertrain(prop="11x7E"), capacity=2.5, throttle=0.9), atol=1e-4,
                      rtol=1e-3, skip=("rpm_limit", "fit_residual_T"))
