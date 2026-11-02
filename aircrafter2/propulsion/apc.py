"""APC propeller performance data (PER3_*.dat) and smooth surrogates.

Data: APC's published performance files (``THESIS/PERFILES2/PER3_*.dat``, outside the repo), the same
source AirCrafter 1.0 used (parse_apc_txt_folder.m / propSelector.m). Each file has blocks
"PROP RPM = N" with columns V (mph), J, Pe, Ct, Cp, PWR (Hp), Torque (in-lbf), Thrust (lbf),
PWR (W), Torque (N-m), Thrust (N), THR/PWR (g/W), Mach, Reyn, FOM.

Surrogate: least-squares polynomials
    C_T(J, r) = sum_{i<=dj, j<=dr} a_ij J^i r^j,   r = RPM / 10000
    C_P(J, r) = same form
fitted over the tabulated (J, RPM) points with C_T >= ct_floor (the windmill branch is
dropped). Smooth and complex-step safe, so propulsion partials are exact.
RPM limit (E props): RPM_max = 150000 / D[in]; AirCrafter 1.0 used 0.85 of it (propSelector.m).

Data folder: the ``folder`` argument (config key ``propulsion.apc_dir``), else the ``AIRCRAFTER2_APC_DIR``
environment variable, else ``THESIS/PERFILES2`` next to the repo.
"""

import os
import re
from dataclasses import dataclass
from pathlib import Path

import numpy as np

INCH = 0.0254
APC_DIR_ENV = "AIRCRAFTER2_APC_DIR"
REPO_DEFAULT_DIR = Path(__file__).resolve().parents[3] / "PERFILES2"  # THESIS/PERFILES2
DEFAULT_DIR = Path(os.environ[APC_DIR_ENV]).expanduser() if os.environ.get(APC_DIR_ENV) else REPO_DEFAULT_DIR


@dataclass
class APCData:
    name: str
    diameter: float  # m
    pitch: float  # m
    rpm: np.ndarray
    J: np.ndarray
    Ct: np.ndarray
    Cp: np.ndarray
    mach_tip: np.ndarray


def parse_apc_file(path):
    path = Path(path)
    name = path.stem.replace("PER3_", "")
    m = re.match(r"^(\d+(?:\.\d+)?)x(\d+(?:\.\d+)?)", name)
    if not m:
        raise ValueError(f"cannot read diameter x pitch from {name!r}")
    d_in, p_in = (float(x) for x in m.groups())
    # APC writes e.g. 105x45 for 10.5 x 4.5 and 11x7E; a 3-digit diameter means one decimal
    if d_in >= 100:
        d_in /= 10.0
    if p_in >= 100 or (len(m.group(2)) == 2 and p_in > d_in * 1.6):
        p_in /= 10.0
    rows, rpm = [], None
    for line in path.read_text(errors="replace").splitlines():
        if "PROP RPM" in line:
            rpm = float(line.split("=")[1])
            continue
        parts = line.split()
        if rpm is None or len(parts) < 15:
            continue
        try:
            vals = [float(x) for x in parts[:15]]
        except ValueError:
            continue
        rows.append([rpm] + vals)
    a = np.array(rows)
    return APCData(
        name=name,
        diameter=d_in * INCH,
        pitch=p_in * INCH,
        rpm=a[:, 0],
        J=a[:, 2],
        Ct=a[:, 4],
        Cp=a[:, 5],
        mach_tip=a[:, 13],
    )


def _basis(J, r, dj, dr):
    J, r = np.asarray(J), np.asarray(r)
    return np.stack([J**i * r**j for i in range(dj + 1) for j in range(dr + 1)], axis=-1)


@dataclass
class PropSurrogate:
    name: str
    diameter: float
    pitch: float
    a_ct: np.ndarray
    a_cp: np.ndarray
    dj: int
    dr: int
    J_max: float
    rpm_min: float
    rpm_max_data: float
    rpm_limit: float  # 150000 / D[in]
    rms_ct: float
    rms_cp: float

    def ct(self, J, rpm):
        return _basis(J, rpm / 1e4, self.dj, self.dr) @ self.a_ct

    def cp(self, J, rpm):
        return _basis(J, rpm / 1e4, self.dj, self.dr) @ self.a_cp


def fit_prop(data: APCData, dj=4, dr=2, ct_floor=0.0):
    keep = data.Ct >= ct_floor
    A = _basis(data.J[keep], data.rpm[keep] / 1e4, dj, dr)
    a_ct, *_ = np.linalg.lstsq(A, data.Ct[keep], rcond=None)
    a_cp, *_ = np.linalg.lstsq(A, data.Cp[keep], rcond=None)
    rms = lambda a, y: float(np.sqrt(np.mean((A @ a - y) ** 2)))  # noqa: E731
    return PropSurrogate(
        name=data.name,
        diameter=data.diameter,
        pitch=data.pitch,
        a_ct=a_ct,
        a_cp=a_cp,
        dj=dj,
        dr=dr,
        J_max=float(data.J[keep].max()),
        rpm_min=float(data.rpm.min()),
        rpm_max_data=float(data.rpm.max()),
        rpm_limit=150000.0 / (data.diameter / INCH),
        rms_ct=rms(a_ct, data.Ct[keep]),
        rms_cp=rms(a_cp, data.Cp[keep]),
    )


def load_prop(name, folder=None):
    folder = Path(folder).expanduser() if folder else DEFAULT_DIR
    path = folder / f"PER3_{name}.dat"
    if not path.is_file():
        raise FileNotFoundError(
            f"APC data file not found: {path}. Point propulsion.apc_dir (config) or the "
            f"{APC_DIR_ENV} environment variable at the folder holding the PER3_*.dat files."
        )
    return fit_prop(parse_apc_file(path))
