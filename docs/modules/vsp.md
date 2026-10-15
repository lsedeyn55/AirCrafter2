# OpenVSP geometry mock-up (`aircrafter2/vsp/`)

A 3D model of any AirCrafter 2.0 result, for the DBF team and for figures:
- an OpenVSP `.vsp3` file;
- a STEP file for CAD;
- an STL mesh;
- thesis figures.

It is a **visual and consistency mock-up only**. It never enters the OpenMDAO model or the optimiser, and nothing
from it feeds back into the framework.

## Environment

OpenVSP 3.55.0 is optional. It is installed locally into `third_party/openvsp/`, with its Python API in the project
venv; see the README section "OpenVSP (optional, installed locally)".

Without OpenVSP:
- the package, the tests, the CLI and the UI all still work;
- the OpenVSP tests are skipped;
- `aircrafter2 vsp` and `run --vsp` print the README pointer and exit with code 2;
- the UI hides the button.

## Pipeline

```
run_design  ->  outputs/<run>/geometry.json   (aircrafter2.geometry.export; plain JSON, no OpenVSP)
aircrafter2 vsp outputs/<run>/  ->  outputs/<run>/vsp/
```

The second step writes:
- `<run>.vsp3`;
- `<run>.stp` (markers excluded);
- `<run>.stl` (markers excluded);
- `<run>_wing.dat`;
- `components/*.stl`;
- `vsp_check.md`;
- `<run>_three_view.pdf/.png` and `<run>_isometric.pdf/.png`.

The builder (`vsp/build.py`) reads **only** `geometry.json`, so it works on any saved run, including a future
`configs/woombah_2025.yaml` run. Other entry points:
- `aircrafter2 run <cfg> --vsp` builds right after the run;
- the Streamlit UI has a "Build OpenVSP model" button;
- `examples/vsp_mockup.py` rebuilds the twin-study designs and the paper figure.

## What comes from the result and what is illustrative

| From the result (`geometry.json`) | Illustrative (`mockup_assumptions`, with sources) |
|---|---|
| Wing span and chord (rectangular); airfoil: the exact Kulfan coordinates as a file airfoil | Vertical placement: high wing on the cargo bay (Woombah Sec. 3.5) |
| HT and VT: area, aspect ratio, c/4 station, t/c (NACA 00xx) | Boom on the cargo-bay centreline; HT on the boom; VT on the HT |
| Tail boom: start, length, radius | Cargo-bay corner radius; nose and tail fairings (`mockup.fairings`, off by default) |
| Cargo bay: l × w × h and its station | Twin nacelle thrust axis 0.8" below the wing chord line (Woombah Sec. 5.3.2) |
| Prop diameter; motor station x_motor; twin y_m | Blade shape and count, spinner, nose motor mount (n = 1) |
| Nacelle length and diameter (twin) | Nacelle shape (OpenVSP POD) |
| Spar station and radius; CG and NP (markers, never exported) | Wing incidence: none by default; `mockup.wing_incidence: cruise` tilts the model to α_cruise |

## Coordinates and units

The framework's own frame:
- origin at the wing root leading edge;
- x aft, y to starboard, z up;
- metres.

OpenVSP 3.55.0 models carry no length unit. The model is built in metres, and the STEP and IGES export unit is set
to metres (`STEPSettings:LenUnit = LEN_M`, saved in the `.vsp3`).

## Opening the model

```bash
third_party/openvsp/vsp outputs/<run>/vsp/<run>.vsp3
```

The `export` set holds everything except the CG, NP and spar-tube markers.

## Consistency check (`vsp_check.md`)

The check reloads the saved `.vsp3` and compares it with `geometry.json`:
- span, area and chord;
- airfoil t/c and its location;
- HT and VT areas, and the tail arm;
- the cargo-bay box;
- boom start and length;
- prop diameters and stations;
- the twin's measured prop clearances against the framework margins;
- nacelle size;
- CompGeom wetted areas against the framework's own formulas (`aero/parasite.py`).

Every parameter matches to < 0.01%. The exception is the x location of maximum t/c (≈ 1%), which comes from the
sampling grid. The wetted-area differences are explained in each report. Those over 5% are listed in
OPEN_QUESTIONS Q31.

The report also gives the shape difference between OpenVSP's own CST airfoil type and the exported Kulfan
coordinates (≈ 0.004 c). OpenVSP's CST has no Kulfan leading-edge term, so the model uses a file airfoil.

## OpenVSP 3.55.0 behaviour the builder works around

- **No headless rendering:** `ScreenGrab` needs the FLTK GUI. The figures are drawn with matplotlib from the
  component STLs; meshes are rasterised at 300 dpi, text and axes stay vector.
- **Default FUSELAGE end sections carry ±45° skinning tangents.** They bulge a "constant" section a lot. The
  builder cuts each body to two sections and zeroes the tangent angles.
- **POD `FineRatio` is length/radius:** the builder sets 2L/d.
- **Bounding-box read-back:** after each build the builder compares bounding boxes with `geometry.json` and raises
  if any differs by more than 1%.

## Limitations

- **Rectangular wing only:** no taper, sweep, dihedral or twist; the framework models none.
- **No fuselage shape model:** a cargo-bay box plus a boom.
- **Illustrative propellers, spinners and nacelle shapes.**
- **No control surfaces, landing gear or payload.**
- **No analysis:** VSPAERO and AVL are not run (see PLAN §6i for how they could cross-check the framework later).
