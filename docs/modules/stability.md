# Stability (`aircrafter2/stability/static.py`)

- **Neutral point:** x_np = x_ac,w + η_h V_h (a_h/a_w)(1 − dε/dα) c, from Nelson, *Flight Stability and Automatic
  Control*, Ch. 2, and Raymer Ch. 16. a_h is Helmbold's low-AR slope, dε/dα = 2a_w/(πA), and a_w is the coupled
  VLM C_L,α.
- **CG:** mass-weighted component stations. The battery and cargo-bay stations are design variables, so the
  optimiser can trim the CG.
- **Static margin:** SM = (x_np − x_cg)/c.
- **Constraints:** SM, V_h and V_v bands. All are placeholders (Q9).
- **AVL later:** an AVL-based component only needs to output x_np (and could add lateral derivatives); the
  interface is in the module docstring.
