"""OpenVSP 3D geometry mock-up of an AirCrafter 2.0 result (visual and consistency check only).

The builder reads only geometry.json (aircrafter2.geometry.export); it never touches OpenMDAO and never feeds
anything back into the framework. OpenVSP is optional: see the README section "OpenVSP (optional, installed
locally)". Everything except `build` (which needs OpenVSP) works without it.
"""

INSTALL_HINT = ("OpenVSP is not available in this environment. It is optional; see the README section "
                "'OpenVSP (optional, installed locally)' to install it into the project venv.")


def have_openvsp():
    try:
        import openvsp  # noqa: F401
    except ImportError:
        return False
    return True


def require_openvsp():
    if not have_openvsp():
        raise RuntimeError(INSTALL_HINT)
