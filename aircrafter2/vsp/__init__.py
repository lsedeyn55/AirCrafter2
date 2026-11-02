"""OpenVSP 3D geometry mock-up of an AirCrafter 2.0 result, a visual and consistency check only.

The builder reads only geometry.json (geometry/export.py); it never touches OpenMDAO and never feeds anything back into
the framework. OpenVSP is optional: see the README section "OpenVSP (optional, installed locally)". Everything except
`build` (which needs OpenVSP) works without it. Details: docs/modules/vsp.md.
"""

INSTALL_HINT = (
    "OpenVSP is not available in this environment. It is optional; see the README section "
    "'OpenVSP (optional, installed locally)' to install it into the project venv."
)


def have_openvsp():
    """True if OpenVSP can be imported."""
    try:
        import openvsp  # noqa: F401
    except ImportError:
        return False
    return True


def require_openvsp():
    """Raise RuntimeError with the install hint if OpenVSP is missing."""
    if not have_openvsp():
        raise RuntimeError(INSTALL_HINT)
