import numpy as np
import pytest

from aircrafter2.smooth import ks_max, ks_min, smooth_abs, softplus


def test_ks_max_bounds():
    a, b = np.array([1e-4]), np.array([2e-4])
    f, (da, db) = ks_max([a, b], rho=100.0)
    assert b[0] * (1 - 1e-12) <= f[0] <= b[0] * 2 ** (1 / 100)
    assert da[0] + db[0] == pytest.approx(f[0] / b[0], rel=1e-2)


def test_ks_max_is_scale_invariant():
    xs = [np.array([3.0]), np.array([2.9])]
    f1, _ = ks_max(xs, 50.0)
    f2, _ = ks_max([x * 1e-6 for x in xs], 50.0)
    assert f2[0] == pytest.approx(f1[0] * 1e-6, rel=1e-12)


def test_ks_max_tie_overestimate():
    f, _ = ks_max([np.array([1.0]), np.array([1.0])], rho=100.0)
    assert f[0] == pytest.approx(2 ** (1 / 100), rel=1e-12)


def test_hard_max_and_min():
    f, (da, db) = ks_max([np.array([1.0, 5.0]), np.array([2.0, 3.0])], mode="hard")
    assert list(f) == [2.0, 5.0] and list(da) == [0.0, 1.0] and list(db) == [1.0, 0.0]
    g, _ = ks_min([np.array([1.0]), np.array([2.0])], rho=200.0)
    assert 2 ** (-1 / 200) <= g[0] <= 1.0


@pytest.mark.parametrize("fn", ["ks_max", "ks_min", "softplus", "smooth_abs"])
def test_complex_step_derivatives(fn):
    x = np.array([0.7])
    h = 1e-30
    if fn == "ks_max":
        f = lambda v: ks_max([v, np.array([0.65])], 30.0)  # noqa: E731
    elif fn == "ks_min":
        f = lambda v: ks_min([v, np.array([0.75])], 30.0)  # noqa: E731
    elif fn == "softplus":
        f = lambda v: (lambda r: (r[0], [r[1]]))(softplus(v - 0.69, 0.01))  # noqa: E731
    else:
        f = lambda v: (lambda r: (r[0], [r[1]]))(smooth_abs(v - 0.69, 0.01))  # noqa: E731
    val, grads = f(x)
    cs = np.imag(f(x + 1j * h)[0]) / h
    assert grads[0][0] == pytest.approx(cs[0], rel=1e-10)


def test_softplus_limits():
    f, d = softplus(np.array([-1.0, 0.0, 1.0]), 1e-3)
    assert f[0] == pytest.approx(0.0, abs=1e-12) and f[2] == pytest.approx(1.0)
    assert f[1] == pytest.approx(1e-3 * np.log(2)) and d[1] == pytest.approx(0.5)
