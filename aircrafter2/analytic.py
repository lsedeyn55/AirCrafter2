"""Base class for scalar components with hand-written analytic partials, used by the geometry and mass components.

A subclass implements io(self) -> (inputs, outputs), each a dict name -> (default, units), and evaluate(self, x) ->
(values, grads). x maps input names to arrays, values maps output names to arrays, and grads maps output names to a D (a
sparse gradient: input name -> partial derivative). The partials to declare are discovered with one dry run of evaluate
at the default inputs, so each equation's derivative is written in one place, next to the equation itself.

evaluate must be complex-step safe (tests check every component with check_partials(method="cs")).
"""

import numpy as np
import openmdao.api as om


class D(dict):
    """Sparse gradient {input_name: dvalue/dinput} with linear algebra."""

    # make `ndarray * D` dispatch to D.__rmul__ instead of numpy broadcasting
    __array_ufunc__ = None

    def __add__(self, other):
        """Sum of two gradients; 0 is the identity, so sum() works."""
        if isinstance(other, (int, float)) and other == 0:
            return D(self)
        out = D(self)
        for k, v in other.items():
            out[k] = out[k] + v if k in out else v
        return out

    __radd__ = __add__

    def __mul__(self, c):
        """Multiply every partial by c."""
        return D({k: v * c for k, v in self.items()})

    __rmul__ = __mul__

    def __neg__(self):
        """Flip every partial."""
        return self * -1.0

    def __sub__(self, other):
        """Difference of two gradients."""
        return self + (-other)


def combine(weights, grads):
    """sum_i weights[i] * grads[i] (e.g. for a smooth max).

    Parameters
    ----------
    weights : list of float
    grads : list of D

    Returns
    -------
    D
    """
    return sum((w * g for w, g in zip(weights, grads)), D())


class AnalyticComponent(om.ExplicitComponent):
    """ExplicitComponent whose partials come from ``evaluate``'s gradient dicts."""

    def io(self):  # pragma: no cover - abstract
        raise NotImplementedError

    def evaluate(self, x):  # pragma: no cover - abstract
        raise NotImplementedError

    def setup(self):
        """Declare inputs and outputs from io(), and partials from a dry run of evaluate."""
        ins, outs = self.io()
        self._in_names = list(ins)
        for name, (val, units) in ins.items():
            self.add_input(name, val=val, units=units)
        for name, (val, units) in outs.items():
            self.add_output(name, val=val, units=units)

        x0 = {n: np.atleast_1d(np.asarray(v, dtype=float)) for n, (v, _) in ins.items()}
        _, grads = self.evaluate(x0)
        for out, g in grads.items():
            for wrt in g:
                self.declare_partials(out, wrt)

    def _x(self, inputs):
        """The inputs as a plain dict, for evaluate."""
        return {n: inputs[n] for n in self._in_names}

    def compute(self, inputs, outputs):
        values, _ = self.evaluate(self._x(inputs))
        for k, v in values.items():
            outputs[k] = v

    def compute_partials(self, inputs, partials):
        _, grads = self.evaluate(self._x(inputs))
        for out, g in grads.items():
            for wrt, v in g.items():
                partials[out, wrt] = v
