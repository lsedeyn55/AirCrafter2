"""Node outputs -> means, standard deviations, the robust objective and the robust constraint values.

For each tracked quantity q with node values f_j (j = 0 .. N-1) and a propagation rule (quadrature.Rule):

    mu    = a . f
    var   = f^T Q f                                   (Q = diag(D) - U diag(c) U^T)
    var+  = var                                       if Q is positive semi-definite (udr, sobol)
          = (var + sqrt(var^2 + eps^4)) / 2           otherwise (smolyak: weights can be negative)
    sigma = sqrt(var+ + eps^2)                        (finite derivative at zero variance)
    lo    = mu - beta sigma        (robust form of q >= lower)
    hi    = mu + beta sigma        (robust form of q <= upper)
    robust (objective quantity only) = mu + sign k sigma,   sign = +1 for a minimised score, -1 for a maximised one

All partials are analytic: d mu/df = a, d var/df = 2 Q f, d sigma/df = (d var+/d var) (d var/df) / (2 sigma).
"""

import numpy as np
import openmdao.api as om


def key_of(name):
    """Variable-name-safe key of a promoted output name (cruise.cl_margin_sec -> cruise__cl_margin_sec)."""
    return name.replace(".", "__")


class RobustStats(om.ExplicitComponent):
    def initialize(self):
        self.options.declare("rule", desc="quadrature.Rule")
        self.options.declare("quantities", types=dict, desc="{key: eps} (eps in the quantity's units)")
        self.options.declare("objective", default=None, allow_none=True, desc="key of the objective quantity")
        self.options.declare("k", default=1.0)
        self.options.declare("sign", default=1.0, desc="+1: minimised score (mu + k sigma); -1: maximised")
        self.options.declare("beta", default=2.0)

    def setup(self):
        N = self.options["rule"].n
        for q in self.options["quantities"]:
            for j in range(N):
                self.add_input(f"{q}_n{j}", 0.0)
            outs = ["mu", "sigma", "lo", "hi"] + (["robust"] if q == self.options["objective"] else [])
            for o in outs:
                self.add_output(f"{q}_{o}", 0.0)
                for j in range(N):
                    self.declare_partials(f"{q}_{o}", f"{q}_n{j}")

    def _moments(self, f, eps):
        r = self.options["rule"]
        mu = r.a @ f
        var = r.var(f)
        dvar = r.var_grad(f)
        if r.psd:
            vp, dvp = var, 1.0
        else:
            root = np.sqrt(var * var + eps**4)
            vp, dvp = 0.5 * (var + root), 0.5 * (1.0 + var / root)
        sigma = np.sqrt(vp + eps * eps)
        return mu, sigma, dvp * dvar / (2.0 * sigma)

    def _f(self, inputs, q):
        return np.array([inputs[f"{q}_n{j}"][0] for j in range(self.options["rule"].n)])

    def compute(self, inputs, outputs):
        beta, k, sign = self.options["beta"], self.options["k"], self.options["sign"]
        for q, eps in self.options["quantities"].items():
            mu, sigma, _ = self._moments(self._f(inputs, q), eps)
            outputs[f"{q}_mu"] = mu
            outputs[f"{q}_sigma"] = sigma
            outputs[f"{q}_lo"] = mu - beta * sigma
            outputs[f"{q}_hi"] = mu + beta * sigma
            if q == self.options["objective"]:
                outputs[f"{q}_robust"] = mu + sign * k * sigma

    def compute_partials(self, inputs, J):
        a = self.options["rule"].a
        beta, k, sign = self.options["beta"], self.options["k"], self.options["sign"]
        for q, eps in self.options["quantities"].items():
            _, _, ds = self._moments(self._f(inputs, q), eps)
            for j in range(len(a)):
                x = f"{q}_n{j}"
                J[f"{q}_mu", x] = a[j]
                J[f"{q}_sigma", x] = ds[j]
                J[f"{q}_lo", x] = a[j] - beta * ds[j]
                J[f"{q}_hi", x] = a[j] + beta * ds[j]
                if q == self.options["objective"]:
                    J[f"{q}_robust", x] = a[j] + sign * k * ds[j]
