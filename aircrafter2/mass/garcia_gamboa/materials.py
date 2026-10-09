"""Allowable stresses from ultimate stresses and the safety factor.

Garcia & Gamboa (2022), Eq. 42: SF = ultimate stress / allowable stress.
The material tables (Tables 5, 9, 13) list ultimate values.
Moduli (E, G) are not divided by SF.
"""

from ...analytic import AnalyticComponent, D

ULTIMATE = ["sigma_spar", "tau_spar", "sigma_balsa", "tau_balsa", "sigma_tb", "tau_tb"]


class Allowables(AnalyticComponent):
    def io(self):
        ins = {"SF": (1.5, None)}
        ins.update({name: (100e6, "Pa") for name in ULTIMATE})
        outs = {f"{name}_allow": (1.0, "Pa") for name in ULTIMATE}
        return ins, outs

    def evaluate(self, x):
        sf = x["SF"]
        values, grads = {}, {}
        for name in ULTIMATE:
            v = x[name] / sf  # Eq. 42
            values[f"{name}_allow"] = v
            grads[f"{name}_allow"] = D({name: 1.0 / sf, "SF": -v / sf})
        return values, grads
