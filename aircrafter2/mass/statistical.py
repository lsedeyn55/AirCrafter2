"""Statistical (class I) total-mass models, Garcia & Gamboa (2022) Section 4.1.

    Eq. 124 (all aircraft):        m = 3.2005 b^0.1986 c^0.1234 m_pay^0.5554
    Eq. 125 (balsa aircraft):      m = 5.7001 b^0.2442 c^0.2649 m_pay^0.3365
    Eq. 126 (composite aircraft):  m = 3.1408 b^0.2423 c^0.1338 m_pay^0.5530

b in m, c the wing (mean) chord in m, m_pay in kg. These were fitted to ACC
aircraft with m = 8.9-16 kg and m_pay = 5-12 kg (Tables 1-2). DBF aircraft
are outside that range, so the model is used only as the initial guess of the
mass closure and as a sanity-check column (PLAN G21).
"""

import numpy as np

from ..analytic import AnalyticComponent, D

COEFFS = {
    "all": (3.2005, 0.1986, 0.1234, 0.5554),  # Eq. 124
    "balsa": (5.7001, 0.2442, 0.2649, 0.3365),  # Eq. 125
    "composite": (3.1408, 0.2423, 0.1338, 0.5530),  # Eq. 126
}


def statistical_mass(b, c, m_pay, model="all"):
    k, eb, ec, ep = COEFFS[model]
    return k * b**eb * c**ec * m_pay**ep


class StatisticalMass(AnalyticComponent):
    def initialize(self):
        self.options.declare("model", default="all", values=list(COEFFS))

    def io(self):
        ins = {"b": (3.0, "m"), "c_mean": (0.3, "m"), "m_pay": (5.0, "kg")}
        return ins, {"m_stat": (8.0, "kg")}

    def evaluate(self, x):
        _, eb, ec, ep = COEFFS[self.options["model"]]
        b, c, mp = x["b"], x["c_mean"], x["m_pay"]
        # report-only column, but its row is in the coupled Newton matrix: keep it finite if an intermediate Newton
        # iterate makes a computed payload (stores) non-positive (values > 1e-6 kg are untouched)
        mp = np.where(np.real(mp) > 1e-6, mp, 1e-6 + 0.0 * mp)
        m = statistical_mass(b, c, mp, self.options["model"])
        return {"m_stat": m}, {"m_stat": D({"b": eb * m / b, "c_mean": ec * m / c, "m_pay": ep * m / mp})}
