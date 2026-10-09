# How to add a scoring plug-in

The objective lives in a *scoring plug-in*, so each year's DBF rules can be added without touching the
disciplines. Two generic placeholders ship with the framework:

- `three_lap_time`: minimise the 3-lap time.
- `laps_in_window`: maximise the continuous lap count within the mission time and the usable battery energy.

## 1. Write the plug-in

Create a file, e.g. `aircrafter2/scoring/dbf_2027.py` (or anywhere on your Python path):

```python
from aircrafter2.scoring import ScoringPlugin, register


@register("dbf_2027_m2")
class DBF2027Mission2(ScoringPlugin):
    sense = "max"                    # "min" or "max"
    ref = 1.0                        # rough magnitude of the score (optimiser scaling)
    inputs = {                       # promoted model names it reads, with units (None = dimensionless)
        "m_pay": "kg",
        "ThreeLapTime": "s",
        "laps_frac": None,
    }

    def score(self, x):
        # x[name] are numpy arrays. Use numpy arithmetic only (no if/max/round on values):
        # the framework differentiates this with the complex step.
        return x["m_pay"] / x["ThreeLapTime"]

    def constraints(self):
        # optional extra constraints: (promoted name, add_constraint keyword arguments)
        return [("laps_frac", dict(lower=3.0))]
```

Model names you can use include the following (see `aircrafter2/api.py` `SUMMARY_KEYS` and the N2 diagram):

| Group | Names |
|---|---|
| Lap model | `ThreeLapTime`, `Lap1time`, `otherLapTime`, `laps_frac`, `energy_3laps`, `Preq`, `Tstatic` |
| Masses | `m_total`, `m_empty`, `m_pay`, `m_wing` |
| Geometry | `b`, `c`, `S` |
| Flight | `v` (cruise speed) |
| Propulsion | `E_usable`, `E_mission`, `T_static` |

## 2. Make it available

Either import the module somewhere (e.g. add `from . import dbf_2027` to `aircrafter2/scoring/__init__.py`), or
point the config at it directly:

```yaml
scoring:
  plugin: "aircrafter2.scoring.dbf_2027:DBF2027Mission2"   # or the registered name "dbf_2027_m2"
  options: {}                                             # passed to the plug-in constructor (self.options)
```

`aircrafter2 plugins` lists the registered names.

## 3. Rules that need integers or branches

Competition scores often use counts ("number of laps", "number of passengers") or `max()` over missions. Keep the
optimiser's objective smooth:

- **Continuous surrogates.** Use `laps_frac`, not `lapCount`, and treat counts as continuous design variables;
  round them afterwards and re-run the analysis.
- **Smooth maxima and minima.** Replace `max`/`min` with `aircrafter2.smooth.ks_max` / `ks_min`.
- **Discrete choices outside the optimiser.** Enumerate choices such as the propeller in an outer loop, as
  `examples/full_mdo.py` does.

## 4. Test it

Copy `tests/test_step8.py::test_custom_plugin_is_wired_as_objective` and change the plug-in name. It checks that
the score is computed, is the objective, and has finite derivatives.
