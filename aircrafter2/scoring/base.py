"""Scoring plug-in interface.

A scoring plug-in turns model outputs into the optimisation objective (and, optionally, extra
constraints). Year-specific DBF rules live in plug-ins; the framework stays generic.

To write one (see docs/how_to_add_a_scoring_plugin.md):

    from aircrafter2.scoring import ScoringPlugin, register

    @register("my_score_2027")
    class MyScore(ScoringPlugin):
        sense = "max"                                     # "min" or "max"
        inputs = {"laps_frac": None, "m_pay": "kg"}      # model outputs/inputs it reads (promoted name: units)

        def score(self, x):                              # x: dict of numpy arrays, complex-step safe
            return x["laps_frac"] * x["m_pay"]

        def constraints(self):                           # optional: [(name, dict(lower=..., upper=...))]
            return []

The framework wires the inputs by promoted name, adds an ExecComp-like component computing
`score` (complex-step partials), and sets it as the objective with the right sign. Plug-ins can
also be loaded from a dotted path in the config: scoring: {plugin: "mypkg.module:MyScore"}.
"""

from abc import ABC, abstractmethod
from importlib import import_module

import numpy as np
import openmdao.api as om

_REGISTRY = {}


def register(name):
    def deco(cls):
        cls.name = name
        _REGISTRY[name] = cls
        return cls

    return deco


def available():
    return sorted(_REGISTRY)


def get_plugin(spec, **options):
    """spec: a registered name or 'module.path:ClassName'."""
    if ":" in spec:
        mod, cls = spec.split(":")
        return getattr(import_module(mod), cls)(**options)
    if spec not in _REGISTRY:
        raise KeyError(f"unknown scoring plug-in {spec!r}; available: {available()}")
    return _REGISTRY[spec](**options)


class ScoringPlugin(ABC):
    name = "unnamed"
    sense = "min"
    inputs: dict = {}
    ref = 1.0  # objective scaling hint

    def __init__(self, **options):
        self.options = options

    @abstractmethod
    def score(self, x):
        """Score from a dict of input arrays. Must be complex-step safe (numpy arithmetic only)."""

    def constraints(self):
        return []

    def component(self):
        return ScoreComponent(plugin=self)


class ScoreComponent(om.ExplicitComponent):
    def initialize(self):
        self.options.declare("plugin")

    def setup(self):
        for name, units in self.options["plugin"].inputs.items():
            self.add_input(name, 1.0, units=units)
        self.add_output("score", 0.0)
        self.declare_partials("score", "*", method="cs")

    def compute(self, inputs, outputs):
        outputs["score"] = np.asarray(self.options["plugin"].score({k: inputs[k] for k in inputs}))


def add_scoring(model, plugin, objective=True):
    """Add the plug-in's score component (promoted) and set the objective and constraints."""
    model.add_subsystem("scoring", plugin.component(), promotes_inputs=list(plugin.inputs), promotes_outputs=["score"])
    if objective:
        model.add_objective("score", ref=plugin.ref if plugin.sense == "min" else -plugin.ref)
    for name, kw in plugin.constraints():
        model.add_constraint(name, **kw)
