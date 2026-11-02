"""Scoring plug-in interface: turns model outputs into the optimisation objective and, optionally, extra constraints.

Year-specific DBF rules live in plug-ins, so the framework stays generic. A plug-in reads the lap and mass outputs by
promoted name; add_scoring adds its component and sets the objective with the right sign. Plug-ins can also be loaded
from a dotted path in the config: scoring: {plugin: "mypkg.module:MyScore"}. To write one:
docs/how_to_add_a_scoring_plugin.md.
"""

from abc import ABC, abstractmethod
from importlib import import_module

import numpy as np
import openmdao.api as om

_REGISTRY = {}


def register(name):
    """Class decorator: register a plug-in under a name.

    Parameters
    ----------
    name : str

    Returns
    -------
    callable
    """

    def deco(cls):
        """Name the class and add it to the registry."""
        cls.name = name
        _REGISTRY[name] = cls
        return cls

    return deco


def available():
    """The sorted names of the registered plug-ins."""
    return sorted(_REGISTRY)


def get_plugin(spec, **options):
    """An instance of a plug-in.

    Parameters
    ----------
    spec : str
        A registered name or 'module.path:ClassName'.
    **options
        The plug-in's options.

    Returns
    -------
    ScoringPlugin
        Raises KeyError for an unknown name.
    """
    if ":" in spec:
        mod, cls = spec.split(":")
        return getattr(import_module(mod), cls)(**options)
    if spec not in _REGISTRY:
        raise KeyError(f"unknown scoring plug-in {spec!r}; available: {available()}")
    return _REGISTRY[spec](**options)


class ScoringPlugin(ABC):
    """Base class of the plug-ins.

    Attributes
    ----------
    name : str
    sense : "min" or "max"
    inputs : dict
        {promoted name: units} of the model outputs and inputs it reads.
    ref : float
        Objective scaling hint.
    """

    name = "unnamed"
    sense = "min"
    inputs: dict = {}
    ref = 1.0  # objective scaling hint

    def __init__(self, **options):
        """Keep the options."""
        self.options = options

    @abstractmethod
    def score(self, x):
        """Score from a dict of input arrays. Must be complex-step safe (numpy arithmetic only)."""

    def constraints(self):
        """Extra constraints: [(name, add_constraint kwargs)]. None by default."""
        return []

    def component(self):
        """The OpenMDAO component that computes the score."""
        return ScoreComponent(plugin=self)


class ScoreComponent(om.ExplicitComponent):
    """score from the plug-in. Partials are complex step, because plug-ins are arbitrary code.

    Options
    -------
    plugin : ScoringPlugin

    Inputs
    ------
    The plug-in's inputs.

    Outputs
    -------
    score
    """

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
    """Add the plug-in's score component (promoted), and set the objective and constraints.

    Parameters
    ----------
    model : om.Group
    plugin : ScoringPlugin
    objective : bool
        Set the objective. False in analysis runs.
    """
    model.add_subsystem("scoring", plugin.component(), promotes_inputs=list(plugin.inputs), promotes_outputs=["score"])
    if objective:
        model.add_objective("score", ref=plugin.ref if plugin.sense == "min" else -plugin.ref)
    for name, kw in plugin.constraints():
        model.add_constraint(name, **kw)
