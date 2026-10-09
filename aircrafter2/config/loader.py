"""YAML loading with a simple ``extends:`` mechanism.

A file may contain ``extends: <relative path>``. The parent is loaded first,
and the child is deep-merged over it (dicts merge, everything else replaces).
"""

from pathlib import Path

import yaml


def _merge(base, over):
    out = dict(base)
    for k, v in over.items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _merge(out[k], v)
        else:
            out[k] = v
    return out


def load_yaml(path):
    path = Path(path)
    with open(path) as fh:
        data = yaml.safe_load(fh) or {}
    parent = data.pop("extends", None)
    if parent is not None:
        data = _merge(load_yaml(path.parent / parent), data)
    return data
