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


# --- placeholder and ruleset layers (docs/PLACEHOLDERS_RULESETS_PLAN.md, README "Placeholders and rulesets") ---
CONFIGS_DIR = Path(__file__).resolve().parents[2] / "configs"
LAYER_KEYS = ("placeholders", "ruleset")


def layer_path(name, path):
    """A layer file named in a config: relative to the config's folder, else to configs/."""
    p = Path(name)
    if p.is_absolute():
        return p
    for base in (Path(path).parent, CONFIGS_DIR):
        if (base / p).exists():
            return base / p
    raise FileNotFoundError(f"layer file {name!r} named in {Path(path).name} not found (configs/: {CONFIGS_DIR})")


def expand_dotted(data):
    """Top-level dotted keys to nested dicts: {"propulsion.Kv": 1050} -> {"propulsion": {"Kv": 1050}}. The layer
    files are written this way, one value per line. Under ``bottles.`` the middle segments are one key (the bottle
    name, e.g. 0.5L): bottles.0.5L.diameter -> bottles: {0.5L: {diameter}}."""
    out = {}
    for key, val in data.items():
        parts = key.split(".") if isinstance(key, str) else [key]
        if parts[0] == "bottles" and len(parts) > 3:
            parts = [parts[0], ".".join(parts[1:-1]), parts[-1]]
        node = out
        for k in parts[:-1]:
            node = node.setdefault(k, {})
        node[parts[-1]] = (
            _merge(node[parts[-1]], val) if isinstance(val, dict) and isinstance(node.get(parts[-1]), dict) else val
        )
    return out


def load_layer(path):
    """A placeholders or ruleset file (dotted keys expanded)."""
    return expand_dotted(load_yaml(path))


def _keep(layer, fields):
    """The top-level keys of a layer that the schema has (fields None: all of them)."""
    return {k: v for k, v in layer.items() if fields is None or k in fields}


def load_layers(path, placeholders=None, ruleset=None):
    """(placeholders data, ruleset data, config data) of a problem config: the config with extends: resolved and its
    layer keys removed. placeholders/ruleset override the files the config names (the UI's unsaved edits)."""
    path = Path(path)
    data = load_yaml(path)
    ph_name = data.pop("placeholders", "placeholders.yaml")
    rs_name = data.pop("ruleset", None)
    ph = placeholders if placeholders is not None else (load_layer(layer_path(ph_name, path)) if ph_name else {})
    rs = ruleset if ruleset is not None else (load_layer(layer_path(rs_name, path)) if rs_name else {})
    ph, rs = expand_dotted(ph), expand_dotted(rs)
    return ph, rs, data


def load_layered(path, fields=None, multipoint=False, placeholders=None, ruleset=None):
    """A problem config merged over its layers, lowest first: placeholders.yaml, the ruleset, the config (and its
    extends: chain). Only the layers' keys that the schema has are kept (fields: its field names; the config's own
    keys are not filtered, so a typo still fails). A single-point load ignores the ruleset's multipoint: block.

    The layers' ``uncertainty:`` lists are returned as ``uncertainty_pool`` (placeholders first, merged by name) for
    full_config_from_dict; ``mass`` stays a plain dict, so mass_config is merged underneath it by the caller."""
    ph, rs, data = load_layers(path, placeholders, ruleset)
    pool = {}
    for layer in (ph, rs):
        for p in layer.get("uncertainty") or []:
            pool[p["name"]] = dict(p)
    keep = set(fields) | ({"multipoint"} if multipoint else set()) if fields is not None else None
    out = {}
    for layer in (ph, rs):
        layer = {k: v for k, v in layer.items() if k not in ("uncertainty", "bottles")}
        if not multipoint:
            layer.pop("multipoint", None)
        out = _merge(out, _keep(layer, keep))
    out = _merge(out, data)
    if pool and (fields is None or "robust" in fields):  # only schemas with a robust block use spreads
        out["uncertainty_pool"] = list(pool.values())
    return out
