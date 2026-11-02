# Configuration (`aircrafter2/config/`)

Every run is one YAML file, validated by pydantic schemas that forbid unknown keys ([plan-d7](../DECISIONS.md#plan-d7)). The top-level
schemas live with their problems (`problems/`). This package holds the airfoil, aero and mass sections, and the loading.

## Loading (`loader.py`)

A file may contain `extends: <relative path>`. The parent is loaded first, and the child is deep-merged over it: dicts merge, everything
else replaces. Every problem config sits over two layers, lowest first (README, "Placeholders and rulesets";
[pr-layers](../DECISIONS.md#pr-layers)):

1. code defaults
2. `configs/placeholders.yaml`
3. the ruleset, `configs/rulesets/<ruleset>.yaml`
4. the problem config and its `extends:` chain
5. the study variant

## Layer files (`layer_files.py`)

The files are written one value per line, as a dotted path with a trailing comment:

    propulsion.Kv: 1050.0                        # rpm/V; cited; Q7; T-Motor V3115 1050 KV [W p. 22]

under `# --- area ---` headers, plus an `uncertainty:` list of flow mappings (the robust-design spreads).

Comment fields: `<units>; <PLACEHOLDER | cited>; <Q..>; <source or reason>`. The source may contain `;`.

`layer_files.py` parses these into rows for the UI, and writes edited rows back by replacing only the lines whose value, status or source
changed. Every other line, comment and the order stay byte-identical: there's no YAML round trip and no new dependency.
