"""Read, edit and save the layer files in place: configs/placeholders.yaml and configs/rulesets/<ruleset>.yaml.

Parses them into rows for the UI's editors (ui/app.py, constraints_view.py), and writes edited rows back by replacing
only the lines whose value, status or source changed: every other line, comment and the order stay byte-identical (no
YAML round trip, no new dependency). File format: docs/modules/config.md, "Layer files".
"""

import re
from pathlib import Path

import yaml

from .loader import CONFIGS_DIR, expand_dotted, load_yaml

AREA_RE = re.compile(r"^#\s*---\s*(.*?)\s*---")
LINE_RE = re.compile(r"^(?P<key>[A-Za-z_][\w.]*):(?P<gap>\s*)(?P<value>[^#]*?)(?P<pad>\s*)(?:#\s?(?P<comment>.*))?$")
ITEM_RE = re.compile(r"^(?P<indent>\s*-\s*)(?P<value>\{.*\})(?P<pad>\s*)(?:#\s?(?P<comment>.*))?$")
STATUSES = ("PLACEHOLDER", "cited")


def split_comment(comment):
    """(units, status, Q, source) of a trailing comment; a comment not in that form is all source.

    Parameters
    ----------
    comment : str or None

    Returns
    -------
    (str, str, str, str)
    """
    parts = [p.strip() for p in (comment or "").split(";")]
    if len(parts) >= 3 and parts[1] in STATUSES:
        return parts[0], parts[1], parts[2], "; ".join(parts[3:])
    return "", "", "", (comment or "").strip()


def join_comment(units, status, q, source):
    """The trailing comment of a line, the inverse of split_comment.

    Parameters
    ----------
    units, status, q, source : str

    Returns
    -------
    str
    """
    return "; ".join([units, status, q] + ([source] if source else []))


def fmt_value(x):
    """A scalar as it's written in the file: floats by repr (bit-exact round trip), None as null.

    Parameters
    ----------
    x : object

    Returns
    -------
    str
    """
    if x is None:
        return "null"
    if isinstance(x, bool):
        return "true" if x else "false"
    if isinstance(x, float):
        return repr(x)
    if isinstance(x, int):
        return str(x)
    s = str(x)
    return s if yaml.safe_load(s) == s and s.strip() == s and s else repr(s)


def parse_value(text):
    """The file's text of a value as YAML gives it, so '7.2e+8' is a float.

    Parameters
    ----------
    text : str

    Returns
    -------
    object
    """
    v = yaml.safe_load(text) if text.strip() else None
    if isinstance(v, str):  # YAML 1.1 reads 5.0e-4 / 7.2e+8 as floats, but '1e-3' (no dot) as a string
        try:
            return float(v)
        except ValueError:
            return v
    return v


def read_rows(path):
    """The rows of a layer file.

    Parameters
    ----------
    path : str or Path

    Returns
    -------
    (list of dict, list of dict)
        Value rows: area, parameter (dotted path), value, units, status, Q, source, line (1-based). Spread rows: the
        UncertainParam fields, units, status, Q, source, line.
    """
    rows, spreads, area, in_unc = [], [], "", False
    for i, line in enumerate(Path(path).read_text().splitlines(), start=1):
        m = AREA_RE.match(line)
        if m:
            area, in_unc = m.group(1), False
            continue
        if line.startswith("uncertainty:"):
            in_unc = True
            continue
        if in_unc:
            m = ITEM_RE.match(line)
            if m:
                units, status, q, source = split_comment(m.group("comment"))
                spreads.append(
                    dict(yaml.safe_load(m.group("value")), comment_units=units, status=status, Q=q, note=source, line=i)
                )
            elif line and not line.startswith((" ", "#")):
                in_unc = False
            if in_unc:
                continue
        if line.startswith(("#", " ")) or not line.strip():
            continue
        m = LINE_RE.match(line)
        if m:
            units, status, q, source = split_comment(m.group("comment"))
            rows.append(
                dict(
                    area=area,
                    parameter=m.group("key"),
                    value=parse_value(m.group("value")),
                    units=units,
                    status=status,
                    Q=q,
                    source=source,
                    line=i,
                )
            )
    return rows, spreads


SPREAD_FIELDS = ("name", "dist", "nominal", "spread", "relative", "units", "kv_mode", "source")


def _spread_text(row, original):
    """The flow mapping of a spread row, in the original line's key order (new keys appended)."""
    keys = list(original) + [k for k in SPREAD_FIELDS if k not in original and row.get(k) not in (None, "")]
    parts = []
    for k in keys:
        v = row.get(k, original.get(k))
        if v is None or (isinstance(v, float) and v != v) or v == "":
            continue
        parts.append(f"{k}: {fmt_value(v)}")
    return "{" + ", ".join(parts) + "}"


def save_rows(path, rows, spreads=None):
    """Write edited rows back into the file in place.

    Only lines whose value, status, Q or source (or a spread's fields) changed are rewritten, keeping the key, its
    column and the comment column; everything else is untouched.

    Parameters
    ----------
    path : str or Path
    rows : list of dict
        Value rows, as read_rows gives them.
    spreads : list of dict, optional
        Spread rows.

    Returns
    -------
    list of int
        The rewritten line numbers.
    """
    path = Path(path)
    lines = path.read_text().splitlines(keepends=True)
    cur_rows, cur_spreads = read_rows(path)
    by_line = {r["line"]: r for r in cur_rows}
    changed = []
    for row in rows:
        old = by_line.get(row["line"])
        if old is None or old["parameter"] != row["parameter"]:
            raise ValueError(f"{path.name}: row {row['parameter']!r} is not at line {row['line']} (file changed?)")
        same_value = fmt_value(old["value"]) == fmt_value(row["value"]) or old["value"] == row["value"]
        if same_value and all(old[k] == row[k] for k in ("status", "Q", "source", "units")):
            continue
        text = lines[row["line"] - 1]
        m = LINE_RE.match(text.rstrip("\n"))
        value_text = m.group("value") if same_value else fmt_value(row["value"])
        head = f"{m.group('key')}:{m.group('gap') or ' '}{value_text}"
        col = text.index("#") if "#" in text else len(head) + 2
        comment = join_comment(row["units"], row["status"], row["Q"], row["source"])
        lines[row["line"] - 1] = head.ljust(max(col, len(head) + 2)) + "# " + comment + "\n"
        changed.append(row["line"])
    for row in spreads or []:
        old = next((s for s in cur_spreads if s["line"] == row["line"]), None)
        if old is None or old["name"] != row["name"]:
            raise ValueError(f"{path.name}: spread {row.get('name')!r} is not at line {row['line']} (file changed?)")
        fields_same = all(
            (old.get(k) == row.get(k))
            or (old.get(k) is None and row.get(k) in (None, ""))
            or (isinstance(row.get(k), float) and row.get(k) != row.get(k) and old.get(k) is None)
            for k in SPREAD_FIELDS
        )
        meta_same = all(old[k] == row[k] for k in ("comment_units", "status", "Q", "note"))
        if fields_same and meta_same:
            continue
        text = lines[row["line"] - 1]
        m = ITEM_RE.match(text.rstrip("\n"))
        original = yaml.safe_load(m.group("value"))
        value = m.group("value") if fields_same else _spread_text(row, original)
        head = m.group("indent") + value
        comment = join_comment(row["comment_units"], row["status"], row["Q"], row["note"])
        lines[row["line"] - 1] = head + (m.group("pad") or "  ") + "# " + comment + "\n"
        changed.append(row["line"])
    if changed:
        path.write_text("".join(lines))
    return changed


def save_config_value(path, line, key, value):
    """Set one value in a problem config in place, keeping the rest of the line and the comment column.

    The file is re-parsed before writing: the key must read back as the value, with everything else unchanged.

    Parameters
    ----------
    path : str or Path
    line : int
        1-based line: a block line 'key: value  # comment' or a flow mapping '{a: 1, key: value}'.
    key : str
    value : object
    """
    path = Path(path)
    lines = path.read_text().splitlines(keepends=True)
    text = lines[line - 1].rstrip("\n")
    body, hash_, comment = text.partition("#")
    new = "[" + ", ".join(fmt_value(v) for v in value) + "]" if isinstance(value, (list, tuple)) else fmt_value(value)
    m = re.search(rf"(^|[\s{{,]){re.escape(key)}:(\s*)(\[[^\]]*\]|[^,}}]*?)(\s*)(?=[,}}]|$)", body)
    if m is None:
        raise ValueError(f"{path.name}:{line}: no '{key}:' to edit")
    head = body[: m.start(3)] + new
    tail = body[m.end(3) :]
    stripped = (head + tail).rstrip()
    if hash_:
        out = stripped.ljust(len(body)) if len(stripped) < len(body) else stripped + "  "
        out = out + "#" + comment
    else:
        out = stripped
    lines[line - 1] = out + "\n"
    old_data, new_data = yaml.safe_load("".join(path.read_text())), yaml.safe_load("".join(lines))
    if old_data is not None and _strip_key(old_data, key) != _strip_key(new_data, key):
        raise ValueError(f"{path.name}:{line}: editing {key} would change other values")
    path.write_text("".join(lines))
    return line


def _strip_key(data, key):
    """data with every `key` removed (to compare a file before and after one value changed)."""
    if isinstance(data, dict):
        return {k: _strip_key(v, key) for k, v in data.items() if k != key}
    if isinstance(data, list):
        return [_strip_key(v, key) for v in data]
    return data


def rows_to_layer(rows, spreads=None):
    """Layer data (as load_layer gives it) from rows: the UI's unsaved edits applied to a run.

    Parameters
    ----------
    rows : list of dict
    spreads : list of dict, optional

    Returns
    -------
    dict
    """
    data = expand_dotted({r["parameter"]: r["value"] for r in rows})
    if spreads is not None:
        data["uncertainty"] = [
            {k: s[k] for k in SPREAD_FIELDS if s.get(k) is not None and s.get(k) == s.get(k) and s.get(k) != ""}
            for s in spreads
        ]
    return data


# --- where a config overrides a layer value ---------------------------------------------------------------------


def _chain(path):
    """The config files of path's extends: chain, the config itself first."""
    out, path = [], Path(path)
    while True:
        out.append(path)
        parent = (yaml.safe_load(path.read_text()) or {}).get("extends")
        if not parent:
            return out
        path = path.parent / parent


def _get(data, parts):
    """True if the nested keys parts exist in data."""
    for k in parts:
        if not isinstance(data, dict) or k not in data:
            return False
        data = data[k]
    return True


def _line_of(path, parts):
    """1-based line in path that sets the nested key parts (the leaf key under its top-level section)."""
    lines = Path(path).read_text().splitlines()
    top, leaf = parts[0], parts[-1]
    start = next((i for i, l in enumerate(lines) if re.match(rf"^{re.escape(top)}:", l)), None)
    if start is None:
        return None
    if len(parts) == 1:
        return start + 1
    for i in range(start, len(lines)):
        if i > start and lines[i] and not lines[i].startswith((" ", "#")):
            break
        if re.search(rf"(^|[\s{{,]){re.escape(leaf)}:", lines[i].split("#")[0]):
            return i + 1
    return start + 1


def set_in(config_path, parameter):
    """Where a dotted parameter is set in a config's extends: chain.

    Parameters
    ----------
    config_path : str or Path
    parameter : str

    Returns
    -------
    (Path, int) or None
        The first file of the chain that sets it, and the 1-based line; None if none does.
    """
    parts = parameter.split(".")
    for f in _chain(config_path):
        raw = yaml.safe_load(f.read_text()) or {}
        if _get(raw, parts):
            return f, _line_of(f, parts)
    return None


def overridden_by(config_path, parameter, ruleset_path=None):
    """'file:line' of the file that overrides a layer value for this config.

    Its extends: chain, or the ruleset for a placeholders value. mass.inputs.* is matched against the config's mass:
    block (mass_config sits below the placeholders, so it never overrides).

    Parameters
    ----------
    config_path : str or Path
    parameter : str
    ruleset_path : str or Path, optional

    Returns
    -------
    str
        '' when the layer value reaches the run.
    """
    hit = set_in(config_path, parameter)
    if hit is not None:
        return f"{_rel(hit[0])}:{hit[1]}"
    parts = parameter.split(".")
    if ruleset_path is not None:
        rs = expand_dotted(load_yaml(ruleset_path))
        if _get(rs, parts):
            return f"{_rel(Path(ruleset_path))}"
    return ""


def spread_overridden_by(config_path, name):
    """The file that replaces a shared spread for this config.

    Its uncertainty_config file (standalone, or an entry of that name) or its robust.parameters.

    Parameters
    ----------
    config_path : str or Path
    name : str

    Returns
    -------
    str
        '' if the shared spread is used, or the config isn't robust.
    """
    from .loader import load_yaml as _load

    data = _load(config_path)
    rob = data.get("robust") or {}
    if not rob.get("enabled"):
        return "(not a robust config)"
    if any(p.get("name") == name for p in rob.get("parameters") or []):
        return f"{_rel(Path(config_path))} (robust.parameters)"
    if rob.get("uncertainty_config"):
        f = Path(config_path).parent / rob["uncertainty_config"]
        unc = _load(f)
        if unc.get("standalone"):
            return f"{_rel(f)} (standalone)"
        for p in unc.get("parameters") or []:
            if p.get("name") == name:
                return _rel(f)
    return ""


def _rel(p):
    """A path relative to the repository, or just its name."""
    p = Path(p).resolve()
    try:
        return str(p.relative_to(CONFIGS_DIR.parent))
    except ValueError:
        return p.name


def layer_files_of(config_path):
    """The layer files a config names, with defaults resolved.

    Parameters
    ----------
    config_path : str or Path

    Returns
    -------
    (Path or None, Path or None)
        Placeholders path, ruleset path.
    """
    from .loader import layer_path

    data = load_yaml(config_path)
    ph = data.get("placeholders", "placeholders.yaml")
    rs = data.get("ruleset")
    return (layer_path(ph, config_path) if ph else None), (layer_path(rs, config_path) if rs else None)


# --- editor frames (the UI's st.data_editor tables; the values are shown and edited as text) ----------------------

ROW_COLS = ["area", "parameter", "units", "value", "status", "Q", "source", "overridden by", "line"]
SPREAD_COLS = [
    "name",
    "dist",
    "nominal",
    "spread",
    "relative",
    "units",
    "kv_mode",
    "source",
    "status",
    "Q",
    "note",
    "overridden by",
    "line",
]


def rows_to_frame(rows, overridden=None):
    """Value rows as the UI editor's table.

    Parameters
    ----------
    rows : list of dict
    overridden : dict, optional
        {parameter: overriding file}.

    Returns
    -------
    pandas.DataFrame
    """
    import pandas as pd

    return pd.DataFrame(
        [
            dict(r, value=fmt_value(r["value"]), **{"overridden by": (overridden or {}).get(r["parameter"], "")})
            for r in rows
        ],
        columns=ROW_COLS,
    )


def frame_to_rows(df):
    """Value rows from the UI editor's table.

    Parameters
    ----------
    df : pandas.DataFrame

    Returns
    -------
    list of dict
    """
    out = []
    for r in df.to_dict("records"):
        out.append(
            dict(
                area=r["area"],
                parameter=r["parameter"],
                value=parse_value(str(r["value"])),
                units=r["units"] or "",
                status=r["status"] or "",
                Q=r["Q"] or "",
                source=r["source"] or "",
                line=int(r["line"]),
            )
        )
    return out


def spreads_to_frame(spreads, overridden=None):
    """Spread rows as the UI editor's table.

    Parameters
    ----------
    spreads : list of dict
    overridden : dict, optional
        {name: overriding file}.

    Returns
    -------
    pandas.DataFrame
    """
    import pandas as pd

    return pd.DataFrame(
        [
            dict(
                {k: s.get(k) for k in SPREAD_FIELDS},
                status=s["status"],
                Q=s["Q"],
                note=s["note"],
                line=s["line"],
                units=s.get("units") or s["comment_units"],
                **{"overridden by": (overridden or {}).get(s["name"], "")},
            )
            for s in spreads
        ],
        columns=SPREAD_COLS,
    )


def frame_to_spreads(df, original):
    """Spread rows from the editor frame; the comment's units field is kept from the file.

    Parameters
    ----------
    df : pandas.DataFrame
    original : list of dict
        The file's spread rows.

    Returns
    -------
    list of dict
    """
    by_line = {s["line"]: s for s in original}
    out = []
    for r in df.to_dict("records"):
        o = by_line[int(r["line"])]
        row = {
            k: (None if (isinstance(r.get(k), float) and r.get(k) != r.get(k)) or r.get(k) == "" else r.get(k))
            for k in SPREAD_FIELDS
        }
        row["units"] = o.get("units")  # the mapping's own units key (unchanged by the editor)
        if row.get("relative") is not None:
            row["relative"] = bool(row["relative"])
        out.append(
            dict(
                row,
                comment_units=o["comment_units"],
                status=r["status"] or "",
                Q=r["Q"] or "",
                note=r["note"] or "",
                line=o["line"],
            )
        )
    return out
