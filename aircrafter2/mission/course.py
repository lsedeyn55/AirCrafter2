"""The flight course and the take-off switch: config data model, presets and validation (docs/modules/mission.md).

A course is one lap from the start line, as an ordered list of legs:
    straight   length_ft > 0 and the along-track wind on it: head, tail or none (crosswind: no along-track component)
    turn       angle_deg > 0 (turns ignore the wind unless turn_drift is on)
Lengths in feet and angles in degrees, as the rules write them. ``lap1_legs`` optionally gives a different first lap.

Presets are YAML files in configs/courses/ (``name``, ``citation``, ``legs``, optional ``lap1_legs``). The default,
``dbf_2025``, is Table 3.5 of Kallarackel (2025), the course the lap model always flew: legs given inline override the
preset.

Validation (``check_course``):
    errors     an empty lap, a lap with no straight, a length or angle <= 0, and, with take-off in the mission time,
               a lap 1 that does not start with a head-wind straight (the ground roll is into the wind)
    warnings   total turning that is not a multiple of 360 deg, and head-wind and tail-wind distances that differ (the
               lap would not end at the start line). The lap model does not need a closed course, so these only warn.

The take-off block (``TakeoffConfig``) is the switch of docs/TAKEOFF_COURSE_PLAN.md: off by default, which is the
original model (lap 1 starts in the air at V_stall).
"""

import warnings
from pathlib import Path
from typing import Literal, Optional

from pydantic import BaseModel, ConfigDict, Field, model_validator

COURSES_DIR = Path(__file__).resolve().parents[2] / "configs" / "courses"
SHIPPED_PRESETS = ("dbf_2025", "dbf_2025_360_mid", "oval")
WIND_SIGN = {"head": 1.0, "tail": -1.0, "none": 0.0}
TURN_TOL_DEG = 1e-6
LENGTH_TOL_FT = 1e-6


class Leg(BaseModel):
    model_config = ConfigDict(extra="forbid")
    kind: Literal["straight", "turn"]
    length_ft: Optional[float] = Field(None, description="straights: ground length (ft)")
    angle_deg: Optional[float] = Field(None, description="turns: heading change (deg)")
    wind: Literal["head", "tail", "none"] = Field("none", description="straights: along-track wind direction")

    def size(self):
        return self.length_ft if self.kind == "straight" else self.angle_deg


class TakeoffConfig(BaseModel):
    model_config = ConfigDict(extra="forbid")
    in_mission_time: bool = Field(False, description="add the ground roll (rest to lift-off) to lap 1's time and "
                                  "energy; rules p. 11-12: time starts at throttle advance. False: the original model")
    k_lof: float = Field(1.1, gt=1.0, description="lift-off at k_lof V_stall (performance_modeller.m l. 405: 'full "
                         "power until 1.1 Vstall')")
    CL_ground: float = Field(0.15, ge=0.0, description="ground-roll lift coefficient, PLACEHOLDER "
                             "(docs/TAKEOFF_COURSE_PLAN.md; OPEN_QUESTIONS)")
    start_offset_ft: float = Field(0.0, ge=0.0, description="brake release behind the start line: lengthens lap 1's "
                                   "first straight (take-off on only); PLACEHOLDER")


class CourseConfig(BaseModel):
    model_config = ConfigDict(extra="forbid")
    preset: Optional[str] = Field("dbf_2025", description="a preset in configs/courses/ (name or path)")
    legs: Optional[list[Leg]] = Field(None, description="one lap from the start line, in order (overrides the preset)")
    lap1_legs: Optional[list[Leg]] = Field(None, description="a different first lap (default: the same legs)")
    pilot_reaction_time_s: float = Field(3.0, ge=0.0, description="s per lap (performance_modeller.m)")
    turn_drift: bool = Field(False, description="carry each turn's wind drift (wind x turn time) to the next straight "
                             "(docs/RESULTS_2026_PLAN.md §4.7). False: the original model (turns ignore the wind)")
    lap1_cluster: float = Field(2.0, ge=1.0, description="grid exponent of lap 1's first straight (nodes at "
                                "s_i = L (i/N)^p, clustered at its slow start; OPEN_QUESTIONS Q45). 1 = the uniform "
                                "grid of every result before 2026-10-27")
    takeoff: TakeoffConfig = TakeoffConfig()

    @model_validator(mode="after")
    def _check(self):
        lap1, other = self.resolve()
        errors, warns = check_course(lap1, other, self.takeoff.in_mission_time)
        if errors:
            raise ValueError("course: " + "; ".join(errors))
        for w in warns:
            warnings.warn(f"course: {w}", stacklevel=2)
        return self

    def resolve(self):
        """(lap-1 legs, lap-2+ legs) as lists of Leg: inline legs, else the preset's."""
        if self.legs is not None:
            other = list(self.legs)
            lap1 = list(self.lap1_legs) if self.lap1_legs is not None else other
            return lap1, other
        if self.preset is None:
            raise ValueError("course: give a preset or legs")
        p_lap1, p_other = load_preset(self.preset)
        lap1 = list(self.lap1_legs) if self.lap1_legs is not None else p_lap1
        return lap1, p_other

    def lap_options(self):
        """(lap1, lap_other) in the lap model's leg format (lap_legs)."""
        lap1, other = self.resolve()
        return lap_legs(lap1), lap_legs(other)

    def banner_stowed_legs(self):
        """Lap-1 leg indices flown before the banner is deployed: up to the first downwind (tail-wind) straight that
        follows the first turn (rules 2026 p. 20: deployed 'after the first upwind turn ... on the down wind leg')."""
        lap1, _ = self.resolve()
        seen_turn = False
        for i, leg in enumerate(lap1):
            if leg.kind == "turn":
                seen_turn = True
            elif seen_turn and leg.wind == "tail":
                return tuple(range(i))
        raise ValueError("course: no downwind straight after a turn in lap 1, so the banner cannot be deployed")

    def summary(self):
        """The course actually flown, for a run's outputs (results.json)."""
        lap1, other = self.resolve()
        return dict(preset=self.preset if self.legs is None else None,
                    legs=[leg.model_dump(exclude_none=True) for leg in other],
                    lap1_legs=None if lap1 == other else [leg.model_dump(exclude_none=True) for leg in lap1],
                    lap_length_ft=lap_length_ft(other), pilot_reaction_time_s=self.pilot_reaction_time_s,
                    takeoff=self.takeoff.model_dump(), turn_drift=self.turn_drift)


def preset_path(name):
    p = Path(name)
    if p.suffix in (".yaml", ".yml") and p.exists():
        return p
    return COURSES_DIR / f"{name}.yaml"


def list_presets():
    return sorted(p.stem for p in COURSES_DIR.glob("*.yaml"))


def load_preset_data(name):
    from ..config.loader import load_yaml

    path = preset_path(name)
    if not path.exists():
        raise ValueError(f"course preset {name!r} not found (configs/courses/: {', '.join(list_presets())})")
    return load_yaml(path)


def load_preset(name):
    data = load_preset_data(name)
    other = [Leg(**leg) for leg in data["legs"]]
    lap1 = [Leg(**leg) for leg in data["lap1_legs"]] if data.get("lap1_legs") else other
    return lap1, other


def save_preset(name, legs, lap1_legs=None, citation="user-defined course", directory=None):
    """Write configs/courses/<name>.yaml; refuses the shipped presets and names that are not plain identifiers."""
    import yaml

    if name in SHIPPED_PRESETS:
        raise ValueError(f"{name!r} is a shipped preset: choose another name")
    if not name or not all(ch.isalnum() or ch in "_-" for ch in name):
        raise ValueError(f"preset name {name!r}: use letters, digits, '_' and '-' only")
    legs = [Leg(**leg) if isinstance(leg, dict) else leg for leg in legs]
    lap1 = None if lap1_legs is None else [Leg(**leg) if isinstance(leg, dict) else leg for leg in lap1_legs]
    errors, _ = check_course(lap1 or legs, legs, takeoff_on=False)
    if errors:
        raise ValueError("course: " + "; ".join(errors))
    data = dict(name=name, citation=citation, legs=[leg.model_dump(exclude_none=True) for leg in legs])
    if lap1 is not None:
        data["lap1_legs"] = [leg.model_dump(exclude_none=True) for leg in lap1]
    path = Path(directory or COURSES_DIR) / f"{name}.yaml"
    path.write_text(yaml.safe_dump(data, sort_keys=False))
    return path


def _check_lap(legs, label):
    errors, warns = [], []
    if not legs:
        return [f"{label} is empty"], []
    for i, leg in enumerate(legs, start=1):
        size = leg.size()
        if size is None or not size > 0.0:
            what = "length_ft" if leg.kind == "straight" else "angle_deg"
            errors.append(f"{label} leg {i} ({leg.kind}): {what} must be > 0, got {size}")
    straights = [leg for leg in legs if leg.kind == "straight"]
    if not straights:
        errors.append(f"{label} has no straight")
    if errors:
        return errors, warns
    turning = sum(leg.angle_deg for leg in legs if leg.kind == "turn")
    if abs(turning / 360.0 - round(turning / 360.0)) * 360.0 > TURN_TOL_DEG or round(turning / 360.0) == 0:
        warns.append(f"{label}: total turning {turning:g} deg is not a (non-zero) multiple of 360 deg")
    head = sum(leg.length_ft for leg in straights if leg.wind == "head")
    tail = sum(leg.length_ft for leg in straights if leg.wind == "tail")
    if abs(head - tail) > LENGTH_TOL_FT:
        warns.append(f"{label}: head-wind distance {head:g} ft and tail-wind distance {tail:g} ft differ (the lap "
                     "would not end at the start line)")
    return errors, warns


def check_course(lap1, other, takeoff_on=False):
    """(errors, warnings) of a course given as lists of Leg."""
    errors, warns = _check_lap(other, "lap")
    if lap1 is not other and lap1 != other:
        e1, w1 = _check_lap(lap1, "lap 1")
        errors += e1
        warns += w1
    if takeoff_on and lap1 and not (lap1[0].kind == "straight" and lap1[0].wind == "head"):
        errors.append("take-off in the mission time needs lap 1 to start with a head-wind straight (the ground roll "
                      "is into the wind)")
    return errors, warns


def lap_legs(legs):
    """Legs in the lap model's format: ("s", length_ft, wind_sign) and ("t", angle_deg, None)."""
    return [("s", float(leg.length_ft), WIND_SIGN[leg.wind]) if leg.kind == "straight"
            else ("t", float(leg.angle_deg), None) for leg in legs]


def lap_length_ft(legs):
    return float(sum(leg.length_ft for leg in legs if leg.kind == "straight"))


def total_turning_deg(legs):
    return float(sum(leg.angle_deg for leg in legs if leg.kind == "turn"))


def rows_from_legs(legs):
    """Leg-table rows (the UI's editor): order, kind, length_ft (straights), angle_deg (turns), wind (straights)."""
    return [dict(order=i, kind=leg.kind, length_ft=leg.length_ft if leg.kind == "straight" else None,
                 angle_deg=leg.angle_deg if leg.kind == "turn" else None,
                 wind=leg.wind if leg.kind == "straight" else None) for i, leg in enumerate(legs, start=1)]


def legs_from_rows(rows, label="lap"):
    """(legs, errors) from leg-table rows, sorted by their order column (ties keep the table order); rows without a
    kind are skipped. A missing length or angle is an error, as is a row that does not make a Leg."""
    def num(x):
        try:
            x = float(x)
        except (TypeError, ValueError):
            return None
        return None if x != x else x  # NaN (an empty cell) -> None

    keyed = []
    for i, r in enumerate(rows):
        o = num(r.get("order"))
        keyed.append((o if o is not None else float("inf"), i, r))
    legs, errors = [], []
    for n, (_, _, r) in enumerate(sorted(keyed, key=lambda t: (t[0], t[1])), start=1):
        kind = r.get("kind")
        if kind not in ("straight", "turn"):
            if kind not in (None, "") and kind == kind:
                errors.append(f"{label} leg {n}: kind must be straight or turn, got {kind!r}")
            continue
        size = num(r.get("length_ft" if kind == "straight" else "angle_deg"))
        if size is None:
            errors.append(f"{label} leg {n} ({kind}): give its {'length (ft)' if kind == 'straight' else 'angle (deg)'}")
            continue
        wind = r.get("wind") if kind == "straight" else "none"
        wind = wind if wind in WIND_SIGN else "none"
        try:
            legs.append(Leg(kind=kind, **({"length_ft": size, "wind": wind} if kind == "straight" else {"angle_deg": size})))
        except ValueError as exc:
            errors.append(f"{label} leg {n}: {exc}")
    return legs, errors
