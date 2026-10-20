"""SCARA CAD SR6 - offline model, reachability, and G-code generator.

Production RHO4 driver/runner are deliberately NOT imported or distributed.
"""
from __future__ import annotations

import json
import math
from copy import deepcopy
from pathlib import Path
from typing import Any

from scara_kinematics import SR6Geometry, tool_to_flange, flange_to_tool

SCHEMA_VERSION = 4
DEFAULT_TOOL = {"name": "Bez nastroje", "x": 0.0, "y": 0.0, "z": 0.0, "r": 0.0}
DEFAULT_START = {"x": 450.0, "y": 0.0, "z": 100.0, "r": 0.0}
MOVE_TYPES = ("ptp", "rapid", "linear", "arc")
OP_TYPES = MOVE_TYPES + ("delay", "wait_input")
SOLVER = SR6Geometry()  # Purely offline; no connection code exists.


def new_project():
    return {"schema": SCHEMA_VERSION, "name": "Novy projekt", "start": dict(DEFAULT_START),
            "move_to_start": False, "operations": [],
            "tools": [dict(DEFAULT_TOOL)], "active_tool": DEFAULT_TOOL["name"],
            "tcp_compensate": False}


def pose(data):
    return tuple(float(data[key]) for key in ("x", "y", "z", "r"))


def _clean_float(value, field):
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"Neplatne cislo {field}: {value}")
    return result


def normalize_tools(raw):
    """Normalize a portable library of local-flange tool profiles."""
    if not isinstance(raw, list) or not 1 <= len(raw) <= 100:
        raise ValueError("Knihovna musi obsahovat 1..100 nastroju.")
    cleaned = []
    names = set()
    for idx, item in enumerate(raw, 1):
        if not isinstance(item, dict):
            raise ValueError(f"Nastroj {idx}: neplatny zaznam.")
        name = str(item.get("name", "")).strip()
        if not name or len(name) > 80 or name.casefold() in names:
            raise ValueError(f"Nastroj {idx}: nazev chybi, je duplicitni nebo prilis dlouhy.")
        names.add(name.casefold())
        offsets: dict[str, float] = {}
        for axis in ("x", "y", "z", "r"):
            value = _clean_float(item[axis], f"nastroj {name}.{axis}")
            if abs(value) > (500.0 if axis != "r" else 360.0):
                raise ValueError(f"Nastroj {name}: offset {axis} je mimo konfiguracni rozsah.")
            offsets[axis] = value
        cleaned.append({"name": name, **offsets})
    return cleaned


def create_tool_profile(project, name):
    """Add a validated profile, without changing project on failure.

    A new profile copies the currently selected offsets for easier editing.
    Changes are project-local until saved as .scara or as global library.
    """
    new_name = str(name).strip()
    candidate = dict(active_tool(project))
    candidate["name"] = new_name
    profiles = normalize_tools([*project["tools"], candidate])
    project["tools"] = profiles
    project["active_tool"] = new_name
    return candidate


def rename_tool_profile(project, new_name):
    """Rename the active profile, validating before any project mutation."""
    name = str(new_name).strip()
    old = project["active_tool"]
    profiles = normalize_tools([
        {**tool, "name": name} if tool["name"] == old else dict(tool)
        for tool in project["tools"]
    ])
    project["tools"] = profiles
    project["active_tool"] = name


def remove_tool_profile(project):
    """Delete the active profile while retaining at least one profile."""
    if len(project["tools"]) <= 1:
        raise ValueError("V projektu musi zustat alespon jeden nastroj.")
    profiles = normalize_tools([
        tool for tool in project["tools"] if tool["name"] != project["active_tool"]
    ])
    project["tools"] = profiles
    project["active_tool"] = profiles[0]["name"]


def active_tool(project):
    return next(t for t in project["tools"] if t["name"] == project["active_tool"])


def machine_pose(tcp, project):
    return tool_to_flange(tcp, active_tool(project))


def _has_tool_offset(tool):
    return any(abs(tool[axis]) > 1e-9 for axis in ("x", "y", "z", "r"))


def normalize_project(obj):
    if not isinstance(obj, dict) or obj.get("schema") not in (1, 2, 3, SCHEMA_VERSION):
        raise ValueError("Nepodporovana verze formatu projektu .scara (schema 1..4).")
    from_v01 = (obj["schema"] == 1)
    start = obj.get("start")
    if not isinstance(start, dict):
        raise ValueError("Projekt nema platny START XYZR.")
    clean_start = {k: _clean_float(start[k], f"start.{k}") for k in ("x", "y", "z", "r")}
    move_to_start = obj.get("move_to_start", False)
    if not isinstance(move_to_start, bool):
        raise ValueError("move_to_start musi byt true/false.")
    operations = obj.get("operations", [])
    if not isinstance(operations, list) or len(operations) > 10000:
        raise ValueError("Neplatny nebo prilis dlouhy seznam operaci.")
    tools = normalize_tools(obj.get("tools", [dict(DEFAULT_TOOL)]))
    active_name = str(obj.get("active_tool", tools[0]["name"]))
    if active_name not in [t["name"] for t in tools]:
        raise ValueError("Aktivni nastroj neni v knihovne.")
    compensation = obj.get("tcp_compensate", False)
    if not isinstance(compensation, bool):
        raise ValueError("tcp_compensate musi byt true/false.")
    cleaned = []
    for no, raw in enumerate(operations, 1):
        if not isinstance(raw, dict):
            raise ValueError(f"Operace {no}: ocekavan objekt.")
        kind = raw.get("type")
        # In v0.1 the misleading name 'ptp' was exported as linear G0.
        # Preserve old physical semantics on load: migrate it to 'rapid'.
        if from_v01 and kind == "ptp":
            kind = "rapid"
        if kind not in OP_TYPES:
            raise ValueError(f"Operace {no}: neznamy typ '{kind}'.")
        op: dict[str, Any] = {"type": kind}
        if kind in MOVE_TYPES:
            for k in ("x", "y", "z", "r", "feed"):
                op[k] = _clean_float(raw[k], f"operace {no}.{k}")
            if kind == "arc":
                for k in ("mx", "my"):
                    op[k] = _clean_float(raw[k], f"operace {no}.{k}")
        elif kind == "delay":
            op["seconds"] = _clean_float(raw["seconds"], f"operace {no}.seconds")
        elif kind == "wait_input":
            ch = int(raw["channel"])
            if ch < 0 or ch > 9999:
                raise ValueError(f"Operace {no}: cislo vstupu mimo rozsah 0..9999.")
            op["channel"] = ch
            op["level"] = 1 if int(raw["level"]) else 0
        cleaned.append(op)
    return {
        "schema": SCHEMA_VERSION,
        "name": str(obj.get("name", "Novy projekt"))[:120],
        "start": clean_start,
        "move_to_start": move_to_start,
        "tools": tools, "active_tool": active_name, "tcp_compensate": compensation,
        "operations": cleaned,
    }


def load_project(filename):
    with open(filename, "r", encoding="utf-8") as f:
        return normalize_project(json.load(f))


def save_project(filename, project):
    data = normalize_project(project)
    with open(filename, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2, allow_nan=False)
        f.write("\n")


def reachability_error(p, tool=None):
    """Reachability for displayed TCP pose, checked at controller/flange pose."""
    if tool is None:
        tool = DEFAULT_TOOL
    if not all(math.isfinite(v) for v in p):
        return "neplatne TCP souradnice (NaN/Inf)"
    x, y, z, r = tool_to_flange(p, tool)
    if not all(math.isfinite(v) for v in (x, y, z, r)):
        return "neplatne souradnice A4 (NaN/Inf)"
    if math.hypot(x, y) > SOLVER.XY_RADIUS_MAX + 1e-9:
        return "osa A4 XY mimo limit polomeru 590 mm"
    if not SOLVER.Z_MIN <= z <= SOLVER.Z_MAX:
        return "Z mimo rozsah 5..195 mm"
    if not SOLVER.R_MIN <= r <= SOLVER.R_MAX:
        return "R mimo rozsah -175..175 stupnu"
    if not SOLVER.check_xy_kinematics(x, y):
        return "XY bez reseni IK v limitech A1/A2"
    try:
        SOLVER.check_target(x, y, z, r)
    except ValueError as exc:
        return str(exc)
    return None


def circle_from_3_points(a, m, b):
    """Circle and through-MID sweep in XY; returns center/radius/angles/direction."""
    ax, ay = a[:2]
    bx, by = m[:2]
    cx, cy = b[:2]
    cross = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)
    distances = [math.hypot(ax-bx, ay-by), math.hypot(bx-cx, by-cy), math.hypot(cx-ax, cy-ay)]
    if min(distances) < 1e-6 or abs(cross) < 1e-9 * max(1, max(distances)**2):
        raise ValueError("ARC: body START/MID/END jsou shodne nebo temer na primce.")
    denom = 2.0 * (ax*(by-cy) + bx*(cy-ay) + cx*(ay-by))
    a2 = ax*ax + ay*ay
    b2 = bx*bx + by*by
    c2 = cx*cx + cy*cy
    center_x = (a2*(by-cy) + b2*(cy-ay) + c2*(ay-by)) / denom
    center_y = (a2*(cx-bx) + b2*(ax-cx) + c2*(bx-ax)) / denom
    radius = math.hypot(ax-center_x, ay-center_y)
    if radius > 100000 or not math.isfinite(radius):
        raise ValueError("ARC: nestabilni polomer kruznice.")
    start_angle = math.atan2(ay-center_y, ax-center_x)
    mid_angle = math.atan2(by-center_y, bx-center_x)
    end_angle = math.atan2(cy-center_y, cx-center_x)
    if cross > 0:
        sweep = (end_angle - start_angle) % (2*math.pi)
        through = (mid_angle - start_angle) % (2*math.pi)
        code = "G3"
    else:
        sweep = -((start_angle - end_angle) % (2*math.pi))
        through = -((start_angle - mid_angle) % (2*math.pi))
        code = "G2"
    if not (1e-8 < abs(sweep) < 2*math.pi + 1e-8) or not (0 < through / sweep < 1):
        raise ValueError("ARC: bod MID nelezi uvnitr oblouku.")
    return {"cx": center_x, "cy": center_y, "radius": radius,
            "a0": start_angle, "sweep": sweep, "code": code,
            "mid_fraction": through/sweep}


def endpoint(prev, op):
    if op["type"] in MOVE_TYPES:
        return pose(op)
    return prev


def path_samples(start, op, step_mm=2.0):
    """Sample the displayed G0/G1/G2/G3 path; no physical robot state used."""
    kind = op["type"]
    if kind not in MOVE_TYPES:
        return []
    end = pose(op)
    if kind == "ptp":
        # Native joint PTP has no known TCP interpolation in this offline CAD.
        return [end]
    if kind == "arc":
        if abs(start[2]-end[2]) > 1e-6 or abs(start[3]-end[3]) > 1e-6:
            raise ValueError("G2/G3 umoznuje jen XY oblouk pri konstantnim Z a R.")
        info = circle_from_3_points(start, (op["mx"], op["my"]), end)
        count = max(2, math.ceil(abs(info["sweep"])*info["radius"]/step_mm))
        if count > 12000:
            raise ValueError("ARC: prilis mnoho vzorku trajektorie.")
        fractions = sorted({i/count for i in range(1, count+1)} | {info["mid_fraction"]})
        result = []
        for u in fractions:
            angle = info["a0"] + info["sweep"]*u
            p = (info["cx"]+info["radius"]*math.cos(angle),
                 info["cy"]+info["radius"]*math.sin(angle), start[2], start[3])
            result.append(p)
        result[-1] = end
        return result
    length = math.dist(start[:3], end[:3])
    dr = (end[3]-start[3]+180.0)%360.0-180.0
    count = max(1, math.ceil(length/step_mm), math.ceil(abs(dr)/5.0))
    if count > 12000:
        raise ValueError("Prilis dlouha trajektorie.")
    result = []
    for i in range(1, count+1):
        u = i / count
        result.append(tuple(start[j]+(end[j]-start[j])*u for j in range(3)) + (start[3]+dr*u,))
    # Use the exact end. The driver also normalizes angle deltas at boundaries.
    result[-1] = end
    return result


def _joint_delta(a, b):
    return (a-b+180.0)%360.0-180.0


def _joint_continuity(previous, possibilities, maximum=10.0):
    """Retain *all* plausible IK branches from the assumed start joint pose."""
    new_states = []
    for candidate in possibilities:
        for old in previous:
            jumps = [abs(_joint_delta(candidate[k], old[k])) for k in ("a1", "a2", "a4")]
            if max(jumps) <= maximum + 1e-9:
                if not any(abs(candidate["a2"]-existing["a2"]) < 1e-6 for existing in new_states):
                    new_states.append(candidate)
                break
    return new_states


def validate_project(project, include_unsupported=True):
    """Return [(op_index_or_-1, description)] errors, offline advisory only.

    As with the driver, samples are ~2 mm / 5 deg; actual controller constraints,
    robot pose and collision risks must be checked separately.
    """
    errors = []
    start = pose(project["start"])
    tool = active_tool(project)
    if _has_tool_offset(tool) and not project["tcp_compensate"] and include_unsupported:
        errors.append((-1, "TCP offset je nenulovy, ale CAD kompenzace neni potvrzena. "
                          "Over nastaveni TCP v RHO4 a zapni kompenzaci v CADu."))
    err = reachability_error(start, tool)
    if err:
        errors.append((-1, f"START: {err}"))
        states = []
    else:
        states = SOLVER.check_target(*machine_pose(start, project))
    current = start
    for index, op in enumerate(project["operations"]):
        kind = op["type"]
        if kind == "delay":
            if not 0 <= op["seconds"] <= 86400:
                errors.append((index, "DELAY musi byt v intervalu 0..86400 s"))
            continue
        if kind == "wait_input":
            if include_unsupported:
                errors.append((index, "WAIT INPUT: aktualni runner nema prikaz cekani na digitalni vstup."))
            continue
        if kind == "ptp" and include_unsupported:
            errors.append((index, "PTP: runner neumi export nativniho klouboveho PTP. Pouzij G0 (LINEAR), nebo pockej na rozsireni runneru."))
        if kind in ("linear", "arc") and (op["feed"] <= 0 or not math.isfinite(op["feed"])):
            errors.append((index, "F musi byt kladne (mm/min)"))
        # A single native G0/G1 maps a straight *flange* trajectory.
        # With a side-offset TCP and changing R, it would NOT be a straight
        # TCP trajectory. Refuse to export instead of silently changing path.
        side_offset = math.hypot(tool["x"], tool["y"]) > 1e-9
        r_delta = abs(_joint_delta(op["r"], current[3])) if kind in MOVE_TYPES else 0
        if side_offset and r_delta > 1e-6 and kind in ("linear", "rapid") and include_unsupported:
            errors.append((index, "LINEAR/G0 se zmenou R a bocnim TCP offsetem zatim nelze exportovat "
                                  "(skutecna TCP draha by nebyla primka). Rozdel pohyb."))
        try:
            segment = path_samples(current, op)
            # Pure-Z retains physical elbow branch (driver special case).
            machine_start = machine_pose(current, project)
            machine_end = machine_pose(pose(op), project)
            pure_z = (kind in ("linear", "rapid") and
                      abs(machine_end[0]-machine_start[0]) <= 0.05 and
                      abs(machine_end[1]-machine_start[1]) <= 0.05 and
                      abs(_joint_delta(machine_end[3], machine_start[3])) <= 0.05)
            for si, pt in enumerate(segment):
                local_error = reachability_error(pt, tool)
                if local_error and not pure_z:
                    raise ValueError(f"trajektorie {si+1}/{len(segment)}: {local_error}")
                if local_error and pure_z:
                    # Pure-Z keeps the current physical elbow branch, but
                    # still needs valid physical Z/R and XY radial limits.
                    x, y, z, r = machine_pose(pt, project)
                    if not (SOLVER.Z_MIN <= z <= SOLVER.Z_MAX and SOLVER.R_MIN <= r <= SOLVER.R_MAX
                            and math.hypot(x, y) <= SOLVER.XY_RADIUS_MAX + 1e-9):
                        raise ValueError(local_error)
                if states and not pure_z:
                    choices = SOLVER.check_target(*machine_pose(pt, project))
                    following = _joint_continuity(states, choices)
                    if not following:
                        raise ValueError(f"mozny skok kloubu >10° v {si+1}/{len(segment)} (offline IK)")
                    states = following
            if kind == "arc":
                # G-code rendering uses 5 decimals, driver tolerance 0.20 mm.
                info = circle_from_3_points(current, (op["mx"], op["my"]), pose(op))
                r2 = math.hypot(op["x"] - info["cx"], op["y"] - info["cy"])
                if abs(r2-info["radius"]) > 0.001:
                    raise ValueError("Oblouk nema konzistentni polomer.")
        except (ValueError, ZeroDivisionError, OverflowError) as exc:
            errors.append((index, str(exc)))
            states = []
        current = endpoint(current, op)
    return errors


def _n(number, digits=4):
    value = round(float(number), digits)
    if value == 0: value = 0.0
    return f"{value:.{digits}f}"


def generate_gcode(project):
    """Generate machine-absolute G-code; rejects unsupported PTP and WAIT INPUT."""
    project = normalize_project(project)
    problems = validate_project(project)
    if problems:
        raise ValueError("Export zablokovan:\n" + "\n".join(
            (f"START: {msg}" if ix == -1 else f"Operace {ix+1}: {msg}")
            for ix, msg in problems))
    tool = active_tool(project)
    start_machine = machine_pose(pose(project["start"]), project)
    lines = [
        "; SCARA CAD SR6 - G-code for rho4_gcode_runner.py",
        "; MACHINE ABSOLUTE COORDINATES: RUN WITH --machine-coordinates",
        "; START TCP: " + " ".join(f"{a}{_n(v)}" for a, v in zip("XYZR", pose(project["start"]))),
        "; ACTIVE TOOL: " + tool["name"],
        "; LOCAL FLANGE OFFSETS: " + " ".join(f"{a}{_n(tool[k])}" for a, k in zip("XYZR", ("x", "y", "z", "r"))),
        "; CONTROLLER TCP OFFSET MUST BE DISABLED WHEN CAD COMPENSATES",
        "; G0 = LINEAR (CMD16), native joint PTP not supported",
        "G21 G90 G17 G91.1 G94",
        "G64",
    ]
    if project["move_to_start"]:
        lines.extend([
            "; MOVE TO START ENABLED: G0 LINEAR from ACTUAL robot position",
            "; WARNING: approach from actual position was NOT checked offline",
            "G0 " + " ".join(f"{axis}{_n(value)}" for axis, value in zip("XYZR", start_machine)),
        ])
    else:
        lines.append("; MOVE TO START DISABLED: robot MUST already be at START")
    current = pose(project["start"])
    for ix, op in enumerate(project["operations"], 1):
        kind = op["type"]
        lines.append(f"; {ix:03d} {kind.upper()}")
        if kind in MOVE_TYPES:
            # All movement types have a non-optional target pose.
            flange_end = machine_pose(pose(op), project)
            if kind == "rapid":
                lines.append("; G0 is LINEAR in the current runner, not native joint PTP")
                lines.append("G0 " + " ".join(f"{axis}{_n(value)}" for axis, value in zip("XYZR", flange_end)))
            elif kind == "linear":
                lines.append("G1 " + " ".join(f"{axis}{_n(value)}" for axis, value in zip("XYZR", flange_end)) + f" F{_n(op['feed'], 3)}")
            elif kind == "arc":
                info = circle_from_3_points(current, (op["mx"], op["my"]), pose(op))
                # With constant R and Z the TCP->flange XY transform is a
                # translation. I/J relative to the start remain unchanged.
                i = info["cx"] - current[0]
                j = info["cy"] - current[1]
                lines.append(f"{info['code']} X{_n(flange_end[0])} Y{_n(flange_end[1])} "
                             f"I{_n(i, 5)} J{_n(j, 5)} F{_n(op['feed'], 3)}")
            else:
                raise AssertionError("PTP must be rejected by validation")
        elif kind == "delay":
            lines.append(f"G4 P{_n(op['seconds'], 3)}")
        else:
            raise AssertionError("Unsupported operation escaped validation")
        current = endpoint(current, op)
    lines += ["M2", ""]
    return "\n".join(lines)
