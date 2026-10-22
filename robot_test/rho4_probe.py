"""SCARA SR6 – initial Z probing helper (v0.5.0, deliberately standalone).

No CAD or runner modifications. This module never moves XY and never launches
an engraving job. A live probe requires explicit --confirm PROBE and calibrated
probe geometry. Offline simulation cannot write a valid reference file.

Probe geometry: get_position() / @MPOS are interpreted as flange XYZR.
Tool offsets match CAD's scara_kinematics.flange_to_tool convention.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import sys
import tempfile

DEFAULT_SETTINGS = Path(__file__).with_name("rho4_machine.json")


class ProbeError(RuntimeError):
    pass


def number(value, name: str) -> float:
    if isinstance(value, bool):
        raise ProbeError(f"{name}: ocekavam cislo")
    try:
        v = float(value)
    except (TypeError, ValueError) as exc:
        raise ProbeError(f"{name}: ocekavam cislo") from exc
    if not math.isfinite(v):
        raise ProbeError(f"{name}: neplatne cislo")
    return v


def load_settings(path: Path | str = DEFAULT_SETTINGS) -> dict:
    with Path(path).open(encoding="utf-8") as f:
        settings = json.load(f)
    if not isinstance(settings, dict) or settings.get("schema") != 1:
        raise ProbeError("Nepodporovana konfigurace stroje (schema musi byt 1)")
    machine = settings.get("machine", {})
    probe = settings.get("probe", {})
    actuator = settings.get("actuator", {})
    if not all(isinstance(x, dict) for x in (machine, probe, actuator)):
        raise ProbeError("machine/probe/actuator musi byt objekty")
    if not str(machine.get("host", "")).strip():
        raise ProbeError("machine.host nesmi byt prazdny")
    port = number(machine.get("port"), "machine.port")
    if not port.is_integer() or not (1 <= port <= 65535):
        raise ProbeError("machine.port mimo rozsah")
    if probe.get("mode") != "manual":
        raise ProbeError("Zatim je podporovana pouze rucni vymena sondy (manual)")
    if actuator.get("mode") != "none":
        raise ProbeError("Aktuator zatim NENI implementovan – odmitam provest pohyb")
    if type(probe.get("calibrated")) is not bool:
        raise ProbeError("probe.calibrated musi byt true/false")
    offset = probe.get("tip_offset_mm")
    if not isinstance(offset, dict):
        raise ProbeError("Chybi probe.tip_offset_mm")
    for axis in ("x", "y", "z"):
        if abs(number(offset.get(axis), f"probe.tip_offset_mm.{axis}")) > 300:
            raise ProbeError("Offset sondy je podezrele velky")
    sign = probe.get("approach_z_sign")
    if sign not in (-1, 1) or type(sign) is bool:
        raise ProbeError("approach_z_sign musi byt -1 nebo 1 – over skutecny smer osy Z")
    limits = {
        "max_travel_mm": (0.1, 25),
        "speed_mm_s": (0.05, 1),
        "timeout_s": (1, 180),
        "xy_tolerance_mm": (0.01, 2.0),
    }
    for key, (lo, hi) in limits.items():
        value = number(probe.get(key), f"probe.{key}")
        if not lo <= value <= hi:
            raise ProbeError(f"probe.{key} musi byt {lo}..{hi}")
    return settings


def tip_position(flange, offset) -> tuple[float, float, float, float]:
    """Matches scara_kinematics.flange_to_tool for X/Y/Z, with no tip R."""
    x, y, z, r = (number(v, f"flange[{i}]") for i, v in enumerate(flange))
    angle = math.radians(r)
    dx = number(offset["x"], "offset.x")
    dy = number(offset["y"], "offset.y")
    dz = number(offset["z"], "offset.z")
    return (x + dx * math.cos(angle) - dy * math.sin(angle),
            y + dx * math.sin(angle) + dy * math.cos(angle),
            z + dz, r)


def reference_from_hit(settings: dict, measured_flange: tuple,
                       cad_surface_z_mm: float, *, simulated=False) -> dict:
    offset = settings["probe"]["tip_offset_mm"]
    measured = tuple(number(v, "namereny bod") for v in measured_flange)
    if len(measured) != 4:
        raise ProbeError("Namereny bod musi byt XYZR")
    tip = tip_position(measured, offset)
    cad_z = number(cad_surface_z_mm, "cad_surface_z_mm")
    return {
        "schema": 1,
        "valid_for_robot": not simulated,
        "source": "SIMULACE" if simulated else "PCPROBE_CMD39_MPOS",
        "utc": datetime.now(timezone.utc).isoformat(),
        "measured_flange_xyzr": list(measured),
        "probe_tip_machine_xyzr": list(tip),
        "cad_surface_z_mm": cad_z,
        "delta_z_machine_mm": tip[2] - cad_z,
        "calibration_confirmed": settings["probe"]["calibrated"],
        "note": "Absolutni strojni Z posun pro CAD export; NENI G92."
    }


def atomic_write_json(path: Path | str, data: dict) -> None:
    path = Path(path).resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(prefix=".probe-", suffix=".tmp", dir=path.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
            f.write("\n")
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, path)
    finally:
        if os.path.exists(tmp):
            os.unlink(tmp)


def assess_ready(driver, settings: dict, expected_x: float, expected_y: float):
    """Nonmoving validation before any command that can move the robot."""
    if not settings["probe"]["calibrated"]:
        raise ProbeError("Sonda neni kalibrovana; nastavit true az po overeni offsetu")
    if not driver.ping():
        raise ProbeError("RHO4 neodpovedelo na PING")
    status = driver.get_status()
    for key in ("alarm", "automatic", "referenced", "in_position"):
        if key not in status:
            raise ProbeError(f"Neni dostupny stav robota: {key}")
    if status["alarm"] or not all(status[k] for k in ("automatic", "referenced", "in_position")):
        raise ProbeError(f"Robot neni pripraven: {status}")
    state, error, proc = driver.get_move_state()
    if state not in (0, 2) or proc != -1 or error != 0:
        raise ProbeError(f"Motion neni v klidu: state={state}, error={error}, proc={proc}")
    io = driver.get_io_status()
    if io.get("probe_input") is not False:
        raise ProbeError("Sonda jiz sepnuta nebo stav vstupu neznamy – odmitam pohyb")
    if io.get("spindle_state") is not False:
        raise ProbeError("Vreteno je aktivni – odmitam pohyb")
    flange = driver.get_position()
    tip = tip_position(flange, settings["probe"]["tip_offset_mm"])
    tolerance = float(settings["probe"]["xy_tolerance_mm"])
    if math.hypot(tip[0] - expected_x, tip[1] - expected_y) > tolerance:
        raise ProbeError(
            f"Sonda neni v mernem XY: tip=({tip[0]:.3f},{tip[1]:.3f}), "
            f"ocekavano=({expected_x:.3f},{expected_y:.3f}), tolerance={tolerance:.3f}"
        )
    return tuple(float(v) for v in flange)


def probe_once(driver, settings: dict, *, start_x: float, start_y: float,
               cad_surface_z: float, simulated: bool = False) -> dict:
    """One Z-only probe at the current XY; no XY repositioning or retract."""
    start_x = number(start_x, "start_x")
    start_y = number(start_y, "start_y")
    cad_surface_z = number(cad_surface_z, "cad_surface_z")
    initial = assess_ready(driver, settings, start_x, start_y)
    probe = settings["probe"]
    target_z = initial[2] + probe["approach_z_sign"] * probe["max_travel_mm"]
    if not 5.0 <= target_z <= 195.0:
        raise ProbeError(f"Cil sondovani Z={target_z:.3f} mimo hardcoded limit 5..195 mm")
    moved = False
    try:
        moved = True
        result = driver.probe_linear(initial[0], initial[1], target_z, initial[3],
                                     speed_mm_s=probe["speed_mm_s"],
                                     timeout=probe["timeout_s"])
        if not result or result.get("hit") is not True or result.get("no_hit"):
            raise ProbeError("PCPROBE nedal jednoznacne potvrzeny HIT")
        measured = result.get("measured")
        if not isinstance(measured, (tuple, list)) or len(measured) != 4:
            raise ProbeError("CMD39 vratil neplatnou polohu")
        mz = number(measured[2], "measured.Z")
        direction = probe["approach_z_sign"]
        distance = (mz - initial[2]) * direction
        if not -0.1 <= distance <= probe["max_travel_mm"] + 0.1:
            raise ProbeError("Trigger mimo rozsah probe pohybu")
        if (abs(number(measured[0], "measured.X") - initial[0]) > 0.1 or
            abs(number(measured[1], "measured.Y") - initial[1]) > 0.1):
            raise ProbeError("Probe zmenil XY, nebezpecny vysledek")
        return reference_from_hit(settings, tuple(measured), cad_surface_z, simulated=simulated)
    except BaseException:
        if moved:
            # Best effort: hardware E-STOP is independent of Python.
            try:
                driver.stop()
            except Exception:
                pass
        raise


def _make_driver(settings: dict):
    # The production driver is located next to this helper in robot_test/.
    from rho4_driver import Rho4
    return Rho4(host=settings["machine"]["host"], port=int(settings["machine"]["port"]))


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="SR6 probe V0.5 – samostatny diagnosticky helper")
    parser.add_argument("--config", type=Path, default=DEFAULT_SETTINGS)
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("validate", help="Pouze kontrola JSON, bez pripojeni")
    sub.add_parser("diagnose", help="Jen cteni stavu robota/sondy, bez pohybu")
    s = sub.add_parser("simulate", help="Ciste offline prepocet, nikdy neplati jako reference")
    s.add_argument("--hit-z", type=float, required=True, help="Simulovane namerene strojni Z priruby")
    s.add_argument("--cad-surface-z", type=float, required=True)
    p = sub.add_parser("probe", help="FYZICKY pohyb Z; vyzaduje kalibraci a potvrzeni")
    p.add_argument("--start-x", type=float, required=True, help="XY merneho bodu v absolutnich stroj. souradnicich")
    p.add_argument("--start-y", type=float, required=True)
    p.add_argument("--cad-surface-z", type=float, required=True, help="Z povrchu ve vykresu v mm")
    p.add_argument("--confirm", required=True, help="Musi byt doslova PROBE")
    p.add_argument("--simulator", action="store_true", help="Povolí localhost test, výsledek není platný pro robot")
    p.add_argument("--output", type=Path, default=Path("rho4_z_reference.json"))
    args = parser.parse_args(argv)
    try:
        settings = load_settings(args.config)
        if args.command == "validate":
            print("Konfigurace OK. Kalibrovano:", settings["probe"]["calibrated"])
            print("Zadny pohyb ani spojeni se strojem.")
            return 0
        if args.command == "simulate":
            result = reference_from_hit(settings, (0., 0., args.hit_z, 0.), args.cad_surface_z,
                                        simulated=True)
            print(json.dumps(result, ensure_ascii=False, indent=2))
            print("SIMULACE: vysledek nelze pouzit pro automaticke spusteni robota!")
            return 0
        if args.command == "probe" and args.confirm != "PROBE":
            raise ProbeError("Pro skutecny pohyb je nutne --confirm PROBE")
        if args.command == "probe" and not settings["probe"]["calibrated"]:
            raise ProbeError("Sonda neni kalibrovana – odmitam realny probe")
        if args.command == "probe":
            loopback = settings["machine"]["host"].strip().lower() in ("127.0.0.1", "localhost", "::1")
            if loopback != args.simulator:
                raise ProbeError("Pro localhost vyzadujeme --simulator; pro realny SR6 --simulator zakazan")
            # Invalidate an older result first: a failed new probe must never
            # leave a previously valid reference available for a later job.
            atomic_write_json(args.output, {
                "schema": 1, "valid_for_robot": False,
                "status": "PENDING_OR_FAILED", "utc": datetime.now(timezone.utc).isoformat()
            })
        drv = _make_driver(settings)
        try:
            drv.connect()
            if args.command == "diagnose":
                print("Stav robota:", drv.get_status())
                print("I/O:", drv.get_io_status())
                print("Poloha priruby:", drv.get_position())
                return 0
            result = probe_once(drv, settings, start_x=args.start_x, start_y=args.start_y,
                                cad_surface_z=args.cad_surface_z, simulated=args.simulator)
            atomic_write_json(args.output, result)
            print(json.dumps(result, ensure_ascii=False, indent=2))
            print("HIT overen.")
            if args.simulator:
                print("SIMULATOR: reference je zamerne neplatna pro skutecneho robota.")
            print("POZOR: sonda zustava na skutecne koncove Z; automaticky odjezd neni implementovan.")
            print("Zatim NEPOUSTEJ CAD G-code podle tohoto souboru bez dalsi kontroly.")
            return 0
        finally:
            drv.disconnect()
    except (OSError, ValueError, ProbeError, RuntimeError, TimeoutError) as exc:
        print("CHYBA:", exc, file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
