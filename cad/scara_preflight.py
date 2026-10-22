"""Small offline G-code statistics helper for SCARA CAD SR6.

This intentionally does NOT predict physical PTP paths, acceleration, G0
speed, collision clearance, or RHO4 runtime scheduling. It never connects to
or commands the robot. Source of truth is the generated machine-space G-code.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import re

_WORD = re.compile(r"([A-Z])\s*([-+]?(?:\d+(?:\.\d*)?|\.\d+))")
_MOTION = re.compile(r"^G(0|1|2|3|4|100|101)(?!\d)")


@dataclass(frozen=True)
class ProgramStatistics:
    feed_length_mm: float = 0.0       # G1/G2/G3; machine XYZ, incl. Z plunges
    rapid_length_mm: float = 0.0      # G0; except unknown initial approach
    feed_time_s: float = 0.0          # ideal length / programmed F
    dwell_time_s: float = 0.0         # G4
    feed_moves: int = 0
    rapid_moves: int = 0
    ptp_moves: int = 0
    rotate_moves: int = 0
    dwell_moves: int = 0
    unknown_initial_rapid: bool = False
    unknown_feed_moves: int = 0

    def as_text(self) -> str:
        lines = [
            "PREHLED PROGRAMU – odhad z vysledneho G-codu (osa A4)",
            f"G1/G2/G3: {self.feed_moves} pohybu, delka {self.feed_length_mm:.2f} mm",
            f"Idealni cas G1/G2/G3 pri F: {self.feed_time_s:.1f} s",
            f"G4 prodlevy: {self.dwell_moves}x, celkem {self.dwell_time_s:.1f} s",
            f"G0: {self.rapid_moves} prejezdu, znama delka {self.rapid_length_mm:.2f} mm (cas neznamy)",
            f"G100 PTP: {self.ptp_moves}x; G101 ROTATE: {self.rotate_moves}x (cas a skutecna draha nezname)",
            f"Soucet idealnich G1/G2/G3 + G4: {self.feed_time_s+self.dwell_time_s:.1f} s – NENI to celkovy cas programu.",
        ]
        if self.unknown_initial_rapid:
            lines.append("POZOR: prvni G0 na START ma neznamou vychozi polohu; jeho delka neni zapoctena.")
        if self.unknown_feed_moves:
            lines.append(f"POZOR: {self.unknown_feed_moves} posuvu bez platneho F nema casovy odhad.")
        lines.extend([
            "Nezahrnuje dynamiku/akceleraci, preruseni, zpozdeni I/O ani cas G0/PTP/ROTATE.",
            "Neoveruje kolize, bezpecne prejezdy ani skutecny stav a polohu robota.",
        ])
        return "\n".join(lines)


def analyze_gcode(gcode: str, start_machine_xyzr, *, unknown_initial_rapid: bool = False) -> ProgramStatistics:
    """Statistics of supported, already-rendered RHO4 CAD G-code.

    The first G0 generated for move_to_start has an *unknown actual origin*.
    When unknown_initial_rapid=True its length is not counted. Later points
    are still tracked accurately in nominal machine space.
    """
    current = [float(x) for x in start_machine_xyzr]
    if len(current) != 4 or not all(map(math.isfinite, current)):
        raise ValueError("START XYZ/R musi byt konecne hodnoty")
    feed = None  # modal G94 mm/min
    flen = rlen = ftime = dtime = 0.0
    nf = nr = np = nrot = ndwell = missing_f = 0
    skipped_first = False

    for original in gcode.splitlines():
        line = original.split(";", 1)[0].strip().upper()
        match = _MOTION.match(line)
        if not match:
            continue
        g = int(match.group(1))
        words = {k: float(v) for k, v in _WORD.findall(line)}
        if "F" in words:
            feed = words["F"]
        if g == 4:
            ndwell += 1
            dtime += max(0.0, words.get("P", 0.0))
            continue
        if g == 101:
            nrot += 1
            if "R" in words:
                current[3] = words["R"]
            continue
        target = [words.get(axis, current[i]) for i, axis in enumerate("XYZR")]
        if g == 100:
            np += 1
            current = target
            continue
        if g == 0:
            nr += 1
            if unknown_initial_rapid and not skipped_first:
                skipped_first = True
            else:
                rlen += math.dist(current[:3], target[:3])
            current = target
            continue
        if g == 1:
            length = math.dist(current[:3], target[:3])
        elif g in (2, 3):
            cx, cy = current[0] + words.get("I", 0.0), current[1] + words.get("J", 0.0)
            radius = math.hypot(current[0] - cx, current[1] - cy)
            if math.hypot(current[0] - target[0], current[1] - target[1]) < 0.000001:
                sweep = 2.0 * math.pi
            else:
                a0 = math.atan2(current[1] - cy, current[0] - cx)
                a1 = math.atan2(target[1] - cy, target[0] - cx)
                sweep = (a0 - a1) % (2 * math.pi) if g == 2 else (a1 - a0) % (2 * math.pi)
            length = math.hypot(radius * sweep, target[2] - current[2])
        else:
            continue
        nf += 1
        flen += length
        if feed is None or feed <= 0 or not math.isfinite(feed):
            missing_f += 1
        else:
            ftime += 60.0 * length / feed
        current = target

    return ProgramStatistics(
        feed_length_mm=flen, rapid_length_mm=rlen,
        feed_time_s=ftime, dwell_time_s=dtime,
        feed_moves=nf, rapid_moves=nr, ptp_moves=np, rotate_moves=nrot,
        dwell_moves=ndwell, unknown_initial_rapid=skipped_first,
        unknown_feed_moves=missing_f,
    )
