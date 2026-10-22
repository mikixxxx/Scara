"""SR6 Mono Stroke - self-contained single-line engraving alphabet.

Original vector stroke shapes for this project, no external font file or
runtime dependency. Coordinates: 0..6 horizontally, 0..8 vertically.
Origin = left baseline; glyph height = 8 units. All paths are open strokes.
You may use and modify this font freely with the SCARA CAD source.
"""
from __future__ import annotations

import math

# Each '|' starts a new disconnected stroke (pen-up); x,y vertices are
# separated by spaces. No outline doubling or filled glyphs.
_GLYPHS = {
    "A": "0,0 0,6 2,8 4,8 6,6 6,0|0,4 6,4",
    "B": "0,0 0,8 4,8 6,6 6,5 4,4 0,4|4,4 6,3 6,2 4,0 0,0",
    "C": "6,7 4,8 2,8 0,6 0,2 2,0 4,0 6,1",
    "D": "0,0 0,8 3,8 6,5 6,3 3,0 0,0",
    "E": "6,8 0,8 0,0 6,0|0,4 5,4",
    "F": "0,0 0,8 6,8|0,4 5,4",
    "G": "6,7 4,8 2,8 0,6 0,2 2,0 4,0 6,2 6,4 4,4",
    "H": "0,0 0,8|6,0 6,8|0,4 6,4",
    "I": "0,8 6,8|3,8 3,0|0,0 6,0",
    "J": "0,2 2,0 4,0 6,2 6,8",
    "K": "0,0 0,8|6,8 0,3|2,5 6,0",
    "L": "0,8 0,0 6,0",
    "M": "0,0 0,8 3,4 6,8 6,0",
    "N": "0,0 0,8 6,0 6,8",
    "O": "2,0 4,0 6,2 6,6 4,8 2,8 0,6 0,2 2,0",
    "P": "0,0 0,8 4,8 6,6 6,5 4,4 0,4",
    "Q": "2,0 4,0 6,2 6,6 4,8 2,8 0,6 0,2 2,0|3,2 6,-1",
    "R": "0,0 0,8 4,8 6,6 6,5 4,4 0,4|3,4 6,0",
    "S": "6,7 4,8 2,8 0,6 2,4 4,4 6,2 4,0 2,0 0,1",
    "T": "0,8 6,8|3,8 3,0",
    "U": "0,8 0,2 2,0 4,0 6,2 6,8",
    "V": "0,8 3,0 6,8",
    "W": "0,8 1,0 3,4 5,0 6,8",
    "X": "0,8 6,0|0,0 6,8",
    "Y": "0,8 3,4 6,8|3,4 3,0",
    "Z": "0,8 6,8 0,0 6,0",
    "0": "2,0 4,0 6,2 6,6 4,8 2,8 0,6 0,2 2,0|1,1 5,7",
    "1": "1,6 3,8 3,0|1,0 5,0",
    "2": "0,6 2,8 4,8 6,6 6,5 0,0 6,0",
    "3": "0,7 2,8 4,8 6,6 4,4 2,4|4,4 6,2 4,0 2,0 0,1",
    "4": "5,0 5,8 0,3 6,3",
    "5": "6,8 0,8 0,4 4,4 6,2 4,0 2,0 0,1",
    "6": "6,7 4,8 2,8 0,5 0,2 2,0 4,0 6,2 6,3 4,4 0,4",
    "7": "0,8 6,8 2,0",
    "8": "2,4 0,6 2,8 4,8 6,6 4,4 2,4 0,2 2,0 4,0 6,2 4,4",
    "9": "6,4 2,4 0,6 2,8 4,8 6,6 6,3 4,0 2,0 0,1",
    "-": "0,4 6,4",
    "+": "0,4 6,4|3,1 3,7",
    ".": "3,0 3.2,0.2",
    ",": "3,1 2,-1",
    ":": "3,6 3.2,6.2|3,1 3.2,1.2",
    "/": "0,0 6,8",
    "_": "0,0 6,0",
    "=": "0,5 6,5|0,2 6,2",
    "!": "3,8 3,2|3,0 3.2,0.2",
    "?": "0,6 2,8 4,8 6,6 3,4 3,2|3,0 3.2,0.2",
    "(": "5,8 2,6 1,4 2,2 5,0",
    ")": "1,8 4,6 5,4 4,2 1,0",
    " ": "",
}

SUPPORTED_CHARACTERS = frozenset(_GLYPHS)


def strokes(text: str, x: float, y: float, height: float, spacing: float = 1.0,
            angle_deg: float = 0.0):
    """Return pen-down polylines in XY; rotate about text anchor (x,y).

    angle_deg is a GEOMETRIC XY angle (CCW), not robot TCP R.
    """
    value = str(text).upper()
    if not 1 <= len(value) <= 80 or any(c not in _GLYPHS for c in value):
        invalid = sorted(set(c for c in value if c not in _GLYPHS))
        raise ValueError(f"TEXT: nepodporovane znaky {invalid}, povoleno A-Z, 0-9, zakladni interpunkce")
    h = float(height)
    sp = float(spacing)
    if not 0.5 <= h <= 150 or not 0 <= sp <= 50:
        raise ValueError("TEXT: vyska 0.5..150 mm, mezera 0..50 mm")
    angle = float(angle_deg)
    if not math.isfinite(angle) or not -360.0 <= angle <= 360.0:
        raise ValueError("TEXT: natoceni musi byt -360..360 stupnu")
    c, s = math.cos(math.radians(angle)), math.sin(math.radians(angle))
    scale = h / 8.0
    advance = 6.0 * scale + sp
    result = []
    for index, char in enumerate(value):
        for raw_path in filter(None, _GLYPHS[char].split("|")):
            polyline = []
            for vertex in raw_path.split():
                gx, gy = (float(n) for n in vertex.split(","))
                dx = index * advance + gx * scale
                dy = gy * scale
                polyline.append((float(x) + c * dx - s * dy,
                                 float(y) + s * dx + c * dy))
            if len(polyline) >= 2:
                result.append(polyline)
    if not result:
        raise ValueError("TEXT: nejsou zadne tahy k gravirovani")
    return result
