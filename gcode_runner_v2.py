import math
import re
import threading


class GCodeRunner:
    """
    G-code executor pro Rho4.

    V2:
      G90 / G91
      G0 / G1
      G2 / G3 v rovine XY pomoci I/J
      modalni G0/G1/G2/G3
      X Y Z R
      F [mm/min]
      M2 / M30
      ; komentar
      (komentar)

    G0/G1 pouzivaji RHO4 LINEAR (CMD16).
    G2/G3 pouzivaji RHO4 CIRCULAR (CMD18).

    Omezeni V2:
      - G2/G3 podporuji pouze I/J format (stred jako offset od START).
      - I/J jsou vzdy relativni ke START, nezavisle na G90/G91.
      - G2/G3 jsou zatim jen v XY rovine.
      - Z musi zustat pri G2/G3 konstantni.
      - R na radku G2/G3 je zakazano, aby se nepletlo
        s beznym CNC vyznamem R=radius.
      - G17 je akceptovano jako XY plane.
      - G18/G19 zatim nejsou podporovany.
    """

    ARC_RADIUS_TOLERANCE_MM = 0.20
    ARC_EPS = 1e-9

    def __init__(
        self,
        rho,
        default_feed=600.0,
        rapid_feed=1200.0,
        move_timeout=120.0,
    ):
        self.rho = rho
        self.default_feed = float(default_feed)
        self.rapid_feed = float(rapid_feed)
        self.move_timeout = float(move_timeout)

        self.absolute = True
        self.feed = self.default_feed
        self.motion_mode = None
        self.position = None

        self._stop_event = threading.Event()
        self._pause_event = threading.Event()
        self._pause_event.set()

        self.running = False
        self.line_number = 0

    def sync_position(self):
        x, y, z, r = self.rho.get_position()
        self.position = {
            "X": float(x),
            "Y": float(y),
            "Z": float(z),
            "R": float(r),
        }
        return self.position.copy()

    def reset(self):
        self.absolute = True
        self.feed = self.default_feed
        self.motion_mode = None
        self._stop_event.clear()
        self._pause_event.set()
        self.line_number = 0
        self.sync_position()

    def pause(self):
        self._pause_event.clear()

    def resume(self):
        self._pause_event.set()

    def stop(self):
        self._stop_event.set()
        self._pause_event.set()
        if self.rho.is_connected():
            try:
                self.rho.stop()
            except Exception as e:
                print("STOP warning:", e)

    @staticmethod
    def _strip_comments(line):
        line = re.sub(r"\([^)]*\)", "", line)
        line = line.split(";", 1)[0]
        return line.strip().upper()

    @staticmethod
    def _words(line):
        return [
            (letter, float(value))
            for letter, value in re.findall(
                r"([A-Z])\s*([-+]?(?:\d+(?:\.\d*)?|\.\d+))",
                line,
            )
        ]

    @staticmethod
    def _values(words):
        values = {}
        for letter, value in words:
            values[letter] = value
        return values

    def _wait_if_paused(self):
        while not self._pause_event.wait(0.1):
            if self._stop_event.is_set():
                return False
        return not self._stop_event.is_set()

    def _update_feed(self, values):
        if "F" not in values:
            return
        if values["F"] <= 0:
            raise ValueError("F musi byt > 0 mm/min")
        self.feed = float(values["F"])

    def _target_from_values(self, values):
        if self.position is None:
            self.sync_position()

        assert self.position is not None
        target = self.position.copy()

        for axis in ("X", "Y", "Z", "R"):
            if axis not in values:
                continue

            if self.absolute:
                target[axis] = float(values[axis])
            else:
                target[axis] += float(values[axis])

        return target

    def _execute_linear(self, words, motion_g):
        if self.position is None:
            self.sync_position()

        values = self._values(words)
        self._update_feed(values)
        target = self._target_from_values(values)

        has_axis = any(axis in values for axis in ("X", "Y", "Z", "R"))
        if not has_axis:
            return

        feed_mm_min = self.rapid_feed if motion_g == 0 else self.feed
        speed_mm_s = feed_mm_min / 60.0

        print(
            f"[{self.line_number:04d}] G{motion_g} "
            f"X={target['X']:.3f} Y={target['Y']:.3f} "
            f"Z={target['Z']:.3f} R={target['R']:.3f} "
            f"F={feed_mm_min:.1f} mm/min ({speed_mm_s:.3f} mm/s)"
        )

        ok = self.rho.move_linear(
            target["X"], target["Y"], target["Z"], target["R"],
            speed_mm_s=speed_mm_s,
            timeout=self.move_timeout,
        )

        if not ok:
            raise RuntimeError("LINEAR pohyb byl zastaven")

        self.sync_position()

    def _arc_geometry(self, values, motion_g):
        if self.position is None:
            self.sync_position()

        assert self.position is not None

        if motion_g not in (2, 3):
            raise ValueError("Interni chyba: motion_g musi byt G2 nebo G3")

        if "R" in values:
            raise ValueError(
                f"Radek {self.line_number}: u G2/G3 zatim nepouzivej R. "
                "V beznem CNC G-code znamena R polomer, ale u naseho robota "
                "R znamena rotaci. Pro oblouk pouzij I/J."
            )

        if "I" not in values and "J" not in values:
            raise ValueError(
                f"Radek {self.line_number}: G{motion_g} vyzaduje I a/nebo J"
            )

        sx = self.position["X"]
        sy = self.position["Y"]
        sz = self.position["Z"]
        sr = self.position["R"]

        if "Z" in values:
            target_z = float(values["Z"]) if self.absolute else sz + float(values["Z"])
            if abs(target_z - sz) > 1e-6:
                raise ValueError(
                    f"Radek {self.line_number}: G2/G3 se zmenou Z zatim "
                    "nepodporujeme (helix)."
                )

        ex = sx
        ey = sy

        if "X" in values:
            ex = float(values["X"]) if self.absolute else sx + float(values["X"])
        if "Y" in values:
            ey = float(values["Y"]) if self.absolute else sy + float(values["Y"])

        i = float(values.get("I", 0.0))
        j = float(values.get("J", 0.0))

        cx = sx + i
        cy = sy + j

        rs = math.hypot(sx - cx, sy - cy)
        re = math.hypot(ex - cx, ey - cy)

        if rs < self.ARC_EPS:
            raise ValueError(
                f"Radek {self.line_number}: nulovy polomer G{motion_g}"
            )

        full_circle = (
            math.hypot(ex - sx, ey - sy) <= self.ARC_RADIUS_TOLERANCE_MM
        )

        if not full_circle:
            radius_error = abs(rs - re)
            if radius_error > self.ARC_RADIUS_TOLERANCE_MM:
                raise ValueError(
                    f"Radek {self.line_number}: G{motion_g} ma rozdilny "
                    f"START/END radius: {rs:.3f} vs {re:.3f} mm "
                    f"(rozdil {radius_error:.3f} mm)"
                )

        a0 = math.atan2(sy - cy, sx - cx)
        a1 = math.atan2(ey - cy, ex - cx)

        if full_circle:
            sweep = -2.0 * math.pi if motion_g == 2 else 2.0 * math.pi
        elif motion_g == 3:
            sweep = (a1 - a0) % (2.0 * math.pi)
        else:
            sweep = -((a0 - a1) % (2.0 * math.pi))

        if abs(sweep) < self.ARC_EPS:
            raise ValueError(
                f"Radek {self.line_number}: G{motion_g} ma nulovy oblouk"
            )

        amid = a0 + 0.5 * sweep
        mid = {
            "X": cx + rs * math.cos(amid),
            "Y": cy + rs * math.sin(amid),
            "Z": sz,
            "R": sr,
        }
        end = {
            "X": ex,
            "Y": ey,
            "Z": sz,
            "R": sr,
        }

        return {
            "CX": cx,
            "CY": cy,
            "radius": rs,
            "sweep": sweep,
            "mid": mid,
            "end": end,
            "full_circle": full_circle,
            "a0": a0,
        }

    def _run_native_arc(self, mid, end, speed_mm_s):
        ok = self.rho.move_circular(
            mid["X"], mid["Y"], mid["Z"], mid["R"],
            end["X"], end["Y"], end["Z"], end["R"],
            speed_mm_s=speed_mm_s,
            timeout=self.move_timeout,
        )

        if not ok:
            raise RuntimeError("CIRCULAR pohyb byl zastaven")

        self.sync_position()

    def _execute_arc(self, words, motion_g):
        if self.position is None:
            self.sync_position()

        values = self._values(words)
        self._update_feed(values)

        geometry = self._arc_geometry(values, motion_g)
        speed_mm_s = self.feed / 60.0
        mid = geometry["mid"]
        end = geometry["end"]

        direction = "CW" if motion_g == 2 else "CCW"

        print(
            f"[{self.line_number:04d}] G{motion_g} {direction} "
            f"END X={end['X']:.3f} Y={end['Y']:.3f} "
            f"Z={end['Z']:.3f} R={end['R']:.3f} "
            f"C=({geometry['CX']:.3f},{geometry['CY']:.3f}) "
            f"RAD={geometry['radius']:.3f} "
            f"F={self.feed:.1f} mm/min ({speed_mm_s:.3f} mm/s)"
        )

        if not geometry["full_circle"]:
            self._run_native_arc(mid, end, speed_mm_s)
            return

        assert self.position is not None

        cx = geometry["CX"]
        cy = geometry["CY"]
        radius = geometry["radius"]
        a0 = geometry["a0"]
        sweep = geometry["sweep"]

        a_quarter = a0 + 0.25 * sweep
        a_half = a0 + 0.50 * sweep
        a_three_quarter = a0 + 0.75 * sweep

        z = self.position["Z"]
        r = self.position["R"]

        mid1 = {
            "X": cx + radius * math.cos(a_quarter),
            "Y": cy + radius * math.sin(a_quarter),
            "Z": z,
            "R": r,
        }
        end1 = {
            "X": cx + radius * math.cos(a_half),
            "Y": cy + radius * math.sin(a_half),
            "Z": z,
            "R": r,
        }

        self._run_native_arc(mid1, end1, speed_mm_s)

        mid2 = {
            "X": cx + radius * math.cos(a_three_quarter),
            "Y": cy + radius * math.sin(a_three_quarter),
            "Z": z,
            "R": r,
        }
        end2 = {
            "X": end["X"],
            "Y": end["Y"],
            "Z": end["Z"],
            "R": end["R"],
        }

        self._run_native_arc(mid2, end2, speed_mm_s)

    def execute_line(self, raw_line):
        line = self._strip_comments(raw_line)
        if not line:
            return True

        words = self._words(line)
        if not words:
            return True

        values = self._values(words)
        g_codes = [int(v) for l, v in words if l == "G"]
        m_codes = [int(v) for l, v in words if l == "M"]

        explicit_motion = None

        for g in g_codes:
            if g == 90:
                self.absolute = True
                print(f"[{self.line_number:04d}] G90 ABSOLUTE")

            elif g == 91:
                self.absolute = False
                print(f"[{self.line_number:04d}] G91 RELATIVE")

            elif g == 17:
                print(f"[{self.line_number:04d}] G17 XY PLANE")

            elif g in (0, 1, 2, 3):
                if explicit_motion is not None and explicit_motion != g:
                    raise ValueError(
                        f"Radek {self.line_number}: vice pohybovych G kodu"
                    )
                explicit_motion = g

            elif g in (18, 19):
                raise ValueError(
                    f"Radek {self.line_number}: G{g} zatim neni podporovano; "
                    "pouzivame XY rovinu G17"
                )

            else:
                raise ValueError(
                    f"Radek {self.line_number}: nepodporovany G{g}"
                )

        for m in m_codes:
            if m in (2, 30):
                print(f"[{self.line_number:04d}] M{m} END")
                return False
            raise ValueError(
                f"Radek {self.line_number}: nepodporovany M{m}"
            )

        if explicit_motion is not None:
            self.motion_mode = explicit_motion

        self._update_feed(values)

        has_linear_axis = any(axis in values for axis in ("X", "Y", "Z", "R"))
        has_arc_center = "I" in values or "J" in values
        should_move = has_linear_axis or has_arc_center

        if should_move:
            if self.motion_mode is None:
                raise ValueError(
                    f"Radek {self.line_number}: souradnice bez aktivniho "
                    "modalniho pohybu G0/G1/G2/G3"
                )

            if self.motion_mode in (0, 1):
                if has_arc_center:
                    raise ValueError(
                        f"Radek {self.line_number}: I/J lze pouzit pouze s G2/G3"
                    )
                self._execute_linear(words, self.motion_mode)

            elif self.motion_mode in (2, 3):
                self._execute_arc(words, self.motion_mode)

        elif "F" in values:
            print(f"[{self.line_number:04d}] F={self.feed:.1f} mm/min")

        return True

    def run_lines(self, lines):
        if self.running:
            raise RuntimeError("G-code uz bezi")

        self.running = True
        self._stop_event.clear()
        self._pause_event.set()
        self.sync_position()

        try:
            for number, raw_line in enumerate(lines, start=1):
                self.line_number = number

                if self._stop_event.is_set():
                    print("G-code STOP")
                    return False

                if not self._wait_if_paused():
                    print("G-code STOP")
                    return False

                if not self.execute_line(raw_line):
                    return True

            return True

        finally:
            self.running = False

    def run_file(self, filename):
        with open(filename, "r", encoding="utf-8") as f:
            return self.run_lines(f)


if __name__ == "__main__":
    from rho4 import Rho4

    rho = Rho4("192.168.4.1", 6051)
    rho.connect()

    try:
        print("PING:", rho.ping())
        print("START:", rho.get_position())

        runner = GCodeRunner(
            rho,
            default_feed=600.0,
            rapid_feed=1200.0,
        )

        runner.run_file("test.gcode")

        print("END:", rho.get_position())

    finally:
        rho.disconnect()
