import math
import re
import threading


class GCodeRunner:
    """
    G-code executor V3 pro RHO4 PCPATH.

    Hlavni zmena proti V2:
      - G1 se neprikazuje okamzite. Koncove body se skladaji do PCPATH bufferu.
      - G2/G3 se v Pythonu linearizuji na kratke LINEAR useky.
      - Az 16 bodu se posle jednim CMD20/CMD21.
      - RHO4 pak projede buffer jako jeden PROGR_SLOPE / VIA blok.

    Podpora:
      G90 / G91
      G17
      G0
      G1
      G2 / G3 pomoci I/J
      modalni G0/G1/G2/G3
      X Y Z R
      F [mm/min]
      M2 / M30
      ; komentar
      (komentar)

    V3 omezeni:
      - PCPATH V1 obsahuje pouze LINEAR body.
      - G2/G3 se proto linearizuji.
      - G2/G3 pouze XY + I/J.
      - I/J jsou offset od START i v G90.
      - Helix (zmena Z behem G2/G3) zatim ne.
      - R na G2/G3 je zakazano; u robota je R rotace.
      - Jeden controller buffer ma max 16 bodu.
        Mezi dvema buffery je zatim TO -> robot muze zastavit.
    """

    ARC_RADIUS_TOLERANCE_MM = 0.20
    ARC_EPS = 1e-9

    def __init__(
        self,
        rho,
        default_feed=600.0,
        rapid_feed=1200.0,
        move_timeout=120.0,
        path_capacity=16,
        arc_tolerance_mm=0.20,
        arc_max_angle_deg=30.0,
        verify_endpoint_mm=0.10,
    ):
        self.rho = rho

        self.default_feed = float(default_feed)
        self.rapid_feed = float(rapid_feed)
        self.move_timeout = float(move_timeout)

        self.path_capacity = int(path_capacity)
        if not 1 <= self.path_capacity <= 16:
            raise ValueError("path_capacity musi byt 1..16")

        self.arc_tolerance_mm = float(arc_tolerance_mm)
        if self.arc_tolerance_mm <= 0:
            raise ValueError("arc_tolerance_mm musi byt > 0")

        self.arc_max_angle_deg = float(arc_max_angle_deg)
        if not 0 < self.arc_max_angle_deg <= 180:
            raise ValueError("arc_max_angle_deg musi byt 0..180")

        self.verify_endpoint_mm = float(verify_endpoint_mm)

        self.absolute = True
        self.feed = self.default_feed
        self.motion_mode = None

        # Logical/planned position. It advances as points are appended.
        self.position = None

        self.path_buffer = []
        self.path_feed = None
        self.path_blocks_sent = 0
        self.path_points_sent = 0

        self._stop_event = threading.Event()
        self._pause_event = threading.Event()
        self._pause_event.set()

        self.running = False
        self.line_number = 0

    # =========================================================
    # BASIC STATE
    # =========================================================

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

        self.path_buffer.clear()
        self.path_feed = None
        self.path_blocks_sent = 0
        self.path_points_sent = 0

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
            except Exception as exc:
                print("STOP warning:", exc)

    def _wait_if_paused(self):
        while not self._pause_event.wait(0.1):
            if self._stop_event.is_set():
                return False
        return not self._stop_event.is_set()

    # =========================================================
    # PARSER
    # =========================================================

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

    # =========================================================
    # PCPATH BUFFER
    # =========================================================

    @staticmethod
    def _dict_to_tuple(point):
        return (
            float(point["X"]),
            float(point["Y"]),
            float(point["Z"]),
            float(point["R"]),
        )

    def _verify_actual_endpoint(self, expected):
        actual = self.rho.get_position()

        dxyz = math.sqrt(
            (actual[0] - expected[0])**2
            + (actual[1] - expected[1])**2
            + (actual[2] - expected[2])**2
        )
        dr = abs(
            (actual[3] - expected[3] + 180.0) % 360.0 - 180.0
        )

        if dxyz > self.verify_endpoint_mm or dr > 0.2:
            raise RuntimeError(
                "PCPATH endpoint nesedi: "
                f"dXYZ={dxyz:.3f} mm, dR={dr:.3f} deg"
            )

    def flush_path(self, reason="flush"):
        """
        Odesle aktualni buffer jako jeden PCPATH.
        Blokuje do konce controller bufferu.
        """
        if not self.path_buffer:
            return True

        if self._stop_event.is_set():
            return False

        if not self._wait_if_paused():
            return False

        points = list(self.path_buffer)
        path_feed = self.path_feed
        if path_feed is None:
            raise RuntimeError("PCPATH buffer is missing feed value")

        feed = float(path_feed)
        speed_mm_s = feed / 60.0
        expected = points[-1]

        self.path_blocks_sent += 1
        block_no = self.path_blocks_sent

        print(
            f"  -> PCPATH #{block_no}: "
            f"{len(points)} bodu, "
            f"F={feed:.1f} mm/min "
            f"({speed_mm_s:.3f} mm/s), "
            f"{reason}"
        )

        ok = self.rho.move_path(
            points,
            speed_mm_s=speed_mm_s,
            timeout=self.move_timeout,
            validate=True,
        )

        if not ok:
            raise RuntimeError("PCPATH byl zastaven")

        self._verify_actual_endpoint(expected)

        # At this moment logical endpoint == actual endpoint.
        self.sync_position()

        self.path_points_sent += len(points)
        self.path_buffer.clear()
        self.path_feed = None
        return True

    def _set_feed(self, new_feed):
        new_feed = float(new_feed)
        if new_feed <= 0:
            raise ValueError("F musi byt > 0 mm/min")

        # Existing points were planned with the old feed.
        if (
            self.path_buffer
            and self.path_feed is not None
            and abs(new_feed - self.path_feed) > 1e-9
        ):
            self.flush_path("zmena F")

        self.feed = new_feed

    def _append_path_point(self, point, feed_mm_min):
        feed_mm_min = float(feed_mm_min)

        if self.path_buffer:
            if self.path_feed is not None and abs(feed_mm_min - self.path_feed) > 1e-9:
                self.flush_path("zmena feedu")
        else:
            self.path_feed = feed_mm_min

        point_tuple = self._dict_to_tuple(point)
        self.path_buffer.append(point_tuple)

        # Planned position advances immediately.
        self.position = {
            "X": point_tuple[0],
            "Y": point_tuple[1],
            "Z": point_tuple[2],
            "R": point_tuple[3],
        }

        if len(self.path_buffer) >= self.path_capacity:
            self.flush_path("buffer full")

    # =========================================================
    # TARGET / LINEAR
    # =========================================================

    def _target_from_values(self, values):
        if self.position is None:
            self.sync_position()

        if self.position is None:
            raise RuntimeError("Unable to determine current position")

        target = self.position.copy()

        for axis in ("X", "Y", "Z", "R"):
            if axis not in values:
                continue

            if self.absolute:
                target[axis] = float(values[axis])
            else:
                target[axis] += float(values[axis])

        return target

    def _execute_g0(self, values):
        """
        Rapid stays a standalone CMD16 move.
        Before G0, any pending cutting path is flushed.
        """
        if self.path_buffer:
            self.flush_path("pred G0")

        target = self._target_from_values(values)

        if not any(axis in values for axis in ("X", "Y", "Z", "R")):
            return

        speed_mm_s = self.rapid_feed / 60.0

        print(
            f"[{self.line_number:04d}] G0 "
            f"X={target['X']:.3f} Y={target['Y']:.3f} "
            f"Z={target['Z']:.3f} R={target['R']:.3f} "
            f"F={self.rapid_feed:.1f} mm/min "
            f"({speed_mm_s:.3f} mm/s)"
        )

        ok = self.rho.move_linear(
            target["X"],
            target["Y"],
            target["Z"],
            target["R"],
            speed_mm_s=speed_mm_s,
            timeout=self.move_timeout,
        )

        if not ok:
            raise RuntimeError("G0 LINEAR byl zastaven")

        self.sync_position()

    def _execute_g1(self, values):
        target = self._target_from_values(values)

        if not any(axis in values for axis in ("X", "Y", "Z", "R")):
            return

        print(
            f"[{self.line_number:04d}] G1 queue "
            f"X={target['X']:.3f} Y={target['Y']:.3f} "
            f"Z={target['Z']:.3f} R={target['R']:.3f} "
            f"F={self.feed:.1f}"
        )

        self._append_path_point(target, self.feed)

    # =========================================================
    # ARC -> LINEARIZATION
    # =========================================================

    def _arc_geometry(self, values, motion_g):
        if self.position is None:
            self.sync_position()

        if motion_g not in (2, 3):
            raise ValueError("Interni chyba: motion_g musi byt G2 nebo G3")

        if "R" in values:
            raise ValueError(
                f"Radek {self.line_number}: u G2/G3 nepouzivej R. "
                "U robota R znamena rotaci; pro radius pouzij I/J."
            )

        if "I" not in values and "J" not in values:
            raise ValueError(
                f"Radek {self.line_number}: G{motion_g} vyzaduje I a/nebo J"
            )
        if self.position is None:
            raise RuntimeError("Unable to determine current position")
        sx = self.position["X"]
        sy = self.position["Y"]
        sz = self.position["Z"]
        sr = self.position["R"]

        if "Z" in values:
            target_z = (
                float(values["Z"])
                if self.absolute
                else sz + float(values["Z"])
            )
            if abs(target_z - sz) > 1e-6:
                raise ValueError(
                    f"Radek {self.line_number}: "
                    "G2/G3 se zmenou Z zatim nepodporujeme"
                )

        ex = sx
        ey = sy

        if "X" in values:
            ex = (
                float(values["X"])
                if self.absolute
                else sx + float(values["X"])
            )

        if "Y" in values:
            ey = (
                float(values["Y"])
                if self.absolute
                else sy + float(values["Y"])
            )

        # I/J are always relative to START.
        i = float(values.get("I", 0.0))
        j = float(values.get("J", 0.0))

        cx = sx + i
        cy = sy + j

        radius_start = math.hypot(sx - cx, sy - cy)
        radius_end = math.hypot(ex - cx, ey - cy)

        if radius_start < self.ARC_EPS:
            raise ValueError(
                f"Radek {self.line_number}: nulovy polomer G{motion_g}"
            )

        full_circle = (
            math.hypot(ex - sx, ey - sy)
            <= self.ARC_RADIUS_TOLERANCE_MM
        )

        if not full_circle:
            radius_error = abs(radius_start - radius_end)
            if radius_error > self.ARC_RADIUS_TOLERANCE_MM:
                raise ValueError(
                    f"Radek {self.line_number}: rozdil START/END radius "
                    f"{radius_start:.3f} vs {radius_end:.3f} mm"
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
                f"Radek {self.line_number}: nulovy oblouk"
            )

        return {
            "sx": sx,
            "sy": sy,
            "sz": sz,
            "sr": sr,
            "ex": ex,
            "ey": ey,
            "cx": cx,
            "cy": cy,
            "radius": radius_start,
            "a0": a0,
            "sweep": sweep,
            "full_circle": full_circle,
        }

    def _arc_segment_count(self, radius, sweep):
        """
        Chord error <= arc_tolerance_mm, plus absolute max angular step.
        """
        tol = self.arc_tolerance_mm
        max_angle = math.radians(self.arc_max_angle_deg)

        if tol >= radius:
            tolerance_angle = math.pi
        else:
            arg = 1.0 - tol / radius
            arg = max(-1.0, min(1.0, arg))
            tolerance_angle = 2.0 * math.acos(arg)

        step_angle = min(max_angle, tolerance_angle)

        if step_angle <= 1e-9:
            step_angle = max_angle

        return max(
            1,
            int(math.ceil(abs(sweep) / step_angle))
        )

    def _execute_arc(self, values, motion_g):
        geometry = self._arc_geometry(values, motion_g)

        radius = geometry["radius"]
        sweep = geometry["sweep"]
        segments = self._arc_segment_count(radius, sweep)

        direction = "CW" if motion_g == 2 else "CCW"

        print(
            f"[{self.line_number:04d}] G{motion_g} {direction} -> queue "
            f"END X={geometry['ex']:.3f} Y={geometry['ey']:.3f} "
            f"RADIUS={radius:.3f} "
            f"segments={segments} "
            f"tol={self.arc_tolerance_mm:.3f} mm "
            f"F={self.feed:.1f}"
        )

        for index in range(1, segments + 1):
            u = index / segments
            angle = geometry["a0"] + geometry["sweep"] * u

            # Last point is forced to exact G-code endpoint to avoid FP drift.
            if index == segments:
                x = geometry["ex"]
                y = geometry["ey"]
            else:
                x = geometry["cx"] + radius * math.cos(angle)
                y = geometry["cy"] + radius * math.sin(angle)

            point = {
                "X": x,
                "Y": y,
                "Z": geometry["sz"],
                "R": geometry["sr"],
            }

            self._append_path_point(point, self.feed)

    # =========================================================
    # LINE EXECUTION
    # =========================================================

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
                    f"Radek {self.line_number}: G{g} neni podporovano; "
                    "pouzivame G17"
                )

            else:
                raise ValueError(
                    f"Radek {self.line_number}: nepodporovany G{g}"
                )

        if explicit_motion is not None:
            self.motion_mode = explicit_motion

        # Feed change must close an old buffer before the new feed is applied.
        if "F" in values:
            self._set_feed(values["F"])

        # M2/M30 first flush all queued motion.
        for m in m_codes:
            if m in (2, 30):
                if self.path_buffer:
                    self.flush_path(f"M{m}")
                print(f"[{self.line_number:04d}] M{m} END")
                return False

            raise ValueError(
                f"Radek {self.line_number}: nepodporovany M{m}"
            )

        has_axis = any(
            axis in values for axis in ("X", "Y", "Z", "R")
        )
        has_arc_center = "I" in values or "J" in values
        should_move = has_axis or has_arc_center

        if should_move:
            if self.motion_mode is None:
                raise ValueError(
                    f"Radek {self.line_number}: souradnice bez modalniho "
                    "G0/G1/G2/G3"
                )

            if self.motion_mode in (0, 1) and has_arc_center:
                raise ValueError(
                    f"Radek {self.line_number}: I/J pouze pro G2/G3"
                )

            if self.motion_mode == 0:
                self._execute_g0(values)

            elif self.motion_mode == 1:
                self._execute_g1(values)

            elif self.motion_mode in (2, 3):
                self._execute_arc(values, self.motion_mode)

        elif "F" in values:
            print(
                f"[{self.line_number:04d}] "
                f"F={self.feed:.1f} mm/min"
            )

        return True

    # =========================================================
    # JOB
    # =========================================================

    def run_lines(self, lines):
        if self.running:
            raise RuntimeError("G-code uz bezi")

        self.running = True
        self._stop_event.clear()
        self._pause_event.set()

        self.path_buffer.clear()
        self.path_feed = None
        self.path_blocks_sent = 0
        self.path_points_sent = 0

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
                    print(
                        f"PCPATH summary: "
                        f"{self.path_blocks_sent} bloku, "
                        f"{self.path_points_sent} bodu"
                    )
                    return True

            # EOF without M2/M30.
            if self.path_buffer:
                self.flush_path("EOF")

            print(
                f"PCPATH summary: "
                f"{self.path_blocks_sent} bloku, "
                f"{self.path_points_sent} bodu"
            )
            return True

        finally:
            self.running = False

    def run_file(self, filename):
        with open(filename, "r", encoding="utf-8") as file:
            return self.run_lines(file)


if __name__ == "__main__":
    from rho4 import Rho4

    rho = Rho4("127.0.0.1", 6051)
    rho.connect()

    try:
        print("PING:", rho.ping())
        print("START:", rho.get_position())

        runner = GCodeRunner(
            rho,
            default_feed=300.0,
            rapid_feed=600.0,
            arc_tolerance_mm=0.20,
        )

        runner.run_file("pcpath_demo.gcode")

        print("END:", rho.get_position())

    finally:
        rho.disconnect()
