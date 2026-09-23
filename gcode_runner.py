import re
import threading
import time


class GCodeRunner:
    """
    Jednoduchy G-code executor pro Rho4.

    V1:
      G90 / G91
      G0 / G1
      X Y Z R
      F [mm/min]
      M2 / M30
      ; komentar
      (komentar)

    G0 i G1 pouzivaji RHO4 LINEAR (CMD16).
    """

    def __init__(self, rho, default_feed=600.0, rapid_feed=1200.0, move_timeout=120.0):
        self.rho = rho

        self.default_feed = float(default_feed)   # mm/min
        self.rapid_feed = float(rapid_feed)       # mm/min
        self.move_timeout = float(move_timeout)

        self.absolute = True
        self.feed = self.default_feed

        self.position = None

        self._stop_event = threading.Event()
        self._pause_event = threading.Event()
        self._pause_event.set()

        self.running = False
        self.line_number = 0

    def sync_position(self):
        """Nacte skutecnou XYZR pozici z RHO4/simulatoru."""
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
        self._stop_event.clear()
        self._pause_event.set()
        self.line_number = 0
        self.sync_position()

    def pause(self):
        """
        Pozastavi G-code mezi segmenty.
        Aktualne rozjety LINEAR pohyb dojede.
        """
        self._pause_event.clear()

    def resume(self):
        self._pause_event.set()

    def stop(self):
        """
        Zastavi executor a posle CMD15 do RHO4.
        """
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
        # napr. "G1 X300.5 Y-20 F600"
        return [
            (letter, float(value))
            for letter, value in re.findall(
                r"([A-Z])\s*([-+]?(?:\d+(?:\.\d*)?|\.\d+))",
                line
            )
        ]

    def _wait_if_paused(self):
        while not self._pause_event.wait(0.1):
            if self._stop_event.is_set():
                return False
        return not self._stop_event.is_set()

    def _execute_move(self, words, motion_g):
        if self.position is None:
            self.sync_position()

        values = {}
        for letter, value in words:
            if letter in ("X", "Y", "Z", "R", "F"):
                values[letter] = value

        if "F" in values:
            if values["F"] <= 0:
                raise ValueError("F musi byt > 0 mm/min")
            self.feed = values["F"]

        assert self.position is not None
        target = self.position.copy()

        for axis in ("X", "Y", "Z", "R"):
            if axis not in values:
                continue

            if self.absolute:
                target[axis] = values[axis]
            else:
                target[axis] += values[axis]

        # G0 ma vlastni travel feed, G1 pouziva modalni F.
        feed_mm_min = self.rapid_feed if motion_g == 0 else self.feed
        speed_mm_s = feed_mm_min / 60.0

        # Radek obsahujici pouze F nemusi vyvolat pohyb.
        has_axis = any(axis in values for axis in ("X", "Y", "Z", "R"))
        if not has_axis:
            return

        print(
            f"[{self.line_number:04d}] "
            f"G{motion_g} "
            f"X={target['X']:.3f} "
            f"Y={target['Y']:.3f} "
            f"Z={target['Z']:.3f} "
            f"R={target['R']:.3f} "
            f"F={feed_mm_min:.1f} mm/min "
            f"({speed_mm_s:.3f} mm/s)"
        )

        # move_linear -> start_linear -> check_linear_path -> CMD16
        ok = self.rho.move_linear(
            target["X"],
            target["Y"],
            target["Z"],
            target["R"],
            speed_mm_s=speed_mm_s,
            timeout=self.move_timeout,
        )

        if not ok:
            raise RuntimeError("LINEAR pohyb byl zastaven")

        # Po dojeti synchronizujeme skutecnou pozici.
        self.sync_position()

    def execute_line(self, raw_line):
        line = self._strip_comments(raw_line)
        if not line:
            return True

        words = self._words(line)
        if not words:
            return True

        g_codes = [int(v) for l, v in words if l == "G"]
        m_codes = [int(v) for l, v in words if l == "M"]

        for g in g_codes:
            if g == 90:
                self.absolute = True
                print(f"[{self.line_number:04d}] G90 ABSOLUTE")

            elif g == 91:
                self.absolute = False
                print(f"[{self.line_number:04d}] G91 RELATIVE")

            elif g not in (0, 1):
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

        motion = None
        if 0 in g_codes:
            motion = 0
        if 1 in g_codes:
            motion = 1

        if motion is not None:
            self._execute_move(words, motion)
        else:
            # Samostatne modalni F.
            for letter, value in words:
                if letter == "F":
                    if value <= 0:
                        raise ValueError("F musi byt > 0 mm/min")
                    self.feed = value
                    print(
                        f"[{self.line_number:04d}] "
                        f"F={self.feed:.1f} mm/min"
                    )

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

    rho = Rho4("127.0.0.1", 6051)
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
