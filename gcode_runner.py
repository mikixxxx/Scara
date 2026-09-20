import re
from rho4 import Rho4




class GCodeRunner:
    def __init__(self, robot):
        self.robot = robot
        self.absolute_mode = True
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.r = 0.0
        self.feed_mm_min = 600.0

    def set_current_position_from_robot(self):
        self.x,self.y,self.z,self.r = self.robot.get_position()

    def parse_words(self, line):
        line = line.split(";",1)[0]
        line = re.sub(r"\([^)]*\)", "", line)
        line.upper().strip()
        if not line:
            return
        words = {}

        for letter, vallue in re.findall(r"([A-Z])\s*([-+]?\d*\.?\d+)",line):
            words[letter] = float(vallue)
        return words

    def target_from_words(self, words):
        x = self.x
        y = self.y
        z = self.z
        r = self.r

        if self.absolute_mode:
            if "X" in words:
                x = words["X"]

            if "Y" in words:
                y = words["Y"]

            if "Z" in words:
                z = words["Z"]

            if "R" in words:
                r = words["R"]
        else:
            if "X" in words:
                x += words["X"]

            if "Y" in words:
                y += words["Y"]

            if "Z" in words:
                z += words["Z"]

            if "R" in words:
                r += words["R"]

        return x,y,z,r

    def execute_line(self, line):
        words = self.parse_words(line)
        if not words:
            return
        # feed rate
        if "F" in words:
            self.feed_mm_min = words["F"]

        g = int(words["G"]) if "G" in words else None
        m = int(words["M"]) if "M" in words else None

        # G90 abs souradnice
        if g == 90:
            self.absolute_mode = True
            print("G90 -> ABSOLUTE")
            return

        # G91 rel souradnice
        if g == 91:
            self.absolute_mode = False
            print("G91 -> RELATIVE")
            return

        # G0 rychly presun
        if g == 0:
            x, y, z, r = self.target_from_words(words)
            print(f"G0 -> " f"X={x:.3f} Y={y:.3f} " f"Z={z:.3f} R={r:.3f}")
            self.robot.move_ptp(x,y,z,r,speed=5,timeout=30.0)
            self.x = x
            self.y = y
            self.z = z
            self.r = r

            return

        # G1 linearni pohyb
        if g == 1:
            x, y, z, r = self.target_from_words(words)
            speed_mm_s = self.feed_mm_min / 60.0
            print(f"G1 -> " f"X={x:.3f} Y={y:.3f} " f"Z={z:.3f} R={r:.3f} " f"F={self.feed_mm_min:.1f} " f"({speed_mm_s:.2f} mm/s)")

            self.robot.move_linear(
                x,
                y,
                z,
                r,
                speed_mm_s=speed_mm_s,
                timeout=30.0
            )

            self.x = x
            self.y = y
            self.z = z
            self.r = r

            return

        # M kody jen vypiseme
        if m == 3:
            print("M3 -> tool ON")
            return
        if m == 5:
            print("M5 -> tool OFF")
            return
        if m in (2,30):
            print("PROGRAM END")
            return
        print("IGNORED: ",line.strip())


    def run_file(self, filename):
        self.set_current_position_from_robot()
        print(
            f"Start position: "
            f"X={self.x:.3f} "
            f"Y={self.y:.3f} "
            f"Z={self.z:.3f} "
            f"R={self.r:.3f}"
        )

        with open(filename, "r", encoding="utf-8") as f:
            for line_number, line in enumerate(f, 1):
                line = line.strip()
                if not line:
                    continue
                print(f"{line_number:04d}: {line}")
                try:
                    self.execute_line(line)
                except Exception as e:
                    raise RuntimeError(f"G-code chyba na radku "f"{line_number}: {line}\n{e}") from e
        

def main():
    rho = Rho4()
    rho.connect()

    try:
        runner = GCodeRunner(rho)
        runner.run_file("test.gcode")
    finally:
        rho.disconnect


if __name__ == "__main__":
    main()