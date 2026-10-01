
    
  
from rho4 import Rho4
from gcode_runner_v2 import GCodeRunner

HOST = "192.168.4.1"
PORT = 6051

RADIUS = 10.0


def main():
    rho = Rho4(HOST, PORT, timeout=3.0)
    rho.connect()

    try:
        print("PING:", rho.ping())
        print("STATUS:", rho.get_status())
        print("MOVE:", rho.get_move_state())

        x, y, z, r = rho.get_position()

        print(
            f"\nSTART: X={x:.3f} Y={y:.3f} "
            f"Z={z:.3f} R={r:.3f}"
        )
        runner = GCodeRunner(
            rho,
            default_feed=120.0,
            rapid_feed=120.0,
            move_timeout=60.0,
        )

        ok = runner.run_file("test.gcode")
        print("\nG-code result:", ok)

        fx, fy, fz, fr = rho.get_position()

        print(
            f"END:   X={fx:.3f} Y={fy:.3f} "
            f"Z={fz:.3f} R={fr:.3f}"
        )

    finally:
        rho.disconnect()


if __name__ == "__main__":
    main()

