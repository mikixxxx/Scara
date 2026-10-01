import statistics
import time

from rho4 import Rho4


HOST = "192.168.4.1"   # real robot tomorrow
PORT = 6051
SAMPLES = 10


def main():
    rho = Rho4(HOST, PORT, timeout=3.0)
    rho.connect()

    try:
        print("PING:", rho.ping())
        print("STATUS:", rho.get_status())
        print("MOVE:", rho.get_move_state())
        print("POS:", rho.get_position())
        print("JOINT:", rho.get_joint_position())

        state, error, proc = rho.get_move_state()
        if state == rho.MOVE_RUNNING:
            raise RuntimeError("Robot se hybe - benchmark konci")

        rho.clear_stream()
        print("STREAM after clear:", rho.get_stream_status())

        p = rho.get_position()
        points = [tuple(p)] * 16

        ping_ms = []
        status_ms = []
        upload_ms = []

        for i in range(SAMPLES):
            t0 = time.perf_counter()
            rho.ping()
            ping_ms.append((time.perf_counter() - t0) * 1000.0)

            t0 = time.perf_counter()
            rho.get_stream_status()
            status_ms.append((time.perf_counter() - t0) * 1000.0)

            # Upload only. NO CMD23 is sent, therefore this test commands
            # no motion. CMD25 frees the slot again after every sample.
            t0 = time.perf_counter()
            rho.upload_stream_slot(
                0, 0, points, 2.0,
                validate_points=True,
            )
            upload_ms.append((time.perf_counter() - t0) * 1000.0)

            st = rho.get_stream_status()
            if st["a_free"] or st["a_seq"] != 0:
                raise RuntimeError(f"Unexpected A state after upload: {st}")

            rho.clear_stream()

        def show(name, values):
            print(
                f"{name:12s} "
                f"min={min(values):7.2f} ms  "
                f"avg={statistics.mean(values):7.2f} ms  "
                f"max={max(values):7.2f} ms"
            )

        print("\nNO-MOTION LINK BENCHMARK")
        show("PING", ping_ms)
        show("CMD24", status_ms)
        show("CMD22 268B", upload_ms)
        print("\nNo CMD23 was sent. Robot motion was not requested.")

    finally:
        rho.disconnect()


if __name__ == "__main__":
    main()
