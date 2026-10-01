import argparse
import math

from rho4 import Rho4


DEFAULT_HOST = "localhost"   # real robot tomorrow
PORT = 6051

STAGES = {
    # Stage 1: no dynamic refill. Tests A -> B continuity only.
    1: dict(points=32, diameter=20.0, speed=2.0),

    # Stage 2: one real refill A <- seq2 while B is running.
    2: dict(points=48, diameter=30.0, speed=2.0),

    # Stage 3: full six-buffer ping-pong.
    3: dict(points=96, diameter=50.0, speed=2.0),
}


def make_circle(start, point_count, diameter):
    sx, sy, sz, sr = map(float, start)
    radius = diameter * 0.5
    cx = sx + radius
    cy = sy

    points = []
    for i in range(1, point_count + 1):
        a = math.pi - 2.0 * math.pi * i / point_count
        points.append((
            cx + radius * math.cos(a),
            cy + radius * math.sin(a),
            sz,
            sr,
        ))

    points[-1] = (sx, sy, sz, sr)
    return (cx, cy), points


def split16(points, speed):
    if len(points) % 16:
        raise ValueError("Point count must be a multiple of 16")

    return [
        {
            "points": points[i:i + 16],
            "speed_mm_s": speed,
        }
        for i in range(0, len(points), 16)
    ]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--stage", type=int, choices=(1, 2, 3), default=1)
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--diameter", type=float)
    parser.add_argument("--speed", type=float)
    parser.add_argument(
        "--run",
        action="store_true",
        help="Actually send CMD23 and move. Without this flag: DRY RUN only.",
    )
    args = parser.parse_args()

    cfg = dict(STAGES[args.stage])
    if args.diameter is not None:
        cfg["diameter"] = args.diameter
    if args.speed is not None:
        cfg["speed"] = args.speed

    rho = Rho4(args.host, PORT, timeout=3.0)
    rho.connect()

    try:
        print("PING:", rho.ping())
        print("STATUS:", rho.get_status())
        print("MOVE:", rho.get_move_state())

        state, error, proc = rho.get_move_state()
        if state == rho.MOVE_RUNNING:
            raise RuntimeError("Robot uz provadi pohyb")
        if state == rho.MOVE_ERROR:
            raise RuntimeError(
                f"Robot je v MOVE_ERROR={error}, proc={proc}"
            )
        if state in (rho.MOVE_DONE, rho.MOVE_STOPPED):
            rho.reset_move_state()

        start = rho.get_position()
        center, points = make_circle(
            start,
            cfg["points"],
            cfg["diameter"],
        )
        blocks = split16(points, cfg["speed"])

        print("\nPHYSICAL STREAM TEST")
        print(f"stage       : {args.stage}")
        print(f"buffers     : {len(blocks)} x 16")
        print(f"points      : {len(points)}")
        print(f"diameter    : {cfg['diameter']:.1f} mm")
        print(f"speed       : {cfg['speed']:.2f} mm/s")
        print(
            f"start       : X={start[0]:.3f} Y={start[1]:.3f} "
            f"Z={start[2]:.3f} R={start[3]:.3f}"
        )
        print(
            f"center      : X={center[0]:.3f} Y={center[1]:.3f}"
        )
        print(
            f"envelope X  : {start[0]:.3f} .. "
            f"{start[0] + cfg['diameter']:.3f}"
        )
        print(
            f"envelope Y  : "
            f"{start[1] - cfg['diameter']/2:.3f} .. "
            f"{start[1] + cfg['diameter']/2:.3f}"
        )

        report = rho.check_stream_path(blocks)
        print(
            "\nPREFLIGHT OK: "
            f"length={report['xyz_length_mm']:.2f} mm, "
            f"samples={report['samples']}"
        )

        windows = rho.estimate_refill_windows(blocks)
        if windows:
            print("Conservative refill windows:")
            for w in windows:
                print(
                    f"  while seq{w['intervening_seq']} runs -> "
                    f"prepare seq{w['refill_seq']}: "
                    f"~{w['window_s']:.3f} s"
                )
        else:
            print("No dynamic refill is needed in this stage.")

        if not args.run:
            print(
                "\nDRY RUN ONLY. Nothing was uploaded or started.\n"
                "To execute this exact test, add: --run"
            )
            return

        print(
            "\nRUN ENABLED. Keep E-stop accessible and free space around TCP."
        )

        ok = rho.stream_blocks(
            blocks,
            timeout=180.0,
            poll_interval=0.005,
            validate_points=False,   # already did one full preflight above
            full_preflight=False,
            min_refill_window_s=0.50,
            clear_before_start=True,
        )

        end = rho.get_position()
        print("\nstream_blocks() ->", ok)
        print(
            f"END: X={end[0]:.3f} Y={end[1]:.3f} "
            f"Z={end[2]:.3f} R={end[3]:.3f}"
        )
        print(
            "DELTA: "
            f"dX={end[0]-start[0]:+.3f} "
            f"dY={end[1]-start[1]:+.3f} "
            f"dZ={end[2]-start[2]:+.3f} "
            f"dR={end[3]-start[3]:+.3f}"
        )

    except KeyboardInterrupt:
        print("\nKeyboardInterrupt -> controlled CMD15 STOP")
        try:
            rho.stop()
        except Exception as stop_exc:
            print("STOP error:", stop_exc)
        raise

    finally:
        rho.disconnect()


if __name__ == "__main__":
    main()
