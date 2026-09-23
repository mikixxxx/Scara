import argparse
import math
import threading
import time
from typing import TypedDict

from rho4 import Rho4

class MoveResult(TypedDict):
    value: bool | None
    error: Exception | None

HOST = "127.0.0.1"
PORT = 6051

START = (300.0, 300.0, 180.0, 0.0)
TOP = (320.0, 320.0, 180.0, 0.0)
RIGHT = (340.0, 300.0, 180.0, 0.0)
BOTTOM = (320.0, 280.0, 180.0, 0.0)


def close_enough(a, b, tol=0.05):
    return all(abs(x - y) <= tol for x, y in zip(a, b))


def print_pos(label, p):
    print(
        f"{label}: X={p[0]:.3f} Y={p[1]:.3f} "
        f"Z={p[2]:.3f} R={p[3]:.3f}"
    )


def ensure_start(rho):
    p = rho.get_position()
    if close_enough(p, START):
        return

    print("Presun do testovaciho START bodu...")
    ok = rho.move_linear(*START, speed_mm_s=50.0, timeout=20.0)
    if not ok:
        raise RuntimeError("Presun do START byl zastaven")

    p = rho.get_position()
    if not close_enough(p, START):
        raise RuntimeError(f"Nepodarilo se dosahnout START: {p}")


def test_half_circle(rho):
    print("\n=== TEST 1: PULKRUH ===")
    ensure_start(rho)

    path = rho.check_circular_path(*TOP, *RIGHT, step_mm=2.0)
    print(f"Checker: {len(path)} vzorku")

    t0 = time.monotonic()
    ok = rho.move_circular(
        *TOP,
        *RIGHT,
        speed_mm_s=20.0,
        timeout=10.0,
    )
    elapsed = time.monotonic() - t0

    if not ok:
        raise RuntimeError("Pulkruh byl zastaven")

    end = rho.get_position()
    print_pos("END", end)
    print(f"Cas: {elapsed:.3f} s")

    if not close_enough(end, RIGHT):
        raise RuntimeError(f"Spatny koncovy bod pulkruhu: {end}")

    expected = math.pi * 20.0 / 20.0
    print(f"Teoreticky cas simulatoru ~ {expected:.3f} s")
    print("TEST 1: OK")


def test_full_circle(rho):
    print("\n=== TEST 2: CELA KRUZNICE = 2x CIRCULAR ===")
    ensure_start(rho)

    # Horni pulkruh: START -> TOP -> RIGHT
    ok = rho.move_circular(
        *TOP,
        *RIGHT,
        speed_mm_s=30.0,
        timeout=10.0,
    )
    if not ok:
        raise RuntimeError("Prvni pulkruh byl zastaven")

    p1 = rho.get_position()
    print_pos("Po 1. pulkruhu", p1)
    if not close_enough(p1, RIGHT):
        raise RuntimeError(f"Spatny bod po 1. pulkruhu: {p1}")

    # Dolni pulkruh: RIGHT -> BOTTOM -> START
    ok = rho.move_circular(
        *BOTTOM,
        *START,
        speed_mm_s=30.0,
        timeout=10.0,
    )
    if not ok:
        raise RuntimeError("Druhy pulkruh byl zastaven")

    end = rho.get_position()
    print_pos("END", end)

    if not close_enough(end, START):
        raise RuntimeError(f"Kruznice se nevratila do START: {end}")

    print("TEST 2: OK")


def test_stop(rho):
    print("\n=== TEST 3: STOP UPROSTRED CIRCULAR ===")
    ensure_start(rho)

    result: MoveResult = {"value":None, "error":None,}

    #result = {"value": None, "error": None}

    def mover():
        try:
            result["value"] = rho.move_circular(
                *TOP,
                *RIGHT,
                speed_mm_s=5.0,
                timeout=30.0,
            )
        except Exception as exc:
            result["error"] = exc

    th = threading.Thread(target=mover, daemon=True)
    th.start()

    # Pockame, az se pohyb skutecne rozbehne.
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        state, error, proc = rho.get_move_state()
        if state == rho.MOVE_RUNNING:
            break
        time.sleep(0.02)
    else:
        raise RuntimeError("CIRCULAR se nerozbehl")

    time.sleep(1.0)
    before_stop = rho.get_position()
    print_pos("Pred STOP", before_stop)

    rho.stop()
    th.join(timeout=5.0)

    if th.is_alive():
        raise RuntimeError("move_circular po STOP neukoncil cekani")

    if result["error"] is not None:
        raise result["error"]

    if result["value"] is not False:
        raise RuntimeError(
            f"Po STOP se cekal navrat False, prislo: {result['value']!r}"
        )

    state, error, proc = rho.get_move_state()
    after_stop = rho.get_position()

    print_pos("Po STOP", after_stop)
    print(f"State={state}, Error={error}, ProcStatus={proc}")

    if state != rho.MOVE_STOPPED:
        raise RuntimeError(f"Po STOP se cekal MOVE_STOPPED, je {state}")

    if close_enough(after_stop, RIGHT, tol=0.2):
        raise RuntimeError("STOP prisel az po dosazeni END bodu")

    print("TEST 3: OK")


def main():
    parser = argparse.ArgumentParser(
        description="CMD18 CIRCULAR test pro RHO4 simulator"
    )
    parser.add_argument(
        "test",
        nargs="?",
        choices=("half", "full", "stop", "all"),
        default="all",
    )
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    args = parser.parse_args()

    rho = Rho4(host=args.host, port=args.port, timeout=3.0)
    rho.connect()

    try:
        print("PING:", rho.ping())
        print_pos("START POSITION", rho.get_position())

        if args.test in ("half", "all"):
            test_half_circle(rho)

        if args.test in ("full", "all"):
            test_full_circle(rho)

        if args.test in ("stop", "all"):
            test_stop(rho)

        print("\n=== VSECHNY VYBRANE TESTY PROSLY ===")

    finally:
        rho.disconnect()


if __name__ == "__main__":
    main()
