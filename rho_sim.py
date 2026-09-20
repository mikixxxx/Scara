import socket
import struct
import threading
import time
import math

HOST = "127.0.0.1"
PORT = 6051

# Driver-compatible constants
MOVE_IDLE = 0
MOVE_RUNNING = 1
MOVE_DONE = 2
MOVE_ERROR = 3
MOVE_STOPPED = 4

START_OK = 10
START_MANUAL = -10
START_BUSY = -11
START_RC_ERROR = -12

# Simulator state
lock = threading.RLock()

position = [300.0, 300.0, 180.0, 0.0]
target = position.copy()
ptp_speed_factor = 0.02       # driver sends speed / 100

automatic = True
alarm = False
referenced = True
in_position = True
win_status = 0
protocol_version = 1

move_state = MOVE_IDLE
move_error = 0
procstatus = -1

stop_requested = False
shutdown_requested = False


def recv_exact(sock, size):
    data = b""
    while len(data) < size:
        chunk = sock.recv(size - len(data))
        if not chunk:
            raise ConnectionError("Client disconnected")
        data += chunk
    return data


def set_move_state(state, error=0, proc=-1):
    global move_state, move_error, procstatus
    with lock:
        move_state = state
        move_error = error
        procstatus = proc


def motion_worker():
    global position, in_position, stop_requested

    with lock:
        start = position.copy()
        dest = target.copy()
        speed_factor = max(0.0001, ptp_speed_factor)
        stop_requested = False
        in_position = False
        set_move_state(MOVE_RUNNING, 0, 1)

    # This is intentionally not a physical RHO4 velocity model.
    # It only provides deterministic motion in time for driver testing.
    max_delta = max(abs(dest[i] - start[i]) for i in range(4))
    velocity = 1000.0 * speed_factor   # mm/s or deg/s-ish
    duration = max(0.15, max_delta / max(velocity, 1.0))
    t0 = time.monotonic()

    while True:
        with lock:
            if stop_requested:
                in_position = False
                set_move_state(MOVE_STOPPED, 0, -1)
                return

        elapsed = time.monotonic() - t0
        u = min(1.0, elapsed / duration)

        # smoothstep interpolation
        s = u * u * (3.0 - 2.0 * u)

        with lock:
            position = [
                start[i] + (dest[i] - start[i]) * s
                for i in range(4)
            ]

        if u >= 1.0:
            break

        time.sleep(0.01)

    with lock:
        position = dest.copy()
        in_position = True

        # Real driver waits specifically for state=2, procstatus=-1.
        set_move_state(MOVE_DONE, 0, -1)


def start_motion():
    global stop_requested

    with lock:
        if not automatic:
            return START_MANUAL

        if move_state == MOVE_RUNNING:
            return START_BUSY

        if move_state == MOVE_ERROR:
            return START_RC_ERROR

        if alarm:
            # Simulator-specific RC-like error
            set_move_state(MOVE_ERROR, 144384, 144384)
            return START_RC_ERROR

        stop_requested = False

    threading.Thread(target=motion_worker, daemon=True).start()
    return START_OK


def handle_client(conn, addr):
    global target, ptp_speed_factor
    global move_state, move_error, procstatus
    global stop_requested

    print(f"[TCP] Client connected: {addr}")

    try:
        while True:
            raw = recv_exact(conn, 4)
            cmd = struct.unpack("<i", raw)[0]

            # CMD 1 - PING
            if cmd == 1:
                conn.sendall(struct.pack("<i", 123456))

            # CMD 2 - POSITION
            elif cmd == 2:
                with lock:
                    p = position.copy()
                conn.sendall(struct.pack("<ffff", *p))

            # CMD 3 - STATUS
            elif cmd == 3:
                with lock:
                    status = (
                        protocol_version,
                        win_status,
                        int(alarm),
                        int(automatic),
                        int(in_position),
                        int(referenced),
                    )
                conn.sendall(struct.pack("<iiiiii", *status))

            # CMD 10 - START PCMOVE
            elif cmd == 10:
                response = start_motion()
                print(f"[PCMOVE] start -> {response}")
                conn.sendall(struct.pack("<i", response))

            # CMD 11 - SET TARGET
            elif cmd == 11:
                data = recv_exact(conn, 16)
                new_target = list(struct.unpack("<ffff", data))
                with lock:
                    target = new_target
                print(
                    "[TARGET] "
                    f"X={target[0]:.3f} Y={target[1]:.3f} "
                    f"Z={target[2]:.3f} R={target[3]:.3f}"
                )
                conn.sendall(struct.pack("<ffff", *target))

            # CMD 12 - MOVE STATE
            elif cmd == 12:
                with lock:
                    response = (move_state, move_error, procstatus)
                conn.sendall(struct.pack("<iii", *response))

            # CMD 13 - RESET MOVE STATE
            elif cmd == 13:
                with lock:
                    if move_state != MOVE_RUNNING:
                        set_move_state(MOVE_IDLE, 0, -1)
                conn.sendall(struct.pack("<i", 13))

            # CMD 14 - PTP SPEED
            elif cmd == 14:
                data = recv_exact(conn, 4)
                requested = struct.unpack("<f", data)[0]
                accepted = max(0.0001, min(0.8, requested))
                with lock:
                    ptp_speed_factor = accepted
                print(f"[SPEED] {accepted * 100.0:.2f} %")
                conn.sendall(struct.pack("<f", accepted))

            # CMD 15 - STOP
            elif cmd == 15:
                with lock:
                    stop_requested = True
                print("[PCMOVE] STOP")
                conn.sendall(struct.pack("<i", 15))

            else:
                print(f"[WARN] Unknown command: {cmd}")
                # Do not send an arbitrary reply: the real protocol has
                # fixed reply sizes and an invented reply could desync it.

    except (ConnectionError, ConnectionResetError, BrokenPipeError, OSError):
        pass
    finally:
        try:
            conn.close()
        except OSError:
            pass
        print(f"[TCP] Client disconnected: {addr}")


def console_worker():
    """Simple fault/status injection console."""
    global automatic, alarm, referenced, position
    global move_state, move_error, procstatus

    print("""
Simulator console:
  auto 1 / auto 0
  alarm 1 / alarm 0
  ref 1 / ref 0
  error <number>       e.g. error 144384
  clear
  pos
  setpos X Y Z R
  state
  help
""")

    while True:
        try:
            line = input("SIM> ").strip()
        except (EOFError, KeyboardInterrupt):
            return

        if not line:
            continue

        parts = line.split()
        cmd = parts[0].lower()

        try:
            with lock:
                if cmd == "auto" and len(parts) == 2:
                    automatic = bool(int(parts[1]))
                    print("automatic =", automatic)

                elif cmd == "alarm" and len(parts) == 2:
                    alarm = bool(int(parts[1]))
                    print("alarm =", alarm)

                elif cmd == "ref" and len(parts) == 2:
                    referenced = bool(int(parts[1]))
                    print("referenced =", referenced)

                elif cmd == "error" and len(parts) == 2:
                    err = int(parts[1], 0)
                    set_move_state(MOVE_ERROR, err, err)
                    print("Injected RC error:", err)

                elif cmd == "clear":
                    alarm = False
                    set_move_state(MOVE_IDLE, 0, -1)
                    print("Errors cleared")

                elif cmd == "pos":
                    print(
                        f"X={position[0]:.3f} Y={position[1]:.3f} "
                        f"Z={position[2]:.3f} R={position[3]:.3f}"
                    )

                elif cmd == "setpos" and len(parts) == 5:
                    position = [float(x) for x in parts[1:5]]
                    print("Position changed:", position)

                elif cmd == "state":
                    print(
                        f"state={move_state}, error={move_error}, "
                        f"procstatus={procstatus}, auto={automatic}, "
                        f"alarm={alarm}, referenced={referenced}"
                    )

                elif cmd == "help":
                    print(
                        "auto 0|1, alarm 0|1, ref 0|1, error N, clear, "
                        "pos, setpos X Y Z R, state"
                    )

                else:
                    print("Unknown console command. Type: help")

        except ValueError as e:
            print("Bad value:", e)


def main():
    threading.Thread(target=console_worker, daemon=True).start()

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((HOST, PORT))
        server.listen(5)

        print(f"RHO4 simulator listening on {HOST}:{PORT}")

        while True:
            conn, addr = server.accept()
            threading.Thread(
                target=handle_client,
                args=(conn, addr),
                daemon=True
            ).start()


if __name__ == "__main__":
    main()
