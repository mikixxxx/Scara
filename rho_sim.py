import socket
import struct
import threading
import time
import math
import tkinter as tk
from collections import deque

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

# SR6 geometry (from the robot manual)
ARM_L1 = 330.0
ARM_L2 = 270.0
A1_MIN = -140.0
A1_MAX = 140.0
A2_MIN = -150.0
A2_MAX = 150.0

# A1 is calculated/displayed only for now.
# We will enable A1 limit checking after verifying the robot XY zero direction.
CHECK_A1_LIMIT = True

# Simulator-only travel-range-like error
TRAVEL_RANGE_ERROR = 22144

# Simulator state
lock = threading.RLock()

position = [300.0, 300.0, 180.0, 0.0]
target = position.copy()
ptp_speed_factor = 0.02       # driver sends speed / 100
linear_speed_mm_s = 10.0      # CMD16 speed, mm/s
circular_speed_mm_s = 10.0    # CMD18 speed, mm/s

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

# Actual TCP path trace for the GUI. Bounded for long G-code jobs.
trajectory = deque(maxlen=10000)
TRACE_MIN_DISTANCE_MM = 0.5


def inverse_xy(x, y):
    """Return the two mathematical planar 2R IK solutions (A1, A2) in degrees."""
    l1 = ARM_L1
    l2 = ARM_L2
    r2 = x * x + y * y

    cos_a2 = (r2 - l1 * l1 - l2 * l2) / (2.0 * l1 * l2)
    if cos_a2 < -1.0 or cos_a2 > 1.0:
        return []

    cos_a2 = max(-1.0, min(1.0, cos_a2))
    a2_abs = math.acos(cos_a2)
    solutions = []

    for a2 in (a2_abs, -a2_abs):
        k1 = l1 + l2 * math.cos(a2)
        k2 = l2 * math.sin(a2)
        a1 = math.atan2(y, x) - math.atan2(k2, k1)
        a1 = math.atan2(math.sin(a1), math.cos(a1))
        solutions.append((math.degrees(a1), math.degrees(a2)))

    return solutions


def valid_xy_solutions(x, y):
    """Return IK solutions allowed by currently enabled software limits."""
    valid = []
    for a1, a2 in inverse_xy(x, y):
        if not (A2_MIN <= a2 <= A2_MAX):
            continue
        if CHECK_A1_LIMIT and not (A1_MIN <= a1 <= A1_MAX):
            continue
        valid.append((a1, a2))
    return valid


def xy_reachable(x, y):
    return bool(valid_xy_solutions(x, y))


def choose_ik_solution(x, y, previous=None):
    """Choose a stable solution for drawing; prefer the branch closest to previous."""
    solutions = valid_xy_solutions(x, y)
    if not solutions:
        return None

    if previous is None:
        # No previous joint state is known yet.
        return min(solutions, key=lambda q: abs(q[0]) + abs(q[1]))

    return min(
        solutions,
        key=lambda q: abs(q[0] - previous[0]) + abs(q[1] - previous[1])
    )


def get_joint_position():
    # Relation verified on real SR6: R=A1+A2+A4, A3=Z.
    with lock:
        p = position.copy()
    ik = choose_ik_solution(p[0], p[1])
    if ik is None:
        raise ValueError(f"Current XY unreachable: X={p[0]:.3f} Y={p[1]:.3f}")
    a1, a2 = ik
    return a1, a2, p[2], p[3] - a1 - a2


def _angle_delta_deg(a, b):
    return (a - b + 180.0) % 360.0 - 180.0


def _v_add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _v_sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _v_scale(v, k):
    return (v[0] * k, v[1] * k, v[2] * k)


def _v_dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _v_cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _v_norm(v):
    return math.sqrt(_v_dot(v, v))


def circle_from_3_points_3d(start, mid, end):
    """Return center, radius and local basis for a circle through 3 XYZ points."""
    p0 = tuple(float(v) for v in start[:3])
    p1 = tuple(float(v) for v in mid[:3])
    p2 = tuple(float(v) for v in end[:3])

    v1 = _v_sub(p1, p0)
    v2 = _v_sub(p2, p0)
    len1 = _v_norm(v1)
    len2 = _v_norm(v2)

    if len1 < 1e-9:
        raise ValueError("CIRCULAR: MID je shodny se START")
    if len2 < 1e-9:
        raise ValueError("CIRCULAR: END je shodny se START; pouzij dva oblouky")

    normal_raw = _v_cross(v1, v2)
    normal_len = _v_norm(normal_raw)
    if normal_len < 1e-9 * max(1.0, len1 * len2):
        raise ValueError("CIRCULAR: START, MID a END lezi na primce")

    e1 = _v_scale(v1, 1.0 / len1)
    normal = _v_scale(normal_raw, 1.0 / normal_len)
    e2 = _v_cross(normal, e1)

    # Plane coordinates: START=(0,0), MID=(x1,0), END=(x2,y2)
    x1 = len1
    x2 = _v_dot(v2, e1)
    y2 = _v_dot(v2, e2)
    if abs(y2) < 1e-9:
        raise ValueError("CIRCULAR: body nedefinuji kruznici")

    center_u = x1 * 0.5
    center_v = (x2*x2 + y2*y2 - 2.0*center_u*x2) / (2.0*y2)

    center = _v_add(
        p0,
        _v_add(_v_scale(e1, center_u), _v_scale(e2, center_v))
    )
    radius = math.hypot(center_u, center_v)

    if radius < 1e-9 or not math.isfinite(radius):
        raise ValueError("CIRCULAR: neplatny polomer")

    return center, radius, e1, e2


def _circle_angle(point, center, e1, e2):
    rel = _v_sub(tuple(float(v) for v in point[:3]), center)
    return math.atan2(_v_dot(rel, e2), _v_dot(rel, e1))


def circular_sweep(start_angle, mid_angle, end_angle):
    """Signed sweep START->END that passes through MID."""
    tau = 2.0 * math.pi
    ccw_total = (end_angle - start_angle) % tau
    ccw_mid = (mid_angle - start_angle) % tau

    if ccw_total < 1e-10:
        raise ValueError("CIRCULAR: START a END jsou shodne; pouzij dva pulkruhy")

    if ccw_mid <= ccw_total + 1e-10:
        return ccw_total

    return -((start_angle - end_angle) % tau)


def circular_geometry(start, mid, end):
    center, radius, e1, e2 = circle_from_3_points_3d(start, mid, end)
    a_start = _circle_angle(start, center, e1, e2)
    a_mid = _circle_angle(mid, center, e1, e2)
    a_end = _circle_angle(end, center, e1, e2)
    sweep = circular_sweep(a_start, a_mid, a_end)
    return center, radius, e1, e2, a_start, sweep


def circular_point(geometry, u):
    """Return XYZ point on a precomputed arc, u=0..1."""
    center, radius, e1, e2, a_start, sweep = geometry
    angle = a_start + sweep * u
    radial = _v_add(
        _v_scale(e1, radius * math.cos(angle)),
        _v_scale(e2, radius * math.sin(angle))
    )
    return _v_add(center, radial)


def validate_circular_xy_path(geometry, step_mm=2.0):
    """Sample the complete arc against the simulator's A1/A2 XY limits."""
    radius = geometry[1]
    sweep = geometry[5]
    arc_length = abs(sweep) * radius
    samples = max(1, int(math.ceil(arc_length / max(step_mm, 0.1))))

    for i in range(samples + 1):
        xyz = circular_point(geometry, i / samples)
        if not xy_reachable(xyz[0], xyz[1]):
            raise ValueError(
                f"CIRCULAR path unreachable at {i/samples*100.0:.1f}%: "
                f"X={xyz[0]:.3f} Y={xyz[1]:.3f}"
            )


def _record_trace_unlocked(p):
    """Append current XY to trace. Caller must hold lock."""
    xy = (float(p[0]), float(p[1]))
    if not trajectory:
        trajectory.append(xy)
        return

    if math.hypot(xy[0]-trajectory[-1][0], xy[1]-trajectory[-1][1]) >= TRACE_MIN_DISTANCE_MM:
        trajectory.append(xy)


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


def motion_worker(start, dest, speed_factor):
    global position, in_position

    # This is intentionally not a physical RHO4 velocity model.
    max_delta = max(abs(dest[i] - start[i]) for i in range(4))
    velocity = 1000.0 * speed_factor
    duration = max(0.15, max_delta / max(velocity, 1.0))
    t0 = time.monotonic()

    while True:
        with lock:
            if stop_requested:
                in_position = False
                set_move_state(MOVE_STOPPED, 0, -1)
                return

        u = min(1.0, (time.monotonic() - t0) / duration)
        s = u*u*(3.0 - 2.0*u)

        with lock:
            position = [
                start[i] + (dest[i] - start[i]) * s
                for i in range(4)
            ]
            _record_trace_unlocked(position)

        if u >= 1.0:
            break
        time.sleep(0.01)

    with lock:
        position = dest.copy()
        _record_trace_unlocked(position)
        in_position = True
        set_move_state(MOVE_DONE, 0, -1)


def start_motion():
    global stop_requested, in_position

    with lock:
        if not automatic:
            return START_MANUAL
        if move_state == MOVE_RUNNING:
            return START_BUSY
        if move_state == MOVE_ERROR:
            return START_RC_ERROR
        if alarm:
            set_move_state(MOVE_ERROR, 144384, 144384)
            return START_RC_ERROR
        if not xy_reachable(target[0], target[1]):
            set_move_state(MOVE_ERROR, TRAVEL_RANGE_ERROR, TRAVEL_RANGE_ERROR)
            print(f"[LIMIT] XY target unreachable: X={target[0]:.3f} Y={target[1]:.3f}")
            return START_RC_ERROR

        start = position.copy()
        dest = target.copy()
        speed_factor = max(0.0001, ptp_speed_factor)
        stop_requested = False
        in_position = False
        set_move_state(MOVE_RUNNING, 0, 1)
        _record_trace_unlocked(start)

    threading.Thread(
        target=motion_worker,
        args=(start, dest, speed_factor),
        daemon=True
    ).start()
    return START_OK


def linear_motion_worker(dest, speed_mm_s):
    """
    Simulated Cartesian LINEAR move.

    The TCP follows a straight line in X/Y/Z. R is interpolated together
    with the move. smoothstep only changes acceleration/deceleration;
    the geometric path remains a straight line.
    """
    global position, in_position

    with lock:
        start = position.copy()

    dx = dest[0] - start[0]
    dy = dest[1] - start[1]
    dz = dest[2] - start[2]

    distance = math.sqrt(dx*dx + dy*dy + dz*dz)
    velocity = max(0.1, float(speed_mm_s))
    duration = max(0.15, distance / velocity)

    t0 = time.monotonic()

    while True:
        with lock:
            if stop_requested:
                in_position = False
                set_move_state(MOVE_STOPPED, 0, -1)
                return

        elapsed = time.monotonic() - t0
        u = min(1.0, elapsed / duration)

        # Acceleration/deceleration profile.
        # Every coordinate uses the same parameter, so the XYZ path is straight.
        s = u * u * (3.0 - 2.0 * u)

        with lock:
            position = [
                start[i] + (dest[i] - start[i]) * s
                for i in range(4)
            ]
            _record_trace_unlocked(position)

        if u >= 1.0:
            break

        time.sleep(0.01)

    with lock:
        position = dest.copy()
        _record_trace_unlocked(position)
        in_position = True
        set_move_state(MOVE_DONE, 0, -1)


def start_linear_motion(dest, speed_mm_s):
    global stop_requested, target, linear_speed_mm_s, in_position

    with lock:
        if not automatic:
            return START_MANUAL
        if move_state == MOVE_RUNNING:
            return START_BUSY
        if move_state == MOVE_ERROR:
            return START_RC_ERROR
        if alarm:
            set_move_state(MOVE_ERROR, 144384, 144384)
            return START_RC_ERROR
        if speed_mm_s <= 0.0:
            print(f"[LINEAR] Invalid speed: {speed_mm_s}")
            return START_RC_ERROR
        if not xy_reachable(dest[0], dest[1]):
            set_move_state(MOVE_ERROR, TRAVEL_RANGE_ERROR, TRAVEL_RANGE_ERROR)
            print(f"[LIMIT] LINEAR target unreachable: X={dest[0]:.3f} Y={dest[1]:.3f}")
            return START_RC_ERROR

        target = list(dest)
        linear_speed_mm_s = float(speed_mm_s)
        stop_requested = False
        in_position = False
        set_move_state(MOVE_RUNNING, 0, 1)
        _record_trace_unlocked(position)

    threading.Thread(
        target=linear_motion_worker,
        args=(list(dest), float(speed_mm_s)),
        daemon=True
    ).start()
    return START_OK


def circular_motion_worker(start, dest, speed_mm_s, geometry):
    """Simulated native CIRCULAR move through START -> MID -> END."""
    global position, in_position

    radius = geometry[1]
    sweep = geometry[5]
    arc_length = abs(sweep) * radius
    duration = max(0.15, arc_length / max(0.1, float(speed_mm_s)))
    dr = _angle_delta_deg(dest[3], start[3])
    t0 = time.monotonic()

    while True:
        with lock:
            if stop_requested:
                in_position = False
                set_move_state(MOVE_STOPPED, 0, -1)
                return

        u = min(1.0, (time.monotonic() - t0) / duration)
        s = u*u*(3.0 - 2.0*u)
        xyz = circular_point(geometry, s)
        r = start[3] + dr*s

        with lock:
            position = [xyz[0], xyz[1], xyz[2], r]
            _record_trace_unlocked(position)

        if u >= 1.0:
            break
        time.sleep(0.01)

    with lock:
        position = list(dest)
        _record_trace_unlocked(position)
        in_position = True
        set_move_state(MOVE_DONE, 0, -1)


def start_circular_motion(mid, dest, speed_mm_s):
    global stop_requested, target, circular_speed_mm_s, in_position

    with lock:
        if not automatic:
            return START_MANUAL
        if move_state == MOVE_RUNNING:
            return START_BUSY
        if move_state == MOVE_ERROR:
            return START_RC_ERROR
        if alarm:
            set_move_state(MOVE_ERROR, 144384, 144384)
            return START_RC_ERROR
        if speed_mm_s <= 0.0:
            print(f"[CIRCULAR] Invalid speed: {speed_mm_s}")
            return START_RC_ERROR

        start = position.copy()
        try:
            geometry = circular_geometry(start, mid, dest)
            validate_circular_xy_path(geometry, step_mm=2.0)
        except ValueError as exc:
            set_move_state(MOVE_ERROR, TRAVEL_RANGE_ERROR, TRAVEL_RANGE_ERROR)
            print(f"[CIRCULAR] {exc}")
            return START_RC_ERROR

        target = list(dest)
        circular_speed_mm_s = float(speed_mm_s)
        stop_requested = False
        in_position = False
        set_move_state(MOVE_RUNNING, 0, 1)
        _record_trace_unlocked(start)

    threading.Thread(
        target=circular_motion_worker,
        args=(start, list(dest), float(speed_mm_s), geometry),
        daemon=True
    ).start()
    return START_OK


def handle_client(conn, addr):
    global target, ptp_speed_factor, linear_speed_mm_s, circular_speed_mm_s
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
                print("[MOVE] STOP")
                conn.sendall(struct.pack("<i", 15))

            # CMD 16 - START LINEAR
            #
            # Payload:
            #   float32 X
            #   float32 Y
            #   float32 Z
            #   float32 R
            #   float32 speed_mm_s
            #
            # Reply:
            #   int32 START_OK / START_MANUAL / START_BUSY / START_RC_ERROR
            elif cmd == 16:
                data = recv_exact(conn, 20)
                x, y, z, r, speed_mm_s = struct.unpack("<fffff", data)

                response = start_linear_motion(
                    [x, y, z, r],
                    speed_mm_s
                )

                print(
                    "[LINEAR] "
                    f"X={x:.3f} Y={y:.3f} Z={z:.3f} R={r:.3f} "
                    f"V={speed_mm_s:.3f} mm/s -> {response}"
                )

                conn.sendall(struct.pack("<i", response))

            # CMD 17 - ACTUAL JOINT POSITION: A1, A2, A3, A4
            elif cmd == 17:
                try:
                    joints = get_joint_position()
                    conn.sendall(struct.pack("<ffff", *joints))
                except ValueError as e:
                    print(f"[JOINT] {e}")
                    conn.sendall(struct.pack("<ffff", float("nan"), float("nan"), float("nan"), float("nan")))

            # CMD 18 - START CIRCULAR
            # Payload: MID X,Y,Z,R + END X,Y,Z,R + speed_mm_s = 9 float32
            elif cmd == 18:
                data = recv_exact(conn, 36)
                values = struct.unpack("<fffffffff", data)
                mid = list(values[0:4])
                dest = list(values[4:8])
                speed_mm_s = values[8]

                response = start_circular_motion(mid, dest, speed_mm_s)
                print(
                    "[CIRCULAR] "
                    f"MID=({mid[0]:.3f},{mid[1]:.3f},{mid[2]:.3f},{mid[3]:.3f}) "
                    f"END=({dest[0]:.3f},{dest[1]:.3f},{dest[2]:.3f},{dest[3]:.3f}) "
                    f"V={speed_mm_s:.3f} mm/s -> {response}"
                )
                conn.sendall(struct.pack("<i", response))

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
  cleartrace
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

                elif cmd == "cleartrace":
                    trajectory.clear()
                    print("Trajectory cleared")

                elif cmd == "help":
                    print(
                        "auto 0|1, alarm 0|1, ref 0|1, error N, clear, "
                        "pos, setpos X Y Z R, state, cleartrace"
                    )

                else:
                    print("Unknown console command. Type: help")

        except ValueError as e:
            print("Bad value:", e)


class SimulatorView:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Bosch Rexroth SR6 - RHO4 Simulator")
        self.root.geometry("820x805")
        self.root.resizable(False,False)

        self.canvas = tk.Canvas(
            self.root,
            width=780,
            height=620,
            bg="white",
            highlightthickness=1,
            highlightbackground="#777"
        )
        self.canvas.pack(padx=20, pady=(20, 8))

        controls = tk.Frame(self.root)
        controls.pack(fill="x", padx=20, pady=(0, 6))
        tk.Button(controls, text="Clear trace", command=self.clear_trace).pack(side="left")

        self.info = tk.Label(
            self.root,
            text="",
            justify="left",
            anchor="w",
            font=("Consolas", 11)
        )
        self.info.pack(fill="x", padx=20, pady=(0, 15))

        self.last_ik = None
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.update_view()

    def on_close(self):
        self.root.destroy()

    def clear_trace(self):
        with lock:
            trajectory.clear()

    def world_to_canvas(self, x, y):
        # Base in the middle. Mathematical +Y is drawn upward.
        cx = 390.0
        cy = 310.0
        scale = 0.46
        return cx + x * scale, cy - y * scale

    def draw_workspace(self):
        cx, cy = self.world_to_canvas(0.0, 0.0)
        scale = 0.46

        # Geometrical maximum reach
        rmax = (ARM_L1 + ARM_L2) * scale
        self.canvas.create_oval(
            cx-rmax, cy-rmax, cx+rmax, cy+rmax,
            outline="#bbbbbb", dash=(5, 4), width=1
        )

        # Inner radius produced by |A2| <= 150 deg.
        a2 = math.radians(A2_MAX)
        rmin_mm = math.sqrt(
            ARM_L1**2 + ARM_L2**2 +
            2.0 * ARM_L1 * ARM_L2 * math.cos(a2)
        )
        rmin = rmin_mm * scale
        self.canvas.create_oval(
            cx-rmin, cy-rmin, cx+rmin, cy+rmin,
            outline="#dddddd", dash=(3, 3), width=1
        )

        # Axes
        self.canvas.create_line(30, cy, 750, cy, fill="#dddddd")
        self.canvas.create_line(cx, 30, cx, 590, fill="#dddddd")
        self.canvas.create_text(742, cy-12, text="+X", fill="#777")
        self.canvas.create_text(cx+16, 38, text="+Y", fill="#777")

    def draw_robot(self, x, y, ik):
        bx, by = self.world_to_canvas(0.0, 0.0)

        if ik is None:
            self.canvas.create_oval(
                bx-7, by-7, bx+7, by+7,
                fill="black", outline=""
            )
            return

        a1, a2 = ik
        q1 = math.radians(a1)
        q2 = math.radians(a2)

        elbow_x = ARM_L1 * math.cos(q1)
        elbow_y = ARM_L1 * math.sin(q1)

        ex, ey = self.world_to_canvas(elbow_x, elbow_y)
        tx, ty = self.world_to_canvas(x, y)

        self.canvas.create_line(
            bx, by, ex, ey,
            width=7, fill="#3b82f6"
        )
        self.canvas.create_line(
            ex, ey, tx, ty,
            width=7, fill="#ef4444"
        )

        self.canvas.create_oval(bx-8, by-8, bx+8, by+8, fill="black")
        self.canvas.create_oval(ex-7, ey-7, ex+7, ey+7, fill="#222")
        self.canvas.create_oval(tx-6, ty-6, tx+6, ty+6, fill="#16a34a")

    def update_view(self):
        global shutdown_requested

        with lock:
            p = position.copy()
            t = target.copy()
            state = move_state
            err = move_error
            proc = procstatus
            speed = ptp_speed_factor
            lin_speed = linear_speed_mm_s
            circ_speed = circular_speed_mm_s
            trace = list(trajectory)
            auto = automatic
            alm = alarm
            ref = referenced

        self.canvas.delete("all")
        self.draw_workspace()

        if len(trace) >= 2:
            coords = []
            for x, y in trace:
                cx, cy = self.world_to_canvas(x, y)
                coords.extend((cx, cy))
            self.canvas.create_line(*coords, fill="#0ea5e9", width=2)

        ik = choose_ik_solution(p[0], p[1], self.last_ik)
        if ik is not None:
            self.last_ik = ik

        self.draw_robot(p[0], p[1], ik)

        # Target cross
        tx, ty = self.world_to_canvas(t[0], t[1])
        self.canvas.create_line(tx-8, ty, tx+8, ty, fill="#8b5cf6", width=2)
        self.canvas.create_line(tx, ty-8, tx, ty+8, fill="#8b5cf6", width=2)
        self.canvas.create_text(tx+10, ty-12, text="TARGET", anchor="w", fill="#8b5cf6")

        if ik is None:
            ik_text = "A1=---  A2=---  A3=---  A4=---  XY: UNREACHABLE"
        else:
            a1, a2 = ik
            a3 = p[2]
            a4 = p[3] - a1 - a2
            ik_text = (
                f"A1={a1:8.2f}°  A2={a2:8.2f}°  "
                f"A3={a3:8.2f}   A4={a4:8.2f}°"
            )

        target_ok = xy_reachable(t[0], t[1])

        self.info.config(
            text=(
                f"POSITION  X={p[0]:8.3f}  Y={p[1]:8.3f}  "
                f"Z={p[2]:8.3f}  R={p[3]:8.3f}\n"
                f"IK        {ik_text}\n"
                f"TARGET    X={t[0]:8.3f}  Y={t[1]:8.3f}  "
                f"Z={t[2]:8.3f}  R={t[3]:8.3f}  "
                f"{'OK' if target_ok else 'UNREACHABLE'}\n"
                f"STATE={state}  ERROR={err}  PROC={proc}  "
                f"PTP={speed*100.0:.2f}%  LIN={lin_speed:.2f} mm/s  "
                f"CIRC={circ_speed:.2f} mm/s  "
                f"AUTO={int(auto)} ALARM={int(alm)} REF={int(ref)}\n"
                f"A1 limit check: {'ON' if CHECK_A1_LIMIT else 'OFF'}  CMD17: ON  CMD18: ON"
            )
        )

        self.root.after(30, self.update_view)

    def run(self):
        self.root.mainloop()


def server_worker():
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


def main():
    threading.Thread(target=console_worker, daemon=True).start()
    threading.Thread(target=server_worker, daemon=True).start()

    view = SimulatorView()
    view.run()


if __name__ == "__main__":
    main()
