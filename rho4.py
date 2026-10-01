import socket
import struct
import time
import threading
import math

class Rho4:

    # =========================================================
    # MOVE STATES
    # =========================================================

    MOVE_IDLE = 0
    MOVE_RUNNING = 1
    MOVE_DONE = 2
    MOVE_ERROR = 3
    MOVE_STOPPED = 4

    # =========================================================
    # START PCMOVE RESPONSES
    # =========================================================

    START_OK = 10
    START_MANUAL = -10
    START_BUSY = -11
    START_RC_ERROR = -12

    # PCPATH protocol
    PATH_UPLOAD_OK = 20
    PATH_COUNT_ERROR = -20
    PATH_SPEED_ERROR = -21
    PATH_MAX_POINTS = 16

    # STREAM V1 protocol - simulator prototype
    STREAM_UPLOAD_OK = 22
    STREAM_SLOT_ERROR = -22
    STREAM_SLOT_BUSY = -23
    STREAM_NOT_READY = -24
    STREAM_SEQUENCE_ERROR = -25
    STREAM_UNDERRUN_ERROR = 29001

    # ==========================================================
    # SR6 GEOMETRY
    # ==========================================================

    ARM_L1 = 330.0
    ARM_L2 = 270.0

    A1_MIN = -140.0
    A1_MAX =  140.0

    A2_MIN = -150.0
    A2_MAX =  150.0

    A4_MIN = -180.0
    A4_MAX =  180.0

    Z_MIN = 5.0
    Z_MAX = 195.0

    R_MIN = -175.0
    R_MAX = 175.0

    # ==========================================================
    # SOFTWARE LIMITS
    # ==========================================================

    XY_RADIUS_MAX = 590

    X_MIN = -500.0
    X_MAX = 500.0

    Y_MIN = -500.0
    Y_MAX = 500.0


    def __init__(
        self,
        host="127.0.0.1",
        port=6051,
        timeout=3.0
    ):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.sock = None
        self._lock = threading.RLock()


    # =========================================================
    # CONNECTION
    # =========================================================

    def connect(self):
        with self._lock:
            if self.sock is not None:
                try:
                    self.sock.close()
                except OSError:
                    pass
                self.sock = None
            self.sock = socket.create_connection((self.host, self.port),timeout=self.timeout)
            self.sock.settimeout(self.timeout)



    def disconnect(self):
        with self._lock:
            if self.sock is not None:
                try:
                    self.sock.close()
                except OSError:
                    pass

            self.sock = None


    def is_connected(self):
        return self.sock is not None


    # =========================================================
    # LOW LEVEL COMMUNICATION
    # =========================================================

    def _recv_exact(self, size):
        data = b""

        while len(data) < size:
            if self.sock is None:
                raise ConnectionError("Nejsi pripojen k RHO")

            chunk = self.sock.recv(size - len(data))

            if not chunk:
                # _recv_exact() is normally called while _lock is held.
                # Close the socket directly here instead of calling
                # disconnect(), which would try to acquire the same lock.
                try:
                    self.sock.close()
                except OSError:
                    pass
                self.sock = None
                raise ConnectionError("RHO ukoncilo TCP spojeni")

            data += chunk

        return data


    def _send_cmd(self, cmd):
        if self.sock is None:
            raise ConnectionError("Nejsi pripojen k RHO")

        self.sock.sendall(
            struct.pack("<i", cmd)
        )


    # =========================================================
    # BASIC COMMANDS
    # =========================================================

    def ping(self):
        with self._lock:
            self._send_cmd(1)

            data = self._recv_exact(4)

            response = struct.unpack(
                "<i",
                data
            )[0]
        return response == 123456


    def get_position(self):
        with self._lock:
            self._send_cmd(2)

            data = self._recv_exact(16)

            x, y, z, r = struct.unpack(
                "<ffff",
                data
            )

            return x, y, z, r


    def get_status(self):
        with self._lock:
            self._send_cmd(3)

            data = self._recv_exact(24)

            (
                version,
                win_status,
                alarm,
                automatic,
                in_position,
                referenced
            ) = struct.unpack(
                "<iiiiii",
                data
            )

        return {
            "version": version,
            "win_status": win_status,
            "alarm": bool(alarm),
            "automatic": bool(automatic),
            "in_position": bool(in_position),
            "referenced": bool(referenced)
        }


    # =========================================================
    # TARGET
    # =========================================================

    def set_target(self, x, y, z, r):
        with self._lock:
            self._send_cmd(11)
            if self.sock is None:
                raise ConnectionError(
                    "Nejsi pripojen k RHO"
                )

            self.sock.sendall(
                struct.pack(
                    "<ffff",
                    x,
                    y,
                    z,
                    r
                )
            )

            data = self._recv_exact(16)

            return struct.unpack(
                "<ffff",
                data
            )


    # =========================================================
    # PCMOVE CONTROL
    # =========================================================

    def start_pcmove(self):
        with self._lock:
            self._send_cmd(10)
            data = self._recv_exact(4)
            response = struct.unpack(
                "<i",
                data
            )[0]

        return response


    def get_move_state(self):
        with self._lock:
            self._send_cmd(12)

            data = self._recv_exact(12)

            state, error, procstatus = struct.unpack(
                "<iii",
                data
            )
            return state, error, procstatus


    def reset_move_state(self):
        with self._lock:
            self._send_cmd(13)
            data = self._recv_exact(4)
            response = struct.unpack(
                "<i",
                data
            )[0]
            return response


    def set_ptp_speed(self, speed):
        speed = float(speed)
        if speed < 0.01 or speed > 80.0:
            raise ValueError(
                "PTP rychlost musi byt v rozmezi 0.01 - 80.0"
            )
        speed_factor = speed / 100.0
        with self._lock:
            self._send_cmd(14)
            if self.sock is None:
                raise ConnectionError("Nejsi pripojen k RHO")
            self.sock.sendall(struct.pack("<f", speed_factor))
            data = self._recv_exact(4)
            accepted_factor = struct.unpack("<f", data)[0]
        return accepted_factor * 100.0


    def start_ptp(self, x,y,z,r,speed=2.0):
        self.check_target(x,y,z,r)
        accepted_speed = self.set_ptp_speed(speed)
        target = self.set_target(x,y,z,r)

        # Start PCMOVE

        response = self.start_pcmove()
        if response == self.START_MANUAL:
            raise RuntimeError("Robot je v manual rezimu")
        if response == self.START_BUSY:
            raise RuntimeError("Robot uz provadi pohyb")
        if response == self.START_RC_ERROR:
            state,error,procstatus = self.get_move_state()
            raise RuntimeError(f"PCMOVE je v RC error: {error} "f"(ProcStatus={procstatus})")
        if response != self.START_OK:
            raise RuntimeError(f"Chyba pri startu PCMOVE: {response}")
        return True

    def wait_move(self, timeout=30.0):
        # ceka na ukonceni aktualniho PTP/LINEAR/CIRCULAR/PCPATH pohybu
        # True -> Dokonceno

        start_time = time.monotonic()
        last_status = None

        while True:
            state,error,procstatus = self.get_move_state()
            current_status = (state,error,procstatus)
            if current_status != last_status:
                print(
                f"State: {state}, "
                f"Error: {error}, "
                f"ProcStatus: {procstatus}")
                last_status = current_status

            if state == self.MOVE_ERROR:
                raise RuntimeError(f"Pohyb skoncil RC chybou: {error}")
            if state == self.MOVE_STOPPED:
                return False
            if (state == self.MOVE_DONE and procstatus == -1):
                return True
            if time.monotonic() - start_time > timeout:
                raise TimeoutError("Robot nedokoncil pohyb v casovem limitu")
            time.sleep(0.05)
        

    def check_target(self, x, y, z, r):
        solutions = self.inverse_xy(float(x), float(y))
        if not solutions:
            raise ValueError(f"Target X={x:.3f} Y={y:.3f} je mimo geometricky dosah robota")

        valid = []
        for a1, a2 in solutions:
            # Overeno na skutecnem SR6: R = A1 + A2 + A4
            a4 = float(r) - a1 - a2

            if not (self.A1_MIN <= a1 <= self.A1_MAX):
                continue
            if not (self.A2_MIN <= a2 <= self.A2_MAX):
                continue
            if not (self.A4_MIN <= a4 <= self.A4_MAX):
                continue

            valid.append({
                "a1": a1,
                "a2": a2,
                "a3": float(z),
                "a4": a4,
            })

        if not valid:
            raise ValueError(
                f"Target X={x:.3f} Y={y:.3f} Z={z:.3f} R={r:.3f} "
                "nema zadnou konfiguraci v limitech A1/A2/A4"
            )
        return valid

    def inverse_xy(self, x, y):
        l1 = self.ARM_L1
        l2 = self.ARM_L2
        r2 = x*x + y*y

        cos_a2 = (r2 - l1*l1 - l2*l2) / (2.0*l1*l2)
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

    def check_xy_kinematics(self, x, y):
        valid = []
        for a1, a2 in self.inverse_xy(x, y):
            if (
                self.A1_MIN <= a1 <= self.A1_MAX
                and self.A2_MIN <= a2 <= self.A2_MAX
            ):
                valid.append((a1, a2))
        return valid

    @staticmethod
    def _angle_delta_deg(a, b):
        return (a - b + 180.0) % 360.0 - 180.0

    def select_target_configuration(self, x, y, z, r):
        valid = self.check_target(x, y, z, r)
        current_a1, current_a2, _, current_a4 = self.get_joint_position()

        def distance(solution):
            da1 = self._angle_delta_deg(solution["a1"], current_a1)
            da2 = self._angle_delta_deg(solution["a2"], current_a2)
            da4 = self._angle_delta_deg(solution["a4"], current_a4)
            return da1*da1 + da2*da2 + da4*da4

        return min(valid, key=distance)


    def start_linear(self, x,y,z,r,speed_mm_s=10.0):
        self.check_linear_path(x,y,z,r,step_mm=2.0)

        if speed_mm_s <= 0:
            raise ValueError("LINEAR speed musi byt > 0 mm/s")

        with self._lock:
            #CMD16
            if self.sock is None:
                raise ConnectionError("Nejsi pripojen k RHO")
            self.sock.sendall(struct.pack("<ifffff",16,float(x),float(y),float(z),float(r),float(speed_mm_s)))
            response = struct.unpack("<i", self._recv_exact(4))[0]

        if response == self.START_MANUAL:
            raise RuntimeError("Robot je v MANUAL rezimu")
        if response == self.START_BUSY:
            raise RuntimeError("Robot uz provadi pohyb")
        if response == self.START_RC_ERROR:
            state, error, procstatus = self.get_move_state()
            raise RuntimeError(
                f"LINEAR RC error: {error} "
                f"(ProcStatus={procstatus})"
            )

        if response != self.START_OK:
            raise RuntimeError(f"Chyba pri startu LINEAR: {response}")

        return True

    def get_joint_position(self):
        with self._lock:
            if self.sock is None:
                raise ConnectionError("Nejsi pripojen k RHO")
            self.sock.sendall(struct.pack("<i",17))
            data = self._recv_exact(16)
            return struct.unpack("<ffff",data)

    def check_linear_path(
        self,
        x, y, z, r,
        step_mm=2.0,
        joint_step_limit=10.0,
        angular_step_deg=5.0
    ):
        if step_mm <= 0:
            raise ValueError("step_mm musi byt > 0")
        if angular_step_deg <= 0:
            raise ValueError("angular_step_deg musi byt > 0")

        sx, sy, sz, sr = self.get_position()
        ca1, ca2, ca3, ca4 = self.get_joint_position()

        dx = float(x) - sx
        dy = float(y) - sy
        dz = float(z) - sz
        dr = self._angle_delta_deg(float(r), sr)

        length = math.sqrt(dx*dx + dy*dy + dz*dz)
        xyz_samples = int(math.ceil(length / step_mm))
        r_samples = int(math.ceil(abs(dr) / angular_step_deg))
        samples = max(1, xyz_samples, r_samples)

        previous = {
            "a1": ca1,
            "a2": ca2,
            "a3": ca3,
            "a4": ca4,
        }

        path = []

        for i in range(1, samples + 1):
            t = i / samples

            px = sx + dx * t
            py = sy + dy * t
            pz = sz + dz * t
            pr = sr + dr * t

            solutions = self.check_target(px, py, pz, pr)

            def joint_distance(sol):
                da1 = self._angle_delta_deg(sol["a1"], previous["a1"])
                da2 = self._angle_delta_deg(sol["a2"], previous["a2"])
                da4 = self._angle_delta_deg(sol["a4"], previous["a4"])
                return da1*da1 + da2*da2 + da4*da4

            selected = min(solutions, key=joint_distance)

            da1 = abs(self._angle_delta_deg(selected["a1"], previous["a1"]))
            da2 = abs(self._angle_delta_deg(selected["a2"], previous["a2"]))
            da4 = abs(self._angle_delta_deg(selected["a4"], previous["a4"]))

            if max(da1, da2, da4) > joint_step_limit:
                raise ValueError(
                    "LINEAR path: prilis velky skok jointu "
                    f"v {t*100:.1f}% drahy: "
                    f"dA1={da1:.2f} "
                    f"dA2={da2:.2f} "
                    f"dA4={da4:.2f}"
                )

            point = {
                "t": t,
                "x": px,
                "y": py,
                "z": pz,
                "r": pr,
                "a1": selected["a1"],
                "a2": selected["a2"],
                "a3": selected["a3"],
                "a4": selected["a4"],
            }

            path.append(point)
            previous = selected

        return path

    # =========================================================
    # CIRCULAR GEOMETRY / SAFETY CHECK
    # =========================================================

    @staticmethod
    def _v_add(a, b):
        return (a[0] + b[0], a[1] + b[1], a[2] + b[2])

    @staticmethod
    def _v_sub(a, b):
        return (a[0] - b[0], a[1] - b[1], a[2] - b[2])

    @staticmethod
    def _v_scale(a, k):
        return (a[0] * k, a[1] * k, a[2] * k)

    @staticmethod
    def _v_dot(a, b):
        return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]

    @staticmethod
    def _v_cross(a, b):
        return (
            a[1]*b[2] - a[2]*b[1],
            a[2]*b[0] - a[0]*b[2],
            a[0]*b[1] - a[1]*b[0],
        )

    @classmethod
    def _v_norm(cls, a):
        return math.sqrt(cls._v_dot(a, a))

    @classmethod
    def _circle_geometry(cls, start, mid, end):
        """
        Circle through START/MID/END in XYZ.

        Returns:
            center, radius, e1, e2, start_angle, sweep, mid_fraction
        """
        p0 = tuple(float(v) for v in start[:3])
        p1 = tuple(float(v) for v in mid[:3])
        p2 = tuple(float(v) for v in end[:3])

        v1 = cls._v_sub(p1, p0)
        v2 = cls._v_sub(p2, p0)
        len1 = cls._v_norm(v1)
        len2 = cls._v_norm(v2)

        if len1 < 1e-9:
            raise ValueError("CIRCULAR: MID je shodny se START")
        if len2 < 1e-9:
            raise ValueError(
                "CIRCULAR: END je shodny se START; "
                "celou kruznici rozdel na dva oblouky"
            )

        normal_raw = cls._v_cross(v1, v2)
        normal_len = cls._v_norm(normal_raw)

        if normal_len < 1e-9 * max(1.0, len1 * len2):
            raise ValueError("CIRCULAR: START, MID a END lezi na primce")

        e1 = cls._v_scale(v1, 1.0 / len1)
        normal = cls._v_scale(normal_raw, 1.0 / normal_len)
        e2 = cls._v_cross(normal, e1)

        # Local plane:
        # START=(0,0), MID=(x1,0), END=(x2,y2)
        x1 = len1
        x2 = cls._v_dot(v2, e1)
        y2 = cls._v_dot(v2, e2)

        if abs(y2) < 1e-9:
            raise ValueError("CIRCULAR: body nedefinuji kruznici")

        center_u = x1 * 0.5
        center_v = (
            x2*x2 + y2*y2 - 2.0*center_u*x2
        ) / (2.0*y2)

        center = cls._v_add(
            p0,
            cls._v_add(
                cls._v_scale(e1, center_u),
                cls._v_scale(e2, center_v)
            )
        )

        radius = math.hypot(center_u, center_v)

        if radius < 1e-9 or not math.isfinite(radius):
            raise ValueError("CIRCULAR: neplatny polomer")

        def angle_of(point):
            rel = cls._v_sub(
                tuple(float(v) for v in point[:3]),
                center
            )
            return math.atan2(
                cls._v_dot(rel, e2),
                cls._v_dot(rel, e1)
            )

        a_start = angle_of(start)
        a_mid = angle_of(mid)
        a_end = angle_of(end)

        tau = 2.0 * math.pi
        ccw_total = (a_end - a_start) % tau
        ccw_mid = (a_mid - a_start) % tau

        if ccw_total < 1e-10:
            raise ValueError(
                "CIRCULAR: START a END jsou shodne; "
                "celou kruznici rozdel na dva oblouky"
            )

        if ccw_mid <= ccw_total + 1e-10:
            sweep = ccw_total
            mid_sweep = ccw_mid
        else:
            sweep = -((a_start - a_end) % tau)
            mid_sweep = -((a_start - a_mid) % tau)

        if abs(sweep) < 1e-10:
            raise ValueError("CIRCULAR: nulovy uhel oblouku")

        mid_fraction = mid_sweep / sweep

        if not (0.0 < mid_fraction < 1.0):
            raise ValueError("CIRCULAR: MID nelezi uvnitr vybraneho oblouku")

        return (
            center,
            radius,
            e1,
            e2,
            a_start,
            sweep,
            mid_fraction,
        )

    @classmethod
    def _circular_xyz(cls, geometry, u):
        center, radius, e1, e2, a_start, sweep, _ = geometry
        angle = a_start + sweep * u
        radial = cls._v_add(
            cls._v_scale(e1, radius * math.cos(angle)),
            cls._v_scale(e2, radius * math.sin(angle))
        )
        return cls._v_add(center, radial)

    def check_circular_path(
        self,
        mid_x, mid_y, mid_z, mid_r,
        end_x, end_y, end_z, end_r,
        step_mm=2.0,
        joint_step_limit=10.0,
        angular_step_deg=5.0
    ):
        """
        Dry-run safety check for CIRCULAR.

        Geometry is sampled along the complete START->MID->END arc.
        R is conservatively sampled through START_R -> MID_R -> END_R.
        """
        if step_mm <= 0:
            raise ValueError("step_mm musi byt > 0")
        if angular_step_deg <= 0:
            raise ValueError("angular_step_deg musi byt > 0")

        sx, sy, sz, sr = self.get_position()
        ca1, ca2, ca3, ca4 = self.get_joint_position()

        start = (sx, sy, sz, sr)
        mid = (
            float(mid_x), float(mid_y),
            float(mid_z), float(mid_r)
        )
        end = (
            float(end_x), float(end_y),
            float(end_z), float(end_r)
        )

        geometry = self._circle_geometry(start, mid, end)
        radius = geometry[1]
        sweep = geometry[5]
        mid_fraction = geometry[6]

        arc_length = abs(sweep) * radius

        dr1 = self._angle_delta_deg(mid[3], sr)
        mid_r_unwrapped = sr + dr1
        dr2 = self._angle_delta_deg(end[3], mid_r_unwrapped)

        xyz_samples = int(math.ceil(arc_length / step_mm))
        r_samples = int(
            math.ceil((abs(dr1) + abs(dr2)) / angular_step_deg)
        )
        samples = max(1, xyz_samples, r_samples)

        # Include MID exactly even when it does not fall on the regular grid.
        u_values = {i / samples for i in range(1, samples + 1)}
        u_values.add(mid_fraction)
        u_values = sorted(u_values)

        previous = {
            "a1": ca1,
            "a2": ca2,
            "a3": ca3,
            "a4": ca4,
        }

        path = []

        for u in u_values:
            px, py, pz = self._circular_xyz(geometry, u)

            if u <= mid_fraction:
                local = u / mid_fraction
                pr = sr + dr1 * local
            else:
                local = (u - mid_fraction) / (1.0 - mid_fraction)
                pr = mid_r_unwrapped + dr2 * local

            solutions = self.check_target(px, py, pz, pr)

            def joint_distance(sol):
                da1 = self._angle_delta_deg(sol["a1"], previous["a1"])
                da2 = self._angle_delta_deg(sol["a2"], previous["a2"])
                da4 = self._angle_delta_deg(sol["a4"], previous["a4"])
                return da1*da1 + da2*da2 + da4*da4

            selected = min(solutions, key=joint_distance)

            da1 = abs(self._angle_delta_deg(
                selected["a1"], previous["a1"]
            ))
            da2 = abs(self._angle_delta_deg(
                selected["a2"], previous["a2"]
            ))
            da4 = abs(self._angle_delta_deg(
                selected["a4"], previous["a4"]
            ))

            if max(da1, da2, da4) > joint_step_limit:
                raise ValueError(
                    "CIRCULAR path: prilis velky skok jointu "
                    f"v {u*100:.1f}% drahy: "
                    f"dA1={da1:.2f} "
                    f"dA2={da2:.2f} "
                    f"dA4={da4:.2f}"
                )

            point = {
                "t": u,
                "x": px,
                "y": py,
                "z": pz,
                "r": pr,
                "a1": selected["a1"],
                "a2": selected["a2"],
                "a3": selected["a3"],
                "a4": selected["a4"],
            }

            path.append(point)
            previous = selected

        return path

    def start_circular(
        self,
        mid_x, mid_y, mid_z, mid_r,
        end_x, end_y, end_z, end_r,
        speed_mm_s=10.0
    ):
        if speed_mm_s <= 0:
            raise ValueError("CIRCULAR speed musi byt > 0 mm/s")

        # Client-side dry-run.  This is an additional guard; native RHO4
        # travel limits remain authoritative.
        self.check_circular_path(
            mid_x, mid_y, mid_z, mid_r,
            end_x, end_y, end_z, end_r,
            step_mm=2.0
        )

        payload = struct.pack(
            "<ifffffffff",
            18,
            float(mid_x),
            float(mid_y),
            float(mid_z),
            float(mid_r),
            float(end_x),
            float(end_y),
            float(end_z),
            float(end_r),
            float(speed_mm_s)
        )

        with self._lock:
            if self.sock is None:
                raise ConnectionError("Nejsi pripojen k RHO")

            self.sock.sendall(payload)
            response = struct.unpack(
                "<i",
                self._recv_exact(4)
            )[0]

        if response == self.START_OK:
            return True

        if response == self.START_MANUAL:
            raise RuntimeError("Robot neni v AUTO")

        if response == self.START_BUSY:
            raise RuntimeError("Predchozi pohyb jeste probiha")

        if response == self.START_RC_ERROR:
            state, error, procstatus = self.get_move_state()
            raise RuntimeError(
                f"CIRCULAR RC error: {error} "
                f"(ProcStatus={procstatus})"
            )

        raise RuntimeError(
            f"Neznama odpoved z CMD18: {response}"
        )


    # =========================================================
    # PCPATH - BUFFERED LINEAR PATH (CMD20 / CMD21)
    # =========================================================

    @staticmethod
    def _normalize_path_point(point):
        """
        Accept either:
          (x, y, z, r)
        or:
          {"X": ..., "Y": ..., "Z": ..., "R": ...}
        """
        if isinstance(point, dict):
            try:
                return (
                    float(point["X"]),
                    float(point["Y"]),
                    float(point["Z"]),
                    float(point["R"]),
                )
            except KeyError as exc:
                raise ValueError(
                    "PCPATH dict bod musi obsahovat X,Y,Z,R"
                ) from exc

        if len(point) != 4:
            raise ValueError("PCPATH bod musi mit 4 hodnoty: X,Y,Z,R")

        return tuple(float(v) for v in point)


    def check_path(
        self,
        points,
        step_mm=2.0,
        joint_step_limit=10.0,
        angular_step_deg=5.0
    ):
        """
        Dry-run celeho PCPATH bufferu pred odeslanim.

        Kontroluje nejen koncove body, ale i LINEAR useky mezi nimi
        a drzi kontinuitu zvolene IK vetve pres cely buffer.
        """
        if step_mm <= 0:
            raise ValueError("step_mm musi byt > 0")
        if angular_step_deg <= 0:
            raise ValueError("angular_step_deg musi byt > 0")
        if joint_step_limit <= 0:
            raise ValueError("joint_step_limit musi byt > 0")

        normalized = [
            self._normalize_path_point(point)
            for point in points
        ]

        if not 1 <= len(normalized) <= self.PATH_MAX_POINTS:
            raise ValueError(
                f"PCPATH podporuje 1..{self.PATH_MAX_POINTS} bodu"
            )

        sx, sy, sz, sr = self.get_position()
        ca1, ca2, ca3, ca4 = self.get_joint_position()

        previous_xyzr = (sx, sy, sz, sr)
        previous_joint = {
            "a1": ca1,
            "a2": ca2,
            "a3": ca3,
            "a4": ca4,
        }

        checked = []

        for segment_index, dest in enumerate(normalized, start=1):
            ax, ay, az, ar = previous_xyzr
            bx, by, bz, br_raw = dest

            dx = bx - ax
            dy = by - ay
            dz = bz - az
            dr = self._angle_delta_deg(br_raw, ar)

            length = math.sqrt(dx*dx + dy*dy + dz*dz)
            xyz_samples = int(math.ceil(length / step_mm))
            r_samples = int(math.ceil(abs(dr) / angular_step_deg))
            samples = max(1, xyz_samples, r_samples)

            segment_checked = []

            for sample_index in range(1, samples + 1):
                t = sample_index / samples

                px = ax + dx*t
                py = ay + dy*t
                pz = az + dz*t
                pr = ar + dr*t

                solutions = self.check_target(px, py, pz, pr)

                def joint_distance(sol):
                    da1 = self._angle_delta_deg(
                        sol["a1"], previous_joint["a1"]
                    )
                    da2 = self._angle_delta_deg(
                        sol["a2"], previous_joint["a2"]
                    )
                    da4 = self._angle_delta_deg(
                        sol["a4"], previous_joint["a4"]
                    )
                    return da1*da1 + da2*da2 + da4*da4

                selected = min(solutions, key=joint_distance)

                da1 = abs(self._angle_delta_deg(
                    selected["a1"], previous_joint["a1"]
                ))
                da2 = abs(self._angle_delta_deg(
                    selected["a2"], previous_joint["a2"]
                ))
                da4 = abs(self._angle_delta_deg(
                    selected["a4"], previous_joint["a4"]
                ))

                if max(da1, da2, da4) > joint_step_limit:
                    raise ValueError(
                        "PCPATH: prilis velky skok jointu "
                        f"v segmentu {segment_index}, "
                        f"{t*100.0:.1f}%: "
                        f"dA1={da1:.2f} "
                        f"dA2={da2:.2f} "
                        f"dA4={da4:.2f}"
                    )

                sample = {
                    "segment": segment_index,
                    "t": t,
                    "x": px,
                    "y": py,
                    "z": pz,
                    "r": pr,
                    "a1": selected["a1"],
                    "a2": selected["a2"],
                    "a3": selected["a3"],
                    "a4": selected["a4"],
                }
                segment_checked.append(sample)
                previous_joint = selected

            checked.append(segment_checked)
            previous_xyzr = (bx, by, bz, br_raw)

        return checked


    def upload_path(
        self,
        points,
        speed_mm_s=5.0,
        validate=True
    ):
        """
        CMD20:
          INTEGER PATH_COUNT
          REAL PATH_SPEED
          16 x (X,Y,Z,R) float32

        Payload ma vzdy pevnou velikost; nepouzite body jsou nulove.
        """
        normalized = [
            self._normalize_path_point(point)
            for point in points
        ]

        if not 1 <= len(normalized) <= self.PATH_MAX_POINTS:
            raise ValueError(
                f"PCPATH podporuje 1..{self.PATH_MAX_POINTS} bodu"
            )

        speed_mm_s = float(speed_mm_s)
        if speed_mm_s <= 0.0:
            raise ValueError("PCPATH speed musi byt > 0 mm/s")

        if validate:
            self.check_path(normalized)

        padded = list(normalized)
        while len(padded) < self.PATH_MAX_POINTS:
            padded.append((0.0, 0.0, 0.0, 0.0))

        flat = []
        for point in padded:
            flat.extend(point)

        payload = struct.pack(
            "<iif64f",
            20,
            len(normalized),
            speed_mm_s,
            *flat
        )

        with self._lock:
            if self.sock is None:
                raise ConnectionError("Nejsi pripojen k RHO")

            self.sock.sendall(payload)
            response = struct.unpack(
                "<i",
                self._recv_exact(4)
            )[0]

        if response == self.PATH_UPLOAD_OK:
            return response

        if response == self.PATH_COUNT_ERROR:
            raise RuntimeError(
                "RHO odmitlo PCPATH: PATH_COUNT mimo 1..16"
            )

        if response == self.PATH_SPEED_ERROR:
            raise RuntimeError(
                "RHO odmitlo PCPATH: PATH_SPEED <= 0"
            )

        raise RuntimeError(
            f"Neznama odpoved z CMD20: {response}"
        )


    def start_path(self):
        """CMD21 - spusti drive nahrany PCPATH buffer."""
        with self._lock:
            self._send_cmd(21)
            response = struct.unpack(
                "<i",
                self._recv_exact(4)
            )[0]

        if response == self.START_OK:
            return True

        if response == self.START_MANUAL:
            raise RuntimeError("Robot neni v AUTO")

        if response == self.START_BUSY:
            raise RuntimeError("Predchozi pohyb jeste probiha")

        if response == self.START_RC_ERROR:
            state, error, procstatus = self.get_move_state()
            raise RuntimeError(
                f"PCPATH RC error: {error} "
                f"(ProcStatus={procstatus})"
            )

        if response == self.PATH_COUNT_ERROR:
            raise RuntimeError(
                "PCPATH nelze spustit: PATH_COUNT mimo 1..16"
            )

        if response == self.PATH_SPEED_ERROR:
            raise RuntimeError(
                "PCPATH nelze spustit: PATH_SPEED <= 0"
            )

        raise RuntimeError(
            f"Neznama odpoved z CMD21: {response}"
        )


    def move_path(
        self,
        points,
        speed_mm_s=5.0,
        timeout=30.0,
        validate=True
    ):
        """
        Blokujici PCPATH:
          check -> CMD20 upload -> CMD21 start -> wait_move
        """
        self.upload_path(
            points,
            speed_mm_s=speed_mm_s,
            validate=validate
        )
        self.start_path()
        return self.wait_move(timeout=timeout)



    # =========================================================
    # STREAM V1 - TWO-SLOT PING-PONG (CMD22/23/24)
    # Simulator prototype.
    # =========================================================

    def upload_stream_slot(
        self,
        slot,
        sequence,
        points,
        speed_mm_s,
        final=False,
        validate_points=True
    ):
        slot = int(slot)
        sequence = int(sequence)
        speed_mm_s = float(speed_mm_s)

        if slot not in (0, 1):
            raise ValueError("STREAM slot musi byt 0 (A) nebo 1 (B)")
        if sequence < 0:
            raise ValueError("STREAM sequence musi byt >= 0")
        if speed_mm_s <= 0:
            raise ValueError("STREAM speed musi byt > 0 mm/s")

        normalized = [
            self._normalize_path_point(point)
            for point in points
        ]

        if not 1 <= len(normalized) <= self.PATH_MAX_POINTS:
            raise ValueError("STREAM buffer podporuje 1..16 bodu")

        if validate_points:
            # Endpoint validation only. Full continuous-path validation for
            # an arbitrarily long stream will be added before real RHO4 use.
            for point in normalized:
                self.check_target(*point)

        padded = list(normalized)
        while len(padded) < self.PATH_MAX_POINTS:
            padded.append((0.0, 0.0, 0.0, 0.0))

        flat = []
        for point in padded:
            flat.extend(point)

        payload = struct.pack(
            "<iiiiif64f",
            22,
            slot,
            sequence,
            len(normalized),
            int(bool(final)),
            speed_mm_s,
            *flat
        )

        with self._lock:
            if self.sock is None:
                raise ConnectionError("Nejsi pripojen k RHO")

            self.sock.sendall(payload)
            response = struct.unpack(
                "<i",
                self._recv_exact(4)
            )[0]

        if response == self.STREAM_UPLOAD_OK:
            return True

        if response == self.STREAM_SLOT_ERROR:
            raise RuntimeError("STREAM: neplatny slot")

        if response == self.STREAM_SLOT_BUSY:
            raise RuntimeError(
                f"STREAM: slot {'AB'[slot]} jeste neni volny"
            )

        if response == self.PATH_COUNT_ERROR:
            raise RuntimeError("STREAM: count mimo 1..16")

        if response == self.PATH_SPEED_ERROR:
            raise RuntimeError("STREAM: speed <= 0")

        if response == self.START_RC_ERROR:
            raise RuntimeError("STREAM: bod mimo simulator workspace")

        raise RuntimeError(
            f"STREAM CMD22 neznamy response={response}"
        )


    def start_stream(self):
        with self._lock:
            self._send_cmd(23)
            response = struct.unpack(
                "<i",
                self._recv_exact(4)
            )[0]

        if response == self.START_OK:
            return True

        if response == self.START_MANUAL:
            raise RuntimeError("STREAM: robot neni v AUTO")

        if response == self.START_BUSY:
            raise RuntimeError("STREAM: pohyb uz probiha")

        if response == self.START_RC_ERROR:
            state, error, procstatus = self.get_move_state()
            raise RuntimeError(
                f"STREAM RC error: {error} "
                f"(ProcStatus={procstatus})"
            )

        if response == self.STREAM_NOT_READY:
            raise RuntimeError(
                "STREAM: pred START musi byt pripraven seq0 "
                "a u vicebufferove drahy take seq1"
            )

        raise RuntimeError(
            f"STREAM CMD23 neznamy response={response}"
        )


    def get_stream_status(self):
        with self._lock:
            self._send_cmd(24)
            data = self._recv_exact(32)
            (
                running,
                active_slot,
                active_seq,
                completed_seq,
                occupied_mask,
                free_mask,
                underruns,
                active_segment,
            ) = struct.unpack("<8i", data)

        return {
            "running": bool(running),
            "active_slot": active_slot,
            "active_seq": active_seq,
            "completed_seq": completed_seq,
            "occupied_mask": occupied_mask,
            "free_mask": free_mask,
            "underruns": underruns,
            "active_segment": active_segment,
        }


    def stream_blocks(
        self,
        blocks,
        timeout=120.0,
        poll_interval=0.01,
        validate_points=True
    ):
        """
        Execute an ordered list of stream blocks by ping-ponging slots A/B.

        block format:
          {
            "points": [(x,y,z,r), ...],   # 1..16
            "speed_mm_s": 20.0
          }

        First two blocks are preloaded. While one slot is ACTIVE and the
        other is READY, Python refills the just-freed slot with the next
        sequence.
        """
        if not blocks:
            raise ValueError("STREAM blocks nesmi byt prazdne")

        normalized_blocks = []

        for index, block in enumerate(blocks):
            points = [
                self._normalize_path_point(point)
                for point in block["points"]
            ]
            speed = float(block["speed_mm_s"])

            if not 1 <= len(points) <= self.PATH_MAX_POINTS:
                raise ValueError(
                    f"STREAM block {index}: musi mit 1..16 bodu"
                )
            if speed <= 0:
                raise ValueError(
                    f"STREAM block {index}: speed musi byt > 0"
                )

            normalized_blocks.append({
                "points": points,
                "speed_mm_s": speed,
            })

        # Initial preload.
        self.upload_stream_slot(
            0,
            0,
            normalized_blocks[0]["points"],
            normalized_blocks[0]["speed_mm_s"],
            final=(len(normalized_blocks) == 1),
            validate_points=validate_points,
        )

        next_to_upload = 1

        if len(normalized_blocks) >= 2:
            self.upload_stream_slot(
                1,
                1,
                normalized_blocks[1]["points"],
                normalized_blocks[1]["speed_mm_s"],
                final=(len(normalized_blocks) == 2),
                validate_points=validate_points,
            )
            next_to_upload = 2

        self.start_stream()

        started = time.monotonic()
        last_print = None

        while True:
            state, error, procstatus = self.get_move_state()
            status = self.get_stream_status()

            display = (
                state,
                error,
                procstatus,
                status["active_slot"],
                status["active_seq"],
                status["completed_seq"],
                status["free_mask"],
                status["underruns"],
            )

            if display != last_print:
                slot_name = (
                    "AB"[status["active_slot"]]
                    if status["active_slot"] in (0, 1)
                    else "-"
                )
                print(
                    "STREAM "
                    f"state={state} err={error} proc={procstatus} "
                    f"slot={slot_name} "
                    f"seq={status['active_seq']} "
                    f"done={status['completed_seq']} "
                    f"free=0b{status['free_mask']:02b} "
                    f"underrun={status['underruns']}"
                )
                last_print = display

            if state == self.MOVE_ERROR:
                raise RuntimeError(
                    f"STREAM skoncil chybou: {error}"
                )

            if state == self.MOVE_STOPPED:
                return False

            # Refill any newly freed slot. There can be at most one useful
            # refill at a time in a two-slot ping-pong scheme.
            if next_to_upload < len(normalized_blocks):
                free_mask = status["free_mask"]

                free_slot = None
                if free_mask & 0b01:
                    free_slot = 0
                elif free_mask & 0b10:
                    free_slot = 1

                if free_slot is not None:
                    block = normalized_blocks[next_to_upload]
                    final = (
                        next_to_upload
                        == len(normalized_blocks) - 1
                    )

                    self.upload_stream_slot(
                        free_slot,
                        next_to_upload,
                        block["points"],
                        block["speed_mm_s"],
                        final=final,
                        validate_points=validate_points,
                    )

                    print(
                        f"  refill {'AB'[free_slot]} "
                        f"<- seq {next_to_upload} "
                        f"({len(block['points'])} bodu)"
                    )

                    next_to_upload += 1
                    # Fetch a fresh status before another upload.
                    continue

            if (
                state == self.MOVE_DONE
                and procstatus == -1
                and not status["running"]
            ):
                if status["underruns"] != 0:
                    raise RuntimeError(
                        f"STREAM skoncil s underrun="
                        f"{status['underruns']}"
                    )
                return True

            if time.monotonic() - started > timeout:
                raise TimeoutError(
                    "STREAM nedokoncil drahu v casovem limitu"
                )

            time.sleep(max(0.001, float(poll_interval)))


    # =========================================================
    # HIGH LEVEL LINEAR MOVE
    # =========================================================
    
    def move_linear(self,x,y,z,r,speed_mm_s=10.0,timeout=30.0):
        self.start_linear(x,y,z,r,speed_mm_s=speed_mm_s)
        return self.wait_move(timeout=timeout)

    def move_circular(self,mid_x, mid_y, mid_z, mid_r,end_x, end_y, end_z, end_r,speed_mm_s=10.0,timeout=30.0):
        self.start_circular(mid_x, mid_y, mid_z, mid_r,end_x, end_y, end_z, end_r,speed_mm_s)
        return self.wait_move(timeout=timeout)


        


    # =========================================================
    # HIGH LEVEL PTP MOVE
    # =========================================================
    def move_ptp(self,x,y,z,r,speed=2.0,timeout=30.0):
        # blokujici ptp pohyb
        self.start_ptp(x,y,z,r,speed=speed)
        return self.wait_move(timeout=timeout)

    def stop(self):
        # zastavi PCMOVE
        with self._lock:
            self._send_cmd(15)
            data = self._recv_exact(4)
            response = struct.unpack("<i", data)[0]
        if response != 15:
            raise RuntimeError(f"Chyba pri STOP: {response}")
        return True


    def jog_start(self,axis,direction,speed=0.5, distance=50.0):
        axis = axis.upper()

        if axis not in ("X","Y","Z","R"):
            raise ValueError("Neplatna osa. Pouzij X,Y,Z,R")
        if direction not in (-1,1):
            raise ValueError("direction musi by +1 nebo -1")

        x,y,z,r = self.get_position()
        if axis == "X":
            x += direction*distance
            x = max(self.X_MIN,min(self.X_MAX,x))
        elif axis == "Y":
            y += direction*distance
            y = max(self.Y_MIN,min(self.Y_MAX,y))
        elif axis == "Z":
            z += direction*distance
            z = max(self.Z_MIN,min(self.Z_MAX,z))
        elif axis == "R":
            r += direction*distance
            r = max(self.R_MIN,min(self.R_MAX,r))

        self.start_ptp(x,y,z,r,speed=speed)
        return True

    def jog_stop(self):
        return self.stop()

