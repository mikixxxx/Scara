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
        self._lock = threading.Lock()


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
                self.disconnect()
                raise ConnectionError(
                    "RHO ukoncilo TCP spojeni"
                )

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
        # ceka na ukonceni aktualniho PTP pohybu
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
                raise RuntimeError(f"PCMOVE skoncil RC chybou: {error}")
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
        current_a1, current_a2, _, _ = self.get_joint_position()

        def distance(solution):
            da1 = self._angle_delta_deg(solution["a1"], current_a1)
            da2 = self._angle_delta_deg(solution["a2"], current_a2)
            return da1*da1 + da2*da2

        return min(valid, key=distance)


    def start_linear(self, x,y,z,r,speed_mm_s=10.0):
        self.check_linear_path(x,y,z,r,step_mm=2.0)

        if speed_mm_s <= 0:
            raise ValueError("LINEAR speed musi byt > 0 mm/s")

        with self._lock:
            #CMD16
            if self.sock is None:
                return False
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
            assert self.sock is not None
            self.sock.sendall(struct.pack("<i",17))
            data = self._recv_exact(16)
            return struct.unpack("<ffff",data)

    def check_linear_path(self,x,y,z,r,step_mm=2.0, joint_step_limit=10.0):
        if step_mm <= 0:
            raise ValueError("step_mm musi byt > 0")
        sx,sy,sz,sr = self.get_position()
        ca1,ca2,ca3,ca4 = self.get_joint_position()

        dx = float(x) - sx
        dy = float(y) - sy
        dz = float(z) - sz
        dr = self._angle_delta_deg(float(r),sr)

        length = math.sqrt(dx*dx + dy*dy + dz*dz)
        samples = max(1, int(math.ceil(length/step_mm)))
        previous = {"a1": ca1, "a2": ca2, "a3": ca3, "a4": ca4,}

        path = []
        for i in range(1, samples +1):
            t = i/samples
            px = sx + dx *t
            py = sy + dy * t
            pz = sz + dz * t
            pr = sr + dr * t
            solutions = self.check_target(px,py,pz,pr)

            def joint_distance(sol):
                da1 = self._angle_delta_deg(
                    sol["a1"],
                    previous["a1"]
                )

                da2 = self._angle_delta_deg(
                    sol["a2"],
                    previous["a2"]
                )

                da4 = self._angle_delta_deg(
                    sol["a4"],
                    previous["a4"]
                )
                return (da1*da1 + da2*da2 + da4*da4)

            selected = min(solutions, key=joint_distance)
            da1 = abs(self._angle_delta_deg(selected["a1"],previous["a1"]))
            da2 = abs(self._angle_delta_deg(selected["a2"],previous["a2"]))
            da4 = abs(self._angle_delta_deg(selected["a4"],previous["a4"]))

            if max(da1,da2,da4) > joint_step_limit:
                            raise ValueError(
                                "LINEAR path: prilis velky skok jointu "
                                f"v {t*100:.1f}% drahy: "
                                f"dA1={da1:.2f} "
                                f"dA2={da2:.2f} "
                                f"dA4={da4:.2f}")
            point = {
                "t": t,
                "x": px,
                "y": py,
                "z": pz,
                "r": pr,
                "a1": selected["a1"],
                "a2": selected["a2"],
                "a3": selected["a3"],
                "a4": selected["a4"],}
            
            path.append(point)
            previous = selected
        return path
    
          
    # =========================================================
    # HIGH LEVEL LINEAR MOVE
    # =========================================================
    
    def move_linear(self,x,y,z,r,speed_mm_s=10.0,timeout=30.0):
        self.start_linear(x,y,z,r,speed_mm_s=speed_mm_s)
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



    # def move_ptp(self, x, y, z, r, speed=2.0, timeout=30.0):
    #     # -----------------------------------------------------
    #     # Nastavime rychlost
    #     # -----------------------------------------------------

    #     self.set_ptp_speed(speed)

    #     # -----------------------------------------------------
    #     # Posleme cilovou pozici
    #     # -----------------------------------------------------

    #     target = self.set_target(x, y, z, r)

    #     # -----------------------------------------------------
    #     # Spustime PCMOVE
    #     # -----------------------------------------------------

    #     response = self.start_pcmove()


    #     # MANUAL
    #     if response == self.START_MANUAL:
    #         raise RuntimeError(
    #             "Robot je v MANUAL rezimu"
    #         )


    #     # BUSY
    #     if response == self.START_BUSY:
    #         raise RuntimeError(
    #             "Robot uz provadi pohyb"
    #         )


    #     # PCMOVE uz je v RC ERROR
    #     if response == self.START_RC_ERROR:

    #         state, error, procstatus = (
    #             self.get_move_state()
    #         )

    #         raise RuntimeError(
    #             f"PCMOVE je v RC error: "
    #             f"{error} "
    #             f"(ProcStatus={procstatus})"
    #         )


    #     # Neznama odpoved
    #     if response != self.START_OK:
    #         raise RuntimeError(
    #             f"Chyba pri startu PCMOVE: "
    #             f"{response}"
    #         )


    #     # -----------------------------------------------------
    #     # Cekame na dokonceni
    #     # -----------------------------------------------------

    #     start_time = time.monotonic()
    #     last_status = None
    #     while True:

    #         state, error, procstatus = (self.get_move_state())
    #         current_status = (state, error, procstatus)
    #         if current_status != last_status:
    #             last_status = current_status
    #             print(
    #                 f"State={state}, "
    #                 f"Error={error}, "
    #                 f"ProcStatus={procstatus}")
    #             last_status = current_status

    #         # ---------------------------------------------
    #         # RC ERROR
    #         # ---------------------------------------------

    #         if state == self.MOVE_ERROR:

    #             raise RuntimeError(
    #                 f"PCMOVE skoncil RC chybou: "
    #                 f"{error}"
    #             )


    #         # ---------------------------------------------
    #         # HOTOVO
    #         #
    #         # Overene experimentem:
    #         #
    #         # State=2 ProcStatus=1
    #         #   jeste neni definitivni vysledek
    #         #
    #         # State=2 ProcStatus=-1
    #         #   normalni konec PCMOVE
    #         # ---------------------------------------------

    #         if (
    #             state == self.MOVE_DONE
    #             and procstatus == -1
    #         ):
    #             return True


    #         # ---------------------------------------------
    #         # TIMEOUT
    #         # ---------------------------------------------

    #         if (
    #             time.monotonic() - start_time
    #             > timeout
    #         ):
    #             raise TimeoutError(
    #                 "Robot nedokoncil pohyb "
    #                 "v casovem limitu"
    #             )


    #         time.sleep(0.05)