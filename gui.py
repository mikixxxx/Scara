import tkinter as tk
from tkinter import ttk, messagebox
import threading
import queue
import time
import tkinter.font as font

from rho4 import Rho4


class RobotGUI:

    def __init__(self, root):

        self.root = root
        self.root.title("Bosch Rexroth RHO4 - Robot Control")
        self.root.geometry("500x650")
        self.root.resizable(False,False)

        # ==================================================
        # ROBOT
        # ==================================================

        self.rho = Rho4()

        self.connected = False
        self.jog_active = False

        # posledni data z robota
        self.robot_data = None

        # ==================================================
        # COMMAND QUEUE
        # ==================================================

        self.command_queue = queue.PriorityQueue()

        self.worker_running = True

        # ==================================================
        # CONNECTION FRAME
        # ==================================================

        connection_frame = ttk.LabelFrame(
            root,
            text="Connection",
            padding=10
        )

        connection_frame.pack(
            fill="x",
            padx=10,
            pady=10
        )

        self.connect_button = ttk.Button(
            connection_frame,
            text="CONNECT",
            command=self.toggle_connection
        )

        self.connect_button.pack(
            side="left",
            padx=5
        )

        self.connection_label = ttk.Label(
            connection_frame,
            text="Disconnected"
        )

        self.connection_label.pack(
            side="left",
            padx=20
        )

        # ==================================================
        # POSITION
        # ==================================================

        position_frame = ttk.LabelFrame(
            root,
            text="Robot position",
            padding=10
        )

        position_frame.pack(
            fill="x",
            padx=10,
            pady=5
        )

        self.position_vars = {}

        for i, axis in enumerate(
            ("X", "Y", "Z", "R")
        ):

            ttk.Label(
                position_frame,
                text=axis + ":"
            ).grid(
                row=0,
                column=i * 2,
                padx=(10, 2)
            )

            var = tk.StringVar(
                value="---"
            )

            self.position_vars[axis] = var

            ttk.Label(
                position_frame,
                textvariable=var,
                width=10
            ).grid(
                row=0,
                column=i * 2 + 1,
                padx=(0, 10)
            )

        # ==================================================
        # STATUS
        # ==================================================

        status_frame = ttk.LabelFrame(
            root,
            text="Status",
            padding=10
        )

        status_frame.pack(
            fill="x",
            padx=10,
            pady=5
        )

        self.auto_var = tk.StringVar(
            value="AUTO: ---"
        )

        self.alarm_var = tk.StringVar(
            value="ALARM: ---"
        )

        self.move_var = tk.StringVar(
            value="MOVE: ---"
        )

        ttk.Label(
            status_frame,
            textvariable=self.auto_var
        ).pack(
            side="left",
            padx=20
        )

        ttk.Label(
            status_frame,
            textvariable=self.alarm_var
        ).pack(
            side="left",
            padx=20
        )

        ttk.Label(
            status_frame,
            textvariable=self.move_var
        ).pack(
            side="left",
            padx=20
        )

        # ==================================================
        # SPEED
        # ==================================================

        speed_frame = ttk.LabelFrame(
            root,
            text="JOG speed",
            padding=10
        )

        speed_frame.pack(
            fill="x",
            padx=10,
            pady=5
        )

        self.speed_var = tk.DoubleVar(
            value=0.5
        )

        self.speed_scale = ttk.Scale(
            speed_frame,
            from_=0.1,
            to=5.0,
            variable=self.speed_var,
            orient="horizontal",
            length=350,
            command=self.speed_changed
        )

        self.speed_scale.pack(
            side="left",
            padx=10
        )

        self.speed_label = ttk.Label(
            speed_frame,
            text="0.50 %",
            width=10
        )

        self.speed_label.pack(
            side="left",
            padx=10
        )

        # ==================================================
        # PTP TARGET
        # ==================================================

        target_frame = ttk.LabelFrame(root,text="PTP target",padding=10)
        target_frame.pack(fill="x",padx=10,pady=5)
        self.target_vars = {}
        for i, axis in enumerate(("X","Y","Z","R")):
            ttk.Label(target_frame,text=axis + ":").grid(row=0,column=i*2,padx=(5,2))
            var = tk.StringVar(value="0.000")
            self.target_vars[axis] = var
            ttk.Entry(target_frame,textvariable=var,width=9).grid(row=0,column=i*2+1,padx=(0,8))

        self.current_button = ttk.Button(target_frame,text="CURRENT",command=self.target_current)
        self.current_button.grid(row=1,column=1,columnspan=2,pady=(10,0))
        self.move_button = ttk.Button(target_frame,text="MOVE",command=self.target_move)
        self.move_button.grid(row=1,column=5,columnspan=2,pady=(10, 0))
        # ==================================================
        # JOG
        # ==================================================

        jog_frame = ttk.LabelFrame(
            root,
            text="JOG",
            padding=15
        )

        jog_frame.pack(
            fill="both",
            expand=True,
            padx=10,
            pady=5
        )

        # XY ------------------------------------------------

        self.make_jog_button(
            jog_frame,
            "Y+",
            "Y",
            +1,
            1,
            2
        )

        self.make_jog_button(
            jog_frame,
            "X-",
            "X",
            -1,
            2,
            1
        )

        self.make_jog_button(
            jog_frame,
            "X+",
            "X",
            +1,
            2,
            3
        )

        self.make_jog_button(
            jog_frame,
            "Y-",
            "Y",
            -1,
            3,
            2
        )

        # Z -------------------------------------------------

        self.make_jog_button(
            jog_frame,
            "Z+",
            "Z",
            +1,
            1,
            5
        )

        self.make_jog_button(
            jog_frame,
            "Z-",
            "Z",
            -1,
            2,
            5
        )

        # R -------------------------------------------------

        self.make_jog_button(
            jog_frame,
            "R+",
            "R",
            +1,
            1,
            7
        )

        self.make_jog_button(
            jog_frame,
            "R-",
            "R",
            -1,
            2,
            7
        )

        # STOP ----------------------------------------------

        self.stop_button = tk.Button(
            jog_frame,
            text="STOP",
            bg="red",
            font=font.Font(family="Helvetica", size=12, weight="bold"),
            width=24,
            height=2,
            command=self.software_stop
        )

        self.stop_button.grid(
            row=4,
            column=2,
            columnspan=6,
            pady=20
        )

        # ==================================================
        # START WORKER
        # ==================================================

        self.worker_thread = threading.Thread(
            target=self.robot_worker,
            daemon=True
        )

        self.worker_thread.start()

        # ==================================================
        # GUI UPDATE
        # ==================================================

        self.root.after(
            200,
            self.update_gui
        )

        # ==================================================
        # WINDOW CLOSE
        # ==================================================

        self.root.protocol(
            "WM_DELETE_WINDOW",
            self.on_close
        )

    # ======================================================
    # JOG BUTTON CREATION
    # ======================================================

    def make_jog_button(
        self,
        parent,
        text,
        axis,
        direction,
        row,
        column
    ):

        button = ttk.Button(
            parent,
            text=text,
            width=8
        )

        button.grid(
            row=row,
            column=column,
            padx=8,
            pady=8
        )

        button.bind(
            "<ButtonPress-1>",
            lambda event:
            self.jog_press(
                axis,
                direction
            )
        )

        button.bind(
            "<ButtonRelease-1>",
            lambda event:
            self.jog_release()
        )

        return button

    # ======================================================
    # CONNECTION
    # ======================================================

    def toggle_connection(self):

        if self.connected:
            self.disconnect()
        else:
            self.connect()

    def connect(self):

        try:

            self.rho.connect()

            if not self.rho.ping():

                raise RuntimeError(
                    "RHO neodpovedelo na PING"
                )

            self.connected = True

            self.connection_label.config(
                text="Connected"
            )

            self.connect_button.config(
                text="DISCONNECT"
            )

        except Exception as e:

            self.connected = False

            messagebox.showerror(
                "Connection error",
                str(e)
            )

    def disconnect(self):

        # pokud robot jede JOG, nejdrive STOP
        if self.jog_active:

            self.jog_active = False

            self.clear_pending_jog()

            self.command_queue.put(
                (
                    0,
                    time.monotonic_ns(),
                    "STOP",
                    None
                )
            )

            # samotny disconnect nechame workeru
            self.command_queue.put(
                (
                    20,
                    time.monotonic_ns(),
                    "DISCONNECT",
                    None
                )
            )

        else:

            self.command_queue.put(
                (
                    20,
                    time.monotonic_ns(),
                    "DISCONNECT",
                    None
                )
            )

        self.connected = False

        self.connection_label.config(
            text="Disconnected"
        )

        self.connect_button.config(
            text="CONNECT"
        )

        self.robot_data = None

        self.clear_display()

    # ======================================================
    # SPEED
    # ======================================================

    def speed_changed(
        self,
        value
    ):

        try:

            value = float(value)

        except ValueError:

            return

        self.speed_label.config(
            text=f"{value:.2f} %"
        )

    # ======================================================
    # JOG
    # ======================================================

    def jog_press(
        self,
        axis,
        direction
    ):

        if not self.connected:
            return

        # uz jeden JOG probiha
        if self.jog_active:
            return

        self.jog_active = True

        speed = self.speed_var.get()

        # priority 10 = normalni prikaz
        self.command_queue.put(
            (
                10,
                time.monotonic_ns(),
                "JOG",
                (
                    axis,
                    direction,
                    speed
                )
            )
        )

    def jog_release(self):

        if not self.connected:
            return

        if not self.jog_active:
            return

        self.jog_active = False

        # odstranime JOG, ktery jeste ceka ve fronte
        self.clear_pending_jog()

        # priority 0 = STOP
        self.command_queue.put(
            (
                0,
                time.monotonic_ns(),
                "STOP",
                None
            )
        )

    # ======================================================
    # SOFTWARE STOP
    # ======================================================

    def software_stop(self):

        if not self.connected:
            return

        self.jog_active = False

        # zadny cekajici JOG se po STOP nesmi spustit
        self.clear_pending_jog()

        self.command_queue.put(
            (
                0,
                time.monotonic_ns(),
                "STOP",
                None
            )
        )

    # ======================================================
    # CLEAR PENDING JOG
    # ======================================================

    def clear_pending_jog(self):

        saved = []

        while True:

            try:

                item = \
                    self.command_queue.get_nowait()

            except queue.Empty:

                break

            priority, sequence, command, data = item

            self.command_queue.task_done()

            if command != "JOG":

                saved.append(item)

        for item in saved:

            self.command_queue.put(
                item
            )

    # ======================================================
    # ROBOT WORKER
    # ======================================================

    def robot_worker(self):

        while self.worker_running:

            # ==============================================
            # COMMAND QUEUE
            # ==============================================

            try:

                (
                    priority,
                    sequence,
                    command,
                    data
                ) = self.command_queue.get_nowait()

            except queue.Empty:

                command = None
                data = None

            if command is not None:

                try:

                    # --------------------------------------
                    # JOG
                    # --------------------------------------

                    if command == "JOG":
                        if data is None:
                            raise RuntimeError("JOG command nema data")

                        axis, direction, speed = data

                        self.rho.jog_start(
                            axis,
                            direction,
                            speed=speed,
                            distance=50.0
                        )

                    elif command == "MOVE":
                        if data is None:
                            raise RuntimeError("MOVE nema data")
                        x,y,z,r,speed = data
                        self.rho.start_ptp(x,y,z,r,speed=speed)

                    # --------------------------------------
                    # STOP
                    # --------------------------------------

                    elif command == "STOP":

                        self.rho.jog_stop()

                    # --------------------------------------
                    # DISCONNECT
                    # --------------------------------------

                    elif command == "DISCONNECT":

                        self.rho.disconnect()

                except Exception as e:

                    error_message = str(e)

                    self.root.after(
                        0,
                        lambda msg=error_message:
                        messagebox.showerror(
                            "Robot error",
                            msg
                        )
                    )

                finally:

                    self.command_queue.task_done()

            # ==============================================
            # READ ROBOT
            # ==============================================

            if self.connected:

                try:

                    x, y, z, r = \
                        self.rho.get_position()

                    status = \
                        self.rho.get_status()

                    (
                        move_state,
                        error,
                        proc
                    ) = self.rho.get_move_state()

                    self.robot_data = (
                        x,
                        y,
                        z,
                        r,
                        status,
                        move_state,
                        error,
                        proc
                    )

                except Exception:
                    # zatim pouze ignorujeme kratkodoby
                    # communication error
                    pass

            # 10 Hz worker loop
            time.sleep(
                0.10
            )

    # ======================================================
    # GUI UPDATE
    # ======================================================

    def update_gui(self):

        data = self.robot_data

        if data is not None:

            (
                x,
                y,
                z,
                r,
                status,
                move_state,
                error,
                proc
            ) = data

            self.update_display(
                x,
                y,
                z,
                r,
                status,
                move_state,
                error
            )

        self.root.after(
            200,
            self.update_gui
        )

    # ======================================================
    # DISPLAY
    # ======================================================

    def update_display(
        self,
        x,
        y,
        z,
        r,
        status,
        move_state,
        error
    ):

        self.position_vars["X"].set(
            f"{x:.3f}"
        )

        self.position_vars["Y"].set(
            f"{y:.3f}"
        )

        self.position_vars["Z"].set(
            f"{z:.3f}"
        )

        self.position_vars["R"].set(
            f"{r:.3f}"
        )

        # AUTO ---------------------------------------------

        if status["automatic"]:

            self.auto_var.set(
                "AUTO: YES"
            )

        else:

            self.auto_var.set(
                "AUTO: NO"
            )

        # ALARM --------------------------------------------

        if status["alarm"]:

            self.alarm_var.set(
                "ALARM: YES"
            )

        else:

            self.alarm_var.set(
                "ALARM: NO"
            )

        # MOVE ---------------------------------------------

        move_names = {

            0: "IDLE",
            1: "RUNNING",
            2: "DONE",
            3: "ERROR",
            4: "STOPPED"

        }

        name = move_names.get(
            move_state,
            str(move_state)
        )

        if move_state == 3:

            self.move_var.set(
                f"MOVE: ERROR {error}"
            )

        else:

            self.move_var.set(
                f"MOVE: {name}"
            )

    # ======================================================
    # CLEAR DISPLAY
    # ======================================================

    def clear_display(self):

        for axis in (
            "X",
            "Y",
            "Z",
            "R"
        ):

            self.position_vars[axis].set(
                "---"
            )

        self.auto_var.set(
            "AUTO: ---"
        )

        self.alarm_var.set(
            "ALARM: ---"
        )

        self.move_var.set(
            "MOVE: ---"
        )

    # ======================================================
    # CLOSE
    # ======================================================

    def on_close(self):

        # Pokud drzime JOG, pokusime se robot zastavit
        if self.connected:

            try:

                self.rho.jog_stop()

            except Exception:

                pass

        self.jog_active = False
        self.connected = False
        self.worker_running = False

        try:

            self.rho.disconnect()

        except Exception:

            pass

        self.root.destroy()

    def target_current(self):
        if self.robot_data is None:
            return

        x, y, z, r, status, move_state, error, proc = self.robot_data

        self.target_vars["X"].set(f"{x:.3f}")
        self.target_vars["Y"].set(f"{y:.3f}")
        self.target_vars["Z"].set(f"{z:.3f}")
        self.target_vars["R"].set(f"{r:.3f}")
    def target_move(self):

        if not self.connected:
            return

        try:

            x = float(
                self.target_vars["X"].get()
                .replace(",", ".")
            )

            y = float(
                self.target_vars["Y"].get()
                .replace(",", ".")
            )

            z = float(
                self.target_vars["Z"].get()
                .replace(",", ".")
            )

            r = float(
                self.target_vars["R"].get()
                .replace(",", ".")
            )

            speed = self.speed_var.get()

        except ValueError:

            messagebox.showerror(
                "PTP MOVE",
                "X, Y, Z a R musi byt cisla."
            )

            return

        # Zadny stary cekajici JOG
        self.clear_pending_jog()

        self.command_queue.put(
            (
                10,
                time.monotonic_ns(),
                "MOVE",
                (
                    x,
                    y,
                    z,
                    r,
                    speed
                )
            )
        )

# ==========================================================
# MAIN
# ==========================================================

if __name__ == "__main__":

    root = tk.Tk()

    app = RobotGUI(
        root
    )

    root.mainloop()