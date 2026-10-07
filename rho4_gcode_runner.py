import argparse
import math
import time

from gcode_runner_v9 import GCodeRunner as V9Runner


class GCodeRunner(V9Runner):
    """
    FINAL CAM + I/O + probe G-code runner for RHO4 HYRING.

    Motion backend:
      G1    -> native LINEAR primitive in HYRING
      G2/G3 -> native CIRCULAR primitive(s) in HYRING
      G0    -> standalone native LINEAR (CMD16), never blended with cutting path

    CAM-oriented additions:
      G20 / G21            inch / mm
      G90 / G91            absolute / incremental distance mode
      G90.1 / G91.1        absolute / incremental I/J arc-center mode
      G17                   XY plane
      G54                   work coordinates; current job start = G54 zero
      G92                   redefine program coordinate at current TCP
      G40                   cutter compensation OFF accepted
      G49                   tool-length compensation OFF accepted
      G80                   canned-cycle cancel accepted
      G94                   feed per minute
      G61 / G64             exact-stop / continuous path mode
      G4 P...               dwell, seconds
      modal G0/G1/G2/G3
      common N/O/S/T words

    Important safety choices:
      - Default V10 coordinate system is WORK coordinates:
            program X0 Y0 Z0 R0 == robot position/orientation at job start.
        This is much safer for normal CAM files than treating G90 X0 Y0 as
        robot machine zero.
      - Use --machine-coordinates only when machine-coordinate G90 is really
        intended.
      - G0 always flushes HYRING and runs as a separate LINEAR move.
      - Spindle/coolant/tool change are NOT connected by this runner.
        M3/M4/M5/M7/M8/M9 require --ignore-aux.
        M6 requires --ignore-tool-change.
      - Tool-length/cutter compensation ON, canned cycles, probing and
        machine-home commands are rejected instead of silently ignored.

    R word:
      The robot already uses R as TCP rotation. Therefore G2/G3 radius-word
      syntax using R is intentionally NOT supported. Use I/J arcs.
    """

    DISTANCE_WORDS = {"X", "Y", "Z", "I", "J"}

    def __init__(
        self,
        rho,
        default_feed=600.0,
        rapid_feed=1200.0,
        move_timeout=1200.0,
        verify_endpoint_mm=0.10,
        min_refill_window_s=0.50,
        refill_lookahead_blocks=11,
        native_arc_max_deg=180.0,
        start_in_work_coordinates=True,
        ignore_aux_mcodes=False,
        ignore_tool_change=False,
    ):
        super().__init__(
            rho,
            default_feed=default_feed,
            rapid_feed=rapid_feed,
            move_timeout=move_timeout,
            verify_endpoint_mm=verify_endpoint_mm,
            min_refill_window_s=min_refill_window_s,
            refill_lookahead_blocks=refill_lookahead_blocks,
            native_arc_max_deg=native_arc_max_deg,
        )

        self.start_in_work_coordinates = bool(start_in_work_coordinates)
        self.ignore_aux_mcodes = bool(ignore_aux_mcodes)
        self.ignore_tool_change = bool(ignore_tool_change)

        self.units_name = "mm"
        self.unit_scale = 1.0

        # Standard RS274 default: I/J incremental from arc START.
        self.arc_center_absolute = False

        # G64-like continuous mode is our normal HYRING behavior.
        self.exact_stop = False

        self.job_origin = None
        self.work_offset = {
            "X": 0.0,
            "Y": 0.0,
            "Z": 0.0,
            "R": 0.0,
        }
        self.work_coordinate_active = self.start_in_work_coordinates

        self.spindle_rpm = None
        self.selected_tool = None

        self.rapid_moves = 0
        self.dwell_count = 0
        self.dwell_total_s = 0.0
        self.aux_noop_count = 0
        self.toolchange_noop_count = 0

        self.probe_count = 0
        self.last_probe_machine = None

    # =========================================================
    # JOB / COORDINATE STATE
    # =========================================================

    def _reset_cam_state_after_sync(self):
        if self.position is None:
            self.sync_position()

        self.units_name = "mm"
        self.unit_scale = 1.0
        self.arc_center_absolute = False
        self.exact_stop = False

        self.job_origin = self.position.copy()

        if self.start_in_work_coordinates:
            self.work_offset = self.job_origin.copy()
            self.work_coordinate_active = True
        else:
            self.work_offset = {
                "X": 0.0,
                "Y": 0.0,
                "Z": 0.0,
                "R": 0.0,
            }
            self.work_coordinate_active = False

        self.spindle_rpm = None
        self.selected_tool = None

        self.rapid_moves = 0
        self.dwell_count = 0
        self.dwell_total_s = 0.0
        self.aux_noop_count = 0
        self.toolchange_noop_count = 0

    def _program_to_machine_axis(self, axis, value):
        value = float(value)
        if self.work_coordinate_active:
            return self.work_offset[axis] + value
        return value

    def _machine_to_program_axis(self, axis, value):
        value = float(value)
        if self.work_coordinate_active:
            return value - self.work_offset[axis]
        return value

    def _convert_values(self, words):
        """
        Convert program linear units to internal mm.

        X/Y/Z/I/J and F follow G20/G21.
        Robot R remains degrees and is never scaled.
        P (dwell) remains seconds.
        """
        values = {}

        for letter, raw in words:
            value = float(raw)

            if letter in self.DISTANCE_WORDS:
                value *= self.unit_scale

            elif letter == "F":
                value *= self.unit_scale

            values[letter] = value

        return values

    def _target_from_values(self, values):
        if self.position is None:
            self.sync_position()

        target = self.position.copy()

        for axis in ("X", "Y", "Z", "R"):
            if axis not in values:
                continue

            value = float(values[axis])

            if self.absolute:
                target[axis] = self._program_to_machine_axis(
                    axis, value
                )
            else:
                target[axis] += value

        return target

    # =========================================================
    # ARC GEOMETRY WITH G90.1 / G91.1 + WORK COORDINATES
    # =========================================================

    def _arc_geometry(self, values, motion_g):
        if self.position is None:
            self.sync_position()

        if motion_g not in (2, 3):
            raise ValueError(
                "Interni chyba: motion_g musi byt G2 nebo G3"
            )

        if "R" in values:
            raise ValueError(
                f"Radek {self.line_number}: u G2/G3 nepouzivej R. "
                "R je rotace robota; CAM musi generovat oblouk pomoci I/J."
            )

        if "I" not in values and "J" not in values:
            raise ValueError(
                f"Radek {self.line_number}: "
                f"G{motion_g} vyzaduje I a/nebo J"
            )

        sx = self.position["X"]
        sy = self.position["Y"]
        sz = self.position["Z"]
        sr = self.position["R"]

        # Native RHO circular is not a helix.
        if "Z" in values:
            target_z = (
                self._program_to_machine_axis("Z", values["Z"])
                if self.absolute
                else sz + float(values["Z"])
            )
            if abs(target_z - sz) > 1e-6:
                raise ValueError(
                    f"Radek {self.line_number}: "
                    "helical G2/G3 se zmenou Z zatim nepodporujeme"
                )

        ex = sx
        ey = sy

        if "X" in values:
            ex = (
                self._program_to_machine_axis("X", values["X"])
                if self.absolute
                else sx + float(values["X"])
            )

        if "Y" in values:
            ey = (
                self._program_to_machine_axis("Y", values["Y"])
                if self.absolute
                else sy + float(values["Y"])
            )

        if self.arc_center_absolute:
            cx = (
                self._program_to_machine_axis(
                    "X", values.get(
                        "I",
                        self._machine_to_program_axis("X", sx),
                    )
                )
            )
            cy = (
                self._program_to_machine_axis(
                    "Y", values.get(
                        "J",
                        self._machine_to_program_axis("Y", sy),
                    )
                )
            )
        else:
            # G91.1 / normal CAM convention:
            # I/J are offsets from the START even in G90.
            cx = sx + float(values.get("I", 0.0))
            cy = sy + float(values.get("J", 0.0))

        radius_start = math.hypot(sx - cx, sy - cy)
        radius_end = math.hypot(ex - cx, ey - cy)

        if radius_start < self.ARC_EPS:
            raise ValueError(
                f"Radek {self.line_number}: nulovy polomer G{motion_g}"
            )

        full_circle = (
            math.hypot(ex - sx, ey - sy)
            <= self.ARC_RADIUS_TOLERANCE_MM
        )

        if not full_circle:
            radius_error = abs(radius_start - radius_end)
            if radius_error > self.ARC_RADIUS_TOLERANCE_MM:
                raise ValueError(
                    f"Radek {self.line_number}: rozdil START/END radius "
                    f"{radius_start:.3f} vs {radius_end:.3f} mm"
                )

        a0 = math.atan2(sy - cy, sx - cx)
        a1 = math.atan2(ey - cy, ex - cx)

        if full_circle:
            sweep = (
                -2.0 * math.pi
                if motion_g == 2
                else 2.0 * math.pi
            )
        elif motion_g == 3:
            sweep = (a1 - a0) % (2.0 * math.pi)
        else:
            sweep = -((a0 - a1) % (2.0 * math.pi))

        if abs(sweep) < self.ARC_EPS:
            raise ValueError(
                f"Radek {self.line_number}: nulovy oblouk"
            )

        return {
            "sx": sx,
            "sy": sy,
            "sz": sz,
            "sr": sr,
            "ex": ex,
            "ey": ey,
            "cx": cx,
            "cy": cy,
            "radius": radius_start,
            "a0": a0,
            "sweep": sweep,
        }

    # =========================================================
    # G0 / DWELL
    # =========================================================

    def _execute_g0(self, values):
        """
        Conservative V10 rapid:
          - finish all queued cutting motion first
          - execute exactly one standalone checked RHO LINEAR move
          - never blend G0 into HYRING
        """
        if self._hybrid_pending():
            self.flush_hybrid("pred G0")

        target = self._target_from_values(values)

        if not any(
            axis in values for axis in ("X", "Y", "Z", "R")
        ):
            return

        current = self._current_tuple()
        end = self._dict_to_tuple(target)

        if self._point_distance(current, end) <= self.POINT_EPS:
            self.position = self._tuple_to_dict(end)
            return

        speed_mm_s = self.rapid_feed / 60.0

        print(
            f"[{self.line_number:04d}] G0 SAFE/STANDALONE "
            f"X={target['X']:.3f} Y={target['Y']:.3f} "
            f"Z={target['Z']:.3f} R={target['R']:.3f} "
            f"Frapid={self.rapid_feed:.1f} mm/min "
            f"({speed_mm_s:.3f} mm/s)"
        )

        ok = self.rho.move_linear(
            target["X"],
            target["Y"],
            target["Z"],
            target["R"],
            speed_mm_s=speed_mm_s,
            timeout=self.move_timeout,
        )

        if not ok:
            raise RuntimeError("G0 LINEAR byl zastaven")

        self.rapid_moves += 1
        self.sync_position()

    def _execute_dwell(self, values):
        if "P" not in values:
            raise ValueError(
                f"Radek {self.line_number}: G4 vyzaduje P v sekundach"
            )

        seconds = float(values["P"])
        if seconds < 0.0:
            raise ValueError("G4 P musi byt >= 0")

        if self._hybrid_pending():
            self.flush_hybrid("pred G4 dwell")

        print(
            f"[{self.line_number:04d}] G4 DWELL "
            f"P={seconds:.3f} s"
        )

        self.dwell_count += 1
        self.dwell_total_s += seconds

        deadline = time.monotonic() + seconds
        while True:
            if self._stop_event.is_set():
                raise RuntimeError("G4 dwell prerusen STOP")

            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break

            time.sleep(min(0.05, remaining))

    # =========================================================
    # CAM MODAL / AUXILIARY COMMANDS
    # =========================================================

    @staticmethod
    def _g_is(value, target, eps=1e-7):
        return abs(float(value) - float(target)) <= eps

    def _set_units(self, inches):
        if inches:
            self.units_name = "inch"
            self.unit_scale = 25.4
            print(
                f"[{self.line_number:04d}] G20 INCH "
                "(internally converted to mm)"
            )
        else:
            self.units_name = "mm"
            self.unit_scale = 1.0
            print(f"[{self.line_number:04d}] G21 MM")

    def _activate_g54(self):
        if self.job_origin is None:
            raise RuntimeError("G54: job origin neni inicializovan")

        self.work_offset = self.job_origin.copy()
        self.work_coordinate_active = True

        print(
            f"[{self.line_number:04d}] G54 WORK ZERO = JOB START "
            f"X={self.work_offset['X']:.3f} "
            f"Y={self.work_offset['Y']:.3f} "
            f"Z={self.work_offset['Z']:.3f} "
            f"R={self.work_offset['R']:.3f}"
        )


    # =========================================================
    # V11 SPINDLE + PROBE
    # =========================================================

    PROBE_MAX_SPEED_MM_S = 2.0
    PROBE_MAX_TRAVEL_MM = 25.0

    def _execute_probe(self, values):
        """
        First V11 probe implementation:
          G38.2 Z... F...

        Deliberately restricted to Z-only probing for the engraving use case.
        RHO4 performs MOVE LINEAR UNTIL fast input 611=1.
        """
        if self._hybrid_pending():
            self.flush_hybrid("pred G38.2 probe")

        if "Z" not in values:
            raise ValueError(
                f"Radek {self.line_number}: G38.2 zatim vyzaduje Z"
            )

        if any(axis in values for axis in ("X", "Y", "R")):
            raise ValueError(
                f"Radek {self.line_number}: V11 zatim povoluje G38.2 pouze v Z"
            )

        target = self._target_from_values(values)
        current = self._current_tuple()

        dz = float(target["Z"]) - float(current[2])
        travel = abs(dz)

        if travel < 1e-6:
            raise ValueError(
                f"Radek {self.line_number}: G38.2 ma nulovy Z pohyb"
            )

        if travel > self.PROBE_MAX_TRAVEL_MM:
            raise ValueError(
                f"Radek {self.line_number}: probe travel {travel:.3f} mm "
                f"> limit {self.PROBE_MAX_TRAVEL_MM:.1f} mm"
            )

        speed_mm_s = self.feed / 60.0

        if speed_mm_s > self.PROBE_MAX_SPEED_MM_S:
            raise ValueError(
                f"Radek {self.line_number}: probe speed "
                f"{speed_mm_s:.3f} mm/s je prilis vysoka; "
                f"maximum je {self.PROBE_MAX_SPEED_MM_S:.3f} mm/s"
            )

        print(
            f"[{self.line_number:04d}] G38.2 FAST PROBE Z "
            f"{current[2]:.3f} -> {target['Z']:.3f} "
            f"F={self.feed:.1f} mm/min "
            f"({speed_mm_s:.3f} mm/s)"
        )

        result = self.rho.probe_linear(
            target["X"],
            target["Y"],
            target["Z"],
            target["R"],
            speed_mm_s=speed_mm_s,
            timeout=self.move_timeout,
        )

        if not result or not result["hit"]:
            raise RuntimeError(
                f"Radek {self.line_number}: G38.2 probe bez triggeru"
            )

        self.last_probe_machine = tuple(result["measured"])
        self.probe_count += 1

        # The robot may stop slightly beyond the captured trigger position.
        # Keep the actual TCP position synchronized for subsequent motion.
        actual = self.sync_position()

        print(
            "  PROBE HIT @MPOS/WC: "
            f"X={self.last_probe_machine[0]:.4f} "
            f"Y={self.last_probe_machine[1]:.4f} "
            f"Z={self.last_probe_machine[2]:.4f} "
            f"R={self.last_probe_machine[3]:.4f}"
        )
        print(
            "  ACTUAL STOP: "
            f"X={actual['X']:.4f} "
            f"Y={actual['Y']:.4f} "
            f"Z={actual['Z']:.4f} "
            f"R={actual['R']:.4f}"
        )

        return result


    def _execute_g92(self, values):
        """
        Probe-aware G92.

        Immediately after a successful Z probe, G92 Z... references the
        captured trigger position (@MPOS/WC), not the slightly later actual
        stop position. All other G92 cases use the V10 work-offset behavior.
        """
        if (
            self.last_probe_machine is not None
            and "Z" in values
            and not any(a in values for a in ("X", "Y", "R"))
        ):
            value = float(values["Z"])

            self.work_offset["Z"] = (
                float(self.last_probe_machine[2]) - value
            )
            self.work_coordinate_active = True

            print(
                f"[{self.line_number:04d}] "
                "G92 Z FROM PROBE @MPOS -> "
                f"work_offset.Z={self.work_offset['Z']:.4f}"
            )

            self.last_probe_machine = None
            return

        if self.position is None:
            self.sync_position()

        axes = [
            axis
            for axis in ("X", "Y", "Z", "R")
            if axis in values
        ]

        if not axes:
            raise ValueError(
                f"Radek {self.line_number}: G92 vyzaduje X/Y/Z/R"
            )

        for axis in axes:
            self.work_offset[axis] = (
                self.position[axis] - float(values[axis])
            )

        self.work_coordinate_active = True

        print(
            f"[{self.line_number:04d}] G92 WORK OFFSET -> "
            f"X={self.work_offset['X']:.3f} "
            f"Y={self.work_offset['Y']:.3f} "
            f"Z={self.work_offset['Z']:.3f} "
            f"R={self.work_offset['R']:.3f}"
        )


    def _handle_aux_mcode(self, m):
        if m == 3:
            if self._hybrid_pending():
                self.flush_hybrid("pred M3 spindle ON")

            status = self.rho.set_spindle(True)
            print(
                f"[{self.line_number:04d}] M3 SPINDLE ON "
                f"(digital output ch{status['spindle_channel']})"
            )
            return

        if m == 5:
            if self._hybrid_pending():
                self.flush_hybrid("pred M5 spindle OFF")

            status = self.rho.set_spindle(False)
            print(
                f"[{self.line_number:04d}] M5 SPINDLE OFF "
                f"(digital output ch{status['spindle_channel']})"
            )
            return

        if m == 4:
            raise RuntimeError(
                f"Radek {self.line_number}: M4 reverse spindle "
                "zatim neni implementovan"
            )

        if m in (7, 8, 9):
            if not self.ignore_aux_mcodes:
                raise RuntimeError(
                    f"Radek {self.line_number}: M{m} je rozpoznan, "
                    "ale spindle/coolant neni pripojen k V10. "
                    "Pro motion-only test pouzij --ignore-aux."
                )

            self.aux_noop_count += 1
            print(
                f"[{self.line_number:04d}] M{m} AUX NO-OP "
                "(explicit --ignore-aux)"
            )
            return

        if m == 6:
            if not self.ignore_tool_change:
                raise RuntimeError(
                    f"Radek {self.line_number}: M6 tool change "
                    "neni implementovan. Pro motion-only test pouzij "
                    "--ignore-tool-change."
                )

            if self._hybrid_pending():
                self.flush_hybrid("pred M6 no-op")

            self.toolchange_noop_count += 1
            print(
                f"[{self.line_number:04d}] M6 TOOL CHANGE NO-OP "
                "(explicit --ignore-tool-change)"
            )
            return

        if m in (0, 1):
            raise RuntimeError(
                f"Radek {self.line_number}: M{m} program stop "
                "zatim neni implementovan"
            )

        raise ValueError(
            f"Radek {self.line_number}: nepodporovany M{m}"
        )

    # =========================================================
    # V10 LINE EXECUTION
    # =========================================================

    def execute_line(self, raw_line):
        line = self._strip_comments(raw_line)

        # Common program delimiters.
        if not line or line == "%":
            return True

        words = self._words(line)
        if not words:
            return True

        g_codes = [float(v) for l, v in words if l == "G"]
        m_codes = [int(v) for l, v in words if l == "M"]

        # -----------------------------
        # 1) Modal G-code groups that affect unit conversion.
        # -----------------------------
        has_g20 = any(self._g_is(g, 20) for g in g_codes)
        has_g21 = any(self._g_is(g, 21) for g in g_codes)

        if has_g20 and has_g21:
            raise ValueError(
                f"Radek {self.line_number}: G20 a G21 v jednom bloku"
            )

        if has_g20:
            self._set_units(True)
        elif has_g21:
            self._set_units(False)

        # Now distance/feed words can be converted correctly.
        values = self._convert_values(words)

        explicit_motion = None
        dwell_requested = False
        g92_requested = False
        probe_requested = False

        for g in g_codes:
            # motion group
            if any(self._g_is(g, code) for code in (0, 1, 2, 3)):
                code = int(round(g))
                if (
                    explicit_motion is not None
                    and explicit_motion != code
                ):
                    raise ValueError(
                        f"Radek {self.line_number}: "
                        "vice pohybovych G kodu"
                    )
                explicit_motion = code
                continue

            # already processed units
            if self._g_is(g, 20) or self._g_is(g, 21):
                continue

            if self._g_is(g, 4):
                dwell_requested = True

            elif self._g_is(g, 17):
                print(f"[{self.line_number:04d}] G17 XY PLANE")

            elif self._g_is(g, 18) or self._g_is(g, 19):
                raise ValueError(
                    f"Radek {self.line_number}: G{g:g} neni podporovano; "
                    "native CAM runner pouziva XY plane G17"
                )

            elif self._g_is(g, 40):
                print(
                    f"[{self.line_number:04d}] "
                    "G40 CUTTER COMP OFF"
                )

            elif self._g_is(g, 41) or self._g_is(g, 42):
                raise RuntimeError(
                    f"Radek {self.line_number}: G{g:g} cutter compensation "
                    "neni implementovana; kompenzaci musi udelat CAM"
                )

            elif self._g_is(g, 49):
                print(
                    f"[{self.line_number:04d}] "
                    "G49 TOOL LENGTH COMP OFF"
                )

            elif self._g_is(g, 43) or self._g_is(g, 44):
                raise RuntimeError(
                    f"Radek {self.line_number}: G{g:g} tool length "
                    "compensation neni implementovana"
                )

            elif self._g_is(g, 54):
                self._activate_g54()

            elif any(
                self._g_is(g, code)
                for code in (55, 56, 57, 58, 59)
            ):
                raise RuntimeError(
                    f"Radek {self.line_number}: G{g:g} work offset "
                    "neni nakonfigurovan; V10 zatim pouziva G54"
                )

            elif self._g_is(g, 53):
                raise RuntimeError(
                    f"Radek {self.line_number}: G53 machine-coordinate "
                    "override zatim nepodporujeme"
                )

            elif self._g_is(g, 80):
                print(
                    f"[{self.line_number:04d}] "
                    "G80 CANNED CYCLE CANCEL"
                )

            elif 81.0 <= g <= 89.0:
                raise RuntimeError(
                    f"Radek {self.line_number}: G{g:g} canned cycle "
                    "neni implementovan"
                )

            elif self._g_is(g, 90):
                self.absolute = True
                print(f"[{self.line_number:04d}] G90 ABSOLUTE")

            elif self._g_is(g, 91):
                self.absolute = False
                print(f"[{self.line_number:04d}] G91 RELATIVE")

            elif self._g_is(g, 90.1):
                self.arc_center_absolute = True
                print(
                    f"[{self.line_number:04d}] "
                    "G90.1 ARC CENTER ABSOLUTE"
                )

            elif self._g_is(g, 91.1):
                self.arc_center_absolute = False
                print(
                    f"[{self.line_number:04d}] "
                    "G91.1 ARC CENTER INCREMENTAL"
                )

            elif self._g_is(g, 92):
                g92_requested = True

            elif self._g_is(g, 94):
                print(
                    f"[{self.line_number:04d}] "
                    "G94 FEED PER MINUTE"
                )

            elif self._g_is(g, 93) or self._g_is(g, 95):
                raise RuntimeError(
                    f"Radek {self.line_number}: G{g:g} feed mode "
                    "neni podporovan; pouzij G94"
                )

            elif self._g_is(g, 61):
                if self._hybrid_pending():
                    self.flush_hybrid("pred G61 exact-stop mode")
                self.exact_stop = True
                print(
                    f"[{self.line_number:04d}] "
                    "G61 EXACT STOP MODE"
                )

            elif self._g_is(g, 64):
                self.exact_stop = False
                print(
                    f"[{self.line_number:04d}] "
                    "G64 CONTINUOUS PATH"
                )

            elif self._g_is(g, 28) or self._g_is(g, 30):
                raise RuntimeError(
                    f"Radek {self.line_number}: G{g:g} HOME/RETURN "
                    "je z bezpecnostnich duvodu blokovan"
                )

            elif self._g_is(g, 38.2):
                probe_requested = True

            elif 38.0 <= g < 39.0:
                raise RuntimeError(
                    f"Radek {self.line_number}: podporujeme zatim pouze G38.2"
                )

            else:
                raise ValueError(
                    f"Radek {self.line_number}: nepodporovany G{g:g}"
                )

        if explicit_motion is not None:
            self.motion_mode = explicit_motion

        # -----------------------------
        # 2) Common non-motion words.
        # -----------------------------
        if "S" in values:
            self.spindle_rpm = float(values["S"])
            print(
                f"[{self.line_number:04d}] "
                f"S={self.spindle_rpm:.0f} RPM requested"
            )

        if "T" in values:
            self.selected_tool = int(round(values["T"]))
            print(
                f"[{self.line_number:04d}] "
                f"T={self.selected_tool} selected"
            )

        if "F" in values:
            self._set_feed(values["F"])

        # -----------------------------
        # 3) Coordinate-system special blocks.
        # -----------------------------
        if g92_requested:
            if explicit_motion is not None or dwell_requested or probe_requested:
                raise ValueError(
                    f"Radek {self.line_number}: G92 nekombinuj "
                    "s pohybem/G4 v jednom bloku"
                )
            self._execute_g92(values)

        # -----------------------------
        # 4) M-codes.
        # -----------------------------
        for m in m_codes:
            if m in (2, 30):
                if self._hybrid_pending():
                    self.flush_hybrid(f"M{m}")
                print(f"[{self.line_number:04d}] M{m} END")
                return False

            self._handle_aux_mcode(m)

        # -----------------------------
        # 5) Fast probe.
        # -----------------------------
        if probe_requested:
            if explicit_motion is not None or dwell_requested:
                raise ValueError(
                    f"Radek {self.line_number}: G38.2 nekombinuj "
                    "s G0/G1/G2/G3/G4 v jednom bloku"
                )
            self._execute_probe(values)
            return True

        # -----------------------------
        # 6) Dwell.
        # -----------------------------
        if dwell_requested:
            if explicit_motion is not None:
                raise ValueError(
                    f"Radek {self.line_number}: G4 nekombinuj "
                    "s G0/G1/G2/G3 v jednom bloku"
                )
            self._execute_dwell(values)
            return True

        # -----------------------------
        # 7) Modal motion.
        # -----------------------------
        has_axis = any(
            axis in values for axis in ("X", "Y", "Z", "R")
        )
        has_arc_center = "I" in values or "J" in values
        should_move = has_axis or has_arc_center

        if should_move and not g92_requested:
            if self.motion_mode is None:
                raise ValueError(
                    f"Radek {self.line_number}: souradnice bez modalniho "
                    "G0/G1/G2/G3"
                )

            if self.motion_mode in (0, 1) and has_arc_center:
                raise ValueError(
                    f"Radek {self.line_number}: I/J pouze pro G2/G3"
                )

            if self.motion_mode == 0:
                self._execute_g0(values)

            elif self.motion_mode == 1:
                self._execute_g1(values)

            elif self.motion_mode in (2, 3):
                self._execute_arc(values, self.motion_mode)

            if self.exact_stop and self._hybrid_pending():
                self.flush_hybrid("G61 exact stop")

        elif "F" in values:
            print(
                f"[{self.line_number:04d}] "
                f"F={self.feed:.1f} mm/min internal"
            )

        return True

    # =========================================================
    # JOB
    # =========================================================

    def _summary(self):
        super()._summary()
        print(
            "V10 CAM summary: "
            f"G0={self.rapid_moves}, "
            f"dwells={self.dwell_count} "
            f"({self.dwell_total_s:.3f}s), "
            f"aux_noop={self.aux_noop_count}, "
            f"toolchange_noop={self.toolchange_noop_count}, "
            f"probes={self.probe_count}"
        )

    def run_lines(self, lines):
        if self.running:
            raise RuntimeError("G-code uz bezi")

        self.running = True
        self._stop_event.clear()
        self._pause_event.set()

        # V8 state unused by V10/V9 backend.
        self.pending_points.clear()
        self.pending_feed = None
        self.pending_start = None
        self.stream_blocks.clear()

        self.hy_current.clear()
        self.hy_blocks.clear()

        self.hy_sequences_sent = 0
        self.hy_blocks_sent = 0
        self.hy_primitives_sent = 0
        self.hy_linear_sent = 0
        self.hy_circular_sent = 0

        self.absolute = True
        self.feed = self.default_feed
        self.motion_mode = None

        self.sync_position()
        self._reset_cam_state_after_sync()

        print(
            "V10 COORDINATES: "
            + (
                "WORK zero = robot START"
                if self.work_coordinate_active
                else "MACHINE absolute"
            )
        )

        try:
            for number, raw_line in enumerate(lines, start=1):
                self.line_number = number

                if self._stop_event.is_set():
                    print("G-code STOP")
                    return False

                if not self._wait_if_paused():
                    print("G-code STOP")
                    return False

                if not self.execute_line(raw_line):
                    self._summary()
                    return True

            if self._hybrid_pending():
                self.flush_hybrid("EOF")

            self._summary()
            return True

        finally:
            self.running = False


def main():
    from rho4_driver import Rho4

    parser = argparse.ArgumentParser(
        description=(
            "RHO4 G-code V10 CAM-ready native HYRING runner"
        )
    )

    parser.add_argument(
        "gcode",
        nargs="?",
        default="gcode_v10_cam_demo.gcode",
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=6051)
    parser.add_argument(
        "--feed",
        type=float,
        default=300.0,
        help="Default feed in mm/min before the file sets F",
    )
    parser.add_argument(
        "--rapid",
        type=float,
        default=300.0,
        help="Standalone G0 LINEAR speed in mm/min",
    )
    parser.add_argument("--timeout", type=float, default=900.0)
    parser.add_argument(
        "--refill-window",
        type=float,
        default=0.50,
    )
    parser.add_argument(
        "--lookahead",
        type=int,
        default=11,
    )
    parser.add_argument(
        "--max-arc-sweep",
        type=float,
        default=180.0,
    )
    parser.add_argument(
        "--machine-coordinates",
        action="store_true",
        help=(
            "Start in machine coordinates. Default is safer CAM mode: "
            "job START is work X0 Y0 Z0 R0."
        ),
    )
    parser.add_argument(
        "--ignore-aux",
        action="store_true",
        help=(
            "Recognize M3/M4/M5/M7/M8/M9 as explicit no-ops. "
            "Use only for motion-only tests."
        ),
    )
    parser.add_argument(
        "--ignore-tool-change",
        action="store_true",
        help=(
            "Recognize M6 as an explicit no-op for motion-only tests."
        ),
    )
    parser.add_argument(
        "--arm-io",
        action="store_true",
        help=(
            "Explicitly ARM candidate physical spindle output. "
            "Only use after wiring/output channel is verified."
        ),
    )
    parser.add_argument(
        "--run",
        action="store_true",
        help="Required for non-localhost targets",
    )

    args = parser.parse_args()

    if (
        args.host not in ("127.0.0.1", "localhost", "::1")
        and not args.run
    ):
        raise SystemExit(
            "Non-localhost target: add --run only after preflight."
        )

    rho = Rho4(args.host, args.port, timeout=3.0)
    rho.connect()

    try:
        print("PING:", rho.ping())
        print("STATUS:", rho.get_status())
        print("START:", rho.get_position())
        print(
            "V10 PLANNER: CAM-ready native LINEAR/CIRCULAR, "
            f"rapid={args.rapid:.1f} mm/min, "
            f"refill_window={args.refill_window:.3f} s, "
            f"lookahead={args.lookahead}"
        )

        if args.arm_io:
            print(
                "WARNING: ARMING physical spindle output candidate. "
                "CMD35 will first drive it LOW."
            )
            print("IO:", rho.arm_io(True))
        else:
            print(
                "I/O output writes remain LOCKED. "
                "M3/M5 will fail until --arm-io is explicit."
            )

        runner = GCodeRunner(
            rho,
            default_feed=args.feed,
            rapid_feed=args.rapid,
            move_timeout=args.timeout,
            min_refill_window_s=args.refill_window,
            refill_lookahead_blocks=args.lookahead,
            native_arc_max_deg=args.max_arc_sweep,
            start_in_work_coordinates=(
                not args.machine_coordinates
            ),
            ignore_aux_mcodes=args.ignore_aux,
            ignore_tool_change=args.ignore_tool_change,
        )

        ok = runner.run_file(args.gcode)
        print("GCODE V10 ->", ok)
        print("END:", rho.get_position())

    finally:
        try:
            status = rho.get_io_status()
            if status["io_armed"]:
                try:
                    rho.set_spindle(False)
                finally:
                    rho.arm_io(False)
        except Exception as exc:
            print("I/O shutdown warning:", exc)

        rho.disconnect()


if __name__ == "__main__":
    main()
