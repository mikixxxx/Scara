from __future__ import annotations

import argparse
import math
import re
import threading
import time

# This file is intentionally self-contained on the G-code side.
# It does NOT import gcode_runner_v8.py or gcode_runner_v9.py.
# Only the robot communication driver remains a separate module:
#     rho4_driver.py


class _BaseRunnerV8:
    """
    G-code executor V8 pro RHO4 adaptive PCRING streaming.

    V8:
      - G1 body a linearizovane G2/G3 se skladaji do 16bodovych bufferu.
      - Neni zde 32-buffer / 512-point batch limit.
      - Cela souvisla feed sekvence se spousti jednim PCRING.
      - RHO4 strida A/B a Python za jizdy doplnuje uvolneny slot.
      - LAST=1 se posle pouze s poslednim skutecnym bufferem sekvence.
      - Controller hlida SEQ/FREE a pri underrunu vraci MOVE_ERROR=29001.

    Podpora:
      G90 / G91
      G17
      G0
      G1
      G2 / G3 pomoci I/J
      modalni G0/G1/G2/G3
      X Y Z R
      F [mm/min]
      M2 / M30
      ; komentar
      (komentar)

    V8 planner:
      - G2/G3 segmentaci voli automaticky podle geometricke tolerance.
      - Soucasne hlida minimalni odhad refill window.
      - Pokud nelze pri zadanem F splnit oboji, pohyb se nespusti a runner
        vypise maximalni odhadovanou F pro dany konzervativni model.
      - --arc-step neni potreba.

    Omezeni:
      - PCRING je LINEAR-only, G2/G3 se linearizuji v Pythonu.
      - G2/G3 pouze XY + I/J.
      - I/J jsou offset od START i v G90.
      - Helix (zmena Z v G2/G3) zatim neni podporovan.
      - R na G2/G3 je zakazano, protoze u robota R znamena rotaci.
      - G0 zustava standalone LINEAR a proto ukonci aktualni PCRING sekvenci.
      - V8 pred startem naplanuje souvislou sekvenci do RAM; samotne A/B
        buffery se pak za pohybu doplnuji pres PCRING.
      - Pause behem rozjeteho PCRING neni okamzity; STOP je pres CMD15.
    """

    ARC_RADIUS_TOLERANCE_MM = 0.20
    ARC_EPS = 1e-9
    POINT_EPS = 1e-9

    def __init__(
        self,
        rho,
        default_feed=600.0,
        rapid_feed=1200.0,
        move_timeout=1200.0,
        stream_block_points=16,
        arc_tolerance_mm=0.20,
        verify_endpoint_mm=0.10,
        min_refill_window_s=0.50,
        refill_lookahead_blocks=11,
    ):
        self.rho = rho

        self.default_feed = float(default_feed)
        self.rapid_feed = float(rapid_feed)
        self.move_timeout = float(move_timeout)

        self.stream_block_points = int(stream_block_points)
        if self.stream_block_points != 16:
            raise ValueError(
                "PCRING BAPS vyzaduje stream_block_points=16"
            )

        self.arc_tolerance_mm = float(arc_tolerance_mm)
        if self.arc_tolerance_mm <= 0:
            raise ValueError("arc_tolerance_mm musi byt > 0")

        self.verify_endpoint_mm = float(verify_endpoint_mm)
        self.min_refill_window_s = float(min_refill_window_s)
        if self.min_refill_window_s < 0:
            raise ValueError("min_refill_window_s musi byt >= 0")

        self.refill_lookahead_blocks = int(refill_lookahead_blocks)
        if not 0 <= self.refill_lookahead_blocks < self.stream_block_points:
            raise ValueError(
                "refill_lookahead_blocks musi byt 0..15"
            )

        self.refill_guard_segments = (
            self.stream_block_points - self.refill_lookahead_blocks
        )

        self.absolute = True
        self.feed = self.default_feed
        self.motion_mode = None

        # Logical/planned position. Posouva se uz pri queue, ne az po pohybu.
        self.position: dict[str, float] | None = None

        # Body aktualniho feed-run, ktere jeste netvori cely 16bodovy blok.
        self.pending_points = []
        self.pending_feed = None
        self.pending_start = None

        # Hotove 16bodove bloky cekajici na jednu PCRING sekvenci.
        self.stream_blocks = []

        self.stream_batches_sent = 0
        self.stream_blocks_sent = 0
        self.stream_points_sent = 0
        self.padding_points_added = 0

        self._stop_event = threading.Event()
        self._pause_event = threading.Event()
        self._pause_event.set()

        self.running = False
        self.line_number = 0

    # =========================================================
    # BASIC STATE
    # =========================================================

    def sync_position(self) -> dict[str, float]:
        x, y, z, r = self.rho.get_position()
        position = {
            "X": float(x),
            "Y": float(y),
            "Z": float(z),
            "R": float(r),
        }
        self.position = position
        return position.copy()

    def reset(self):
        self.absolute = True
        self.feed = self.default_feed
        self.motion_mode = None

        self.pending_points.clear()
        self.pending_feed = None
        self.pending_start = None
        self.stream_blocks.clear()

        self.stream_batches_sent = 0
        self.stream_blocks_sent = 0
        self.stream_points_sent = 0
        self.padding_points_added = 0

        self._stop_event.clear()
        self._pause_event.set()
        self.line_number = 0

        self.sync_position()

    def pause(self):
        self._pause_event.clear()

    def resume(self):
        self._pause_event.set()

    def stop(self):
        self._stop_event.set()
        self._pause_event.set()

        if self.rho.is_connected():
            try:
                self.rho.stop()
            except Exception as exc:
                print("STOP warning:", exc)

    def _wait_if_paused(self):
        while not self._pause_event.wait(0.1):
            if self._stop_event.is_set():
                return False
        return not self._stop_event.is_set()

    # =========================================================
    # PARSER
    # =========================================================

    @staticmethod
    def _strip_comments(line):
        line = re.sub(r"\([^)]*\)", "", line)
        line = line.split(";", 1)[0]
        return line.strip().upper()

    @staticmethod
    def _words(line):
        return [
            (letter, float(value))
            for letter, value in re.findall(
                r"([A-Z])\s*([-+]?(?:\d+(?:\.\d*)?|\.\d+))",
                line,
            )
        ]

    @staticmethod
    def _values(words):
        values = {}
        for letter, value in words:
            values[letter] = value
        return values

    # =========================================================
    # STREAM PLANNER
    # =========================================================

    @staticmethod
    def _dict_to_tuple(point):
        return (
            float(point["X"]),
            float(point["Y"]),
            float(point["Z"]),
            float(point["R"]),
        )

    @staticmethod
    def _tuple_to_dict(point):
        return {
            "X": float(point[0]),
            "Y": float(point[1]),
            "Z": float(point[2]),
            "R": float(point[3]),
        }

    @staticmethod
    def _point_distance(a, b):
        return math.sqrt(
            (a[0] - b[0])**2
            + (a[1] - b[1])**2
            + (a[2] - b[2])**2
            + ((a[3] - b[3]) / 20.0)**2
        )

    def _verify_actual_endpoint(self, expected):
        actual = self.rho.get_position()

        dxyz = math.sqrt(
            (actual[0] - expected[0])**2
            + (actual[1] - expected[1])**2
            + (actual[2] - expected[2])**2
        )
        dr = abs(
            (actual[3] - expected[3] + 180.0) % 360.0 - 180.0
        )

        if dxyz > self.verify_endpoint_mm or dr > 0.2:
            raise RuntimeError(
                "STREAM endpoint nesedi: "
                f"dXYZ={dxyz:.3f} mm, dR={dr:.3f} deg"
            )

    def _current_tuple(self):
        if self.position is None:
            self.sync_position()
        return self._dict_to_tuple(self.position)

    def _queue_full_block(self, points, feed_mm_min):
        if len(points) != 16:
            raise ValueError("Interni chyba: stream block musi mit 16 bodu")

        speed_mm_s = float(feed_mm_min) / 60.0
        if speed_mm_s <= 0:
            raise ValueError("STREAM speed musi byt > 0")

        self.stream_blocks.append({
            "points": [tuple(map(float, p)) for p in points],
            "speed_mm_s": speed_mm_s,
        })

    def _subdivide_last_nonzero_segment(self, points, start, extra):
        """
        Doplni presny pocet bodu bez zmeny geometrie polyline.

        Najde posledni nenulovy segment a rozdeli pouze ten. Puvodni
        vrcholy pred/za segmentem zustanou zachovane.
        """
        if extra <= 0:
            return list(points)

        pts = list(points)
        nodes = [tuple(start)] + pts

        seg_index = None
        for i in range(len(nodes) - 2, -1, -1):
            if self._point_distance(nodes[i], nodes[i + 1]) > self.POINT_EPS:
                seg_index = i
                break

        if seg_index is None:
            # Cely run je nulovy. Zachovame protokol fixed-16 opakovanim
            # stejneho endpointu. Pri beznem G-code se sem prakticky nejde.
            return pts + [pts[-1]] * extra

        a = nodes[seg_index]
        b = nodes[seg_index + 1]

        inserted = []
        pieces = extra + 1
        for j in range(1, pieces + 1):
            u = j / pieces
            inserted.append(tuple(
                a[k] + (b[k] - a[k]) * u
                for k in range(4)
            ))

        # nodes segment i -> i+1 corresponds to points endpoint index i.
        # Keep points before that endpoint, replace endpoint by subdivisions,
        # then keep all later original vertices unchanged.
        before = pts[:seg_index]
        after = pts[seg_index + 1:]
        return before + inserted + after

    def _close_pending_run(self, reason="feed run end"):
        if not self.pending_points:
            self.pending_feed = None
            self.pending_start = None
            return

        count = len(self.pending_points)
        if count > 16:
            raise RuntimeError("Interni chyba: pending_points > 16")

        points = list(self.pending_points)

        if count < 16:
            extra = 16 - count
            points = self._subdivide_last_nonzero_segment(
                points,
                self.pending_start,
                extra,
            )
            self.padding_points_added += extra
            print(
                f"  -> doplneni feed-run {count}->16 bodu "
                f"(+{extra} subdivision), {reason}"
            )

        self._queue_full_block(points, self.pending_feed)

        self.pending_points.clear()
        self.pending_feed = None
        self.pending_start = None

    def _stream_statistics(self, blocks):
        """
        Diagnostics only. Motion semantics are unchanged from V7.

        XYZ path length and ideal constant-segment time are estimated from
        the packed controller points and each block's programmed speed.
        R-only travel is intentionally not included in the time estimate.
        """
        if not blocks:
            return {
                "length_mm": 0.0,
                "time_s": 0.0,
                "speed_min": 0.0,
                "speed_max": 0.0,
                "speed_values": [],
                "speed_changes": 0,
            }

        previous = tuple(map(float, self.rho.get_position()))
        total_length = 0.0
        total_time = 0.0
        speed_values = []
        speed_changes = 0
        previous_speed = None

        for block in blocks:
            speed = float(block["speed_mm_s"])
            if speed <= 0.0:
                raise ValueError("PCRING speed musi byt > 0")

            if previous_speed is None or abs(speed - previous_speed) > 1e-9:
                speed_values.append(speed)
                if previous_speed is not None:
                    speed_changes += 1
                previous_speed = speed

            for point in block["points"]:
                current = tuple(map(float, point))
                dx = current[0] - previous[0]
                dy = current[1] - previous[1]
                dz = current[2] - previous[2]
                length = math.sqrt(dx*dx + dy*dy + dz*dz)
                total_length += length
                total_time += length / speed
                previous = current

        return {
            "length_mm": total_length,
            "time_s": total_time,
            "speed_min": min(speed_values),
            "speed_max": max(speed_values),
            "speed_values": speed_values,
            "speed_changes": speed_changes,
        }


    def flush_stream(self, reason="flush"):
        """
        Uzavre pending feed-run a spusti vsechny pripravene 16bodove buffery
        jako JEDEN PCRING bez 32-buffer / 512-point batch hranice.
        """
        if self.pending_points:
            self._close_pending_run(reason)

        if not self.stream_blocks:
            return True

        if self._stop_event.is_set():
            return False

        if not self._wait_if_paused():
            return False

        blocks = [
            {
                "points": list(block["points"]),
                "speed_mm_s": float(block["speed_mm_s"]),
            }
            for block in self.stream_blocks
        ]

        expected = tuple(blocks[-1]["points"][-1])

        self.stream_batches_sent += 1
        batch_no = self.stream_batches_sent

        speeds = [float(block["speed_mm_s"]) for block in blocks]

        print(
            f"  => PCRING #{batch_no}: "
            f"{len(blocks)} bufferu / {len(blocks)*16} bodu, "
            f"{reason}"
        )
        print(
            "     speed range: "
            f"{min(speeds):.3f} .. {max(speeds):.3f} mm/s"
        )

        stats = self._stream_statistics(blocks)
        speed_text = ", ".join(
            f"{speed:.2f}"
            for speed in stats["speed_values"]
        )
        print(
            "     V8 plan: "
            f"length={stats['length_mm']:.2f} mm, "
            f"ideal_time={stats['time_s']:.1f} s "
            f"({stats['time_s']/60.0:.1f} min), "
            f"speed_zones={len(stats['speed_values'])}, "
            f"speed_changes={stats['speed_changes']}"
        )
        print(
            "     V8 speeds [mm/s]: "
            f"{speed_text}"
        )

        if len(blocks) > 32:
            print(
                "     > old PCSTREAM limit: "
                f"+{len(blocks)-32} bufferu / "
                f"+{(len(blocks)-32)*16} bodu"
            )

        # Global timing check on the FINAL packed 16-point buffers.
        # This also catches short segments introduced by feed-run padding.
        windows = self.rho.estimate_refill_windows(
            blocks,
            lookahead_blocks=self.refill_lookahead_blocks,
        )

        if windows:
            limiting = min(
                windows,
                key=lambda item: item["window_s"],
            )
            min_window = limiting["window_s"]
            seq = limiting["intervening_seq"]
            speed = float(blocks[seq]["speed_mm_s"])

            print(
                "  V8 refill check: "
                f"min={min_window:.3f} s "
                f"(buffer seq{seq}, required "
                f"{self.min_refill_window_s:.3f} s)"
            )

            if (
                self.min_refill_window_s > 0.0
                and min_window + 1e-12 < self.min_refill_window_s
            ):
                safe_speed = (
                    speed
                    * min_window
                    / self.min_refill_window_s
                )
                safe_feed = safe_speed * 60.0

                raise RuntimeError(
                    "V8 REFILL GUARD: packed stream je pro "
                    f"F={speed*60.0:.1f} mm/min prilis husty. "
                    f"Odhad window={min_window:.3f} s, "
                    f"pozadovano={self.min_refill_window_s:.3f} s. "
                    f"Pro limitujici buffer vychazi maximalni "
                    f"F priblizne {safe_feed:.1f} mm/min. "
                    "Robot nebyl spusten."
                )

        ok = self.rho.stream_blocks_ring(
            blocks,
            timeout=self.move_timeout,
            poll_interval=0.005,
            validate_points=True,
            full_preflight=True,
            min_refill_window_s=self.min_refill_window_s,
            lookahead_blocks=self.refill_lookahead_blocks,
            clear_before_start=True,
        )

        if not ok:
            raise RuntimeError("PCRING byl zastaven")

        self._verify_actual_endpoint(expected)

        self.stream_blocks_sent += len(blocks)
        self.stream_points_sent += len(blocks) * 16
        self.stream_blocks.clear()

        # Logical endpoint is now physically complete.
        self.sync_position()
        return True


    def _set_feed(self, new_feed):
        new_feed = float(new_feed)
        if new_feed <= 0:
            raise ValueError("F musi byt > 0 mm/min")

        # Feed se muze menit mezi A/B buffery bez ukonceni PCRING.
        # Pokud mame rozpracovany <16 bodovy run stareho feedu, uzavreme ho
        # geometricky bezpecnym subdivision paddingem na cely buffer.
        if (
            self.pending_points
            and self.pending_feed is not None
            and abs(new_feed - self.pending_feed) > 1e-9
        ):
            self._close_pending_run("zmena F")

        self.feed = new_feed

    def _append_stream_point(self, point, feed_mm_min):
        feed_mm_min = float(feed_mm_min)

        point_tuple = self._dict_to_tuple(point)
        current = self._current_tuple()

        # Nulovy G1 pohyb neni treba posilat controlleru.
        if self._point_distance(current, point_tuple) <= self.POINT_EPS:
            self.position = self._tuple_to_dict(point_tuple)
            return

        if self.pending_points:
            pending_feed = self.pending_feed
            if pending_feed is None:
                raise RuntimeError("Interni chyba: pending_feed je None")
            if abs(feed_mm_min - pending_feed) > 1e-9:
                self._close_pending_run("zmena feedu")
        else:
            self.pending_feed = feed_mm_min
            self.pending_start = current

        self.pending_points.append(point_tuple)

        # Planned position advances immediately.
        self.position = self._tuple_to_dict(point_tuple)

        if len(self.pending_points) == 16:
            points = list(self.pending_points)
            feed = self.pending_feed
            self.pending_points.clear()
            self.pending_feed = None
            self.pending_start = None
            self._queue_full_block(points, feed)

    # =========================================================
    # TARGET / LINEAR
    # =========================================================

    def _target_from_values(self, values):
        position = self.position
        if position is None:
            position = self.sync_position()

        target = position.copy()

        for axis in ("X", "Y", "Z", "R"):
            if axis not in values:
                continue

            if self.absolute:
                target[axis] = float(values[axis])
            else:
                target[axis] += float(values[axis])

        return target

    def _execute_g0(self, values):
        """
        Rapid zustava standalone CMD16.
        Pred G0 dokoncime aktualni PCRING chain.
        """
        if self.pending_points or self.stream_blocks:
            self.flush_stream("pred G0")

        target = self._target_from_values(values)

        if not any(axis in values for axis in ("X", "Y", "Z", "R")):
            return

        speed_mm_s = self.rapid_feed / 60.0

        print(
            f"[{self.line_number:04d}] G0 "
            f"X={target['X']:.3f} Y={target['Y']:.3f} "
            f"Z={target['Z']:.3f} R={target['R']:.3f} "
            f"F={self.rapid_feed:.1f} mm/min "
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

        self.sync_position()

    def _execute_g1(self, values):
        target = self._target_from_values(values)

        if not any(axis in values for axis in ("X", "Y", "Z", "R")):
            return

        print(
            f"[{self.line_number:04d}] G1 queue "
            f"X={target['X']:.3f} Y={target['Y']:.3f} "
            f"Z={target['Z']:.3f} R={target['R']:.3f} "
            f"F={self.feed:.1f}"
        )

        self._append_stream_point(target, self.feed)

    # =========================================================
    # ARC -> LINEARIZATION
    # =========================================================

    def _arc_geometry(self, values, motion_g):
        position = self.position
        if position is None:
            position = self.sync_position()

        if motion_g not in (2, 3):
            raise ValueError("Interni chyba: motion_g musi byt G2 nebo G3")

        if "R" in values:
            raise ValueError(
                f"Radek {self.line_number}: u G2/G3 nepouzivej R. "
                "U robota R znamena rotaci; pro radius pouzij I/J."
            )

        if "I" not in values and "J" not in values:
            raise ValueError(
                f"Radek {self.line_number}: G{motion_g} vyzaduje I a/nebo J"
            )

        sx = position["X"]
        sy = position["Y"]
        sz = position["Z"]
        sr = position["R"]

        if "Z" in values:
            target_z = (
                float(values["Z"])
                if self.absolute
                else sz + float(values["Z"])
            )
            if abs(target_z - sz) > 1e-6:
                raise ValueError(
                    f"Radek {self.line_number}: "
                    "G2/G3 se zmenou Z zatim nepodporujeme"
                )

        ex = sx
        ey = sy

        if "X" in values:
            ex = (
                float(values["X"])
                if self.absolute
                else sx + float(values["X"])
            )

        if "Y" in values:
            ey = (
                float(values["Y"])
                if self.absolute
                else sy + float(values["Y"])
            )

        # I/J jsou vzdy relativni k START bodu oblouku.
        i = float(values.get("I", 0.0))
        j = float(values.get("J", 0.0))

        cx = sx + i
        cy = sy + j

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
            sweep = -2.0 * math.pi if motion_g == 2 else 2.0 * math.pi
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
            "full_circle": full_circle,
        }

    def _arc_segment_plan(self, radius, sweep, feed_mm_min):
        """
        V8 adaptive arc planner.

        Goal A - geometry:
          sagitta/chord error <= arc_tolerance_mm

        Goal B - refill timing:
          one generated arc chord should be long enough that
          refill_guard_segments such segments take at least
          min_refill_window_s at the programmed feed.

        The geometry limit decides the MINIMUM number of segments.
        The refill limit decides the MAXIMUM useful number of segments.
        If both constraints do not overlap, programmed F is too high for
        the requested absolute arc tolerance under the conservative refill
        model. We do not silently change F.
        """
        radius = float(radius)
        sweep_abs = abs(float(sweep))
        feed_mm_min = float(feed_mm_min)

        if radius <= 0.0:
            raise ValueError("radius musi byt > 0")
        if sweep_abs <= self.ARC_EPS:
            raise ValueError("sweep musi byt nenulovy")
        if feed_mm_min <= 0.0:
            raise ValueError("feed musi byt > 0")

        tol = self.arc_tolerance_mm
        speed_mm_s = feed_mm_min / 60.0

        # Maximum angular step allowed by sagitta tolerance:
        # e = R * (1 - cos(theta/2))
        if tol >= radius:
            theta_tol = math.pi
        else:
            arg = 1.0 - tol / radius
            arg = max(-1.0, min(1.0, arg))
            theta_tol = 2.0 * math.acos(arg)

        theta_tol = max(theta_tol, 1e-12)

        # Fewest segments that still satisfy geometry.
        n_geometry = max(
            1,
            int(math.ceil(sweep_abs / theta_tol)),
        )

        theta_actual = sweep_abs / n_geometry
        chord_mm = 2.0 * radius * math.sin(theta_actual * 0.5)
        sagitta_mm = radius * (
            1.0 - math.cos(theta_actual * 0.5)
        )

        if self.min_refill_window_s <= 0.0:
            required_chord_mm = 0.0
        else:
            required_chord_mm = (
                speed_mm_s
                * self.min_refill_window_s
                / self.refill_guard_segments
            )

        refill_ok = (
            chord_mm + 1e-12 >= required_chord_mm
        )

        if self.min_refill_window_s > 0.0:
            max_safe_speed_mm_s = (
                chord_mm
                * self.refill_guard_segments
                / self.min_refill_window_s
            )
            max_safe_feed_mm_min = max_safe_speed_mm_s * 60.0
        else:
            max_safe_speed_mm_s = float("inf")
            max_safe_feed_mm_min = float("inf")

        return {
            "segments": n_geometry,
            "theta_deg": math.degrees(theta_actual),
            "chord_mm": chord_mm,
            "sagitta_mm": sagitta_mm,
            "required_chord_mm": required_chord_mm,
            "refill_ok": refill_ok,
            "max_safe_feed_mm_min": max_safe_feed_mm_min,
            "guard_segments": self.refill_guard_segments,
        }


    def _execute_arc(self, values, motion_g):
        geometry = self._arc_geometry(values, motion_g)

        radius = geometry["radius"]
        sweep = geometry["sweep"]
        plan = self._arc_segment_plan(
            radius,
            sweep,
            self.feed,
        )
        segments = plan["segments"]

        direction = "CW" if motion_g == 2 else "CCW"

        print(
            f"[{self.line_number:04d}] G{motion_g} {direction} -> queue "
            f"END X={geometry['ex']:.3f} Y={geometry['ey']:.3f} "
            f"RADIUS={radius:.3f} "
            f"segments={segments} "
            f"step={plan['theta_deg']:.2f}deg "
            f"chord={plan['chord_mm']:.3f}mm "
            f"err={plan['sagitta_mm']:.4f}mm "
            f"need>={plan['required_chord_mm']:.3f}mm "
            f"F={self.feed:.1f}"
        )

        if not plan["refill_ok"]:
            raise RuntimeError(
                f"Radek {self.line_number}: V8 ARC/REFILL konflikt. "
                f"Pro R={radius:.3f} mm a toleranci "
                f"{self.arc_tolerance_mm:.3f} mm je nejridsi dovolena "
                f"linearizace {segments} segmentu "
                f"(chord={plan['chord_mm']:.3f} mm), ale pri "
                f"F={self.feed:.1f} mm/min potrebuje refill model "
                f">={plan['required_chord_mm']:.3f} mm/segment. "
                f"Maximalni odhadovana F pro tuto geometrii je "
                f"{plan['max_safe_feed_mm_min']:.1f} mm/min. "
                "Robot nebyl spusten."
            )

        for index in range(1, segments + 1):
            u = index / segments
            angle = geometry["a0"] + geometry["sweep"] * u

            if index == segments:
                x = geometry["ex"]
                y = geometry["ey"]
            else:
                x = geometry["cx"] + radius * math.cos(angle)
                y = geometry["cy"] + radius * math.sin(angle)

            point = {
                "X": x,
                "Y": y,
                "Z": geometry["sz"],
                "R": geometry["sr"],
            }

            self._append_stream_point(point, self.feed)

    # =========================================================
    # LINE EXECUTION
    # =========================================================

    def execute_line(self, raw_line):
        line = self._strip_comments(raw_line)
        if not line:
            return True

        words = self._words(line)
        if not words:
            return True

        values = self._values(words)
        g_codes = [int(v) for l, v in words if l == "G"]
        m_codes = [int(v) for l, v in words if l == "M"]

        explicit_motion = None

        for g in g_codes:
            if g == 90:
                self.absolute = True
                print(f"[{self.line_number:04d}] G90 ABSOLUTE")

            elif g == 91:
                self.absolute = False
                print(f"[{self.line_number:04d}] G91 RELATIVE")

            elif g == 17:
                print(f"[{self.line_number:04d}] G17 XY PLANE")

            elif g in (0, 1, 2, 3):
                if explicit_motion is not None and explicit_motion != g:
                    raise ValueError(
                        f"Radek {self.line_number}: vice pohybovych G kodu"
                    )
                explicit_motion = g

            elif g in (18, 19):
                raise ValueError(
                    f"Radek {self.line_number}: G{g} neni podporovano; "
                    "pouzivame G17"
                )

            else:
                raise ValueError(
                    f"Radek {self.line_number}: nepodporovany G{g}"
                )

        if explicit_motion is not None:
            self.motion_mode = explicit_motion

        if "F" in values:
            self._set_feed(values["F"])

        # M2/M30 nejprve dokonci vsechen naplanovany stream.
        for m in m_codes:
            if m in (2, 30):
                if self.pending_points or self.stream_blocks:
                    self.flush_stream(f"M{m}")
                print(f"[{self.line_number:04d}] M{m} END")
                return False

            raise ValueError(
                f"Radek {self.line_number}: nepodporovany M{m}"
            )

        has_axis = any(
            axis in values for axis in ("X", "Y", "Z", "R")
        )
        has_arc_center = "I" in values or "J" in values
        should_move = has_axis or has_arc_center

        if should_move:
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

        elif "F" in values:
            print(
                f"[{self.line_number:04d}] "
                f"F={self.feed:.1f} mm/min"
            )

        return True

    # =========================================================
    # JOB
    # =========================================================

    def _summary(self):
        print(
            "PCRING summary: "
            f"{self.stream_batches_sent} PCRING sekvenci, "
            f"{self.stream_blocks_sent} bufferu, "
            f"{self.stream_points_sent} controller bodu, "
            f"padding +{self.padding_points_added}"
        )

    def run_lines(self, lines):
        if self.running:
            raise RuntimeError("G-code uz bezi")

        self.running = True
        self._stop_event.clear()
        self._pause_event.set()

        self.pending_points.clear()
        self.pending_feed = None
        self.pending_start = None
        self.stream_blocks.clear()

        self.stream_batches_sent = 0
        self.stream_blocks_sent = 0
        self.stream_points_sent = 0
        self.padding_points_added = 0

        self.sync_position()

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

            if self.pending_points or self.stream_blocks:
                self.flush_stream("EOF")

            self._summary()
            return True

        finally:
            self.running = False

    def run_file(self, filename):
        with open(filename, "r", encoding="utf-8") as file:
            return self.run_lines(file)


class _HybridRunnerV9(_BaseRunnerV8):
    """
    V9 G-code runner.

    Main change:
      G1    -> native LINEAR primitive
      G2/G3 -> native CIRCULAR primitive(s)

    No arc polygonization.
    No per-feed padding.
    Each primitive carries its own speed.
    Up to 16 native primitives are packed into one HYRING A/B slot.

    Full circles and arcs larger than native_arc_max_deg are split into
    multiple exact CIRCULAR primitives. Default maximum is 180 degrees.
    """

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
    ):
        # V8 parser/state is reused, but its LINEAR point-stream planner
        # is not used by V9.
        super().__init__(
            rho,
            default_feed=default_feed,
            rapid_feed=rapid_feed,
            move_timeout=move_timeout,
            stream_block_points=16,
            arc_tolerance_mm=0.05,
            verify_endpoint_mm=verify_endpoint_mm,
            min_refill_window_s=min_refill_window_s,
            refill_lookahead_blocks=refill_lookahead_blocks,
        )

        self.native_arc_max_deg = float(native_arc_max_deg)
        if not 1.0 <= self.native_arc_max_deg <= 180.0:
            raise ValueError(
                "native_arc_max_deg musi byt 1..180"
            )

        self.hy_current = []
        self.hy_blocks = []

        self.hy_sequences_sent = 0
        self.hy_blocks_sent = 0
        self.hy_primitives_sent = 0
        self.hy_linear_sent = 0
        self.hy_circular_sent = 0

    # =========================================================
    # STATE
    # =========================================================

    def reset(self):
        super().reset()

        self.hy_current.clear()
        self.hy_blocks.clear()

        self.hy_sequences_sent = 0
        self.hy_blocks_sent = 0
        self.hy_primitives_sent = 0
        self.hy_linear_sent = 0
        self.hy_circular_sent = 0

    def _set_feed(self, new_feed):
        """
        V9 does NOT close/pad a controller buffer on F change.
        Speed belongs to each primitive.
        """
        new_feed = float(new_feed)
        if new_feed <= 0:
            raise ValueError("F musi byt > 0 mm/min")
        self.feed = new_feed

    # =========================================================
    # HYBRID PRIMITIVE PACKING
    # =========================================================

    def _queue_primitive(
        self,
        ptype,
        end,
        speed_mm_s,
        mid=None,
    ):
        ptype = int(ptype)
        speed_mm_s = float(speed_mm_s)

        if ptype not in (
            self.rho.HY_LINEAR,
            self.rho.HY_CIRCULAR,
        ):
            raise ValueError("V9 primitive type musi byt LINEAR/CIRCULAR")

        if speed_mm_s <= 0.0:
            raise ValueError("V9 primitive speed musi byt > 0")

        end = tuple(float(v) for v in end)

        if mid is None:
            mid = end
        else:
            mid = tuple(float(v) for v in mid)

        primitive = {
            "type": ptype,
            "speed_mm_s": speed_mm_s,
            "end": end,
            "mid": mid,
        }

        self.hy_current.append(primitive)

        # Logical/planned position advances immediately.
        self.position = self._tuple_to_dict(end)

        if len(self.hy_current) == 16:
            self.hy_blocks.append({
                "primitives": list(self.hy_current),
            })
            self.hy_current.clear()

    def _close_hybrid_partial(self):
        if not self.hy_current:
            return

        # V9 protocol supports a partial FINAL slot.
        self.hy_blocks.append({
            "primitives": list(self.hy_current),
        })
        self.hy_current.clear()

    def _hybrid_pending(self):
        return bool(self.hy_current or self.hy_blocks)

    # =========================================================
    # NATIVE LINEAR / CIRCULAR
    # =========================================================

    def _execute_g0(self, values):
        """
        G0 remains standalone native LINEAR for now.
        It terminates the current HYRING sequence first.
        """
        if self._hybrid_pending():
            self.flush_hybrid("pred G0")

        # Parent G0 already uses native CMD16 LINEAR.
        return super()._execute_g0(values)

    def _execute_g1(self, values):
        target = self._target_from_values(values)

        if not any(axis in values for axis in ("X", "Y", "Z", "R")):
            return

        end = self._dict_to_tuple(target)
        current = self._current_tuple()

        if self._point_distance(current, end) <= self.POINT_EPS:
            self.position = self._tuple_to_dict(end)
            return

        speed_mm_s = self.feed / 60.0

        print(
            f"[{self.line_number:04d}] G1 -> native LINEAR "
            f"X={target['X']:.3f} Y={target['Y']:.3f} "
            f"Z={target['Z']:.3f} R={target['R']:.3f} "
            f"F={self.feed:.1f}"
        )

        self._queue_primitive(
            self.rho.HY_LINEAR,
            end,
            speed_mm_s,
        )

    def _execute_arc(self, values, motion_g):
        geometry = self._arc_geometry(values, motion_g)

        sweep = geometry["sweep"]
        radius = geometry["radius"]
        max_sweep = math.radians(self.native_arc_max_deg)

        parts = max(
            1,
            int(math.ceil(
                abs(sweep) / max_sweep - 1e-12
            )),
        )

        direction = "CW" if motion_g == 2 else "CCW"
        speed_mm_s = self.feed / 60.0

        print(
            f"[{self.line_number:04d}] G{motion_g} {direction} "
            f"-> native CIRCULAR x{parts} "
            f"R={radius:.3f} "
            f"sweep={math.degrees(sweep):.2f}deg "
            f"F={self.feed:.1f}"
        )

        for part_index in range(parts):
            u0 = part_index / parts
            u1 = (part_index + 1) / parts
            um = (u0 + u1) * 0.5

            amid = geometry["a0"] + sweep * um
            aend = geometry["a0"] + sweep * u1

            mid = (
                geometry["cx"] + radius * math.cos(amid),
                geometry["cy"] + radius * math.sin(amid),
                geometry["sz"],
                geometry["sr"],
            )

            if part_index == parts - 1:
                end = (
                    geometry["ex"],
                    geometry["ey"],
                    geometry["sz"],
                    geometry["sr"],
                )
            else:
                end = (
                    geometry["cx"] + radius * math.cos(aend),
                    geometry["cy"] + radius * math.sin(aend),
                    geometry["sz"],
                    geometry["sr"],
                )

            self._queue_primitive(
                self.rho.HY_CIRCULAR,
                end,
                speed_mm_s,
                mid=mid,
            )

    # =========================================================
    # FLUSH / EXECUTION
    # =========================================================

    def flush_hybrid(self, reason="flush"):
        self._close_hybrid_partial()

        if not self.hy_blocks:
            return True

        if self._stop_event.is_set():
            return False

        if not self._wait_if_paused():
            return False

        blocks = [
            {
                "primitives": [
                    {
                        "type": int(p["type"]),
                        "speed_mm_s": float(p["speed_mm_s"]),
                        "end": tuple(p["end"]),
                        "mid": tuple(p["mid"]),
                    }
                    for p in block["primitives"]
                ]
            }
            for block in self.hy_blocks
        ]

        # Only the final slot may be partial.
        for index, block in enumerate(blocks[:-1]):
            if len(block["primitives"]) != 16:
                raise RuntimeError(
                    f"V9 internal: non-final block {index} "
                    "does not contain 16 primitives"
                )

        expected = tuple(
            blocks[-1]["primitives"][-1]["end"]
        )

        stats = self.rho.hybrid_plan_statistics(blocks)
        windows = self.rho.estimate_hybrid_refill_windows(
            blocks,
            lookahead_blocks=self.refill_lookahead_blocks,
        )

        self.hy_sequences_sent += 1
        sequence_no = self.hy_sequences_sent

        print(
            f"  => HYRING #{sequence_no}: "
            f"{stats['blocks']} bufferu / "
            f"{stats['primitives']} native primitiv, "
            f"{reason}"
        )
        print(
            f"     LINEAR={stats['linear']} "
            f"CIRCULAR={stats['circular']} "
            f"length={stats['length_mm']:.2f} mm "
            f"ideal_time={stats['ideal_time_s']:.1f} s"
        )
        print(
            f"     speed range "
            f"{stats['speed_min']:.3f} .. "
            f"{stats['speed_max']:.3f} mm/s"
        )

        if windows:
            limiting = min(
                windows,
                key=lambda w: w["window_s"],
            )
            minimum = limiting["window_s"]

            print(
                "     V9 refill check: "
                f"min={minimum:.3f} s "
                f"(seq{limiting['intervening_seq']}, "
                f"required {self.min_refill_window_s:.3f} s)"
            )

            if minimum < self.min_refill_window_s:
                raise RuntimeError(
                    "V9 REFILL GUARD: native mixed stream ma "
                    f"jen {minimum:.3f} s, "
                    f"pozadovano {self.min_refill_window_s:.3f} s. "
                    "Robot nebyl spusten."
                )
        else:
            print(
                "     V9 refill check: n/a "
                "(<=2 controller buffers; no live refill required)"
            )

        ok = self.rho.stream_hybrid_blocks(
            blocks,
            timeout=self.move_timeout,
            poll_interval=0.005,
            validate_targets=True,
            full_preflight=True,
            min_refill_window_s=self.min_refill_window_s,
            lookahead_blocks=self.refill_lookahead_blocks,
            clear_before_start=True,
        )

        if not ok:
            raise RuntimeError("HYRING byl zastaven")

        self._verify_actual_endpoint(expected)

        self.hy_blocks_sent += stats["blocks"]
        self.hy_primitives_sent += stats["primitives"]
        self.hy_linear_sent += stats["linear"]
        self.hy_circular_sent += stats["circular"]

        self.hy_blocks.clear()

        self.sync_position()
        return True

    # =========================================================
    # JOB
    # =========================================================

    def _summary(self):
        print(
            "HYRING summary: "
            f"{self.hy_sequences_sent} sekvenci, "
            f"{self.hy_blocks_sent} bufferu, "
            f"{self.hy_primitives_sent} native primitiv "
            f"(L={self.hy_linear_sent}, "
            f"C={self.hy_circular_sent})"
        )

    def run_lines(self, lines):
        if self.running:
            raise RuntimeError("G-code uz bezi")

        self.running = True
        self._stop_event.clear()
        self._pause_event.set()

        # Unused V8 point-stream state stays empty.
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

        self.sync_position()

        try:
            for number, raw_line in enumerate(lines, start=1):
                self.line_number = number

                if self._stop_event.is_set():
                    print("G-code STOP")
                    return False

                if not self._wait_if_paused():
                    print("G-code STOP")
                    return False

                # Parent execute_line returns immediately on M2/M30 and its
                # V8 flush condition cannot see our V9 buffers. Flush first.
                stripped = self._strip_comments(raw_line)
                if stripped:
                    words = self._words(stripped)
                    m_codes = [
                        int(v)
                        for letter, v in words
                        if letter == "M"
                    ]
                    if any(m in (2, 30) for m in m_codes):
                        if self._hybrid_pending():
                            self.flush_hybrid(
                                "M2/M30"
                            )

                if not self.execute_line(raw_line):
                    self._summary()
                    return True

            if self._hybrid_pending():
                self.flush_hybrid("EOF")

            self._summary()
            return True

        finally:
            self.running = False


class GCodeRunner(_HybridRunnerV9):
    """
    Final CAM + I/O + probe G-code runner for RHO4 HYRING.

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
      - Default FINAL coordinate system is WORK coordinates:
            program X0 Y0 Z0 R0 == robot position/orientation at job start.
        This is much safer for normal CAM files than treating G90 X0 Y0 as
        robot machine zero.
      - Use --machine-coordinates only when machine-coordinate G90 is really
        intended.
      - G0 always flushes HYRING and runs as a separate LINEAR move.
      - M3/M5 control the verified X11 spindle-enable output.
        Physical output writes require explicit --arm-io.
      - M4 is not implemented.
      - M7/M8/M9 can be explicitly ignored with --ignore-aux.
      - M6 can be explicitly ignored with --ignore-tool-change.
      - Tool-length/cutter compensation ON, canned cycles and
        machine-home commands are rejected instead of silently ignored.
      - G38.2 probing is supported intentionally only for Z.

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

        self.job_origin: dict[str, float] | None = None
        self.work_offset = {
            "X": 0.0,
            "Y": 0.0,
            "Z": 0.0,
            "R": 0.0,
        }
        self.work_coordinate_active = self.start_in_work_coordinates

        self.spindle_rpm: float | None = None
        self.selected_tool: int | None = None

        self.rapid_moves = 0
        self.dwell_count = 0
        self.dwell_total_s = 0.0
        self.aux_noop_count = 0
        self.toolchange_noop_count = 0

        self.probe_count = 0
        self.last_probe_machine: tuple[float, float, float, float] | None = None

    # =========================================================
    # JOB / COORDINATE STATE
    # =========================================================

    def _reset_cam_state_after_sync(self):
        position = self.position
        if position is None:
            position = self.sync_position()

        self.units_name = "mm"
        self.unit_scale = 1.0
        self.arc_center_absolute = False
        self.exact_stop = False

        self.job_origin = position.copy()

        if self.start_in_work_coordinates:
            self.work_offset = position.copy()
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
        position = self.position
        if position is None:
            position = self.sync_position()

        target = position.copy()

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
        position = self.position
        if position is None:
            position = self.sync_position()

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

        sx = position["X"]
        sy = position["Y"]
        sz = position["Z"]
        sr = position["R"]

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
        Conservative FINAL rapid:
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
        job_origin = self.job_origin
        if job_origin is None:
            raise RuntimeError("G54: job origin neni inicializovan")

        self.work_offset = job_origin.copy()
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

        measured = result["measured"]
        probe_position = (
            float(measured[0]),
            float(measured[1]),
            float(measured[2]),
            float(measured[3]),
        )
        self.last_probe_machine = probe_position
        self.probe_count += 1

        # The robot may stop slightly beyond the captured trigger position.
        # Keep the actual TCP position synchronized for subsequent motion.
        actual = self.sync_position()

        print(
            "  PROBE HIT @MPOS/WC: "
            f"X={probe_position[0]:.4f} "
            f"Y={probe_position[1]:.4f} "
            f"Z={probe_position[2]:.4f} "
            f"R={probe_position[3]:.4f}"
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
        stop position. All other G92 cases use the standard work-offset behavior.
        """
        probe_position = self.last_probe_machine
        if (
            probe_position is not None
            and "Z" in values
            and not any(a in values for a in ("X", "Y", "R"))
        ):
            value = float(values["Z"])

            self.work_offset["Z"] = (
                probe_position[2] - value
            )
            self.work_coordinate_active = True

            print(
                f"[{self.line_number:04d}] "
                "G92 Z FROM PROBE @MPOS -> "
                f"work_offset.Z={self.work_offset['Z']:.4f}"
            )

            self.last_probe_machine = None
            return

        position = self.position
        if position is None:
            position = self.sync_position()

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
                position[axis] - float(values[axis])
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
                    "ale coolant output zatim neni implementovan. "
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
    # FINAL LINE EXECUTION
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
                    "neni nakonfigurovan; runner zatim pouziva G54"
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
            "FINAL CAM summary: "
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

        # Legacy V8 stream state is unused by the final HYRING backend.
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
            "FINAL COORDINATES: "
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
            "RHO4 final CAM + I/O + probe native HYRING runner"
        )
    )

    parser.add_argument(
        "gcode",
        nargs="?",
        default="rho4_test_final.gcode",
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
            "Recognize M7/M8/M9 as explicit no-ops. "
            "M3/M5 remain real spindle-output commands."
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
            "FINAL PLANNER: CAM-ready native LINEAR/CIRCULAR, "
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
        print("GCODE FINAL ->", ok)
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
