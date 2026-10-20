"""SCARA CAD SR6 v0.3.0 - standalone offline trajectory sketch editor."""
from __future__ import annotations

import copy
import json
import math
import sys
from pathlib import Path

from PySide6.QtCore import Qt, QPointF, QRectF, QSize, QStandardPaths
from PySide6.QtGui import QAction, QColor, QPainter, QPen, QBrush, QPolygonF, QKeySequence
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QSplitter, QGroupBox, QFormLayout, QToolButton, QPushButton,
    QDoubleSpinBox, QSpinBox, QComboBox, QLabel, QListWidget, QListWidgetItem,
    QPlainTextEdit, QTabWidget, QCheckBox, QMessageBox, QFileDialog,
    QStackedWidget, QScrollArea, QFrame, QButtonGroup, QInputDialog, QLineEdit,
)

from scara_core import (
    SOLVER, new_project, load_project, save_project, generate_gcode, validate_project,
    pose, endpoint, path_samples, circle_from_3_points, reachability_error, MOVE_TYPES,
    active_tool, machine_pose, normalize_tools, DEFAULT_TOOL,
    create_tool_profile, rename_tool_profile, remove_tool_profile, text_toolpath_targets,
    _has_tool_offset, vector_toolpath_targets, bezier_toolpath_points,
    refill_report,
)
from scara_vectors import import_vectors, transform_paths

COLOR_BG = QColor("#0e1721")
COLOR_GRID = QColor("#263746")
COLOR_AXIS = QColor("#71879a")
COLOR_LINEAR = QColor("#4ebdee")
COLOR_PTP = QColor("#f3ad68")
COLOR_ARC = QColor("#64d3a3")
COLOR_SELECT = QColor("#ffda6c")
COLOR_START = QColor("#d49af5")
COLOR_FLANGE = QColor("#ffb963")
COLOR_TEXT = QColor("#e0bc74")
COLOR_VECTOR = QColor("#caa5ed")
COLOR_BEZIER = QColor("#67e0c3")

# The view covers the circular workspace; the grid is NOT a movement limit.
GRID_HALF_SIZE = math.ceil(SOLVER.XY_RADIUS_MAX / 50.0) * 50  # 600 mm
MASK_CELL_SIZE = 12
MARKER_RADIUS = 5  # jednotny polomer vsech bodu v pixelech


class SketchCanvas(QWidget):
    def __init__(self, owner):
        super().__init__()
        self.owner = owner
        self.zoom = 1.0
        self.pan_x = self.pan_y = 0.0
        self.setMinimumSize(QSize(400, 380))
        self.setMouseTracking(True)
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        self._drag = None
        self._pan_mouse = None
        self._cursor_world = None
        self._mask = []
        self.rebuild_reachability()

    def rebuild_reachability(self):
        start = self.owner.project["start"]
        z = start["z"]
        r_widget = getattr(self.owner, "mask_r_spin", None)
        linked = getattr(self.owner, "mask_link_checkbox", None)
        r = start["r"] if r_widget is None or linked is None or linked.isChecked() else r_widget.value()
        tool = active_tool(self.owner.project)
        cells = []
        step = MASK_CELL_SIZE
        tcp_reach_max = SOLVER.XY_RADIUS_MAX + math.hypot(tool["x"], tool["y"])
        extent = int(math.ceil(tcp_reach_max / step) * step)
        for x in range(-extent, extent + 1, step):
            for y in range(-extent, extent + 1, step):
                if reachability_error((float(x), float(y), z, r), tool) is None:
                    cells.append((x, y))
        self._mask = cells
        self.update()

    def scale(self):
        tool = active_tool(self.owner.project)
        # TCP may extend outside the A4 radius when the attachment is offset.
        half_range = GRID_HALF_SIZE + math.hypot(tool["x"], tool["y"])
        view_size = 2.0 * (half_range + 25)
        return max(.04, min((self.width()-48)/view_size, (self.height()-55)/view_size)) * self.zoom

    def to_screen(self, x, y):
        s = self.scale()
        return QPointF(self.width()*.5 + self.pan_x + x*s,
                       self.height()*.5 + self.pan_y - y*s)

    def to_world(self, p):
        s = self.scale()
        return ((p.x() - self.width()*.5 - self.pan_x)/s,
                -(p.y() - self.height()*.5 - self.pan_y)/s)

    def snapped(self, xy):
        if not self.owner.snap_checkbox.isChecked():
            return xy
        step = self.owner.snap_step.value()
        return tuple(round(v/step)*step for v in xy)

    def _draw_marker(self, painter, xy, color, selected=False, hollow=False):
        pt = self.to_screen(xy[0], xy[1])
        painter.setPen(QPen(COLOR_SELECT if selected else QColor("#10202c"), 2.0))
        painter.setBrush(QBrush(QColor("#0e1721") if hollow else color))
        painter.drawEllipse(pt, MARKER_RADIUS, MARKER_RADIUS)

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        p.fillRect(self.rect(), COLOR_BG)
        s = self.scale()
        # Reachability is for desired TCP with selected tool and preview R.
        # Must be initialized before grid drawing (not only before A4 overlay).
        tool = active_tool(self.owner.project)
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QColor("#174038"))
        for x, y in self._mask:
            pt = self.to_screen(x-MASK_CELL_SIZE/2, y+MASK_CELL_SIZE/2)
            p.drawRect(QRectF(pt.x(), pt.y(), MASK_CELL_SIZE*s+0.5, MASK_CELL_SIZE*s+0.5))
        p.setBrush(Qt.BrushStyle.NoBrush)
        if not _has_tool_offset(tool):
            p.setPen(QPen(QColor("#415f68"), 1, Qt.PenStyle.DashLine))
            pc = self.to_screen(0, 0)
            p.drawEllipse(pc, SOLVER.XY_RADIUS_MAX*s, SOLVER.XY_RADIUS_MAX*s)
        # Cartesian grid is a visual guide, not a square motion boundary.
        extent = int(math.ceil(GRID_HALF_SIZE + math.hypot(tool["x"], tool["y"])))
        for v in range(-extent, extent + 1, 50):
            strong = (v % 100 == 0)
            col = QColor("#496373") if strong else COLOR_GRID
            p.setPen(QPen(col, 1 if strong else .65))
            p.drawLine(self.to_screen(v, -extent), self.to_screen(v, extent))
            p.drawLine(self.to_screen(-extent, v), self.to_screen(extent, v))
        p.setPen(QPen(COLOR_AXIS, 1.5))
        p.drawLine(self.to_screen(-extent, 0), self.to_screen(extent, 0))
        p.drawLine(self.to_screen(0, -extent), self.to_screen(0, extent))
        p.setPen(QColor("#b9cad9"))
        p.drawText(self.to_screen(extent, 0)+QPointF(5, -5), "X+")
        p.drawText(self.to_screen(0, extent)+QPointF(6, -7), "Y+")
        p.drawText(self.to_screen(0, 0)+QPointF(6, 15), "0,0")
        flange_overlay = getattr(self.owner, "show_flange", None)
        show_flange = (flange_overlay is not None and flange_overlay.isChecked())
        # Draw configured initial TCP point (not automatically sent to robot).
        previous = pose(self.owner.project["start"])
        if show_flange:
            flange = machine_pose(previous, self.owner.project)
            p.setPen(QPen(COLOR_FLANGE, 1.4, Qt.PenStyle.DashLine))
            p.drawLine(self.to_screen(*previous[:2]), self.to_screen(*flange[:2]))
            self._draw_marker(p, flange, COLOR_FLANGE, hollow=True)
            p.setPen(COLOR_FLANGE)
            p.drawText(self.to_screen(*flange[:2])+QPointF(10, 12), "A4")
        self._draw_marker(p, previous, COLOR_START, selected=self.owner.selected == -2)
        p.setPen(COLOR_START)
        p.drawText(self.to_screen(*previous[:2])+QPointF(10, -9), "START")
        for index, op in enumerate(self.owner.project["operations"]):
            kind = op["type"]
            selected = index == self.owner.selected
            if kind == "rotate":
                # End TCP is displaced by any tool offset; flange XY stays fixed.
                end = endpoint(previous, op, tool)
                self._draw_marker(p, end, COLOR_PTP, selected=selected)
                p.setPen(COLOR_SELECT if selected else COLOR_PTP)
                p.drawText(self.to_screen(*end[:2])+QPointF(10, -9), f"R{op['r']:g}°")
                previous = end
            elif kind == "vector":
                try:
                    paths=transform_paths(op['paths'],op['x'],op['y'],op['angle_deg'],op['scale'])
                    p.setPen(QPen(COLOR_SELECT if selected else COLOR_VECTOR,2 if selected else 1.4))
                    for path in paths:
                        p.drawPolyline(QPolygonF([self.to_screen(*xy) for xy in path]))
                    p.setPen(QPen(COLOR_VECTOR,1.3,Qt.PenStyle.DashLine))
                    p.drawLine(self.to_screen(*previous[:2]),self.to_screen(op['x'],op['y']))
                    self._draw_marker(p,(op['x'],op['y']),COLOR_VECTOR,selected=selected)
                    end=endpoint(previous,op,tool)
                    self._draw_marker(p,end,COLOR_VECTOR,selected=selected)
                    p.setPen(COLOR_VECTOR);p.drawText(self.to_screen(*end[:2])+QPointF(8,-8),'END')
                    previous=end
                except ValueError:pass
            elif kind == "text":
                from scara_stroke_font import strokes
                p.setPen(QPen(COLOR_SELECT if selected else COLOR_TEXT, 2.2 if selected else 1.5))
                try:
                    for path in strokes(op["text"], op["x"], op["y"],
                                        op["height"], op["spacing"],
                                        op.get("angle_deg", 0.0)):
                        p.drawPolyline(QPolygonF([self.to_screen(*xy) for xy in path]))
                except ValueError:
                    pass
                p.setPen(QPen(COLOR_TEXT,1.3,Qt.PenStyle.DashLine))
                p.drawLine(self.to_screen(*previous[:2]),self.to_screen(op['x'],op['y']))
                self._draw_marker(p, (op["x"], op["y"]), COLOR_TEXT, selected=selected)
                try:
                    end=endpoint(previous,op,tool)
                    self._draw_marker(p,end,COLOR_TEXT,selected=selected)
                    p.setPen(COLOR_TEXT);p.drawText(self.to_screen(*end[:2])+QPointF(8,-8),'END')
                    previous = end
                except ValueError:
                    pass  # Still allow editing an invalid text object.
            elif kind in MOVE_TYPES:
                end = pose(op)
                color = COLOR_PTP if kind in ("rapid", "ptp") else (COLOR_ARC if kind == "arc" else (COLOR_BEZIER if kind == "bezier" else COLOR_LINEAR))
                p.setPen(QPen(COLOR_SELECT if selected else color, 3.2 if selected else 2.1,
                              Qt.PenStyle.DashLine if kind in ("rapid", "ptp") else Qt.PenStyle.SolidLine))
                if kind == "bezier":
                    try:
                        pts=[previous]+bezier_toolpath_points(previous,op)
                        p.drawPolyline(QPolygonF([self.to_screen(*pt[:2]) for pt in pts]))
                        p.setPen(QPen(COLOR_BEZIER,1.1,Qt.PenStyle.DashLine))
                        p.drawLine(self.to_screen(*previous[:2]),self.to_screen(op['c1x'],op['c1y']))
                        p.drawLine(self.to_screen(op['c1x'],op['c1y']),self.to_screen(op['c2x'],op['c2y']))
                        p.drawLine(self.to_screen(op['c2x'],op['c2y']),self.to_screen(*end[:2]))
                        for k in ('c1','c2'):
                            self._draw_marker(p,(op[k+'x'],op[k+'y']),COLOR_BEZIER,
                                              selected=selected and self.owner.selected_part==k,hollow=True)
                    except ValueError:pass
                elif kind == "arc":
                    try:
                        pts = [previous] + path_samples(previous, op, step_mm=5.0)
                    except Exception:
                        pts = [previous, end]
                    poly = QPolygonF([self.to_screen(pt[0], pt[1]) for pt in pts])
                    p.drawPolyline(poly)
                    self._draw_marker(p, (op["mx"], op["my"]), COLOR_ARC,
                                      selected=selected and self.owner.selected_part == "mid", hollow=True)
                else:
                    p.drawLine(self.to_screen(*previous[:2]), self.to_screen(*end[:2]))
                if show_flange:
                    # Dashed machine/flange preview. Native PTP is not simulated.
                    flange_start = machine_pose(previous, self.owner.project)
                    flange_end = machine_pose(end, self.owner.project)
                    if kind in ("arc", "bezier"):
                        try:
                            fpts = [flange_start] + [machine_pose(pt, self.owner.project)
                                                      for pt in path_samples(previous, op, step_mm=8.0)]
                            p.setPen(QPen(COLOR_FLANGE, 1.2, Qt.PenStyle.DashLine))
                            p.drawPolyline(QPolygonF([self.to_screen(*pt[:2]) for pt in fpts]))
                        except ValueError:
                            pass
                    elif kind != "ptp":
                        p.setPen(QPen(COLOR_FLANGE, 1.2, Qt.PenStyle.DashLine))
                        p.drawLine(self.to_screen(*flange_start[:2]), self.to_screen(*flange_end[:2]))
                    # Native PTP flange path is unknown; only its endpoint
                    # can be displayed as a valid candidate position.
                    if selected:
                        p.drawLine(self.to_screen(*end[:2]), self.to_screen(*flange_end[:2]))
                    self._draw_marker(p, flange_end, COLOR_FLANGE, hollow=True)
                self._draw_marker(p, end, color, selected=selected and self.owner.selected_part != "mid")
                p.setPen(QColor("#dae6ee") if selected else color)
                p.drawText(self.to_screen(*end[:2])+QPointF(8, -8), f"P{index+1}")
                previous = end
            elif kind == "delay":
                pt = self.to_screen(*previous[:2]) + QPointF(9, 12)
                p.setPen(COLOR_SELECT if selected else QColor("#bbc9d4"))
                p.drawText(pt, f"D{index+1}: {op['seconds']:g}s")
            elif kind == "wait_input":
                pt = self.to_screen(*previous[:2]) + QPointF(9, 25)
                p.setPen(QColor("#f18e8e"))
                p.drawText(pt, f"W{index+1}: IN{op['channel']}={op['level']}")
        if self.owner.arc_mid is not None:
            mid = self.owner.arc_mid
            p.setPen(QPen(COLOR_ARC, 2, Qt.PenStyle.DashLine))
            current = self.owner.insertion_start()
            p.drawLine(self.to_screen(*current[:2]), self.to_screen(*mid))
            self._draw_marker(p, mid, COLOR_ARC, hollow=True)
            if self._cursor_world is not None:
                p.drawLine(self.to_screen(*mid), self.to_screen(*self._cursor_world))
        p.setPen(QColor("#b4c4d3"))
        mask_r = (self.owner.project["start"]["r"] if not hasattr(self.owner, "mask_link_checkbox")
                  or self.owner.mask_link_checkbox.isChecked() else self.owner.mask_r_spin.value())
        p.drawText(12, 22, f"SR6 / TCP XY [mm] | nastroj: {tool['name']} | mapa Z={self.owner.project['start']['z']:.1f}, R={mask_r:.1f}")
        if self._cursor_world is not None:
            x, y = self._cursor_world
            p.drawText(12, self.height()-13, f"X {x:+.3f}    Y {y:+.3f}    | Zoom {self.zoom*100:.0f}%")
        p.end()

    def _hit(self, pt):
        hits = []
        origin = pose(self.owner.project["start"])
        hits.append(((self.to_screen(*origin[:2])-pt).manhattanLength(), -2, "start"))
        for index, op in enumerate(self.owner.project["operations"]):
            if op["type"] not in MOVE_TYPES and op["type"] not in ("text", "vector"):
                continue
            hits.append(((self.to_screen(op["x"], op["y"])-pt).manhattanLength(), index, "end"))
            if op["type"] == "arc":
                hits.append(((self.to_screen(op["mx"], op["my"])-pt).manhattanLength(), index, "mid"))
            if op["type"] == "bezier":
                for k in ("c1","c2"):
                    hits.append(((self.to_screen(op[k+'x'],op[k+'y'])-pt).manhattanLength(),index,k))
        dist, i, part = min(hits, key=lambda hit: hit[0])
        return (i, part) if dist < 16 else (None, None)

    def mousePressEvent(self, event):
        if event.button() in (Qt.MouseButton.MiddleButton, Qt.MouseButton.RightButton):
            self._pan_mouse = event.position()
            return
        if event.button() != Qt.MouseButton.LeftButton:
            return
        xy = self.snapped(self.to_world(event.position()))
        tool = self.owner.tool
        if tool != "select":
            if tool == "arc":
                if self.owner.arc_mid is None:
                    self.owner.arc_mid = xy
                    self.owner.info.setText("ARC: klikni na KONCOVY bod oblouku (prvni klik byl MID)")
                else:
                    middle = self.owner.arc_mid
                    self.owner.arc_mid = None
                    self.owner.add_motion("arc", xy, mid=middle)
            else:
                self.owner.add_motion(tool, xy)
            self.update()
            return
        index, part = self._hit(event.position())
        if index is None:
            self.owner.set_selection(-1)
            return
        self.owner.set_selection(index, part)
        self._drag = (index, part, copy.deepcopy(self.owner.project))
        self.update()

    def mouseMoveEvent(self, event):
        if self._pan_mouse is not None:
            delta = event.position() - self._pan_mouse
            self.pan_x += delta.x()
            self.pan_y += delta.y()
            self._pan_mouse = event.position()
            self.update()
            return
        self._cursor_world = self.snapped(self.to_world(event.position()))
        if self._drag is not None and event.buttons() & Qt.MouseButton.LeftButton:
            index, part, _ = self._drag
            x, y = self._cursor_world
            if index == -2:
                self.owner.project["start"]["x"] = x
                self.owner.project["start"]["y"] = y
            elif part == "mid":
                self.owner.project["operations"][index]["mx"] = x
                self.owner.project["operations"][index]["my"] = y
            elif part in ("c1","c2"):
                self.owner.project["operations"][index][part+'x'] = x
                self.owner.project["operations"][index][part+'y'] = y
            else:
                self.owner.project["operations"][index]["x"] = x
                self.owner.project["operations"][index]["y"] = y
        self.update()

    def mouseReleaseEvent(self, event):
        if event.button() in (Qt.MouseButton.MiddleButton, Qt.MouseButton.RightButton):
            self._pan_mouse = None
        if event.button() == Qt.MouseButton.LeftButton and self._drag:
            _, _, before = self._drag
            self._drag = None
            self.owner.finish_change(before, self.owner.selected, "Presun bodu")

    def wheelEvent(self, event):
        old_at_cursor = self.to_world(event.position())
        factor = 1.15 if event.angleDelta().y() > 0 else (1/1.15)
        self.zoom = min(12.0, max(.25, self.zoom * factor))
        new_s = self.scale()
        self.pan_x = event.position().x()-self.width()*.5-old_at_cursor[0]*new_s
        self.pan_y = event.position().y()-self.height()*.5+old_at_cursor[1]*new_s
        self.update()

    def reset_view(self):
        self.zoom = 1.0
        self.pan_x = self.pan_y = 0.0
        self.update()


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.project = new_project()
        # Reuse the application-wide tool library in newly created jobs only.
        try:
            raw = json.loads(self._library_file().read_text(encoding="utf-8"))
            if raw.get("schema") == 1:
                self.project["tools"] = normalize_tools(raw["tools"])
                self.project["active_tool"] = self.project["tools"][0]["name"]
        except (OSError, ValueError, KeyError, TypeError):
            pass
        self.filename = None
        self.changed = False
        self.undo_history = []
        self.redo_history = []
        self.selected = -2
        self.selected_part = "end"
        self.arc_mid = None
        self.tool = "select"
        self._building = False
        self.setWindowTitle("SCARA CAD SR6 – v0.2.1")
        self.resize(1450, 900)
        self.setMinimumSize(980, 650)
        self._build_ui()
        self.refresh()
        self.statusBar().showMessage("Offline editor. Nepohybuje robotem; export vyzaduje --machine-coordinates.", 15000)

    def _build_ui(self):
        root = QSplitter(Qt.Orientation.Horizontal)
        self.setCentralWidget(root)
        tools = QWidget()
        left = QVBoxLayout(tools)
        left.setContentsMargins(10, 12, 10, 10)
        head = QLabel("SCARA CAD  /  SR6")
        head.setObjectName("mainHeading")
        left.addWidget(head)
        small = QLabel("SKETCH TOOLS")
        small.setObjectName("mutedLabel")
        left.addWidget(small)
        self.tool_group = QButtonGroup(self)
        self.tool_group.setExclusive(True)
        for label, name, hint in [
            ("▧  Vyber / editace", "select", "Vyber bod a tazenim ho premisti"),
            ("↗  Rapid / G0 (LINEAR)", "rapid", "Samostatny G0 znamena LINEAR, ne PTP"),
            ("⤳  PTP / G100", "ptp", "Skutecny kloubovy PTP, R lze menit"),
            ("╱  Linear / G1", "linear", "Pridej linearni pohyb"),
            ("◯  Circular / G2/G3", "arc", "Dva kliky: nejprve MID, potom END"),
            ("〰  Bézier / G1", "bezier", "Klikni koncovy bod, potom uprav kontrolni body"),
        ]:
            btn = QToolButton()
            btn.setText(label)
            btn.setToolTip(hint)
            btn.setCheckable(True)
            btn.setMinimumHeight(38)
            btn.setToolButtonStyle(Qt.ToolButtonStyle.ToolButtonTextOnly)
            btn.setAutoRaise(False)
            btn.setObjectName("toolButton")
            self.tool_group.addButton(btn)
            btn.clicked.connect(lambda _checked=False, t=name: self.change_tool(t))
            left.addWidget(btn)
            if name == "select": btn.setChecked(True)
        left.addSpacing(12)
        delay_btn = QPushButton("◷  Delay / G4")
        delay_btn.clicked.connect(self.add_delay)
        left.addWidget(delay_btn)
        rotate_btn = QPushButton("⟳  ROTATE / G101")
        rotate_btn.setToolTip("Otoceni kolem osy A4 (nikoliv kolem TCP)")
        rotate_btn.clicked.connect(self.add_rotate)
        left.addWidget(rotate_btn)
        text_btn = QPushButton("T  Gravirovaci TEXT")
        text_btn.setToolTip("Single-line font A-Z 0-9, Z gravirovani a bezpecna vyska")
        text_btn.clicked.connect(self.add_text)
        left.addWidget(text_btn)
        import_btn = QPushButton("⇩  Import DXF / SVG")
        import_btn.setToolTip("Jedna editovatelna operace Sketch, vektorove kontury uvnitr .scara")
        import_btn.clicked.connect(self.add_vector)
        left.addWidget(import_btn)
        spindle_on_btn = QPushButton("▶  Vreteno ON / M3")
        spindle_on_btn.clicked.connect(lambda: self.add_spindle(True))
        left.addWidget(spindle_on_btn)
        spindle_off_btn = QPushButton("■  Vreteno OFF / M5")
        spindle_off_btn.clicked.connect(lambda: self.add_spindle(False))
        left.addWidget(spindle_off_btn)
        wait_btn = QPushButton("◉  Wait input (navrh)")
        wait_btn.setToolTip("Aktualni runner zatim nema obecny G-code prikaz WAIT INPUT. Export je blokovan.")
        wait_btn.clicked.connect(self.add_wait)
        left.addWidget(wait_btn)
        left.addSpacing(16)
        self.snap_checkbox = QCheckBox("Prichytavat na mrizku")
        self.snap_checkbox.setChecked(True)
        left.addWidget(self.snap_checkbox)
        snap_line = QHBoxLayout()
        snap_line.addWidget(QLabel("Krok [mm]:"))
        self.snap_step = QSpinBox()
        self.snap_step.setRange(1, 100)
        self.snap_step.setValue(5)
        snap_line.addWidget(self.snap_step)
        left.addLayout(snap_line)
        fit = QPushButton("⤢  Zobrazit cely workspace")
        fit.clicked.connect(lambda: self.canvas.reset_view())
        left.addWidget(fit)
        left.addStretch(1)
        legend = QLabel("— G1: LINEAR / konstantni R\n--- G0: LINEAR rapid\n— G2/G3: oblouk\nG100: kloubovy PTP\nG101: rotace kolem A4\nT: jednocharove gravirovani")
        legend.setWordWrap(True)
        legend.setObjectName("mutedLabel")
        left.addWidget(legend)
        tools.setMinimumWidth(192)
        tools.setMaximumWidth(246)
        root.addWidget(tools)

        center = QWidget()
        center_lay = QVBoxLayout(center)
        center_lay.setContentsMargins(4, 9, 4, 9)
        self.info = QLabel("START upravis vyberem radku 00 ve Sketchi. Pravy/mid klik + tazeni = posun, kolecko = zoom.")
        self.info.setWordWrap(True)
        center_lay.addWidget(self.info)
        self.canvas = SketchCanvas(self)
        center_lay.addWidget(self.canvas, 1)
        root.addWidget(center)

        right = QWidget()
        right.setMinimumWidth(330)
        right.setMaximumWidth(510)
        right_lay = QVBoxLayout(right)
        right_lay.setContentsMargins(8, 7, 10, 8)
        # Tool profiles stay embedded in the .scara file; the optional
        # application-wide library is a separate JSON file in AppData.
        tool_group = QGroupBox("Nastroje / TCP offset (lokalni osy A4)")
        tool_layout = QVBoxLayout(tool_group)
        self.tool_combo = QComboBox()
        self.tool_combo.currentTextChanged.connect(self.select_tool)
        tool_layout.addWidget(self.tool_combo)
        buttons = QHBoxLayout()
        for name, callback in (("Novy", self.add_tool), ("Prejmen.", self.rename_tool),
                               ("Smazat", self.remove_tool)):
            button = QPushButton(name)
            button.clicked.connect(lambda _checked=False, fn=callback: fn())
            buttons.addWidget(button)
        tool_layout.addLayout(buttons)
        frm = QFormLayout()
        self.tool_spins = {}
        for key, caption, low, high in (("x", "Offset X [mm]", -500, 500),
                                        ("y", "Offset Y [mm]", -500, 500),
                                        ("z", "Offset Z [mm]", -500, 500),
                                        ("r", "Offset R [deg]", -360, 360)):
            sp = self._number(low, high, 3)
            sp.editingFinished.connect(self.commit_tool)
            frm.addRow(caption, sp)
            self.tool_spins[key] = sp
        tool_layout.addLayout(frm)
        self.tcp_compensate_box = QCheckBox("Kompenzovat TCP v G-codu (po overeni RHO4)")
        self.tcp_compensate_box.setToolTip("Zapni jen pokud RHO4 jiz neprovadi vlastni TCP kompenzaci. "
                                         "Jinak by se offset aplikoval dvakrat!")
        self.tcp_compensate_box.toggled.connect(self.commit_compensation)
        tool_layout.addWidget(self.tcp_compensate_box)
        libr = QHBoxLayout()
        for text, callback in (("Ulozit knihovnu", self.save_tool_library),
                               ("Nacist knihovnu", self.load_tool_library)):
            button = QPushButton(text)
            button.clicked.connect(lambda _checked=False, fn=callback: fn())
            libr.addWidget(button)
        tool_layout.addLayout(libr)
        preview_line = QHBoxLayout()
        self.mask_link_checkbox = QCheckBox("R mapy = START R")
        self.mask_link_checkbox.setChecked(True)
        self.mask_link_checkbox.toggled.connect(self.preview_r_link_changed)
        preview_line.addWidget(self.mask_link_checkbox)
        self.mask_r_spin = self._number(-360, 360, 1)
        self.mask_r_spin.setToolTip("R pro mapu dosazitelnosti, nezavisle na START bodu")
        self.mask_r_spin.setEnabled(False)
        self.mask_r_spin.valueChanged.connect(self.preview_r_changed)
        preview_line.addWidget(self.mask_r_spin)
        tool_layout.addLayout(preview_line)
        self.show_flange = QCheckBox("Zobrazit i osu A4 a jeji drahu")
        self.show_flange.setChecked(False)
        self.show_flange.toggled.connect(lambda _value: self.canvas.update())
        tool_layout.addWidget(self.show_flange)
        tool_note = QLabel("Sketch = TCP. Offset se pouziva interne; pri aktivnim nastroji zobrazujeme pouze TCP. "
                           "CAD nezna nastaveni TCP v RHO4; bez potvrzeni kompenzace "
                           "nenulovy offset zablokuje export.")
        tool_note.setObjectName("mutedLabel")
        tool_note.setWordWrap(True)
        tool_layout.addWidget(tool_note)
        right_lay.addWidget(tool_group)

        sketch_group = QGroupBox("Sketch / poradi operaci")
        sketch_lay = QVBoxLayout(sketch_group)
        self.op_list = QListWidget()
        self.op_list.setAlternatingRowColors(False)
        self.op_list.currentRowChanged.connect(self.selection_from_list)
        sketch_lay.addWidget(self.op_list)
        commands = QHBoxLayout()
        self.operation_buttons = []
        for text, callback in [("↑", lambda:self.reorder(-1)),("↓", lambda:self.reorder(1)),("Dupl.", self.duplicate_selected),("Smazat", self.delete_selected)]:
            button = QPushButton(text)
            button.clicked.connect(callback)
            commands.addWidget(button)
            self.operation_buttons.append(button)
        sketch_lay.addLayout(commands)
        self.start_move_checkbox = QCheckBox("Vlozit presun na START do G-codu (G0)")
        self.start_move_checkbox.setToolTip(
            "G0 = LINEAR ze skutecne polohy robota; neni homing ani PTP. "
            "Pristupova draha neni offline overena.")
        self.start_move_checkbox.toggled.connect(self.commit_start_move)
        sketch_lay.addWidget(self.start_move_checkbox)
        self.ignore_refill_checkbox = QCheckBox("Ignorovat varovani HYRING v CADu")
        self.ignore_refill_checkbox.setToolTip("Ignoruje jen odhad CADu. Runner stale kontroluje refill a muze pohyb odmitnout.")
        self.ignore_refill_checkbox.toggled.connect(self.commit_ignore_refill)
        sketch_lay.addWidget(self.ignore_refill_checkbox)
        right_lay.addWidget(sketch_group, 3)

        self.properties = QGroupBox("Parametry vybrane operace")
        self.prop_container = QVBoxLayout(self.properties)
        self.prop_stack = QStackedWidget()
        self.prop_container.addWidget(self.prop_stack)
        empty = QWidget(); emptl = QVBoxLayout(empty)
        emptl.addWidget(QLabel("Vyber operaci ve Sketchi nebo klikni na bod."))
        emptl.addStretch()
        self.prop_stack.addWidget(empty)
        start_page = QWidget()
        start_form = QFormLayout(start_page)
        self.start_spins = {}
        for axis in ("x", "y", "z", "r"):
            low, high = ((-900, 900) if axis in ("x", "y") else
                         ((-100, 300) if axis == "z" else (-360, 360)))
            widget = self._number(low, high, 3)
            widget.editingFinished.connect(self.commit_start)
            start_form.addRow(f"START {axis.upper()} [{'°' if axis == 'r' else 'mm'}]", widget)
            self.start_spins[axis] = widget
        explanation = QLabel("START = TCP XYZR. Vyber row 00 pro editaci. "
                             "Volba G0 na START je pod seznamem Sketch.")
        explanation.setWordWrap(True)
        start_form.addRow(explanation)
        self.prop_stack.addWidget(start_page)  # index 1
        move = QWidget(); frm = QFormLayout(move)
        self.prop_spins = {}
        for key, label, low, high, decimals in [
            ("x", "End X [mm]", -700,700,3),
            ("y", "End Y [mm]", -700,700,3),
            ("z", "End Z [mm]", -100,300,3),
            ("r", "End R [°]", -360,360,3),
            ("feed", "F [mm/min]", 0,100000,2),
            ("mx", "Mid X [mm]", -700,700,3),
            ("my", "Mid Y [mm]", -700,700,3),
        ]:
            sp = self._number(low, high, decimals)
            sp.editingFinished.connect(self.commit_properties)
            frm.addRow(label, sp)
            self.prop_spins[key] = sp
        self.ptp_speed_spin = self._number(0.01, 80, 2)
        self.ptp_speed_spin.setValue(2.0)
        self.ptp_speed_spin.editingFinished.connect(self.commit_properties)
        frm.addRow("PTP rychlost [%]", self.ptp_speed_spin)
        self.move_form = frm
        self.prop_stack.addWidget(move)
        delay = QWidget(); dform = QFormLayout(delay)
        self.delay_spin = self._number(0, 86400, 3)
        self.delay_spin.editingFinished.connect(self.commit_properties)
        dform.addRow("Cas [s]", self.delay_spin)
        self.prop_stack.addWidget(delay)
        wait = QWidget(); wform = QFormLayout(wait)
        self.wait_channel = QSpinBox(); self.wait_channel.setRange(0, 9999)
        self.wait_channel.editingFinished.connect(self.commit_properties)
        self.wait_level = QComboBox(); self.wait_level.addItems(["LOW (0)", "HIGH (1)"])
        self.wait_level.currentIndexChanged.connect(self.commit_properties)
        wform.addRow("Vstup", self.wait_channel)
        wform.addRow("Stav", self.wait_level)
        wform.addRow(QLabel("Zatim jen v projektu.\nExport G-codu neni podporovan."))
        self.prop_stack.addWidget(wait)
        rotate = QWidget(); rotate_form = QFormLayout(rotate)
        self.rotate_r_spin = self._number(-360, 360, 3)
        self.rotate_speed_spin = self._number(0.01, 80, 2)
        for sp in (self.rotate_r_spin, self.rotate_speed_spin):
            sp.editingFinished.connect(self.commit_properties)
        rotate_form.addRow("Cilove R [°]", self.rotate_r_spin)
        rotate_form.addRow("PTP rychlost [%]", self.rotate_speed_spin)
        rotate_form.addRow(QLabel("Rotace kolem pevne osy A4, nikoli kolem TCP."))
        self.prop_stack.addWidget(rotate)  # index 5
        text_page = QWidget(); text_form = QFormLayout(text_page)
        self.text_edit = QLineEdit()
        self.text_edit.setMaxLength(80)
        self.text_edit.editingFinished.connect(self.commit_properties)
        text_form.addRow("Text (A-Z, 0-9)", self.text_edit)
        self.text_spins = {}
        for key, label, low, high, digits in [
            ("x", "Kotva X [mm]", -900, 900, 3),
            ("y", "Kotva Y [mm]", -900, 900, 3),
            ("z", "Gravirovat Z [mm]", -100, 300, 3),
            ("angle_deg", "Natoceni textu XY [°]", -360, 360, 2),
            ("height", "Vyska pisma [mm]", .5, 150, 2),
            ("spacing", "Mezera [mm]", 0, 50, 2),
            ("safe_z", "Prejezdove Z [mm]", -100, 300, 3),
            ("feed", "Gravirovaci F [mm/min]", 0.01, 10000, 2),
        ]:
            sp = self._number(low,high,digits)
            sp.editingFinished.connect(self.commit_properties)
            text_form.addRow(label, sp)
            self.text_spins[key] = sp
        self.text_robot_r_label = QLabel("TCP R se prebira z predchozi operace.")
        self.text_robot_r_label.setObjectName("mutedLabel")
        text_form.addRow("Orientace TCP R", self.text_robot_r_label)
        text_form.addRow(QLabel("Jednocarove tahy, presuny s nastrojem zvednutym."))
        self.prop_stack.addWidget(text_page)  # index 6
        bezier_page=QWidget(); bform=QFormLayout(bezier_page)
        self.bezier_spins={}
        for k,label,lo,hi,dec in [
            ("c1x","C1 X [mm]",-900,900,3),("c1y","C1 Y [mm]",-900,900,3),
            ("c2x","C2 X [mm]",-900,900,3),("c2y","C2 Y [mm]",-900,900,3),
            ("x","End X [mm]",-900,900,3),("y","End Y [mm]",-900,900,3),
            ("z","Z [mm]",-100,300,3),("tolerance","Tolerance [mm]",0.005,5,3),
            ("feed","F [mm/min]",0.01,100000,2)]:
            w=self._number(lo,hi,dec); w.editingFinished.connect(self.commit_properties)
            bform.addRow(label,w); self.bezier_spins[k]=w
        bform.addRow(QLabel("Pocatek z predchoziho bodu; R zustava konstantni. G1 segmentace."))
        self.prop_stack.addWidget(bezier_page)  # index 7
        vector_page=QWidget(); vform=QFormLayout(vector_page)
        self.vector_name=QLabel(); vform.addRow("Zdroj",self.vector_name)
        self.vector_spins={}
        for k,label,lo,hi,dec in [
            ("x","Kotva X [mm]",-900,900,3),("y","Kotva Y [mm]",-900,900,3),
            ("z","Pracovni Z [mm]",-100,300,3),
            ("angle_deg","Natoceni XY [°]",-360,360,2),
            ("scale","Meritko",0.001,1000,4),
            ("safe_z","Prejezdove Z [mm]",-100,300,3),
            ("feed","F [mm/min]",0.01,100000,2)]:
            w=self._number(lo,hi,dec);w.editingFinished.connect(self.commit_properties)
            vform.addRow(label,w);self.vector_spins[k]=w
        vform.addRow(QLabel("R TCP dedi z predchozi operace; kontury jsou ulozeny v projektu.",wordWrap=True))
        self.prop_stack.addWidget(vector_page)  # index 8
        right_lay.addWidget(self.properties)

        self.tabs = QTabWidget()
        self.gcode_preview = QPlainTextEdit()
        self.gcode_preview.setReadOnly(True)
        self.gcode_preview.setLineWrapMode(QPlainTextEdit.LineWrapMode.NoWrap)
        self.gcode_preview.setMaximumBlockCount(20000)
        self.warnings_view = QPlainTextEdit(); self.warnings_view.setReadOnly(True)
        self.tabs.addTab(self.gcode_preview, "G-code")
        self.tabs.addTab(self.warnings_view, "Kontrola")
        right_lay.addWidget(self.tabs, 2)
        right_scroll = QScrollArea()
        right_scroll.setWidgetResizable(True)
        right_scroll.setMinimumWidth(350)
        right_scroll.setWidget(right)
        root.addWidget(right_scroll)
        root.setStretchFactor(0, 0)
        root.setStretchFactor(1, 1)
        root.setStretchFactor(2, 0)
        root.setSizes([205, 850, 400])

        tb = self.addToolBar("Projekt")
        tb.setMovable(False)
        actions = [
            ("Novy", "Ctrl+N", self.new_document),
            ("Otevrit", "Ctrl+O", self.open_document),
            ("Ulozit", "Ctrl+S", self.save_document),
            ("Ulozit jako", "Ctrl+Shift+S", self.save_document_as),
            ("Export G-code", "Ctrl+E", self.export_gcode),
            ("Zpet", "Ctrl+Z", self.undo),
            ("Znovu", "Ctrl+Y", self.redo),
        ]
        for text, shortcut, func in actions:
            a = QAction(text, self)
            a.setShortcut(QKeySequence(shortcut))
            a.triggered.connect(func)
            tb.addAction(a)
        delete_action = QAction("Delete selected", self)
        delete_action.setShortcut(QKeySequence("Delete"))
        delete_action.triggered.connect(self.delete_selected)
        self.addAction(delete_action)

    def _number(self, low, high, decimals):
        spin = QDoubleSpinBox()
        spin.setRange(low, high)
        spin.setDecimals(decimals)
        spin.setSingleStep(1.0)
        spin.setKeyboardTracking(False)
        spin.setAlignment(Qt.AlignmentFlag.AlignRight)
        return spin

    def _update_title(self):
        name = Path(self.filename).name if self.filename else "bez nazvu.scara"
        self.setWindowTitle(f"SCARA CAD SR6 – {name}{' *' if self.changed else ''}")

    def change_tool(self, tool):
        self.tool = tool
        self.arc_mid = None
        if tool == "arc": self.info.setText("ARC: prvni klik MID, druhy klik END. Z/R zustava konstantni.")
        elif tool == "rapid": self.info.setText("G0: rychloposuv po LINEAR draze. Neni to kloubove PTP!")
        elif tool == "ptp": self.info.setText("PTP G100: klikni na koncovy TCP; R a rychlost PTP uprav vpravo.")
        elif tool == "linear": self.info.setText("LINEAR: klikni na konec pohybu; Z a R se prevezmou z predchozi pozice.")
        else: self.info.setText("Klikni pro vyber, tazenim presun bod. Pravym tlacitkem posouvej plochu.")
        self.canvas.update()

    def insertion_index(self):
        if self.selected == -2:  # permanent START row
            return 0
        return self.selected+1 if self.selected >= 0 else len(self.project["operations"])

    def insertion_start(self):
        prior = pose(self.project["start"])
        for op in self.project["operations"][:self.insertion_index()]:
            try:
                prior = endpoint(prior, op, active_tool(self.project))
            except ValueError:
                pass
        return prior

    def add_motion(self, kind, xy, mid=None):
        ix = self.insertion_index()
        prior = self.insertion_start()
        op = {"type": kind, "x": xy[0], "y": xy[1], "z": prior[2],
              "r": prior[3], "feed": 600.0}
        if kind == "ptp": op["ptp_speed"] = 2.0
        if kind == "bezier":
            dx=xy[0]-prior[0];dy=xy[1]-prior[1]
            op.update({"c1x":prior[0]+dx/3,"c1y":prior[1]+dy/3+20,
                       "c2x":prior[0]+2*dx/3,"c2y":prior[1]+2*dy/3+20,
                       "tolerance":0.10})
        if kind == "arc":
            if mid is None:
                QMessageBox.warning(self, "Neplatny oblouk", "Oblouk potrebuje pruchozi bod MID.")
                return
            op["mx"], op["my"] = mid
            try:
                circle_from_3_points(prior, mid, (xy[0], xy[1]))
            except ValueError as exc:
                QMessageBox.warning(self, "Neplatny oblouk", str(exc))
                return
        before = copy.deepcopy(self.project)
        self.project["operations"].insert(ix, op)
        self.finish_change(before, ix, "Vlozena operace")

    def add_delay(self):
        before = copy.deepcopy(self.project)
        ix = self.insertion_index()
        self.project["operations"].insert(ix, {"type":"delay", "seconds": 1.0})
        self.finish_change(before, ix, "Delay")

    def add_rotate(self):
        previous = self.insertion_start()
        angle, ok = QInputDialog.getDouble(self, "ROTATE", "Cilove TCP R [°]",
                                            previous[3], -175, 175, 3)
        if not ok: return
        before = copy.deepcopy(self.project)
        ix = self.insertion_index()
        self.project["operations"].insert(ix, {"type":"rotate", "r":angle, "ptp_speed":2.0})
        self.finish_change(before, ix, "ROTATE: osa A4 zustava na miste")

    def add_text(self):
        value, ok = QInputDialog.getText(self, "Gravirovaci TEXT", "Text (A-Z, 0-9):",
                                         text="SCARA SR6")
        if not ok: return
        from scara_stroke_font import strokes
        before = copy.deepcopy(self.project)
        prev = self.insertion_start()
        op = {"type":"text", "text":value.upper(), "x":prev[0], "y":prev[1],
              "z":prev[2], "r":prev[3], "height":8.0, "spacing":1.5,
              "safe_z":min(195.0,prev[2]+10.0), "feed":120.0,
              "angle_deg": 0.0}
        try:
            strokes(op["text"], op["x"], op["y"], op["height"],
                    op["spacing"], op["angle_deg"])
            text_toolpath_targets(prev, op)
        except ValueError as exc:
            QMessageBox.warning(self, "Neplatny text", str(exc)); return
        ix = self.insertion_index()
        self.project["operations"].insert(ix, op)
        self.finish_change(before, ix, "Vlozen jednocarovy TEXT")

    def add_vector(self):
        filename,_=QFileDialog.getOpenFileName(self,"Importovat DXF / SVG","",
                                              "Vektory (*.svg *.dxf)")
        if not filename:return
        try:
            imported=import_vectors(filename,tolerance=0.10)
            prev=self.insertion_start()
            op={"type":"vector","source":imported['source'],"paths":imported['paths'],
                "skipped":imported['skipped'],"x":prev[0],"y":prev[1],"z":prev[2],
                "safe_z":min(195.0,prev[2]+10),"angle_deg":0.0,
                "scale":1.0,"feed":120.0}
            vector_toolpath_targets(prev,op)
        except Exception as exc:
            QMessageBox.warning(self,"Import se nezdaril",str(exc));return
        before=copy.deepcopy(self.project);ix=self.insertion_index()
        self.project['operations'].insert(ix,op)
        self.finish_change(before,ix,"Import: "+imported['source'])
        if imported['skipped']:
            QMessageBox.warning(self,"Vynechane prvky",
                                "Neimportovane typy: "+str(imported['skipped']))

    def commit_ignore_refill(self, enabled):
        if self._building:return
        before=copy.deepcopy(self.project)
        self.project['ignore_refill_warning']=bool(enabled)
        self.finish_change(before,self.selected,"HYRING: zmenena volba CAD varovani")

    def add_spindle(self, enabled):
        before = copy.deepcopy(self.project)
        ix = self.insertion_index()
        self.project["operations"].insert(ix, {"type":"spindle", "enabled":bool(enabled)})
        self.finish_change(before, ix, "M3" if enabled else "M5")

    def add_wait(self):
        before = copy.deepcopy(self.project)
        ix = self.insertion_index()
        self.project["operations"].insert(ix, {"type":"wait_input", "channel":611, "level":1})
        self.finish_change(before, ix, "Wait input - zatim neni exportovatelny")
        self.tabs.setCurrentIndex(1)

    def selection_from_list(self, row):
        if self._building:
            return
        self.selected = (-2 if row == 0 else row - 1 if row > 0 else -1)
        self.selected_part = "end"
        self.load_properties()
        self.canvas.update()

    def set_selection(self, idx, part="end"):
        self.selected = idx
        self.selected_part = part
        self._building = True
        self.op_list.setCurrentRow(0 if idx == -2 else idx + 1 if idx >= 0 else -1)
        self._building = False
        self.load_properties()
        self.canvas.update()

    def load_properties(self):
        self._building = True
        try:
            for k, sp in self.start_spins.items():
                sp.setValue(self.project["start"][k])
            self.start_move_checkbox.setChecked(self.project["move_to_start"])
            self.tool_combo.clear()
            self.tool_combo.addItems([t["name"] for t in self.project["tools"]])
            self.tool_combo.setCurrentText(self.project["active_tool"])
            if hasattr(self, "show_flange"):
                using_tool = _has_tool_offset(active_tool(self.project))
                self.show_flange.setVisible(True)
            for k, sp in self.tool_spins.items():
                sp.setValue(active_tool(self.project)[k])
            self.tcp_compensate_box.setChecked(self.project["tcp_compensate"])
            self.ignore_refill_checkbox.setChecked(self.project.get('ignore_refill_warning',False))
            if self.mask_link_checkbox.isChecked():
                self.mask_r_spin.setValue(self.project["start"]["r"])
            for button in self.operation_buttons:
                button.setEnabled(0 <= self.selected < len(self.project["operations"]))
            if self.selected == -2:
                self.prop_stack.setCurrentIndex(1)
                return
            if not 0 <= self.selected < len(self.project["operations"]):
                self.prop_stack.setCurrentIndex(0)
                return
            op = self.project["operations"][self.selected]
            kind = op["type"]
            if kind == 'bezier':
                self.prop_stack.setCurrentIndex(7)
                for k,sp in self.bezier_spins.items():sp.setValue(op[k])
            elif kind == 'vector':
                self.prop_stack.setCurrentIndex(8)
                self.vector_name.setText(f"{op['source']} ({len(op['paths'])} kontur)")
                for k,sp in self.vector_spins.items():sp.setValue(op[k])
            elif kind in MOVE_TYPES:
                self.prop_stack.setCurrentIndex(2)
                for k, sp in self.prop_spins.items():
                    visible = (k not in ("mx","my") or kind == "arc")
                    label = self.move_form.labelForField(sp)
                    if label is not None: label.setVisible(visible)
                    sp.setVisible(visible)
                    if k in op: sp.setValue(op[k])
                feed_sp = self.prop_spins["feed"]
                show_feed = kind in ("linear", "arc")
                feed_sp.setVisible(show_feed)
                feed_label = self.move_form.labelForField(feed_sp)
                if feed_label is not None: feed_label.setVisible(show_feed)
                feed_sp.setEnabled(show_feed)
                self.ptp_speed_spin.setVisible(kind == "ptp")
                label = self.move_form.labelForField(self.ptp_speed_spin)
                if label: label.setVisible(kind == "ptp")
                if kind == "ptp": self.ptp_speed_spin.setValue(op["ptp_speed"])
            elif kind == "rotate":
                self.prop_stack.setCurrentIndex(5)
                self.rotate_r_spin.setValue(op["r"])
                self.rotate_speed_spin.setValue(op["ptp_speed"])
            elif kind == "text":
                self.prop_stack.setCurrentIndex(6)
                self.text_edit.setText(op["text"])
                for k, sp in self.text_spins.items():
                    sp.setValue(op.get(k, 0.0))
                prior = pose(self.project["start"])
                for step in self.project["operations"][:self.selected]:
                    try: prior = endpoint(prior, step, active_tool(self.project))
                    except ValueError: break
                self.text_robot_r_label.setText(f"{prior[3]:.3f}° (z predchoziho kroku)")
            elif kind == "delay":
                self.prop_stack.setCurrentIndex(3)
                self.delay_spin.setValue(op["seconds"])
            elif kind == "spindle":
                self.prop_stack.setCurrentIndex(0)
            else:
                self.prop_stack.setCurrentIndex(4)
                self.wait_channel.setValue(op["channel"])
                self.wait_level.setCurrentIndex(op["level"])
        finally:
            self._building = False

    def _library_file(self):
        base = Path(QStandardPaths.writableLocation(QStandardPaths.StandardLocation.AppDataLocation))
        if not str(base) or str(base) == ".":
            base = Path.home() / ".scara_cad"
        return base / "scara_tools.json"

    def save_tool_library(self):
        filename = self._library_file()
        try:
            filename.parent.mkdir(parents=True, exist_ok=True)
            data = {"schema": 1, "tools": normalize_tools(self.project["tools"])}
            filename.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
            self.statusBar().showMessage(f"Knihovna ulozena: {filename}", 6000)
            QMessageBox.information(self, "Knihovna ulozena", f"Nastroje ulozeny do:\n{filename}")
        except Exception as exc:
            QMessageBox.critical(self, "Knihovna", str(exc))

    def load_tool_library(self):
        filename = self._library_file()
        try:
            raw = json.loads(filename.read_text(encoding="utf-8"))
            if raw.get("schema") != 1:
                raise ValueError("Neznamy format knihovny.")
            tools = normalize_tools(raw["tools"])
        except Exception as exc:
            QMessageBox.warning(self, "Knihovna", f"Knihovnu nelze nacist:\n{filename}\n{exc}")
            return
        if QMessageBox.question(self, "Nacist knihovnu", "Nahradit seznam nastroju v projektu globalni knihovnou?",
                                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                                QMessageBox.StandardButton.No) != QMessageBox.StandardButton.Yes:
            return
        before = copy.deepcopy(self.project)
        self.project["tools"] = tools
        if self.project["active_tool"] not in [t["name"] for t in tools]:
            self.project["active_tool"] = tools[0]["name"]
        self.finish_change(before, self.selected, "Nactena knihovna nastroju", re_mask=True)

    def select_tool(self, name):
        if self._building or not name or name == self.project["active_tool"]:
            return
        before = copy.deepcopy(self.project)
        self.project["active_tool"] = name
        self.finish_change(before, self.selected, "Zmenen aktivni nastroj", re_mask=True)

    def add_tool(self):
        name, ok = QInputDialog.getText(self, "Novy nastroj", "Jmeno noveho nastroje:")
        if not ok:
            return
        before = copy.deepcopy(self.project)
        try:
            create_tool_profile(self.project, name)
        except (ValueError, KeyError, TypeError) as exc:
            QMessageBox.warning(self, "Nastroj nelze vytvorit", str(exc))
            return
        self.finish_change(before, self.selected,
                           "Novy nastroj pridan do projektu; pro globalni knihovnu klikni Ulozit knihovnu.",
                           re_mask=True)

    def rename_tool(self):
        name, ok = QInputDialog.getText(self, "Prejmenovat", "Novy nazev:",
                                        text=self.project["active_tool"])
        if not ok:
            return
        before = copy.deepcopy(self.project)
        try:
            rename_tool_profile(self.project, name)
        except (ValueError, KeyError, TypeError) as exc:
            QMessageBox.warning(self, "Nastroj nelze prejmenovat", str(exc))
            return
        self.finish_change(before, self.selected, "Prejmenovan nastroj", re_mask=True)

    def remove_tool(self):
        before = copy.deepcopy(self.project)
        try:
            remove_tool_profile(self.project)
        except (ValueError, KeyError, TypeError) as exc:
            QMessageBox.information(self, "Nastroj nelze smazat", str(exc))
            return
        self.finish_change(before, self.selected, "Smazan nastroj", re_mask=True)

    def commit_tool(self):
        if self._building:
            return
        before = copy.deepcopy(self.project)
        tool = active_tool(self.project)
        for k, sp in self.tool_spins.items():
            tool[k] = sp.value()
        self.finish_change(before, self.selected, "TCP offset upraven", re_mask=True)

    def commit_compensation(self, checked):
        if self._building:
            return
        if checked and QMessageBox.warning(
                self, "Overeni TCP v RHO4",
                "POTVRD: RHO4 nema aktivni vlastni kompenzaci TCP a G-code XYZR "
                "interpretuje jako osu A4.\n\nPokud to neni overeno, "
                "mohlo by dojit k dvojite kompenzaci a nebezpecnemu pohybu. "
                "Opravdu zapnout CAD kompenzaci?",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                QMessageBox.StandardButton.No) != QMessageBox.StandardButton.Yes:
            self._building = True
            self.tcp_compensate_box.setChecked(False)
            self._building = False
            return
        before = copy.deepcopy(self.project)
        self.project["tcp_compensate"] = bool(checked)
        self.finish_change(before, self.selected, "TCP kompenzace zmenena", re_mask=True)

    def preview_r_link_changed(self, checked):
        self.mask_r_spin.setEnabled(not checked)
        if checked:
            self.mask_r_spin.setValue(self.project["start"]["r"])
        self.canvas.rebuild_reachability()

    def preview_r_changed(self, _value):
        if not self._building and not self.mask_link_checkbox.isChecked():
            self.canvas.rebuild_reachability()

    def commit_start(self):
        if self._building: return
        before = copy.deepcopy(self.project)
        for k, sp in self.start_spins.items():
            self.project["start"][k] = sp.value()
        self.finish_change(before, self.selected, "START upraven", re_mask=True)

    def commit_start_move(self, enabled):
        if self._building: return
        before = copy.deepcopy(self.project)
        self.project["move_to_start"] = bool(enabled)
        self.finish_change(before, self.selected, "Presun na START " + ("zapnut" if enabled else "vypnut"))

    def commit_properties(self, *_):
        if self._building or not 0 <= self.selected < len(self.project["operations"]): return
        before = copy.deepcopy(self.project)
        op = self.project["operations"][self.selected]
        if op['type']=='bezier':
            for k,sp in self.bezier_spins.items():op[k]=sp.value()
        elif op['type']=='vector':
            for k,sp in self.vector_spins.items():op[k]=sp.value()
        elif op["type"] in MOVE_TYPES:
            for k, sp in self.prop_spins.items():
                if k in op and (k != "feed" or op["type"] not in ("rapid", "ptp")):
                    op[k] = sp.value()
            if op["type"] == "ptp": op["ptp_speed"] = self.ptp_speed_spin.value()
        elif op["type"] == "rotate":
            op["r"] = self.rotate_r_spin.value()
            op["ptp_speed"] = self.rotate_speed_spin.value()
        elif op["type"] == "text":
            candidate = {**op, **{k: sp.value() for k, sp in self.text_spins.items()},
                         "text": self.text_edit.text().upper()}
            try:
                # A text object must remain editable even when following moves
                # are temporarily invalid; reject malformed glyphs right away.
                from scara_stroke_font import strokes
                strokes(candidate["text"], candidate["x"], candidate["y"],
                        candidate["height"], candidate["spacing"],
                        candidate["angle_deg"])
                op.update(candidate)
            except ValueError as exc:
                QMessageBox.warning(self, "Neplatny text", str(exc))
                return
        elif op["type"] == "delay":
            op["seconds"] = self.delay_spin.value()
        elif op["type"] == "spindle":
            return
        else:
            op["channel"] = self.wait_channel.value()
            op["level"] = self.wait_level.currentIndex()
        self.finish_change(before, self.selected, "Upraveny parametry")

    def finish_change(self, before, selected, message="", re_mask=False):
        if self.project != before:
            self.undo_history.append(before)
            if len(self.undo_history) > 100: self.undo_history.pop(0)
            self.redo_history.clear()
            self.changed = True
            if (self.project["start"] != before["start"] or
                self.project["tools"] != before["tools"] or
                self.project["active_tool"] != before["active_tool"]):
                re_mask = True
        self.selected = selected
        if re_mask: self.canvas.rebuild_reachability()
        self.refresh()
        if message: self.statusBar().showMessage(message, 5000)

    def refresh(self):
        self._building = True
        try:
            self.op_list.clear()
            current = pose(self.project["start"])
            self.op_list.addItem(QListWidgetItem(
                f"00  START      X{current[0]:.1f} Y{current[1]:.1f} "
                f"Z{current[2]:.1f} R{current[3]:.1f}°"))
            for no, op in enumerate(self.project["operations"], 1):
                kind = op["type"]
                if kind == 'vector':
                    label='IMPORT'; description=f"{op['source'][:20]}  {len(op['paths'])} kontur"
                elif kind == 'bezier':
                    label='BEZIER';description=f"X{op['x']:.1f} Y{op['y']:.1f} tol{op['tolerance']:g}"
                elif kind in MOVE_TYPES:
                    label = ("PTP G100" if kind == "ptp" else ("RAPID G0" if kind == "rapid" else
                             ("LINEAR" if kind == "linear" else "ARC G2/G3")))
                    description = f"X{op['x']:.1f} Y{op['y']:.1f} Z{op['z']:.1f} R{op['r']:.1f}°"
                elif kind == "rotate":
                    label = "ROTATE"; description = f"R{op['r']:.2f}° P{op['ptp_speed']:.2f}%"
                elif kind == "text":
                    label = "TEXT"; description = f"{op['text'][:22]}  H{op['height']:g}  rot{op.get('angle_deg',0):g}°"
                elif kind == "spindle":
                    label = "M3 ON" if op["enabled"] else "M5 OFF"
                    description = "Vreteno (vyzaduje --arm-io)"
                elif kind == "delay":
                    label = "DELAY"; description = f"{op['seconds']:.3f} s"
                else:
                    label = "WAIT IN*"; description = f"I{op['channel']}={op['level']}"
                it = QListWidgetItem(f"{no:02d}  {label:<10}  {description}")
                if kind == "wait_input": it.setForeground(QColor("#f6a1a1"))
                self.op_list.addItem(it)
                try:
                    current = endpoint(current, op, active_tool(self.project))
                except ValueError:
                    pass
            self.op_list.setCurrentRow(0 if self.selected == -2 else
                                       self.selected + 1 if 0 <= self.selected < len(self.project["operations"]) else -1)
        finally:
            self._building = False
        self.load_properties()
        self.canvas.update()
        errors = validate_project(self.project)
        try:
            windows=refill_report(self.project) if not errors else []
        except (ValueError,OverflowError): windows=[]
        refill_info=("\n\nHYRING: " + (f"{len(windows)} rizikovych refill oken. Minimum "
                     f"{min(w['window_s'] for w in windows):.3f}s (limit 0.500s). "
                     + ("IGNOROVANO v CADu, ale runner stale kontroluje." if self.project.get('ignore_refill_warning') else "Zmensi F / zmen toleranci nebo zaskrtni Ignorovat."))
                     if windows else "\n\nHYRING: bez problemu v odhadu refill (nenahrazuje driver).")
        tool = active_tool(self.project)
        self.warnings_view.setPlainText(
            f"Nastroj: {tool['name']} | TCP kompenzace | "
            f"Kompenzace v CADu: {'ANO' if self.project['tcp_compensate'] else 'NE'}\n"
            "BEZPECNOST: bez overeni konfigurace TCP v RHO4 nelze zarucit spravny pohyb.\n\n" +
            "OFFLINE kontrola (nenahrazuje kontrolu skutecneho robota).\n" +
            ("G0 na START je zapnute: offline kontrola NEZNA skutecnou vychozi polohu ani pristupovou drahu.\n\n"
             if self.project["move_to_start"] else
             "G0 na START je vypnute: robot musi pred spustenim byt v poloze OSY A4 odvozene ze START TCP.\n\n") +
            ("\n".join(("START" if i < 0 else f"Operace {i+1}") + ": " + e for i, e in errors)
             if errors else "Bez nalezenych chyb v dostupnem modelu drahy.\n\nPOZOR: kolize, realna konfigurace kloubu, pracovni nula a rychlosti nejsou overeny.") + refill_info)
        try:
            self.gcode_preview.setPlainText(generate_gcode(self.project))
        except ValueError as exc:
            self.gcode_preview.setPlainText("; NELZE EXPORTOVAT\n; " + str(exc).replace("\n", "\n; "))
        self.tabs.setTabText(1, f"Kontrola ({len(errors)+len(windows)})")
        self._update_title()

    def reorder(self, direction):
        ix = self.selected
        other = ix+direction
        if not (0 <= ix < len(self.project["operations"]) and 0 <= other < len(self.project["operations"])):
            return
        before = copy.deepcopy(self.project)
        ops = self.project["operations"]
        ops[ix], ops[other] = ops[other], ops[ix]
        self.finish_change(before, other, "Zmeneno poradi")

    def duplicate_selected(self):
        ix = self.selected
        if not 0 <= ix < len(self.project["operations"]): return
        before = copy.deepcopy(self.project)
        self.project["operations"].insert(ix+1, copy.deepcopy(self.project["operations"][ix]))
        self.finish_change(before, ix+1, "Duplikovana operace")

    def delete_selected(self):
        ix = self.selected
        if not 0 <= ix < len(self.project["operations"]): return
        before = copy.deepcopy(self.project)
        del self.project["operations"][ix]
        self.finish_change(before, min(ix, len(self.project["operations"])-1), "Smazana operace")

    def undo(self):
        if not self.undo_history: return
        self.redo_history.append(copy.deepcopy(self.project))
        self.project = self.undo_history.pop()
        self.changed = True
        self.selected = -1
        self.arc_mid = None
        self.canvas.rebuild_reachability()
        self.refresh()

    def redo(self):
        if not self.redo_history: return
        self.undo_history.append(copy.deepcopy(self.project))
        self.project = self.redo_history.pop()
        self.changed = True
        self.selected = -1
        self.arc_mid = None
        self.canvas.rebuild_reachability()
        self.refresh()

    def maybe_discard(self):
        if not self.changed: return True
        response = QMessageBox.question(self, "Neulozene zmeny", "Projekt obsahuje neulozene zmeny. Zahodit je?",
                                        QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                                        QMessageBox.StandardButton.No)
        return response == QMessageBox.StandardButton.Yes

    def new_document(self):
        if not self.maybe_discard(): return
        self.project = new_project()
        # Reuse the application-wide tool library in newly created jobs only.
        try:
            raw = json.loads(self._library_file().read_text(encoding="utf-8"))
            if raw.get("schema") == 1:
                self.project["tools"] = normalize_tools(raw["tools"])
                self.project["active_tool"] = self.project["tools"][0]["name"]
        except (OSError, ValueError, KeyError, TypeError):
            pass
        self.filename = None
        self.changed = False
        self.selected = -2
        self.undo_history.clear(); self.redo_history.clear()
        self.arc_mid = None
        self.canvas.rebuild_reachability(); self.refresh()

    def open_document(self):
        if not self.maybe_discard(): return
        filename, _ = QFileDialog.getOpenFileName(self, "Otevrit SCARA projekt", "", "SCARA projekt (*.scara);;JSON (*.json)")
        if not filename: return
        try:
            project = load_project(filename)
        except Exception as exc:
            QMessageBox.critical(self, "Nelze otevrit", str(exc)); return
        self.project = project
        self.filename = filename
        self.changed = False
        self.undo_history.clear(); self.redo_history.clear()
        self.selected = -2; self.arc_mid = None
        self.canvas.rebuild_reachability(); self.refresh()

    def save_document(self):
        if not self.filename: return self.save_document_as()
        try: save_project(self.filename, self.project)
        except Exception as exc:
            QMessageBox.critical(self, "Chyba ulozeni", str(exc)); return False
        self.changed = False
        self._update_title()
        self.statusBar().showMessage("Projekt ulozen", 5000)
        return True

    def save_document_as(self):
        filename, _ = QFileDialog.getSaveFileName(self, "Ulozit projekt", self.filename or "projekt.scara", "SCARA projekt (*.scara)")
        if not filename: return False
        if not Path(filename).suffix: filename += ".scara"
        self.filename = filename
        return self.save_document()

    def export_gcode(self):
        try: code = generate_gcode(self.project)
        except ValueError as exc:
            self.tabs.setCurrentIndex(1)
            QMessageBox.warning(self, "Export zamitnut", str(exc))
            return
        start_warning = (
            "VOLBA PRESUNU NA START JE ZAPNUTA: G0 provede LINEAR pohyb ze skutecne aktualni polohy "
            "robota na START XYZR. Pristupovou cestu CAD nezna ani nekontroluje; "
            "pred skutecnym pouzitim ji over v simulatoru!\n\n"
            if self.project["move_to_start"] else
            "Presun na START je vypnuty. Pred spustenim musi robot skutecne stat na START XYZR!\n\n"
        )
        tool = active_tool(self.project)
        tool_note = (f"Aktivni nastroj: {tool['name']}; offset X={tool['x']:.3f} Y={tool['y']:.3f} "
                     f"Z={tool['z']:.3f} R={tool['r']:.3f}.\n"
                     f"Kompenzace v CADu: {'ANO' if self.project['tcp_compensate'] else 'NE'}.\n"
                     "DULEZITE: RHO4 nesmi aplikovat stejny TCP offset podruhe. "
                     "Referencni ramec RHO4 jeste nebyl fyzicky overen.\n\n")
        notes = (tool_note + "POZOR: export pouziva STROJNI souradnice. V runneru je nutne spustit s --machine-coordinates.\n\n"
                 + start_warning +
                 "G0 = LINEAR, G100 = kloubovy PTP, G101 = rotace kolem A4. Pouzij AKTUALIZOVANY runner z tohoto balicku.\n"
                 "M3/M5 vyzaduji explicitni --arm-io a overene zapojeni vystupu!\n\n"
                 "Ulozit G-code?")
        if QMessageBox.question(self, "Bezpecnostni upozorneni", notes,
                                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                                QMessageBox.StandardButton.No) != QMessageBox.StandardButton.Yes:
            return
        suggested = str(Path(self.filename).with_suffix(".gcode")) if self.filename else "scara_program.gcode"
        filename, _ = QFileDialog.getSaveFileName(self, "Export G-code", suggested, "G-code (*.gcode *.nc *.tap)")
        if not filename: return
        if not Path(filename).suffix: filename += ".gcode"
        try: Path(filename).write_text(code, encoding="utf-8")
        except Exception as exc:
            QMessageBox.critical(self, "Chyba exportu", str(exc)); return
        self.statusBar().showMessage(f"G-code exportovan: {filename}", 10000)
        QMessageBox.information(self, "Export hotov", f"Ulozeno:\n{filename}\n\nV runneru je nutne pouzit --machine-coordinates.")

    def closeEvent(self, event):
        if self.maybe_discard(): event.accept()
        else: event.ignore()


def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    app.setStyleSheet("""
      QMainWindow, QWidget { background: #17212c; color: #e8edf5; font-size: 12px; }
      QToolBar { background: #223241; border-bottom: 1px solid #3f5366; spacing: 7px; padding: 6px; }
      QToolButton, QPushButton { background: #2a3d50; color: #ecf1f8; border: 1px solid #435b70;
          border-radius: 5px; padding: 6px 9px; }
      QToolButton:hover, QPushButton:hover { background: #36546a; }
      QToolButton:checked { background: #275e70; border-color: #5ab3c8; }
      QToolButton#toolButton { text-align: left; }
      QGroupBox { border: 1px solid #3a4f60; border-radius: 5px; margin-top: 11px; padding: 8px 4px 4px 4px; }
      QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; color: #a7d4e7; }
      QDoubleSpinBox, QSpinBox, QComboBox, QListWidget, QPlainTextEdit {
          border: 1px solid #395467; border-radius: 3px; padding: 4px;
          background: #0f1b26; color: #ecf1f8; }
      QListWidget::item:selected { background: #28667d; }
      QTabBar::tab { background: #2c4051; padding: 7px 10px; }
      QTabBar::tab:selected { background: #316b81; }
      QLabel#mainHeading { font-size: 17px; font-weight: 700; color: #76d0e4; }
      QLabel#mutedLabel { color: #a1b2c1; }
      QStatusBar { background: #0d1923; }
      QSplitter::handle { background: #3b5364; }
      QScrollArea { border: none; background: #17212c; }
    """)
    window = MainWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
