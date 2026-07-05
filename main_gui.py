import sys
import os
import json
import ast
import csv
import subprocess

import numpy as np
import pandas as pd

from PyQt6.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout,
                             QFormLayout, QLineEdit, QPushButton, QLabel,
                             QFrame, QCheckBox, QGroupBox, QScrollArea,
                             QTabWidget, QComboBox, QDialog, QDialogButtonBox,
                             QGridLayout, QFileDialog, QSizePolicy, QListWidget,
                             QListWidgetItem)
from PyQt6.QtCore import Qt, QProcess, QThread, pyqtSignal
from PyQt6.QtGui import QDoubleValidator

import thrustcurve

from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

# ---------------------------------------------------------------------------
# Paths / constants
# ---------------------------------------------------------------------------
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
MOTORS_DIR = os.path.join(BASE_DIR, "motors")
DATA_DIR = os.path.join(BASE_DIR, "data")
CONFIG_PATH = os.path.join(BASE_DIR, "rocket_config.json")
MAIN_PY = os.path.join(BASE_DIR, "main.py")
VISUALIZER_PY = os.path.join(BASE_DIR, "visualizer.py")
FLIGHT_CSV = os.path.join(DATA_DIR, "flight_data.csv")
FLIGHT_REPORT = os.path.join(DATA_DIR, "flight_report.json")

# The keys that main.py depends on, in order. The original 15 must be written
# exactly; "Drag Coefficient" is optional (main.py defaults it to 0.5).
CONFIG_KEYS = [
    "Total Length (m)", "Dry Mass (kg)", "Propellant Mass (kg)", "Diameter (m)",
    "CP Dist (m)", "CG Dist (m)", "Fin Area (m²)", "Drag Coefficient", "Motor File",
    "Rail Angle (°)", "Rail Length (m)", "Wind Speed (m/s)", "Air Density (kg/m³)",
    "Chute Dia (m)", "Chute Cd", "Chute Delay (s)",
]

# Per-parameter help text (units + what it does)
TOOLTIPS = {
    "Total Length (m)": "Overall length of the airframe, nose tip to tail (metres).",
    "Dry Mass (kg)": "Vehicle mass with an empty motor (no propellant), in kilograms.",
    "Propellant Mass (kg)": "Mass of propellant burned during the motor's thrust phase (kg).",
    "Diameter (m)": "Maximum body-tube diameter, used as one caliber for stability (m).",
    "CP Dist (m)": "Centre of Pressure distance from the nose tip (m).",
    "CG Dist (m)": "Centre of Gravity distance from the nose tip (m).",
    "Fin Area (m²)": "Total exposed planform area of one fin set (square metres).",
    "Drag Coefficient": "Airframe drag coefficient. Typical slender model rocket: 0.4-0.6.",
    "Motor File": "Thrust-curve CSV in the motors/ folder. Use BROWSE MOTORS to pick.",
    "Rail Angle (°)": "Launch rail elevation from horizontal (90 deg = straight up).",
    "Rail Length (m)": "Length of the guide rail; rocket is constrained until it clears (m).",
    "Wind Speed (m/s)": "Steady horizontal wind speed for drift estimation (m/s).",
    "Air Density (kg/m³)": "Sea-level air density used by the drag model (kg/m³).",
    "Chute Dia (m)": "Deployed parachute canopy diameter (m).",
    "Chute Cd": "Parachute drag coefficient (typical flat sheet ~0.75, dome ~1.5).",
    "Chute Delay (s)": "Delay after apogee before the parachute deploys (seconds).",
}

# ---------------------------------------------------------------------------
# Theme
# ---------------------------------------------------------------------------
BG = "#09090b"
PANEL = "#18181b"
CARD = "#1d1d20"
BORDER = "#27272a"
MUTED = "#a1a1aa"
FAINT = "#71717a"
TEXT = "#f4f4f5"
BLUE = "#3b82f6"
GREEN = "#10b981"
AMBER = "#f59e0b"
RED = "#ef4444"

MODERN_STYLE = f"""
QWidget {{ background-color: {BG}; color: {MUTED}; font-family: 'Inter', 'Segoe UI', sans-serif; font-size: 12px; }}
QGroupBox {{ border: 1px solid {BORDER}; border-radius: 8px; margin-top: 16px; padding: 12px 10px 10px 10px; font-weight: bold; color: {TEXT}; }}
QGroupBox::title {{ subcontrol-origin: margin; subcontrol-position: top left; left: 10px; padding: 0 4px; }}
QLineEdit {{ background: transparent; border: none; border-bottom: 1px solid {BORDER}; padding: 4px; color: {BLUE}; font-weight: bold; }}
QLineEdit:focus {{ border-bottom: 2px solid {BLUE}; }}
QLineEdit[invalid="true"] {{ border-bottom: 2px solid {RED}; color: {RED}; }}
QPushButton {{ background-color: #1d4ed8; color: white; border-radius: 6px; padding: 8px; font-size: 11px; font-weight: bold; }}
QPushButton:hover {{ background-color: {BLUE}; }}
QPushButton:disabled {{ background-color: {BORDER}; color: {FAINT}; }}
QPushButton#LaunchBtn {{ background-color: #059669; }}
QPushButton#LaunchBtn:hover {{ background-color: {GREEN}; }}
QPushButton#RunBtn {{ background-color: #1d4ed8; font-size: 13px; padding: 11px; }}
QPushButton#GhostBtn {{ background-color: {BORDER}; color: {TEXT}; }}
QPushButton#GhostBtn:hover {{ background-color: #3f3f46; }}
QLabel#Header {{ color: white; font-size: 15px; font-weight: 800; }}
QCheckBox {{ spacing: 6px; }}
QScrollArea {{ border: none; }}
QToolTip {{ background-color: {PANEL}; color: {TEXT}; border: 1px solid {BORDER}; padding: 4px; }}
"""


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def parse_fin_points(text):
    """Safely parse '(x1,y1), (x2,y2), ...' into a list of [x, y] pairs."""
    parsed = ast.literal_eval(f"[{text}]")
    pts = []
    for p in parsed:
        if len(p) != 2:
            raise ValueError("each point needs exactly 2 numbers")
        pts.append([float(p[0]), float(p[1])])
    if len(pts) < 3:
        raise ValueError("need at least 3 points for a fin polygon")
    return pts


def parse_motor_file(path):
    """Parse a thrust-curve CSV. Returns a dict with metadata + summary stats."""
    name = os.path.splitext(os.path.basename(path))[0]
    times, thrusts = [], []
    header_seen = False
    with open(path, newline="") as f:
        reader = csv.reader(f)
        for row in reader:
            if not row:
                continue
            c0 = row[0].strip().lower()
            if c0 == "motor:" and len(row) > 1 and row[1].strip():
                name = row[1].strip()
                continue
            if c0.startswith("time"):
                header_seen = True
                continue
            # metadata rows we can ignore
            if c0 in ("contributor:", "details:", "comments:"):
                continue
            # data rows: two numeric columns
            try:
                t = float(row[0])
                thr = float(row[1])
            except (ValueError, IndexError):
                continue
            times.append(t)
            thrusts.append(thr)

    if len(times) < 2:
        raise ValueError("no thrust-curve data found")

    times = np.array(times)
    thrusts = np.array(thrusts)
    burn_time = float(times[-1] - times[0])
    # Impulse via trapezoidal integration
    impulse = float(np.trapezoid(thrusts, times)) if hasattr(np, "trapezoid") \
        else float(np.trapz(thrusts, times))
    avg_thrust = impulse / burn_time if burn_time > 0 else 0.0
    peak = float(thrusts.max())
    return {
        "name": name,
        "file": os.path.basename(path),
        "avg_thrust": avg_thrust,
        "peak_thrust": peak,
        "burn_time": burn_time,
        "impulse": impulse,
    }


def scan_motors():
    """Return a list of parsed motor dicts for every CSV in motors/."""
    out = []
    if not os.path.isdir(MOTORS_DIR):
        return out
    for fn in sorted(os.listdir(MOTORS_DIR)):
        if not fn.lower().endswith(".csv"):
            continue
        try:
            out.append(parse_motor_file(os.path.join(MOTORS_DIR, fn)))
        except Exception:
            # Still list the file even if we can't parse the curve
            out.append({"name": fn, "file": fn, "avg_thrust": 0.0,
                        "peak_thrust": 0.0, "burn_time": 0.0, "impulse": 0.0})
    return out


# ---------------------------------------------------------------------------
# Matplotlib canvas
# ---------------------------------------------------------------------------
class MplCanvas(FigureCanvas):
    def __init__(self, parent=None, width=5, height=4, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi, facecolor=BG)
        self.axes = fig.add_subplot(111)
        self.axes.set_facecolor(BG)
        self.axes.tick_params(colors=MUTED)
        fig.tight_layout()
        super().__init__(fig)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)


# ---------------------------------------------------------------------------
# ThrustCurve.org background workers (network must not block the UI thread)
# ---------------------------------------------------------------------------
# Strong references to in-flight workers. A QThread must never be destroyed
# while running (it aborts the process), so workers are unparented and kept
# alive here until they finish - even if the dialog that started them closes.
_ACTIVE_WORKERS = set()


def _track_worker(worker):
    _ACTIVE_WORKERS.add(worker)
    worker.finished.connect(lambda: _ACTIVE_WORKERS.discard(worker))


class _SearchWorker(QThread):
    """Runs a ThrustCurve.org search off the UI thread."""
    done = pyqtSignal(list)          # list[normalized motor dict]
    failed = pyqtSignal(str)         # human-readable error text

    def __init__(self, query, impulse_class, parent=None):
        super().__init__(parent)
        self.query = query
        self.impulse_class = impulse_class

    def run(self):
        try:
            query = (self.query or "").strip()
            imp = self.impulse_class or None
            results = []
            if query:
                # Treat as a motor code (commonName) first.
                results = thrustcurve.search_motors(
                    common_name=query, impulse_class=imp, max_results=50)
                # Fall back to a manufacturer search if nothing matched.
                if not results:
                    results = thrustcurve.search_motors(
                        manufacturer=query, impulse_class=imp, max_results=50)
            else:
                # No text: browse by impulse class alone (if given).
                results = thrustcurve.search_motors(
                    impulse_class=imp, max_results=50)
            self.done.emit(results)
        except thrustcurve.ThrustCurveError as exc:
            self.failed.emit(str(exc))
        except Exception as exc:  # never let a worker crash the app
            self.failed.emit("Unexpected error: {}".format(exc))


class _DownloadWorker(QThread):
    """Downloads samples and saves the CSV off the UI thread."""
    done = pyqtSignal(str, dict)     # (saved filename, motor dict)
    failed = pyqtSignal(str)

    def __init__(self, motor, motors_dir, parent=None):
        super().__init__(parent)
        self.motor = motor
        self.motors_dir = motors_dir

    def run(self):
        try:
            samples = thrustcurve.download_samples(self.motor.get("motor_id"))
            filename = thrustcurve.save_motor_csv(self.motor, samples, self.motors_dir)
            self.done.emit(filename, self.motor)
        except thrustcurve.ThrustCurveError as exc:
            self.failed.emit(str(exc))
        except Exception as exc:
            self.failed.emit("Unexpected error: {}".format(exc))


# ---------------------------------------------------------------------------
# Motor selector dialog (local library + ThrustCurve.org search)
# ---------------------------------------------------------------------------
class MotorSelectorDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Select Rocket Motor")
        self.setMinimumWidth(520)
        self.setStyleSheet(MODERN_STYLE)

        # Which tab produced the pick: "local" or "online".
        self._source = "local"
        # Result of a completed ThrustCurve download (dict) once "USE" happens.
        self._online_pick = None
        self._tc_results = []          # normalized motor dicts from last search
        self._search_worker = None
        self._download_worker = None

        layout = QVBoxLayout(self)

        self.tabs = QTabWidget()
        self.tabs.setStyleSheet(
            f"QTabBar::tab {{ background: {PANEL}; padding: 8px 14px; "
            f"border-top-left-radius: 6px; border-top-right-radius: 6px; }} "
            f"QTabBar::tab:selected {{ background: {BLUE}; color: white; }}"
        )
        self.local_tab = QWidget()
        self.online_tab = QWidget()
        self.tabs.addTab(self.local_tab, "LOCAL LIBRARY")
        self.tabs.addTab(self.online_tab, "THRUSTCURVE.ORG")
        self.tabs.currentChanged.connect(self._on_tab_changed)
        layout.addWidget(self.tabs)

        self._build_local_tab()
        self._build_online_tab()

        # Shared OK / Cancel (drives the LOCAL tab; online uses its own button).
        self.buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok |
                                        QDialogButtonBox.StandardButton.Cancel)
        self.buttons.accepted.connect(self._accept_local)
        self.buttons.rejected.connect(self.reject)
        layout.addWidget(self.buttons)
        if not self.motors:
            self.buttons.button(QDialogButtonBox.StandardButton.Ok).setEnabled(False)

        self.update_detail()

    # ------------------------------------------------------------------
    # LOCAL LIBRARY tab (original behaviour)
    # ------------------------------------------------------------------
    def _build_local_tab(self):
        layout = QVBoxLayout(self.local_tab)
        layout.addWidget(QLabel("Motors found in <b>motors/</b>:"))

        self.motors = scan_motors()
        self.motor_list = QComboBox()
        if self.motors:
            for m in self.motors:
                self.motor_list.addItem(
                    f"{m['name']}  ·  {m['avg_thrust']:.0f} N avg  ·  {m['burn_time']:.2f} s"
                )
        else:
            self.motor_list.addItem("No motor files found")
            self.motor_list.setEnabled(False)
        self.motor_list.currentIndexChanged.connect(self.update_detail)
        layout.addWidget(self.motor_list)

        self.detail = QLabel()
        self.detail.setWordWrap(True)
        self.detail.setStyleSheet(f"color: {MUTED}; background: {PANEL}; "
                                  f"border: 1px solid {BORDER}; border-radius: 6px; padding: 10px;")
        layout.addWidget(self.detail)
        layout.addStretch()

    def _accept_local(self):
        # OK button only commits a LOCAL pick; online commits via its own button.
        if self.tabs.currentWidget() is self.local_tab:
            self._source = "local"
            self.accept()

    def _on_tab_changed(self, _idx):
        # OK button is meaningful only for the local tab.
        on_local = self.tabs.currentWidget() is self.local_tab
        ok_btn = self.buttons.button(QDialogButtonBox.StandardButton.Ok)
        if ok_btn is not None:
            ok_btn.setEnabled(on_local and bool(self.motors))

    def update_detail(self):
        m = self._local_selected()
        if not m:
            self.detail.setText("No motor available.")
            return
        self.detail.setText(
            f"<b>{m['name']}</b><br>File: {m['file']}<br>"
            f"Avg thrust: {m['avg_thrust']:.1f} N<br>"
            f"Peak thrust: {m['peak_thrust']:.1f} N<br>"
            f"Burn time: {m['burn_time']:.2f} s<br>"
            f"Total impulse: {m['impulse']:.1f} N·s"
        )

    def _local_selected(self):
        idx = self.motor_list.currentIndex()
        if 0 <= idx < len(self.motors):
            return self.motors[idx]
        return None

    # ------------------------------------------------------------------
    # THRUSTCURVE.ORG tab
    # ------------------------------------------------------------------
    def _build_online_tab(self):
        layout = QVBoxLayout(self.online_tab)

        # Search row
        search_row = QHBoxLayout()
        self.tc_search = QLineEdit()
        self.tc_search.setPlaceholderText("Search by name, e.g. F40 or Estes")
        self.tc_search.returnPressed.connect(self._start_search)
        search_row.addWidget(self.tc_search, 1)

        self.tc_class = QComboBox()
        self.tc_class.addItem("")  # blank = any
        for c in "ABCDEFGHIJKLMNO":
            self.tc_class.addItem(c)
        self.tc_class.setToolTip("Optional impulse-class filter (A-O).")
        self.tc_class.setFixedWidth(60)
        search_row.addWidget(self.tc_class)

        self.tc_search_btn = QPushButton("Search")
        self.tc_search_btn.clicked.connect(self._start_search)
        search_row.addWidget(self.tc_search_btn)
        layout.addLayout(search_row)

        # Results list
        self.tc_list = QListWidget()
        self.tc_list.setStyleSheet(
            f"QListWidget {{ background: {PANEL}; border: 1px solid {BORDER}; "
            f"border-radius: 6px; color: {TEXT}; }} "
            f"QListWidget::item:selected {{ background: {BLUE}; color: white; }}"
        )
        self.tc_list.setMinimumHeight(150)
        self.tc_list.currentRowChanged.connect(self._update_online_detail)
        layout.addWidget(self.tc_list, 1)

        # Detail panel (reuse local detail styling)
        self.tc_detail = QLabel("Search ThrustCurve.org for a motor.")
        self.tc_detail.setWordWrap(True)
        self.tc_detail.setStyleSheet(f"color: {MUTED}; background: {PANEL}; "
                                     f"border: 1px solid {BORDER}; border-radius: 6px; padding: 10px;")
        layout.addWidget(self.tc_detail)

        # Status label (errors in red)
        self.tc_status = QLabel("")
        self.tc_status.setWordWrap(True)
        self.tc_status.setStyleSheet(f"color: {FAINT};")
        layout.addWidget(self.tc_status)

        # Download & use button
        self.tc_use_btn = QPushButton("DOWNLOAD && USE")
        self.tc_use_btn.setObjectName("LaunchBtn")
        self.tc_use_btn.clicked.connect(self._start_download)
        self.tc_use_btn.setEnabled(False)
        layout.addWidget(self.tc_use_btn)

    def _set_tc_status(self, msg, color=None):
        self.tc_status.setText(msg)
        self.tc_status.setStyleSheet(f"color: {color or FAINT};")

    def _start_search(self):
        if self._search_worker is not None:
            return
        query = self.tc_search.text().strip()
        imp = self.tc_class.currentText().strip()
        if not query and not imp:
            self._set_tc_status("Enter a motor name or pick an impulse class.", AMBER)
            return
        self.tc_search_btn.setEnabled(False)
        self.tc_search_btn.setText("Searching...")
        self.tc_use_btn.setEnabled(False)
        self._set_tc_status("Searching ThrustCurve.org...", BLUE)
        self.tc_list.clear()
        self._tc_results = []

        # Unparented + tracked: outlives the dialog safely if the user closes it.
        self._search_worker = _SearchWorker(query, imp)
        _track_worker(self._search_worker)
        self._search_worker.done.connect(self._on_search_done)
        self._search_worker.failed.connect(self._on_search_failed)
        self._search_worker.finished.connect(self._search_worker_cleanup)
        self._search_worker.start()

    def _search_worker_cleanup(self):
        self.tc_search_btn.setEnabled(True)
        self.tc_search_btn.setText("Search")
        self._search_worker = None

    def _on_search_done(self, results):
        self._tc_results = results
        self.tc_list.clear()
        if not results:
            self._set_tc_status("No motors found. Try a different name or class.", AMBER)
            self.tc_detail.setText("No results.")
            return
        for m in results:
            label = (f"{m['manufacturer']} {m['common_name'] or m['designation']} — "
                     f"{m['avg_thrust_n']:.0f} N avg · {m['burn_time_s']:.1f} s burn · "
                     f"{m['total_impulse_ns']:.0f} Ns")
            QListWidgetItem(label, self.tc_list)
        self._set_tc_status(f"{len(results)} motor(s) found.", GREEN)
        self.tc_list.setCurrentRow(0)

    def _on_search_failed(self, msg):
        self._set_tc_status(
            f"{msg}  Check your internet connection — you can still use the "
            f"Local Library tab.", RED)
        self.tc_detail.setText("Search failed.")

    def _current_online_motor(self):
        row = self.tc_list.currentRow()
        if 0 <= row < len(self._tc_results):
            return self._tc_results[row]
        return None

    def _update_online_detail(self, _row):
        m = self._current_online_motor()
        if not m:
            self.tc_use_btn.setEnabled(False)
            return
        self.tc_use_btn.setEnabled(True)
        parts = [f"<b>{m['name']}</b>"]
        if m["diameter_mm"]:
            parts.append(f"Diameter: {m['diameter_mm']:.0f} mm")
        if m["length_mm"]:
            parts.append(f"Length: {m['length_mm']:.0f} mm")
        if m["type"]:
            parts.append(f"Type: {m['type']}")
        parts.append(f"Avg thrust: {m['avg_thrust_n']:.1f} N")
        parts.append(f"Max thrust: {m['max_thrust_n']:.1f} N")
        parts.append(f"Total impulse: {m['total_impulse_ns']:.1f} N·s")
        parts.append(f"Burn time: {m['burn_time_s']:.2f} s")
        if m["total_weight_g"]:
            parts.append(f"Total weight: {m['total_weight_g']:.1f} g")
        if m["prop_weight_g"]:
            parts.append(f"Propellant: {m['prop_weight_g']:.1f} g")
        if m["info_url"]:
            parts.append(f"Info: {m['info_url']}")
        self.tc_detail.setText("<br>".join(parts))

    def _start_download(self):
        if self._download_worker is not None:
            return
        m = self._current_online_motor()
        if not m:
            self._set_tc_status("Select a motor first.", AMBER)
            return
        self.tc_use_btn.setEnabled(False)
        self.tc_use_btn.setText("Downloading...")
        self.tc_search_btn.setEnabled(False)
        self._set_tc_status("Downloading thrust curve...", BLUE)

        # Unparented + tracked: outlives the dialog safely if the user closes it.
        self._download_worker = _DownloadWorker(m, MOTORS_DIR)
        _track_worker(self._download_worker)
        self._download_worker.done.connect(self._on_download_done)
        self._download_worker.failed.connect(self._on_download_failed)
        self._download_worker.finished.connect(self._download_worker_cleanup)
        self._download_worker.start()

    def _download_worker_cleanup(self):
        self.tc_use_btn.setText("DOWNLOAD && USE")
        self.tc_search_btn.setEnabled(True)
        self._download_worker = None

    def _on_download_done(self, filename, motor):
        # Re-parse the saved CSV so we return the same shape as a local pick.
        path = os.path.join(MOTORS_DIR, filename)
        try:
            parsed = parse_motor_file(path)
        except Exception:
            parsed = {
                "name": motor.get("name", filename),
                "file": filename,
                "avg_thrust": motor.get("avg_thrust_n", 0.0),
                "peak_thrust": motor.get("max_thrust_n", 0.0),
                "burn_time": motor.get("burn_time_s", 0.0),
                "impulse": motor.get("total_impulse_ns", 0.0),
            }
        # Carry the online metadata (weights) for the caller's auto-fill bonus.
        parsed["prop_weight_g"] = motor.get("prop_weight_g", 0.0)
        parsed["total_weight_g"] = motor.get("total_weight_g", 0.0)
        parsed["source"] = "thrustcurve"
        self._online_pick = parsed
        self._source = "online"
        self.accept()

    def _on_download_failed(self, msg):
        self.tc_use_btn.setEnabled(True)
        self._set_tc_status(
            f"{msg}  Check your internet connection — you can still use the "
            f"Local Library tab.", RED)

    # ------------------------------------------------------------------
    # Unified selection contract (works for both tabs)
    # ------------------------------------------------------------------
    def selected(self):
        if self._source == "online" and self._online_pick is not None:
            return self._online_pick
        return self._local_selected()


# ---------------------------------------------------------------------------
# Part edit dialog
# ---------------------------------------------------------------------------
class PartEditDialog(QDialog):
    def __init__(self, part_data, parent=None):
        super().__init__(parent)
        self.setWindowTitle(f"Edit {part_data.get('name', 'Part')}")
        self.setStyleSheet(MODERN_STYLE)
        self.layout = QFormLayout(self)
        self.delete_requested = False
        self.inputs = {}

        for key, value in part_data.items():
            if key == "type":
                continue
            self.inputs[key] = QLineEdit(str(value))
            self.layout.addRow(f"{key.replace('_', ' ').capitalize()}:", self.inputs[key])

        if part_data.get("type") == "motor":
            self.motor_btn = QPushButton("BROWSE MOTORS")
            self.motor_btn.clicked.connect(self.open_motor_browser)
            self.layout.addRow("Selection", self.motor_btn)
        elif part_data.get("type") == "fins":
            help_text = QLabel("<small>Format: (x1,y1), (x2,y2), (x3,y3), (x4,y4)<br>"
                               "Starts from the bottom-inner corner.</small>")
            self.layout.addRow(help_text)

        self.buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok |
                                        QDialogButtonBox.StandardButton.Cancel)
        self.buttons.accepted.connect(self.accept)
        self.buttons.rejected.connect(self.reject)

        self.delete_btn = QPushButton("DELETE COMPONENT")
        self.delete_btn.setStyleSheet(f"background-color: {RED}; color: white; margin-top: 10px;")
        self.delete_btn.clicked.connect(self.request_delete)

        self.layout.addWidget(self.buttons)
        self.layout.addWidget(self.delete_btn)

    def open_motor_browser(self):
        browser = MotorSelectorDialog(self)
        if browser.exec():
            m = browser.selected()
            if not m:
                return
            if "motor_id" in self.inputs:
                self.inputs["motor_id"].setText(m["name"])
            if "motor_file" in self.inputs:
                self.inputs["motor_file"].setText(m["file"])
            # Bonus: for a ThrustCurve download, fill mass / propellant fields if present.
            is_online = m.get("source") == "thrustcurve"
            prop_g = m.get("prop_weight_g", 0.0) or 0.0
            total_g = m.get("total_weight_g", 0.0) or 0.0
            if is_online:
                if "mass" in self.inputs and total_g > 0:
                    self.inputs["mass"].setText(f"{total_g / 1000:.3f}")
                if "propellant_mass" in self.inputs and prop_g > 0:
                    self.inputs["propellant_mass"].setText(f"{prop_g / 1000:.3f}")
            # Push the chosen file to the flight config too
            parent = self.parent()
            if parent is not None and hasattr(parent, "env_inputs"):
                parent.env_inputs["Motor File"].setText(m["file"])
                msg = f"Motor set to {m['name']} ({m['file']})."
                if is_online and prop_g > 0 and hasattr(parent, "inputs"):
                    parent.inputs["Propellant Mass (kg)"].setText(f"{prop_g / 1000:.3f}")
                    msg += f" Propellant mass auto-filled ({prop_g / 1000:.3f} kg)."
                parent.set_status(msg, GREEN)

    def request_delete(self):
        self.delete_requested = True
        self.accept()

    def get_values(self):
        results = {}
        for key, widget in self.inputs.items():
            txt = widget.text()
            try:
                results[key] = float(txt)
            except ValueError:
                results[key] = txt
        return results


# ---------------------------------------------------------------------------
# Main window
# ---------------------------------------------------------------------------
class MissionControl(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("RocketSim | Mission Control")
        self.resize(1280, 820)
        self.setStyleSheet(MODERN_STYLE)

        self.rocket_components = []
        self.proc = None  # QProcess for the simulation

        self.layout = QVBoxLayout(self)

        # Toolbar
        toolbar = QHBoxLayout()
        title = QLabel("<b>ROCKETSIM</b> <span style='color:#71717a;'>Mission Control</span>")
        title.setStyleSheet("font-size: 14px;")
        self.save_btn = QPushButton("Save Project")
        self.load_btn = QPushButton("Load Project")
        self.save_btn.setObjectName("GhostBtn")
        self.load_btn.setObjectName("GhostBtn")
        self.save_btn.clicked.connect(self.save_project)
        self.load_btn.clicked.connect(self.load_project)
        toolbar.addWidget(title)
        toolbar.addStretch()
        toolbar.addWidget(self.load_btn)
        toolbar.addWidget(self.save_btn)
        self.layout.addLayout(toolbar)

        # Tabs
        self.tabs = QTabWidget()
        self.tabs.setStyleSheet(
            f"QTabBar::tab {{ background: {PANEL}; padding: 10px; min-width: 150px; "
            f"border-top-left-radius: 6px; border-top-right-radius: 6px; }} "
            f"QTabBar::tab:selected {{ background: {BLUE}; color: white; }}"
        )
        self.builder_tab = QWidget()
        self.flight_tab = QWidget()
        self.tabs.addTab(self.builder_tab, "VEHICLE BUILDER")
        self.tabs.addTab(self.flight_tab, "FLIGHT TELEMETRY")
        self.layout.addWidget(self.tabs)

        # Status bar (user-visible feedback area)
        self.status_bar = QLabel("Ready.")
        self.status_bar.setStyleSheet(
            f"color: {MUTED}; background: {PANEL}; border: 1px solid {BORDER}; "
            f"border-radius: 6px; padding: 6px 10px;")
        self.layout.addWidget(self.status_bar)

        # Templates for the builder
        self.templates = {
            "body": {"type": "body", "name": "Body Tube", "mass": 0.1, "length": 0.3, "diameter": 0.04, "y_offset": 0.0},
            "nose": {"type": "nose", "name": "Nose Cone", "mass": 0.05, "length": 0.15, "diameter": 0.04, "y_offset": 0.3},
            "fins": {"type": "fins", "name": "Fins", "mass": 0.05, "points": "(0,0), (0.1,0), (0.07,0.08), (0,0.08)", "y_offset": 0.0},
            "motor": {"type": "motor", "name": "Motor", "mass": 0.05, "length": 0.1, "diameter": 0.029, "motor_id": "None", "motor_file": "AeroTech_F40W.csv", "y_offset": 0.01},
        }

        self.setup_builder_tab()
        self.setup_flight_tab()

        # Load persisted config into the flight fields, if present
        self.load_last_config()
        self.update_stability_gauge()
        self.update_schematic()

    # ------------------------------------------------------------------
    # Status helper
    # ------------------------------------------------------------------
    def set_status(self, msg, color=None):
        color = color or MUTED
        self.status_bar.setText(msg)
        self.status_bar.setStyleSheet(
            f"color: {color}; background: {PANEL}; border: 1px solid {BORDER}; "
            f"border-radius: 6px; padding: 6px 10px;")

    def _read_float(self, widget, label):
        """Read a float from a QLineEdit, flag it red + raise on failure."""
        try:
            val = float(widget.text())
            widget.setProperty("invalid", False)
            widget.setStyle(widget.style())
            return val
        except ValueError:
            widget.setProperty("invalid", True)
            widget.setStyle(widget.style())
            raise ValueError(f"'{label}' must be a number (got '{widget.text()}').")

    # ==================================================================
    # BUILDER TAB
    # ==================================================================
    def setup_builder_tab(self):
        layout = QHBoxLayout(self.builder_tab)

        # --- Left: construction kit + parts list ---
        self.controls = QGroupBox("CONSTRUCTION KIT")
        self.controls.setFixedWidth(320)
        self.control_layout = QVBoxLayout(self.controls)

        # Button grid (built ONCE, added once)
        self.btn_container = QWidget()
        self.btn_grid = QGridLayout(self.btn_container)
        self.btn_grid.setContentsMargins(0, 5, 0, 5)
        self.btn_grid.setSpacing(8)
        for i, p_type in enumerate(self.templates.keys()):
            btn = QPushButton(f"+ {p_type.capitalize()}")
            btn.setMinimumHeight(40)
            btn.clicked.connect(lambda checked, t=p_type: self.add_component(t))
            self.btn_grid.addWidget(btn, i // 2, i % 2)
        self.control_layout.addWidget(self.btn_container)

        lbl = QLabel("ACTIVE COMPONENTS")
        lbl.setStyleSheet(f"color: {FAINT}; font-weight: bold; margin-top: 6px;")
        self.control_layout.addWidget(lbl)

        self.part_scroll = QScrollArea()
        self.part_scroll.setWidgetResizable(True)
        self.part_scroll.setFrameShape(QFrame.Shape.NoFrame)
        self.part_scroll.setStyleSheet(f"background: {PANEL}; border-radius: 5px;")
        self.part_container = QWidget()
        self.part_list_layout = QVBoxLayout(self.part_container)
        self.part_list_layout.setAlignment(Qt.AlignmentFlag.AlignTop)
        self.part_scroll.setWidget(self.part_container)
        self.control_layout.addWidget(self.part_scroll)

        self.sync_btn = QPushButton("SYNC TO FLIGHT CONFIG")
        self.sync_btn.setObjectName("LaunchBtn")
        self.sync_btn.setToolTip("Estimate mass, length, CG and propellant from the parts, "
                                 "and push them into the Flight Telemetry config.")
        self.sync_btn.clicked.connect(self.sync_builder_to_flight)
        self.control_layout.addWidget(self.sync_btn)

        # --- Center: schematic ---
        self.schematic_canvas = MplCanvas(width=5, height=8)
        self.schematic_canvas.mpl_connect('pick_event', self.on_part_clicked)

        # --- Right: stability + totals ---
        self.stability_panel = QGroupBox("STABILITY ANALYSIS")
        self.stability_panel.setFixedWidth(260)
        stab_layout = QVBoxLayout(self.stability_panel)

        cap = QLabel("STATIC MARGIN")
        cap.setStyleSheet(f"color: {FAINT}; font-size: 10px; font-weight: bold;")
        stab_layout.addWidget(cap)

        self.margin_label = QLabel("0.00 CAL")
        self.margin_label.setStyleSheet(f"font-size: 30px; font-weight: 900; color: {BLUE};")
        stab_layout.addWidget(self.margin_label)

        self.margin_status = QLabel("Add parts or set CP/CG.")
        self.margin_status.setWordWrap(True)
        self.margin_status.setStyleSheet(f"color: {MUTED};")
        stab_layout.addWidget(self.margin_status)

        stab_layout.addSpacing(14)
        div = QFrame()
        div.setFrameShape(QFrame.Shape.HLine)
        div.setStyleSheet(f"color: {BORDER};")
        stab_layout.addWidget(div)

        self.total_mass_label = QLabel("Total mass: 0.000 kg")
        self.total_len_label = QLabel("Total length: 0.000 m")
        for l in (self.total_mass_label, self.total_len_label):
            l.setStyleSheet(f"color: {TEXT}; margin-top: 6px;")
            stab_layout.addWidget(l)

        legend = QLabel(
            f"<span style='color:{RED};'>&#9632;</span> &lt;1 cal unstable&nbsp; "
            f"<span style='color:{GREEN};'>&#9632;</span> 1-2 optimal&nbsp; "
            f"<span style='color:{AMBER};'>&#9632;</span> 2-3 acceptable&nbsp; "
            f"<span style='color:{RED};'>&#9632;</span> &gt;3 overstable"
        )
        legend.setWordWrap(True)
        legend.setStyleSheet("font-size: 10px; margin-top: 10px;")
        stab_layout.addWidget(legend)
        stab_layout.addStretch()

        layout.addWidget(self.controls)
        layout.addWidget(self.schematic_canvas, 1)
        layout.addWidget(self.stability_panel)

    def add_component(self, part_type):
        if part_type in self.templates:
            self.rocket_components.append(self.templates[part_type].copy())
            self.update_schematic()
            self.refresh_sidebar()
            self.set_status(f"Added {part_type}.", GREEN)

    def refresh_sidebar(self):
        while self.part_list_layout.count():
            item = self.part_list_layout.takeAt(0)
            w = item.widget()
            if w:
                w.deleteLater()

        indexed = list(enumerate(self.rocket_components))
        try:
            indexed.sort(key=lambda x: float(x[1].get('y_offset', 0)), reverse=True)
        except (TypeError, ValueError):
            pass

        for i, part in indexed:
            card = QFrame()
            card.setStyleSheet(
                f"QFrame {{ background-color: {CARD}; border: 1px solid {BORDER}; "
                f"border-radius: 6px; margin-bottom: 2px; }} "
                f"QFrame:hover {{ border: 1px solid {BLUE}; }}"
            )
            card_layout = QHBoxLayout(card)
            name_label = QLabel(f"<b>{part.get('name', '?')}</b>"
                                f"<br><small>{part.get('type', '').upper()}</small>")
            name_label.setStyleSheet(f"border: none; color: {TEXT};")
            card_layout.addWidget(name_label)
            card_layout.addStretch()

            edit_btn = QPushButton("Edit")
            edit_btn.setFixedWidth(56)
            edit_btn.setObjectName("GhostBtn")
            edit_btn.clicked.connect(lambda checked, idx=i: self.open_edit_dialog_by_index(idx))
            card_layout.addWidget(edit_btn)
            self.part_list_layout.addWidget(card)

    def on_part_clicked(self, event):
        # Single edit path: identify the part index and delegate.
        try:
            idx = int(event.artist.part_index)
        except (AttributeError, ValueError, TypeError):
            return
        self.open_edit_dialog_by_index(idx)

    def open_edit_dialog_by_index(self, idx):
        if not (0 <= idx < len(self.rocket_components)):
            return
        part = self.rocket_components[idx]
        dialog = PartEditDialog(part, self)
        if dialog.exec():
            if dialog.delete_requested:
                self.rocket_components.pop(idx)
                self.set_status(f"Deleted {part.get('name', 'part')}.", AMBER)
            else:
                self.rocket_components[idx].update(dialog.get_values())
                self.set_status(f"Updated {part.get('name', 'part')}.", GREEN)
            self.update_schematic()
            self.refresh_sidebar()

    def _body_radius(self):
        for p in self.rocket_components:
            if p.get("type") == "body":
                try:
                    return float(p["diameter"]) / 2
                except (KeyError, ValueError):
                    pass
        return 0.02

    def update_schematic(self):
        ax = self.schematic_canvas.axes
        ax.clear()
        ax.set_facecolor(BG)
        ax.axis('off')

        if not self.rocket_components:
            ax.text(0, 0, "NO COMPONENTS\nAdd parts from the kit", color=FAINT,
                    ha='center', va='center')
            ax.set_xlim(-0.5, 0.5)
            ax.set_ylim(-0.5, 0.5)
            self.schematic_canvas.draw()
            self.update_totals()
            return

        from matplotlib.patches import Rectangle, Polygon
        max_height = 0.0
        r_body = self._body_radius()

        for i, part in enumerate(self.rocket_components):
            ptype = part.get("type")
            try:
                y0 = float(part.get("y_offset", 0))
                w = float(part.get("diameter", 0.04))
                h = float(part.get("length", 0))
            except (ValueError, TypeError):
                continue
            max_height = max(max_height, y0 + h)

            shape = None
            if ptype == "body":
                shape = Rectangle((-w / 2, y0), w, h, color=CARD, ec=BLUE, lw=2, picker=True)
            elif ptype == "nose":
                shape = Polygon([[-w / 2, y0], [w / 2, y0], [0, y0 + h]],
                                color=BLUE, ec='white', lw=1, picker=True)
            elif ptype == "fins":
                try:
                    raw = parse_fin_points(part["points"])
                except Exception as e:
                    self.set_status(f"Fin geometry error: {e}", RED)
                    continue
                right = [[p[0] + r_body, p[1] + y0] for p in raw]
                left = [[-p[0] - r_body, p[1] + y0] for p in raw]
                for pts in (right, left):
                    poly = Polygon(pts, color=BLUE, alpha=0.75, picker=True)
                    poly.part_index = i
                    ax.add_patch(poly)
                max_height = max(max_height, y0 + max(p[1] for p in raw))
                continue
            elif ptype == "motor":
                shape = Rectangle((-w / 2, max(y0, 0.0)), w, h,
                                  color="#52525b", ec=MUTED, lw=1, picker=True)
                ax.text(0, y0 + h / 2, str(part.get("motor_id", "")), color='white',
                        ha='center', va='center', fontsize=7, fontweight='bold')

            if shape is not None:
                shape.part_index = i
                ax.add_patch(shape)

        pad = max(max_height * 0.05, 0.05)
        ax.set_ylim(-pad, max_height + pad)
        span = max(max_height, 0.2)
        ax.set_xlim(-span / 2, span / 2)
        ax.set_aspect('equal')
        self.schematic_canvas.draw()
        self.update_totals()

    def builder_totals(self):
        """Return (dry_mass, prop_mass, total_length, cg)."""
        dry = 0.0
        prop = 0.0
        moment = 0.0
        top = 0.0
        for p in self.rocket_components:
            try:
                m = float(p.get("mass", 0))
            except (ValueError, TypeError):
                m = 0.0
            try:
                y0 = float(p.get("y_offset", 0))
                length = float(p.get("length", 0))
            except (ValueError, TypeError):
                y0, length = 0.0, 0.0
            dry += m
            moment += m * (y0 + length / 2)
            top = max(top, y0 + length)
            if p.get("type") == "motor":
                try:
                    prop += float(p.get("propellant_mass", 0))
                except (ValueError, TypeError):
                    pass
        cg = moment / dry if dry > 0 else 0.0
        return dry, prop, top, cg

    def update_totals(self):
        dry, prop, length, cg = self.builder_totals()
        self.total_mass_label.setText(f"Total mass: {dry + prop:.3f} kg")
        self.total_len_label.setText(f"Total length: {length:.3f} m")

    def estimate_cp(self):
        """Simplified Barrowman estimate of the CP distance from the NOSE TIP.

        Considers the nose cone and one fin set (4-fin plus configuration,
        matching the simulation engine); a cylindrical body contributes ~zero
        normal force. Returns None if the geometry is insufficient.
        """
        nose = next((p for p in self.rocket_components if p.get("type") == "nose"), None)
        fins = next((p for p in self.rocket_components if p.get("type") == "fins"), None)
        if nose is None:
            return None
        _, _, total_len, _ = self.builder_totals()
        try:
            nose_len = float(nose.get("length", 0))
        except (ValueError, TypeError):
            return None
        if total_len <= 0 or nose_len <= 0:
            return None
        r = self._body_radius()
        d = 2 * r

        # Nose cone: CNa = 2, CP at ~2/3 of the nose length from the tip (conical)
        cn_nose = 2.0
        x_nose = (2.0 / 3.0) * nose_len
        if fins is None or d <= 0:
            return x_nose

        try:
            pts = parse_fin_points(fins.get("points", ""))
            fin_y0 = float(fins.get("y_offset", 0))
        except Exception:
            return x_nose

        xs = [p[0] for p in pts]
        span = max(xs)
        if span <= 0:
            return x_nose
        # Root chord: vertical extent near the body (x ~ 0); tip chord near max span.
        root_ys = [p[1] for p in pts if p[0] <= 0.05 * span]
        tip_ys = [p[1] for p in pts if p[0] >= 0.95 * span]
        if not root_ys:
            return x_nose
        cr = max(root_ys) - min(root_ys)
        ct = (max(tip_ys) - min(tip_ys)) if tip_ys else 0.0
        if cr + ct <= 0:
            return x_nose

        # Builder y grows upward from the tail; convert to distance-from-nose.
        root_le_from_nose = total_len - (fin_y0 + max(root_ys))
        tip_le_y = max(tip_ys) if tip_ys else min(p[1] for p in pts)
        m_sweep = max(0.0, max(root_ys) - tip_le_y)  # leading-edge sweep along the body axis

        n_fins = 4
        l_mid = float(np.hypot(span, m_sweep + ct / 2.0 - cr / 2.0))  # mid-chord line
        cn_fins = (1 + r / (span + r)) * (4.0 * n_fins * (span / d) ** 2) / \
            (1 + np.sqrt(1 + (2.0 * l_mid / (cr + ct)) ** 2))
        x_fins = root_le_from_nose + (m_sweep / 3.0) * (cr + 2 * ct) / (cr + ct) + \
            (1.0 / 6.0) * ((cr + ct) - (cr * ct) / (cr + ct))

        return float((cn_nose * x_nose + cn_fins * x_fins) / (cn_nose + cn_fins))

    def _fin_planform_area(self):
        """Shoelace area of one fin polygon, or None."""
        fins = next((p for p in self.rocket_components if p.get("type") == "fins"), None)
        if fins is None:
            return None
        try:
            pts = parse_fin_points(fins.get("points", ""))
        except Exception:
            return None
        area = 0.0
        for i in range(len(pts)):
            x1, y1 = pts[i]
            x2, y2 = pts[(i + 1) % len(pts)]
            area += x1 * y2 - x2 * y1
        return abs(area) / 2.0

    def sync_builder_to_flight(self):
        if not self.rocket_components:
            self.set_status("No components to sync. Add parts first.", AMBER)
            return
        dry, prop, length, cg_from_bottom = self.builder_totals()
        self.inputs["Dry Mass (kg)"].setText(f"{dry:.3f}")
        self.inputs["Total Length (m)"].setText(f"{length:.3f}")
        # Builder positions are measured from the TAIL (y up); the flight config
        # measures CG/CP from the NOSE TIP - convert reference frames.
        if length > 0 and cg_from_bottom > 0:
            self.inputs["CG Dist (m)"].setText(f"{length - cg_from_bottom:.3f}")
        # CP from simplified Barrowman (nose + fins)
        cp = self.estimate_cp()
        if cp is not None:
            self.inputs["CP Dist (m)"].setText(f"{cp:.3f}")
        fin_area = self._fin_planform_area()
        if fin_area:
            self.inputs["Fin Area (m²)"].setText(f"{fin_area:.4f}")
        # Body diameter -> flight diameter
        rb = self._body_radius()
        if rb > 0:
            self.inputs["Diameter (m)"].setText(f"{rb * 2:.3f}")
        # Propellant + motor file from motor part
        for p in self.rocket_components:
            if p.get("type") == "motor":
                if p.get("propellant_mass"):
                    self.inputs["Propellant Mass (kg)"].setText(str(p["propellant_mass"]))
                if p.get("motor_file"):
                    self.env_inputs["Motor File"].setText(str(p["motor_file"]))
                break
        self.update_stability_gauge()
        extra = " CP estimated via Barrowman." if cp is not None else \
            " Add a nose cone for a CP estimate."
        self.set_status(f"Flight config updated from builder (mass, length, CG, diameter).{extra}", GREEN)

    # ==================================================================
    # FLIGHT TAB
    # ==================================================================
    def setup_flight_tab(self):
        layout = QHBoxLayout(self.flight_tab)

        # --- Sidebar ---
        sidebar_scroll = QScrollArea()
        sidebar_scroll.setFixedWidth(350)
        sidebar_scroll.setWidgetResizable(True)
        sidebar_scroll.setFrameShape(QFrame.Shape.NoFrame)
        sidebar_widget = QWidget()
        self.side_layout = QVBoxLayout(sidebar_widget)

        header = QLabel("FLIGHT CONFIGURATION")
        header.setObjectName("Header")
        self.side_layout.addWidget(header)

        # Vehicle geometry
        geom_group = QGroupBox("VEHICLE")
        geom_form = QFormLayout(geom_group)
        self.inputs = {
            "Total Length (m)": QLineEdit("1.2"),
            "Dry Mass (kg)": QLineEdit("1.0"),
            "Propellant Mass (kg)": QLineEdit("0.126"),
            "Diameter (m)": QLineEdit("0.077"),
            "CP Dist (m)": QLineEdit("0.6"),
            "CG Dist (m)": QLineEdit("0.5"),
            "Fin Area (m²)": QLineEdit("0.006"),
            "Drag Coefficient": QLineEdit("0.5"),
        }
        for l, w in self.inputs.items():
            self._decorate_numeric(w, l)
            geom_form.addRow(l, w)
        self.side_layout.addWidget(geom_group)

        # Environment
        env_group = QGroupBox("ENVIRONMENT")
        env_form = QFormLayout(env_group)
        self.env_inputs = {
            "Motor File": QLineEdit("AeroTech_F40W.csv"),
            "Rail Angle (°)": QLineEdit("90"),
            "Rail Length (m)": QLineEdit("1.5"),
            "Wind Speed (m/s)": QLineEdit("5.0"),
            "Air Density (kg/m³)": QLineEdit("1.225"),
        }
        for l, w in self.env_inputs.items():
            w.setToolTip(TOOLTIPS.get(l, ""))
            if l != "Motor File":
                self._decorate_numeric(w, l)
            env_form.addRow(l, w)
        self.motor_browse_btn = QPushButton("BROWSE MOTORS")
        self.motor_browse_btn.setObjectName("GhostBtn")
        self.motor_browse_btn.clicked.connect(self.browse_motor_for_flight)
        env_form.addRow("", self.motor_browse_btn)
        self.side_layout.addWidget(env_group)

        # Recovery
        rec_group = QGroupBox("RECOVERY")
        rec_form = QFormLayout(rec_group)
        self.rec_inputs = {
            "Chute Dia (m)": QLineEdit("0.74"),
            "Chute Cd": QLineEdit("1.5"),
            "Chute Delay (s)": QLineEdit("2.0"),
        }
        for l, w in self.rec_inputs.items():
            self._decorate_numeric(w, l)
            rec_form.addRow(l, w)
        self.side_layout.addWidget(rec_group)

        # Chart toggles
        self.side_layout.addSpacing(10)
        tg_label = QLabel("CHART SERIES")
        tg_label.setStyleSheet(f"color: {FAINT}; font-weight: bold;")
        self.side_layout.addWidget(tg_label)
        # column -> (label, csv column, color, axis)
        self.series = {
            "Altitude (m)": ("alt", BLUE, "left"),
            "Velocity (m/s)": ("vel_m", GREEN, "left"),
            "Acceleration (m/s²)": ("accel", AMBER, "left"),
            "Thrust (N)": ("thrust", "#ec4899", "left"),
            "Pitch (°)": ("pitch", "#a78bfa", "right"),
            "Yaw (°)": ("yaw", "#22d3ee", "right"),
            "Fin X (°)": ("fin_x", "#f97316", "right"),
        }
        self.toggles = {}
        for label in self.series:
            cb = QCheckBox(label)
            cb.setChecked(label in ("Altitude (m)", "Velocity (m/s)", "Acceleration (m/s²)"))
            cb.stateChanged.connect(self.update_graph)
            self.toggles[label] = cb
            self.side_layout.addWidget(cb)

        self.side_layout.addSpacing(14)
        self.run_btn = QPushButton("RUN SIMULATION")
        self.run_btn.setObjectName("RunBtn")
        self.run_btn.setToolTip("Write rocket_config.json and run main.py (Enter).")
        self.run_btn.clicked.connect(self.run_sim)
        self.side_layout.addWidget(self.run_btn)

        self.viz_btn = QPushButton("LAUNCH 3D VISUALIZER")
        self.viz_btn.setObjectName("LaunchBtn")
        self.viz_btn.clicked.connect(self.launch_3d)
        self.side_layout.addWidget(self.viz_btn)

        self.side_layout.addStretch()
        sidebar_scroll.setWidget(sidebar_widget)

        # Live stability update when CP/CG/Diameter change
        for key in ("CP Dist (m)", "CG Dist (m)", "Diameter (m)"):
            self.inputs[key].textChanged.connect(self.update_stability_gauge)

        # --- Center: chart + last-run summary ---
        center = QWidget()
        center_layout = QVBoxLayout(center)
        center_layout.setContentsMargins(0, 0, 0, 0)

        self.summary_bar = QLabel("No simulation run yet.")
        self.summary_bar.setStyleSheet(
            f"color: {MUTED}; background: {PANEL}; border: 1px solid {BORDER}; "
            f"border-radius: 6px; padding: 8px 12px;")
        center_layout.addWidget(self.summary_bar)

        self.canvas = MplCanvas()
        center_layout.addWidget(self.canvas, 1)

        # --- Right: report ---
        self.report_scroll = QScrollArea()
        self.report_scroll.setFixedWidth(280)
        self.report_scroll.setWidgetResizable(True)
        report_widget = QWidget()
        self.report_layout = QVBoxLayout(report_widget)

        # Grouped stats: (group title, [(report_key, display, unit)])
        groups = [
            ("FLIGHT", [
                ("Apogee", "Apogee", "m"),
                ("Max Velocity", "Max Velocity", "m/s"),
                ("Max Mach", "Max Mach", ""),
                ("Burn Time", "Burn Time", "s"),
                ("Apogee Time", "Time to Apogee", "s"),
            ]),
            ("RECOVERY", [
                ("Impact Velocity", "Impact Speed", "m/s"),
                ("Descent Time", "Descent Time", "s"),
                ("Required Chute Dia", "Req. Chute", "m"),
                ("Field Diameter", "Req. Field", "m"),
                ("Max Shock Force Kgf", "Shock Force", "kgf"),
                ("Recovery Success", "Recovery", ""),
            ]),
            ("STABILITY", [
                ("Stability Margin", "Static Margin", "cal"),
                ("CP", "Centre of Pressure", "m"),
                ("CG", "Centre of Gravity", "m"),
            ]),
        ]
        self.stats = {}
        self.stats_meta = {}
        for gtitle, rows in groups:
            gl = QLabel(gtitle)
            gl.setStyleSheet(f"color: {BLUE}; font-size: 11px; font-weight: 800; "
                             f"margin-top: 12px; letter-spacing: 1px;")
            self.report_layout.addWidget(gl)
            for key, disp, unit in rows:
                title = QLabel((disp + (f" ({unit})" if unit else "")).upper())
                title.setStyleSheet(f"color: {FAINT}; font-size: 10px; font-weight: bold; margin-top: 5px;")
                value = QLabel("--")
                value.setStyleSheet(f"color: {BLUE}; font-size: 16px; font-weight: 800;")
                self.stats[key] = value
                self.stats_meta[key] = unit
                self.report_layout.addWidget(title)
                self.report_layout.addWidget(value)

        self.advice_box = QLabel("No flight data yet.")
        self.advice_box.setWordWrap(True)
        self.advice_box.setStyleSheet(
            f"color: #e2e2e2; font-style: italic; border-top: 1px solid {BORDER}; "
            f"padding-top: 10px; margin-top: 10px;")
        self.report_layout.addWidget(self.advice_box)
        self.report_layout.addStretch()
        self.report_scroll.setWidget(report_widget)

        layout.addWidget(sidebar_scroll)
        layout.addWidget(center, 1)
        layout.addWidget(self.report_scroll)

    def _decorate_numeric(self, widget, label):
        widget.setToolTip(TOOLTIPS.get(label, ""))
        v = QDoubleValidator()
        v.setNotation(QDoubleValidator.Notation.StandardNotation)
        widget.setValidator(v)
        # Reset any red flag as the user types
        widget.textChanged.connect(lambda _t, w=widget: self._clear_invalid(w))

    def _clear_invalid(self, widget):
        if widget.property("invalid"):
            widget.setProperty("invalid", False)
            widget.setStyle(widget.style())

    def browse_motor_for_flight(self):
        browser = MotorSelectorDialog(self)
        if browser.exec():
            m = browser.selected()
            if m:
                self.env_inputs["Motor File"].setText(m["file"])
                msg = f"Motor set to {m['name']} ({m['file']})."
                # Bonus: auto-fill propellant mass for a ThrustCurve download.
                prop_g = m.get("prop_weight_g", 0.0) or 0.0
                if m.get("source") == "thrustcurve" and prop_g > 0:
                    self.inputs["Propellant Mass (kg)"].setText(f"{prop_g / 1000:.3f}")
                    msg += f" Propellant mass auto-filled ({prop_g / 1000:.3f} kg)."
                self.set_status(msg, GREEN)

    def keyPressEvent(self, event):
        if event.key() in (Qt.Key.Key_Return, Qt.Key.Key_Enter) and \
                self.tabs.currentWidget() is self.flight_tab:
            self.run_sim()
        else:
            super().keyPressEvent(event)

    # ------------------------------------------------------------------
    # Stability gauge
    # ------------------------------------------------------------------
    def calculate_stability(self):
        """Return (margin_cal, status_text, color) from the flight CP/CG/Diameter."""
        try:
            cp = float(self.inputs["CP Dist (m)"].text())
            cg = float(self.inputs["CG Dist (m)"].text())
            dia = float(self.inputs["Diameter (m)"].text())
        except ValueError:
            return None, "Enter valid CP, CG and diameter.", FAINT
        if dia <= 0:
            return None, "Diameter must be > 0.", RED
        margin = (cp - cg) / dia
        if margin < 1.0:
            return margin, "UNSTABLE - CG too far aft (<1 cal).", RED
        elif margin <= 2.0:
            return margin, "OPTIMAL stability (1-2 cal).", GREEN
        elif margin <= 3.0:
            return margin, "ACCEPTABLE - slightly overstable (2-3 cal).", AMBER
        return margin, "OVERSTABLE - heavy weathercocking (>3 cal).", RED

    def update_stability_gauge(self):
        margin, status, color = self.calculate_stability()
        if margin is None:
            self.margin_label.setText("-- CAL")
            self.margin_label.setStyleSheet(f"font-size: 30px; font-weight: 900; color: {FAINT};")
        else:
            self.margin_label.setText(f"{margin:.2f} CAL")
            self.margin_label.setStyleSheet(f"font-size: 30px; font-weight: 900; color: {color};")
        self.margin_status.setText(status)
        self.margin_status.setStyleSheet(f"color: {color};")

    # ------------------------------------------------------------------
    # Simulation (non-blocking via QProcess)
    # ------------------------------------------------------------------
    def build_config(self):
        """Validate all inputs and return the 15-key config dict. Raises on bad input."""
        all_widgets = {**self.inputs, **self.env_inputs, **self.rec_inputs}
        # Validate numeric fields (everything except Motor File)
        for label, w in all_widgets.items():
            if label == "Motor File":
                continue
            self._read_float(w, label)
        config = {}
        for key in CONFIG_KEYS:
            config[key] = all_widgets[key].text()
        return config

    def write_config(self):
        config = self.build_config()
        with open(CONFIG_PATH, "w") as f:
            json.dump(config, f, indent=4)
        return config

    def run_sim(self):
        if self.proc is not None:
            return  # already running
        try:
            self.write_config()
        except ValueError as e:
            self.set_status(str(e), RED)
            return

        self.run_btn.setEnabled(False)
        self.run_btn.setText("RUNNING...")
        self.set_status("Simulation running...", BLUE)

        self.proc = QProcess(self)
        self.proc.setProgram(sys.executable)
        self.proc.setArguments([MAIN_PY, CONFIG_PATH])
        self.proc.setWorkingDirectory(BASE_DIR)
        self.proc.finished.connect(self.on_sim_finished)
        self.proc.errorOccurred.connect(self.on_sim_error)
        self.proc.start()

    def on_sim_error(self, err):
        self.run_btn.setEnabled(True)
        self.run_btn.setText("RUN SIMULATION")
        self.set_status(f"Failed to start simulation ({err}). Check Python/main.py.", RED)
        self.proc = None

    def on_sim_finished(self, exit_code, exit_status):
        self.run_btn.setEnabled(True)
        self.run_btn.setText("RUN SIMULATION")
        proc = self.proc
        self.proc = None
        if exit_code != 0:
            stderr = ""
            if proc is not None:
                try:
                    stderr = bytes(proc.readAllStandardError()).decode("utf-8", "replace")
                except Exception:
                    stderr = ""
            msg = stderr.strip().splitlines()[-1] if stderr.strip() else "unknown error"
            self.set_status(f"Simulation failed (exit {exit_code}): {msg}", RED)
            return
        self.set_status("Simulation complete. Charts and report updated.", GREEN)
        self.update_graph()

    # ------------------------------------------------------------------
    # Telemetry chart + report
    # ------------------------------------------------------------------
    def update_graph(self):
        try:
            df = pd.read_csv(FLIGHT_CSV)
        except Exception as e:
            self.set_status(f"Could not read flight data: {e}", RED)
            return

        ax = self.canvas.axes
        fig = ax.figure
        # Clear the whole figure so any prior twin axis is removed
        fig.clear()
        ax = fig.add_subplot(111)
        self.canvas.axes = ax
        ax.set_facecolor(BG)
        ax.tick_params(colors=MUTED)
        ax.grid(True, color="#3f3f46", alpha=0.4)
        ax.set_xlabel("Time (s)", color=MUTED)
        ax.set_ylabel("Altitude / Velocity / Accel / Thrust", color=MUTED)
        for spine in ax.spines.values():
            spine.set_color(BORDER)

        ax_right = None
        plotted = False
        for label, cb in self.toggles.items():
            if not cb.isChecked():
                continue
            col, color, side = self.series[label]
            if col not in df.columns:
                continue
            target = ax
            if side == "right":
                if ax_right is None:
                    ax_right = ax.twinx()
                    ax_right.set_ylabel("Angle (deg)", color=MUTED)
                    ax_right.tick_params(colors=MUTED)
                    for spine in ax_right.spines.values():
                        spine.set_color(BORDER)
                target = ax_right
            target.plot(df['t'], df[col], label=label, color=color, linewidth=1.8)
            plotted = True

        # Chute-deploy marker at apogee_time + delay
        report = self.read_report()
        if report:
            try:
                deploy_t = float(report.get("Apogee Time", 0)) + \
                    float(self.rec_inputs["Chute Delay (s)"].text())
                ax.axvline(deploy_t, color=RED, linestyle="--", linewidth=1.2, alpha=0.8)
                ymax = ax.get_ylim()[1]
                ax.text(deploy_t, ymax, " chute deploy", color=RED, fontsize=8,
                        va='top', ha='left')
            except (ValueError, TypeError):
                pass

        # Combined legend across both axes
        handles, labels = ax.get_legend_handles_labels()
        if ax_right is not None:
            h2, l2 = ax_right.get_legend_handles_labels()
            handles += h2
            labels += l2
        if handles:
            ax.legend(handles, labels, facecolor=PANEL, edgecolor=BORDER,
                      labelcolor='white', fontsize=8)
        elif not plotted:
            ax.text(0.5, 0.5, "Select a chart series", color=FAINT,
                    ha='center', va='center', transform=ax.transAxes)

        fig.tight_layout()
        self.canvas.draw()

        if report:
            self.update_report(report)

    def read_report(self):
        if not os.path.exists(FLIGHT_REPORT):
            return None
        try:
            with open(FLIGHT_REPORT) as f:
                return json.load(f)
        except Exception:
            return None

    def update_report(self, r):
        for key, label in self.stats.items():
            if key not in r:
                continue
            val = r[key]
            unit = self.stats_meta.get(key, "")
            if isinstance(val, (int, float)):
                text = f"{val:g}" + (f" {unit}" if unit else "")
            else:
                text = str(val)
            label.setText(text)
            label.setStyleSheet(f"color: {BLUE}; font-size: 16px; font-weight: 800;")

        # Color-code impact velocity (>8 m/s is a hard landing)
        if "Impact Velocity" in r and "Impact Velocity" in self.stats:
            try:
                danger = float(r["Impact Velocity"]) > 8.0
            except (ValueError, TypeError):
                danger = False
            c = RED if danger else GREEN
            self.stats["Impact Velocity"].setStyleSheet(
                f"color: {c}; font-size: 16px; font-weight: 800;")

        # Recovery success color
        if "Recovery Success" in self.stats and "Recovery Success" in r:
            ok = str(r["Recovery Success"]).strip().lower() in ("yes", "true", "1", "success")
            self.stats["Recovery Success"].setStyleSheet(
                f"color: {GREEN if ok else RED}; font-size: 16px; font-weight: 800;")

        # Color-code stability margin (1-2 cal is the safe band)
        if "Stability Margin" in self.stats and "Stability Margin" in r:
            try:
                m = float(r["Stability Margin"])
                if 1.0 <= m <= 2.0:
                    c = GREEN
                elif 2.0 < m <= 3.0:
                    c = AMBER
                else:
                    c = RED
            except (ValueError, TypeError):
                c = BLUE
            self.stats["Stability Margin"].setStyleSheet(
                f"color: {c}; font-size: 16px; font-weight: 800;")

        advice = r.get("Material Advice", "")
        stab = r.get("Stability Status", "")
        self.advice_box.setText(f"ADVICE: {advice}\n\nSTABILITY: {stab}")

        # Top summary bar with pass/fail chips
        rec_ok = str(r.get("Recovery Success", "")).strip().lower() in \
            ("yes", "true", "1", "success")
        try:
            m = float(r.get("Stability Margin", 0))
            if 1.0 <= m <= 2.0:
                stab_word, stab_c = "PASS", GREEN
            elif 2.0 < m <= 3.0:
                stab_word, stab_c = "OK", AMBER
            else:
                stab_word, stab_c = "CHECK", RED
        except (ValueError, TypeError):
            stab_word, stab_c = "CHECK", AMBER
        apogee = r.get("Apogee", "?")
        impact = r.get("Impact Velocity", "?")
        rec_c = GREEN if rec_ok else RED
        self.summary_bar.setText(
            f"Apogee {apogee} m   ·   Impact {impact} m/s   ·   "
            f"Recovery: <b style='color:{rec_c};'>{'PASS' if rec_ok else 'FAIL'}</b>   ·   "
            f"Stability: <b style='color:{stab_c};'>{stab_word}</b>"
        )

    # ------------------------------------------------------------------
    # Visualizer
    # ------------------------------------------------------------------
    def launch_3d(self):
        if not os.path.exists(FLIGHT_CSV):
            self.set_status("Run a simulation first - no flight data for the visualizer.", AMBER)
            return
        try:
            subprocess.Popen([sys.executable, VISUALIZER_PY], cwd=BASE_DIR)
            self.set_status("Launching 3D visualizer...", BLUE)
        except Exception as e:
            self.set_status(f"Could not launch visualizer: {e}", RED)

    # ------------------------------------------------------------------
    # Project save / load / persistence
    # ------------------------------------------------------------------
    def save_project(self):
        project_file = {
            "builder": self.rocket_components,
            "flight_config": {
                "geom": {k: w.text() for k, w in self.inputs.items()},
                "env": {k: w.text() for k, w in self.env_inputs.items()},
                "rec": {k: w.text() for k, w in self.rec_inputs.items()},
            },
        }
        path, _ = QFileDialog.getSaveFileName(self, "Save Rocket Project", "",
                                              "RocketSim Files (*.rkt)")
        if not path:
            return
        if not path.endswith(".rkt"):
            path += ".rkt"
        try:
            with open(path, "w") as f:
                json.dump(project_file, f, indent=4)
            self.set_status(f"Project saved to {os.path.basename(path)}.", GREEN)
        except Exception as e:
            self.set_status(f"Save failed: {e}", RED)

    def load_project(self):
        path, _ = QFileDialog.getOpenFileName(self, "Open Rocket Project", "",
                                              "RocketSim Files (*.rkt)")
        if not path:
            return
        try:
            with open(path) as f:
                data = json.load(f)
        except Exception as e:
            self.set_status(f"Load failed: {e}", RED)
            return
        self.rocket_components = data.get("builder", [])
        self.update_schematic()
        self.refresh_sidebar()
        cfg = data.get("flight_config", {})
        self._apply_field_dict(self.inputs, cfg.get("geom", {}))
        self._apply_field_dict(self.env_inputs, cfg.get("env", {}))
        self._apply_field_dict(self.rec_inputs, cfg.get("rec", {}))
        self.update_stability_gauge()
        self.set_status(f"Project loaded from {os.path.basename(path)}.", GREEN)

    def _apply_field_dict(self, widgets, values):
        for k, v in values.items():
            if k in widgets:
                widgets[k].setText(str(v))

    def load_last_config(self):
        """On startup, load rocket_config.json into the flight fields if present."""
        if not os.path.exists(CONFIG_PATH):
            return
        try:
            with open(CONFIG_PATH) as f:
                cfg = json.load(f)
        except Exception:
            return
        merged = {**self.inputs, **self.env_inputs, **self.rec_inputs}
        for k, v in cfg.items():
            if k in merged:
                merged[k].setText(str(v))
        self.set_status("Loaded last-used flight config.", MUTED)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MissionControl()
    window.show()
    sys.exit(app.exec())
