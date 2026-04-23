import sys
import json
import subprocess
import pandas as pd
import numpy as np
import os
from PyQt6.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, 
                            QFormLayout, QLineEdit, QPushButton, QLabel, 
                            QFrame, QCheckBox, QGroupBox, QScrollArea,
                            QTabWidget, QComboBox, QDialog, QDialogButtonBox,
                            QGridLayout, QFileDialog)
from PyQt6.QtCore import Qt
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.pyplot as plt

MODERN_STYLE = """
QWidget { background-color: #09090b; color: #a1a1aa; font-family: 'Inter', sans-serif; }
QGroupBox { border: 1px solid #27272a; border-radius: 8px; margin-top: 20px; padding-top: 10px; font-weight: bold; color: #f4f4f5; }
QLineEdit { background: transparent; border: none; border-bottom: 1px solid #27272a; padding: 4px; color: #3b82f6; font-weight: bold; }
QLineEdit:focus { border-bottom: 2px solid #3b82f6; }
QPushButton { background-color: #1d4ed8; color: white; border-radius: 4px; padding: 8px; font-size: 11px; }
QPushButton#LaunchBtn { background-color: #059669; }
QLabel#Header { color: white; font-size: 16px; font-weight: 800; }
"""

BASE_DIR = os.path.dirname(os.path.abspath(__file__))

# Utility function to get paths easily
def get_path(subfolder, filename):
    return os.path.join(BASE_DIR, subfolder, filename)

class RocketComponents:
    def __init__(self, name, mass, length, part_type):
        self.name = name
        self.mass = mass
        self.length = length
        self.part_type = part_type # 'nose', 'body', 'fins', etc.

    def get_cg(self, start_y):
        # Simplification: CG is in the middle of the part
        return start_y + (self.length / 2)

class PartEditDialog(QDialog):
    def __init__(self, part_data, parent=None):
        super().__init__(parent)
        self.setWindowTitle(f"Edit {part_data['name']}")
        self.layout = QFormLayout(self)
        self.parent_ui = parent
        self.delete_requested = False # Track if we want to kill this part

        self.inputs = {}

        # Dynamically create inputs based on what keys the part has
        for key, value in part_data.items():
            if key in ["type"]: continue

            self.inputs[key] = QLineEdit(str(value))
            self.layout.addRow(f"{key.capitalize()}:", self.inputs[key])

        if part_data["type"] == "motor":
            self.motor_btn = QPushButton("BROWSE MOTOR DATABASE")
            self.motor_btn.setStyleSheet("background-color: #3b82f6; color: white; margin-bottom: 10px;")
            self.motor_btn.clicked.connect(self.open_motor_browser)
            self.layout.addRow("Selection", self.motor_btn)
        
        elif part_data["type"] == "fins":
            help_text = QLabel("<small>Format: (x1,y1), (x2,y2), (x3,y3), (x4,y4)<br>"
                            "Starts from the bottom-inner corner.</small>")
            self.layout.addRow(help_text)

        self.buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel)
        self.buttons.accepted.connect(self.accept)
        self.buttons.rejected.connect(self.reject)

        self.delete_btn = QPushButton("DELETE COMPONENT")
        self.delete_btn.setStyleSheet("background-color: #ef4444; color: white; margin-top: 10px;")
        self.delete_btn.clicked.connect(self.request_delete)
        
        self.layout.addWidget(self.buttons)
        self.layout.addWidget(self.delete_btn)

    def open_motor_browser(self):
        browser = MotorSelectorDialog(self)
        if browser.exec():
            name, data = browser.get_selected_motor()
            # Update the text box in the dialog immediately
            if "motor_id" in self.inputs:
                self.inputs["motor_id"].setText(name)
            if "propellant_mass" in self.inputs:
                self.inputs["propellant_mass"].setText(str(data["prop_mass"]))
            if "mass" in self.inputs:
                self.inputs["mass"].setText(str(data["mass"]))

    def request_delete(self):
        self.delete_requested = True
        self.accept() # Close dialog as Accepted
    
    def get_values(self):
        # Return the updated dictionary
        results = {}
        for key, widget in self.inputs.items():
            try:
                results[key] = float(widget.text())
            except:
                results[key] = widget.text()
        return results

class MplCanvas(FigureCanvas):
    def __init__(self, parent=None, width=5, height=4, dpi=100):
        # Create the figure with the specified size
        fig = Figure(figsize=(width, height), dpi=dpi, facecolor='#09090b')
        self.axes = fig.add_subplot(111)
        # Set default styling for the axes
        self.axes.set_facecolor('#09090b')
        self.axes.tick_params(colors='#a1a1aa')
        super().__init__(fig)

class MotorSelectorDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Select Rocket Motor")
        self.setMinimumWidth(400)
        layout = QVBoxLayout(self)

        self.label = QLabel("Available Motors (Local Library):")
        layout.addWidget(self.label)

        # The List of Motors
        self.motor_list = QComboBox()
        # Mock data, will be replaced with ThrustCurve API
        self.motors = {
            "Estes D12-5": {"mass": 0.042, "prop_mass": 0.021, "avg_thrust": 12.0},
            "Estes E12-6": {"mass": 0.058, "prop_mass": 0.035, "avg_thrust": 12.0},
            "Cesaroni F15": {"mass": 0.120, "prop_mass": 0.060, "avg_thrust": 15.0}
        }
        self.motor_list.addItems(self.motors.keys())
        layout.addWidget(self.motor_list)

        self.select_btn = QPushButton("Select Motor")
        self.select_btn.clicked.connect(self.accept)
        layout.addWidget(self.select_btn)

    def get_selected_motor(self):
        name = self.motor_list.currentText()
        return name, self.motors[name]

class MissionControl(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("RocketSim | Advanced Telemetry")
        self.resize(1200, 800)
        self.setStyleSheet(MODERN_STYLE)

        # Main Layout
        self.layout = QVBoxLayout(self)

        # Toolbar
        self.toolbar = QHBoxLayout()
        self.save_btn = QPushButton("💾 SAVE PROJECT")
        self.load_btn = QPushButton("📂 LOAD PROJECT")
        self.save_btn.setStyleSheet("background-color: #27272a; padding: 5px;")
        self.load_btn.setStyleSheet("background-color: #27272a; padding: 5px;")
        
        self.save_btn.clicked.connect(self.save_project)
        self.load_btn.clicked.connect(self.load_project)
        
        self.toolbar.addWidget(QLabel("<b>ROCKETSIM PRO</b>"))
        self.toolbar.addStretch()
        self.toolbar.addWidget(self.load_btn)
        self.toolbar.addWidget(self.save_btn)
        self.layout.addLayout(self.toolbar)

        # Create Tabs
        self.tabs = QTabWidget()
        self.tabs.setStyleSheet("QTabBar::tab { background: #18181b; padding: 10px; min-width: 150px; } "
                                "QTabBar::tab:selected { background: #3b82f6; color: white; }")

        self.builder_tab = QWidget()
        self.flight_tab = QWidget()

        self.tabs.addTab(self.builder_tab, "VEHICLE BUILDER")
        self.tabs.addTab(self.flight_tab, "FLIGHT TELEMETRY")

        self.layout.addWidget(self.tabs)

        self.rocket_components = []
        self.current_edit_idx = None

        # MASTER TEMPLATES
        self.templates = {
            "body": {"type": "body", "name": "Body Tube", "mass": 0.1, "length": 0.3, "diameter": 0.04, "y_offset": 0.0},
            "nose": {"type": "nose", "name": "Nose Cone", "mass": 0.05, "length": 0.15, "diameter": 0.04, "y_offset": 0.3},
            "fins": {"type": "fins", "name": "Fins", "mass": 0.05, "points": "(0,0), (0.1,0), (0.07,0.08), (0,0.08)", "y_offset": 0.0},
            "motor": {"type": "motor", "name": "Motor", "mass": 0.05, "length": 0.1, "diameter": 0.029, "motor_id": "None", "y_offset": 0.01}
            # Future expansion is now easy:
            # "parachute": {"type": "parachute", "name": "Chute", "mass": 0.02, "diameter": 0.3}
        }

        self.setup_builder_tab()
        self.set_flight_tab()
    
    def on_part_clicked(self, event):
        try:
            # Get the index of the part we clicked
            idx = int(event.artist.part_index)
            self.open_edit_dialog_by_index(idx)
            part = self.rocket_components[idx]

            # Open the Dialog
            dialog = PartEditDialog(part, self)
            if dialog.exec():

                if dialog.delete_requested:
                    self.rocket_components.pop(idx)
                    print(f"Deleted part at index {idx}")
                else:

                    # Update the data with the new values from the popup
                    new_values = dialog.get_values()
                    self.rocket_components[idx].update(new_values)

                # Refresh the rocket
                self.update_schematic()
        except Exception as e:
            print(f"Click handler error: {e}")

    def add_component(self, part_type):
        if part_type in self.templates:
            # .copy() is important so changing one doesnt change them all!
            new_part = self.templates[part_type].copy()
            self.rocket_components.append(new_part)

            # Update drawing
            self.update_schematic()
            self.refresh_sidebar()
            print(f"Modularly added: {part_type}")

    def refresh_sidebar(self):
        # Clear the existing widgets in the list
        while self.part_list_layout.count():
            item = self.part_list_layout.takeAt(0)
            widget = item.widget()
            if widget:
                widget.deleteLater()

        # List of (index, part) sorted by y_offset decending
        indexed_parts = list(enumerate(self.rocket_components))
        sorted_parts = sorted(indexed_parts, key=lambda x: float(x[1].get('y_offset', 0)), reverse=True)

        # Rebuild the list from rocket_components
        for i, part in sorted_parts:
            # Create a frame for the "card"
            card = QFrame()
            card.setStyleSheet("""
                QFrame {
                    background-color: #1d1d20;
                    border: 1px solid #27272a;
                    border-radius: 6px;
                    margin-bottom: 2px;
                }
                QFrame:hover {border: 1px solid #3b82f6; }
            """)
            card_layout = QHBoxLayout(card)

            # Label with part name and type
            name_label = QLabel(f"<b>{part['name']}<b><br><small>{part['type'].upper()}</small>")
            name_label.setStyleSheet("border: none; color:#f4f4f5;")
            card_layout.addWidget(name_label)

            # Edit Button (Icon-style)
            edit_btn = QPushButton("✎")
            edit_btn.setFixedSize(30, 30)
            edit_btn.setStyleSheet("background: #27272a; padding: 0;")
            # Connect the button to the same click logic as the plot!
            edit_btn.clicked.connect(lambda checked, idx=i: self.open_edit_dialog_by_index(idx))
            
            card_layout.addWidget(edit_btn)
            self.part_list_layout.addWidget(card)

    def open_edit_dialog_by_index(self, idx):
        try:
            self.current_edit_idx = idx
            part = self.rocket_components[idx]

            dialog = PartEditDialog(part, self)
            if dialog.exec():
                if dialog.delete_requested:
                    self.rocket_components.pop(idx)
                else:
                    new_values = dialog.get_values()
                    self.rocket_components[idx].update(new_values)

                self.update_schematic()
                self.refresh_sidebar() # Refresh the list after editing/deleting
        except Exception as e:
            print(f"Dialog error: {e}")

    def setup_builder_tab(self):
        layout = QHBoxLayout(self.builder_tab)

        # --- Left: Component Manager ---
        self.controls = QGroupBox("CONSTRUCTION KIT")
        self.controls.setFixedWidth(320)
        self.control_layout = QVBoxLayout(self.controls)

        # Dynamic Button Grid
        self.btn_container = QWidget()
        self.btn_grid = QGridLayout(self.btn_container)
        self.btn_grid.setContentsMargins(0,5,0,5)
        self.btn_grid.setSpacing(8)

        # Loop though templates and arrange in 2 columns
        for i, p_type in enumerate(self.templates.keys()):
            btn = QPushButton(f"+ {p_type.capitalize()}")
            btn.setMinimumHeight(40)
            btn.clicked.connect(lambda checked, t=p_type: self.add_component(t))

            row = i // 2
            col = i % 2
            self.btn_grid.addWidget(btn, row, col)

            self.control_layout.addWidget(self.btn_container)

        self.control_layout.addWidget(QLabel("ACTIVE COMPONENTS:"))

        self.part_scroll = QScrollArea()
        self.part_scroll.setWidgetResizable(True)
        self.part_scroll.setFrameShape(QFrame.Shape.NoFrame)
        self.part_scroll.setStyleSheet("background: #18181b; border-radius: 5px;")

        self.part_container = QWidget()
        self.part_list_layout = QVBoxLayout(self.part_container)
        self.part_list_layout.setAlignment(Qt.AlignmentFlag.AlignTop)
        self.part_scroll.setWidget(self.part_container)
        
        self.control_layout.addWidget(self.part_scroll)
        
        # Schematic Canvas
        self.schematic_canvas = MplCanvas(width=5, height=8)
        self.schematic_canvas.mpl_connect('pick_event', self.on_part_clicked)

        # Stability Gauge
        self.stability_panel = QGroupBox("STABILITY ANALYSIS")
        self.stability_panel.setFixedWidth(250)
        self.stab_layout = QVBoxLayout()
        self.margin_label = QLabel("0.00 CAL")
        self.margin_label.setStyleSheet("font-size: 30px; font=weight: 900; color: #3b82f6")
        self.stab_layout.addWidget(self.margin_label)
        self.stability_panel.setLayout(self.stab_layout)

        layout.addWidget(self.controls)
        layout.addWidget(self.schematic_canvas, 1)
        layout.addWidget(self.stability_panel)

    def set_flight_tab(self):
        layout = QHBoxLayout(self.flight_tab)
        # --- Sidebar with Scroll ---
        sidebar_scroll = QScrollArea()
        sidebar_scroll.setFixedWidth(350)
        sidebar_scroll.setWidgetResizable(True)
        sidebar_scroll.setFrameShape(QFrame.Shape.NoFrame)
        
        sidebar_widget = QWidget()
        self.side_layout = QVBoxLayout(sidebar_widget)
        
        self.side_layout.addWidget(QLabel("FLIGHT CONFIGURATION", objectName="Header"))

        # Group 1: Vehicle Geometry
        geom_group = QGroupBox("VEHICLE")
        self.geom_form = QFormLayout()
        self.inputs = {
            "Total Length (m)": QLineEdit("1.2"),
            "Dry Mass (kg)": QLineEdit("1.0"),
            "Propellant Mass (kg)": QLineEdit("0.126"),
            "Diameter (m)": QLineEdit("0.077"),
            "CP Dist (m)": QLineEdit("0.6"),
            "CG Dist (m)": QLineEdit("0.5"),
            "Fin Area (m²)": QLineEdit("0.006")
        }
        for l, w in self.inputs.items(): self.geom_form.addRow(l, w)
        geom_group.setLayout(self.geom_form)
        self.side_layout.addWidget(geom_group)

        # Group 2: Environment
        env_group = QGroupBox("ENVIRONMENT")
        self.env_form = QFormLayout()
        self.env_inputs = {
            "Motor File": QLineEdit("AeroTech_F40W.csv"),
            "Rail Angle (°)": QLineEdit("90"),
            "Rail Length (m)": QLineEdit("1.5"),
            "Wind Speed (m/s)": QLineEdit("5.0"),
            "Air Density (kg/m³)": QLineEdit("1.225")
        }
        for l, w in self.env_inputs.items(): self.env_form.addRow(l, w)
        env_group.setLayout(self.env_form)
        self.side_layout.addWidget(env_group)

        # Group 3: Recovery
        rec_group = QGroupBox("RECOVERY")
        self.rec_form = QFormLayout()
        self.rec_inputs = {
            "Chute Dia (m)": QLineEdit("0.74"),
            "Chute Cd": QLineEdit("1.5"),
            "Chute Delay (s)": QLineEdit("2.0")
        }
        for l, w in self.rec_inputs.items(): self.rec_form.addRow(l, w)
        rec_group.setLayout(self.rec_form)
        self.side_layout.addWidget(rec_group)

        # Toggles
        self.side_layout.addSpacing(20)
        self.toggles = {
            "Altitude (m)": QCheckBox("Altitude"),
            "Velocity (m/s)": QCheckBox("Velocity"),
            "Acceleration (m/s²)": QCheckBox("Acceleration")
        }
        for cb in self.toggles.values(): 
            cb.setChecked(True)
            cb.stateChanged.connect(self.update_graph)
            self.side_layout.addWidget(cb)

        self.run_btn = QPushButton("RUN SIMULATION")
        self.run_btn.clicked.connect(self.run_sim)
        self.side_layout.addWidget(self.run_btn)

        self.viz_btn = QPushButton("LAUNCH 3D VISUALIZER", objectName="LaunchBtn")
        self.viz_btn.clicked.connect(self.launch_3d)
        self.side_layout.addWidget(self.viz_btn)

        sidebar_scroll.setWidget(sidebar_widget)
        
        # --- Graph Area ---
        self.canvas = MplCanvas()

        # --- Report panel ---
        self.report_scroll = QScrollArea()
        self.report_scroll.setFixedWidth(280)
        self.report_scroll.setWidgetResizable(True)

        report_widget = QWidget()
        self.report_layout = QVBoxLayout(report_widget)

        self.stats_label = {
            "Apogee": "Apogee (m)",
            "Max Velocity": "Max Velocity (m/s)",
            "Impact Velocity": "Impact Speed (m/s)",
            "Stability Margin": "Stability (cal)",
            "Max Shock Force Kgf": "Shock Force (kgf)",
            "Required Chute Dia": "Req. Chute (m)",
            "Field Diameter": "Req. Field (m)"
        }

        self.stats = {} #Dictionary to store the actual QLabel objects

        for key, display_name in self.stats_label.items():
            title = QLabel(display_name.upper())
            title.setStyleSheet("color: #71717a; font-size: 10px; font-weight: bold; margin-top: 5px;")

            value_label = QLabel("0.0")
            value_label.setStyleSheet("color: #3b82f6; font-size: 16px; font-weight: 800;")

            self.stats[key] = value_label
            self.report_layout.addWidget(title)
            self.report_layout.addWidget(value_label)

        self.advice_box = QLabel("No flight data yet.")
        self.advice_box.setWordWrap(True)
        self.advice_box.setStyleSheet("color: #e2e2e2; font-style: italic; border-top: 1 px solid #27272a; padding-top: 10px;")
        self.report_layout.addWidget(self.advice_box)

        self.report_layout.addStretch()
        self.report_scroll.setWidget(report_widget)

        layout.addWidget(sidebar_scroll)
        layout.addWidget(self.canvas, 1)
        layout.addWidget(self.report_scroll)

    def update_schematic(self):

        if not self.rocket_components:
            ax = self.schematic_canvas.axes
            ax.clear()
            ax.axis('off')
            ax.set_facecolor('#09090b')
            ax.text(0, 0, "NO COMPONENTS", color='gray', ha='center')
            self.schematic_canvas.draw()
            return

        try:
            ax = self.schematic_canvas.axes
            ax.clear()
            ax.set_facecolor('#09090b')
            ax.axis('off')

            current_y = 0
            self.shapes = []
            max_height = 0
            
            # Loop through the modular components list
            for i, part in enumerate(self.rocket_components):
                y0 = float(part.get("y_offset", 0))
                w = float(part.get("diameter", 0.04))
                h = float(part.get("length", 0))

                max_height = max(max_height, y0 + h)

                shape = None

                if part["type"] == "body":
                    # Draw Body Tube
                    shape = plt.Rectangle((-w/2, y0), w, h, 
                                        color='#27272a', ec='#3b82f6', lw=2, picker=True)
                    current_y += h
                
                elif part["type"] == "nose":
                    # Draw Nose Cone
                    pts = [[-w/2, y0], [w/2, y0], [0, y0 + h]]
                    shape = plt.Polygon(pts, color='#3b82f6', ec='white', lw=1, picker=True)
                
                elif part["type"] == "fins":
                    try:
                        # Convert the string "(0,0), (0.1,0)..." into a list
                        raw_pts = eval(f"[{part['points']}]")

                        body_dia = 0.04
                        for p in self.rocket_components:
                            if p["type"] == "body":
                                body_dia = float(p["diameter"])
                                break
                        r = body_dia / 2

                        # Calculate Right Fin Points (Offset by radius)
                        right_pts = [[p[0] + r, p[1] + y0] for p in raw_pts]
                        # Calculate Left Fin Points (Mirrored)
                        left_pts = [[-p[0] - r, p[1] + y0] for p in raw_pts]

                        shape_r = plt.Polygon(right_pts, color='#3b82f6', alpha=0.7, picker=True)
                        shape_l = plt.Polygon(left_pts, color='#3b82f6', alpha=0.7, picker=True)

                        shape_r.part_index = i
                        shape_l.part_index = i
                        ax.add_patch(shape_r)
                        ax.add_patch(shape_l)
                    except Exception as e:
                        print(f"Fin geometry error: {e}")
                    continue

                elif part["type"] == "motor":
                    # Draw motor (starts slightly above 0 to ensure its in the tube)
                    shape = plt.Rectangle((-w/2, 0.01), w, h,
                                        color='#52525b', ec='#a1a1aa', lw=1, picker=True)
                    
                    # Add text label for the motor ID
                    ax.text(0, y0 + h/2, part["motor_id"], color='white',
                            ha='center', va='center', fontsize=7, fontweight='bold')

                if shape:
                    shape.part_index = i # Tag for the click handler
                    ax.add_patch(shape)
            
            # Set plot limits
            ax.set_ylim(-0.05, max_height + 0.1)
            ax.set_xlim(-0.5, 0.5)
            ax.set_aspect('equal')

            self.schematic_canvas.draw()

        except (ValueError, KeyError, IndexError) as e:
            print(f"Drawing error: {e}")

    def run_sim(self):
        # Merge all input dicts into one config
        all_vals = {**self.inputs, **self.env_inputs, **self.rec_inputs}
        config = {l: w.text() for l, w in all_vals.items()}
        
        with open("rocket_config.json", "w") as f:
            json.dump(config, f, indent=4)

        subprocess.run([sys.executable, "main.py", "rocket_config.json"])
        self.update_graph()

    def calculate_stability(self):
        cp = float(self.inputs["CP Dist (m)"].text())
        cg = float(self.inputs["CG Dist (m)"].text())
        dia = float(self.inputs["Diameter (m)"].text())
        
        # Stability is measured in "Calibers" (multiples of the diameter)
        margin = (cp - cg) / dia
        
        if margin < 1.0:
            status = "DANGEROUS / UNSTABLE"
        elif 1.0 <= margin <= 2.0:
            status = "OPTIMAL STABILITY"
        else:
            status = "OVERSTABLE (Heavy Weathercocking)"
        return f"Margin: {margin:.2f} cal ({status})"

    def update_graph(self):
        try:
            df_path = os.path.join(BASE_DIR, "data", "flight_data.csv")
            report_path = os.path.join(BASE_DIR, "data", "flight_report.json")
            df = pd.read_csv(df_path)
            self.canvas.axes.clear()
            self.canvas.axes.grid(True, color='#494949', alpha=0.5)
            
            # Simplified mapping for example
            map_cols = {"Altitude (m)": "alt", "Velocity (m/s)": "vel_m", "Acceleration (m/s²)": "accel"}
            colors = {"alt": "#3b82f6", "vel_m": "#10b981", "accel": "#f59e0b"}

            for label, cb in self.toggles.items():
                if cb.isChecked():
                    col = map_cols[label]
                    self.canvas.axes.plot(df['t'], df[col], label=label, color=colors[col], linewidth=2)

            self.canvas.axes.legend(facecolor='#18181b', edgecolor='#27272a', labelcolor='white')
            self.canvas.draw()

            # Update Report Panel
            if os.path.exists(report_path):
                with open(report_path, "r") as f:
                    r = json.load(f)

                for key in self.stats_label.keys():
                    if key in r:
                        self.stats[key].setText(str(r[key]))

                    advice_text = f"ADVICE: {r['Material Advice']}\n\nSTABILITY: {r['Stability Status']}"
                    self.advice_box.setText(advice_text)

                    # Highlight landing speed in rred if its too high
                    if r["Impact Velocity"] > 8.0:
                        self.stats["Impact Velocity"].setStyleSheet("color: #ef4444; font-size: 16px; font-weight: 800;")
                    else:
                        self.stats["Impact Velocity"].setStyleSheet("color: #10b981; font-size: 16px; font-weight: 800;")

                self.canvas.draw()
        except Exception as e:
            print(f"Update error: {e}")

    def launch_3d(self):
        subprocess.Popen([sys.executable, "visualizer.py"])

    def save_project(self):
        builder_data = self.rocket_components

        flight_data = {
            "geom": {k: w.text() for k, w in self.inputs.items()},
            "env": {k: w.text() for k, w in self.env_inputs.items()},
            "rec": {k: w.text() for k, w in self.rec_inputs.items()}
        }

        # Combine
        project_file = {
            "builder": builder_data,
            "flight_config": flight_data
        }

        # Open Save Dialog
        path, _ = QFileDialog.getSaveFileName(self, "Save Rocket Project", "", "RocketSim Files (*.rkt)")

        if path:
            with open(path, "w") as f:
                json.dump(project_file, f, indent=4)
            print(f"Project saved to {path}")

    def load_project(self):
        path, _ = QFileDialog.getOpenFileName(self, "Open Rocket Project", "", "RocketSim Files (*.rkt)")

        if path:
            with open(path, "r") as f:
                data = json.load(f)

            # Restore builder
            self.rocket_components = data.get("builder", [])
            self.update_schematic()
            self.refresh_sidebar()

            # Restore flight Tab
            f_config = data.get("flight_config", {})
            for k, val in f_config.get("geom", {}).items():
                if k in self.inputs: self.inputs[k].setText(val)
            for k, val in f_config.get("env", {}).items():
                if k in self.env_inputs: self.env_inputs[k].setText(val)
            for k, val in f_config.get("rec", {}).items():
                if k in self.rec_inputs: self.rec_inputs[k].setText(val)

            print(f"Project loaded from {path}")

    def sync_builder_to_flight(self):
        total_mass = sum(float(p.get('mass', 0)) + float(p.get('propellant_mass', 0)) for p in self.rocket_components)
        max_len = 0
        if self.rocket_components:
            # Simple length calc: finds the highest point
            max_len = max(float(p.get('y_offset', 0)) + float(p.get('length', 0)) for p in self.rocket_components)

        # Update the flight tab fields automatically
        self.inputs["Dry Mass (kg)"].setText(f"{total_mass:.3f}")
        self.inputs["Total Length (m)"].setText(f"{max_len:.3f}")
        print("Flight parameters updated from builder!")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MissionControl()
    window.show()
    sys.exit(app.exec())