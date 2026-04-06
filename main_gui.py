import sys
import json
import subprocess
import pandas as pd
import numpy as np
import os
from PyQt6.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, 
                            QFormLayout, QLineEdit, QPushButton, QLabel, 
                            QFrame, QCheckBox, QGroupBox, QScrollArea,
                            QTabWidget, QComboBox, QDialog, QDialogButtonBox)
from PyQt6.QtCore import Qt
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.pyplot as plt

MODERN_STYLE = """
QWidget { background-color: #09090b; color: #a1a1aa; font-family: 'Inter', sans-serif; }
QGroupBox { border: 1px solid #27272a; border-radius: 8px; margin-top: 20px; padding-top: 10px; font-weight: bold; color: #f4f4f5; }
QLineEdit { background: transparent; border: none; border-bottom: 1px solid #27272a; padding: 4px; color: #3b82f6; font-weight: bold; }
QLineEdit:focus { border-bottom: 2px solid #3b82f6; }
QPushButton { background-color: #1d4ed8; color: white; border-radius: 4px; padding: 10px; font-weight: bold; }
QPushButton#LaunchBtn { background-color: #059669; }
QLabel#Header { color: white; font-size: 16px; font-weight: 800; }
"""

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
            if key in ["type", "name"]: continue

            self.inputs[key] = QLineEdit(str(value))
            self.layout.addRow(f"{key.capitalize()}:", self.inputs[key])

        if part_data["type"] == "motor":
            self.motor_btn = QPushButton("BROWSE MOTOR DATABASE")
            self.motor_btn.setStyleSheet("background-color: #3b82f6; color: white; margin-bottom: 10px;")
            self.motor_btn.clicked.connect(self.open_motor_browser)
            self.layout.addRow("Selection", self.motor_btn)

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
            "body": {"type": "body", "name": "New Body Tube", "mass": 0.1, "length": 0.3, "diameter": 0.04},
            "nose": {"type": "nose", "name": "New Nose Cone", "mass": 0.05, "length": 0.15, "diameter": 0.04},
            "fins": {"type": "fins", "name": "Aft Fins", "mass": 0.05, "height": 0.1, "width": 0.08},
            "motor": {
                "type": "motor", 
                "name": "Motor Mount", 
                "mass": 0.05, 
                "length": 0.1, 
                "diameter": 0.029, 
                "motor_id": "None Selected", 
                "propellant_mass": 0.0}
            # Future expansion is now easy:
            # "parachute": {"type": "parachute", "name": "Chute", "mass": 0.02, "diameter": 0.3}
        }

        self.setup_builder_tab()
        self.set_flight_tab()
    
    def on_part_clicked(self, event):
        try:
            # Get the index of the part we clicked
            idx = int(event.artist.part_index)
            self.current_edit_idx = idx
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
            print(f"Modularly added: {part_type}")

            # Eventually add part card to the sidebar
            #self.refresh_sidebar()

    def setup_builder_tab(self):
        layout = QHBoxLayout(self.builder_tab)

        # --- Left: Component Manager ---
        self.controls = QGroupBox("CONSTRUCTION KIT")
        self.controls.setFixedWidth(320)
        self.control_layout = QHBoxLayout()

        # A scroll area to hold the "Part Cards"
        self.part_scroll = QScrollArea()
        self.part_container = QWidget()
        self.part_layout = QHBoxLayout(self.part_container)
        self.part_scroll.setWidget(self.part_container)
        self.part_scroll.setWidgetResizable(True)

        # Buttons to add new parts
        self.btn_layout = QHBoxLayout()
        for p_type in self.templates.keys():
            btn = QPushButton(f"+ {p_type.capitalize()}")
            # We use a default argument (t=p_type) to capture the current p_type in the loop
            btn.clicked.connect(lambda checked, t=p_type: self.add_component(t))
            self.btn_layout.addWidget(btn)

        self.control_layout.addLayout(self.btn_layout)
        self.control_layout.addWidget(self.part_scroll)
        self.controls.setLayout(self.control_layout)
        
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
            
            # Loop through the modular components list
            for i, part in enumerate(self.rocket_components):
                shape = None

                if part["type"] == "body":
                    w = float(part["diameter"])
                    h = float(part["length"])
                    # Draw Body Tube
                    shape = plt.Rectangle((-w/2, current_y), w, h, 
                                        color='#27272a', ec='#3b82f6', lw=2, picker=True)
                    current_y += h
                
                elif part["type"] == "nose":
                    w = float(part["diameter"])
                    h = float(part["length"])
                    # Draw Nose Cone
                    pts = [[-w/2, current_y], [w/2, current_y], [0, current_y + h]]
                    shape = plt.Polygon(pts, color='#3b82f6', ec='white', lw=1, picker=True)
                    current_y += h
                
                elif part["type"] == "fins":
                    # For fins, we assume they are attatched to the base of the rocket (y=0)
                    # or possibly stay at current_y if they need to be higher
                    fw = float(part["width"])
                    fh = float(part["height"])

                    # Get the diameter of the body they attatch to (usually the first body tube)
                    body_dia = 0.04 # fallback
                    for p in self.rocket_components:
                        if p["type"] == "body":
                            body_dia = float(p["diameter"])
                            break

                    r = body_dia / 2

                    # Right fin (Trapezoid)
                    right_pts = [[r, 0], [r + fw, 0], [r + fw*0.7, fh], [r, fh]]
                    shape_r = plt.Polygon(right_pts, color='#3b82f6', alpha=0.7, picker=True)
                    shape_r.part_index = i
                    ax.add_patch(shape_r)

                    # Left fin (mirror of right)
                    left_pts = [[-r, 0], [-r - fw, 0], [-r - fw*0.7, fh], [-r, fh]]
                    shape_l = plt.Polygon(left_pts, color='#3b82f6', alpha=0.7, picker=True)
                    shape_l.part_index = i
                    ax.add_patch(shape_l)

                    # Dont increment current_y for fins as they dont add hieght to the stack
                    continue

                elif part["type"] == "motor":
                    w = float(part["diameter"])
                    h = float(part["length"])
                    # Draw motor (starts slightly above 0 to ensure its in the tube)
                    shape = plt.Rectangle((-w/2, 0.01), w, h,
                                        color='#52525b', ec='#a1a1aa', lw=1, picker=True)
                    
                    # Add text label for the motor ID
                    ax.text(0, 0.01 + h/2, part["motor_id"], color='white',
                            ha='center', va='center', fontsize=7, fontweight='bold')

                if shape:
                    shape.part_index = i # Tag for the click handler
                    ax.add_patch(shape)
            
            # Set plot limits
            ax.set_ylim(-0.05, current_y + 0.2)
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
            df = pd.read_csv("flight_data.csv")
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
            if os.path.exists("flight_report.json"):
                with open("flight_report.json", "r") as f:
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

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MissionControl()
    window.show()
    sys.exit(app.exec())