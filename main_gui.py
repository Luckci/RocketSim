import sys
import json
import subprocess
import pandas as pd
import numpy as np
import os
from PyQt6.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, 
                             QFormLayout, QLineEdit, QPushButton, QLabel, 
                             QFrame, QCheckBox, QGroupBox, QScrollArea)
from PyQt6.QtCore import Qt
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

MODERN_STYLE = """
QWidget { background-color: #09090b; color: #a1a1aa; font-family: 'Inter', sans-serif; }
QGroupBox { border: 1px solid #27272a; border-radius: 8px; margin-top: 20px; padding-top: 10px; font-weight: bold; color: #f4f4f5; }
QLineEdit { background: transparent; border: none; border-bottom: 1px solid #27272a; padding: 4px; color: #3b82f6; font-weight: bold; }
QLineEdit:focus { border-bottom: 2px solid #3b82f6; }
QPushButton { background-color: #1d4ed8; color: white; border-radius: 4px; padding: 10px; font-weight: bold; }
QPushButton#LaunchBtn { background-color: #059669; }
QLabel#Header { color: white; font-size: 16px; font-weight: 800; }
"""

class MplCanvas(FigureCanvas):
    def __init__(self):
        fig = Figure(figsize=(8, 6), facecolor='#09090b')
        self.axes = fig.add_subplot(111)
        self.axes.set_facecolor('#09090b')
        self.axes.tick_params(colors='#71717a')
        self.axes.grid(True, color="#494949")
        super().__init__(fig)

class MissionControl(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("RocketSim | Advanced Telemetry")
        self.resize(1400, 900)
        self.setStyleSheet(MODERN_STYLE)

        main_layout = QHBoxLayout(self)
        
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
        self.report_panel = QGroupBox("FLIGHT SUMMARY")
        self.report_panel.setFixedWidth(250)
        self.report_layout = QVBoxLayout()

        self.stats = {
            "Apogee": QLabel("0.0 m"),
            "Max Velocity": QLabel("0.0 m/s"),
            "Max Acceleration": QLabel("0.0 G"),
            "Burn Time": QLabel("0.0 s"),
            "Landing Speed": QLabel("0.0 m/s")
        }

        for name, label in self.stats.items():
            name_label = QLabel(name)
            name_label.setStyleSheet("color: #71717a; font-size: 11px; font-weight: bold;")
            label.setStyleSheet("color: #3b82f6; font-size: 18px; font-weight: 800; margin-bottom: 10px;")
            self.report_layout.addWidget(name_label)
            self.report_layout.addWidget(label)

        self.report_layout.addStretch()
        self.report_panel.setLayout(self.report_layout)
        
        main_layout.addWidget(sidebar_scroll)
        main_layout.addWidget(self.canvas, 1)
        main_layout.addWidget(self.report_panel)

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

                # Update your labels or text area
                report_string = f"Apogee: {r['Apogee']}m\nStability: {r['Stability Status']}"
                # self.report_display.setText(report_string)
            else:
                print("Report file not found yet.")

            

        except Exception as e:
            print(f"Update error: {e}")

    def launch_3d(self):
        subprocess.Popen([sys.executable, "visualizer.py"])

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MissionControl()
    window.show()
    sys.exit(app.exec())