import sys
import json
import subprocess
import pandas as pd
import numpy as np
import os
from PyQt6.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, 
                            QFormLayout, QLineEdit, QPushButton, QLabel, 
                            QFrame, QCheckBox, QGroupBox, QScrollArea,
                            QTabWidget, QComboBox)
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

class MplCanvas(FigureCanvas):
    def __init__(self, parent=None, width=5, height=4, dpi=100):
        # Create the figure with the specified size
        fig = Figure(figsize=(width, height), dpi=dpi, facecolor='#09090b')
        self.axes = fig.add_subplot(111)
        # Set default styling for the axes
        self.axes.set_facecolor('#09090b')
        self.axes.tick_params(colors='#a1a1aa')
        super().__init__(fig)

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

        self.setup_builder_tab()
        self.set_flight_tab()
    
    def setup_builder_tab(self):
        layout = QHBoxLayout(self.builder_tab)
        
        # Component Tree
        self.controls = QGroupBox("ROCKET ASSEMBLY")
        self.controls.setFixedWidth(300)
        self.control_layout = QVBoxLayout()

        # Simple inputs, "modular" comming
        self.nose_len = QLineEdit("0.15")
        self.body_len = QLineEdit("0.50")
        self.body_dia = QLineEdit("0.04")

        # Connect text changes to the drawing engine
        self.nose_len.textChanged.connect(self.update_schematic)
        self.body_len.textChanged.connect(self.update_schematic)
        self.body_dia.textChanged.connect(self.update_schematic)

        self.control_layout.addWidget(QLabel("Nose Length (m)"))
        self.control_layout.addWidget(self.nose_len)
        self.control_layout.addWidget(QLabel("Body Length (m)"))
        self.control_layout.addWidget(self.body_len)
        self.control_layout.addWidget(QLabel("Diameter (m)"))
        self.control_layout.addWidget(self.body_dia)
        self.control_layout.addStretch()

        self.controls.setLayout(self.control_layout)

        # Schematic Canvas
        self.schematic_canvas = MplCanvas(width=5, height=8)

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
        try:
            ax = self.schematic_canvas.axes
            ax.clear()
            ax.set_facecolor('#09090b')
            ax.axis('off')

            # Get Values
            n_len = float(self.nose_len.text())
            b_len = float(self.body_len.text())
            dia = float(self.body_dia.text())
            radius = dia / 2

            # Draw body tube
            # Center of rocket is X=0, Bottom is at Y=0
            body_rect = plt.Rectangle((-radius, 0), dia, b_len,
                                    color='#27272a', ec='#3b82f6', lw=2)
            ax.add_patch(body_rect)
            
            # Draw nose cone
            nose_pts = [[-radius, b_len], [radius, b_len], [0, b_len + n_len]]
            nose_cone = plt.Polygon(nose_pts, color='#3b82f6', ec='white', lw=1)
            ax.add_patch(nose_cone)

            # Draw fins
            fin_w = radius * 2
            fin_h = radius * 1.5
            left_fin = plt.Polygon([[-radius, 0], [-radius-fin_w, 0], [-radius, fin_h]], color='#3b82f6', alpha=0.8)
            right_fin = plt.Polygon([[radius, 0], [radius+fin_w, 0], [radius, fin_h]], color='#3b82f6', alpha=0.8)
            ax.add_patch(left_fin)
            ax.add_patch(right_fin)

            # Set plot limits so rocket stays centered
            ax.set_ylim(-0.05, b_len + n_len + 0.1)
            ax.set_xlim(-0.3, 0.3)
            ax.set_aspect('equal') 

            self.schematic_canvas.draw()
        except ValueError:
            pass # Ignore errors while user is typing

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