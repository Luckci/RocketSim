import sys
import json
from PyQt6.QtWidgets import (QApplication, QWidget, QVBoxLayout, QFormLayout, QLineEdit, QPushButton, QLabel, QFileDialog, QMessageBox)
from PyQt6.QtCore import Qt

class RocketGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("RocketSim | Project Control Tower")
        self.setMinimumWidth(400)
        
        # Main Layout
        self.layout = QVBoxLayout()
        self.form = QFormLayout()
        
        # Define fields
        self.fields = {
            "Rocket Name": QLineEdit("MyRocket"),
            "Dry Mass (kg)": QLineEdit("1.0"),
            "Diameter (m)": QLineEdit("0.077"),
            "CP Distance (m)": QLineEdit("0.6"),
            "CG Distance (m)": QLineEdit("0.5"),
            "Fin Area (m^2)": QLineEdit("0.0059")
        }
        
        # Add fields to form
        for label, widget in self.fields.items():
            self.form.addRow(label, widget)
            
        self.layout.addLayout(self.form)
        
        # Buttons
        self.run_btn = QPushButton("Save Config & Run")
        self.run_btn.setStyleSheet("background-color : #4CAF50; color : white;")
        self.run_btn.clicked.connect(self.save_and_run)
        self.layout.addWidget(self.run_btn)
        
        self.setLayout(self.layout)

    def save_and_run(self):
        # 1. Collect data
        data = {label: widget.text() for label, widget in self.fields.items()}
        
        # 2. Save to JSON
        try:
            with open("rocket_config.json", "w") as f:
                json.dump(data, f, indent=4)
            
            QMessageBox.information(self, "Success", "Configuration saved to rocket_config.json!")
            
            # 3. Here you would trigger the sim:
            # subprocess.run(["python", "main.py", "rocket_config.json"])
            print("Simulation trigger logic would go here.")
            
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Could not save config: {e}")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = RocketGUI()
    window.show()
    sys.exit(app.exec())