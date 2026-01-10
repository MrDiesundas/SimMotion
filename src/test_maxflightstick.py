# test_maxflightstick.py
# -*- coding: utf-8 -*-

"""
test script for maxflightstick.py
A gui shows, which buttons are pressed
Expected Harware:
- A gamecontroller or MaxFlightStick https://www.maxflightstick.com/
- make sure, the controller driver is installed and seen by Windows
cmd -> control joy.cpl
Author: jomu
Date:   10.01.2026
"""


import sys
from PySide6.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton,
    QVBoxLayout, QHBoxLayout
)
from PySide6.QtCore import Qt, QTimer

import subprocess

from maxflightstick import FlightStick   # <-- your class


class FlightStickGUI(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("FlightStick test V1.0")

        # --- Create FlightStick instance ---
        self.stick = FlightStick()

        # --- Buttons ---
        self.gamectl_btn = QPushButton("Open USB Game Controller Setup")
        self.gamectl_btn.clicked.connect(self.open_game_controller_panel)

        # --- UI elements ---
        layout = QVBoxLayout()
        layout.addWidget(QLabel("Pressed buttons turn green:"))
        layout.addWidget(self.gamectl_btn)

        self.labels = {}

        # Create a label for each button
        for btn in sorted(self.stick.BUTTON_MAP.keys()):
            lbl = QLabel(f"Button {btn}")
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setStyleSheet("background-color: #333; color: white; padding: 6px;")
            layout.addWidget(lbl)
            self.labels[btn] = lbl

        self.setLayout(layout)

        # --- Timer to update UI ---
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_ui)
        self.timer.start(20)  # 50 Hz UI refresh

    def open_game_controller_panel(self):
        try:
            subprocess.Popen(["control", "joy.cpl"])
            print("Opened USB Game Controller panel.")
        except Exception as e:
            print("Failed to open joy.cpl:", e)

    def update_ui(self):
        """Poll button states and update label colors."""
        for btn, lbl in self.labels.items():
            if self.stick.get_button(btn):
                lbl.setStyleSheet("background-color: green; color: black; padding: 6px;")
            else:
                lbl.setStyleSheet("background-color: #333; color: white; padding: 6px;")

    def closeEvent(self, event):
        """Ensure HID device closes cleanly."""
        self.stick.close()
        event.accept()


# ---------------------------------------------------------
# Run app
# ---------------------------------------------------------
if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui = FlightStickGUI()
    gui.resize(250, 300)
    gui.show()
    sys.exit(app.exec())





# import time
# from maxflightstick import FlightStick
#
# def main():
#     stick = FlightStick()
#
#     # Example callback functions
#     def on_button_5():
#         print("Button 5 pressed!")
#
#     def on_button_7():
#         print("Button 7 pressed!")
#
#     # Register callbacks
#     stick.on_press(5, on_button_5)
#     stick.on_press(7, on_button_7)
#
#     print("Listening for button presses... (Ctrl+C to exit)")
#
#     try:
#         while True:
#             # Print current button states every 0.5s
#             states = {btn: stick.get_button(btn) for btn in stick.BUTTON_MAP}
#             print("States:", states)
#             time.sleep(0.5)
#
#     except KeyboardInterrupt:
#         print("\nExiting...")
#
#     finally:
#         stick.close()
#         print("FlightStick closed.")
#
# if __name__ == "__main__":
#     main()
