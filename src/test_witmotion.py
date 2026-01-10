# test_witmotion.py
# -*- coding: utf-8 -*-
"""
test script for witmotion_sensor.py
A gui that shows you pitch, roll and yaw values from a wit motion sensor.
https://www.wit-motion.com/
Make sure, you have set up the sensor accordingly with the witmotion software


Expected Harware:
e.g. WT901C RS232 or WT901C TTL with USB cable
https://de.aliexpress.com/item/1005003003817770.html
https://drive.google.com/file/d/1jhjOwvUfRpSJ8bJrPWObW4o1dszNC2gU/view

Author: jomu
Date:   10.01.2026
"""

import sys
from serial.tools import list_ports
import threading

from PySide6.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton,
    QVBoxLayout, QHBoxLayout, QComboBox
)
from PySide6.QtCore import Qt, QTimer

from witmotion_sensor import WitMotionSensor   # your class


class WitMotionGUI(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("WitMotion WT901 IMU Test V1.0")

        # -----------------------------
        # UI Layout
        # -----------------------------
        layout = QVBoxLayout()

        # --- COM port dropdown ---
        self.port_dropdown = QComboBox()
        self.refresh_ports()
        layout.addWidget(QLabel("Select COM Port:"))
        layout.addWidget(self.port_dropdown)

        # --- Refresh button ---
        self.refresh_btn = QPushButton("Refresh Ports")
        self.refresh_btn.clicked.connect(self.refresh_ports)
        layout.addWidget(self.refresh_btn)

        # --- Connect toggle ---
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.toggle_connection)
        layout.addWidget(self.connect_btn)

        # --- Start/Stop listening ---
        # self.listen_btn = QPushButton("Start Listening")
        # self.listen_btn.clicked.connect(self.toggle_listening)
        # self.listen_btn.setEnabled(False)
        # layout.addWidget(self.listen_btn)

        # --- IMU value labels ---
        self.roll_lbl = QLabel("Roll: ---")
        self.pitch_lbl = QLabel("Pitch: ---")
        self.yaw_lbl = QLabel("Yaw: ---")

        for lbl in (self.roll_lbl, self.pitch_lbl, self.yaw_lbl):
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setStyleSheet("background-color: #222; color: white; padding: 6px;")
            layout.addWidget(lbl)

        self.setLayout(layout)

        # -----------------------------
        # Sensor + State
        # -----------------------------
        self.sensor = None
        self.connected = False
        self.listening = False

        # Timer to update GUI values
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_labels)
        self.timer.start(50)  # 20 Hz GUI refresh

        # IMU values
        self.last_roll = 0
        self.last_pitch = 0
        self.last_yaw = 0

    # ---------------------------------------------------------
    # Port scanning
    # ---------------------------------------------------------
    def refresh_ports(self):
        self.port_dropdown.clear()
        ports = list_ports.comports()

        for p in ports:
            self.port_dropdown.addItem(f"{p.device} ({p.description})", p.device)

        if not ports:
            self.port_dropdown.addItem("No COM ports found", None)

    # ---------------------------------------------------------
    # Connect / Disconnect
    # ---------------------------------------------------------
    def toggle_connection(self):
        if not self.connected:
            port = self.port_dropdown.currentData()
            if port is None:
                print("No valid COM port selected.")
                return

            self.stop_event = threading.Event()

            # Create sensor instance
            self.sensor = WitMotionSensor(
                port=port,
                stop_event=self.stop_event,
                status_callback=self.status_update,
                imu_callback=self.imu_update
            )

            try:
                self.sensor.connect()
                self.connected = True
                self.connect_btn.setText("Disconnect")
                self.connect_btn.setStyleSheet("background-color: green; color: black;")
                # self.listen_btn.setEnabled(True)
                self.toggle_listening()
                print(f"Connected to IMU on {port}")
            except Exception as e:
                print("Connection error:", e)

        else:
            # Disconnect
            if self.sensor:
                self.sensor.disconnect()
            self.connected = False
            self.connect_btn.setText("Connect")
            self.connect_btn.setStyleSheet("")
            # self.listen_btn.setEnabled(False)
            self.toggle_listening()
            print("Disconnected.")

    # ---------------------------------------------------------
    # Start/Stop listening thread
    # ---------------------------------------------------------
    def toggle_listening(self):
        if not self.listening:
            if self.sensor:
                self.stop_event.clear()
                self.sensor.start_listening()
                self.listening = True
                # self.listen_btn.setText("Stop Listening")
                # self.listen_btn.setStyleSheet("background-color: green; color: black;")
                print("IMU listening started.")
        else:
            if self.sensor:
                self.sensor.stop_listening()
            self.stop_event.set()
            self.listening = False
            # self.listen_btn.setText("Start Listening")
            # self.listen_btn.setStyleSheet("")
            print("IMU listening stopped.")

    # ---------------------------------------------------------
    # Callbacks from WitMotionSensor
    # ---------------------------------------------------------
    def status_update(self, msg, color="white"):
        print(f"[STATUS] {msg}")

    def imu_update(self, roll, pitch, yaw):
        self.last_roll = roll
        self.last_pitch = pitch
        self.last_yaw = yaw

    # ---------------------------------------------------------
    # GUI update
    # ---------------------------------------------------------
    def update_labels(self):
        self.roll_lbl.setText(f"Roll:  {self.last_roll: .2f}°")
        self.pitch_lbl.setText(f"Pitch: {self.last_pitch: .2f}°")
        self.yaw_lbl.setText(f"Yaw:   {self.last_yaw: .2f}°")

    # ---------------------------------------------------------
    # Clean shutdown
    # ---------------------------------------------------------
    def closeEvent(self, event):
        if self.sensor:
            try:
                self.sensor.stop_listening()
                self.sensor.disconnect()
            except:
                pass
        event.accept()


# ---------------------------------------------------------
# Run app
# ---------------------------------------------------------
if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui = WitMotionGUI()
    gui.resize(300, 400)
    gui.show()
    sys.exit(app.exec())
