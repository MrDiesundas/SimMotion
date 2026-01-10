# test_motion_platform.py
# -*- coding: utf-8 -*-
"""
test script for motion_platform.py
A gui enables you to start a sine shape motion profile.
you can tune update frequency, amplitude and sine frequency on chosen axis
pitch, roll, yaw.

Expected Harware:
- SimMotion hardware (teensy controller, motion platform)

Author: jomu
Date:   10.01.2026
"""

import sys
import math
import time
import threading
from serial.tools import list_ports

from PySide6.QtWidgets import (
    QApplication, QWidget, QPushButton, QVBoxLayout,
    QLabel, QSlider
)
from PySide6.QtCore import Qt
from PySide6.QtWidgets import QComboBox
from PySide6.QtWidgets import QCheckBox

from motion_platform import MotionPlatform   # your class


class MotionTestGUI(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Motion Platform test V1.0")

        # --- UI ---
        self.connect_btn = QPushButton("Connect")
        self.enable_btn = QPushButton("Enable (M;1)")
        self.start_btn = QPushButton("Start streaming")
        self.port_dropdown = QComboBox()
        self.refresh_ports()

        self.pitch_cb = QCheckBox("Pitch")
        self.roll_cb = QCheckBox("Roll")
        self.yaw_cb = QCheckBox("Yaw")

        self.pitch_cb.setChecked(True)
        self.roll_cb.setChecked(True)
        self.yaw_cb.setChecked(True)

        # self.stop_btn = QPushButton("Stop Stream")

        self.rate_label = QLabel("Update Rate: 25 Hz")
        self.rate_slider = QSlider(Qt.Horizontal)
        self.rate_slider.setMinimum(1)
        self.rate_slider.setMaximum(200)
        self.rate_slider.setValue(25)

        self.amp_label = QLabel("Amplitude: 5°")
        self.amp_slider = QSlider(Qt.Horizontal)
        self.amp_slider.setMinimum(1)
        self.amp_slider.setMaximum(30)
        self.amp_slider.setValue(5)

        self.freq_label = QLabel("Frequency: 0.20 Hz")
        self.freq_slider = QSlider(Qt.Horizontal)
        self.freq_slider.setMinimum(1)  # represents 0.01 Hz
        self.freq_slider.setMaximum(100)  # represents 1.00 Hz
        self.freq_slider.setValue(20)  # default = 0.20 Hz

        layout = QVBoxLayout()
        layout.addWidget(self.port_dropdown)
        layout.addWidget(self.connect_btn)
        layout.addWidget(self.enable_btn)
        layout.addWidget(self.pitch_cb)
        layout.addWidget(self.roll_cb)
        layout.addWidget(self.yaw_cb)
        layout.addWidget(self.start_btn)
        # layout.addWidget(self.stop_btn)
        layout.addWidget(self.rate_label)
        layout.addWidget(self.rate_slider)
        layout.addWidget(self.amp_label)
        layout.addWidget(self.amp_slider)
        layout.addWidget(self.freq_label)
        layout.addWidget(self.freq_slider)
        self.setLayout(layout)

        # --- Motion Platform instance ---
        self.platform = MotionPlatform(
            port="COM3",
            baud=115200,
            status_callback=self.status_update,
            factors_provider=self.fake_factors
        )

        # --- state ---
        self.running = False
        self.thread = None
        self.motion_enabled = False
        self.connected = False
        self.streaming = False

        # --- signals ---
        self.connect_btn.clicked.connect(self.do_connect)
        self.enable_btn.clicked.connect(self.do_enable)
        self.start_btn.clicked.connect(self.toggle_stream)
        # self.stop_btn.clicked.connect(self.stop_stream)
        self.rate_slider.valueChanged.connect(self.update_rate_label)
        self.amp_slider.valueChanged.connect(self.update_amp_label)
        self.freq_slider.valueChanged.connect(self.update_freq_label)

    # ---------------------------------------------------------
    # Status callback
    # ---------------------------------------------------------
    def status_update(self, txt, color):
        print(f"[STATUS] {txt}")

    # ---------------------------------------------------------
    # Fake factors provider (simple 1:1 scaling)
    # ---------------------------------------------------------
    def fake_factors(self):
        return {
            "pitch_enabled": self.pitch_cb.isChecked(),
            "roll_enabled": self.roll_cb.isChecked(),
            "yaw_enabled": self.yaw_cb.isChecked(),

            "pitch_factor": 1.0,
            "roll_factor": 1.0,
            "yaw_factor": 1.0,

            "pitch_dem": (0, 100, 200, 100),
            "roll_dem": (0, 100, 200, 100),
            "yaw_dem": (0, 100, 200, 100),
        }

    def fake_factors_OLD(self):
        return {
            "pitch_enabled": True,
            "roll_enabled": True,
            "yaw_enabled": True,
            "pitch_factor": 1.0,
            "roll_factor": 1.0,
            "yaw_factor": 1.0,
            "pitch_dem": (0, 100, 200, 100),
            "roll_dem": (0, 100, 200, 100),
            "yaw_dem": (0, 100, 200, 100),
        }

    def update_amp_label(self, value):
        self.amp_label.setText(f"Amplitude: {value}°")

    def update_freq_label(self, value):
        freq = value / 100.0
        self.freq_label.setText(f"Frequency: {freq:.2f} Hz")

    # ---------------------------------------------------------
    # Connect + enable
    # ---------------------------------------------------------
    def refresh_ports(self):
        self.port_dropdown.clear()
        ports = list_ports.comports()

        for p in ports:
            # p.device is like "COM7"
            # p.description is like "USB-SERIAL CH340"
            self.port_dropdown.addItem(f"{p.device}  ({p.description})", p.device)

        if not ports:
            self.port_dropdown.addItem("No COM ports found", None)

    def do_connect(self):
        if not self.connected:
            # get selected port
            port = self.port_dropdown.currentData()
            if port is None:
                print("No valid COM port selected.")
                return

            self.platform.port = port

            try:
                self.platform.connect()
                self.connected = True
                self.connect_btn.setText("Disconnect")
                self.connect_btn.setStyleSheet("background-color: green; color: white;")
                print(f"Connected on {port}")
            except Exception as e:
                print("Connection error:", e)

        else:
            try:
                self.platform.disconnect()
                self.connected = False
                self.connect_btn.setText("Connect")
                self.connect_btn.setStyleSheet("")

                print("Disconnected.")
            except Exception as e:
                print("Disconnect error:", e)

    def do_enable(self):
        if not self.motion_enabled:
            # turn ON
            print("Sending M;1")
            self.platform.send_receive("M;1", wait=True)
            self.platform.home()
            self.motion_enabled = True
            self.platform.streaming_enabled = True
            self.enable_btn.setText("Disable Motion (M;0)")
            self.enable_btn.setStyleSheet("background-color: green; color: white;")

        else:
            # turn OFF
            print("Sending M;0")
            self.platform.send_receive("M;0", wait=True)
            self.motion_enabled = False
            self.platform.streaming_enabled = False
            self.enable_btn.setText("Enable Motion (M;1)")
            self.enable_btn.setStyleSheet("")


    # ---------------------------------------------------------
    # Start/stop streaming
    # ---------------------------------------------------------
    def toggle_stream(self):
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self.sine_loop, daemon=True)
            self.thread.start()
            self.platform.set_enabled(True)
            self.start_btn.setText("Stop streaming")
            self.start_btn.setStyleSheet("background-color: green; color: white;")
            print("Streaming started.")
        else:
            self.running = False
            self.platform.set_enabled(False)
            self.start_btn.setText("Start streaming")
            self.start_btn.setStyleSheet("")
            print("Streaming stopped.")


    # ---------------------------------------------------------
    # Update rate label
    # ---------------------------------------------------------
    def update_rate_label(self, value):
        self.rate_label.setText(f"Update Rate: {value} Hz")

    # ---------------------------------------------------------
    # Sine-wave telemetry generator
    # ---------------------------------------------------------
    def sine_loop(self):
        t = 0.0

        # --- rate measurement ---
        last_print = time.time()
        counter = 0

        while self.running:
            rate = self.rate_slider.value()
            dt = 1.0 / rate

            amp = self.amp_slider.value()
            freq = self.freq_slider.value() / 100.0

            telemetry = {
                "pitch": amp * math.sin(2 * math.pi * freq * t),
                "roll": amp * math.sin(2 * math.pi * freq * t),
                "yaw": amp * math.sin(2 * math.pi * freq * t),
                "airspeed": 100.0
            }

            self.platform.receive_telemetry(telemetry)

            # --- update rate measurement ---
            counter += 1
            now = time.time()
            if now - last_print >= 2.0:
                real_rate = counter / (now - last_print)
                print(f"[REAL RATE] {real_rate:.1f} Hz")
                counter = 0
                last_print = now

            time.sleep(dt)
            t += dt

    def sine_loop_old(self):
        t = 0.0

        while self.running:
            rate = self.rate_slider.value()
            dt = 1.0 / rate

            amp = self.amp_slider.value()  # degrees
            freq = self.freq_slider.value() / 100.0  # 0.01–1.00 Hz

            # --- generate synthetic telemetry ---
            telemetry = {
                "pitch": amp * math.sin(2 * math.pi * freq * t),
                "roll":  amp * math.sin(2 * math.pi * freq * t),
                "yaw":   0 * math.sin(2 * math.pi * freq * t),
                "airspeed": 1.0
            }

            # --- feed into your full pipeline ---
            self.platform.receive_telemetry(telemetry)

            time.sleep(dt)
            t += dt


# ---------------------------------------------------------
# Run app
# ---------------------------------------------------------
if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui = MotionTestGUI()
    gui.resize(300, 200)
    gui.show()
    sys.exit(app.exec())
