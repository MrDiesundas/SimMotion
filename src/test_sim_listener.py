# test_sim_listener.py
# -*- coding: utf-8 -*-
"""
test script for msfs_listener.py and xplane_listener.py
A gui that shows stream of FPS, pitch, roll, yaw and airspeed

Expected Software:
- xplane -> set udp stream on and check data to be streamed
- msfs2024 -> make sure you have installed https://www.mobiflight.com/ with WASM (!)
and enables it in the msfs marked place

Author: jomu
Date:   10.01.2026
"""

import sys
import threading
from PySide6.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton,
    QVBoxLayout, QHBoxLayout, QComboBox, QLineEdit
)
from PySide6.QtCore import Qt, QTimer

from src.xplane_listener import XPlaneListener
from src.msfs_listener import MSFSListener   # your MSFS class
from src.mobiflight_variable_requests import MobiFlightVariableRequests
from src.simconnect_mobiflight import SimConnectMobiFlight


class MobiFlightCheckThread(threading.Thread):
    def __init__(self, callback, timeout=3.0):
        super().__init__(daemon=True)
        self.callback = callback
        self.timeout = timeout

    def run(self):
        result = self._detect_with_timeout()
        self.callback(result)

    def _detect_with_timeout(self):
        result_container = {"ok": None}

        def worker():
            try:
                sm = SimConnectMobiFlight()
                mf = MobiFlightVariableRequests(sm)
                mf.get("(A:PLANE PITCH DEGREES, Degrees)")
                sm.exit()
                result_container["ok"] = True
            except Exception:
                result_container["ok"] = False

        t = threading.Thread(target=worker)
        t.start()
        t.join(self.timeout)

        # If still running → SimConnect is stuck → treat as NOT installed
        if t.is_alive():
            return False

        return result_container["ok"]



class SimListenerGUI(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Simulator Listener Test V1.0")

        # -----------------------------
        # Layout
        # -----------------------------
        layout = QVBoxLayout()

        # --- Simulator dropdown ---
        self.sim_dropdown = QComboBox()
        self.sim_dropdown.addItems(["X-Plane", "MSFS 2024"])
        self.sim_dropdown.currentIndexChanged.connect(self.update_sim_ui)
        layout.addWidget(QLabel("Select Simulator:"))
        layout.addWidget(self.sim_dropdown)

        # --- X-Plane IP + Port ---
        self.ip_input = QLineEdit("0.0.0.0")
        self.port_input = QLineEdit("49001")

        layout.addWidget(QLabel("X-Plane UDP IP:"))
        layout.addWidget(self.ip_input)
        layout.addWidget(QLabel("X-Plane UDP Port:"))
        layout.addWidget(self.port_input)

        # --- Start/Stop button ---
        self.start_btn = QPushButton("Start Listener")
        self.start_btn.clicked.connect(self.toggle_listener)
        layout.addWidget(self.start_btn)

        # --- Status label ---
        self.status_lbl = QLabel("Status: idle")
        self.status_lbl.setAlignment(Qt.AlignCenter)
        self.status_lbl.setStyleSheet("background-color: #222; color: white; padding: 6px;")
        layout.addWidget(self.status_lbl)

        # --- Telemetry labels ---
        self.pitch_lbl = QLabel("Pitch: ---")
        self.roll_lbl = QLabel("Roll: ---")
        self.yaw_lbl = QLabel("Yaw: ---")
        self.speed_lbl = QLabel("Airspeed: ---")
        self.fps_lbl = QLabel("Frame Rate: ---")

        for lbl in (self.pitch_lbl, self.roll_lbl, self.yaw_lbl, self.speed_lbl, self.fps_lbl):
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setStyleSheet("background-color: #333; color: white; padding: 6px;")
            layout.addWidget(lbl)

        self.setLayout(layout)

        # -----------------------------
        # State
        # -----------------------------
        self.listener = None
        self.stop_event = None
        self.running = False

        # Telemetry storage
        self.last_data = {
            "pitch": 0,
            "roll": 0,
            "yaw": 0,
            "airspeed": 0,
            "frame_rate": 0,
        }

        # GUI update timer
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_labels)
        self.timer.start(50)

        # Hide X-Plane fields if MSFS selected
        self.update_sim_ui()

    # ---------------------------------------------------------
    # UI logic
    # ---------------------------------------------------------
    def update_sim_ui(self):
        is_xplane = self.sim_dropdown.currentText() == "X-Plane"
        self.ip_input.setVisible(is_xplane)
        self.port_input.setVisible(is_xplane)

    def detect_mobiflight(self):
        try:
            sm = SimConnectMobiFlight()
            mf = MobiFlightVariableRequests(sm)

            # Try reading a known variable
            test = mf.get("(A:PLANE PITCH DEGREES, Degrees)")

            sm.exit()
            return True

        except Exception:
            return False

    # ---------------------------------------------------------
    # Start/Stop listener
    # ---------------------------------------------------------
    def toggle_listener(self):
        sim = self.sim_dropdown.currentText()

        # X-Plane → skip MobiFlight detection
        if sim == "X-Plane":
            if not self.running:
                self.start_listener()
            else:
                self.stop_listener()
            return

        # MSFS → run MobiFlight detection
        self.status_update("Checking MobiFlight...", "yellow")
        thread = MobiFlightCheckThread(self._mobiflight_result)
        thread.start()

    def toggle_listener_OLD(self):
        self.status_update("Checking MobiFlight...", "yellow")

        # run detection in background
        thread = MobiFlightCheckThread(self._mobiflight_result)
        thread.start()

    def _mobiflight_result(self, ok):
        if ok:
            self.status_update("MobiFlight WASM detected", "green")
            if not self.running:
                self.start_listener()
            else:
                self.stop_listener()
        else:
            self.status_update(
                "MobiFlight NOT detected\nInstall MobiFlight WASM and enable it in MSFS2024 → Marketplace",
                "red"
            )

    def _mobiflight_result_OLD(self, ok):
        if ok:
            self.status_update("MobiFlight WASM detected", "green")
            if not self.running:
                self.start_listener()
            else:
                self.stop_listener()
        else:
            self.status_update(
                "MobiFlight NOT detected\nInstall MobiFlight WASM and enable it\n in MSFS2024 → Marketplace",
                "red"
            )

    def toggle_listener_OLD(self):
        ok = self.detect_mobiflight()
        if ok:
            self.status_update("MobiFlight WASM detected", "green")
            if not self.running:
                self.start_listener()
            else:
                self.stop_listener()
        else:
            self.status_update("MobiFlight NOT detected", "red")


    def start_listener(self):
        sim = self.sim_dropdown.currentText()
        self.stop_event = threading.Event()

        if sim == "X-Plane":
            ip = self.ip_input.text().strip()
            port = int(self.port_input.text().strip())

            self.listener = XPlaneListener(
                ip=ip,
                port=port,
                stop_event=self.stop_event,
                status_callback=self.status_update,
                telemetry_callback=self.telemetry_update,
            )
            self.listener.incoming_signal.connect(self.telemetry_update)
            self.listener.start()

        else:  # MSFS
            self.listener = MSFSListener(
                stop_event=self.stop_event,
                status_callback=self.status_update,
                telemetry_callback=self.telemetry_update,
            )
            self.listener.incoming_signal.connect(self.telemetry_update)
            self.listener.start()

        self.running = True
        self.start_btn.setText("Stop Listener")
        self.start_btn.setStyleSheet("background-color: green; color: black;")
        self.status_update("Listener started")

    def stop_listener(self):
        if self.listener:
            self.stop_event.set()
            try:
                self.listener.stop()
            except:
                pass

        self.listener = None
        self.running = False
        self.start_btn.setText("Start Listener")
        self.start_btn.setStyleSheet("")
        self.status_update("Listener stopped")

    # ---------------------------------------------------------
    # Callbacks
    # ---------------------------------------------------------
    def status_update(self, msg, color="white"):
        self.status_lbl.setText(f"Status: {msg}")
        self.status_lbl.setStyleSheet(f"background-color: #222; color: {color}; padding: 6px;")

    def telemetry_update(self, data):
        self.last_data = data

    # ---------------------------------------------------------
    # GUI update
    # ---------------------------------------------------------
    def update_labels(self):
        d = self.last_data
        self.pitch_lbl.setText(f"Pitch: {d['pitch']:.2f}")
        self.roll_lbl.setText(f"Roll: {d['roll']:.2f}")
        self.yaw_lbl.setText(f"Yaw: {d['yaw']:.2f}")
        self.speed_lbl.setText(f"Airspeed: {d['airspeed']:.2f}")
        self.fps_lbl.setText(f"Frame Rate: {d['frame_rate']:.1f} Hz")

    # ---------------------------------------------------------
    # Clean shutdown
    # ---------------------------------------------------------
    def closeEvent(self, event):
        self.stop_listener()
        event.accept()


# ---------------------------------------------------------
# Run app
# ---------------------------------------------------------
if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui = SimListenerGUI()
    gui.resize(350, 500)
    gui.show()
    sys.exit(app.exec())
