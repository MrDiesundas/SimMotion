# SimMotion.py
# -*- coding: utf-8 -*-
"""
SimMotion
Refactored for MMF-based MotionCompensation:
- MotionPlatform in src/motion_platform.py
- WitMotionSensor in src/witmotion_sensor.py
- XPlaneListener in src/xplane_listener.py (legacy, currently unused)
History:
V3.0 - 10.01.2025 - JOMU
    - initial commit
V3.0.1 - 10.01.2025:
    - minor status text adaptation
    - robuster stop methode in xplane_listener
    - test scripts for all modules src/test_....py
    - COMPILE.bat -> to compile SimMotion
    - DEVELOPMENT Flag to compile ui during development
v3.0.2 - 12.01.2025:
    - minor adaptation in motion_start_homing, using now stream_to_platform flag
    - fix update_sliders_from_response float issue
    - motion_power_toggle -> reset homing_done flag
v3.0.3 - 12.01.2025:
    - adapt MSFSListener class with a interpolator (had doupt in frame rate)
    - adapt MotionPlatform class with filter for sending telemetry
    - fix in SimMotion save cmb_sim_software (was absent)
v3.0.4 - 16.01.2025:
    - MotionPlatform class with kalman filter
v3.0.5 - 24.01.2025:
    - MotionPlatform class adapt send_receive -> waiting until nothing comes anymore
    - adapt test_motion_platform accordingly
    - note: reduced stepper microstep from 40000 to 4000 -> use teensy_flight_simulator_v91.ino or higher
v3.0.6 - 25.01.2025:
    - adapt send_receive(self, message: str, wait=False, resp=None) with expected response to fix homing
"""

# TODO: update_sliders_from_response and sliders_update_from_response
# TODO: eliminate yaw
# TODO: cleanup avoid double function


import sys
import os
import time
import threading
from datetime import datetime
import configparser


from PySide6.QtWidgets import QApplication, QMainWindow
from PySide6.QtCore import Signal, QTimer, QMetaObject, Qt, Q_ARG
from PySide6.QtGui import QPixmap

import qdarktheme

from src.motion_platform import MotionPlatform
from src.witmotion_sensor import WitMotionSensor
from src.msfs_listener import MSFSListener
from src.xplane_listener import XPlaneListener
from src.maxflightstick import FlightStick
from src.key_helper import KeyHelper, VK_CONTROL, VK_INSERT


VERSION = "v3.0.6 - 25.01.2025"
BAUD_RATE = 115200
CONFIG_FILE = "config.ini"
DEVELOPMENT = False
# ------------------- START UI COMPILATION --------------------
import subprocess
from pathlib import Path

UI_FILE = Path("SimMotion.ui")
PY_FILE = Path("ui_SimMotion.py")

def compile_if_needed(ui: Path, py: Path):
    # If output doesn't exist → compile
    if not py.exists():
        print(f"Compiling (missing): {ui} → {py}")
        subprocess.run(["pyside6-uic", str(ui), "-o", str(py)], check=True)
        return

    # If UI is newer than PY → compile
    if ui.stat().st_mtime > py.stat().st_mtime:
        print(f"Compiling (updated): {ui} → {py}")
        subprocess.run(["pyside6-uic", str(ui), "-o", str(py)], check=True)
    else:
        print("UI unchanged, skipping compilation")

# Call it
if DEVELOPMENT:
    compile_if_needed(UI_FILE, PY_FILE)
# ------------------- END UI COMPILATION --------------------

from ui_SimMotion import Ui_SimMotion # the generated file

class MyWindow(QMainWindow):
    button7_pressed = Signal()
    button9_pressed = Signal()

    def __init__(self):
        super().__init__()

        # UI setup
        self.ui: Ui_SimMotion = Ui_SimMotion()  # use this to get rid off pycharm warnings
        self.ui.setupUi(self)  # load UI into this window
        self.move(1280, 70)
        self.ui.lbl_version.setText(f"{VERSION}")
        self.setStyleSheet(
            """
            QPushButton:pressed {
                background-color: green;
                color: white;
            }

            QPushButton:checked {
                background-color: green;
                color: white;
            }
        """
        )

        # external components
        # self.motion_platform: MotionPlatformThread | None = None
        self.wit_sensor: WitMotionSensor | None = None
        self.simulator = None  # legacy placeholder
        # self.msfs: MSFSListener | None = None  # TODO remove

        # threading & state
        self.stop_event = threading.Event()
        self.connected = False
        self.homing_done = False
        self.telemetry_thread = None
        self.motion_platform = None
        self.stream_to_platform = False

        # latest telemetry snapshots for GUI
        self.latest_incoming = None
        self.latest_outgoing = None

        self._connect_signals()
        self._populate_serial_ports()


        # GUI refresh timer (1 Hz)
        self.gui_timer = QTimer(self)
        self.gui_timer.timeout.connect(self.gui_refresh)
        self.gui_timer.start(1000)

        #TODO INCLUDE
        self.stick = None
        self.vr_active = False
        self.load_factors()

    # ---------------- UI setup ----------------
    def _connect_signals(self):
        self.ui.btn_connect.clicked.connect(self.connection_toggle)
        self.ui.btn_motion.clicked.connect(self.motion_stream_toggle)
        self.ui.btn_quit.clicked.connect(self.quit_app)
        self.ui.btn_axes.toggled.connect(self.motion_power_toggle)
        self.ui.btn_home.clicked.connect(self.motion_start_homing)
        # TERMINAL
        self.ui.btn_send.clicked.connect(lambda: self.terminal_send_receive_from_ui())
        self.ui.btn_clear.clicked.connect(self.ui.txt_term_rec.clear)
        # SLIDERS
        self.ui.sld_pitch.sliderReleased.connect(lambda: self.send_slider_command("P"))
        self.ui.sld_roll.sliderReleased.connect(lambda: self.send_slider_command("R"))
        self.ui.sld_yaw.sliderReleased.connect(lambda: self.send_slider_command("Y"))

        self.slider_update(self.ui.sld_pitch, self.ui.lbl_sld_pitch)
        self.slider_update(self.ui.sld_roll, self.ui.lbl_sld_roll)
        self.slider_update(self.ui.sld_yaw, self.ui.lbl_sld_yaw)

    def _populate_serial_ports(self):
        from serial.tools import list_ports

        self.ui.cmb_serial_motion.clear()
        self.ui.cmb_serial_imu.clear()
        for port in list_ports.comports():
            self.ui.cmb_serial_motion.addItem(port.device, port.device)
            self.ui.cmb_serial_imu.addItem(port.device, port.device)

    def slider_update(self, slider, label):
        slider.valueChanged.connect(lambda v: label.setText(str(v)))

    # ---------------- config ----------------
    def load_factors(self):
        config = configparser.ConfigParser()
        if os.path.exists(CONFIG_FILE):
            self.update_status("config.ini present")
            config.read(CONFIG_FILE)
            try:
                if "Factors" in config:
                    f = config["Factors"]
                    self.ui.txt_pitch_factor.setText(f.get("txt_pitch_factor", "3.2"))
                    self.ui.txt_roll_factor.setText(f.get("txt_roll_factor", "3.2"))
                    self.ui.txt_yaw_factor.setText(f.get("txt_yaw_factor", "1.0"))

                    self.ui.txt_pitch_dem_low_percent.setText(f.get("txt_pitch_dem_low_percent", "0"))
                    self.ui.txt_pitch_dem_low_knt.setText(f.get("txt_pitch_dem_low_knt", "0"))
                    self.ui.txt_pitch_dem_hi_percent.setText(f.get("txt_pitch_dem_hi_percent", "0"))
                    self.ui.txt_pitch_dem_hi_knt.setText(f.get("txt_pitch_dem_hi_knt", "0"))

                    self.ui.txt_roll_dem_low_percent.setText(f.get("txt_roll_dem_low_percent", "0"))
                    self.ui.txt_roll_dem_low_knt.setText(f.get("txt_roll_dem_low_knt", "0"))
                    self.ui.txt_roll_dem_hi_percent.setText(f.get("txt_roll_dem_hi_percent", "0"))
                    self.ui.txt_roll_dem_hi_knt.setText(f.get("txt_roll_dem_hi_knt", "0"))

                    self.ui.txt_yaw_dem_low_percent.setText(f.get("txt_yaw_dem_low_percent", "0"))
                    self.ui.txt_yaw_dem_low_knt.setText(f.get("txt_yaw_dem_low_knt", "0"))
                    self.ui.txt_yaw_dem_hi_percent.setText(f.get("txt_yaw_dem_hi_percent", "0"))
                    self.ui.txt_yaw_dem_hi_knt.setText(f.get("txt_yaw_dem_hi_knt", "0"))

                    self.ui.ckb_pitch_axes.setChecked(config.get("Factors", "ckb_pitch_axes", fallback="0") == "1")
                    self.ui.ckb_roll_axes.setChecked(config.get("Factors", "ckb_roll_axes", fallback="0") == "1")
                    self.ui.ckb_yaw_axes.setChecked(config.get("Factors", "ckb_yaw_axes", fallback="0") == "1")

                    self.ui.txt_sim_ip.setText(config.get("Factors", "txt_sim_ip", fallback="0.0.0.0"))
                    self.ui.txt_sim_port.setText(config.get("Factors", "txt_sim_port", fallback="49001"))

                    saved_port = config.get("Factors", "cmb_serial_motion", fallback="")
                    if saved_port:
                        idx = self.ui.cmb_serial_motion.findText(saved_port)
                        if idx >= 0:
                            self.ui.cmb_serial_motion.setCurrentIndex(idx)

                    saved_port_2 = config.get("Factors", "cmb_serial_imu", fallback="")
                    if saved_port_2:
                        idx2 = self.ui.cmb_serial_imu.findText(saved_port_2)
                        if idx2 >= 0:
                            self.ui.cmb_serial_imu.setCurrentIndex(idx2)

                    sim_software = config.get("Factors", "cmb_sim_software", fallback="")
                    if sim_software:
                        idx3 = self.ui.cmb_sim_software.findText(sim_software)
                        if idx3 >= 0:
                            self.ui.cmb_sim_software.setCurrentIndex(idx3)
            except Exception:
                self.update_status("config.ini has wrong format", "red")
        else:
            self.update_status("no config.ini found", "red")
            self.ui.txt_pitch_factor.setText("3.2")
            self.ui.txt_roll_factor.setText("3.2")
            self.ui.txt_yaw_factor.setText("1.0")

    def save_factors(self):
        config = configparser.ConfigParser()
        if os.path.exists(CONFIG_FILE):
            config.read(CONFIG_FILE)
        if "Factors" not in config:
            config["Factors"] = {}
        f = config["Factors"]

        f["txt_pitch_factor"] = self.ui.txt_pitch_factor.text()
        f["txt_roll_factor"] = self.ui.txt_roll_factor.text()
        f["txt_yaw_factor"] = self.ui.txt_yaw_factor.text()

        f["txt_pitch_dem_low_percent"] = self.ui.txt_pitch_dem_low_percent.text()
        f["txt_pitch_dem_low_knt"] = self.ui.txt_pitch_dem_low_knt.text()
        f["txt_pitch_dem_hi_percent"] = self.ui.txt_pitch_dem_hi_percent.text()
        f["txt_pitch_dem_hi_knt"] = self.ui.txt_pitch_dem_hi_knt.text()

        f["txt_roll_dem_low_percent"] = self.ui.txt_roll_dem_low_percent.text()
        f["txt_roll_dem_low_knt"] = self.ui.txt_roll_dem_low_knt.text()
        f["txt_roll_dem_hi_percent"] = self.ui.txt_roll_dem_hi_percent.text()
        f["txt_roll_dem_hi_knt"] = self.ui.txt_roll_dem_hi_knt.text()

        f["txt_yaw_dem_low_percent"] = self.ui.txt_yaw_dem_low_percent.text()
        f["txt_yaw_dem_low_knt"] = self.ui.txt_yaw_dem_low_knt.text()
        f["txt_yaw_dem_hi_percent"] = self.ui.txt_yaw_dem_hi_percent.text()
        f["txt_yaw_dem_hi_knt"] = self.ui.txt_yaw_dem_hi_knt.text()

        f["ckb_pitch_axes"] = "1" if self.ui.ckb_pitch_axes.isChecked() else "0"
        f["ckb_roll_axes"] = "1" if self.ui.ckb_roll_axes.isChecked() else "0"
        f["ckb_yaw_axes"] = "1" if self.ui.ckb_yaw_axes.isChecked() else "0"

        f["cmb_sim_software"] = self.ui.cmb_sim_software.currentText()
        f["cmb_serial_motion"] = self.ui.cmb_serial_motion.currentText()
        f["cmb_serial_imu"] = self.ui.cmb_serial_imu.currentText()
        f["txt_sim_ip"] = self.ui.txt_sim_ip.text()
        f["txt_sim_port"] = self.ui.txt_sim_port.text()

        with open(CONFIG_FILE, "w") as ini:
            config.write(ini)

    # --------------- TERMINAL ---------------------
    def terminal_send_receive_from_ui(self):
        msg = self.ui.txt_term_send.text()
        if self.motion_platform is not None:
            resp = self.motion_platform.send_receive(msg, wait=True)
            if resp and resp.strip():
                self.ui.txt_term_rec.append(resp.strip())


    # ---------------- IMU callback ----------------

    def imu_handle_update(self, roll_adj, pitch_adj, yaw_adj):
        """
        Called by WitMotionSensor for each new IMU sample (offset-corrected rig angles in degrees).
        WitMotionSensor itself writes to MMF for MotionCompensation; here we only update the GUI.
        """
        QMetaObject.invokeMethod(
            self.ui.lbl_IMU_roll,
            "setText",
            Qt.QueuedConnection,
            Q_ARG(str, f"{roll_adj:.2f}"),
        )
        QMetaObject.invokeMethod(
            self.ui.lbl_IMU_pitch,
            "setText",
            Qt.QueuedConnection,
            Q_ARG(str, f"{pitch_adj:.2f}"),
        )
        QMetaObject.invokeMethod(
            self.ui.lbl_IMU_yaw,
            "setText",
            Qt.QueuedConnection,
            Q_ARG(str, f"{yaw_adj:.2f}"),
        )


    # ---------------- connection management ----------------

    def connection_toggle(self):
        if not self.connected:
            self._connect_all()
            self.update_sliders_from_response()
        else:
            self.connection_disconnect_all()
            self.connected = False
            self.ui.btn_connect.setText("Connect")
            # self.update_status("Disconnected")

    def _connect_all(self):
        try:
            self.stop_event.clear()

            # --- Read UI values ---
            sim = self.ui.cmb_sim_software.currentText().lower()

            # --- 1. Start Motion Platform Thread ---
            self._start_motion_platform(self.ui.cmb_serial_motion.currentData())

            # --- 2. Start Simulator Listener ---
            if sim == "msfs2024":
                self._start_msfs_listener()
            elif sim == "xplane":
                self._start_xplane_listener()
            else:
                self.update_status("Unknown simulator selected", "red")
                raise RuntimeError("Unknown simulator selected")

            # --- 3. Start IMU sensor ---
            self._start_imu(self.ui.cmb_serial_imu.currentData())

            # --- 4. Start FlightStick ---
            self._start_flightstick()

            # --- Success ---
            self.connected = True
            self.ui.btn_connect.setText("Disconnect")
            self.update_status("Connections established")

        except Exception as e:
            self.update_status(f"Connection failed: {e}", "red")
            print(e)
            self.connection_disconnect_all()
            self.connected = False
            self.ui.btn_connect.setText("Connect")

    def _start_xplane_listener(self):
        self.update_status("Starting X-Plane listener...")
        self.simulator = XPlaneListener(
            self.ui.txt_sim_ip.text(),
            int(self.ui.txt_sim_port.text()),
            stop_event=self.stop_event,
            status_callback=self.update_status,
            telemetry_callback=self.motion_send_telemetry,
        )

        # GUI receives incoming telemetry to display in GUI
        self.simulator.incoming_signal.connect(self.gui_update_incoming)  
        self.simulator.start()
        self.update_status("Simulation software connected", "green")

    def _start_msfs_listener(self):
        self.update_status("Starting MSFS2024 listener...")
        self.simulator = MSFSListener(
            stop_event=self.stop_event,
            status_callback=self.update_status,
            poll_hz=50,
            telemetry_callback=self.motion_send_telemetry,
        )

        # GUI receives incoming telemetry to display in GUI
        self.simulator.incoming_signal.connect(self.gui_update_incoming)
        self.simulator.start()
        self.update_status("X-Plane connected", "green")

    def _start_motion_platform(self, motor_port: str):
        self.motion_platform = MotionPlatform(
            port=motor_port,
            baud=115200,
            status_callback=self.update_status,
            factors_provider=self.get_factors_for_motion,
            stop_stream_callback=self.motion_force_stop
        )

        # GUI receives outgoing motion telemetry
        self.motion_platform.outgoing_signal.connect(self.gui_update_outgoing)

        # Connect to Teensy
        self.motion_platform.connect()

    def _start_imu(self, wit_port: str):
        self.wit_sensor = WitMotionSensor(
            wit_port,
            BAUD_RATE,
            stop_event=self.stop_event,
            status_callback=self.update_status,
            imu_callback=self.imu_handle_update,
        )
        self.wit_sensor.connect()  # may throw
        self.wit_sensor.start_listening()

    def _start_flightstick(self):
        self.button7_pressed.connect(self.motion_stream_toggle)
        self.button9_pressed.connect(self.VR_motion_compensation_trigger)
        try:
            self.stick = FlightStick()
        except Exception:
            self.update_status("FlightStick not found", "red")
            raise RuntimeError("FlightStick not found")

        # button 7 toggles motion on/off
        self.stick.on_press(7, lambda: self.button7_pressed.emit())
        # button 9 trigger motion compensation
        self.stick.on_press(9, lambda: self.button9_pressed.emit())

    def connection_disconnect_all(self):
        self.update_status("Disconnecting...")

        # 1) Stop callbacks immediately
        self.stop_event.set()
        self.homing_done = False

        # 2) Stop simulator listener (XPlane or MSFS)
        if self.simulator is not None:
            sim = self.ui.cmb_sim_software.currentText().lower()
            try:
                self.simulator.stop()
                self.update_status(f"{sim} disconnected")
            except Exception as e:
                self.update_status(f"Simulator listener stop error: {e}", "red")
            self.simulator = None

        # 3) Stop motion platform
        if self.motion_platform is not None:
            try:
                # MotionPlatform has no .stop(), only disconnect()
                self.motion_platform.disconnect()
                self.update_status("motion platform disconnected")
            except Exception as e:
                self.update_status(f"MotionPlatform disconnect error: {e}", "red")
            self.motion_platform = None

        # 4) Stop IMU sensor
        if self.wit_sensor is not None:
            try:
                self.wit_sensor.stop_listening()
                self.wit_sensor.disconnect()
                self.update_status("IMU disconnected")
            except Exception as e:
                self.update_status(f"WitMotion disconnect error: {e}", "red")
            self.wit_sensor = None

        # 5) Stop FlightStick
        if self.stick is not None:
            try:
                self.button7_pressed.disconnect()
                self.button9_pressed.disconnect()
                self.stick.close()
                self.update_status("FlightStick disconnected")
            except Exception as e:
                self.update_status(f"FlightStick disconnect error: {e}", "red")
            self.stick = None

        self.update_status("Bye Bye")

    # ---------------- status & motion toggle ----------------
    def update_status(self, txt: str = "", color: str = ""):
        self.ui.lbl_status.setStyleSheet("" if color == "" else f"color: {color};")
        self.ui.lbl_status.setText(str(txt))
        QApplication.processEvents()
        print(txt)

    # ---------------- MOTION PLATFORM ----------------
    def motion_start_homing(self):
        if self.ui.btn_axes.isChecked():
            self.update_status("start motion platform homing", "yellow")
            previous_btn_state = self.ui.btn_motion.isChecked()
            self.ui.btn_motion.setChecked(False)
            try:
                ok = self.motion_platform.home()
            except Exception as e:
                ok = False
                self.homing_done = False
                self.update_status(f"homing error {e}" , "red")
            if ok and self.wit_sensor:
                self.wit_sensor.capture_offsets(real_yaw=float(self.ui.lbl_sim_yaw.text()))
                self.wit_sensor.reset_translation()
                self.homing_done = True
                # QTimer.singleShot(500, self._after_homing_settle)
                self.update_status("motion platform homed")
            self.ui.btn_motion.setChecked(previous_btn_state)
            self.update_sliders_from_response()
        else:
            self.update_status("motion platform not powered", "red")
        self.ui.btn_home.setChecked(self.homing_done)

    def send_slider_command(self, axis: str):
        if self.motion_platform is None or not self.motion_platform.serial.is_open:
            self.update_status("Serial port is not open.", "red")
            return
        self.ui.btn_home.setChecked(False)
        if axis == "P":
            val = self.ui.sld_pitch.value() * float(self.ui.txt_pitch_factor.text())
        elif axis == "R":
            val = self.ui.sld_roll.value() * float(self.ui.txt_roll_factor.text())
        elif axis == "Y":
            val = self.ui.sld_yaw.value() * float(self.ui.txt_yaw_factor.text())
        else:
            self.update_status("Invalid axis in send_slider_command", "red")
            return
        self.motion_platform.send_axis_value(axis, val)

    def update_sliders_from_response(self):
        if not self.motion_platform:
            return
        resp = self.motion_platform.request_status()
        if not resp or not resp.startswith("E;"):
            self.update_status(f"Invalid response format: {resp}", "red")
            return
        try:
            parts = resp[2:].split(";")
            values = {}
            for part in parts:
                if "=" in part:
                    key, val = part.split("=", 1)
                    try:
                        values[key.strip()] = float(val.strip())
                    except:
                        values[key.strip()] = val.strip()
            self.ui.sld_pitch.setValue(int(values.get("pitch", 0)))
            self.ui.sld_roll.setValue(int(values.get("roll", 0)))
            self.ui.sld_yaw.setValue(int(values.get("yaw", 0)))

            motion_power = int(values.get("motionPowerToggle", 0))
            self.ui.btn_axes.setChecked(motion_power == 1)
        except Exception as e:
            self.update_status(f"ERROR update_sliders_from_response: {e}", "red")

    def motion_stream_toggle(self, checked=None):
        if not hasattr(self, "motion_platform") or self.motion_platform is None:
            self.update_status("Motion thread not running", "red")
            return

        if not self.stream_to_platform:
            self.ui.btn_motion.setChecked(True)
            self.motion_power_toggle(True)
            self.motion_start_homing()
            self.motion_platform.set_enabled(True)
            self.stream_to_platform = True
        else:
            self.ui.btn_motion.setChecked(False)
            self.motion_power_toggle(False)
            self.motion_platform.set_enabled(False)
            self.stream_to_platform = False
        QApplication.processEvents()

    def motion_force_stop(self):
        print('force stop')
        print("motion_force_stop CALLED")

        self.ui.btn_motion.setChecked(False)
        # self.motion_power_toggle(False)
        #self.motion_platform.set_enabled(False)

    def motion_power_toggle(self, checked: bool):
        if self.connected and self.motion_platform is not None:
            self.ui.btn_axes.setChecked(checked)
            self.motion_platform.send_receive("M;1" if checked else "M;0")
            self.ui.btn_axes.setText("ON" if checked else "OFF")
            if not checked:
                self.homing_done = False
                self.ui.btn_home.setChecked(self.homing_done)
        else:
            self.ui.btn_axes.setChecked(False)
            self.update_status("Connect first", color="yellow")

    def motion_send_telemetry(self, data: dict):
        mp = self.motion_platform # check if motion_platform is alive
        if mp is None:
            return
        mp.receive_telemetry(data)

    def VR_motion_compensation_trigger(self):
        """
        Called by botton 9
        Sends CTRL+1 and updates UI button based on MotionCompensation log.
        """
        print('motion compensation triggered')
        def get_motioncompensation_status(log_path=None):
            if log_path is None:
                # C:\Users\jmcub\AppData\Local\OpenXR-MotionCompensation
                log_path = os.path.expanduser(r"~\\AppData\\Local\\OpenXR-MotionCompensation\\OpenXR-MotionCompensation.log")

            if not os.path.exists(log_path):
                self.update_status("no logfile from motion compensation", "red")
                return None

            status = None
            with open(log_path, "r", encoding="utf-8", errors="ignore") as f:
                for line in f:
                    s = line.strip().lower()
                    if "motion compensation activated" in s:
                        status = True
                        self.vr_active = True
                    elif "motion compensation deactivated" in s:
                        status = False
                        self.vr_active = False
            return status

        try:
            # press CTRL+INS
            KeyHelper.tap_combo(VK_CONTROL, VK_INSERT)
            self.update_status("Sent CTRL + INS")
            time.sleep(1)
            status = get_motioncompensation_status()
            self.update_status(f"motion compensation status: {status}")

            if status is not None:
                self.ui.btn_motion_compensation_reload.setChecked(status)
                self.ui.btn_motion_compensation_reload.setText("ON" if status else "OFF")
                self.update_status(f"Motion compensation: {status}")
                # self.ui.btn_motion_compensation_reload.setStyleSheet("background-color: green; color: white;" if status else "")
        except Exception as e:
            self.update_status(f"ERROR VR_motion_compensation_trigger: {e}", "red")

    # ---------------- GUI updates ----------------
    def gui_update_incoming(self, data: dict):
        self.latest_incoming = data

    def gui_update_outgoing(self, data: dict):
        self.latest_outgoing = data

    def gui_refresh(self):
        # Incoming telemetry (sim)
        if self.latest_incoming:
            data = self.latest_incoming

            def safe_fmt(value, fmt="{:.2f}"):
                try:
                    return fmt.format(float(value))
                except Exception:
                    return fmt.format(0.0)

            # UI labels
            self.ui.lbl_sim_pitch.setText(safe_fmt(data.get("pitch")))
            self.ui.lbl_sim_roll.setText(safe_fmt(data.get("roll")))
            self.ui.lbl_sim_yaw.setText(safe_fmt(data.get("yaw")))
            self.ui.lbl_sim_speed.setText(safe_fmt(data.get("airspeed")))
            self.ui.lbl_sim_update.setText(
                safe_fmt(data.get("frame_rate"), "{:.0f}")
            )

        # Outgoing telemetry (motion platform)
        if self.latest_outgoing:
            data = self.latest_outgoing
            # TODO cleanup time and update
            self.ui.lbl_motion_update.setText(f"{data.get('update_rate', 0):.0f}")
            self.ui.lbl_motion_timestamp.setText(datetime.now().strftime("%H:%M:%S"))
            self.ui.lbl_motion_pitch.setText(f"{data.get('pitch_converted', 0):.2f}")
            self.ui.lbl_motion_pitch_rate.setText(f"{data.get('pitch_rate_converted', 0):.2f}")
            self.ui.lbl_motion_roll.setText(f"{data.get('roll_converted', 0):.2f}")
            self.ui.lbl_motion_roll_rate.setText(f"{data.get('roll_rate_converted', 0):.2f}")
            self.ui.lbl_motion_yaw.setText(f"{data.get('yaw_converted', 0):.2f}")
            self.ui.lbl_motion_yaw_rate.setText(f"{data.get('yaw_rate_converted', 0):.2f}")

    # ---------------- scaling factors ----------------
    # TODO check _float_from_text
    def get_factors_for_motion(self):
        """
        Returns all GUI-configured scaling and deminute factors.
        MotionPlatformThread uses this to compute:
            - scaled angles
            - Euler rates
            - deminute based on airspeed
        """
        return {
            "pitch_enabled": self.ui.ckb_pitch_axes.isChecked(),
            "roll_enabled": self.ui.ckb_roll_axes.isChecked(),
            "yaw_enabled": self.ui.ckb_yaw_axes.isChecked(),
            "pitch_factor": float(self.ui.txt_pitch_factor.text() or 0),
            "roll_factor": float(self.ui.txt_roll_factor.text() or 0),
            "yaw_factor": float(self.ui.txt_yaw_factor.text() or 0),
            "pitch_dem": (
                float(self.ui.txt_pitch_dem_low_knt.text() or 0),
                float(self.ui.txt_pitch_dem_low_percent.text() or 0),
                float(self.ui.txt_pitch_dem_hi_knt.text() or 0),
                float(self.ui.txt_pitch_dem_hi_percent.text() or 0),
            ),
            "roll_dem": (
                float(self.ui.txt_roll_dem_low_knt.text() or 0),
                float(self.ui.txt_roll_dem_low_percent.text() or 0),
                float(self.ui.txt_roll_dem_hi_knt.text() or 0),
                float(self.ui.txt_roll_dem_hi_percent.text() or 0),
            ),
            "yaw_dem": (
                float(self.ui.txt_yaw_dem_low_knt.text() or 0),
                float(self.ui.txt_yaw_dem_low_percent.text() or 0),
                float(self.ui.txt_yaw_dem_hi_knt.text() or 0),
                float(self.ui.txt_yaw_dem_hi_percent.text() or 0),
            ),
        }

    # ---------------- shutdown ----------------
    def quit_app(self):
        try:
            self.update_status("Shutting down...")
            self.connection_disconnect_all()
        except Exception as e:
            self.update_status(f"Cleanup error: {e}", "red")

        try:
            self.save_factors()
            self.update_status("ini file has been written")
        except Exception as e:
            self.update_status(f"write ini file failed: {e}", "red")
        QApplication.quit()

    def closeEvent(self, event):
        self.quit_app()
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    qdarktheme.setup_theme()
    window = MyWindow()
    window.show()
    sys.exit(app.exec())
