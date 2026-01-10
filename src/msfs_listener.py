from PySide6.QtCore import QThread, Signal
from src.mobiflight_variable_requests import MobiFlightVariableRequests
import time
import ctypes
from ctypes import wintypes
from SimConnect import SimConnect
from SimConnect.Enum import SIMCONNECT_CLIENT_DATA_ID, SIMCONNECT_RECV_ID, SIMCONNECT_RECV_CLIENT_DATA
import logging, logging.handlers


class SimConnectMobiFlight(SimConnect):

    def __init__(self, auto_connect=True, library_path=None):
        self.client_data_handlers = []
        if library_path:
            super().__init__(auto_connect, library_path)
        else:
            super().__init__(auto_connect)
        # Fix missing types
        self.dll.MapClientDataNameToID.argtypes = [wintypes.HANDLE, ctypes.c_char_p, SIMCONNECT_CLIENT_DATA_ID]


    def register_client_data_handler(self, handler):
        if not handler in self.client_data_handlers:
            logging.info("Register new client data handler")
            self.client_data_handlers.append(handler)


    def unregister_client_data_handler(self, handler):
        if handler in self.client_data_handlers:
            logging.info("Unregister client data handler")
            self.client_data_handlers.remove(handler)


    def my_dispatch_proc(self, pData, cbData, pContext):
        dwID = pData.contents.dwID
        if dwID == SIMCONNECT_RECV_ID.SIMCONNECT_RECV_ID_CLIENT_DATA:
            client_data = ctypes.cast(pData, ctypes.POINTER(SIMCONNECT_RECV_CLIENT_DATA)).contents
            for handler in self.client_data_handlers:
                handler(client_data)
        else:
            super().my_dispatch_proc(pData, cbData, pContext)

class MSFSListener(QThread):
    """
    Minimal MSFS telemetry listener using MobiFlightVariableRequests.
    Emits ONLY raw telemetry via Qt signal:
        - pitch (deg)
        - roll (deg)
        - yaw (deg true)
        - airspeed (knots)
        - frame_rate (Hz)
    """

    incoming_signal = Signal(dict)

    def __init__(
        self,
        stop_event,
        status_callback=None,
        poll_hz=50,
        reconnect_delay=3.0,
        telemetry_callback=None,
    ):
        super().__init__()
        self.stop_event = stop_event
        self.status_callback = status_callback or (lambda txt, col=None: None)
        self.telemetry_callback = telemetry_callback  # for Teensy

        self.poll_dt = 1.0 / poll_hz
        self.reconnect_delay = reconnect_delay

        # Frame rate tracking
        self._cycle_counter = 0
        self._last_fps_time = time.time()
        self._frame_rate = 0.0

        self.sm = None
        self.mf = None

    # ---------- helpers ----------

    def _status(self, txt, color="white"):
        self.status_callback(txt, color or "")

    def _connect(self):
        """Establish SimConnect + MobiFlightVariableRequests."""
        self.sm = SimConnectMobiFlight()
        self.mf = MobiFlightVariableRequests(self.sm)
        self._status("SimConnect (MobiFlight) connected")

    def _disconnect(self):
        try:
            if self.sm:
                self.sm.exit()
        except Exception:
            pass
        self.sm = None
        self.mf = None
        self._status("SimConnect (MobiFlight) disconnected")

    # ---------- lifecycle ----------

    def stop_OLD(self):
        self.stop_event.set()
        self.wait()

    def stop(self):
        # 1) Signal thread to stop
        self.stop_event.set()

        # 2) Wait for thread to finish
        self.wait()

        # 3) Now it is safe to disconnect SimConnect
        try:
            if self.sm:
                self.sm.exit()
        except:
            pass

        self.sm = None
        self.mf = None

    # ---------- main loop with auto-reconnect ----------

    def run(self):
        while not self.stop_event.is_set():
            try:
                self._connect()
            except Exception as e:
                self._status(f"ERROR connecting SimConnect: {e}", "red")
                if self.stop_event.wait(self.reconnect_delay):
                    break
                continue

            try:
                self._poll_loop()
            except Exception as e:
                self._status(f"ERROR in MSFS listener loop: {e}", "red")

            # REMOVED 10.1.2026
            # self._disconnect()
            if self.stop_event.is_set():
                break
            if self.stop_event.wait(self.reconnect_delay):
                break

    def _poll_loop(self):
        while not self.stop_event.is_set():
            self._poll_simvars()
            time.sleep(self.poll_dt)

    # ---------- telemetry polling (raw) ----------

    def _poll_simvars(self):
        """
        Reads MSFS telemetry via MobiFlightVariableRequests.
        Emits raw values only.
        If telemetry is missing, emits zeros and warns the GUI.
        """
        try:
            pitch = self.mf.get("(A:PLANE PITCH DEGREES, Degrees)")
            roll = self.mf.get("(A:PLANE BANK DEGREES, Degrees)")
            yaw = self.mf.get("(A:PLANE HEADING DEGREES TRUE, Degrees)")
            airspeed = self.mf.get("(A:AIRSPEED INDICATED, Knots)")

            # Detect missing telemetry (None or invalid)
            if any(v is None for v in (pitch, roll, yaw, airspeed)):
                raise ValueError("Telemetry returned None")

        except Exception:
            # Warn GUI once per reconnect cycle
            self._status("WARNING: No telemetry available (sending zeros)", "yellow")

            pitch = 0.0
            roll = 0.0
            yaw = 0.0
            airspeed = 0.0

        # --- frame rate computation ---
        self._cycle_counter += 1
        now = time.time()
        dt = now - self._last_fps_time

        if dt >= 1.0:
            self._frame_rate = self._cycle_counter / dt
            self._cycle_counter = 0
            self._last_fps_time = now

        data = {
                "pitch": pitch,
                "roll": roll,
                "yaw": yaw,
                "airspeed": airspeed,
                "frame_rate": self._frame_rate,
                "timestamp": now,
            }
        # print(f'debug: {data}')
        # 1) Send to motion platform (callback)
        if self.telemetry_callback:
            self.telemetry_callback(data)

        # 2) Emit raw telemetry for GUI
        self.incoming_signal.emit(data)
