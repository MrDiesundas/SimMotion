from PySide6.QtCore import QThread, Signal, QTimer
from src.mobiflight_variable_requests import MobiFlightVariableRequests
import time
import ctypes
from ctypes import wintypes
from SimConnect import SimConnect
from SimConnect.Enum import SIMCONNECT_CLIENT_DATA_ID, SIMCONNECT_RECV_ID, SIMCONNECT_RECV_CLIENT_DATA
import logging, logging.handlers
import threading


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

        # interpolator
        self.prev_sample = None
        self.last_sample = None
        self.epsilon = 0.0001  # threshold for detecting real new data
        self.interp_timer = None

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

    # For interpolator store new samples
    def _push_new_sample(self, sample):
        """
        Store new telemetry sample if it is actually new.
        """
        if self.last_sample is None:
            self.last_sample = sample
            return True

        # detect real change
        changed = (
                abs(sample["pitch"] - self.last_sample["pitch"]) > self.epsilon or
                abs(sample["roll"] - self.last_sample["roll"]) > self.epsilon or
                abs(sample["yaw"] - self.last_sample["yaw"]) > self.epsilon or
                abs(sample["airspeed"] - self.last_sample["airspeed"]) > self.epsilon
        )

        if not changed:
            return False

        # if changed:
        #     print(f"NEW SAMPLE at {sample['timestamp']:.3f}")

        # shift samples
        self.prev_sample = self.last_sample
        self.last_sample = sample
        return True

    # Interpolator
    def _interpolate(self):
        """
        Motion‑grade interpolation with jitter compensation and prediction.
        - Interpolates normally when MSFS updates are regular.
        - Predicts forward when MSFS stalls (jitter > jitter_threshold).
        """

        if self.prev_sample is None or self.last_sample is None:
            return self.last_sample

        t_now = time.time()
        t1 = self.prev_sample["timestamp"]
        t2 = self.last_sample["timestamp"]

        dt = t2 - t1
        if dt <= 0:
            return self.last_sample

        # --- compute velocities (deg/sec, knots/sec) ---
        vx_pitch = (self.last_sample["pitch"] - self.prev_sample["pitch"]) / dt
        vx_roll = (self.last_sample["roll"] - self.prev_sample["roll"]) / dt
        vx_yaw = (self.last_sample["yaw"] - self.prev_sample["yaw"]) / dt
        vx_airspeed = (self.last_sample["airspeed"] - self.prev_sample["airspeed"]) / dt

        # --- time since last real sample ---
        dt_now = t_now - t2

        # --- jitter threshold (msfs stalls) ---
        jitter_threshold = 0.040  # 40 ms → anything slower than 25 Hz triggers prediction

        # --- normal interpolation factor ---
        alpha = dt_now / dt
        alpha = max(0.0, min(1.0, alpha))

        # --- blending factor between interpolation and prediction ---
        # 0 = pure interpolation
        # 1 = pure prediction
        if dt_now <= jitter_threshold:
            blend = 0.0
        else:
            # smoothly increase prediction weight as stall grows
            blend = min(1.0, (dt_now - jitter_threshold) / 0.100)  # full prediction at +100ms stall

        # --- interpolation ---
        def lerp(a, b):
            return a + (b - a) * alpha

        interp_pitch = lerp(self.prev_sample["pitch"], self.last_sample["pitch"])
        interp_roll = lerp(self.prev_sample["roll"], self.last_sample["roll"])
        interp_yaw = lerp(self.prev_sample["yaw"], self.last_sample["yaw"])
        interp_airspeed = lerp(self.prev_sample["airspeed"], self.last_sample["airspeed"])

        # --- prediction ---
        pred_pitch = self.last_sample["pitch"] + vx_pitch * dt_now
        pred_roll = self.last_sample["roll"] + vx_roll * dt_now
        pred_yaw = self.last_sample["yaw"] + vx_yaw * dt_now
        pred_airspeed = self.last_sample["airspeed"] + vx_airspeed * dt_now

        # --- blend interpolation + prediction ---
        def blend_val(interp, pred):
            return interp * (1.0 - blend) + pred * blend

        return {
            "pitch": blend_val(interp_pitch, pred_pitch),
            "roll": blend_val(interp_roll, pred_roll),
            "yaw": blend_val(interp_yaw, pred_yaw),
            "airspeed": blend_val(interp_airspeed, pred_airspeed),
            "frame_rate": self.last_sample["frame_rate"],
            "timestamp": t_now,
        }

    def _interpolate_OLD(self):
        """
        Returns an interpolated sample between prev_sample and last_sample.
        If no interpolation possible, returns last_sample.
        """
        if self.prev_sample is None or self.last_sample is None:
            return self.last_sample

        t_now = time.time()
        t1 = self.prev_sample["timestamp"]
        t2 = self.last_sample["timestamp"]

        dt = t2 - t1
        if dt <= 0:
            return self.last_sample

        alpha = (t_now - t2) / dt
        alpha = max(0.0, min(1.0, alpha))  # clamp

        def lerp(a, b):
            return a + (b - a) * alpha

        return {
            "pitch": lerp(self.prev_sample["pitch"], self.last_sample["pitch"]),
            "roll": lerp(self.prev_sample["roll"], self.last_sample["roll"]),
            "yaw": lerp(self.prev_sample["yaw"], self.last_sample["yaw"]),
            "airspeed": lerp(self.prev_sample["airspeed"], self.last_sample["airspeed"]),
            "frame_rate": self.last_sample["frame_rate"],
            "timestamp": t_now,
        }

    def _emit_interpolated(self):
        """
        Called at 100 Hz. Sends interpolated data to motion platform.
        """
        if self.telemetry_callback is None:
            return

        interp = self._interpolate()
        if interp is not None:
            self.telemetry_callback(interp)

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
        # Start interpolation loop (100 Hz) inside the thread
        def interpolation_loop():
            while not self.stop_event.is_set():
                self._emit_interpolated()
                time.sleep(0.01)  # 100 Hz

        self.interp_thread = threading.Thread(target=interpolation_loop, daemon=True)
        self.interp_thread.start()

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
                "pitch": -pitch,
                "roll": -roll,
                "yaw": yaw,
                "airspeed": airspeed,
                "frame_rate": self._frame_rate,
                "timestamp": now,
            }
        # print(f'debug: {data}')
        # 1) Send to motion platform (callback)
        # if self.telemetry_callback:
        #     self.telemetry_callback(data)

        # Store sample for interpolation
        self._push_new_sample(data)

        # 2) Emit raw telemetry for GUI
        self.incoming_signal.emit(data)
