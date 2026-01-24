import time
import serial
from PySide6.QtCore import QObject, Signal
import threading

# todo check, where stop_stream_callback makes sense
# todo pack telemetry in a class to prepare json
# todo check send_and recieve if still works with simulator


class MotionPlatform(QObject):
    outgoing_signal = Signal(dict)
    def __init__(self, port, baud, status_callback=None, factors_provider=None, stop_stream_callback=None):
        super().__init__()  # REQUIRED for Qt signals to work

        self.port = port
        self.baud = baud
        self.serial = None
        self.streaming_enabled = False
        self.stop_stream_callback = stop_stream_callback

        self.status_callback = status_callback or (lambda *a, **k: None)
        self.factors_provider = factors_provider
        self.serial_lock = threading.Lock()  # <-- important

        # for rate calculation
        self.prev_time = None
        self.prev_pitch = 0.0
        self.prev_roll = 0.0
        self.prev_yaw = 0.0

        # FILTER
        self.filter = 'kal_filter' # exp_filter or kal_filter

        # exponential smoothing filter (simple solution)
        self.pitch_rate_smooth = 0.0
        self.roll_rate_smooth = 0.0
        self.yaw_rate_smooth = 0.0

        # Kalman filter state for each axis (complex solution, better performance)
        self.kf_pitch_rate = 0.0
        self.kf_roll_rate = 0.0
        self.kf_yaw_rate = 0.0

        # Kalman filter covariance
        self.kf_pitch_P = 1.0
        self.kf_roll_P = 1.0
        self.kf_yaw_P = 1.0

        # Tunable noise parameters
        self.kf_Q = 0.75  # process noise (how fast rate can change)
        self.kf_R = 4  # measurement noise (how noisy raw rate is)
        # tuning:
        # Q = 0.1, R = 10 → very smooth (laggy)
        # Q = 1.0, R = 3 → more responsive (hard, rumbling)
        # Q = 0.5, R = 5 → good general‑purpose   setting

        print("MotionPlatform received stop_stream_callback =", stop_stream_callback)

    # ---------- helpers ----------

    def _status(self, txt: str, color: str | None = None):
        self.status_callback(txt, color or "")

    # ---------------------------------------------------------
    # Serial connection
    # ---------------------------------------------------------
    def connect(self):
        self.serial = serial.Serial(self.port, self.baud, timeout=1, write_timeout=1)
        self.serial.reset_input_buffer()
        self.serial.reset_output_buffer()
        self._status(f"Motion platform connected on {self.port}")

    def disconnect(self):
        if self.serial and self.serial.is_open:
            self.serial.close()
            self._status("Motion platform disconnected")

    def set_enabled(self, state: bool):
        self.streaming_enabled = state
        self._status(f"Motion {'enabled' if state else 'disabled'}")

    # ---------------------------------------------------------
    # Called by XPlaneListener whenever new telemetry arrives
    # ---------------------------------------------------------

    def receive_telemetry(self, data: dict):
        corr = self._process_telemetry(data)
        packet = self._build_packet(corr)
        if self.streaming_enabled:
            self._send_packet(packet)

        # Emit corr back to SimMotion
        self.outgoing_signal.emit(corr)

    # ---------------------------------------------------------
    # Build Teensy packet
    # ---------------------------------------------------------
    def _build_packet(self, corr):
        packet = (
            f"X;"
            f"pitch={corr['pitch_converted']:.2f};"
            f"pitch_rate={corr['pitch_rate_converted']:.2f};"
            f"roll={corr['roll_converted']:.2f};"
            f"roll_rate={corr['roll_rate_converted']:.2f};"
            f"yaw={corr['yaw_converted']:.2f};"
            f"yaw_rate={corr['yaw_rate_converted']:.2f};"
            f"airspeed={corr['airspeed']:.2f}\n"
        )
        return packet

    # ---------------------------------------------------------
    # High‑frequency telemetry stream
    # ---------------------------------------------------------
    def _send_packet(self, packet: str):
        if not self.serial or not self.serial.is_open:
            return

        if not self.streaming_enabled:
            return

        try:
            with self.serial_lock:
                if not packet.endswith("\n"):
                    packet += "\n"
                self.serial.write(packet.encode("ascii"))
        except Exception as e:
            self._status(f"Serial write error: {e}", "red")

    # ---------------------------------------------------------
    # Command/response interface
    # ---------------------------------------------------------
    def send_receive(self, message: str, wait=False):
        if not self.serial or not self.serial.is_open:
            self._status("Serial port not open", "red")
            return None

        previous_streaming_status = self.streaming_enabled
        with self.serial_lock:
            self.streaming_enabled = False
            try:
                self.serial.reset_input_buffer()

                if not message.endswith("\n"):
                    message += "\n"
                self.serial.write(message.encode("ascii", errors="ignore"))

                if wait:
                    lines = []

                    while True:
                        # If no bytes available, wait briefly to catch last fragments
                        if self.serial.in_waiting == 0:
                            time.sleep(0.01)
                            if self.serial.in_waiting == 0:
                                break

                        raw = self.serial.readline()
                        if not raw:
                            break

                        line = raw.decode(errors="ignore").strip()
                        if line:
                            lines.append(line)

                    reply = "\n".join(lines) if lines else None
                else:
                    reply = None

            except Exception as e:
                self.serial.reset_output_buffer()
                self._status(f"Serial error: {e}", "red")
                reply = None

            finally:
                self.streaming_enabled = previous_streaming_status

        return reply

    def send_receive_OLD(self, message: str, wait=False):
        if not self.serial or not self.serial.is_open:
            self._status("Serial port not open", "red")
            return None

        previous_streaming_status = self.streaming_enabled
        with self.serial_lock:
            # pause streaming
            self.streaming_enabled = False
            try:
                # flush stale bytes
                self.serial.reset_input_buffer()

                # send command
                if not message.endswith("\n"):
                    message += "\n"
                self.serial.write(message.encode("ascii", errors="ignore"))

                # do serial stuff
                if wait:
                    lines = []
                    while True:
                        raw = self.serial.readline()
                        print(raw)
                        if not raw:
                            break
                        lines.append(raw.decode(errors="ignore").strip())
                    reply = "\n".join(lines) if lines else None
                else:
                    reply = None

            except Exception as e:
                self.serial.reset_output_buffer()
                self._status(f"Serial error: {e}", "red")
                reply = None

            finally:
                # resume streaming
                self.streaming_enabled = previous_streaming_status

        return reply

    def home(self):
        if not self.serial.is_open:
            self._status("Cannot home: not connected", "red")
            return False

        old_timeout = self.serial.timeout
        self.serial.timeout = 5

        try:
            self.send_receive("H", wait=True)

            # reset rates
            self.prev_time = None
            self.prev_pitch = 0
            self.prev_roll = 0
            self.prev_yaw = 0

            self._status("Homing complete")
            return True
        except Exception as e:
            self._status(f"Homing error: {e}", "red")
            return False
        finally:
            self.serial.timeout = old_timeout

    def send_axis_value(self, axis: str, value: float) -> None:
        """
        Send axis command like 'P;123.4'.
        """
        cmd = f"{axis};{value}"
        self.send_receive(cmd, wait=True)
        self._status(f"Sent: {cmd.strip()}")

    def request_status(self) -> str | None:
        """
        Send 'E;' to read offset/status from the platform.
        """
        return self.send_receive("E;", wait=True)


    # ---------------------------------------------------------
    # Process telemetry (scaling, rates, deminute)
    # ---------------------------------------------------------
    def _kalman_update(self, x, P, measurement, Q, R):
        """
        Simple 1D Kalman filter update.
        x = previous estimate
        P = previous covariance
        measurement = new noisy measurement
        Q = process noise
        R = measurement noise
        """
        # Prediction step
        x_pred = x
        P_pred = P + Q

        # Update step
        K = P_pred / (P_pred + R)
        x_new = x_pred + K * (measurement - x_pred)
        P_new = (1 - K) * P_pred

        return x_new, P_new

    def _process_telemetry(self, t: dict):
        now = time.time()

        pitch = t["pitch"]
        roll = t["roll"]
        yaw = t["yaw"]
        airspeed = t["airspeed"]

        # dt
        if self.prev_time is None:
            dt = 0.01
        else:
            dt = max(0.0001, now - self.prev_time)
        # clamp dt to avoid rate spikes after pauses
        dt = min(dt, 0.1)

        # rates
        # negative sign: convert sim convention to platform convention
        # TODO: Fix in new Teensy FW !!!!
        # pitch_rate = -(pitch - self.prev_pitch) / dt
        # roll_rate = -(roll - self.prev_roll) / dt
        # yaw_rate = -(yaw - self.prev_yaw) / dt

        # Apply exponential filter
        if self.filter == 'exp_filter':
            alpha = 0.4  # smoothing factor (tune 0.1–0.3 -> the higher, the smoother)
            pitch_rate_raw = -(pitch - self.prev_pitch) / dt
            roll_rate_raw = -(roll - self.prev_roll) / dt
            yaw_rate_raw = -(yaw - self.prev_yaw) / dt

            self.pitch_rate_smooth = (alpha * pitch_rate_raw + (1 - alpha) * self.pitch_rate_smooth)
            self.roll_rate_smooth = (alpha * roll_rate_raw + (1 - alpha) * self.roll_rate_smooth)
            self.yaw_rate_smooth = (alpha * yaw_rate_raw + (1 - alpha) * self.yaw_rate_smooth)
            pitch_rate = self.pitch_rate_smooth
            roll_rate = self.roll_rate_smooth
            yaw_rate = self.yaw_rate_smooth
        elif self.filter == 'kal_filter':
            pitch_rate_raw = -(pitch - self.prev_pitch) / dt
            roll_rate_raw = -(roll - self.prev_roll) / dt
            yaw_rate_raw = -(yaw - self.prev_yaw) / dt

            # Kalman filter update for pitch rate
            self.kf_pitch_rate, self.kf_pitch_P = self._kalman_update(
                self.kf_pitch_rate,
                self.kf_pitch_P,
                pitch_rate_raw,
                self.kf_Q,
                self.kf_R
            )

            # Kalman filter update for roll rate
            self.kf_roll_rate, self.kf_roll_P = self._kalman_update(
                self.kf_roll_rate,
                self.kf_roll_P,
                roll_rate_raw,
                self.kf_Q,
                self.kf_R
            )

            # Kalman filter update for yaw rate
            self.kf_yaw_rate, self.kf_yaw_P = self._kalman_update(
                self.kf_yaw_rate,
                self.kf_yaw_P,
                yaw_rate_raw,
                self.kf_Q,
                self.kf_R
            )

            pitch_rate = self.kf_pitch_rate
            roll_rate = self.kf_roll_rate
            yaw_rate = self.kf_yaw_rate

        # calc update rate
        update_rate = 1 / dt

        # save for next iteration
        self.prev_time = now
        self.prev_pitch = pitch
        self.prev_roll = roll
        self.prev_yaw = yaw

        # scaling
        pitch_conv = roll_conv = yaw_conv = 0.0
        pitch_rate_conv = roll_rate_conv = yaw_rate_conv = 0.0

        if self.factors_provider:
            try:
                fac = self.factors_provider()

                # --- VALIDATION CHECK ---
                if not isinstance(fac, dict):
                    self._status("Invalid factors provider output", "red")
                    return {
                        "pitch": pitch,
                        "roll": roll,
                        "yaw": yaw,
                        "airspeed": airspeed,
                        "pitch_converted": 0.0,
                        "pitch_rate_converted": 0.0,
                        "roll_converted": 0.0,
                        "roll_rate_converted": 0.0,
                        "yaw_converted": 0.0,
                        "yaw_rate_converted": 0.0,
                        "update_rate": update_rate,
                    }
                # --------------------------------

                if fac.get("pitch_enabled", False):

                    raw = pitch * fac["pitch_factor"]
                    pitch_conv, pitch_rate_conv = self.deminute(
                        raw, pitch_rate, airspeed, *fac["pitch_dem"]
                    )

                if fac.get("roll_enabled", False):
                    raw = roll * fac["roll_factor"]
                    roll_conv, roll_rate_conv = self.deminute(
                        raw, roll_rate, airspeed, *fac["roll_dem"]
                    )

                if fac.get("yaw_enabled", False):
                    raw = yaw * fac["yaw_factor"]
                    yaw_conv, yaw_rate_conv = self.deminute(
                        raw, yaw_rate, airspeed, *fac["yaw_dem"]
                    )

            except Exception as e:
                self._status(f"Scaling error: {e}", "red")
                self.stop_stream_callback()  # <--- STOP STREAM

        return {
            "pitch": pitch,
            "roll": roll,
            "yaw": yaw,
            "airspeed": airspeed,
            "pitch_converted": pitch_conv,
            "pitch_rate_converted": pitch_rate_conv,
            "roll_converted": roll_conv,
            "roll_rate_converted": roll_rate_conv,
            "yaw_converted": yaw_conv,
            "yaw_rate_converted": yaw_rate_conv,
            "update_rate": update_rate
        }



    @staticmethod
    def deminute(move, rate, speed, low_knt, low_percent, hi_knt, hi_percent):
        low_factor = low_percent / 100.0
        hi_factor = hi_percent / 100.0

        if speed <= low_knt:
            factor = low_factor
        elif speed >= hi_knt:
            factor = hi_factor
        else:
            t = (speed - low_knt) / (hi_knt - low_knt) if hi_knt != low_knt else 0.0
            factor = low_factor + t * (hi_factor - low_factor)

        return move * factor, rate * factor