import time
import serial
from PySide6.QtCore import QObject, Signal
import threading

# todo check, where stop_stream_callback makes sense
# todo pack telemetry in a class to prepare json


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

        # smoothin filter
        self.pitch_rate_smooth = 0.0
        self.roll_rate_smooth = 0.0
        self.yaw_rate_smooth = 0.0

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
                        if not raw:
                            break
                        lines.append(raw.decode(errors="ignore").strip())

                    reply = "\n".join(lines) if lines else None
                else:
                    # raw = self.serial.read_all()
                    reply = None

                # prepare response
                # if wait:
                #     if not lines:
                #         # timeout
                #         self._status("Motion Platform Timeout", "red")
                #         reply = None
                #     else:
                #         # reply = raw.decode(errors="ignore").strip()
                #         reply = lines
                # else:
                #     reply = None

                # if wait:
                #     raw = self.serial.readline()
                #     if not raw:
                #
                #         reply = None
                #     else:
                #         reply = raw.decode(errors="ignore").strip()
                # else:
                #     reply = None

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
            # TODO implement teensy response
            # resp = self.send_receive("Z;180", wait=True)
            # if not resp or not resp.startswith("OK"):
            #     self._status("Homing failed: Z command", "red")
            #     return False

            # self.send_receive("Z;180", wait=True)
            # self.send_receive("P;0", wait=True)
            # self.send_receive("R;0", wait=True)
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

        # Apply filter
        alpha = 0.2  # smoothing factor (tune 0.1–0.3)
        pitch_rate_raw = -(pitch - self.prev_pitch) / dt
        roll_rate_raw = -(roll - self.prev_roll) / dt
        yaw_rate_raw = -(yaw - self.prev_yaw) / dt

        self.pitch_rate_smooth = (alpha * pitch_rate_raw + (1 - alpha) * self.pitch_rate_smooth)
        self.roll_rate_smooth = (alpha * roll_rate_raw + (1 - alpha) * self.roll_rate_smooth)
        self.yaw_rate_smooth = (alpha * yaw_rate_raw + (1 - alpha) * self.yaw_rate_smooth)
        pitch_rate = self.pitch_rate_smooth
        roll_rate = self.roll_rate_smooth
        yaw_rate = self.yaw_rate_smooth

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