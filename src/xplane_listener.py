# src/xplane_listener.py
# -*- coding: utf-8 -*-

import socket
import struct
import threading
import time
from PySide6.QtCore import QObject, Signal

class XPlaneListener(QObject):
    incoming_signal = Signal(dict)
    def __init__(
        self,
        ip: str,
        port: int,
        stop_event,
        status_callback=None,
        telemetry_callback=None,
    ):
        super().__init__()
        self.ip = ip
        self.port = port
        self.stop_event = stop_event

        self.status_callback = status_callback or (lambda txt, col=None: None)
        # self.status_callback = status_callback or (lambda *a, **k: None)
        self.telemetry_callback = telemetry_callback  # for Teensy


        self.sock: socket.socket | None = None
        self.thread: threading.Thread | None = None

        self.packet_counter = 0
        self.rate_timer = time.time()

        # debug data speed
        self.debug = False
        self.temp_rate = []


    # ---------- helpers ----------

    def _status(self, txt: str, color: str | None = None):
        self.status_callback(txt, color or "")

    # ---------- lifecycle ----------
    def start(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.ip, self.port))
        self.sock.settimeout(0.5)
        self._status(f"UDP socket bound to {self.ip}:{self.port}")

        self.thread = threading.Thread(target=self._listen_loop, daemon=True)
        self.thread.start()

    def stop(self, timeout=1.0):
        # Signal thread to stop
        self.stop_event.set()

        # Wait for thread to finish
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=timeout)

        # Now it is safe to close the socket
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass

        self.thread = None
        self.sock = None

    def stop_OLD(self, timeout=1.0):
        if self.sock:
            try:
                self.sock.settimeout(0.1)
            except Exception:
                pass
            self.sock.close()

        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=timeout)
        self.thread = None

    # ---------- main loop ----------

    def _listen_loop(self):
        try:
            while not self.stop_event.is_set():
                try:
                    data, _ = self.sock.recvfrom(2048)

                except socket.timeout:
                    continue

                except OSError as e:
                    # Windows UDP quirk: ignore "connection forcibly closed"
                    if e.errno == 10054:
                        continue

                    self._status(f"ERROR in listen_xplane: {e}", "red")
                    print(e)
                    break

                self.packet_counter += 1
                parsed = self._parse_data(data)
                if not parsed:
                    continue

                self._process_telemetry(parsed)
                time.sleep(0.0005)

        except Exception as e:
            self._status(f"ERROR (outer) in listen_xplane: {e}", "red")
            print(e)


    # ---------- parsing ----------

    def _parse_data(self, data: bytes):
        if not data.startswith(b"DATA*"):
            return None

        pitch = roll = yaw = None
        airspeed = None
        frame_rate = None

        for i in range(5, len(data), 36):
            group = data[i : i + 36]
            if len(group) < 36:
                continue
            try:
                index = struct.unpack("<i", group[:4])[0]
                values = struct.unpack("<8f", group[4:])
            except struct.error:
                continue

            if index == 0:  # frame rate
                frame_rate = values[0]
            elif index == 17:  # pitch, roll, yaw
                pitch, roll, yaw = values[0], values[1], values[2]
            elif index == 3:  # airspeed
                airspeed = values[0]

        if None not in (pitch, roll, yaw, airspeed):
            return {
                "pitch": pitch,
                "roll": roll,
                "yaw": yaw,
                "airspeed": airspeed,
                "frame_rate": frame_rate,
            }
        return None

    # ---------- prepare callback ----------
    def _process_telemetry(self, result: dict):
        if not self.telemetry_callback:
            return

        now = time.time()

        # --- Measure incoming telemetry rate ---
        if self.debug:

            self.temp_rate.append(now)
            if len(self.temp_rate) >= 100:
                # Compute time differences between consecutive samples
                diffs = [
                    self.temp_rate[i] - self.temp_rate[i - 1]
                    for i in range(1, len(self.temp_rate))
                ]
                avg_dt = sum(diffs) / len(diffs)
                hz = 1.0 / avg_dt if avg_dt > 0 else 0
                print(f"[DEBUG] Sim update rate: {hz:.1f} Hz (avg dt = {avg_dt * 1000:.2f} ms)")
                # Reset buffer
                self.temp_rate.clear()

        # --- build data package ---
        data = {
            "timestamp": now,
            "source": "incoming",
            "pitch": result["pitch"],
            "roll": result["roll"],
            "yaw": result["yaw"],
            "airspeed": result["airspeed"],
            "frame_rate": result["frame_rate"],
        }

        # 1) Send to motion platform (callback)
        if self.telemetry_callback:
            self.telemetry_callback(data)

        # 2) Emit to GUI
        self.incoming_signal.emit(data)



