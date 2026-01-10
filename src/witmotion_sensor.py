# witmotion_sensor.py
# Final version for WitMotion WT901 IMU + MMF output for OpenXR-MotionCompensation

import time
import threading
from src.motioncomp_mmf import MotionCompMMF
from witmotion import IMU


class WitMotionSensor:
    """
    Reads IMU data from WitMotion WT901 sensor and outputs:
    - roll, pitch, yaw (degrees, offset-corrected)
    - surge, sway, heave (meters, integrated)
    Writes everything into a Memory-Mapped File for OpenXR-MotionCompensation.
    """

    def __init__(self, port, baudrate=115200,
                 stop_event=None,
                 status_callback=None,
                 imu_callback=None,
                 mmf_name="YawVRGEFile"):  # YawVRGEFile old OxrMcInput

        # external callbacks
        self.stop_event = stop_event
        self.status_callback = status_callback
        self.imu_callback = imu_callback

        # IMU device (opens automatically)
        self.imu = IMU(port, baudrate)

        # MMF writer for MotionCompensation
        self.mmf = MotionCompMMF(mmf_name)
        self.mmf_valid = True

        # integrators for surge/sway/heave
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        # orientation offsets (set during homing)
        self.roll_offset = 0.0
        self.pitch_offset = 0.0
        self.yaw_offset = 0.0

        self.last_time = time.time()

        # connection state
        self.is_open = False
        self.thread = None

    # ---------------------------------------------------------
    # Connection handling
    # ---------------------------------------------------------

    def connect(self):
        """IMU opens automatically in constructor; just mark as open."""
        try:
            self.is_open = True
            if self.status_callback:
                self.status_callback("WitMotion IMU connected")
        except Exception as e:
            if self.status_callback:
                self.status_callback(f"IMU connect error: {e}", "red")
            raise

    def disconnect(self):
        """Close IMU and MMF."""
        self.is_open = False
        try:
            if hasattr(self.imu, "close"):
                self.imu.close()
        except:
            pass

        try:
            self.mmf.close()
        except:
            pass

    # ---------------------------------------------------------
    # Threaded listener
    # ---------------------------------------------------------

    def start_listening(self):
        """Start IMU reading thread."""
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def stop_listening(self):
        """Stop IMU thread."""
        if self.thread and self.thread.is_alive():
            if self.stop_event:
                self.stop_event.set()
            self.thread.join(timeout=1.0)

    def _loop(self):
        """Main IMU loop."""
        while not self.stop_event.is_set():
            self.update()
            time.sleep(0.01)  # 100 Hz

    # ---------------------------------------------------------
    # Offset capture (called during homing)
    # ---------------------------------------------------------

    def capture_offsets(self, real_yaw=0):
        """Capture IMU orientation as zero reference."""
        roll, pitch, yaw = self.imu.get_angle()
        self.roll_offset = roll
        self.pitch_offset = pitch
        self.yaw_offset = yaw - real_yaw

        if self.status_callback:
            self.status_callback("IMU offsets captured")

    def reset_translation(self):
        """Reset integrated translation and velocity to zero."""
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

    # ---------------------------------------------------------
    # Main update loop
    # ---------------------------------------------------------

    def update(self):
        """Read IMU, apply offsets, integrate translation, write to MMF."""
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        # --- 1) Read IMU orientation ---
        roll, pitch, yaw = self.imu.get_angle()

        # If IMU has not yet produced valid data, skip this cycle
        if roll is None or pitch is None or yaw is None:
            return

        # ignore uninitialized IMU values
        if roll == 0.0 and pitch == 0.0 and yaw == 0.0:
            return

        # apply offsets
        roll_adj = roll - self.roll_offset
        pitch_adj = pitch - self.pitch_offset
        yaw_adj = yaw - self.yaw_offset

        # --- 2) Read acceleration ---
        ax, ay, az = self.imu.get_acceleration()

        # remove gravity from Z axis
        az -= 9.80665

        # --- 3) Integrate acceleration → velocity → position ---
        self.vx += ax * dt
        self.vy += ay * dt
        self.vz += az * dt

        self.x += self.vx * dt
        self.y += self.vy * dt
        self.z += self.vz * dt

        # --- 4) MMF handling (robust activation/deactivation cycle) ---

        # If MMF is invalid, try to reopen it (MotionComp reactivated)
        if not self.mmf_valid:
            try:
                self.mmf.reopen()  # your MotionCompMMF.reopen() method
            except Exception:
                return  # still closed, skip this cycle

            # If reopen failed, skip this cycle
            if not self.mmf_valid:
                return

        # MMF is valid → try writing
        try:
            self.mmf.write_pose(
                yaw_adj, roll_adj, pitch_adj,
                0.0, 0.0, 0.0
            )
        except Exception:
            # MMF was closed by MotionComp (CTRL+INS deactivation)
            self.mmf_valid = False
            if self.status_callback:
                self.status_callback("MMF closed by MotionComp", "red")
            return

        # --- 5) GUI callback ---
        if self.imu_callback:
            self.imu_callback(roll_adj, pitch_adj, yaw_adj)

