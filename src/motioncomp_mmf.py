# motioncomp_mmf.py
# Provides MotionCompMMF class for writing pose data to OpenXR-MotionCompensation via MMF

import mmap
import struct
import threading

class MotionCompMMF:
    def __init__(self, mmf_name="YawVRGEFile"):
        self.mmf_name = "Local\\" + mmf_name
        self.struct_format = "<6f"
        self.struct_size = struct.calcsize(self.struct_format)

        self.lock = threading.Lock()

        self.mmf = mmap.mmap(
            -1,
            self.struct_size,
            tagname=self.mmf_name,
            access=mmap.ACCESS_WRITE
        )

    def write_pose(self, roll_deg, pitch_deg, yaw_deg, surge_m, sway_m, heave_m):
        data = struct.pack(
            self.struct_format,
            float(roll_deg),
            float(pitch_deg),
            float(yaw_deg),
            float(surge_m),
            float(sway_m),
            float(heave_m)
        )

        try:
            with self.lock:
                self.mmf.seek(0)
                self.mmf.write(data)

        except (BufferError, ValueError, OSError):
            # MMF was closed by MotionComp — stop writing
            print("MMF closed by MotionComp. Stopping writer.")
            self.mmf_valid = False

    def reopen(self):
        try:
            self.mmf = mmap.mmap(
                -1,
                self.struct_size,
                tagname=self.mmf_name,
                access=mmap.ACCESS_WRITE
            )
            self.mmf_valid = True
            print("MMF reopened after MotionComp activation")
        except Exception as e:
            # Still not available — MotionComp hasn't reopened it yet
            pass

    def close(self):
        self.mmf.close()

