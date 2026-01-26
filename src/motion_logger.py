import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Do NOT import SciPy here — it blocks SimMotion startup
correlate = None


class MotionLogAnalyzer:
    def __init__(self, csv_path="SimMotion.csv"):
        self.csv_path = csv_path
        self.df = None
        self.inc = None
        self.out = None
        self.imu = None

    # ---------------------------------------------------------
    # Load and preprocess CSV
    # ---------------------------------------------------------
    def load(self):
        self.df = pd.read_csv(
            self.csv_path,
            sep=";",
            names=["timestamp", "source", "pitch", "roll", "yaw"],
            header=None
        )

        # Normalize source names
        self.df["source"] = self.df["source"].str.strip().str.lower()

        # Relative time
        self.df["time"] = self.df["timestamp"] - self.df["timestamp"].min()

        # Split
        self.inc = self.df[self.df["source"] == "incoming"].copy()
        self.out = self.df[self.df["source"] == "outgoing"].copy()
        self.imu = self.df[self.df["source"] == "imu"].copy()

        # Normalize signals
        def normalize(series):
            return series / series.iloc[0]

        for src in [self.inc, self.imu]:
            src["pitch_n"] = normalize(src["pitch"])
            src["roll_n"]  = normalize(src["roll"])
            src["yaw_n"]   = normalize(src["yaw"])

    # ---------------------------------------------------------
    # Cross-correlation latency estimation
    # ---------------------------------------------------------
    def estimate_latency(self, t1, v1, t2, v2):
        global correlate
        if correlate is None:
            from scipy.signal import correlate  # lazy import
        t_min = max(t1.min(), t2.min())
        t_max = min(t1.max(), t2.max())
        t_common = np.linspace(t_min, t_max, 2000)

        v1i = np.interp(t_common, t1, v1)
        v2i = np.interp(t_common, t2, v2)

        corr = correlate(v2i - v2i.mean(), v1i - v1i.mean(), mode="full")
        lag_idx = corr.argmax() - (len(v1i) - 1)
        dt = t_common[1] - t_common[0]
        return lag_idx * dt

    # ---------------------------------------------------------
    # Compute and print latency
    # ---------------------------------------------------------
    def compute_latency(self):
        lat_pitch = self.estimate_latency(
            self.inc["time"], self.inc["pitch_n"],
            self.imu["time"], self.imu["pitch_n"]
        )
        lat_roll = self.estimate_latency(
            self.inc["time"], self.inc["roll_n"],
            self.imu["time"], self.imu["roll_n"]
        )
        lat_yaw = self.estimate_latency(
            self.inc["time"], self.inc["yaw_n"],
            self.imu["time"], self.imu["yaw_n"]
        )

        print("\nEstimated latency (IMU behind incoming):")
        print(f"Pitch: {lat_pitch*1000:.1f} ms")
        print(f"Roll:  {lat_roll*1000:.1f} ms")
        print(f"Yaw:   {lat_yaw*1000:.1f} ms")

        return lat_pitch, lat_roll, lat_yaw

    # ---------------------------------------------------------
    # Plot incoming vs IMU
    # ---------------------------------------------------------
    def plot(self, lat_pitch, lat_roll, lat_yaw):
        fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

        # Pitch
        axes[0].plot(self.inc["time"], self.inc["pitch_n"], label="incoming", linewidth=2)
        axes[0].plot(self.imu["time"], self.imu["pitch_n"], label="imu", linewidth=2)
        axes[0].set_ylabel("Normalized Pitch")
        axes[0].set_title(f"Pitch vs Time  —  Latency: {lat_pitch * 1000:.1f} ms")
        axes[0].legend()

        # Roll
        axes[1].plot(self.inc["time"], self.inc["roll_n"], label="incoming", linewidth=2)
        axes[1].plot(self.imu["time"], self.imu["roll_n"], label="imu", linewidth=2)
        axes[1].set_ylabel("Normalized Roll")
        axes[1].set_title(f"Roll vs Time  —  Latency: {lat_roll * 1000:.1f} ms")
        axes[1].legend()

        # Yaw
        axes[2].plot(self.inc["time"], self.inc["yaw_n"], label="incoming", linewidth=2)
        axes[2].plot(self.imu["time"], self.imu["yaw_n"], label="imu", linewidth=2)
        axes[2].set_ylabel("Normalized Yaw")
        axes[2].set_xlabel("Time (s)")
        axes[2].set_title(f"Yaw vs Time  —  Latency: {lat_yaw * 1000:.1f} ms")
        axes[2].legend()

        plt.tight_layout()
        plt.show()

    def plot_OLD(self):
        fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

        # Pitch
        axes[0].plot(self.inc["time"], self.inc["pitch_n"], label="incoming", linewidth=2)
        axes[0].plot(self.imu["time"], self.imu["pitch_n"], label="imu", linewidth=2)
        axes[0].set_ylabel("Normalized Pitch")
        axes[0].set_title("Pitch vs Time")
        axes[0].legend()

        # Roll
        axes[1].plot(self.inc["time"], self.inc["roll_n"], label="incoming", linewidth=2)
        axes[1].plot(self.imu["time"], self.imu["roll_n"], label="imu", linewidth=2)
        axes[1].set_ylabel("Normalized Roll")
        axes[1].set_title("Roll vs Time")
        axes[1].legend()

        # Yaw
        axes[2].plot(self.inc["time"], self.inc["yaw_n"], label="incoming", linewidth=2)
        axes[2].plot(self.imu["time"], self.imu["yaw_n"], label="imu", linewidth=2)
        axes[2].set_ylabel("Normalized Yaw")
        axes[2].set_xlabel("Time (s)")
        axes[2].set_title("Yaw vs Time")
        axes[2].legend()

        plt.tight_layout()
        plt.show()

    # ---------------------------------------------------------
    # Convenience method: load → compute → plot
    # ---------------------------------------------------------
    def analyze(self):
        self.load()
        lat_pitch, lat_roll, lat_yaw = self.compute_latency()
        self.plot(lat_pitch, lat_roll, lat_yaw)