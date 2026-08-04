import matplotlib.pyplot as plt
from array import array
import numpy as np


class IMUSampleTimeSeriesPlotter:
    def plot(self, motionData: dict[str, array | dict[str, array]]) -> None:
        if len(motionData["seq"]) == 0:
            print("No samples available to plot")
            return

        # Convert legacy dict-of-arrays to numpy arrays and delegate
        seq = np.array(motionData["seq"])
        a = np.column_stack(
            (motionData["a"]["x"], motionData["a"]["y"], motionData["a"]["z"])
        )
        w = np.column_stack(
            (motionData["w"]["roll"], motionData["w"]["pitch"], motionData["w"]["yaw"])
        )
        self.plotNew(seq, a, w)

    def plotNew(self, seq: np.ndarray, a: np.ndarray, w: np.ndarray) -> None:
        if seq.size == 0:
            print("No samples available to plot")
            return

        fig, axes = plt.subplots(3, 2, figsize=(12, 8))

        lowerBoundAccel = min(np.min(a[:, 0]), np.min(a[:, 1]), np.min(a[:, 2]))
        upperBoundAccel = max(np.max(a[:, 0]), np.max(a[:, 1]), np.max(a[:, 2]))

        lowerBoundGyro = min(np.min(w[:, 0]), np.min(w[:, 1]), np.min(w[:, 2]))
        upperBoundGyro = max(np.max(w[:, 0]), np.max(w[:, 1]), np.max(w[:, 2]))

        axes[0, 0].plot(seq, a[:, 0])
        axes[0, 0].set_ylabel("x (m/s^2)")
        axes[0, 0].set_xlabel("Sequence number")
        axes[0, 0].set_title("Linear Acceleration")
        axes[0, 0].set_ylim(lowerBoundAccel, upperBoundAccel)

        axes[0, 1].plot(seq, w[:, 0])
        axes[0, 1].set_ylabel("roll (deg/s)")
        axes[0, 1].set_xlabel("Sequence number")
        axes[0, 1].set_title("Angular Velocity")
        axes[0, 1].set_ylim(lowerBoundGyro, upperBoundGyro)

        axes[1, 0].plot(seq, a[:, 1])
        axes[1, 0].set_ylabel("y (m/s^2)")
        axes[1, 0].set_xlabel("Sequence number")
        axes[1, 0].set_ylim(lowerBoundAccel, upperBoundAccel)

        axes[1, 1].plot(seq, w[:, 1])
        axes[1, 1].set_ylabel("pitch (deg/s)")
        axes[1, 1].set_xlabel("Sequence number")
        axes[1, 1].set_ylim(lowerBoundGyro, upperBoundGyro)

        axes[2, 0].plot(seq, a[:, 2])
        axes[2, 0].set_ylabel("z (m/s^2)")
        axes[2, 0].set_xlabel("Sequence number")
        axes[2, 0].set_ylim(lowerBoundAccel, upperBoundAccel)

        axes[2, 1].plot(seq, w[:, 2])
        axes[2, 1].set_ylabel("yaw (deg/s)")
        axes[2, 1].set_xlabel("Sequence number")
        axes[2, 1].set_ylim(lowerBoundGyro, upperBoundGyro)

        fig.suptitle("IMU Recorded Data")
        plt.tight_layout()
        plt.show()
