import matplotlib.pyplot as plt
from array import array


class IMUSampleTimeSeriesPlotter:
    def plot(self, motionData: dict[str, array | dict[str, array]]) -> None:
        if len(motionData["seq"]) == 0:
            print("No samples available to plot")
            return

        fig, axes = plt.subplots(3, 2, figsize=(12, 8))

        axes[0, 0].plot(motionData["seq"], motionData["a"]["x"])  # type: ignore
        axes[0, 0].set_ylabel("x (m/s^2)")
        axes[0, 0].set_xlabel("Sequence number")
        axes[0, 0].set_title("Linear Acceleration")

        axes[0, 1].plot(motionData["seq"], motionData["w"]["roll"])  # type: ignore
        axes[0, 1].set_ylabel("roll (deg/s)")
        axes[0, 1].set_xlabel("Sequence number")
        axes[0, 1].set_title("Angular Velocity")

        axes[1, 0].plot(motionData["seq"], motionData["a"]["y"])  # type: ignore
        axes[1, 0].set_ylabel("y (m/s^2)")
        axes[1, 0].set_xlabel("Sequence number")

        axes[1, 1].plot(motionData["seq"], motionData["w"]["pitch"])  # type: ignore
        axes[1, 1].set_ylabel("pitch (deg/s)")
        axes[1, 1].set_xlabel("Sequence number")

        axes[2, 0].plot(motionData["seq"], motionData["a"]["z"])  # type: ignore
        axes[2, 0].set_ylabel("z (m/s^2)")
        axes[2, 0].set_xlabel("Sequence number")

        axes[2, 1].plot(motionData["seq"], motionData["w"]["yaw"])  # type: ignore
        axes[2, 1].set_ylabel("yaw (deg/s)")
        axes[2, 1].set_xlabel("Sequence number")

        fig.suptitle("IMU Recorded Data")
        plt.tight_layout()
        plt.show()
