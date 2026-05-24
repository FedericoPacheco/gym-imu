import matplotlib.pyplot as plt
from array import array


class IMUSampleTimeSeriesPlotter:
    def plot(self, motionData: dict[str, array | dict[str, array]]) -> None:
        if len(motionData["seq"]) == 0:
            print("No samples available to plot")
            return

        fig, axes = plt.subplots(3, 2, figsize=(12, 8))

        lowerBoundAccel = min(min(motionData["a"]["x"]), min(motionData["a"]["y"]), min(motionData["a"]["z"]))  # type: ignore
        upperBoundAccel = max(max(motionData["a"]["x"]), max(motionData["a"]["y"]), max(motionData["a"]["z"]))  # type: ignore

        lowerBoundGyro = min(min(motionData["w"]["roll"]), min(motionData["w"]["pitch"]), min(motionData["w"]["yaw"]))  # type: ignore
        upperBoundGyro = max(max(motionData["w"]["roll"]), max(motionData["w"]["pitch"]), max(motionData["w"]["yaw"]))  # type: ignore

        axes[0, 0].plot(motionData["seq"], motionData["a"]["x"])  # type: ignore
        axes[0, 0].set_ylabel("x (m/s^2)")
        axes[0, 0].set_xlabel("Sequence number")
        axes[0, 0].set_title("Linear Acceleration")
        axes[0, 0].set_ylim(lowerBoundAccel, upperBoundAccel)

        axes[0, 1].plot(motionData["seq"], motionData["w"]["roll"])  # type: ignore
        axes[0, 1].set_ylabel("roll (deg/s)")
        axes[0, 1].set_xlabel("Sequence number")
        axes[0, 1].set_title("Angular Velocity")
        axes[0, 1].set_ylim(lowerBoundGyro, upperBoundGyro)

        axes[1, 0].plot(motionData["seq"], motionData["a"]["y"])  # type: ignore
        axes[1, 0].set_ylabel("y (m/s^2)")
        axes[1, 0].set_xlabel("Sequence number")
        axes[1, 0].set_ylim(lowerBoundAccel, upperBoundAccel)

        axes[1, 1].plot(motionData["seq"], motionData["w"]["pitch"])  # type: ignore
        axes[1, 1].set_ylabel("pitch (deg/s)")
        axes[1, 1].set_xlabel("Sequence number")
        axes[1, 1].set_ylim(lowerBoundGyro, upperBoundGyro)

        axes[2, 0].plot(motionData["seq"], motionData["a"]["z"])  # type: ignore
        axes[2, 0].set_ylabel("z (m/s^2)")
        axes[2, 0].set_xlabel("Sequence number")
        axes[2, 0].set_ylim(lowerBoundAccel, upperBoundAccel)

        axes[2, 1].plot(motionData["seq"], motionData["w"]["yaw"])  # type: ignore
        axes[2, 1].set_ylabel("yaw (deg/s)")
        axes[2, 1].set_xlabel("Sequence number")
        axes[2, 1].set_ylim(lowerBoundGyro, upperBoundGyro)

        fig.suptitle("IMU Recorded Data")
        plt.tight_layout()
        plt.show()
