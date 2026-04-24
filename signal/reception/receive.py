import asyncio
import matplotlib.pyplot as plt

from IMUSampleReceiver import IMUSampleReceiver


async def main() -> None:
    receiver = IMUSampleReceiver(listenDurationSeconds=30.0)
    await receiver.connect()
    motionData = await receiver.receive(exportedFileName="motionData.csv")
    await receiver.disconnect()
    plot(motionData)


def plot(motionData) -> None:
    if len(motionData["t"]) == 0:
        print("No samples available to plot")
        return

    tSeconds = [t / 1000000.0 for t in motionData["t"]]

    fig, axes = plt.subplots(3, 2, figsize=(12, 8))

    axes[0, 0].plot(tSeconds, motionData["a"]["x"])
    axes[0, 0].set_ylabel("x (m/s^2)")
    axes[0, 0].set_xlabel("time (s)")
    axes[0, 0].set_title("Linear Acceleration")

    axes[0, 1].plot(tSeconds, motionData["w"]["roll"])
    axes[0, 1].set_ylabel("roll (deg/s)")
    axes[0, 1].set_xlabel("time (s)")
    axes[0, 1].set_title("Angular Velocity")

    axes[1, 0].plot(tSeconds, motionData["a"]["y"])
    axes[1, 0].set_ylabel("y (m/s^2)")
    axes[1, 0].set_xlabel("time (s)")

    axes[1, 1].plot(tSeconds, motionData["w"]["pitch"])
    axes[1, 1].set_ylabel("pitch (deg/s)")
    axes[1, 1].set_xlabel("time (s)")

    axes[2, 0].plot(tSeconds, motionData["a"]["z"])
    axes[2, 0].set_ylabel("z (m/s^2)")
    axes[2, 0].set_xlabel("time (s)")

    axes[2, 1].plot(tSeconds, motionData["w"]["yaw"])
    axes[2, 1].set_ylabel("yaw (deg/s)")
    axes[2, 1].set_xlabel("time (s)")

    fig.suptitle("IMU Recorded Data")
    plt.tight_layout()
    fig.savefig("imuPlot.png")
    plt.show()
    print("Plot saved as imuPlot.png")


asyncio.run(main())
