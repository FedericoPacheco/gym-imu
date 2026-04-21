from array import array
import struct
from typing import Any

import matplotlib.pyplot as plt

# Device sends batches of IMU samples within each BLE notification as raw bytes in little-endian format.
# Refer to IMUSensorPort.hpp for the exact C++ struct.
IMU_SAMPLE_STRUCT = struct.Struct("<ffffffq")  # 6 floats, 1 int64_t timestamp
IMU_SAMPLE_SIZE_BYTES = IMU_SAMPLE_STRUCT.size

LOGGING_PERIOD_IN_NOTIFICATIONS = 5
notificationCount = 0

EXPORTED_DATA_FILE_NAME = "motionData"

# Prefer C-typed arrays over Python lists/dictionaries for better performance and lower memory overhead
ax = array("f")
ay = array("f")
az = array("f")
roll = array("f")
pitch = array("f")
yaw = array("f")
tMicroseconds = array("q")


def onImuNotification(characteristic: Any, data: bytearray) -> None:
    global notificationCount

    payload = memoryview(data)
    payloadSize = payload.nbytes

    if payloadSize == 0:
        return

    if payloadSize % IMU_SAMPLE_STRUCT.size != 0:
        raise ValueError("Invalid payload size")

    for (
        sampleAx,
        sampleAy,
        sampleAz,
        sampleRoll,
        samplePitch,
        sampleYaw,
        sampleT,
    ) in IMU_SAMPLE_STRUCT.iter_unpack(payload):
        ax.append(sampleAx)
        ay.append(sampleAy)
        az.append(sampleAz)
        roll.append(sampleRoll)
        pitch.append(samplePitch)
        yaw.append(sampleYaw)
        tMicroseconds.append(sampleT)

    notificationCount += 1
    if notificationCount >= LOGGING_PERIOD_IN_NOTIFICATIONS:
        print(
            f"Sample from batch: ax: {ax[-1]}, ay: {ay[-1]}, az: {az[-1]}, "
            f"roll: {roll[-1]}, pitch: {pitch[-1]}, yaw: {yaw[-1]}, "
            f"timestamp: {tMicroseconds[-1]}"
        )
        notificationCount = 0


def plot() -> None:
    if len(tMicroseconds) == 0:
        print("No samples available to plot")
        return

    tSeconds = [t / 1000000.0 for t in tMicroseconds]

    fig, axes = plt.subplots(3, 2, figsize=(12, 8))

    axes[0, 0].plot(tSeconds, ax)
    axes[0, 0].set_ylabel("x (m/s^2)")
    axes[0, 0].set_xlabel("time (s)")
    axes[0, 0].set_title("Linear Acceleration")

    axes[0, 1].plot(tSeconds, roll)
    axes[0, 1].set_ylabel("roll (deg/s)")
    axes[0, 1].set_xlabel("time (s)")
    axes[0, 1].set_title("Angular Velocity")

    axes[1, 0].plot(tSeconds, ay)
    axes[1, 0].set_ylabel("y (m/s^2)")
    axes[1, 0].set_xlabel("time (s)")

    axes[1, 1].plot(tSeconds, pitch)
    axes[1, 1].set_ylabel("pitch (deg/s)")
    axes[1, 1].set_xlabel("time (s)")

    axes[2, 0].plot(tSeconds, az)
    axes[2, 0].set_ylabel("z (m/s^2)")
    axes[2, 0].set_xlabel("time (s)")

    axes[2, 1].plot(tSeconds, yaw)
    axes[2, 1].set_ylabel("yaw (deg/s)")
    axes[2, 1].set_xlabel("time (s)")

    fig.suptitle("IMU Recorded Data")
    plt.tight_layout()
    fig.savefig("imuPlot.png")
    plt.show()
    print("Plot saved as imuPlot.png")


def exportMotionData() -> None:
    with open(f"{EXPORTED_DATA_FILE_NAME}.csv", "w", encoding="utf-8") as f:
        f.write("t_us,ax,ay,az,roll,pitch,yaw\n")
        for (
            sampleT,
            sampleAx,
            sampleAy,
            sampleAz,
            sampleRoll,
            samplePitch,
            sampleYaw,
        ) in zip(
            tMicroseconds,
            ax,
            ay,
            az,
            roll,
            pitch,
            yaw,
        ):
            f.write(
                f"{sampleT},{sampleAx},{sampleAy},{sampleAz},{sampleRoll},{samplePitch},{sampleYaw}\n"
            )
    print(f"Motion data saved to {EXPORTED_DATA_FILE_NAME}.csv")


if __name__ == "__main__":
    # Local decode test without BLE: build one fake batch notification
    fakePayload = bytearray(
        IMU_SAMPLE_STRUCT.pack(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1 * 1000000)
        + IMU_SAMPLE_STRUCT.pack(1.0, 2.0, 3.0, 45.0, 90.0, 180.0, 2 * 1000000)
        + IMU_SAMPLE_STRUCT.pack(2.0, 4.0, 6.0, 90.0, 180.0, 360.0, 3 * 1000000)
    )
    onImuNotification(None, fakePayload)
    exportMotionData()
    plot()
