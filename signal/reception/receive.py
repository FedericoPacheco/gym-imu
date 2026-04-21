from array import array
import asyncio
import contextlib
import struct
from typing import Any

import matplotlib.pyplot as plt
from bleak import BleakClient, BleakScanner

# ---------------------------------------------------------------------------------------------------------

# DEVICE CONNECTION PARAMETERS
# Extracted from BLE.hpp.
DEVICE_NAME = "Gym-IMU"
IMU_SERVICE_UUID = "12345678-1234-5678-1234-56789abcdef0"
IMU_CHARACTERISTIC_UUID = "c21c340b-f231-4da2-ab20-716f9ed67c3a"

# DATA FORMAT
# Device sends batches of IMU samples within each BLE notification as raw bytes in little-endian format.
# Refer to IMUSensorPort.hpp for the exact C++ struct.
IMU_SAMPLE_STRUCT = struct.Struct("<ffffffq")  # 6 floats, 1 int64_t timestamp
IMU_SAMPLE_SIZE_BYTES = IMU_SAMPLE_STRUCT.size

# TIMING PARAMETERS
DEVICE_SCAN_TIMEOUT_SECONDS = 60.0
FIRST_SAMPLE_TIMEOUT = 30.0
LISTEN_DURATION_SECONDS = 30.0

# MISCELLANEOUS
LOGGING_PERIOD_IN_NOTIFICATIONS = 10
EXPORTED_DATA_FILE_NAME = "motionData"

# --------------------------------------------------------------------------------------------------------

# NOTIFICATIONS
notificationCount = 0
totalNotificationCount = 0
receivedFirstSampleEvent: asyncio.Event | None = None

# MOTION DATA
# Prefer C-typed arrays over Python lists/dictionaries for better performance and lower memory overhead
ax = array("f")
ay = array("f")
az = array("f")
roll = array("f")
pitch = array("f")
yaw = array("f")
tMicroseconds = array("q")

# ---------------------------------------------------------------------------------------------------------


async def main() -> None:
    await listenForSamples()
    plot()
    exportMotionData()


async def listenForSamples() -> None:
    global receivedFirstSampleEvent

    target = await findTargetDevice()

    print("Connecting to device...")
    async with BleakClient(target) as client:
        if not client.is_connected:
            raise RuntimeError("Connection failed")
        print("Connected")

        services = client.services
        service = services.get_service(IMU_SERVICE_UUID)
        if service is None:
            raise RuntimeError(f"Required service not found: {IMU_SERVICE_UUID}")

        characteristic = services.get_characteristic(IMU_CHARACTERISTIC_UUID)
        if characteristic is None:
            raise RuntimeError(
                f"Required characteristic not found: {IMU_CHARACTERISTIC_UUID}"
            )

        hasNotifyProperty = "notify" in characteristic.properties
        if not hasNotifyProperty:
            raise RuntimeError("Characteristic does not support notifications")

        subscribed = False
        try:
            # Create the event before subscribing to avoid missing the very first notification
            receivedFirstSampleEvent = asyncio.Event()
            await client.start_notify(IMU_CHARACTERISTIC_UUID, onImuNotification)
            subscribed = True
            print("Subscribed to IMU notifications")
            print(
                "Press the wearable button now to start transmission. "
                "Waiting for first sample..."
            )
            try:
                await asyncio.wait_for(
                    receivedFirstSampleEvent.wait(), timeout=FIRST_SAMPLE_TIMEOUT
                )
            except asyncio.TimeoutError as exc:
                raise RuntimeError(
                    "No samples received after subscribing. "
                    "Press the wearable button and retry."
                ) from exc

            print(f"Receiving samples for {LISTEN_DURATION_SECONDS:.0f} seconds...")
            await asyncio.sleep(LISTEN_DURATION_SECONDS)
            print("Capture window completed")
        finally:
            receivedFirstSampleEvent = None
            if subscribed:
                with contextlib.suppress(Exception):
                    await client.stop_notify(IMU_CHARACTERISTIC_UUID)
                print("Unsubscribed from IMU notifications")


async def findTargetDevice() -> Any:
    print(
        f"Scanning for BLE device '{DEVICE_NAME}' for up to {DEVICE_SCAN_TIMEOUT_SECONDS:.0f}s..."
    )

    target = await BleakScanner.find_device_by_filter(
        lambda _, ad: matchesDevice(ad.local_name, ad.service_uuids),
        timeout=DEVICE_SCAN_TIMEOUT_SECONDS,
    )

    if target is None:
        raise RuntimeError(
            "BLE device not found. Check that it is powered and advertising."
        )

    print(f"Found device: name={target.name}, address={target.address}")
    return target


def matchesDevice(
    advertisementDataName: str | None, serviceUuids: list[str] | None
) -> bool:
    hasMatchingName = advertisementDataName == DEVICE_NAME
    hasMatchingService = False
    if serviceUuids:
        hasMatchingService = IMU_SERVICE_UUID.lower() in {
            uuid.lower() for uuid in serviceUuids
        }
    return hasMatchingName or hasMatchingService


def onImuNotification(_characteristic: Any, data: bytearray) -> None:
    global notificationCount
    global totalNotificationCount

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

    totalNotificationCount += 1
    notificationCount += 1
    if receivedFirstSampleEvent is not None and not receivedFirstSampleEvent.is_set():
        receivedFirstSampleEvent.set()

    if notificationCount >= LOGGING_PERIOD_IN_NOTIFICATIONS:
        print(
            f"Notification #{totalNotificationCount}, sample from batch: "
            f"ax: {ax[-1]}, ay: {ay[-1]}, az: {az[-1]}, "
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


asyncio.run(main())
