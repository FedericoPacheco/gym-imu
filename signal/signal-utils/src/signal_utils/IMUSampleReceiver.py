from array import array
import asyncio
import contextlib
import struct
from typing import Any

from bleak import BleakClient, BleakScanner


class IMUSampleReceiver:
    # DEVICE CONNECTION PARAMETERS
    # Extracted from BLE.hpp.
    DEVICE_NAME = "Gym-IMU"
    IMU_SERVICE_UUID = "12345678-1234-5678-1234-56789abcdef0"
    IMU_CHARACTERISTIC_UUID = "c21c340b-f231-4da2-ab20-716f9ed67c3a"

    # DATA FORMAT
    # Device sends batches of IMU samples within each BLE notification as raw bytes in little-endian format.
    # Refer to IMUSensorPort.hpp for the exact C++ struct.
    IMU_SAMPLE_STRUCT = struct.Struct("<ffffffq")  # 6 floats, 1 int64_t timestamp

    # TIMING PARAMETERS
    DEVICE_SCAN_TIMEOUT_SECONDS = 90.0
    FIRST_SAMPLE_TIMEOUT_SECONDS = 90.0

    # MISCELLANEOUS
    LOGGING_PERIOD_IN_NOTIFICATIONS = 10

    def __init__(
        self,
        listenDurationSeconds: float = 30.0,
    ) -> None:
        self.listenDurationSeconds = listenDurationSeconds
        self.client = None

    def isConnected(self) -> bool:
        return self.client is not None and self.client.is_connected

    async def connect(self) -> None:
        target = await self._findTargetDevice()
        self.client = BleakClient(target)

        try:
            await self.client.connect()
            print(f"Connected to {target.address} ({target.name})")
        except Exception as e:
            raise RuntimeError(f"Failed to connect to the device: {e}")

        services = self.client.services
        service = services.get_service(self.IMU_SERVICE_UUID)
        if service is None:
            raise RuntimeError(f"Required service not found: {self.IMU_SERVICE_UUID}")

        characteristic = services.get_characteristic(self.IMU_CHARACTERISTIC_UUID)
        if characteristic is None:
            raise RuntimeError(
                f"Required characteristic not found: {self.IMU_CHARACTERISTIC_UUID}"
            )

        hasNotifyProperty = "notify" in characteristic.properties
        if not hasNotifyProperty:
            raise RuntimeError("Characteristic does not support notifications")

    async def _findTargetDevice(self) -> Any:
        print(
            f"Scanning for BLE device '{self.DEVICE_NAME}' for up to {self.DEVICE_SCAN_TIMEOUT_SECONDS:.0f}s..."
        )

        target = await BleakScanner.find_device_by_filter(
            lambda _, ad: self._matchesDevice(ad.local_name, ad.service_uuids),
            timeout=self.DEVICE_SCAN_TIMEOUT_SECONDS,
        )

        if target is None:
            raise RuntimeError(
                "BLE device not found. Check that it is powered and advertising."
            )

        print(f"Found device: name={target.name}, address={target.address}")
        return target

    def _matchesDevice(
        self, advertisementDataName: str | None, serviceUuids: list[str] | None
    ) -> bool:
        hasMatchingName = advertisementDataName == self.DEVICE_NAME
        hasMatchingService = False
        if serviceUuids:
            hasMatchingService = self.IMU_SERVICE_UUID.lower() in {
                uuid.lower() for uuid in serviceUuids
            }
        return hasMatchingName or hasMatchingService

    async def disconnect(self) -> None:
        if self.isConnected():
            assert self.client is not None
            await self.client.disconnect()
            print("Disconnected from the device")

    async def receive(
        self, exportedFileName: str = "motionData.csv"
    ) -> dict[str, array | dict[str, array]]:

        await self._listenForNotifications()
        self._export(exportedFileName)

        return {
            "a": {"x": self.ax[:], "y": self.ay[:], "z": self.az[:]},
            "w": {"roll": self.roll[:], "pitch": self.pitch[:], "yaw": self.yaw[:]},
            "t": self.tMicroseconds[:],
        }

    async def _listenForNotifications(self):
        if not self.isConnected():
            raise RuntimeError("Device not connected")
        assert self.client is not None

        # Prefer C-typed arrays over Python lists/dictionaries for better performance and lower memory overhead
        self.ax = array("f")
        self.ay = array("f")
        self.az = array("f")
        self.roll = array("f")
        self.pitch = array("f")
        self.yaw = array("f")
        self.tMicroseconds = array("q")

        self.notificationCount = 0
        self.totalNotificationCount = 0

        # Create the event before subscribing to avoid missing the very first notification
        self.receivedFirstSampleEvent = asyncio.Event()

        subscribed = False
        try:
            await self.client.start_notify(
                self.IMU_CHARACTERISTIC_UUID,
                lambda _characteristic, data: self._onImuNotification(
                    _characteristic, data
                ),
            )
            subscribed = True
            print("Subscribed to IMU notifications")
            print(
                "Press the wearable button now to start transmission. "
                "Waiting for first sample..."
            )
            try:
                await asyncio.wait_for(
                    self.receivedFirstSampleEvent.wait(),
                    timeout=self.FIRST_SAMPLE_TIMEOUT_SECONDS,
                )
            except asyncio.TimeoutError as e:
                raise RuntimeError(
                    "No samples received after subscribing. "
                    "Press the wearable button and retry."
                ) from e

            print(f"Receiving samples for {self.listenDurationSeconds:.0f} seconds...")
            await asyncio.sleep(self.listenDurationSeconds)
            print("Capture window completed")
        finally:
            self.receivedFirstSampleEvent = None
            if subscribed:
                with contextlib.suppress(Exception):
                    await self.client.stop_notify(self.IMU_CHARACTERISTIC_UUID)
                print("Unsubscribed from IMU notifications")

    def _onImuNotification(self, _characteristic: Any, data: bytearray) -> None:
        payload = memoryview(data)
        payloadSize = payload.nbytes

        if payloadSize == 0:
            return

        if payloadSize % self.IMU_SAMPLE_STRUCT.size != 0:
            raise ValueError("Invalid payload size")

        for (
            sampleAx,
            sampleAy,
            sampleAz,
            sampleRoll,
            samplePitch,
            sampleYaw,
            sampleT,
        ) in self.IMU_SAMPLE_STRUCT.iter_unpack(payload):
            self.ax.append(sampleAx)
            self.ay.append(sampleAy)
            self.az.append(sampleAz)
            self.roll.append(sampleRoll)
            self.pitch.append(samplePitch)
            self.yaw.append(sampleYaw)
            self.tMicroseconds.append(sampleT)

        self.totalNotificationCount += 1
        self.notificationCount += 1
        if (
            self.receivedFirstSampleEvent is not None
            and not self.receivedFirstSampleEvent.is_set()
        ):
            self.receivedFirstSampleEvent.set()

        if self.notificationCount >= self.LOGGING_PERIOD_IN_NOTIFICATIONS:
            print(
                f"Notification #{self.totalNotificationCount}, sample from batch: "
                f"ax: {self.ax[-1]}, ay: {self.ay[-1]}, az: {self.az[-1]}, "
                f"roll: {self.roll[-1]}, pitch: {self.pitch[-1]}, yaw: {self.yaw[-1]}, "
                f"timestamp: {self.tMicroseconds[-1]}"
            )
            self.notificationCount = 0

    def _export(self, fileName: str) -> None:
        with open(fileName, "w", encoding="utf-8") as f:
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
                self.tMicroseconds,
                self.ax,
                self.ay,
                self.az,
                self.roll,
                self.pitch,
                self.yaw,
            ):
                f.write(
                    f"{sampleT},{sampleAx},{sampleAy},{sampleAz},{sampleRoll},{samplePitch},{sampleYaw}\n"
                )
        print(f"Motion data saved to {fileName}")
