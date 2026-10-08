import asyncio
import contextlib
import struct
from typing import Any
from collections import deque
from bleak import BleakClient, BleakScanner
import numpy as np


class IMUSampleReceiver:
    # DEVICE CONNECTION PARAMETERS
    # Extracted from BLE.hpp.
    DEVICE_NAME = "Gym-IMU"
    IMU_SERVICE_UUID = "12345678-1234-5678-1234-56789abcdef0"
    IMU_CHARACTERISTIC_UUID = "c21c340b-f231-4da2-ab20-716f9ed67c3a"

    # DATA FORMAT
    # Device sends batches of IMU samples within each BLE notification as raw bytes in little-endian format.
    # Refer to IMUSensorPort.hpp for the exact C++ struct
    # Docs: https://docs.python.org/3/library/struct.html
    IMU_SAMPLE_STRUCT = struct.Struct("<ffffffI")  # 6 floats, 1 unsigned int

    # TIMING PARAMETERS
    DEVICE_SCAN_TIMEOUT_SECONDS = 90.0
    FIRST_SAMPLE_TIMEOUT_SECONDS = 90.0

    # MISCELLANEOUS
    # samplingPeriod / samplesPerBLEPacket -> 1 log/second
    # Adjust in sync with the firmware settings.
    LOGGING_PERIOD_IN_NOTIFICATIONS = 100 / 6  
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
        self,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        await self._listenForNotifications()
        seq = np.array(self.seqQueue, dtype=np.uint32)
        f = np.array(self.fQueue, dtype=np.float32)
        w = np.array(self.wQueue, dtype=np.float32)

        self._computeLostSamples(seq)

        return (seq, f, w)

    async def _listenForNotifications(self):
        if not self.isConnected():
            raise RuntimeError("Device not connected")
        assert self.client is not None

        # deque: append the stream of samples at the end at O(1) cost.
        # At the end of the capture, convert to numpy array at O(n) cost.
        self.seqQueue = deque()
        self.fQueue = deque()
        self.wQueue = deque()

        self.logQueue = deque()
        self.notificationCount = 0
        self.totalNotificationCount = 0

        # Create the event before subscribing to avoid missing the very first notification
        self.receivedFirstSampleEvent = asyncio.Event()

        subscribed = False
        try:
            # BlueZ: Linux bluetooth backend. 
            # At least on my laptop, it fails unless StartNotify 
            # is used instead of AcquireNotify. 
            # The additional parameter shouldn't affect windows.
            await self.client.start_notify(
                self.IMU_CHARACTERISTIC_UUID,
                lambda _characteristic, data: self._onImuNotification(
                    _characteristic, data
                ),
                bluez={"use_start_notify": True},
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
            loop = asyncio.get_running_loop()
            captureDeadline = loop.time() + self.listenDurationSeconds
            remaining = captureDeadline - loop.time()
            while remaining > 0:
                await asyncio.sleep(min(0.1, remaining))
                self._printQueuedLogs()
                remaining = captureDeadline - loop.time()
            self._printQueuedLogs()
            print("Capture window completed")
        finally:
            self.receivedFirstSampleEvent = None
            if subscribed:
                with contextlib.suppress(Exception):
                    await self.client.stop_notify(self.IMU_CHARACTERISTIC_UUID)
                print("Unsubscribed from IMU notifications")

    def _printQueuedLogs(self) -> None:
        while self.logQueue:
            print(self.logQueue.popleft())

    def _onImuNotification(self, characteristic: Any, data: bytearray) -> None:
        payload = memoryview(data)
        payloadSize = payload.nbytes

        if payloadSize == 0:
            return

        if payloadSize % self.IMU_SAMPLE_STRUCT.size != 0:
            raise ValueError("Invalid payload size")

        for (
            ax,
            ay,
            az,
            roll,
            pitch,
            yaw,
            seq,
        ) in self.IMU_SAMPLE_STRUCT.iter_unpack(payload):
            self.fQueue.append(np.array([ax, ay, az]))
            self.wQueue.append(np.array([roll, pitch, yaw]))
            self.seqQueue.append(seq)

        self.totalNotificationCount += 1
        self.notificationCount += 1
        if (
            self.receivedFirstSampleEvent is not None
            and not self.receivedFirstSampleEvent.is_set()
        ):
            self.receivedFirstSampleEvent.set()

        if self.notificationCount >= self.LOGGING_PERIOD_IN_NOTIFICATIONS:
            lastF = self.fQueue[-1]
            lastW = self.wQueue[-1]
            lastSeq = self.seqQueue[-1]
            self.logQueue.append(
                f"Notification #{self.totalNotificationCount}, sample from batch: "
                f"f: ({lastF[0]:.6f}, {lastF[1]:.6f}, {lastF[2]:.6f}), "
                f"w: ({lastW[0]:.6f}, {lastW[1]:.6f}, {lastW[2]:.6f}), "
                f"seq: {lastSeq}"
            )
            self.notificationCount = 0

    def _computeLostSamples(self, seq: np.ndarray) -> None:
        if len(seq) == 0:
            print("No samples received, skipping lost sample computation.")
            return

        diff = 0
        total = 0
        lost = []
        for i in range(1, len(seq)):
            diff = seq[i] - seq[i - 1]
            if diff > 1:
                total += diff - 1
                for j in range(1, diff):
                    lost.append(seq[i - 1] + j)

        print(
            "SAMPLE LOSS ANALYSIS:\n"
            f"Sequence numbers: {','.join(map(str, lost))}\n"
            f"Periodicity: {','.join(str(lost[i] - lost[i - 1]) for i in range(1, len(lost)))}\n"
            f"Total: {total} ({total*100.0/len(seq):.2f}%)"
        )
