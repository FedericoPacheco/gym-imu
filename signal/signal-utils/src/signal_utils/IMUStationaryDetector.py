from pyparsing.common import abstractmethod

from signal_utils.IMUSampleReader import IMUSampleReader
import numpy as np
import math
import os

"""
IMPORTANT:
Be careful with the captures provided to computeTolerances(). 
Stationality detection changes after each processing step.
Why? Because the mean and standard deviation meaningfully change, 
and using an inappropriate criteria would yield false positives or negatives!
* When calibrating, use the raw specific force and gyro
* When finding orientation, use the calibrated/low-passed specific force and gyro
* When solving for velocity, use gravity-free acceleration and calibrated/low-passed/corrected gyro

LIMITATIONS:
Notice that true stillness is INDISTINGUISHABLE from movement at constant velocity and 
no rotations based on the IMU data alone. Why? If v = c, then a = dv/dt = 0, and if there's
no rotations, w = d(theta)/dt = 0. Another external measurement would ideally be needed.

INVARIANT:
Stationarity detection should be frame-independent, as applying a rotation matrix o quaternion 
pre/post multiplication should NOT affect the norms. 
"""


class IMUStationaryDetector:

    def __init__(
        self,
        kAccel: float = 1.0,
        kGyro: float = 1.0,
        reader=IMUSampleReader(),
    ):
        self.reader = reader
        self.kAccel = kAccel
        self.kGyro = kGyro
        self.accelTol = float("-inf")
        self.gyroTol = float("-inf")
        self.accelCenter = float("-inf")
        self.gyroCenter = float("-inf")
        self.reset()

    @abstractmethod
    def computeTolerances(self, capturePaths: list[str], doPrintResults=True):
        pass

    @abstractmethod
    def isStationarySample(
        self,
        a: np.ndarray,
        w: np.ndarray,
    ) -> bool:
        pass

    @abstractmethod
    def reset(self):
        pass

    def areStationarySamples(self, a: np.ndarray, w: np.ndarray) -> np.ndarray:
        self.reset()
        return np.array(
            [self.isStationarySample(a[i, :], w[i, :]) for i in range(len(a))]
        )

    def findStationaryIntervals(
        self, seq: np.ndarray, a: np.ndarray, w: np.ndarray
    ) -> list[tuple[int, int]]:
        checks = self.areStationarySamples(a, w)
        wasStationary = checks[0]
        lastLowerBound = int(seq[0])
        stationaryIntervals = []
        for i, isStationary in enumerate(checks[1:], start=1):
            if wasStationary and not isStationary:
                stationaryIntervals.append((lastLowerBound, int(seq[i - 1])))
                wasStationary = False
            if not wasStationary and isStationary:
                lastLowerBound = int(seq[i])
                wasStationary = True

        if wasStationary:
            stationaryIntervals.append((lastLowerBound, int(seq[-1])))

        return stationaryIntervals


class InstantaneousIMUStationaryDetector(IMUStationaryDetector):
    DFLT_ACCEL_STDS = 3
    DFLT_GYRO_STDS = 3

    def __init__(
        self,
        reader=IMUSampleReader(),
        kAccel: float = DFLT_ACCEL_STDS,
        kGyro: float = DFLT_GYRO_STDS,
    ):
        super().__init__(kAccel=kAccel, kGyro=kGyro, reader=reader)

    def computeTolerances(self, capturePaths: list[str], doPrintResults=True):
        if not capturePaths or len(capturePaths) == 0:
            raise ValueError("No capture paths provided for tolerance computation.")

        captureAccelNorms = []
        captureGyroNorms = []
        for path in capturePaths:
            samples = self.reader.read(path)
            a = samples[1]
            ax = a[:, 0]
            ay = a[:, 1]
            az = a[:, 2]
            captureAccelNorms.append(np.sqrt(ax**2 + ay**2 + az**2))

            w = samples[2]
            wroll = w[:, 0]
            wpitch = w[:, 1]
            wyaw = w[:, 2]
            captureGyroNorms.append(np.sqrt(wroll**2 + wpitch**2 + wyaw**2))

        accelNorms = np.concatenate(captureAccelNorms)
        gyroNorms = np.concatenate(captureGyroNorms)

        self.accelCenter = np.mean(accelNorms)
        accelNormsStdev = np.std(accelNorms)
        self.accelTol = self.kAccel * accelNormsStdev

        self.gyroCenter = np.mean(gyroNorms)
        gyroNormsStdev = np.std(gyroNorms)
        self.gyroTol = self.kGyro * gyroNormsStdev

        if doPrintResults:
            print(
                f"Acceleration norms:"
                f"\n\tMean = {self.accelCenter:.6f} m/s²"
                f"\n\tStdev = {accelNormsStdev:.6f} (tol = {self.accelTol:.6f}) m/s²"
                f"\n\tStationary interval = [{self.accelCenter - self.accelTol:.6f}, {self.accelCenter + self.accelTol:.6f}] m/s²"
            )
            print(
                f"Gyroscope norms:"
                f"\n\tMean = {self.gyroCenter:.6f} deg/s"
                f"\n\tStdev = {gyroNormsStdev:.6f} (tol = {self.gyroTol:.6f}) deg/s"
                f"\n\tStationary interval = [{self.gyroCenter - self.gyroTol:.6f}, {self.gyroCenter + self.gyroTol:.6f}] deg/s"
            )
            print()

    def isStationarySample(
        self,
        a: np.ndarray,
        w: np.ndarray,
    ) -> bool:
        if self.accelTol < 0:
            raise ValueError("Tolerance not computed. Call computeTolerances() first.")

        accelNorm = np.linalg.norm(a)
        gyroNorm = np.linalg.norm(w)

        isAccelStationary = bool(abs(accelNorm - self.accelCenter) <= self.accelTol)
        isGyroStationary = bool(abs(gyroNorm - self.gyroCenter) <= self.gyroTol)

        return isAccelStationary and isGyroStationary

    def areStationarySamples(self, a: np.ndarray, w: np.ndarray) -> np.ndarray:
        if self.accelTol < 0:
            raise ValueError("Tolerance not computed. Call computeTolerances() first.")

        ax = a[:, 0]
        ay = a[:, 1]
        az = a[:, 2]
        accelNorms = np.sqrt(ax**2 + ay**2 + az**2)

        wroll = w[:, 0]
        wpitch = w[:, 1]
        wyaw = w[:, 2]
        gyroNorms = np.sqrt(wroll**2 + wpitch**2 + wyaw**2)

        areAccelStationary = np.abs(accelNorms - self.accelCenter) <= self.accelTol
        areGyroStationary = np.abs(gyroNorms - self.gyroCenter) <= self.gyroTol

        return areAccelStationary & areGyroStationary


class WindowedIMUStationaryDetector(IMUStationaryDetector):
    DFLT_ACCEL_MADS = 3 * 1.4826
    DFLT_GYRO_MADS = 6 * 1.4826

    # Short but meaningful:
    # 7.5 samples at 30 Hz
    # 15 samples at 60 Hz
    # 30 samples at 120 Hz
    WINDOW_TIME_SECONDS = 0.25

    def __init__(
        self,
        samplingFrequency: float,
        kAccel: float = DFLT_ACCEL_MADS,
        kGyro: float = DFLT_GYRO_MADS,
        reader=IMUSampleReader(),
    ):
        super().__init__(reader=reader, kAccel=kAccel, kGyro=kGyro)
        self.windowSize = math.floor(self.WINDOW_TIME_SECONDS * samplingFrequency)

    def computeTolerances(self, capturePaths: list[str], doPrintResults=True):
        if not capturePaths or len(capturePaths) == 0:
            raise ValueError("No capture paths provided for tolerance computation.")

        captureAccelNorms = []
        captureGyroNorms = []
        for path in capturePaths:
            samples = self.reader.read(path)
            a = samples[1]
            w = samples[2]
            captureAccelNorms.append(np.linalg.norm(a, axis=1))
            captureGyroNorms.append(np.linalg.norm(w, axis=1))
        accelNorms = np.concatenate(captureAccelNorms)
        gyroNorms = np.concatenate(captureGyroNorms)

        self.accelCenter = np.median(accelNorms)
        self.gyroCenter = np.median(gyroNorms)
        accelMad = np.median(np.abs(accelNorms - self.accelCenter))
        gyroMad = np.median(np.abs(gyroNorms - self.gyroCenter))
        self.accelTol = self.kAccel * accelMad
        self.gyroTol = self.kGyro * gyroMad

        if doPrintResults:
            print(
                f"Acceleration norms:"
                f"\n\tMedian = {self.accelCenter:.6f} m/s²"
                f"\n\tMAD = {accelMad:.6f} (tol = {self.accelTol:.6f}) m/s²"
                f"\n\tStationary interval = [{self.accelCenter - self.accelTol:.6f}, {self.accelCenter + self.accelTol:.6f}] m/s²"
            )
            print(
                f"Gyroscope norms:"
                f"\n\tMedian = {self.gyroCenter:.6f} deg/s"
                f"\n\tMAD = {gyroMad:.6f} (tol = {self.gyroTol:.6f}) deg/s"
                f"\n\tStationary interval = [{self.gyroCenter - self.gyroTol:.6f}, {self.gyroCenter + self.gyroTol:.6f}] deg/s"
            )
            print()

    # Meant to be called repeatedly with samples from a data stream
    def isStationarySample(self, a: np.ndarray, w: np.ndarray) -> bool:
        if a.shape != (3,) or w.shape != (3,):
            raise ValueError("Input arrays must have shape (3,).")

        isAccelStationary = (
            np.abs(np.linalg.norm(a) - self.accelCenter) <= self.accelTol
        )
        isGyroStationary = np.abs(np.linalg.norm(w) - self.gyroCenter) <= self.gyroTol

        if isAccelStationary & isGyroStationary:
            self.stationaryCount += 1
            if self.stationaryCount >= self.windowSize:
                return True
        else:
            self.stationaryCount = 0
        return False

    def reset(self):
        self.stationaryCount = 0
