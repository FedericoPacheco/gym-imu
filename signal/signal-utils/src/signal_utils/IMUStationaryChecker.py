from signal_utils.IMUSampleReader import IMUSampleReader
import numpy as np
import math


class IMUStationaryChecker:
    ACCEL_STD = 6
    GYRO_STD = 3

    def __init__(self):
        self.reader = IMUSampleReader()
        self.accelTol = -1.0
        self.gyroTol = -1.0
        self.g = -1.0

    def computeTolerances(self, capturePaths: list[str]):
        if not capturePaths:
            raise ValueError("No capture paths provided for tolerance computation.")

        captureAccelNorms = []
        captureGyroNorms = []
        for path in capturePaths:
            _, a, w = self.reader.readRaw(path)
            ax = a[:, 0]
            ay = a[:, 1]
            az = a[:, 2]
            captureAccelNorms.append(np.sqrt(ax**2 + ay**2 + az**2))

            wroll = w[:, 0]
            wpitch = w[:, 1]
            wyaw = w[:, 2]
            captureGyroNorms.append(np.sqrt(wroll**2 + wpitch**2 + wyaw**2))

        accelNorms = np.concatenate(captureAccelNorms)
        gyroNorms = np.concatenate(captureGyroNorms)

        accelNormsMean = np.mean(accelNorms)
        accelNormsStdev = np.std(accelNorms)
        self.g = accelNormsMean
        self.accelTol = self.ACCEL_STD * accelNormsStdev

        gyroNormsStdev = np.std(gyroNorms)
        self.gyroTol = self.GYRO_STD * gyroNormsStdev

        print(
            f"Accel norms: mean = {accelNormsMean:.6f}, stdev = {accelNormsStdev:.6f}\n Stationary tol = {self.accelTol:.6f}"
            f"\nGyro norms: stdev = {gyroNormsStdev:.6f}\n Stationary tol = {self.gyroTol:.6f}"
        )

    def isStationarySample(
        self, ax: float, ay: float, az: float, wroll: float, wpitch: float, wyaw: float
    ) -> bool:
        if self.accelTol < 0:
            raise ValueError("Tolerance not computed. Call computeTolerance() first.")

        isAccelStationary = bool(
            abs(math.sqrt(ax**2 + ay**2 + az**2) - self.g) <= self.accelTol
        )
        isGyroStationary = bool(
            math.sqrt(wroll**2 + wpitch**2 + wyaw**2) <= self.gyroTol
        )

        return isAccelStationary and isGyroStationary

    def areStationarySamples(self, a: np.ndarray, w: np.ndarray) -> np.ndarray:
        if self.accelTol < 0:
            raise ValueError("Tolerance not computed. Call computeTolerance() first.")

        ax = a[:, 0]
        ay = a[:, 1]
        az = a[:, 2]
        accelNorms = np.sqrt(ax**2 + ay**2 + az**2)

        wroll = w[:, 0]
        wpitch = w[:, 1]
        wyaw = w[:, 2]
        gyroNorms = np.sqrt(wroll**2 + wpitch**2 + wyaw**2)

        areAccelStationary = np.abs(accelNorms - self.g) <= self.accelTol
        areGyroStationary = gyroNorms <= self.gyroTol

        return areAccelStationary & areGyroStationary

    def findStationaryIntervals(self, inputFile: str) -> list[tuple[int, int]]:
        seq, a, w = self.reader.readRaw(inputFile)

        checks = self.areStationarySamples(a, w)
        wasStationary = True
        lastLowerBound = int(seq[0])
        stationaryIntervals = []
        for i, isStationary in enumerate(checks):
            if wasStationary and not isStationary:
                stationaryIntervals.append((lastLowerBound, int(seq[i - 1])))
                wasStationary = False
            if not wasStationary and isStationary:
                lastLowerBound = int(seq[i])
                wasStationary = True

        if wasStationary:
            stationaryIntervals.append((lastLowerBound, int(seq[-1])))

        return stationaryIntervals
