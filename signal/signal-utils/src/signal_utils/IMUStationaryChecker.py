from signal_utils.IMUSampleReader import IMUSampleReader
import numpy as np
import math


class IMUStationaryChecker:
    # https://en.wikipedia.org/wiki/68%E2%80%9395%E2%80%9399.7_rule#Table_of_numerical_values
    ACCEL_STD = 5
    GYRO_STD = 3
    # EDGE_COUNT = 3

    def __init__(self):
        self.reader = IMUSampleReader()
        self.accelTol = -1.0
        self.gyroTol = -1.0
        self.g = -1.0
        self.sampleCount = 0

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

    # Assumes it's called repeatedly on sequential samples
    def isStationarySample(
        self, ax: float, ay: float, az: float, wroll: float, wpitch: float, wyaw: float
    ) -> bool:
        if self.accelTol < 0:
            raise ValueError("Tolerance not computed. Call computeTolerances() first.")

        isAccelStationary = bool(
            abs(math.sqrt(ax**2 + ay**2 + az**2) - self.g) <= self.accelTol
        )
        isGyroStationary = bool(
            math.sqrt(wroll**2 + wpitch**2 + wyaw**2) <= self.gyroTol
        )

        # if isAccelStationary and isGyroStationary:
        #     self.sampleCount = min(self.sampleCount + 1, self.EDGE_COUNT)
        #     if self.sampleCount == self.EDGE_COUNT:
        #         return True
        # else:
        #     self.sampleCount = max(self.sampleCount - 1, 0)
        #     if self.sampleCount > 0:
        #         return True
        # return False

        return isAccelStationary and isGyroStationary

    def areStationarySamples(self, a: np.ndarray, w: np.ndarray) -> np.ndarray:
        # self.sampleCount = 0
        # results = np.zeros(a.shape[0], dtype=bool)
        # for i in range(len(a)):
        #     results[i] = self.isStationarySample(
        #         a[i, 0], a[i, 1], a[i, 2], w[i, 0], w[i, 1], w[i, 2]
        #     )
        # return results

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

        areAccelStationary = np.abs(accelNorms - self.g) <= self.accelTol
        areGyroStationary = gyroNorms <= self.gyroTol

        return areAccelStationary & areGyroStationary

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
