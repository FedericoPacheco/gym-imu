from signal_utils.IMUSampleReader import IMUSampleReader
import numpy as np
import math


class IMUStationaryChecker:
    TOL_STD_DEVIATIONS = 6
    G = 9.80665  # m/s^2

    def __init__(self):
        self.reader = IMUSampleReader()
        self.tol = -1.0

    def computeTolerance(self, capturePaths: list[str]):
        if not capturePaths:
            raise ValueError("No capture paths provided for tolerance computation.")

        captureNorms = []
        for path in capturePaths:
            _, a, _ = self.reader.readRaw(path)
            ax = a[:, 0]
            ay = a[:, 1]
            az = a[:, 2]
            captureNorms.append(np.sqrt(ax**2 + ay**2 + az**2))
        accelNorms = np.concatenate(captureNorms)

        accelNormsMean = np.mean(accelNorms)
        accelNormsStdev = np.std(accelNorms)
        self.tol = self.TOL_STD_DEVIATIONS * accelNormsStdev
        print(
            f"Accel norms: mean={accelNormsMean:.6f}, stdev={accelNormsStdev:.6f}\nStationary tol = {self.tol:.6f}"
        )

    def isStationarySample(self, ax: float, ay: float, az: float) -> bool:
        if self.tol < 0:
            raise ValueError("Tolerance not computed. Call computeTolerance() first.")

        return bool(abs(math.sqrt(ax**2 + ay**2 + az**2) - self.G) <= self.tol)

    def areStationarySamples(self, a: np.ndarray) -> np.ndarray:
        if self.tol < 0:
            raise ValueError("Tolerance not computed. Call computeTolerance() first.")

        ax = a[:, 0]
        ay = a[:, 1]
        az = a[:, 2]
        accelNorms = np.sqrt(ax**2 + ay**2 + az**2)
        return np.abs(accelNorms - self.G) <= self.tol

    def findStationaryIntervals(self, inputFile: str) -> list[tuple[int, int]]:
        seq, a, _ = self.reader.readRaw(inputFile)

        checks = self.areStationarySamples(a)
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
