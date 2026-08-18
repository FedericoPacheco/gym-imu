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
* When calibrating, use the raw accel and gyro
* When finding orientation, use the calibrated/low-passed accel and gyro
* When solving for velocity, use gravity-free accel and calibrated/low-passed gyro

LIMITATIONS:
Notice that true stillness is INDISTINGUISHABLE from movement at constant velocity and 
no rotations based on the IMU data alone. Why? If v = c, then a = dv/dt = 0, and if there's
no rotations, w = d(theta)/dt = 0. Another external measurement would ideally be needed.
"""


class IMUStationaryChecker:
    ACCEL_STDS = 3
    GYRO_STDS = 3

    def __init__(self):
        self.reader = IMUSampleReader()
        self.accelTol = -1.0
        self.gyroTol = -1.0
        self.accelMean = -1.0
        self.gyroMean = -1.0

    def computeTolerances(self, capturePaths: list[str]):
        if not capturePaths or len(capturePaths) == 0:
            raise ValueError("No capture paths provided for tolerance computation.")

        # Kind of hacky, but it works
        read = lambda path: self.reader.read(path)
        try:
            read(capturePaths[0])
        except Exception as e:
            try:
                read = lambda path: self.reader.readWithOrientation(path)
                read(capturePaths[0])
            except Exception as e2:
                raise ValueError(
                    f"Failed to read capture {capturePaths[0]} with both read() and readWithOrientation()."
                    f"\nCurrent working directory: {os.getcwd()}"
                ) from e2

        captureAccelNorms = []
        captureGyroNorms = []
        for path in capturePaths:
            samples = read(path)
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

        self.accelMean = np.mean(accelNorms)
        accelNormsStdev = np.std(accelNorms)
        self.accelTol = self.ACCEL_STDS * accelNormsStdev

        self.gyroMean = np.mean(gyroNorms)
        gyroNormsStdev = np.std(gyroNorms)
        self.gyroTol = self.GYRO_STDS * gyroNormsStdev

        print(
            f"Acceleration norms:"
            f"\n\tMean = {self.accelMean:.6f}"
            f"\n\tStdev = {accelNormsStdev:.6f} (tol = {self.accelTol:.6f})"
            f"\n\tStationary interval = [{self.accelMean - self.accelTol:.6f}, {self.accelMean + self.accelTol:.6f}]"
        )
        print(
            f"Gyroscope norms:"
            f"\n\tMean = {self.gyroMean:.6f}"
            f"\n\tStdev = {gyroNormsStdev:.6f} (tol = {self.gyroTol:.6f})"
            f"\n\tStationary interval = [{self.gyroMean - self.gyroTol:.6f}, {self.gyroMean + self.gyroTol:.6f}]"
        )

    # TODO: receive a and w vectors instead of individual components
    def isStationarySample(
        self,
        ax: float,
        ay: float,
        az: float,
        wroll: float,
        wpitch: float,
        wyaw: float,
    ) -> bool:
        if self.accelTol < 0:
            raise ValueError("Tolerance not computed. Call computeTolerances() first.")

        accelNorm = math.sqrt(ax**2 + ay**2 + az**2)
        gyroNorm = math.sqrt(wroll**2 + wpitch**2 + wyaw**2)

        isAccelStationary = bool(abs(accelNorm - self.accelMean) <= self.accelTol)
        isGyroStationary = bool(abs(gyroNorm - self.gyroMean) <= self.gyroTol)

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

        areAccelStationary = np.abs(accelNorms - self.accelMean) <= self.accelTol
        areGyroStationary = np.abs(gyroNorms - self.gyroMean) <= self.gyroTol

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
