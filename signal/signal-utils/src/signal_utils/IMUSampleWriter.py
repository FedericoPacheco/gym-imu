from __future__ import annotations

from pathlib import Path
import numpy as np
from array import array


class IMUSampleWriter:
    # Write legacy `motionData` produced by `IMUSampleReader.read()`
    def write(self, outputFile: str, motionData: dict[str, array | dict[str, array]]):
        seq = np.asarray(motionData["seq"])

        accelX = np.asarray(motionData["a"]["x"])  # type: ignore
        accelY = np.asarray(motionData["a"]["y"])  # type: ignore
        accelZ = np.asarray(motionData["a"]["z"])  # type: ignore
        a = np.column_stack((accelX, accelY, accelZ))

        gyroRoll = np.asarray(motionData["w"]["roll"])  # type: ignore
        gyroPitch = np.asarray(motionData["w"]["pitch"])  # type: ignore
        gyroYaw = np.asarray(motionData["w"]["yaw"])  # type: ignore
        w = np.column_stack((gyroRoll, gyroPitch, gyroYaw))

        self.writeRaw(outputFile, seq, a, w)

    def writeRaw(self, outputFile: str, seq: np.ndarray, a: np.ndarray, w: np.ndarray):
        out = Path(outputFile)
        out.parent.mkdir(parents=True, exist_ok=True)

        seqColumn = self._normalizeUint(seq)
        accelArray = self._normalizeFloat(a)
        gyroArray = self._normalizeFloat(w)

        if not (accelArray.shape[0] == gyroArray.shape[0] == seqColumn.shape[0]):
            raise ValueError("All inputs must have the same number of rows")

        data = np.hstack((seqColumn, accelArray, gyroArray))
        header = "seq,ax,ay,az,wroll,wpitch,wyaw"
        format = ["%u"] + [
            "%.6f"
        ] * 6  # One unsigned int for seq, six floats for accel and gyro
        # Doc: https://numpy.org/doc/stable/reference/generated/numpy.savetxt.html
        np.savetxt(out, data, delimiter=",", header=header, comments="", fmt=format)

    def writeFiltered(
        self,
        outputFile: str,
        seq: np.ndarray,
        a: np.ndarray,
        w: np.ndarray,
        angle: np.ndarray,
    ):
        out = Path(outputFile)
        out.parent.mkdir(parents=True, exist_ok=True)

        seqColumn = self._normalizeUint(seq)
        accelArray = self._normalizeFloat(a)
        gyroArray = self._normalizeFloat(w)
        angleArray = self._normalizeFloat(angle)

        if not (
            accelArray.shape[0]
            == gyroArray.shape[0]
            == angleArray.shape[0]
            == seqColumn.shape[0]
        ):
            raise ValueError("All inputs must have the same number of rows")

        data = np.hstack((seqColumn, accelArray, gyroArray, angleArray))
        header = "seq,ax,ay,az,wroll,wpitch,wyaw,roll,pitch,yaw"
        format = ["%u"] + ["%.6f"] * 9
        np.savetxt(out, data, delimiter=",", header=header, comments="", fmt=format)

    def _normalizeUint(self, seq: np.ndarray) -> np.ndarray:
        seqArray = np.asarray(seq)
        if seqArray.dtype != np.uint32:
            seqArray = seqArray.astype(np.uint32, copy=False)
        return seqArray.reshape(-1, 1)

    def _normalizeFloat(self, arr: np.ndarray) -> np.ndarray:
        arrArray = np.asarray(arr)
        if arrArray.dtype != np.float32:
            arrArray = arrArray.astype(np.float32, copy=False)
        return arrArray.reshape(-1, 3)
