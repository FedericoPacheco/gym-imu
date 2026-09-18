import csv
from array import array
from pathlib import Path
import numpy as np


class IMUSampleReader:

    def read(self, inputFile: str) -> tuple[np.ndarray, ...]:
        filePath = self._checkFileExists(inputFile)

        with open(filePath, "r", encoding="utf-8", newline="") as csvFile:
            header = csvFile.readline().strip().split(",")

        data = np.loadtxt(filePath, delimiter=",", skiprows=1, dtype=np.float32)
        data = np.atleast_2d(data)

        result = ()
        remainingCols = header
        if remainingCols[0] == "seq":
            result += (data[:, 0].astype(np.uint32, copy=False),)
            remainingCols = remainingCols[1:]
            data = data[:, 1:]
        if remainingCols[0:3] == ["ax", "ay", "az"]:
            result += (data[:, 0:3].astype(np.float32, copy=False),)
            remainingCols = remainingCols[3:]
            data = data[:, 3:]
        if remainingCols[0:3] == ["wroll", "wpitch", "wyaw"]:
            result += (data[:, 0:3].astype(np.float32, copy=False),)
            remainingCols = remainingCols[3:]
            data = data[:, 3:]
        if remainingCols[0:3] == ["roll", "pitch", "yaw"]:
            result += (data[:, 0:3].astype(np.float32, copy=False),)
            remainingCols = remainingCols[3:]
            data = data[:, 3:]
        if remainingCols[0:4] == ["q0", "q1", "q2", "q3"]:
            result += (data[:, 0:4].astype(np.float32, copy=False),)
            remainingCols = remainingCols[4:]
            data = data[:, 4:]
        if remainingCols[0:3] == ["vx", "vy", "vz"]:
            result += (data[:, 0:3].astype(np.float32, copy=False),)
            remainingCols = remainingCols[3:]
            data = data[:, 3:]

        if len(remainingCols) > 0:
            raise ValueError(f"Unexpected remaining columns: {remainingCols}")

        return result

    def _checkFileExists(self, inputFile: str) -> Path:
        filePath = Path(inputFile)
        if not filePath.exists():
            raise FileNotFoundError(f"Capture file not found: {filePath}")
        return filePath
