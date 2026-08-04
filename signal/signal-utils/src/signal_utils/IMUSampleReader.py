import csv
from array import array
from pathlib import Path
import numpy as np


class IMUSampleReader:

    def read(self, inputFile: str) -> tuple[
        np.ndarray,
        np.ndarray,
        np.ndarray,
    ]:
        filePath = self._checkFileExists(inputFile)

        headers = ["seq", "ax", "ay", "az", "wroll", "wpitch", "wyaw"]
        self._checkHeaders(filePath, headers)

        data = np.loadtxt(filePath, delimiter=",", skiprows=1, dtype=np.float32)
        data = np.atleast_2d(data)

        seq = data[:, 0].astype(np.uint32, copy=False)
        a = data[:, 1:4]
        w = data[:, 4:7]

        return (seq, a, w)

    def readWithOrientation(self, inputFile: str) -> tuple[
        np.ndarray,
        np.ndarray,
        np.ndarray,
        np.ndarray,
    ]:
        filePath = self._checkFileExists(inputFile)

        headers = [
            "seq",
            "ax",
            "ay",
            "az",
            "wroll",
            "wpitch",
            "wyaw",
            "roll",
            "pitch",
            "yaw",
        ]
        self._checkHeaders(filePath, headers)

        data = np.loadtxt(filePath, delimiter=",", skiprows=1, dtype=np.float32)
        data = np.atleast_2d(data)

        seq = data[:, 0].astype(np.uint32, copy=False)
        a = data[:, 1:4]
        w = data[:, 4:7]
        angle = data[:, 7:10]

        return (seq, a, w, angle)

    def _checkFileExists(self, inputFile: str) -> Path:
        filePath = Path(inputFile)
        if not filePath.exists():
            raise FileNotFoundError(f"Capture file not found: {filePath}")
        return filePath

    def _checkHeaders(self, filePath: Path, expectedHeaders: list[str]):
        with open(filePath, "r", encoding="utf-8", newline="") as csvFile:
            header = csvFile.readline().strip().split(",")
            if header != expectedHeaders:
                raise ValueError(f"Unexpected CSV header in {filePath}: {header}")
