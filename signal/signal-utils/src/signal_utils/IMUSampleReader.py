import csv
from array import array
from pathlib import Path
import numpy as np


class IMUSampleReader:
    FIELDS = ["seq", "ax", "ay", "az", "roll", "pitch", "yaw"]

    def read(self, inputFile: str) -> dict[str, array | dict[str, array]]:
        filePath = Path(inputFile)
        if not filePath.exists():
            raise FileNotFoundError(f"Capture file not found: {filePath}")

        motionData = {
            "a": {"x": array("f"), "y": array("f"), "z": array("f")},
            "w": {"roll": array("f"), "pitch": array("f"), "yaw": array("f")},
            "seq": array("I"),
        }

        fields = ["seq", "ax", "ay", "az", "wroll", "wpitch", "wyaw"]
        with open(filePath, "r", encoding="utf-8", newline="") as csvFile:
            reader = csv.DictReader(csvFile)
            if reader.fieldnames != fields:
                raise ValueError(
                    f"Unexpected CSV header in {filePath}: {reader.fieldnames}"
                )

            for row in reader:
                motionData["seq"].append(int(row["seq"]))
                motionData["a"]["x"].append(float(row["ax"]))
                motionData["a"]["y"].append(float(row["ay"]))
                motionData["a"]["z"].append(float(row["az"]))
                motionData["w"]["roll"].append(float(row["wroll"]))
                motionData["w"]["pitch"].append(float(row["wpitch"]))
                motionData["w"]["yaw"].append(float(row["wyaw"]))

        return motionData

    def readRaw(self, inputFile: str) -> dict[str, array | dict[str, array]]:
        return self.read(inputFile)

    def readFiltered(self, inputFile: str) -> tuple[
        np.ndarray,
        np.ndarray,
        np.ndarray,
    ]:
        filePath = Path(inputFile)
        if not filePath.exists():
            raise FileNotFoundError(f"Capture file not found: {filePath}")

        with open(filePath, "r", encoding="utf-8", newline="") as csvFile:
            header = csvFile.readline().strip().split(",")
            if header != self.FIELDS:
                raise ValueError(f"Unexpected CSV header in {filePath}: {header}")

        data = np.loadtxt(filePath, delimiter=",", skiprows=1, dtype=np.float32)
        data = np.atleast_2d(data)

        seq = data[:, 0].astype(np.uint32, copy=False)
        a = data[:, 1:4]
        angle = data[:, 4:7]

        return (seq, a, angle)
