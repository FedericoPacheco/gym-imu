import csv
from array import array
from pathlib import Path


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

        with open(filePath, "r", encoding="utf-8", newline="") as csvFile:
            reader = csv.DictReader(csvFile)
            if reader.fieldnames != self.FIELDS:
                raise ValueError(
                    f"Unexpected CSV header in {filePath}: {reader.fieldnames}"
                )

            for row in reader:
                motionData["seq"].append(int(row["seq"]))
                motionData["a"]["x"].append(float(row["ax"]))
                motionData["a"]["y"].append(float(row["ay"]))
                motionData["a"]["z"].append(float(row["az"]))
                motionData["w"]["roll"].append(float(row["roll"]))
                motionData["w"]["pitch"].append(float(row["pitch"]))
                motionData["w"]["yaw"].append(float(row["yaw"]))

        return motionData
