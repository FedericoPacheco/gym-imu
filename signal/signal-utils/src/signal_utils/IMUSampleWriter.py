from __future__ import annotations

from pathlib import Path
from typing import Optional
import numpy as np


class IMUSampleWriter:

    def write(
        self,
        outputFile: str,
        seq: np.ndarray,
        a: np.ndarray,
        w: np.ndarray,
        angle: Optional[np.ndarray] = None,
        q: Optional[np.ndarray] = None,
        v: Optional[np.ndarray] = None,
    ):
        if seq is None:
            raise ValueError("Sequence array cannot be None")
        if a is None:
            raise ValueError("Acceleration array cannot be None")
        if w is None:
            raise ValueError("Gyroscope array cannot be None")

        data = [
            self._normalizeUint(seq),
            self._normalizeFloat(a),
            self._normalizeFloat(w),
        ]
        header = "seq,ax,ay,az,wroll,wpitch,wyaw"
        format = ["%u"] + [
            "%.6f"
        ] * 6  # One unsigned int for seq, six floats for accel and gyro
        if angle is not None:
            data.append(self._normalizeFloat(angle))
            header += ",roll,pitch,yaw"
            format += ["%.6f"] * 3
        if q is not None:
            data.append(self._normalizeFloat(q, 4))
            header += ",q0,q1,q2,q3"
            format += ["%.6f"] * 4
        if v is not None:
            data.append(self._normalizeFloat(v))
            header += ",vx,vy,vz"
            format += ["%.6f"] * 3

        lengths = [d.shape[0] for d in data]
        if not all(l == lengths[0] for l in lengths):
            raise ValueError("All input arrays must have the same length")

        out = Path(outputFile)
        out.parent.mkdir(parents=True, exist_ok=True)
        np.savetxt(
            out,
            np.hstack(data),
            delimiter=",",
            header=header,
            comments="",
            fmt=format,
        )

    def _normalizeUint(self, seq: np.ndarray) -> np.ndarray:
        seqArray = np.asarray(seq)
        if seqArray.dtype != np.uint32:
            seqArray = seqArray.astype(np.uint32, copy=False)
        return seqArray.reshape(-1, 1)

    def _normalizeFloat(self, arr: np.ndarray, cols: int = 3) -> np.ndarray:
        arrArray = np.asarray(arr)
        if arrArray.dtype != np.float32:
            arrArray = arrArray.astype(np.float32, copy=False)
        return arrArray.reshape(-1, cols)
