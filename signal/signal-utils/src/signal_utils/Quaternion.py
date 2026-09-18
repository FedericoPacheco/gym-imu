import numpy as np
import math


def clip(lowerBound, value, upperBound):
    return max(lowerBound, min(value, upperBound))


class Quaternion:
    ABS_TOL = 1e-8
    REL_TOL = 1e-5

    def __init__(self, s: float = 0.0, v: np.ndarray = np.zeros(3, dtype=np.float32)):
        self.s = s
        self.v = np.array(v, dtype=np.float32)

    def __str__(self):
        return f"Quaternion(s={self.s}, v={self.v})"

    def __eq__(self, other):
        return math.isclose(
            self.s, other.s, abs_tol=self.ABS_TOL, rel_tol=self.REL_TOL
        ) and np.allclose(self.v, other.v, atol=self.ABS_TOL, rtol=self.REL_TOL)

    def __add__(self, other):
        if not isinstance(other, Quaternion):
            return NotImplemented
        return Quaternion(
            self.s + other.s,
            self.v + other.v,
        )

    def __mul__(self, other):
        if isinstance(other, Quaternion):
            return Quaternion(
                self.s * other.s - np.dot(self.v, other.v),
                self.s * other.v + other.s * self.v + np.cross(self.v, other.v),
            )
        if isinstance(other, (int, float)):
            return Quaternion(
                self.s * other,
                self.v * other,
            )
        return NotImplemented

    # Scalar multiplication from the left: forward to __mul__() commuting the operators
    def __rmul__(self, scalar):
        return self * scalar

    def conjugate(self):
        return Quaternion(self.s, -self.v)

    def norm(self):
        return math.sqrt(self.s**2 + np.dot(self.v, self.v))

    def normalized(self):
        magnitude = self.norm()
        if math.isclose(magnitude, 0.0, abs_tol=self.ABS_TOL, rel_tol=self.REL_TOL):
            raise ValueError("Cannot normalize a zero quaternion.")
        return Quaternion(self.s / magnitude, self.v / magnitude)

    def rotate(self, u):
        magnitude = self.norm()
        if math.isclose(magnitude, 0.0, abs_tol=self.ABS_TOL, rel_tol=self.REL_TOL):
            raise ValueError("Cannot rotate a vector with a zero quaternion.")
        if not math.isclose(magnitude, 1.0, abs_tol=self.ABS_TOL, rel_tol=self.REL_TOL):
            raise ValueError("Quaternion must be normalized to rotate a vector.")
        p = Quaternion(0.0, np.array(u, dtype=np.float32))
        rotated = self * p * self.conjugate()
        return rotated.v

    def toEulerAngles(self, toDegrees=True):
        roll = math.atan2(
            2.0 * (self.s * self.v[0] + self.v[1] * self.v[2]),
            1.0 - 2.0 * (self.v[0] ** 2 + self.v[1] ** 2),
        )
        # Clamp value to avoid exception "math domain error"
        pitch = math.asin(
            clip(-1.0, 2 * (self.s * self.v[1] - self.v[0] * self.v[2]), 1.0)
        )
        yaw = math.atan2(
            2.0 * (self.s * self.v[2] + self.v[0] * self.v[1]),
            1.0 - 2.0 * (self.v[1] ** 2 + self.v[2] ** 2),
        )

        if toDegrees:
            roll = math.degrees(roll)
            pitch = math.degrees(pitch)
            yaw = math.degrees(yaw)

        return np.array(
            [
                roll,
                pitch,
                yaw,
            ]
        )

    @staticmethod
    def fromEulerAngles(roll, pitch, yaw, inDegrees=True):
        rollRad = roll
        pitchRad = pitch
        yawRad = yaw
        if inDegrees:
            rollRad = math.radians(roll)
            pitchRad = math.radians(pitch)
            yawRad = math.radians(yaw)

        cosRoll = math.cos(0.5 * rollRad)
        sinRoll = math.sin(0.5 * rollRad)
        cosPitch = math.cos(0.5 * pitchRad)
        sinPitch = math.sin(0.5 * pitchRad)
        cosYaw = math.cos(0.5 * yawRad)
        sinYaw = math.sin(0.5 * yawRad)

        s = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw
        v = np.zeros(3, dtype=np.float32)
        v[0] = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw
        v[1] = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw
        v[2] = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw

        return Quaternion(s, v)

    def toDCM(self):
        magnitude = self.norm()
        if not math.isclose(magnitude, 1.0, abs_tol=self.ABS_TOL, rel_tol=self.REL_TOL):
            raise ValueError("Quaternion must be normalized to convert it to a DCM.")

        c11 = self.s**2 + self.v[0] ** 2 - self.v[1] ** 2 - self.v[2] ** 2
        c12 = 2 * (self.v[0] * self.v[1] - self.s * self.v[2])
        c13 = 2 * (self.v[0] * self.v[2] + self.s * self.v[1])
        c21 = 2 * (self.v[0] * self.v[1] + self.s * self.v[2])
        c22 = self.s**2 - self.v[0] ** 2 + self.v[1] ** 2 - self.v[2] ** 2
        c23 = 2 * (self.v[1] * self.v[2] - self.s * self.v[0])
        c31 = 2 * (self.v[0] * self.v[2] - self.s * self.v[1])
        c32 = 2 * (self.v[1] * self.v[2] + self.s * self.v[0])
        c33 = self.s**2 - self.v[0] ** 2 - self.v[1] ** 2 + self.v[2] ** 2

        return np.array([[c11, c12, c13], [c21, c22, c23], [c31, c32, c33]])

    @staticmethod
    def fromDCM(C: np.ndarray):
        den = np.zeros(4, dtype=np.float32)
        den[0] = 1 + C[0, 0] + C[1, 1] + C[2, 2]
        den[1] = 1 + C[0, 0] - C[1, 1] - C[2, 2]
        den[2] = 1 - C[0, 0] + C[1, 1] - C[2, 2]
        den[3] = 1 - C[0, 0] - C[1, 1] + C[2, 2]

        maxDenIdx = np.argmax(den)
        s = 0
        v = np.zeros(3, dtype=np.float32)
        if maxDenIdx == 0:
            s = 0.5 * math.sqrt(max(den[0], 0))
            v[0] = (C[2, 1] - C[1, 2]) / (4 * s)
            v[1] = (C[0, 2] - C[2, 0]) / (4 * s)
            v[2] = (C[1, 0] - C[0, 1]) / (4 * s)
        elif maxDenIdx == 1:
            v[0] = 0.5 * math.sqrt(max(den[1], 0))
            s = (C[2, 1] - C[1, 2]) / (4 * v[0])
            v[1] = (C[0, 1] + C[1, 0]) / (4 * v[0])
            v[2] = (C[0, 2] + C[2, 0]) / (4 * v[0])
        elif maxDenIdx == 2:
            v[1] = 0.5 * math.sqrt(max(den[2], 0))
            s = (C[0, 2] - C[2, 0]) / (4 * v[1])
            v[0] = (C[0, 1] + C[1, 0]) / (4 * v[1])
            v[2] = (C[1, 2] + C[2, 1]) / (4 * v[1])
        else:
            v[2] = 0.5 * math.sqrt(max(den[3], 0))
            s = (C[1, 0] - C[0, 1]) / (4 * v[2])
            v[0] = (C[0, 2] + C[2, 0]) / (4 * v[2])
            v[1] = (C[1, 2] + C[2, 1]) / (4 * v[2])

        return Quaternion(s, v).normalized()
