#pragma once
// #include "esp_dsp.h"
// TODO: configure ESP-DSP to use optimized routines for
// the ESP32 S3 when I finally switch from the C3
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numbers>

// Helpers not defined in the standard library
inline bool isScalarClose(float a, float b, float relTol, float absTol) {
  return std::abs(a - b) <
         std::max(absTol, relTol * std::max(std::abs(a), std::abs(b)));
}
template <std::size_t N>
inline bool isVectorClose(const std::array<float, N> &u,
                          const std::array<float, N> &v, float relTol,
                          float absTol) {
  for (std::size_t i = 0; i < N; i++) {
    if (!std::abs(u[i] - v[i]) <
        std::max(absTol, relTol * std::max(std::abs(u[i]), std::abs(v[i]))))
      return false;
  }
  return true;
}
template <std::size_t N> inline int argMax(const std::array<float, N> &arr) {
  if (N == 0)
    return -1;
  else {
    int maxIdx = 0;
    for (int i = 1; i < N; i++)
      if (arr[i] > arr[maxIdx])
        maxIdx = i;
    return maxIdx;
  }
}

/* Translated from Quaternion python class at signal/signal-utils.
  fromEulerAngles() and toDCM() are not implemented to save code
   space since they are not used.
   Using the variants: std::fabsf(), std::sqrtf(), std::atan2f(), std::asinf(),
   etc. is not necessary since the standard functions are overloaded and
   select the appropriate implementation based on the argument types.

   Miscellaneous sources:
   https://en.wikipedia.org/wiki/Operators_in_C_and_C%2B%2B
   https://en.wikipedia.org/wiki/C_mathematical_functions
   https://en.cppreference.com/cpp/numeric/constants
   https://en.cppreference.com/cpp/container/array
   */

class Quaternion {
public:
  static const Quaternion IDENTITY;
  static const Quaternion ZERO;

  constexpr Quaternion(float s, std::array<float, 3> v) : s(s), v(v) {}
  Quaternion() = default;
  ~Quaternion() = default;

  static Quaternion fromDCM(std::array<std::array<float, 3>, 3> C) {
    std::array<float, 4> den = {0.0f, 0.0f, 0.0f, 0.0f};
    den[0] = 1 + C[0][0] + C[1][1] + C[2][2];
    den[1] = 1 + C[0][0] - C[1][1] - C[2][2];
    den[2] = 1 - C[0][0] + C[1][1] - C[2][2];
    den[3] = 1 - C[0][0] - C[1][1] + C[2][2];

    int maxDenIdx = argMax<4>(den);
    float s = 0.0f;
    std::array<float, 3> v = {0.0f, 0.0f, 0.0f};
    if (maxDenIdx == 0) {
      s = 0.5f * std::sqrt(std::max(den[0], 0.0f));
      v[0] = (C[2][1] - C[1][2]) / (4 * s);
      v[1] = (C[0][2] - C[2][0]) / (4 * s);
      v[2] = (C[1][0] - C[0][1]) / (4 * s);
    } else if (maxDenIdx == 1) {
      v[0] = 0.5f * std::sqrt(std::max(den[1], 0.0f));
      s = (C[2][1] - C[1][2]) / (4 * v[0]);
      v[1] = (C[0][1] + C[1][0]) / (4 * v[0]);
      v[2] = (C[0][2] + C[2][0]) / (4 * v[0]);
    } else if (maxDenIdx == 2) {
      v[1] = 0.5f * std::sqrt(std::max(den[2], 0.0f));
      s = (C[0][2] - C[2][0]) / (4 * v[1]);
      v[0] = (C[0][1] + C[1][0]) / (4 * v[1]);
      v[2] = (C[1][2] + C[2][1]) / (4 * v[1]);
    } else {
      v[2] = 0.5f * std::sqrt(std::max(den[3], 0.0f));
      s = (C[1][0] - C[0][1]) / (4 * v[2]);
      v[0] = (C[0][2] + C[2][0]) / (4 * v[2]);
      v[1] = (C[1][2] + C[2][1]) / (4 * v[2]);
    }
    return Quaternion(s, v).normalized();
  }

  std::array<float, 3> toEulerAngles(bool toDegrees = true) const {
    float roll = std::atan2(
        2.0f * (this->s * this->v[0] + this->v[1] * this->v[2]),
        1.0f - 2.0f * (this->v[0] * this->v[0] + this->v[1] * this->v[1]));
    float pitch = std::asin(std::clamp(
        2.0f * (this->s * this->v[1] - this->v[0] * this->v[2]), -1.0f, 1.0f));
    float yaw = std::atan2(
        2.0f * (this->s * this->v[2] + this->v[0] * this->v[1]),
        1.0f - 2.0f * (this->v[1] * this->v[1] + this->v[2] * this->v[2]));

    if (toDegrees) {
      roll = roll * 180.0f / std::numbers::pi_v<float>;
      pitch = pitch * 180.0f / std::numbers::pi_v<float>;
      yaw = yaw * 180.0f / std::numbers::pi_v<float>;
    }

    return std::array<float, 3>{roll, pitch, yaw};
  }

  Quaternion operator+(Quaternion other) const {
    Quaternion result;
    result.s = this->s + other.s;
    result.v[0] = this->v[0] + other.v[0];
    result.v[1] = this->v[1] + other.v[1];
    result.v[2] = this->v[2] + other.v[2];
    return result;
  }

  Quaternion operator*(Quaternion other) const {
    // More convoluted to use ESP-DSP here, and performance gains may not be big
    // due to wrapping, unwrapping, and possibly other operations, and the small
    // size of the matrix. Options: dspm_mult_4x4x1_f32(), dspm::Mat. Expand the
    // matrix multiplication instead for now:
    float s = this->s * other.s - this->v[0] * other.v[0] -
              this->v[1] * other.v[1] - this->v[2] * other.v[2];
    std::array<float, 3> v = {
        this->v[0] * other.s + this->s * other.v[0] - this->v[2] * other.v[1] +
            this->v[1] * other.v[2],
        this->v[1] * other.s + this->v[2] * other.v[0] + this->s * other.v[1] -
            this->v[0] * other.v[2],
        this->v[2] * other.s - this->v[1] * other.v[0] +
            this->v[0] * other.v[1] + this->s * other.v[2]};
    return Quaternion(s, v);
  }

  // Post-multiplication by a scalar
  Quaternion operator*(float s) const {
    Quaternion result(this->s, this->v);
    result.s *= s;
    result.v[0] *= s;
    result.v[1] *= s;
    result.v[2] *= s;
    return result;
  }

  // Pre-multiplication by a scalar. Use "friend" to define non-member operator
  // inside the class.
  friend Quaternion operator*(float s, Quaternion q) { return q * s; }

  bool operator==(const Quaternion &other) const {
    return isScalarClose(this->s, other.s, DFLT_REL_TOL, DFLT_ABS_TOL) &&
           isVectorClose(this->v, other.v, DFLT_REL_TOL, DFLT_ABS_TOL);
  }

  Quaternion conjugate() const {
    Quaternion result(this->s, this->v);
    result.s = this->s;
    result.v[0] = -this->v[0];
    result.v[1] = -this->v[1];
    result.v[2] = -this->v[2];
    return result;
  }

  float norm() const {
    return std::sqrt(this->s * this->s + this->v[0] * this->v[0] +
                     this->v[1] * this->v[1] + this->v[2] * this->v[2]);
  }

  // Intentionally different. Ensure at least degraded operation instead of
  // throwing an error on zero magnitude.
  Quaternion normalized() const {
    float magnitude = this->norm();
    if (isScalarClose(magnitude, 0.0f, DFLT_REL_TOL, DFLT_ABS_TOL))
      return *this;
    return Quaternion(this->s, this->v) * (1.0f / magnitude);
  }

  // Intentionally different. Ensure at least degraded operation instead of
  // throwing an error on zero magnitude or non-unit quaternions.
  std::array<float, 3> rotate(std::array<float, 3> u) const {
    float magnitude = this->norm();
    Quaternion q = *this;
    if (isScalarClose(magnitude, 0.0f, DFLT_REL_TOL, DFLT_ABS_TOL))
      return u;
    if (!isScalarClose(magnitude, 1.0f, DFLT_REL_TOL, DFLT_ABS_TOL))
      q = q.normalized(); // Ensure a valid rotation
    Quaternion p = Quaternion(0.0f, u);
    Quaternion rotated = q * p * q.conjugate();
    return rotated.v;
  }

private:
  static constexpr float DFLT_ABS_TOL = 1e-8f;
  static constexpr float DFLT_REL_TOL = 1e-5f;

  float s = 0.0f;
  std::array<float, 3> v = {0.0f, 0.0f, 0.0f};
};

inline constexpr Quaternion Quaternion::IDENTITY{1.0f, {0.0f, 0.0f, 0.0f}};
inline constexpr Quaternion Quaternion::ZERO{0.0f, {0.0f, 0.0f, 0.0f}};
