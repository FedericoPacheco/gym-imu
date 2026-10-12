#pragma once
#include <array>
#include <cmath>
#include <cstddef>

// Python's math.isclose()
inline bool isScalarClose(float a, float b, float relTol, float absTol) {
  return std::abs(a - b) <
         std::max(absTol, relTol * std::max(std::abs(a), std::abs(b)));
}

// Numpy's allclose() for arrays
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

// Numpy's argmax() for arrays
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