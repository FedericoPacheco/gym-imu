#include <IMUNoiseReducer.hpp>
#include <StationaryChecker.hpp>
#include <array>

/*
Rationale on implementation:
On signal/2-noiseReduction, a causal moving average filters looks at past
indexes to update the window sum. This is obviously not possible on a streaming
scenario. Instead, use a circular buffer to store the last N samples, where N is
the window length. When the window is full, remove the oldest one and place the
newest one in its place. Notice that updating is place is cheaper than shifting
the entire array and placing the new one at the end (O(1) vs O(N)). Also notice
that the window sum is still used instead of using two pointers/indexes (as in a
typical leetcode sliding window problem), which again is cheaper than summing
the entire window every time (O(1) vs O(N)).
*/

class MPU6050NoiseReducer : public IMUNoiseReducer {
private:
  static const int WINDOW_SUM_LENGTH = 5;

  std::array<double, 3> windowSumGyro = {0.0, 0.0, 0.0};
  std::array<double, 3> windowSumAccel = {0.0, 0.0, 0.0};
  std::array<IMUSample, WINDOW_SUM_LENGTH> windowSamples;
  int windowLength = 0;
  int windowIdx = 0;

public:
  MPU6050NoiseReducer() = default;
  ~MPU6050NoiseReducer() = default;

  void filter(IMUSample &sample) override {
    if (windowLength >= WINDOW_SUM_LENGTH) {
      IMUSample oldest = windowSamples[windowIdx];

      windowSumAccel[0] -= oldest.a.x;
      windowSumAccel[1] -= oldest.a.y;
      windowSumAccel[2] -= oldest.a.z;
      windowSumGyro[0] -= oldest.w.roll;
      windowSumGyro[1] -= oldest.w.pitch;
      windowSumGyro[2] -= oldest.w.yaw;
    } else {
      windowLength++;
    }

    windowSamples[windowIdx] = sample;

    windowSumAccel[0] += sample.a.x;
    windowSumAccel[1] += sample.a.y;
    windowSumAccel[2] += sample.a.z;
    windowSumGyro[0] += sample.w.roll;
    windowSumGyro[1] += sample.w.pitch;
    windowSumGyro[2] += sample.w.yaw;

    // Modify the sample in place to save memory instead of returning a new
    // sample
    sample.a.x = windowSumAccel[0] / windowLength;
    sample.a.y = windowSumAccel[1] / windowLength;
    sample.a.z = windowSumAccel[2] / windowLength;

    sample.w.roll = windowSumGyro[0] / windowLength;
    sample.w.pitch = windowSumGyro[1] / windowLength;
    sample.w.yaw = windowSumGyro[2] / windowLength;

    windowIdx = (windowIdx + 1) % WINDOW_SUM_LENGTH;
  }
};