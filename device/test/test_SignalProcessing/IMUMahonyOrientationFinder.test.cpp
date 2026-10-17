#include "IMUSampleCsv.hpp"
#include "Types.hpp"
#include <3-Orientation/IMUMahonyOrientationFinder.hpp>
#include <Constants.hpp>
#include <array>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <stdint.h>

const std::array<testsupport::SeriesCase, 3> testCases = {{
    {.name = "dips-3",
     .inputPath = "../signal/2-noiseReduction/output/"
                  "mpu6050-dips-3-calib-affine-sixf-tilted-a-filt-box5.csv",
     .expectedPath =
         "../signal/3-orientation/output/mahony/km=0.250-kp=5.000-ki=0.050/"
         "mpu6050-dips-3-calib-affine-sixf-tilted-a-"
         "filt-box5-or-mahony.csv"},
    {.name = "pull-ups-3",
     .inputPath = "../signal/2-noiseReduction/output/"
                  "mpu6050-pull-ups-3-calib-affine-sixf-tilted-a-filt-box5.csv",
     .expectedPath =
         "../signal/3-orientation/output/mahony/km=0.250-kp=5.000-ki=0.050/"
         "mpu6050-pull-ups-3-calib-affine-sixf-tilted-a-filt-box5-"
         "or-mahony.csv"},
    {.name = "90-deg-push-ups-1",
     .inputPath = "../signal/2-noiseReduction/output/"
                  "mpu6050-90-deg-push-ups-1-calib-affine-sixf-tilted-a-filt-"
                  "box5.csv",
     .expectedPath =
         "../signal/3-orientation/output/mahony/km=0.250-kp=5.000-ki=0.050/"
         "mpu6050-90-deg-push-ups-1-calib-affine-sixf-tilted-a-"
         "filt-box5-or-mahony.csv"},
}};

TEST(IMUMahonyOrientationFinder_find, ProducesOrientationSeriesCorrectly) {
  for (const testsupport::SeriesCase &series : testCases) {
    SCOPED_TRACE(series.name);

    const float dt = 1.0f / 30.0f;
    const uint32_t initSamplesCount = 30;
    IMUMahonyOrientationFinder orientator(dt);
    auto noiseFreeSamples =
        testsupport::readIMUSamplesFromCSV(series.inputPath);
    const auto [_, expectedQuaternions] =
        testsupport::readIMUSampleWithQuaternionsFromCSV(series.expectedPath);
    std::vector<Quaternion> orientationResultsSamples;

    for (uint32_t i = 0; i < initSamplesCount; i++) {
      orientator.initialize(noiseFreeSamples[i]);
    }
    for (uint32_t i = initSamplesCount - 1; i < noiseFreeSamples.size(); i++) {
      orientationResultsSamples.push_back(orientator.find(noiseFreeSamples[i]));
    }

    EXPECT_NEAR_QUATERNION_SERIES(expectedQuaternions,
                                  orientationResultsSamples, Q_TEST_TOLERANCE);

    orientator.reset();
  }
}
