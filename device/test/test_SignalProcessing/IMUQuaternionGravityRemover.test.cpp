#include "IMUSampleCsv.hpp"
#include "Types.hpp"
#include <4-GravityRemoval/IMUQuaternionGravityRemover.hpp>
#include <array>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

const std::array<testsupport::SeriesCase, 3> testCases = {{
    {.name = "dips-1",
     .inputPath =
         "../signal/3-orientation/output/mahony/km=0.250-kp=5.000-ki=0.050/"
         "mpu6050-dips-1-calib-affine-sixf-tilted-a-"
         "filt-box5-or-mahony.csv",
     .expectedPath = "../signal/4-gravityRemoval/output/"
                     "mpu6050-dips-1-calib-affine-sixf-tilted-a-filt-box5-or-"
                     "mahony-g-free.csv"},
    {.name = "pull-ups-1",
     .inputPath =
         "../signal/3-orientation/output/mahony/km=0.250-kp=5.000-ki=0.050/"
         "mpu6050-pull-ups-1-calib-affine-sixf-tilted-a-filt-box5-"
         "or-mahony.csv",
     .expectedPath =
         "../signal/4-gravityRemoval/output/"
         "mpu6050-pull-ups-1-calib-affine-sixf-tilted-a-filt-box5-or-"
         "mahony-g-free.csv"},
    {.name = "90-deg-push-ups-2",
     .inputPath =
         "../signal/3-orientation/output/mahony/km=0.250-kp=5.000-ki=0.050/"
         "mpu6050-90-deg-push-ups-2-calib-affine-sixf-tilted-a-"
         "filt-box5-or-mahony.csv",
     .expectedPath = "../signal/4-gravityRemoval/output/"
                     "mpu6050-90-deg-push-ups-2-calib-affine-sixf-tilted-a-"
                     "filt-box5-or-mahony-g-free.csv"},
}};

TEST(IMUQuaternionGravityRemover_remove, RemovesGravityCorrectly) {
  for (const testsupport::SeriesCase &series : testCases) {
    SCOPED_TRACE(series.name);

    IMUQuaternionGravityRemover remover;
    auto [imuSamples, orientations] =
        testsupport::readIMUSampleWithQuaternionsFromCSV(series.inputPath);
    auto [gravityFreeImuSamples, _] =
        testsupport::readIMUSampleWithQuaternionsFromCSV(series.expectedPath);

    std::vector<IMUSample> results;
    for (int i = 0; i < imuSamples.size(); ++i) {
      remover.remove(imuSamples[i], orientations[i]);
      results.push_back(imuSamples[i]);
    }

    EXPECT_NEAR_IMU_SERIES(results, gravityFreeImuSamples, A_TEST_TOLERANCE,
                           W_TEST_TOLERANCE);
  }
}
