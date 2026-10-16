#include "IMUSampleCsv.hpp"
#include "IMUSensorPort.hpp"
#include "IMUIntegrationVelocityEstimator.hpp"
#include <array>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

const std::array<testsupport::SeriesCase, 3> testCases = {{
    {.name = "dips-2",
     .inputPath =
         "../signal/4-gravityRemoval/output/"
                     "mpu6050-dips-2-calib-affine-sixf-tilted-a-filt-box5-or-"
                     "mahony-g-free.csv",
     .expectedPath = "../signal/5-velocity/output/"
                     "mpu6050-dips-2-calib-affine-sixf-tilted-a-filt-box5-or-"
                     "mahony-g-free-online-a-v-global-hard-zupt-axis-hold.csv"},
    {.name = "pull-ups-2",
     .inputPath =
         "../signal/4-gravityRemoval/output/"
         "mpu6050-pull-ups-2-calib-affine-sixf-tilted-a-filt-box5-or-"
         "mahony-g-free.csv",
     .expectedPath =
         "../signal/5-velocity/output/"
         "mpu6050-pull-ups-2-calib-affine-sixf-tilted-a-filt-box5-or-"
         "mahony-g-free-online-a-v-global-hard-zupt-axis-hold.csv"},
    {.name = "90-deg-push-ups-1",
     .inputPath =
         "../signal/4-gravityRemoval/output/"
                     "mpu6050-90-deg-push-ups-1-calib-affine-sixf-tilted-a-"
                     "filt-box5-or-mahony-g-free.csv",
     .expectedPath = "../signal/5-velocity/output/"
                     "mpu6050-90-deg-push-ups-1-calib-affine-sixf-tilted-a-"
                     "filt-box5-or-mahony-g-free-online-a-v-global-hard-zupt-axis-hold.csv"},
}};

TEST(IMUIntegrationVelocityEstimator_estimate, EstimatesVelocityCorrectly) {
  for (const testsupport::SeriesCase &series : testCases) {
    SCOPED_TRACE(series.name);

    const float dt = 1.0f / 30.0f;
    IMUIntegrationVelocityEstimator estimator(dt);
    auto [gravityFreeImuSamples, orientations] =
        testsupport::readIMUSampleWithQuaternionsFromCSV(series.inputPath);
    auto [onlineCalibratedImuSamples, _, velocitySamples] =
        testsupport::readIMUSamplesWithVelocitiesFromCSV(series.expectedPath);

    std::vector<IMUSample> calibratedImuSamplesResults;
    std::vector<VelocitySample> velocitySamplesResults;
    for (int i = 0; i < gravityFreeImuSamples.size(); ++i) {
        auto velocitySample = estimator.estimate(gravityFreeImuSamples[i], orientations[i]);
        calibratedImuSamplesResults.push_back(gravityFreeImuSamples[i]);
        velocitySamplesResults.push_back(velocitySample);
    }

    EXPECT_NEAR_IMU_SERIES(onlineCalibratedImuSamples, calibratedImuSamplesResults, A_TEST_TOLERANCE,
                           W_TEST_TOLERANCE);
    EXPECT_NEAR_VELOCITY_SERIES(velocitySamples, velocitySamplesResults, V_TEST_TOLERANCE);
  }
}
