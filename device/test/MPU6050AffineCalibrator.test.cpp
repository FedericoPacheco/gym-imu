#include "IMUSampleCsv.hpp"
#include "IMUSensorPort.hpp"
#include <MPU6050AffineCalibrator.hpp>
#include <array>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

/*
Misc docs:
* https://google.github.io/googletest/reference/testing.html#SCOPED_TRACE
*/

constexpr float A_TOLERANCE = 0.0001f;
constexpr float W_TOLERANCE = 0.0001f;

const std::array<testsupport::SeriesCase, 3> testCases = {{
    {.name = "dips-1",
     .inputPath = "../signal/0-capture/real-exercises/apr-28-2026/dips-1.csv",
     .expectedPath = "../signal/1-calibration/output/"
                     "dips-1-calib-affine-sixf-tilted-a-online-w.csv"},
    {.name = "pull-ups-1",
     .inputPath =
         "../signal/0-capture/real-exercises/apr-28-2026/pull-ups-1.csv",
     .expectedPath = "../signal/1-calibration/output/"
                     "pull-ups-1-calib-affine-sixf-tilted-a-online-w.csv"},
    {.name = "90-deg-push-ups-1",
     .inputPath =
         "../signal/0-capture/real-exercises/apr-28-2026/90-deg-push-ups-1.csv",
     .expectedPath =
         "../signal/1-calibration/output/"
         "90-deg-push-ups-1-calib-affine-sixf-tilted-a-online-w.csv"},
}};

TEST(MPU6050AffineCalibrator_calibrate, CalibratesSeriesCorrectly) {
  for (const testsupport::SeriesCase &series : testCases) {
    SCOPED_TRACE(series.name);

    MPU6050AffineCalibrator calibrator;
    std::vector<IMUSample> rawSamples =
        testsupport::readIMUSamplesFromCSV(series.inputPath);
    const std::vector<IMUSample> expectedSamples =
        testsupport::readIMUSamplesFromCSV(series.expectedPath);

    for (IMUSample &sample : rawSamples) {
      calibrator.calibrate(sample);
    }

    EXPECT_NEAR_IMU_SERIES(expectedSamples, rawSamples, A_TOLERANCE,
                           W_TOLERANCE);
  }
}
