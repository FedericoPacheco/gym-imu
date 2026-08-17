#include "IMUSampleCsv.hpp"
#include "IMUSensorPort.hpp"
#include <Constants.hpp>
#include <MPU6050NoiseReducer.hpp>
#include <array>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

const std::array<testsupport::SeriesCase, 3> testCases = {{
    {.name = "dips-2",
     .inputPath = "../signal/1-calibration/output/"
                  "dips-2-calib-affine-sixf-tilted-a-online-w.csv",
     .expectedPath =
         "../signal/2-noiseReduction/output/"
         "dips-2-calib-affine-sixf-tilted-a-online-w-filt-box5.csv"},
    {.name = "pull-ups-2",
     .inputPath = "../signal/1-calibration/output/"
                  "pull-ups-2-calib-affine-sixf-tilted-a-online-w.csv",
     .expectedPath =
         "../signal/2-noiseReduction/output/"
         "pull-ups-2-calib-affine-sixf-tilted-a-online-w-filt-box5.csv"},
    {.name = "90-deg-push-ups-2",
     .inputPath = "../signal/1-calibration/output/"
                  "90-deg-push-ups-2-calib-affine-sixf-tilted-a-online-w.csv",
     .expectedPath =
         "../signal/2-noiseReduction/output/"
         "90-deg-push-ups-2-calib-affine-sixf-tilted-a-online-w-filt-box5.csv"},
}};

TEST(MPU6050NoiseReducer_filter, FiltersSeriesCorrectly) {
  for (const testsupport::SeriesCase &series : testCases) {
    SCOPED_TRACE(series.name);

    MPU6050NoiseReducer noiseReducer;
    std::vector<IMUSample> calibratedSamples =
        testsupport::readIMUSamplesFromCSV(series.inputPath);
    const std::vector<IMUSample> expectedSamples =
        testsupport::readIMUSamplesFromCSV(series.expectedPath);

    for (IMUSample &sample : calibratedSamples) {
      noiseReducer.filter(sample);
    }

    EXPECT_NEAR_IMU_SERIES(expectedSamples, calibratedSamples, A_TEST_TOLERANCE,
                           W_TEST_TOLERANCE);
  }
}
