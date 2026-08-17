#include "IMUSampleCsv.hpp"
#include "IMUSensorPort.hpp"
#include <MPU6050ComplementaryOrientationFinder.hpp>
#include <array>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

constexpr float A_TOLERANCE = 0.0001f;
constexpr float W_TOLERANCE = 0.0001f;
constexpr float ANGLE_TOLERANCE = 0.0001f;

const std::array<testsupport::SeriesCase, 3> testCases = {{
    {.name = "dips-1",
     .inputPath = "../signal/2-noiseReduction/output/"
                  "dips-1-calib-affine-sixf-tilted-a-online-w-filt-box5.csv",
     .expectedPath = "../signal/3-orientation/output/ast=0.750-amv=0.950/"
                     "dips-1-calib-affine-sixf-tilted-a-online-w-"
                     "filt-box5-or-compl.csv"},
    {.name = "pull-ups-1",
     .inputPath =
         "../signal/2-noiseReduction/output/"
         "pull-ups-1-calib-affine-sixf-tilted-a-online-w-filt-box5.csv",
     .expectedPath = "../signal/3-orientation/output/ast=0.750-amv=0.950/"
                     "pull-ups-1-calib-affine-sixf-tilted-a-online-w-filt-box5-"
                     "or-compl.csv"},
    {.name = "90-deg-push-ups-1",
     .inputPath =
         "../signal/2-noiseReduction/output/"
         "90-deg-push-ups-1-calib-affine-sixf-tilted-a-online-w-filt-box5.csv",
     .expectedPath = "../signal/3-orientation/output/ast=0.750-amv=0.950/"
                     "90-deg-push-ups-1-calib-affine-sixf-tilted-a-online-w-"
                     "filt-box5-or-compl.csv"},
}};

TEST(MPU6050ComplementaryOrientationFinder_find,
     ProducesOrientationSeriesCorrectly) {
  for (const testsupport::SeriesCase &series : testCases) {
    SCOPED_TRACE(series.name);

    MPU6050ComplementaryOrientationFinder orientator;
    const std::vector<IMUSample> noiseFreeSamples =
        testsupport::readIMUSamplesFromCSV(series.inputPath);
    const std::vector<std::tuple<IMUSample, EulerOrientationSample>>
        expectedSamples =
            testsupport::readIMUSampleWithAnglesFromCSV(series.expectedPath);
    std::vector<EulerOrientationSample> expectedAngles;
    for (const auto [_, expectedAngle] : expectedSamples) {
      expectedAngles.push_back(expectedAngle);
    }
    std::vector<EulerOrientationSample> orientationResultsSamples;

    for (IMUSample sample : noiseFreeSamples) {
      orientationResultsSamples.push_back(orientator.find(sample));
    }

    EXPECT_NEAR_ANGLES_SERIES(expectedAngles, orientationResultsSamples,
                              ANGLE_TOLERANCE);
  }
}
