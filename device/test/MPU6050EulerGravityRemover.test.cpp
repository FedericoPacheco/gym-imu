#include "IMUSampleCsv.hpp"
#include "IMUSensorPort.hpp"
#include <MPU6050EulerGravityRemover.hpp>
#include <array>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

const std::array<testsupport::SeriesCase, 3> testCases = {{
    {.name = "dips-1",
     .inputPath = "../signal/3-orientation/output/ast=0.750-amv=0.950/"
                  "dips-1-calib-affine-sixf-tilted-a-online-w-"
                  "filt-box5-or-compl.csv",
     .expectedPath = "../signal/4-gravityRemoval/output/"
                     "dips-1-calib-affine-sixf-tilted-a-online-w-filt-box5-or-"
                     "compl-g-free-rot-mat.csv"},
    {.name = "pull-ups-1",
     .inputPath = "../signal/3-orientation/output/ast=0.750-amv=0.950/"
                  "pull-ups-1-calib-affine-sixf-tilted-a-online-w-filt-box5-"
                  "or-compl.csv",
     .expectedPath =
         "../signal/4-gravityRemoval/output/"
         "pull-ups-1-calib-affine-sixf-tilted-a-online-w-filt-box5-or-"
         "compl-g-free-rot-mat.csv"},
    {.name = "90-deg-push-ups-2",
     .inputPath = "../signal/3-orientation/output/ast=0.750-amv=0.950/"
                  "90-deg-push-ups-2-calib-affine-sixf-tilted-a-online-w-"
                  "filt-box5-or-compl.csv",
     .expectedPath =
         "../signal/4-gravityRemoval/output/"
         "90-deg-push-ups-2-calib-affine-sixf-tilted-a-online-w-filt-box5-or-"
         "compl-g-free-rot-mat.csv"},
}};

TEST(MPU6050EulerGravityRemover_find, RemovesGravityCorrectly) {
  for (const testsupport::SeriesCase &series : testCases) {
    SCOPED_TRACE(series.name);

    MPU6050EulerGravityRemover remover;
    std::vector<std::tuple<IMUSample, EulerOrientationSample>>
        orientatedSamples =
            testsupport::readIMUSampleWithAnglesFromCSV(series.inputPath);
    std::vector<IMUSample> gravityFreeSamples;
    for (const auto &[sample, _] :
         testsupport::readIMUSampleWithAnglesFromCSV(series.expectedPath)) {
      gravityFreeSamples.push_back(sample);
    }

    std::vector<IMUSample> results;
    for (auto &[sample, orientation] : orientatedSamples) {
      remover.remove(sample, orientation);
      results.push_back(sample);
    }

    EXPECT_NEAR_IMU_SERIES(results, gravityFreeSamples, A_TEST_TOLERANCE,
                           W_TEST_TOLERANCE);
  }
}
