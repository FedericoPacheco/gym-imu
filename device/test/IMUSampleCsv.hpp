#pragma once

#include "IMUSensorPort.hpp"
#include "IMUSignalProcessor.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

/*
On failure, make sure to run the tests with greater verbosity to see the failed
assertions: pio run -e host -v. Otherwise it just states "FAILED".

Misc docs:
* https://cplusplus.com/reference/string/string/
* https://en.cppreference.com/cpp/string/basic_string/getline
* https://en.cppreference.com/cpp/io/basic_ifstream
* https://en.cppreference.com/cpp/string/basic_string/stof
* https://en.cppreference.com/cpp/string/basic_string/stoul
* https://en.cppreference.com/cpp/language/function_template
* https://google.github.io/googletest/advanced.html#using-a-function-that-returns-an-assertionresult
*/

namespace testsupport {

struct SeriesCase {
  const char *name;
  std::filesystem::path inputPath;
  std::filesystem::path expectedPath;
};

// Declarations to avoid compiler errors.
// Callers are defined first, then callees.
std::ifstream openCSVFile(const std::filesystem::path &filePath);
std::string readCSVHeader(std::ifstream &file,
                          const std::filesystem::path &filePath);
void assertCSVHeader(const std::string &header,
                     const std::string &expectedHeader,
                     const std::filesystem::path &filePath);
std::vector<std::string> splitCSVRow(const std::string &line);
IMUSample parseIMUSampleRow(const std::vector<std::string> &columns);
std::tuple<IMUSample, EulerOrientationSample>
parseIMUSampleWithAnglesRow(const std::vector<std::string> &columns);
std::tuple<IMUSample, EulerOrientationSample, VelocitySample>
parseVelocitySampleRow(const std::vector<std::string> &columns);
template <typename Sample>
::testing::AssertionResult
assertSeriesSameSize(const char *expectedExpr, const char *actualExpr,
                     const std::vector<Sample> &expected,
                     const std::vector<Sample> &actual);
::testing::AssertionResult assertNearField(size_t index, const char *fieldName,
                                           float expectedValue,
                                           float actualValue, float tolerance);

// --------------------------

inline std::vector<IMUSample>
readIMUSamplesFromCSV(const std::filesystem::path &filePath) {
  std::ifstream file = openCSVFile(filePath);
  assertCSVHeader(readCSVHeader(file, filePath),
                  "seq,ax,ay,az,wroll,wpitch,wyaw", filePath);

  std::vector<IMUSample> samples;
  std::string line;
  while (std::getline(file, line)) {
    if (line.empty()) {
      continue;
    }

    const std::vector<std::string> columns = splitCSVRow(line);
    if (columns.size() != 7) {
      throw std::runtime_error("Unexpected CSV column count in file: " +
                               filePath.string());
    }

    samples.push_back(parseIMUSampleRow(columns));
  }

  return samples;
}

inline std::ifstream openCSVFile(const std::filesystem::path &filePath) {
  std::ifstream file(filePath);
  if (!file.is_open()) {
    throw std::runtime_error("Unable to open CSV file: " + filePath.string());
  }

  return file;
}

inline std::string readCSVHeader(std::ifstream &file,
                                 const std::filesystem::path &filePath) {
  std::string header;
  if (!std::getline(file, header)) {
    throw std::runtime_error("CSV file is empty: " + filePath.string());
  }
  return header;
}
inline void assertCSVHeader(const std::string &header,
                            const std::string &expectedHeader,
                            const std::filesystem::path &filePath) {
  if (header != expectedHeader) {
    throw std::runtime_error("Unexpected CSV header in file: " +
                             filePath.string());
  }
}

inline std::vector<std::string> splitCSVRow(const std::string &line) {
  std::vector<std::string> columns;
  std::stringstream row(line);
  std::string cell;
  while (std::getline(row, cell, ',')) {
    columns.push_back(cell);
  }
  return columns;
}

inline IMUSample parseIMUSampleRow(const std::vector<std::string> &columns) {
  return IMUSample{.a = {.x = std::stof(columns[1]),
                         .y = std::stof(columns[2]),
                         .z = std::stof(columns[3])},
                   .w = {.roll = std::stof(columns[4]),
                         .pitch = std::stof(columns[5]),
                         .yaw = std::stof(columns[6])},
                   .seq = static_cast<SequenceNumber>(std::stoul(columns[0]))};
}

inline std::vector<std::tuple<IMUSample, EulerOrientationSample>>
readIMUSampleWithAnglesFromCSV(const std::filesystem::path &filePath) {
  std::ifstream file = openCSVFile(filePath);
  assertCSVHeader(readCSVHeader(file, filePath),
                  "seq,ax,ay,az,wroll,wpitch,wyaw,roll,pitch,yaw", filePath);

  std::vector<std::tuple<IMUSample, EulerOrientationSample>> samples;
  std::string line;
  while (std::getline(file, line)) {
    if (line.empty()) {
      continue;
    }

    const std::vector<std::string> columns = splitCSVRow(line);
    if (columns.size() != 10) {
      throw std::runtime_error("Unexpected CSV column count in file: " +
                               filePath.string());
    }

    samples.push_back(parseIMUSampleWithAnglesRow(columns));
  }

  return samples;
}

inline std::tuple<IMUSample, EulerOrientationSample>
parseIMUSampleWithAnglesRow(const std::vector<std::string> &columns) {
  return {IMUSample{
              .a = {.x = std::stof(columns[1]),
                    .y = std::stof(columns[2]),
                    .z = std::stof(columns[3])},
              .w = {.roll = std::stof(columns[4]),
                    .pitch = std::stof(columns[5]),
                    .yaw = std::stof(columns[6])},
              .seq = static_cast<SequenceNumber>(std::stoul(columns[0])),
          },
          EulerOrientationSample{.roll = std::stof(columns[7]),
                                 .pitch = std::stof(columns[8]),
                                 .yaw = std::stof(columns[9])}};
}

inline std::vector<
    std::tuple<IMUSample, EulerOrientationSample, VelocitySample>>
readVelocitySamplesFromCSV(const std::filesystem::path &filePath) {
  std::ifstream file = openCSVFile(filePath);
  assertCSVHeader(readCSVHeader(file, filePath),
                  "seq,ax,ay,az,wroll,wpitch,wyaw,roll,pitch,yaw,vx,vy,vz",
                  filePath);

  std::vector<std::tuple<IMUSample, EulerOrientationSample, VelocitySample>>
      samples;
  std::string line;
  while (std::getline(file, line)) {
    if (line.empty()) {
      continue;
    }

    const std::vector<std::string> columns = splitCSVRow(line);
    if (columns.size() != 13) {
      throw std::runtime_error("Unexpected CSV column count in file: " +
                               filePath.string());
    }

    samples.push_back(parseVelocitySampleRow(columns));
  }

  return samples;
}

inline std::tuple<IMUSample, EulerOrientationSample, VelocitySample>
parseVelocitySampleRow(const std::vector<std::string> &columns) {
  return {IMUSample{
              .a = {.x = std::stof(columns[1]),
                    .y = std::stof(columns[2]),
                    .z = std::stof(columns[3])},
              .w = {.roll = std::stof(columns[4]),
                    .pitch = std::stof(columns[5]),
                    .yaw = std::stof(columns[6])},
              .seq = static_cast<SequenceNumber>(std::stoul(columns[0])),
          },
          EulerOrientationSample{.roll = std::stof(columns[7]),
                                 .pitch = std::stof(columns[8]),
                                 .yaw = std::stof(columns[9])},
          VelocitySample{.x = std::stof(columns[10]),
                         .y = std::stof(columns[11]),
                         .z = std::stof(columns[12])}};
}

inline ::testing::AssertionResult assertNearIMUSeries(
    const char *expectedExpr, const char *actualExpr, const char *aTolExpr,
    const char *wTolExpr, const std::vector<IMUSample> &expected,
    const std::vector<IMUSample> &actual, float aTolerance, float wTolerance) {
  auto sizeResult =
      assertSeriesSameSize(expectedExpr, actualExpr, expected, actual);
  if (!sizeResult) {
    return sizeResult;
  }

  for (size_t i = 0; i < expected.size(); ++i) {
    const IMUSample &expectedSample = expected[i];
    const IMUSample &actualSample = actual[i];

    if (expectedSample.seq != actualSample.seq) {
      return ::testing::AssertionFailure()
             << "Sample " << i << " sequence mismatch: expected "
             << expectedSample.seq << ", actual " << actualSample.seq;
    }

    auto result = assertNearField(i, "ax", expectedSample.a.x, actualSample.a.x,
                                  aTolerance);
    if (!result) {
      return result;
    }
    result = assertNearField(i, "ay", expectedSample.a.y, actualSample.a.y,
                             aTolerance);
    if (!result) {
      return result;
    }
    result = assertNearField(i, "az", expectedSample.a.z, actualSample.a.z,
                             aTolerance);
    if (!result) {
      return result;
    }
    result = assertNearField(i, "wroll", expectedSample.w.roll,
                             actualSample.w.roll, wTolerance);
    if (!result) {
      return result;
    }
    result = assertNearField(i, "wpitch", expectedSample.w.pitch,
                             actualSample.w.pitch, wTolerance);
    if (!result) {
      return result;
    }
    result = assertNearField(i, "wyaw", expectedSample.w.yaw,
                             actualSample.w.yaw, wTolerance);
    if (!result) {
      return result;
    }
  }

  return ::testing::AssertionSuccess();
}

template <typename Sample>
inline ::testing::AssertionResult
assertSeriesSameSize(const char *expectedExpr, const char *actualExpr,
                     const std::vector<Sample> &expected,
                     const std::vector<Sample> &actual) {
  if (expected.size() != actual.size()) {
    return ::testing::AssertionFailure()
           << expectedExpr << " and " << actualExpr
           << " have different sizes: expected " << expected.size()
           << ", actual " << actual.size();
  }

  return ::testing::AssertionSuccess();
}

inline ::testing::AssertionResult
assertNearField(size_t index, const char *fieldName, float expectedValue,
                float actualValue, float tolerance) {
  if (std::abs(expectedValue - actualValue) <= tolerance) {
    return ::testing::AssertionSuccess();
  }
  return ::testing::AssertionFailure()
         << "Sample " << index << " field '" << fieldName
         << "' differs more than tolerance: expected " << expectedValue
         << ", actual " << actualValue << ", tolerance " << tolerance;
}

inline ::testing::AssertionResult assertNearAnglesSeries(
    const char *expectedExpr, const char *actualExpr, const char *angleTolExpr,
    const std::vector<EulerOrientationSample> &expected,
    const std::vector<EulerOrientationSample> &actual, float angleTolerance) {
  auto sizeResult =
      assertSeriesSameSize(expectedExpr, actualExpr, expected, actual);
  if (!sizeResult) {
    return sizeResult;
  }

  for (size_t i = 0; i < expected.size(); ++i) {
    const auto &expectedAngle = expected[i];
    const auto &actualAngle = actual[i];

    auto result = assertNearField(i, "roll", expectedAngle.roll,
                                  actualAngle.roll, angleTolerance);
    if (!result) {
      return result;
    }
    result = assertNearField(i, "pitch", expectedAngle.pitch, actualAngle.pitch,
                             angleTolerance);
    if (!result) {
      return result;
    }
    result = assertNearField(i, "yaw", expectedAngle.yaw, actualAngle.yaw,
                             angleTolerance);
    if (!result) {
      return result;
    }
  }

  return ::testing::AssertionSuccess();
}

inline ::testing::AssertionResult assertNearVelocitySeries(
    const char *expectedExpr, const char *actualExpr, const char *vTolExpr,
    const std::vector<VelocitySample> &expected,
    const std::vector<VelocitySample> &actual, float velocityTolerance) {
  auto sizeResult =
      assertSeriesSameSize(expectedExpr, actualExpr, expected, actual);
  if (!sizeResult) {
    return sizeResult;
  }

  for (size_t i = 0; i < expected.size(); ++i) {
    const auto &expectedVelocity = expected[i];
    const auto &actualVelocity = actual[i];

    auto result = assertNearField(i, "vx", expectedVelocity.x, actualVelocity.x,
                                  velocityTolerance);
    if (!result) {
      return result;
    }
    result = assertNearField(i, "vy", expectedVelocity.y, actualVelocity.y,
                             velocityTolerance);
    if (!result) {
      return result;
    }
    result = assertNearField(i, "vz", expectedVelocity.z, actualVelocity.z,
                             velocityTolerance);
    if (!result) {
      return result;
    }
  }

  return ::testing::AssertionSuccess();
}

} // namespace testsupport

#define EXPECT_NEAR_IMU_SERIES(expected, actual, aTolerance, wTolerance)       \
  EXPECT_PRED_FORMAT4(testsupport::assertNearIMUSeries, expected, actual,      \
                      aTolerance, wTolerance)

#define EXPECT_NEAR_ANGLES_SERIES(expected, actual, angleTolerance)            \
  EXPECT_PRED_FORMAT3(testsupport::assertNearAnglesSeries, expected, actual,   \
                      angleTolerance)

#define EXPECT_NEAR_VELOCITY_SERIES(expected, actual, velocityTolerance)       \
  EXPECT_PRED_FORMAT3(testsupport::assertNearVelocitySeries, expected, actual, \
                      velocityTolerance)
