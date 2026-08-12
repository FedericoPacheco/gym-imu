#pragma once

#include "IMUSensorPort.hpp"
#include <cmath>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace testsupport {

struct OrientationSample {
  float roll;
  float pitch;
  float yaw;
};

struct Velocity3DSample {
  float x;
  float y;
  float z;
};

struct IMUWithAnglesSample {
  AccelerationSample a;
  AngularVelocitySample w;
  SequenceNumber seq;
  OrientationSample angle;
};

struct IMUWithVelocitySample {
  AccelerationSample a;
  AngularVelocitySample w;
  SequenceNumber seq;
  OrientationSample angle;
  Velocity3DSample v;
};

inline std::vector<std::string> splitCSVRow(const std::string &line) {
  std::vector<std::string> columns;
  std::stringstream row(line);
  std::string cell;
  while (std::getline(row, cell, ',')) {
    columns.push_back(cell);
  }
  return columns;
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

inline std::ifstream openCSVFile(const std::filesystem::path &filePath) {
  std::ifstream file(filePath);
  if (!file.is_open()) {
    throw std::runtime_error("Unable to open CSV file: " + filePath.string());
  }

  return file;
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

inline IMUWithAnglesSample
parseIMUWithAnglesSampleRow(const std::vector<std::string> &columns) {
  return IMUWithAnglesSample{
      .a = {.x = std::stof(columns[1]),
            .y = std::stof(columns[2]),
            .z = std::stof(columns[3])},
      .w = {.roll = std::stof(columns[4]),
            .pitch = std::stof(columns[5]),
            .yaw = std::stof(columns[6])},
      .seq = static_cast<SequenceNumber>(std::stoul(columns[0])),
      .angle = {.roll = std::stof(columns[7]),
                .pitch = std::stof(columns[8]),
                .yaw = std::stof(columns[9])}};
}

inline IMUWithVelocitySample
parseIMUWithVelocitySampleRow(const std::vector<std::string> &columns) {
  return IMUWithVelocitySample{
      .a = {.x = std::stof(columns[1]),
            .y = std::stof(columns[2]),
            .z = std::stof(columns[3])},
      .w = {.roll = std::stof(columns[4]),
            .pitch = std::stof(columns[5]),
            .yaw = std::stof(columns[6])},
      .seq = static_cast<SequenceNumber>(std::stoul(columns[0])),
      .angle = {.roll = std::stof(columns[7]),
                .pitch = std::stof(columns[8]),
                .yaw = std::stof(columns[9])},
      .v = {.x = std::stof(columns[10]),
            .y = std::stof(columns[11]),
            .z = std::stof(columns[12])}};
}

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

inline std::vector<IMUWithAnglesSample>
readIMUWithAnglesSamplesFromCSV(const std::filesystem::path &filePath) {
  std::ifstream file = openCSVFile(filePath);
  assertCSVHeader(readCSVHeader(file, filePath),
                  "seq,ax,ay,az,wroll,wpitch,wyaw,roll,pitch,yaw", filePath);

  std::vector<IMUWithAnglesSample> samples;
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

    samples.push_back(parseIMUWithAnglesSampleRow(columns));
  }

  return samples;
}

inline std::vector<IMUWithVelocitySample>
readIMUWithVelocitySamplesFromCSV(const std::filesystem::path &filePath) {
  std::ifstream file = openCSVFile(filePath);
  assertCSVHeader(readCSVHeader(file, filePath),
                  "seq,ax,ay,az,wroll,wpitch,wyaw,roll,pitch,yaw,vx,vy,vz",
                  filePath);

  std::vector<IMUWithVelocitySample> samples;
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

    samples.push_back(parseIMUWithVelocitySampleRow(columns));
  }

  return samples;
}

template <typename Sample>
inline ::testing::AssertionResult
AssertSeriesSameSize(const char *expectedExpr, const char *actualExpr,
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

inline ::testing::AssertionResult AssertNearIMUSeries(
    const char *expectedExpr, const char *actualExpr, const char *aTolExpr,
    const char *wTolExpr, const std::vector<IMUSample> &expected,
    const std::vector<IMUSample> &actual, float aTolerance, float wTolerance) {
  auto sizeResult =
      AssertSeriesSameSize(expectedExpr, actualExpr, expected, actual);
  if (!sizeResult) {
    return sizeResult;
  }

  for (size_t index = 0; index < expected.size(); ++index) {
    const IMUSample &expectedSample = expected[index];
    const IMUSample &actualSample = actual[index];

    if (expectedSample.seq != actualSample.seq) {
      return ::testing::AssertionFailure()
             << "Sample " << index << " sequence mismatch: expected "
             << expectedSample.seq << ", actual " << actualSample.seq;
    }

    auto result = assertNearField(index, "ax", expectedSample.a.x,
                                  actualSample.a.x, aTolerance);
    if (!result) {
      return result;
    }
    result = assertNearField(index, "ay", expectedSample.a.y, actualSample.a.y,
                             aTolerance);
    if (!result) {
      return result;
    }
    result = assertNearField(index, "az", expectedSample.a.z, actualSample.a.z,
                             aTolerance);
    if (!result) {
      return result;
    }
    result = assertNearField(index, "wroll", expectedSample.w.roll,
                             actualSample.w.roll, wTolerance);
    if (!result) {
      return result;
    }
    result = assertNearField(index, "wpitch", expectedSample.w.pitch,
                             actualSample.w.pitch, wTolerance);
    if (!result) {
      return result;
    }
    result = assertNearField(index, "wyaw", expectedSample.w.yaw,
                             actualSample.w.yaw, wTolerance);
    if (!result) {
      return result;
    }
  }

  return ::testing::AssertionSuccess();
}

inline ::testing::AssertionResult AssertNearIMUWithAnglesSeries(
    const char *expectedExpr, const char *actualExpr, const char *aTolExpr,
    const char *wTolExpr, const char *angleTolExpr,
    const std::vector<IMUWithAnglesSample> &expected,
    const std::vector<IMUWithAnglesSample> &actual, float aTolerance,
    float wTolerance, float angleTolerance) {
  auto sizeResult =
      AssertSeriesSameSize(expectedExpr, actualExpr, expected, actual);
  if (!sizeResult) {
    return sizeResult;
  }

  for (size_t index = 0; index < expected.size(); ++index) {
    const IMUWithAnglesSample &expectedSample = expected[index];
    const IMUWithAnglesSample &actualSample = actual[index];

    if (expectedSample.seq != actualSample.seq) {
      return ::testing::AssertionFailure()
             << "Sample " << index << " sequence mismatch: expected "
             << expectedSample.seq << ", actual " << actualSample.seq;
    }

    auto result = assertNearField(index, "ax", expectedSample.a.x,
                                  actualSample.a.x, aTolerance);
    if (!result) {
      return result;
    }
    result = assertNearField(index, "ay", expectedSample.a.y, actualSample.a.y,
                             aTolerance);
    if (!result) {
      return result;
    }
    result = assertNearField(index, "az", expectedSample.a.z, actualSample.a.z,
                             aTolerance);
    if (!result) {
      return result;
    }
    result = assertNearField(index, "wroll", expectedSample.w.roll,
                             actualSample.w.roll, wTolerance);
    if (!result) {
      return result;
    }
    result = assertNearField(index, "wpitch", expectedSample.w.pitch,
                             actualSample.w.pitch, wTolerance);
    if (!result) {
      return result;
    }
    result = assertNearField(index, "wyaw", expectedSample.w.yaw,
                             actualSample.w.yaw, wTolerance);
    if (!result) {
      return result;
    }
    result = assertNearField(index, "roll", expectedSample.angle.roll,
                             actualSample.angle.roll, angleTolerance);
    if (!result) {
      return result;
    }
    result = assertNearField(index, "pitch", expectedSample.angle.pitch,
                             actualSample.angle.pitch, angleTolerance);
    if (!result) {
      return result;
    }
    result = assertNearField(index, "yaw", expectedSample.angle.yaw,
                             actualSample.angle.yaw, angleTolerance);
    if (!result) {
      return result;
    }
  }

  return ::testing::AssertionSuccess();
}

inline ::testing::AssertionResult AssertNearIMUWithVelocitySeries(
    const char *expectedExpr, const char *actualExpr, const char *aTolExpr,
    const char *wTolExpr, const char *angleTolExpr, const char *vTolExpr,
    const std::vector<IMUWithVelocitySample> &expected,
    const std::vector<IMUWithVelocitySample> &actual, float aTolerance,
    float wTolerance, float angleTolerance, float velocityTolerance) {
  auto sizeResult =
      AssertSeriesSameSize(expectedExpr, actualExpr, expected, actual);
  if (!sizeResult) {
    return sizeResult;
  }

  for (size_t index = 0; index < expected.size(); ++index) {
    const IMUWithVelocitySample &expectedSample = expected[index];
    const IMUWithVelocitySample &actualSample = actual[index];

    if (expectedSample.seq != actualSample.seq) {
      return ::testing::AssertionFailure()
             << "Sample " << index << " sequence mismatch: expected "
             << expectedSample.seq << ", actual " << actualSample.seq;
    }

    auto result = assertNearField(index, "ax", expectedSample.a.x,
                                  actualSample.a.x, aTolerance);
    if (!result) {
      return result;
    }
    result = assertNearField(index, "ay", expectedSample.a.y, actualSample.a.y,
                             aTolerance);
    if (!result) {
      return result;
    }
    result = assertNearField(index, "az", expectedSample.a.z, actualSample.a.z,
                             aTolerance);
    if (!result) {
      return result;
    }
    result = assertNearField(index, "wroll", expectedSample.w.roll,
                             actualSample.w.roll, wTolerance);
    if (!result) {
      return result;
    }
    result = assertNearField(index, "wpitch", expectedSample.w.pitch,
                             actualSample.w.pitch, wTolerance);
    if (!result) {
      return result;
    }
    result = assertNearField(index, "wyaw", expectedSample.w.yaw,
                             actualSample.w.yaw, wTolerance);
    if (!result) {
      return result;
    }
    result = assertNearField(index, "roll", expectedSample.angle.roll,
                             actualSample.angle.roll, angleTolerance);
    if (!result) {
      return result;
    }
    result = assertNearField(index, "pitch", expectedSample.angle.pitch,
                             actualSample.angle.pitch, angleTolerance);
    if (!result) {
      return result;
    }
    result = assertNearField(index, "yaw", expectedSample.angle.yaw,
                             actualSample.angle.yaw, angleTolerance);
    if (!result) {
      return result;
    }
    result = assertNearField(index, "vx", expectedSample.v.x, actualSample.v.x,
                             velocityTolerance);
    if (!result) {
      return result;
    }
    result = assertNearField(index, "vy", expectedSample.v.y, actualSample.v.y,
                             velocityTolerance);
    if (!result) {
      return result;
    }
    result = assertNearField(index, "vz", expectedSample.v.z, actualSample.v.z,
                             velocityTolerance);
    if (!result) {
      return result;
    }
  }

  return ::testing::AssertionSuccess();
}

} // namespace testsupport

#define EXPECT_NEAR_IMU_SERIES(expected, actual, aTolerance, wTolerance)       \
  EXPECT_PRED_FORMAT4(testsupport::AssertNearIMUSeries, expected, actual,      \
                      aTolerance, wTolerance)

#define EXPECT_NEAR_IMU_WITH_ANGLES_SERIES(expected, actual, aTolerance,       \
                                           wTolerance, angleTolerance)         \
  EXPECT_PRED_FORMAT5(testsupport::AssertNearIMUWithAnglesSeries, expected,    \
                      actual, aTolerance, wTolerance, angleTolerance)

#define EXPECT_NEAR_IMU_WITH_VELOCITY_SERIES(expected, actual, aTolerance,     \
                                             wTolerance, angleTolerance,       \
                                             velocityTolerance)                \
  EXPECT_PRED_FORMAT6(testsupport::AssertNearIMUWithVelocitySeries, expected,  \
                      actual, aTolerance, wTolerance, angleTolerance,          \
                      velocityTolerance)
