#include "IMUSensorPort.hpp"
#include <MPU6050AffineCalibrator.hpp>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

constexpr float G = 9.80665f;
constexpr float A_TOLERANCE = 0.1f;
constexpr float W_TOLERANCE =
    0.5f; // Method was more basic for angular velocities

TEST(MPU6050AffineCalibrator_calibrate,
     TransformsStationarySampleWithXAxisUpCorrectly) {

  auto calibrator = new MPU6050AffineCalibrator();
  Timestamp timestamp = 0;
  IMUSample sample = {
      .a = {.x = 10.1110f, .y = 0.3520f, .z = -0.1431f},
      .w = {.roll = -5.1195f, .pitch = 3.1892f, .yaw = -1.3810f},
      .t = timestamp};

  calibrator->calibrate(sample);

  EXPECT_NEAR(G, sample.a.x, A_TOLERANCE);
  EXPECT_NEAR(0.0f, sample.a.y, A_TOLERANCE);
  EXPECT_NEAR(0.0f, sample.a.z, A_TOLERANCE);
  EXPECT_NEAR(0.0f, sample.w.roll, W_TOLERANCE);
  EXPECT_NEAR(0.0f, sample.w.pitch, W_TOLERANCE);
  EXPECT_NEAR(0.0f, sample.w.yaw, W_TOLERANCE);
  EXPECT_EQ(sample.t, timestamp);
}

TEST(MPU6050AffineCalibrator_calibrate,
     TransformsStationarySampleWithYAxisUpCorrectly) {

  auto calibrator = new MPU6050AffineCalibrator();
  Timestamp timestamp = 0;
  IMUSample sample = {
      .a = {.x = 0.2107f, .y = 9.9925f, .z = 0.3478f},
      .w = {.roll = -5.1195f, .pitch = 3.1587f, .yaw = -1.4420f},
      .t = timestamp};

  calibrator->calibrate(sample);

  EXPECT_NEAR(0.0f, sample.a.x, A_TOLERANCE);
  EXPECT_NEAR(G, sample.a.y, A_TOLERANCE);
  EXPECT_NEAR(0.0f, sample.a.z, A_TOLERANCE);
  EXPECT_NEAR(0.0f, sample.w.roll, W_TOLERANCE);
  EXPECT_NEAR(0.0f, sample.w.pitch, W_TOLERANCE);
  EXPECT_NEAR(0.0f, sample.w.yaw, W_TOLERANCE);
  EXPECT_EQ(sample.t, timestamp);
}

TEST(MPU6050AffineCalibrator_calibrate,
     TransformsStationarySampleWithZAxisUpCorrectly) {

  auto calibrator = new MPU6050AffineCalibrator();
  Timestamp timestamp = 0;
  IMUSample sample = {
      .a = {.x = 0.4609f, .y = 0.1371f, .z = 10.2044f},
      .w = {.roll = -5.1424f, .pitch = 3.1434f, .yaw = -1.3810f},
      .t = timestamp};

  calibrator->calibrate(sample);

  EXPECT_NEAR(0.0f, sample.a.x, A_TOLERANCE);
  EXPECT_NEAR(0.0f, sample.a.y, A_TOLERANCE);
  EXPECT_NEAR(G, sample.a.z, A_TOLERANCE);
  EXPECT_NEAR(0.0f, sample.w.roll, W_TOLERANCE);
  EXPECT_NEAR(0.0f, sample.w.pitch, W_TOLERANCE);
  EXPECT_NEAR(0.0f, sample.w.yaw, W_TOLERANCE);
  EXPECT_EQ(sample.t, timestamp);
}
