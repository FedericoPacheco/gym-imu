#pragma once

#include <cstdint>

inline constexpr float g = 9.80665f; // m/s²

inline constexpr int SAMPLING_FREQUENCY_HZ = 30;

inline constexpr uint32_t SAMPLING_PIPE_SIZE = 128;
inline constexpr uint32_t TRANSMISSION_PIPE_SIZE = SAMPLING_PIPE_SIZE;

inline constexpr int IMU_READ_TASK_STACK_SIZE = 4096;
inline constexpr int IMU_READ_TASK_MAX_BATCH = 12;
inline constexpr int IMU_READ_TASK_PRIORITY = 5;

inline constexpr int PROCESS_TASK_STACK_SIZE = 4096;
inline constexpr int PROCESS_TASK_PRIORITY = 5;

inline constexpr int BLE_TASK_PRIORITY = 4;
inline constexpr int BLE_TASK_STACK_SIZE = 4096;
inline constexpr int TRANSMIT_TASK_PRIORITY = 4;
inline constexpr int TRANSMIT_TASK_STACK_SIZE = 4096;

inline constexpr float A_TEST_TOLERANCE = 0.0001f;
inline constexpr float W_TEST_TOLERANCE = 0.0001f;
inline constexpr float ANGLE_TEST_TOLERANCE = 0.0001f;