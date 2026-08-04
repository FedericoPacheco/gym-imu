#pragma once

#include <cstdint>

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