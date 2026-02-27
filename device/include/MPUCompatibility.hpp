#pragma once

#include <ESPCompatibility.hpp>

#if defined(UNIT_TEST) && !defined(ESP_PLATFORM)

namespace mpud {
inline namespace types {

typedef enum {
  MPU_I2CADDRESS_AD0_LOW = 0x68,
  MPU_I2CADDRESS_AD0_HIGH = 0x69
} mpu_i2caddr_t;
using mpu_addr_handle_t = mpu_i2caddr_t;

typedef enum { GYRO_FS_250DPS = 0 } gyro_fs_t;
typedef enum { ACCEL_FS_2G = 0 } accel_fs_t;

using selftest_t = uint32_t;

struct raw_axes_t {
  int16_t x;
  int16_t y;
  int16_t z;
};

struct float_axes_t {
  float x;
  float y;
  float z;
};

using fifo_config_t = uint16_t;
static constexpr fifo_config_t FIFO_CFG_ACCEL = 1 << 3;
static constexpr fifo_config_t FIFO_CFG_GYRO = 1 << 4;

using int_en_t = uint8_t;
static constexpr int_en_t INT_EN_RAWDATA_READY = 1 << 0;

typedef enum { INT_LVL_ACTIVE_HIGH = 0 } int_lvl_t;
typedef enum { INT_DRV_PUSHPULL = 0 } int_drive_t;
typedef enum { INT_MODE_PULSE50US = 0 } int_mode_t;
typedef enum { INT_CLEAR_STATUS_REG = 0 } int_clear_t;

struct int_config_t {
  int_lvl_t level;
  int_drive_t drive;
  int_mode_t mode;
  int_clear_t clear;
};

} // namespace types

namespace math {
inline float_axes_t accelGravity(raw_axes_t aRaw, accel_fs_t) {
  return {.x = static_cast<float>(aRaw.x),
          .y = static_cast<float>(aRaw.y),
          .z = static_cast<float>(aRaw.z)};
}

inline float_axes_t gyroDegPerSec(raw_axes_t wRaw, gyro_fs_t) {
  return {.x = static_cast<float>(wRaw.x),
          .y = static_cast<float>(wRaw.y),
          .z = static_cast<float>(wRaw.z)};
}
} // namespace math

} // namespace mpud

#else

// #include "I2Cbus.hpp"
#include "MPU.hpp"
#include "mpu/math.hpp"
#include "mpu/types.hpp"

#endif
