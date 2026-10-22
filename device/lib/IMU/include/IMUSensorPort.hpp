#pragma once
#include <Types.hpp>
#include <optional>

class IMUSensorPort {
protected:
  IMUSensorPort() = default;

public:
  enum class State { IDLE, READING, STOPPING };

  // Bit mask indicating that the IMU has stopped. Used to notify the
  // coordinator task.
  static constexpr uint32_t STOPPED_MASK = 1u << 0;

  virtual ~IMUSensorPort() = default;

  virtual std::optional<IMUSample> readSync() = 0;
  virtual std::optional<IMUSample> readAsync() = 0;
  virtual void beginAsync() = 0;
  virtual void requestStopAsync() = 0;
};