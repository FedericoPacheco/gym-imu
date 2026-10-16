#pragma once
#include <Types.hpp>
#include <optional>

class IMUSensorPort {
protected:
  IMUSensorPort() = default;

public:
  virtual ~IMUSensorPort() = default;

  virtual std::optional<IMUSample> readSync() = 0;
  virtual std::optional<IMUSample> readAsync() = 0;
  virtual void beginAsync() = 0;
  virtual void stopAsync() = 0;
};