#pragma once
#include "time.h"
#include <cstdint>
#include <optional>

struct PipeMetrics {
  uint32_t drops;
  uint32_t maxDepth;
  uint32_t currentDepth;
};

template <typename T, uint32_t N> class Pipe {
public:
  virtual ~Pipe() = default;

  virtual bool push(const T &item) = 0;
  virtual std::optional<T> pop(bool blockWhenEmpty = true) = 0;
  virtual uint32_t itemsFilled() const = 0;
  virtual uint32_t itemsLeft() const = 0;
  virtual PipeMetrics getMetrics() const = 0;

protected:
  Pipe() = default;
};