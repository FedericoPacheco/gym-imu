#pragma once
#include <LoggerPort.hpp>
#include <Pipe.hpp>
#include <atomic>
#include <cstdint>
#include <memory>
#include <optional>
#include <ports/FreeRTOS/FreeRTOSPort.hpp>

template <typename T, uint32_t N> class QueuePipe : public Pipe<T, N> {
public:
  static std::shared_ptr<QueuePipe<T, N>> create(LoggerPort *logger) {
    std::shared_ptr<QueuePipe<T, N>> queuePipe(new (std::nothrow)
                                                   QueuePipe<T, N>(logger));

    if (!queuePipe) {
      if (logger)
        logger->error("Failed to allocate QueuePipe");
      return nullptr;
    }

    queuePipe->handle = xQueueCreateStatic(N, sizeof(T), queuePipe->buffer,
                                           &queuePipe->controlBlock);
    if (queuePipe->handle == NULL) {
      if (logger)
        logger->error("Failed to create sample queue");
      return nullptr;
    }

    return queuePipe;
  }

  ~QueuePipe() { vQueueDelete(this->handle); }

  bool push(const T &item) override {
    // Try a non-blocking enqueue
    if (xQueueSend(this->handle, &item, 0) == pdPASS) {
      updateMax(this->maxDepth, this->itemsFilled());
      return true;
    }

    // Policy: if full, drop one oldest item and retry once
    this->logger->warn("Enqueue failed: pipe full, dropping oldest sample");
    T droppedItem;
    if (xQueueReceive(this->handle, &droppedItem, 0) != pdPASS) {
      if (this->logger)
        this->logger->warn(
            "Enqueue failed and oldest item could not be dropped");
      return false;
    }
    this->incrementDrops();
    if (this->logger)
      this->logger->debug("Dropped oldest sample to make space");

    if (xQueueSend(this->handle, &item, 0) != pdPASS) {
      this->incrementDrops();
      if (this->logger)
        this->logger->warn("Enqueue retry failed after dropping oldest sample");
      return false;
    }

    return true;
  }

  std::optional<T> pop(bool blockWhenEmpty = true) override {
    T item;
    TickType_t timeout = blockWhenEmpty ? portMAX_DELAY : 0;
    if (xQueueReceive(this->handle, &item, timeout) == pdPASS) {
      return item;
    } else
      return std::nullopt;
  }

  void empty() override {
    while (this->pop(false).has_value())
      ;
  }

  uint32_t itemsFilled() const override {
    return uxQueueMessagesWaiting(this->handle);
  }

  uint32_t itemsLeft() const override {
    return uxQueueSpacesAvailable(this->handle);
  }

  PipeMetrics getMetrics() const override {
    return {.drops = this->getDrops(),
            .maxDepth = this->getMaxDepth(),
            .currentDepth = this->itemsFilled()};
  }

private:
  LoggerPort *logger;

  uint8_t buffer[N * sizeof(T)];
  StaticQueue_t controlBlock;
  QueueHandle_t handle;

  std::atomic<uint32_t> drops;
  std::atomic<uint32_t> maxDepth;

  QueuePipe(LoggerPort *logger) : logger(logger), drops(0), maxDepth(0) {}

  // Atomic operation to capture transient max values
  static inline void updateMax(std::atomic<uint32_t> &maxValue,
                               uint32_t newValue) {
    uint32_t previousMax = maxValue.load(std::memory_order_relaxed);
    while (previousMax < newValue &&
           !maxValue.compare_exchange_weak(previousMax, newValue,
                                           std::memory_order_relaxed)) {
    }
  }

  uint32_t inline getDrops() const {
    return drops.load(std::memory_order_relaxed);
  }
  uint32_t inline incrementDrops() {
    return this->drops.fetch_add(1, std::memory_order_relaxed) + 1;
  }
  uint32_t inline getMaxDepth() const {
    return maxDepth.load(std::memory_order_relaxed);
  }
};