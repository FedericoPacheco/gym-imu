#include "Runner.hpp"

class DeterministicRunner : public Runner {
public:
  bool start(std::function<void(void *, uint32_t)> loopFunction,
             void *argForCallback) override {
    this->startCallCount++;
    this->storedArgForCallback = argForCallback;
    this->storedLoopFunction = std::move(loopFunction);
    return true;
  }
  void notify() override { this->pendingNotifications++; }
  void notifyFromISR() override { this->pendingNotifications++; }
  void runOneStep() override {
    if (this->pendingNotifications > 0) {
      this->pendingNotifications--;
      this->storedLoopFunction(this->storedArgForCallback, 1);
    }
  }
  void stop() override {
    this->pendingNotifications = 0;
    this->storedArgForCallback = nullptr;
    this->storedLoopFunction = nullptr;
  }

  int getPendingPendingNotifications() const {
    return this->pendingNotifications;
  }
  int getStartCallCount() const { return this->startCallCount; }

private:
  int pendingNotifications{0};
  int startCallCount{0};
  void *storedArgForCallback{nullptr};
  std::function<void(void *, uint32_t)> storedLoopFunction{nullptr};
};