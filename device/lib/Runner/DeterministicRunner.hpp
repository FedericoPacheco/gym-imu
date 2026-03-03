#include "Runner.hpp"

class DeterministicRunner : public Runner {
public:
  bool start(std::function<void(void *, uint32_t)> loopFunction,
             void *argForCallback) override {
    this->storedArgForCallback = argForCallback;
    this->storedLoopFunction = std::move(loopFunction);
    return true;
  }
  void notify() override { this->pending++; }
  void notifyFromISR() override { this->pending++; }
  void runOneStep() override {
    if (this->pending > 0) {
      this->pending--;
      this->storedLoopFunction(this->storedArgForCallback, 1);
    }
  }
  void stop() override {
    this->pending = 0;
    this->storedArgForCallback = nullptr;
    this->storedLoopFunction = nullptr;
  }

private:
  int pending{0};
  void *storedArgForCallback{nullptr};
  std::function<void(void *, uint32_t)> storedLoopFunction{nullptr};
};