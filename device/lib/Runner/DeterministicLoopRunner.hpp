#pragma once

#include <LoopRunner.hpp>
#include <utility>

class DeterministicLoopRunner : public LoopRunner {
public:
  bool start(std::function<void(void *)> loopFunction,
             void *argForCallback) override {
    this->startCallCount++;
    this->storedArgForCallback = argForCallback;
    this->storedLoopFunction = std::move(loopFunction);
    this->isRunning = true;
    return true;
  }

  void runOneStep() override {
    if (this->isRunning && this->storedLoopFunction)
      this->storedLoopFunction(this->storedArgForCallback);
  }

  void stop() override {
    this->isRunning = false;
    this->storedArgForCallback = nullptr;
    this->storedLoopFunction = nullptr;
  }

  int getStartCallCount() const { return this->startCallCount; }

private:
  bool isRunning{false};
  int startCallCount{0};
  void *storedArgForCallback{nullptr};
  std::function<void(void *)> storedLoopFunction{nullptr};
};
