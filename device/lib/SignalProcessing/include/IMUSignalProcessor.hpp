#pragma once

#include <Auxiliary/Quaternion.hpp>
#include <Constants.hpp>
#include <IMUCalibrator.hpp>
#include <IMUGravityRemover.hpp>
#include <IMUNoiseReducer.hpp>
#include <IMUOrientationFinder.hpp>
#include <IMUVelocityEstimator.hpp>
#include <LoggerPort.hpp>
#include <LoopRunner.hpp>
#include <Pipe.hpp>
#include <Types.hpp>
#include <atomic>
#include <cstdint>
#include <memory>
#include <ports/FreeRTOS/FreeRTOSPort.hpp>

class IMUSignalProcessor {
public:
  enum class State { IDLE, INITIALIZING, PROCESSING, DRAINING };

  // Bit mask indicating that the processor has stopped. Used to notify the
  // coordinator task.
  static constexpr uint32_t STOPPED_MASK = 1u << 1;

  static IMUSignalProcessor *getInstance(
      std::shared_ptr<Pipe<IMUSample, SAMPLING_PIPE_SIZE>> inputPipe,
      std::shared_ptr<Pipe<ProcessedSample, TRANSMISSION_PIPE_SIZE>> outputPipe,
      std::unique_ptr<LoopRunner> runner, LoggerPort *logger,
      std::unique_ptr<IMUCalibrator> calibrator,
      std::unique_ptr<IMUNoiseReducer> noiseReducer,
      std::unique_ptr<IMUOrientationFinder<Quaternion>> orientationFinder,
      std::unique_ptr<IMUGravityRemover<Quaternion>> gravityRemover,
      std::unique_ptr<IMUVelocityEstimator> velocityEstimator,
      uint32_t initializationSamplesCount, TaskHandle_t coordinator);

#if defined(UNIT_TEST) && !defined(ESP_PLATFORM)
  static void resetInstanceForTests();
#endif

  ~IMUSignalProcessor();

  void beginProcessing();
  void requestStopProcessing();
  State getState();

private:
  std::shared_ptr<Pipe<IMUSample, SAMPLING_PIPE_SIZE>> inputPipe;
  std::shared_ptr<Pipe<ProcessedSample, TRANSMISSION_PIPE_SIZE>> outputPipe;
  std::unique_ptr<LoopRunner> runner;
  LoggerPort *logger;
  std::unique_ptr<IMUCalibrator> calibrator;
  std::unique_ptr<IMUNoiseReducer> noiseReducer;
  std::unique_ptr<IMUOrientationFinder<Quaternion>> orientationFinder;
  std::unique_ptr<IMUGravityRemover<Quaternion>> gravityRemover;
  std::unique_ptr<IMUVelocityEstimator> velocityEstimator;
  uint32_t initializationSamplesCount;
  uint32_t sampleCount;

  struct InstanceState {
    std::unique_ptr<IMUSignalProcessor> instance;
    SemaphoreHandle_t semaphoreHandle;
    StaticSemaphore_t semaphoreControlBlock;
    portMUX_TYPE mux;
  };
  static InstanceState instanceState;

  std::atomic<State> state{State::IDLE};
  TaskHandle_t coordinator;

  static std::unique_ptr<IMUSignalProcessor> create(
      std::shared_ptr<Pipe<IMUSample, SAMPLING_PIPE_SIZE>> inputPipe,
      std::shared_ptr<Pipe<ProcessedSample, TRANSMISSION_PIPE_SIZE>> outputPipe,
      std::unique_ptr<LoopRunner> runner, LoggerPort *logger,
      std::unique_ptr<IMUCalibrator> calibrator,
      std::unique_ptr<IMUNoiseReducer> noiseReducer,
      std::unique_ptr<IMUOrientationFinder<Quaternion>> orientationFinder,
      std::unique_ptr<IMUGravityRemover<Quaternion>> gravityRemover,
      std::unique_ptr<IMUVelocityEstimator> velocityEstimator,
      uint32_t initializationSamplesCount, TaskHandle_t coordinator);

  IMUSignalProcessor(
      std::shared_ptr<Pipe<IMUSample, SAMPLING_PIPE_SIZE>> inputPipe,
      std::shared_ptr<Pipe<ProcessedSample, TRANSMISSION_PIPE_SIZE>> outputPipe,
      std::unique_ptr<LoopRunner> runner, LoggerPort *logger,
      std::unique_ptr<IMUCalibrator> calibrator,
      std::unique_ptr<IMUNoiseReducer> noiseReducer,
      std::unique_ptr<IMUOrientationFinder<Quaternion>> orientationFinder,
      std::unique_ptr<IMUGravityRemover<Quaternion>> gravityRemover,
      std::unique_ptr<IMUVelocityEstimator> velocityEstimator,
      uint32_t initializationSamplesCount, TaskHandle_t coordinator);

  void setState(State state);
  static void processingLoopFunction(void *arg);
};
