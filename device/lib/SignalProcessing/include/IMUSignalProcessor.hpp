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
  static IMUSignalProcessor *getInstance(
      std::shared_ptr<Pipe<IMUSample, SAMPLING_PIPE_SIZE>> inputPipe,
      std::shared_ptr<Pipe<ProcessedSample, TRANSMISSION_PIPE_SIZE>> outputPipe,
      std::unique_ptr<LoopRunner> runner, LoggerPort *logger,
      std::unique_ptr<IMUCalibrator> calibrator,
      std::unique_ptr<IMUNoiseReducer> noiseReducer,
      std::unique_ptr<IMUOrientationFinder<Quaternion>> orientationFinder,
      std::unique_ptr<IMUGravityRemover<Quaternion>> gravityRemover,
      std::unique_ptr<IMUVelocityEstimator> velocityEstimator,
      uint32_t initializationSamplesCount);

#if defined(UNIT_TEST) && !defined(ESP_PLATFORM)
  static void resetInstanceForTests();
#endif

  ~IMUSignalProcessor();

  void beginProcessing();
  void stopProcessing();

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

  std::atomic<bool> doProcess{false};

  static std::unique_ptr<IMUSignalProcessor> create(
      std::shared_ptr<Pipe<IMUSample, SAMPLING_PIPE_SIZE>> inputPipe,
      std::shared_ptr<Pipe<ProcessedSample, TRANSMISSION_PIPE_SIZE>> outputPipe,
      std::unique_ptr<LoopRunner> runner, LoggerPort *logger,
      std::unique_ptr<IMUCalibrator> calibrator,
      std::unique_ptr<IMUNoiseReducer> noiseReducer,
      std::unique_ptr<IMUOrientationFinder<Quaternion>> orientationFinder,
      std::unique_ptr<IMUGravityRemover<Quaternion>> gravityRemover,
      std::unique_ptr<IMUVelocityEstimator> velocityEstimator,
      uint32_t initializationSamplesCount);

  IMUSignalProcessor(
      std::shared_ptr<Pipe<IMUSample, SAMPLING_PIPE_SIZE>> inputPipe,
      std::shared_ptr<Pipe<ProcessedSample, TRANSMISSION_PIPE_SIZE>> outputPipe,
      std::unique_ptr<LoopRunner> runner, LoggerPort *logger,
      std::unique_ptr<IMUCalibrator> calibrator,
      std::unique_ptr<IMUNoiseReducer> noiseReducer,
      std::unique_ptr<IMUOrientationFinder<Quaternion>> orientationFinder,
      std::unique_ptr<IMUGravityRemover<Quaternion>> gravityRemover,
      std::unique_ptr<IMUVelocityEstimator> velocityEstimator,
      uint32_t initializationSamplesCount);

  void setDoProcess(bool value);
  bool getDoProcess();
  static void processingLoopFunction(void *arg);
};
