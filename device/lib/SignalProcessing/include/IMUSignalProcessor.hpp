#pragma once
#include <Constants.hpp>
#include <IMUCalibrator.hpp>
#include <IMUGravityRemover.hpp>
#include <IMUNoiseReducer.hpp>
#include <IMUOrientationFinder.hpp>
#include <IMUSensorPort.hpp>
#include <LoggerPort.hpp>
#include <LoopRunner.hpp>
#include <Pipe.hpp>
#include <Quaternion.hpp>
#include <atomic>
#include <memory>
#include <ports/FreeRTOS/FreeRTOSPort.hpp>

// TODO: move to velocity computation strategy interface
typedef LinearAxes VelocitySample; // m/s

class IMUSignalProcessor {
public:
  static IMUSignalProcessor *getInstance(
      std::shared_ptr<Pipe<IMUSample, SAMPLING_PIPE_SIZE>> inputPipe,
      std::shared_ptr<Pipe<IMUSample, TRANSMISSION_PIPE_SIZE>> outputPipe,
      std::unique_ptr<LoopRunner> runner, LoggerPort *logger,
      std::unique_ptr<IMUCalibrator> calibrator,
      std::unique_ptr<IMUNoiseReducer> noiseReducer,
      std::unique_ptr<IMUOrientationFinder<Quaternion>> orientationFinder,
      std::unique_ptr<IMUGravityRemover<Quaternion>> gravityRemover,
      uint32_t initializationSamplesCount) {

    // Protect instance creation with a mutex to ensure thread safety
    if (IMUSignalProcessor::instanceState.semaphoreHandle == nullptr) {
      rtosTaskEnterCritical(&IMUSignalProcessor::instanceState.mux);
      IMUSignalProcessor::instanceState.semaphoreHandle = rtosCreateMutexStatic(
          &IMUSignalProcessor::instanceState.semaphoreControlBlock);
      if (IMUSignalProcessor::instanceState.semaphoreHandle == nullptr) {
        logger->error("Failed to create mutex for IMUSignalProcessor instance");
        rtosTaskExitCritical(&IMUSignalProcessor::instanceState.mux);
        return nullptr;
      }
      rtosTaskExitCritical(&IMUSignalProcessor::instanceState.mux);
    }

    if (rtosSemaphoreTake(IMUSignalProcessor::instanceState.semaphoreHandle,
                          portMAX_DELAY) == pdTRUE) {
      if (!IMUSignalProcessor::instanceState.instance)
        IMUSignalProcessor::instanceState.instance = IMUSignalProcessor::create(
            inputPipe, outputPipe, std::move(runner), logger,
            std::move(calibrator), std::move(noiseReducer),
            std::move(orientationFinder), std::move(gravityRemover),
            initializationSamplesCount);
      rtosSemaphoreGive(IMUSignalProcessor::instanceState.semaphoreHandle);
    }
    return IMUSignalProcessor::instanceState.instance.get();
  }

#if defined(UNIT_TEST) && !defined(ESP_PLATFORM)
  static void resetInstanceForTests() {
    IMUSignalProcessor::instanceState.instance.reset();
    IMUSignalProcessor::instanceState.semaphoreHandle = nullptr;
    IMUSignalProcessor::instanceState.semaphoreControlBlock = {};
    IMUSignalProcessor::instanceState.mux = {};
  }
#endif

  ~IMUSignalProcessor() {
    this->logger->info("Shutting down Signal processor");

    this->setDoProcess(false);

    if (this->runner) {
      this->runner->stop();
    }

    this->logger->info("Signal processor shut down successfully");
  }

  void beginProcessing() {
    this->sampleCount = 0;
    this->setDoProcess(true);
  }
  void stopProcessing() { this->setDoProcess(false); }

private:
  std::shared_ptr<Pipe<IMUSample, SAMPLING_PIPE_SIZE>> inputPipe;
  std::shared_ptr<Pipe<IMUSample, TRANSMISSION_PIPE_SIZE>> outputPipe;
  std::unique_ptr<LoopRunner> runner;
  LoggerPort *logger;
  std::unique_ptr<IMUCalibrator> calibrator;
  std::unique_ptr<IMUNoiseReducer> noiseReducer;
  std::unique_ptr<IMUOrientationFinder<Quaternion>> orientationFinder;
  std::unique_ptr<IMUGravityRemover<Quaternion>> gravityRemover;
  uint32_t initializationSamplesCount;
  uint32_t sampleCount;

  struct InstanceState {
    std::unique_ptr<IMUSignalProcessor> instance;
    SemaphoreHandle_t semaphoreHandle;
    StaticSemaphore_t semaphoreControlBlock;
    portMUX_TYPE mux;
  };
  inline static InstanceState instanceState;

  std::atomic<bool> doProcess{false};

  static std::unique_ptr<IMUSignalProcessor>
  create(std::shared_ptr<Pipe<IMUSample, SAMPLING_PIPE_SIZE>> inputPipe,
         std::shared_ptr<Pipe<IMUSample, TRANSMISSION_PIPE_SIZE>> outputPipe,
         std::unique_ptr<LoopRunner> runner, LoggerPort *logger,
         std::unique_ptr<IMUCalibrator> calibrator,
         std::unique_ptr<IMUNoiseReducer> noiseReducer,
         std::unique_ptr<IMUOrientationFinder<Quaternion>> orientationFinder,
         std::unique_ptr<IMUGravityRemover<Quaternion>> gravityRemover,
         uint32_t initializationSamplesCount) {

    if (!logger)
      return nullptr;
    if (!inputPipe) {
      logger->error("Input pipe is null");
      return nullptr;
    }
    if (!outputPipe) {
      logger->error("Output pipe is null");
      return nullptr;
    }
    if (!runner) {
      logger->error("Runner is null");
      return nullptr;
    }
    if (!calibrator) {
      logger->error("Calibrator is null");
      return nullptr;
    }
    if (!noiseReducer) {
      logger->error("Noise reducer is null");
      return nullptr;
    }
    if (!orientationFinder) {
      logger->error("Orientation finder is null");
      return nullptr;
    }
    if (!gravityRemover) {
      logger->error("Gravity remover is null");
      return nullptr;
    }

    std::unique_ptr<IMUSignalProcessor> processor(
        new (std::nothrow) IMUSignalProcessor(
            inputPipe, outputPipe, std::move(runner), logger,
            std::move(calibrator), std::move(noiseReducer),
            std::move(orientationFinder), std::move(gravityRemover),
            initializationSamplesCount));

    if (!processor->runner->start(IMUSignalProcessor::processingLoopFunction,
                                  static_cast<void *>(processor.get()))) {
      logger->error("Failed to start IMU signal processing task");
      return nullptr;
    }

    return processor;
  }

  IMUSignalProcessor(
      std::shared_ptr<Pipe<IMUSample, SAMPLING_PIPE_SIZE>> inputPipe,
      std::shared_ptr<Pipe<IMUSample, TRANSMISSION_PIPE_SIZE>> outputPipe,
      std::unique_ptr<LoopRunner> runner, LoggerPort *logger,
      std::unique_ptr<IMUCalibrator> calibrator,
      std::unique_ptr<IMUNoiseReducer> noiseReducer,
      std::unique_ptr<IMUOrientationFinder<Quaternion>> orientationFinder,
      std::unique_ptr<IMUGravityRemover<Quaternion>> gravityRemover,
      uint32_t initializationSamplesCount)
      : inputPipe(inputPipe), outputPipe(outputPipe), runner(std::move(runner)),
        logger(logger), calibrator(std::move(calibrator)),
        noiseReducer(std::move(noiseReducer)),
        orientationFinder(std::move(orientationFinder)),
        gravityRemover(std::move(gravityRemover)),
        initializationSamplesCount(initializationSamplesCount), sampleCount(0) {
  }

  inline void setDoProcess(bool value) {
    this->doProcess.store(value, std::memory_order_relaxed);
  }
  inline bool getDoProcess() {
    return this->doProcess.load(std::memory_order_relaxed);
  }

  static void processingLoopFunction(void *arg) {
    IMUSignalProcessor *self = static_cast<IMUSignalProcessor *>(arg);

    if (!self)
      return;

    if (!self->getDoProcess() && self->inputPipe->itemsLeft() == 0) {
      self->logger->debug(
          "Processing loop: not processing and input pipe has no items left");
      self->calibrator->reset();
      self->orientationFinder->reset();
      return;
    }

    std::optional<IMUSample> optSample = self->inputPipe->pop();
    if (!optSample.has_value()) {
      self->logger->warn(
          "Processing loop: failed to read sample from input pipe");
      return;
    }

    IMUSample sample = optSample.value();
    self->sampleCount++;

    if (self->sampleCount <= self->initializationSamplesCount) {
      self->logger->debug("Processing loop: still initializing, "
                          "sampleCount = %u",
                          self->sampleCount);

      self->calibrator->initialize(sample);
      self->orientationFinder->initialize(sample);
      return;
    }

    self->calibrator->calibrate(sample);
    self->noiseReducer->filter(sample);
    auto orientation = self->orientationFinder->find(sample);
    self->gravityRemover->remove(sample, orientation);

    self->logger->debug(
        "Processing loop: gravity-free accel = <%.3f, %.3f, %.3f>", sample.a.x,
        sample.a.y, sample.a.z);

    // TODO: temporary until all the components are implemented: must
    // push linear/angular velocities + seqs
    if (!self->outputPipe->push(sample))
      self->logger->warn(
          "Processing loop: failed to push sample to output pipe");
  }
};