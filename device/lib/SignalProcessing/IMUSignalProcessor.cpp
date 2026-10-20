#include "include/IMUSignalProcessor.hpp"
#include <IMUSignalProcessor.hpp>

IMUSignalProcessor::InstanceState IMUSignalProcessor::instanceState{};

IMUSignalProcessor *IMUSignalProcessor::getInstance(
    std::shared_ptr<Pipe<IMUSample, SAMPLING_PIPE_SIZE>> inputPipe,
    std::shared_ptr<Pipe<ProcessedSample, TRANSMISSION_PIPE_SIZE>> outputPipe,
    std::unique_ptr<LoopRunner> runner, LoggerPort *logger,
    std::unique_ptr<IMUCalibrator> calibrator,
    std::unique_ptr<IMUNoiseReducer> noiseReducer,
    std::unique_ptr<IMUOrientationFinder<Quaternion>> orientationFinder,
    std::unique_ptr<IMUGravityRemover<Quaternion>> gravityRemover,
    std::unique_ptr<IMUVelocityEstimator> velocityEstimator,
    uint32_t initializationSamplesCount) {
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
          std::move(velocityEstimator), initializationSamplesCount);
    rtosSemaphoreGive(IMUSignalProcessor::instanceState.semaphoreHandle);
  }
  return IMUSignalProcessor::instanceState.instance.get();
}

#if defined(UNIT_TEST) && !defined(ESP_PLATFORM)
void IMUSignalProcessor::resetInstanceForTests() {
  IMUSignalProcessor::instanceState.instance.reset();
  IMUSignalProcessor::instanceState.semaphoreHandle = nullptr;
  IMUSignalProcessor::instanceState.semaphoreControlBlock = {};
  IMUSignalProcessor::instanceState.mux = {};
}
#endif

IMUSignalProcessor::~IMUSignalProcessor() {
  this->logger->info("Shutting down Signal processor");

  this->setDoProcess(false);
  this->setState(IMUSignalProcessor::State::IDLE);

  if (this->runner) {
    this->runner->stop();
  }

  this->logger->info("Signal processor shut down successfully");
}

void IMUSignalProcessor::beginProcessing() {
  this->sampleCount = 0;
  this->setDoProcess(true);
  this->setState(IMUSignalProcessor::State::INITIALIZING);
}

void IMUSignalProcessor::stopProcessing() {
  this->setDoProcess(false);
  this->setState(IMUSignalProcessor::State::IDLE);
}

std::unique_ptr<IMUSignalProcessor> IMUSignalProcessor::create(
    std::shared_ptr<Pipe<IMUSample, SAMPLING_PIPE_SIZE>> inputPipe,
    std::shared_ptr<Pipe<ProcessedSample, TRANSMISSION_PIPE_SIZE>> outputPipe,
    std::unique_ptr<LoopRunner> runner, LoggerPort *logger,
    std::unique_ptr<IMUCalibrator> calibrator,
    std::unique_ptr<IMUNoiseReducer> noiseReducer,
    std::unique_ptr<IMUOrientationFinder<Quaternion>> orientationFinder,
    std::unique_ptr<IMUGravityRemover<Quaternion>> gravityRemover,
    std::unique_ptr<IMUVelocityEstimator> velocityEstimator,
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
  if (!velocityEstimator) {
    logger->error("Velocity estimator is null");
    return nullptr;
  }

  std::unique_ptr<IMUSignalProcessor> processor(
      new (std::nothrow) IMUSignalProcessor(
          inputPipe, outputPipe, std::move(runner), logger,
          std::move(calibrator), std::move(noiseReducer),
          std::move(orientationFinder), std::move(gravityRemover),
          std::move(velocityEstimator), initializationSamplesCount));

  if (!processor->runner->start(IMUSignalProcessor::processingLoopFunction,
                                static_cast<void *>(processor.get()))) {
    logger->error("Failed to start IMU signal processing task");
    return nullptr;
  }

  return processor;
}

IMUSignalProcessor::IMUSignalProcessor(
    std::shared_ptr<Pipe<IMUSample, SAMPLING_PIPE_SIZE>> inputPipe,
    std::shared_ptr<Pipe<ProcessedSample, TRANSMISSION_PIPE_SIZE>> outputPipe,
    std::unique_ptr<LoopRunner> runner, LoggerPort *logger,
    std::unique_ptr<IMUCalibrator> calibrator,
    std::unique_ptr<IMUNoiseReducer> noiseReducer,
    std::unique_ptr<IMUOrientationFinder<Quaternion>> orientationFinder,
    std::unique_ptr<IMUGravityRemover<Quaternion>> gravityRemover,
    std::unique_ptr<IMUVelocityEstimator> velocityEstimator,
    uint32_t initializationSamplesCount)
    : inputPipe(inputPipe), outputPipe(outputPipe), runner(std::move(runner)),
      logger(logger), calibrator(std::move(calibrator)),
      noiseReducer(std::move(noiseReducer)),
      orientationFinder(std::move(orientationFinder)),
      gravityRemover(std::move(gravityRemover)),
      velocityEstimator(std::move(velocityEstimator)),
      initializationSamplesCount(initializationSamplesCount), sampleCount(0) {}

void IMUSignalProcessor::setDoProcess(bool value) {
  this->doProcess.store(value, std::memory_order_relaxed);
}
bool IMUSignalProcessor::getDoProcess() {
  return this->doProcess.load(std::memory_order_relaxed);
}
void IMUSignalProcessor::setState(State state) {
  this->state.store(state, std::memory_order_relaxed);
}
IMUSignalProcessor::State IMUSignalProcessor::getState() {
  return this->state.load(std::memory_order_relaxed);
}

void IMUSignalProcessor::processingLoopFunction(void *arg) {
  IMUSignalProcessor *self = static_cast<IMUSignalProcessor *>(arg);

  if (!self)
    return;

  if (!self->getDoProcess() && self->inputPipe->itemsLeft() == 0) {
    self->logger->debug(
        "Processing loop: not processing and input pipe has no items left");
    self->calibrator->reset();
    self->orientationFinder->reset();
    self->velocityEstimator->reset();
    self->setState(IMUSignalProcessor::State::IDLE);
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

  if (self->getState() == IMUSignalProcessor::State::INITIALIZING) {
    self->setState(IMUSignalProcessor::State::PROCESSING);
  }

  self->calibrator->calibrate(sample);
  self->noiseReducer->filter(sample);
  auto orientation = self->orientationFinder->find(sample);
  self->gravityRemover->remove(sample, orientation);
  auto velocitySample = self->velocityEstimator->estimate(sample, orientation);

  self->logger->debug("Processing loop: velocity = <%.3f, %.3f, %.3f>",
                      velocitySample.x, velocitySample.y, velocitySample.z);

  ProcessedSample processedSample = {
      .v = velocitySample,
      .w = sample.w,
      .seq = self->sampleCount,
  };
  if (!self->outputPipe->push(processedSample))
    self->logger->warn("Processing loop: failed to push sample to output pipe");
}
