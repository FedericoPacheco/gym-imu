#include <VBTSystem.hpp>

VBTSystem::VBTSystem(const Dependencies &dependencies)
    : imu(dependencies.imu), ble(dependencies.ble),
      processor(dependencies.processor), button(dependencies.button),
      led(dependencies.led), samplingPipe(dependencies.samplingPipe),
      transmissionPipe(dependencies.transmissionPipe),
      logger(dependencies.logger),
      samplingPipeLogger(dependencies.samplingPipeLogger),
      transmissionPipeLogger(dependencies.transmissionPipeLogger),
      signalProcessingEnabled(dependencies.signalProcessingEnabled) {}

VBTSystem *VBTSystem::getInstance(const Dependencies &dependencies) {
  static std::unique_ptr<VBTSystem> instance;
  if (instance)
    return instance.get();

  if (!dependencies.imu || !dependencies.ble || !dependencies.button ||
      !dependencies.led || !dependencies.samplingPipe || !dependencies.logger ||
      !dependencies.samplingPipeLogger ||
      (dependencies.signalProcessingEnabled &&
       (!dependencies.processor || !dependencies.transmissionPipe ||
        !dependencies.transmissionPipeLogger))) {
    if (dependencies.logger)
      dependencies.logger->error("Invalid VBTSystem dependencies");
    return nullptr;
  }

  instance.reset(new (std::nothrow) VBTSystem(dependencies));
  if (!instance) {
    dependencies.logger->error("Failed to allocate VBTSystem");
    return nullptr;
  }
  return instance.get();
}

bool VBTSystem::run() {
  if (!this->button->enableAsync()) {
    this->logger->error("Failed to enable asynchronous button input");
    return false;
  }

  bool isRecording = false;
  while (true) {
    rtosTaskDelay(rtosMillisecondsToTicks(250));

    if (this->button->wasPressedAsync() && this->ble->isConnected()) {
      isRecording = !isRecording;
      if (isRecording) {
        this->startPipeline();
      } else {
        if (!this->stopPipeline()) {
          this->button->disableAsync();
          return false;
        }
        this->logPipeMetrics();
      }
    }
    if (isRecording && !this->ble->isConnected()) {
      isRecording = false;
      if (!this->stopPipeline()) {
        this->button->disableAsync();
        return false;
      }
    }

    this->updateLed(isRecording);
  }

  this->button->disableAsync();
  return true;
}

void VBTSystem::startPipeline() {
  this->ble->beginTransmission();
  if (this->signalProcessingEnabled)
    this->processor->beginProcessing();
  this->imu->beginAsync();
}

bool VBTSystem::stopPipeline() {
  this->imu->requestStopAsync();
  if (!this->waitForStop(IMUSensorPort::STOPPED_MASK, "IMU"))
    return false;

  if (this->signalProcessingEnabled) {
    this->processor->requestStopProcessing();
    if (!this->waitForStop(IMUSignalProcessor::STOPPED_MASK,
                           "signal processor"))
      return false;
  }

  this->ble->requestStopTransmission();
  return this->waitForStop(BLETransmitterPort::STOPPED_MASK, "BLE");
}

bool VBTSystem::waitForStop(uint32_t notificationMask, const char *stage) {
  uint32_t notificationValue = 0;
  const BaseType_t notified =
      rtosTaskNotifyWait(0, notificationMask, &notificationValue,
                         MAX_BLOCKING_TIME_AFTER_STOP_TICKS);
  if (notified == pdTRUE && (notificationValue & notificationMask) != 0)
    return true;

  this->logger->error("Failed to stop %s properly; rebooting", stage);
  rtosTaskDelay(rtosMillisecondsToTicks(100));
  restartDevice(); // Simplest recovery strategy: restart the device
  return false;
}

void VBTSystem::updateLed(bool isRecording) {
  if (!this->signalProcessingEnabled) {
    if (isRecording)
      this->led->turnOn();
    else
      this->led->turnOff();
    return;
  }

  switch (this->processor->getState()) {
  case IMUSignalProcessor::State::IDLE:
    this->led->turnOff();
    break;
  case IMUSignalProcessor::State::INITIALIZING:
    this->led->blink();
    break;
  case IMUSignalProcessor::State::PROCESSING:
  case IMUSignalProcessor::State::DRAINING:
    this->led->turnOn();
    break;
  }
}

void VBTSystem::logPipeMetrics() {
  const PipeMetrics samplingMetrics = this->samplingPipe->getMetrics();
  this->samplingPipeLogger->info(
      "Sampling pipe metrics: drops=%u, maxDepth=%u, currentDepth=%u",
      samplingMetrics.drops, samplingMetrics.maxDepth,
      samplingMetrics.currentDepth);

  if (this->signalProcessingEnabled) {
    const PipeMetrics transmissionMetrics =
        this->transmissionPipe->getMetrics();
    this->transmissionPipeLogger->info(
        "Transmission pipe metrics: drops=%u, maxDepth=%u, currentDepth=%u",
        transmissionMetrics.drops, transmissionMetrics.maxDepth,
        transmissionMetrics.currentDepth);
  }
}
